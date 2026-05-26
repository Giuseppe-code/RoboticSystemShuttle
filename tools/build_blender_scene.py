#!/usr/bin/env python3
"""Build a Blender terrain/road scene from prepared OSM data and export glTF."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import Any

import bpy
from mathutils import Vector


def arguments() -> argparse.Namespace:
    raw_args = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--blend", type=Path, required=True)
    parser.add_argument("--glb", type=Path, required=True)
    return parser.parse_args(raw_args)


def safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value)[:60]


def clean_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for collection in list(bpy.data.collections):
        if collection.name != "Collection":
            bpy.data.collections.remove(collection)
    base = bpy.data.collections.get("Collection")
    if base:
        base.name = "Map"


def collection(name: str) -> bpy.types.Collection:
    result = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(result)
    return result


def material(name: str, rgba: tuple[float, float, float, float], roughness: float = 0.8) -> bpy.types.Material:
    result = bpy.data.materials.new(name=name)
    result.diffuse_color = rgba
    result.roughness = roughness
    return result


def mesh_object(
    name: str,
    vertices: list[tuple[float, float, float]],
    faces: list[tuple[int, ...]],
    parent: bpy.types.Collection,
    surface_material: bpy.types.Material,
) -> bpy.types.Object:
    mesh = bpy.data.meshes.new(f"{name}_Mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.materials.append(surface_material)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    parent.objects.link(obj)
    return obj


class Terrain:
    def __init__(self, value: dict[str, Any]) -> None:
        self.size = int(value["grid_size"])
        self.extent = float(value["extent_m"])
        self.spacing = float(value["spacing_m"])
        self.samples = value["samples_blender_xyz_m"]

    def z(self, east: float, north: float) -> float:
        col = max(0.0, min(self.size - 1.0, (east + self.extent) / self.spacing))
        row = max(0.0, min(self.size - 1.0, (north + self.extent) / self.spacing))
        c0, r0 = int(math.floor(col)), int(math.floor(row))
        c1, r1 = min(c0 + 1, self.size - 1), min(r0 + 1, self.size - 1)
        tx, ty = col - c0, row - r0
        values = [
            self.samples[r0 * self.size + c0][2],
            self.samples[r0 * self.size + c1][2],
            self.samples[r1 * self.size + c0][2],
            self.samples[r1 * self.size + c1][2],
        ]
        return (
            values[0] * (1 - tx) * (1 - ty)
            + values[1] * tx * (1 - ty)
            + values[2] * (1 - tx) * ty
            + values[3] * tx * ty
        )


def terrain_mesh(value: dict[str, Any], parent: bpy.types.Collection, mat: bpy.types.Material) -> bpy.types.Object:
    size = int(value["grid_size"])
    vertices = [tuple(point) for point in value["samples_blender_xyz_m"]]
    faces: list[tuple[int, ...]] = []
    for row in range(size - 1):
        for col in range(size - 1):
            lower = row * size + col
            faces.append((lower, lower + 1, lower + size + 1, lower + size))
    obj = mesh_object("Terrain-col", vertices, faces, parent, mat)
    obj["description"] = "DEM terrain; -col suffix is intended for Godot collision import"
    return obj


def strip_geometry(points: list[list[float]], width: float) -> tuple[list[tuple[float, float, float]], list[tuple[int, ...]]]:
    vertices: list[tuple[float, float, float]] = []
    for index, point in enumerate(points):
        if index == 0:
            tangent = Vector((points[1][0] - point[0], points[1][1] - point[1]))
        elif index == len(points) - 1:
            tangent = Vector((point[0] - points[index - 1][0], point[1] - points[index - 1][1]))
        else:
            tangent = Vector((points[index + 1][0] - points[index - 1][0], points[index + 1][1] - points[index - 1][1]))
        tangent.normalize()
        normal = Vector((-tangent.y, tangent.x)) * width * 0.5
        vertices.extend(
            [
                (point[0] + normal.x, point[1] + normal.y, point[2]),
                (point[0] - normal.x, point[1] - normal.y, point[2]),
            ]
        )
    faces = [(2 * i, 2 * i + 1, 2 * i + 3, 2 * i + 2) for i in range(len(points) - 1)]
    return vertices, faces


def polygon_points(points: list[list[float]]) -> list[tuple[float, float, float]]:
    result = [tuple(point) for point in points]
    if len(result) > 2 and result[0][:2] == result[-1][:2]:
        return result[:-1]
    return result


def building_height(tags: dict[str, str]) -> float:
    raw_height = tags.get("height", "")
    match = re.search(r"(\d+(?:[.,]\d+)?)", raw_height)
    if match:
        return float(match.group(1).replace(",", "."))
    raw_levels = tags.get("building:levels", "")
    if raw_levels.isdigit():
        return max(3.0, float(raw_levels) * 3.0)
    return 7.5


def extruded_polygon(
    name: str,
    points: list[list[float]],
    height: float,
    parent: bpy.types.Collection,
    mat: bpy.types.Material,
) -> bpy.types.Object | None:
    base = polygon_points(points)
    if len(base) < 3:
        return None
    vertices = base + [(x, y, z + height) for x, y, z in base]
    count = len(base)
    faces: list[tuple[int, ...]] = [tuple(reversed(range(count))), tuple(range(count, 2 * count))]
    faces.extend((i, (i + 1) % count, count + (i + 1) % count, count + i) for i in range(count))
    return mesh_object(name, vertices, faces, parent, mat)


def flat_polygon(
    name: str,
    points: list[list[float]],
    parent: bpy.types.Collection,
    mat: bpy.types.Material,
) -> bpy.types.Object | None:
    vertices = polygon_points(points)
    if len(vertices) < 3:
        return None
    return mesh_object(name, vertices, [tuple(range(len(vertices)))], parent, mat)


def attach_osm_metadata(obj: bpy.types.Object | None, feature: dict[str, Any]) -> None:
    if not obj:
        return
    obj["osm_way_id"] = int(feature["osm_way_id"])
    obj["osm_tags"] = json.dumps(feature.get("tags", {}), ensure_ascii=True)


def add_sun_and_camera(extent: float, terrain: Terrain) -> None:
    sun_data = bpy.data.lights.new("Sun", type="SUN")
    sun_data.energy = 2.0
    sun = bpy.data.objects.new("Sun", sun_data)
    bpy.context.scene.collection.objects.link(sun)
    sun.rotation_euler = (math.radians(25), math.radians(-20), math.radians(25))
    camera_data = bpy.data.cameras.new("OverviewCamera")
    camera = bpy.data.objects.new("OverviewCamera", camera_data)
    bpy.context.scene.collection.objects.link(camera)
    camera.location = (extent * 0.15, -extent * 1.8, extent * 2.0 + terrain.z(0, 0))
    direction = Vector((0, 0, terrain.z(0, 0))) - camera.location
    camera.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    camera_data.lens = 30
    camera_data.clip_end = 5000.0
    bpy.context.scene.camera = camera


def build_scene(scene_data: dict[str, Any]) -> None:
    clean_scene()
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.world.color = (0.04, 0.06, 0.08)
    scene["map_provenance"] = json.dumps(scene_data["provenance"], ensure_ascii=True)
    scene["map_summary"] = json.dumps(scene_data["summary"], ensure_ascii=True)

    terrain_collection = collection("Terrain")
    roads_collection = collection("Roads_Drivable")
    paths_collection = collection("Paths_NonDrivable")
    parking_collection = collection("Parking")
    parks_collection = collection("Parks")
    buildings_collection = collection("Buildings")

    mats = {
        "terrain": material("Terrain", (0.27, 0.36, 0.22, 1.0)),
        "road": material("Road", (0.08, 0.085, 0.095, 1.0), 0.92),
        "path": material("Path", (0.31, 0.27, 0.23, 1.0)),
        "parking": material("Parking", (0.18, 0.2, 0.22, 1.0)),
        "park": material("Park", (0.12, 0.39, 0.16, 1.0)),
        "building": material("Building", (0.52, 0.48, 0.42, 1.0)),
    }
    terrain = Terrain(scene_data["terrain"])
    terrain_mesh(scene_data["terrain"], terrain_collection, mats["terrain"])
    for feature in scene_data["features"]:
        kind = feature["kind"]
        points = feature["points_blender_xyz_m"]
        osm_id = feature["osm_way_id"]
        if kind == "road" and len(points) >= 2:
            vertices, faces = strip_geometry(points, float(feature["width_m"]))
            target = roads_collection if feature["drivable"] else paths_collection
            prefix = "NAV_Road" if feature["drivable"] else "Path"
            obj = mesh_object(f"{prefix}_{osm_id}", vertices, faces, target, mats["road" if feature["drivable"] else "path"])
            obj["width_m"] = float(feature["width_m"])
            obj["width_source"] = feature["width_source"]
            obj["drivable"] = bool(feature["drivable"])
            attach_osm_metadata(obj, feature)
        elif kind == "parking":
            attach_osm_metadata(flat_polygon(f"Parking_{osm_id}", points, parking_collection, mats["parking"]), feature)
        elif kind == "park":
            attach_osm_metadata(flat_polygon(f"Park_{osm_id}", points, parks_collection, mats["park"]), feature)
        elif kind == "building":
            obj = extruded_polygon(
                f"Building_{osm_id}",
                points,
                building_height(feature.get("tags", {})),
                buildings_collection,
                mats["building"],
            )
            attach_osm_metadata(obj, feature)
    add_sun_and_camera(terrain.extent, terrain)


def main() -> None:
    args = arguments()
    with args.input.open("r", encoding="utf-8") as handle:
        scene_data = json.load(handle)
    args.blend.parent.mkdir(parents=True, exist_ok=True)
    args.glb.parent.mkdir(parents=True, exist_ok=True)
    build_scene(scene_data)
    bpy.ops.wm.save_as_mainfile(filepath=str(args.blend.resolve()))
    bpy.ops.export_scene.gltf(
        filepath=str(args.glb.resolve()),
        export_format="GLB",
        export_yup=True,
        export_apply=True,
    )
    print(f"Saved Blender scene: {args.blend}")
    print(f"Exported glTF: {args.glb}")


if __name__ == "__main__":
    main()

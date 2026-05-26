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
from mathutils.geometry import tessellate_polygon


ROAD_SURFACE_OFFSET_M = 0.12
PATH_SURFACE_OFFSET_M = 0.10
AREA_SURFACE_OFFSET_M = 0.08
ROAD_SAMPLE_STEP_M = 5.0
AREA_SAMPLE_STEP_M = 10.0
TERRAIN_SUBDIVISIONS = 4


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


def terrain_mesh(terrain: Terrain, parent: bpy.types.Collection, mat: bpy.types.Material) -> bpy.types.Object:
    size = (terrain.size - 1) * TERRAIN_SUBDIVISIONS + 1
    spacing = 2.0 * terrain.extent / (size - 1)
    vertices: list[tuple[float, float, float]] = []
    for row in range(size):
        north = -terrain.extent + row * spacing
        for col in range(size):
            east = -terrain.extent + col * spacing
            vertices.append((east, north, terrain.z(east, north)))
    faces: list[tuple[int, ...]] = []
    for row in range(size - 1):
        for col in range(size - 1):
            lower = row * size + col
            faces.extend(
                [
                    (lower, lower + 1, lower + size + 1),
                    (lower, lower + size + 1, lower + size),
                ]
            )
    obj = mesh_object("Terrain-col", vertices, faces, parent, mat)
    obj["description"] = "Subdivided DEM terrain; -col suffix is intended for Godot collision import"
    return obj


def horizontal_distance(left: tuple[float, float], right: tuple[float, float]) -> float:
    return math.hypot(right[0] - left[0], right[1] - left[1])


def draped_polyline(points: list[list[float]], terrain: Terrain, offset: float) -> list[tuple[float, float, float]]:
    result: list[tuple[float, float, float]] = []
    for left, right in zip(points, points[1:]):
        length = horizontal_distance((left[0], left[1]), (right[0], right[1]))
        pieces = max(1, int(math.ceil(length / ROAD_SAMPLE_STEP_M)))
        for index in range(pieces):
            ratio = index / pieces
            east = left[0] + (right[0] - left[0]) * ratio
            north = left[1] + (right[1] - left[1]) * ratio
            if not result or horizontal_distance((east, north), result[-1][:2]) > 1e-5:
                result.append((east, north, terrain.z(east, north) + offset))
    last = points[-1]
    result.append((last[0], last[1], terrain.z(last[0], last[1]) + offset))
    return result


def strip_geometry(
    points: list[list[float]], width: float, terrain: Terrain, offset: float
) -> tuple[list[tuple[float, float, float]], list[tuple[int, ...]]]:
    centerline = draped_polyline(points, terrain, offset)
    vertices: list[tuple[float, float, float]] = []
    for index, point in enumerate(centerline):
        if index == 0:
            tangent = Vector((centerline[1][0] - point[0], centerline[1][1] - point[1])).normalized()
            normal = Vector((-tangent.y, tangent.x))
        elif index == len(centerline) - 1:
            tangent = Vector((point[0] - centerline[index - 1][0], point[1] - centerline[index - 1][1])).normalized()
            normal = Vector((-tangent.y, tangent.x))
        else:
            previous = Vector((point[0] - centerline[index - 1][0], point[1] - centerline[index - 1][1])).normalized()
            following = Vector((centerline[index + 1][0] - point[0], centerline[index + 1][1] - point[1])).normalized()
            previous_normal = Vector((-previous.y, previous.x))
            following_normal = Vector((-following.y, following.x))
            normal = previous_normal + following_normal
            if normal.length < 1e-6:
                normal = following_normal
            else:
                normal.normalize()
            correction = abs(normal.dot(following_normal))
            normal *= min(2.25, 1.0 / max(correction, 0.45))
        normal *= width * 0.5
        left = (point[0] + normal.x, point[1] + normal.y)
        right = (point[0] - normal.x, point[1] - normal.y)
        vertices.extend(
            [
                (left[0], left[1], terrain.z(*left) + offset),
                (right[0], right[1], terrain.z(*right) + offset),
            ]
        )
    faces = [(2 * i, 2 * i + 1, 2 * i + 3, 2 * i + 2) for i in range(len(centerline) - 1)]
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
    terrain: Terrain,
    parent: bpy.types.Collection,
    mat: bpy.types.Material,
) -> bpy.types.Object | None:
    base = polygon_points(points)
    if len(base) < 3:
        return None
    base_z = min(terrain.z(x, y) for x, y, _z in base) + 0.03
    vertices = [(x, y, base_z) for x, y, _z in base] + [(x, y, base_z + height) for x, y, _z in base]
    count = len(base)
    faces: list[tuple[int, ...]] = [tuple(reversed(range(count))), tuple(range(count, 2 * count))]
    faces.extend((i, (i + 1) % count, count + (i + 1) % count, count + i) for i in range(count))
    return mesh_object(name, vertices, faces, parent, mat)


def subdivided_triangle(
    triangle: tuple[Vector, Vector, Vector],
    terrain: Terrain,
    offset: float,
    vertices: list[tuple[float, float, float]],
    faces: list[tuple[int, ...]],
) -> None:
    a, b, c = triangle
    if max((a - b).length, (b - c).length, (c - a).length) > AREA_SAMPLE_STEP_M:
        ab, bc, ca = (a + b) * 0.5, (b + c) * 0.5, (c + a) * 0.5
        for child in ((a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)):
            subdivided_triangle(child, terrain, offset, vertices, faces)
        return
    start = len(vertices)
    vertices.extend((point.x, point.y, terrain.z(point.x, point.y) + offset) for point in triangle)
    faces.append((start, start + 1, start + 2))


def draped_polygon(
    name: str,
    points: list[list[float]],
    terrain: Terrain,
    parent: bpy.types.Collection,
    mat: bpy.types.Material,
) -> bpy.types.Object | None:
    boundary = polygon_points(points)
    if len(boundary) < 3:
        return None
    polygon = [Vector((x, y, 0.0)) for x, y, _z in boundary]
    triangles = tessellate_polygon([polygon])
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, ...]] = []
    for triangle in triangles:
        subdivided_triangle(tuple(polygon[index] for index in triangle), terrain, AREA_SURFACE_OFFSET_M, vertices, faces)
    return mesh_object(name, vertices, faces, parent, mat) if faces else None


def junction_geometry(
    east: float, north: float, radius: float, terrain: Terrain
) -> tuple[list[tuple[float, float, float]], list[tuple[int, ...]]]:
    segments = 16
    offset = ROAD_SURFACE_OFFSET_M + 0.004
    vertices = [(east, north, terrain.z(east, north) + offset)]
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        x = east + radius * math.cos(angle)
        y = north + radius * math.sin(angle)
        vertices.append((x, y, terrain.z(x, y) + offset))
    faces = [(0, index + 1, (index + 1) % segments + 1) for index in range(segments)]
    return vertices, faces


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
    junction_collection = collection("Road_Junctions")
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
    terrain_mesh(terrain, terrain_collection, mats["terrain"])
    road_endpoints: dict[tuple[float, float], list[tuple[float, float, float]]] = {}
    for feature in scene_data["features"]:
        kind = feature["kind"]
        points = feature["points_blender_xyz_m"]
        osm_id = feature["osm_way_id"]
        if kind == "road" and len(points) >= 2:
            surface_offset = ROAD_SURFACE_OFFSET_M if feature["drivable"] else PATH_SURFACE_OFFSET_M
            vertices, faces = strip_geometry(points, float(feature["width_m"]), terrain, surface_offset)
            target = roads_collection if feature["drivable"] else paths_collection
            prefix = "NAV_Road" if feature["drivable"] else "Path"
            obj = mesh_object(f"{prefix}_{osm_id}", vertices, faces, target, mats["road" if feature["drivable"] else "path"])
            obj["width_m"] = float(feature["width_m"])
            obj["width_source"] = feature["width_source"]
            obj["drivable"] = bool(feature["drivable"])
            attach_osm_metadata(obj, feature)
            if feature["drivable"]:
                for point in (points[0], points[-1]):
                    if abs(point[0]) < terrain.extent - 0.1 and abs(point[1]) < terrain.extent - 0.1:
                        key = (round(point[0], 2), round(point[1], 2))
                        road_endpoints.setdefault(key, []).append((point[0], point[1], float(feature["width_m"])))
        elif kind == "parking":
            attach_osm_metadata(draped_polygon(f"Parking_{osm_id}", points, terrain, parking_collection, mats["parking"]), feature)
        elif kind == "park":
            attach_osm_metadata(draped_polygon(f"Park_{osm_id}", points, terrain, parks_collection, mats["park"]), feature)
        elif kind == "building":
            obj = extruded_polygon(
                f"Building_{osm_id}",
                points,
                building_height(feature.get("tags", {})),
                terrain,
                buildings_collection,
                mats["building"],
            )
            attach_osm_metadata(obj, feature)
    for index, entries in enumerate(road_endpoints.values()):
        if len(entries) < 2:
            continue
        east, north, _width = entries[0]
        radius = max(item[2] for item in entries) * 0.52
        vertices, faces = junction_geometry(east, north, radius, terrain)
        mesh_object(f"Road_Junction_{index:03d}", vertices, faces, junction_collection, mats["road"])
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

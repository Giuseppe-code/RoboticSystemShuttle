#!/usr/bin/env python3
"""Fetch OSM geometry and DEM elevations, then prepare road metadata for Blender/Godot."""

from __future__ import annotations

import argparse
import json
import math
import re
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any


USER_AGENT = "shuttle-osm-to-blender/1.0 (autonomous-bus simulation prototype)"
DRIVABLE_TYPES = {
    "motorway",
    "trunk",
    "primary",
    "secondary",
    "tertiary",
    "unclassified",
    "residential",
    "living_street",
    "service",
    "track",
}
DEFAULT_WIDTHS_M = {
    "motorway": 14.0,
    "trunk": 12.0,
    "primary": 10.5,
    "secondary": 8.0,
    "tertiary": 7.0,
    "unclassified": 6.0,
    "residential": 6.0,
    "living_street": 5.0,
    "service": 4.0,
    "track": 3.5,
    "pedestrian": 5.0,
    "footway": 2.0,
    "cycleway": 2.5,
    "path": 1.5,
    "steps": 1.5,
}


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, ensure_ascii=True, indent=2)
        handle.write("\n")


def request_json(url: str, *, data: bytes | None = None, attempts: int = 3) -> Any:
    request = urllib.request.Request(
        url,
        data=data,
        headers={"User-Agent": USER_AGENT, "Content-Type": "application/json"},
        method="POST" if data else "GET",
    )
    for attempt in range(attempts):
        try:
            with urllib.request.urlopen(request, timeout=120) as response:
                return json.loads(response.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError):
            if attempt == attempts - 1:
                raise
            time.sleep(2.0 * (attempt + 1))
    raise RuntimeError("unreachable")


def project_point(lat: float, lon: float, lat0: float, lon0: float) -> list[float]:
    east = (lon - lon0) * 111_320.0 * math.cos(math.radians(lat0))
    north = (lat - lat0) * 110_540.0
    return [round(east, 4), round(north, 4)]


def geographic_point(east: float, north: float, lat0: float, lon0: float) -> tuple[float, float]:
    lat = lat0 + north / 110_540.0
    lon = lon0 + east / (111_320.0 * math.cos(math.radians(lat0)))
    return lat, lon


def line_segment_clip(
    p0: list[float], p1: list[float], extent: float
) -> tuple[list[float], list[float]] | None:
    """Liang-Barsky clip to a local square extent."""
    x0, y0 = p0
    dx, dy = p1[0] - x0, p1[1] - y0
    lower, upper = 0.0, 1.0
    for p, q in ((-dx, x0 + extent), (dx, extent - x0), (-dy, y0 + extent), (dy, extent - y0)):
        if abs(p) < 1e-9:
            if q < 0:
                return None
            continue
        ratio = q / p
        if p < 0:
            lower = max(lower, ratio)
        else:
            upper = min(upper, ratio)
        if lower > upper:
            return None
    return (
        [round(x0 + lower * dx, 4), round(y0 + lower * dy, 4)],
        [round(x0 + upper * dx, 4), round(y0 + upper * dy, 4)],
    )


def clip_polyline(points: list[list[float]], extent: float) -> list[list[list[float]]]:
    parts: list[list[list[float]]] = []
    current: list[list[float]] = []
    for left, right in zip(points, points[1:]):
        clipped = line_segment_clip(left, right, extent)
        if clipped is None:
            if len(current) >= 2:
                parts.append(current)
            current = []
            continue
        start, end = clipped
        if not current or current[-1] != start:
            if len(current) >= 2:
                parts.append(current)
            current = [start]
        current.append(end)
    if len(current) >= 2:
        parts.append(current)
    return parts


def bbox_intersects(points: list[list[float]], extent: float) -> bool:
    if not points:
        return False
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return max(xs) >= -extent and min(xs) <= extent and max(ys) >= -extent and min(ys) <= extent


def parse_width(tags: dict[str, str]) -> tuple[float, str]:
    width_value = tags.get("width")
    if width_value:
        match = re.search(r"(\d+(?:[.,]\d+)?)", width_value)
        if match:
            return float(match.group(1).replace(",", ".")), "osm:width"
    lanes_value = tags.get("lanes")
    if lanes_value and lanes_value.isdigit():
        return max(3.0, int(lanes_value) * 3.1 + 0.8), "derived:lanes"
    highway = tags.get("highway", "service")
    return DEFAULT_WIDTHS_M.get(highway, 4.0), "derived:highway_default"


class TerrainGrid:
    def __init__(self, extent: float, size: int, elevations: list[float]) -> None:
        self.extent = extent
        self.size = size
        self.elevations = elevations
        self.base_elevation = min(elevations)
        self.spacing = 2.0 * extent / (size - 1)

    def relative_z(self, east: float, north: float) -> float:
        col = max(0.0, min(self.size - 1.0, (east + self.extent) / self.spacing))
        row = max(0.0, min(self.size - 1.0, (north + self.extent) / self.spacing))
        c0, r0 = int(math.floor(col)), int(math.floor(row))
        c1, r1 = min(c0 + 1, self.size - 1), min(r0 + 1, self.size - 1)
        tx, ty = col - c0, row - r0
        z00 = self.elevations[r0 * self.size + c0]
        z10 = self.elevations[r0 * self.size + c1]
        z01 = self.elevations[r1 * self.size + c0]
        z11 = self.elevations[r1 * self.size + c1]
        elevation = (
            z00 * (1 - tx) * (1 - ty)
            + z10 * tx * (1 - ty)
            + z01 * (1 - tx) * ty
            + z11 * tx * ty
        )
        return round(elevation - self.base_elevation, 4)

    def as_json(self) -> dict[str, Any]:
        samples: list[list[float]] = []
        for row in range(self.size):
            north = -self.extent + self.spacing * row
            for col in range(self.size):
                east = -self.extent + self.spacing * col
                index = row * self.size + col
                samples.append(
                    [
                        round(east, 4),
                        round(north, 4),
                        round(self.elevations[index] - self.base_elevation, 4),
                    ]
                )
        return {
            "grid_size": self.size,
            "extent_m": self.extent,
            "spacing_m": round(self.spacing, 4),
            "base_elevation_m": round(self.base_elevation, 3),
            "samples_blender_xyz_m": samples,
        }


def fetch_elevations(config: dict[str, Any]) -> TerrainGrid:
    center = config["center"]
    lat0, lon0 = center["latitude"], center["longitude"]
    extent, size = float(config["radius_m"]), int(config["terrain_grid_size"])
    spacing = 2.0 * extent / (size - 1)
    locations: list[dict[str, float]] = []
    for row in range(size):
        north = -extent + spacing * row
        for col in range(size):
            east = -extent + spacing * col
            lat, lon = geographic_point(east, north, lat0, lon0)
            locations.append({"latitude": lat, "longitude": lon})
    elevations: list[float] = []
    endpoint = config["sources"]["elevation_api"]
    if "opentopodata.org" in endpoint:
        for offset in range(0, len(locations), 80):
            chunk = locations[offset : offset + 80]
            encoded = "|".join(f"{item['latitude']},{item['longitude']}" for item in chunk)
            url = f"{endpoint}?{urllib.parse.urlencode({'locations': encoded})}"
            response = request_json(url)
            if response.get("status") != "OK":
                raise RuntimeError(f"Elevation request failed: {response}")
            elevations.extend(float(value["elevation"]) for value in response["results"])
            if offset + 80 < len(locations):
                time.sleep(1.05)
    else:
        for offset in range(0, len(locations), 120):
            payload = json.dumps({"locations": locations[offset : offset + 120]}).encode("utf-8")
            values = request_json(endpoint, data=payload)["results"]
            elevations.extend(float(value["elevation"]) for value in values)
    if len(elevations) != size * size:
        raise RuntimeError("Elevation response size does not match terrain grid")
    return TerrainGrid(extent, size, elevations)


def fetch_osm(config: dict[str, Any]) -> dict[str, Any]:
    center = config["center"]
    radius = int(config["radius_m"])
    around = f"(around:{radius},{center['latitude']},{center['longitude']})"
    query = (
        "[out:json][timeout:90];("
        f"way{around}[highway];"
        f"way{around}[amenity=parking];"
        f"way{around}[parking];"
        f"way{around}[leisure=park];"
        f"way{around}[building];"
        ");out tags geom;"
    )
    endpoint = config["sources"]["overpass_api"]
    data = urllib.parse.urlencode({"data": query}).encode("utf-8")
    return request_json(endpoint, data=data)


def local_geometry(
    element: dict[str, Any], config: dict[str, Any]
) -> list[list[float]]:
    center = config["center"]
    return [
        project_point(item["lat"], item["lon"], center["latitude"], center["longitude"])
        for item in element.get("geometry", [])
    ]


def feature_kind(tags: dict[str, str]) -> str | None:
    if "highway" in tags:
        return "road"
    if tags.get("amenity") == "parking" or "parking" in tags:
        return "parking"
    if tags.get("leisure") == "park":
        return "park"
    if "building" in tags:
        return "building"
    return None


def add_elevation(points: list[list[float]], terrain: TerrainGrid, offset: float = 0.0) -> list[list[float]]:
    return [[point[0], point[1], round(terrain.relative_z(point[0], point[1]) + offset, 4)] for point in points]


def calculate_curve(points: list[list[float]], index: int) -> dict[str, float] | None:
    if index <= 0 or index >= len(points) - 1:
        return None
    a, b, c = points[index - 1], points[index], points[index + 1]
    ab = math.hypot(b[0] - a[0], b[1] - a[1])
    bc = math.hypot(c[0] - b[0], c[1] - b[1])
    ac = math.hypot(c[0] - a[0], c[1] - a[1])
    cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
    if min(ab, bc, ac) < 0.05 or abs(cross) < 0.01:
        return None
    radius = ab * bc * ac / (2.0 * abs(cross))
    dot = (a[0] - b[0]) * (c[0] - b[0]) + (a[1] - b[1]) * (c[1] - b[1])
    angle = math.degrees(math.acos(max(-1.0, min(1.0, dot / (ab * bc)))))
    return {"point_index": index, "radius_m": round(radius, 3), "turn_angle_deg": round(180.0 - angle, 3)}


def road_network_entry(
    osm_id: int,
    part_index: int,
    tags: dict[str, str],
    points: list[list[float]],
    terrain: TerrainGrid,
) -> dict[str, Any]:
    width, width_source = parse_width(tags)
    elevated = add_elevation(points, terrain, 0.045)
    godot_points = [[p[0], p[2], -p[1]] for p in elevated]
    grade_segments = []
    total_length = 0.0
    weighted_grade = 0.0
    for index, (left, right) in enumerate(zip(elevated, elevated[1:])):
        length = math.hypot(right[0] - left[0], right[1] - left[1])
        grade = 100.0 * (right[2] - left[2]) / length if length else 0.0
        grade_segments.append({"from_index": index, "length_m": round(length, 3), "grade_pct": round(grade, 3)})
        total_length += length
        weighted_grade += grade * length
    curves = [curve for i in range(len(elevated)) if (curve := calculate_curve(elevated, i))]
    grades = [part["grade_pct"] for part in grade_segments] or [0.0]
    return {
        "id": f"way/{osm_id}/part/{part_index}",
        "osm_way_id": osm_id,
        "highway": tags.get("highway"),
        "name": tags.get("name", ""),
        "drivable": tags.get("highway") in DRIVABLE_TYPES and tags.get("access") != "no",
        "oneway": tags.get("oneway") in {"yes", "1", "true"},
        "lanes": int(tags["lanes"]) if tags.get("lanes", "").isdigit() else None,
        "width_m": round(width, 3),
        "width_source": width_source,
        "tags": tags,
        "length_m": round(total_length, 3),
        "points_blender_xyz_m": elevated,
        "points_godot_xyz_m": godot_points,
        "grade_segments": grade_segments,
        "grade_summary_pct": {
            "min": min(grades),
            "max": max(grades),
            "mean_weighted": round(weighted_grade / total_length if total_length else 0.0, 3),
        },
        "curves": curves,
    }


def create_dataset(config: dict[str, Any], raw: dict[str, Any], terrain: TerrainGrid) -> tuple[dict[str, Any], dict[str, Any]]:
    extent = float(config["radius_m"])
    seen: set[int] = set()
    scene_features: list[dict[str, Any]] = []
    road_network: list[dict[str, Any]] = []
    for element in raw.get("elements", []):
        osm_id = int(element["id"])
        if osm_id in seen:
            continue
        seen.add(osm_id)
        tags: dict[str, str] = element.get("tags", {})
        kind = feature_kind(tags)
        geometry = local_geometry(element, config)
        if not kind or len(geometry) < 2:
            continue
        if kind == "road":
            for part_index, part in enumerate(clip_polyline(geometry, extent)):
                road = road_network_entry(osm_id, part_index, tags, part, terrain)
                road_network.append(road)
                scene_features.append(
                    {
                        "kind": "road",
                        "id": road["id"],
                        "osm_way_id": osm_id,
                        "tags": tags,
                        "width_m": road["width_m"],
                        "width_source": road["width_source"],
                        "drivable": road["drivable"],
                        "points_blender_xyz_m": road["points_blender_xyz_m"],
                    }
                )
        elif bbox_intersects(geometry, extent):
            scene_features.append(
                {
                    "kind": kind,
                    "id": f"way/{osm_id}",
                    "osm_way_id": osm_id,
                    "tags": tags,
                    "points_blender_xyz_m": add_elevation(geometry, terrain, 0.02),
                }
            )
    drivable = [road for road in road_network if road["drivable"]]
    road_width_tagged = sum(road["width_source"] == "osm:width" for road in road_network)
    summary = {
        "features_total": len(scene_features),
        "roads_total": len(road_network),
        "roads_drivable": len(drivable),
        "roads_with_explicit_osm_width": road_width_tagged,
        "parking_areas": sum(item["kind"] == "parking" for item in scene_features),
        "parks": sum(item["kind"] == "park" for item in scene_features),
        "buildings": sum(item["kind"] == "building" for item in scene_features),
        "drivable_length_m": round(sum(road["length_m"] for road in drivable), 3),
    }
    provenance = {
        "area": config["name"],
        "center": config["center"],
        "radius_m": config["radius_m"],
        "coordinate_system": {
            "origin": "configured map center",
            "blender": "X=east, Y=north, Z=relative elevation in metres",
            "godot": "X=east, Y=relative elevation, Z=-north in metres (glTF convention)",
        },
        "data_limitations": [
            "Road widths use OSM width tags when present; otherwise they are estimated from lanes or road class.",
            "Elevation and grade are SRTM 90m DEM estimates sampled through OpenTopoData, not surveyed road profiles.",
            "OSM geometry describes mapped centerlines and polygons; lane boundaries and curbs may be absent.",
        ],
        "sources": config["sources"],
    }
    scene_data = {
        "provenance": provenance,
        "summary": summary,
        "terrain": terrain.as_json(),
        "features": scene_features,
    }
    network_data = {
        "provenance": provenance,
        "summary": summary,
        "roads": road_network,
    }
    return scene_data, network_data


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config/parco_gandhi_catania.json"))
    parser.add_argument("--output-dir", type=Path, default=Path("data/parco_gandhi_catania"))
    args = parser.parse_args()
    config = load_json(args.config)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    raw_osm = fetch_osm(config)
    terrain = fetch_elevations(config)
    scene_data, network_data = create_dataset(config, raw_osm, terrain)
    write_json(args.output_dir / "osm_overpass_raw.json", raw_osm)
    write_json(args.output_dir / "scene_data.json", scene_data)
    write_json(args.output_dir / "road_network.json", network_data)
    print(json.dumps(scene_data["summary"], indent=2))
    print(f"Base elevation: {scene_data['terrain']['base_elevation_m']} m")


if __name__ == "__main__":
    main()

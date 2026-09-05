#!/usr/bin/env python3
"""Refine provisional L402 semantics from the latest calibrated real-map data.

The global sim-to-real transform remains the baseline.  A local inverse-distance
weighted displacement derived from nearby calibrated objects adjusts each
remaining provisional object.  Narrow-passage poses and centerlines are rebuilt
from the user-authored alignment-zone polygons.
"""

from __future__ import annotations

import argparse
import cmath
import copy
import math
from pathlib import Path
from typing import Any

import yaml


CURRENT_TO_REFERENCE = {
    "front-door": "front_door",
    "back-door": "back_door",
    "rxy-table": "desk_rxy",
    "tool-box": "tool_box",
}


def load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def save_yaml(path: Path, data: dict[str, Any]) -> None:
    path.write_text(
        yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def polygon_center(points: list[dict[str, Any]]) -> complex:
    if not points:
        raise ValueError("Cannot compute the center of an empty polygon")
    return sum(complex(float(point["x"]), float(point["y"])) for point in points) / len(points)


def transform_point(point: dict[str, Any], coefficient: complex, offset: complex) -> complex:
    return coefficient * complex(float(point["x"]), float(point["y"])) + offset


def reference_center(item: dict[str, Any], coefficient: complex, offset: complex) -> complex:
    return sum(transform_point(point, coefficient, offset) for point in item["map_polygon"]) / len(
        item["map_polygon"]
    )


def shift_xy(item: dict[str, Any], delta: complex) -> None:
    item["x"] = round(float(item["x"]) + delta.real, 4)
    item["y"] = round(float(item["y"]) + delta.imag, 4)


def shift_geometry(item: dict[str, Any], delta: complex) -> None:
    for point in item.get("map_polygon") or []:
        shift_xy(point, delta)
    if item.get("map_pose"):
        shift_xy(item["map_pose"], delta)


def local_displacement(
    position: complex,
    anchors: list[tuple[str, complex, complex]],
    neighbor_count: int,
) -> tuple[complex, list[str]]:
    nearest = sorted(anchors, key=lambda anchor: abs(anchor[1] - position))[:neighbor_count]
    weights = [1.0 / max(abs(anchor_position - position), 0.2) ** 2 for _, anchor_position, _ in nearest]
    displacement = sum(weight * delta for weight, (_, _, delta) in zip(weights, nearest)) / sum(weights)
    return displacement, [name for name, _, _ in nearest]


def update_object_geometry(item: dict[str, Any], segment: dict[str, Any], delta: complex) -> None:
    region = item.get("map_region")
    if region is not None:
        region["polygon"] = copy.deepcopy(segment.get("map_polygon") or [])
    if item.get("pose"):
        shift_xy(item["pose"], delta)
    for approach in item.get("approach_poses") or []:
        pose = approach.get("pose", approach)
        if "x" in pose and "y" in pose:
            shift_xy(pose, delta)


def refine_semantics(
    reference_dir: Path,
    current_dir: Path,
    neighbor_count: int,
) -> dict[str, Any]:
    segment_path = current_dir / "semantic_segments.yaml"
    semantic_path = current_dir / "semantic_map.yaml"
    metadata_path = current_dir / "metadata.yaml"
    segments_data = load_yaml(segment_path)
    semantic_data = load_yaml(semantic_path)
    metadata = load_yaml(metadata_path)
    reference_segments_data = load_yaml(reference_dir / "semantic_segments.yaml")

    transform = metadata["semantic_import"]
    coefficient = float(transform["scale"]) * cmath.exp(
        1j * math.radians(float(transform["yaw_deg"]))
    )
    offset = complex(float(transform["translation_x"]), float(transform["translation_y"]))

    segments = {item["id"]: item for item in segments_data.get("segments", [])}
    reference_segments = {
        item["id"]: item for item in reference_segments_data.get("segments", [])
    }
    objects = {item["id"]: item for item in semantic_data.get("objects", [])}

    anchors: list[tuple[str, complex, complex]] = []
    for current_id, segment in segments.items():
        reference_id = CURRENT_TO_REFERENCE.get(current_id, current_id)
        if (
            not segment.get("calibrated")
            or current_id == "tool-box"
            or reference_id not in reference_segments
        ):
            continue
        initial_center = reference_center(reference_segments[reference_id], coefficient, offset)
        anchors.append((current_id, initial_center, polygon_center(segment["map_polygon"]) - initial_center))

    if len(anchors) < neighbor_count:
        raise ValueError(f"Need at least {neighbor_count} calibrated anchors, found {len(anchors)}")

    refined: dict[str, Any] = {}
    for current_id, segment in segments.items():
        reference_id = CURRENT_TO_REFERENCE.get(current_id, current_id)
        if segment.get("calibrated") or reference_id not in reference_segments:
            continue
        initial_center = reference_center(reference_segments[reference_id], coefficient, offset)
        displacement, neighbors = local_displacement(initial_center, anchors, neighbor_count)
        desired_center = initial_center + displacement
        current_center = polygon_center(segment["map_polygon"])
        applied_shift = desired_center - current_center
        shift_geometry(segment, applied_shift)
        segment["refinement"] = {
            "method": "local_idw_from_calibrated_real_objects",
            "neighbor_count": neighbor_count,
            "anchors": neighbors,
            "delta_x": round(displacement.real, 4),
            "delta_y": round(displacement.imag, 4),
        }
        if current_id in objects:
            update_object_geometry(objects[current_id], segment, applied_shift)
            objects[current_id]["refinement"] = copy.deepcopy(segment["refinement"])
        refined[current_id] = {
            "center_x": round(desired_center.real, 4),
            "center_y": round(desired_center.imag, 4),
            "delta_x": round(displacement.real, 4),
            "delta_y": round(displacement.imag, 4),
            "anchors": neighbors,
        }

    metadata["semantic_refinement"] = {
        "method": "local_idw_from_calibrated_real_objects",
        "calibrated_anchor_count": len(anchors),
        "excluded_anchor": "tool-box (movable)",
        "neighbor_count": neighbor_count,
        "refined_objects": list(refined),
        "note": "Geometry is still provisional and remains disabled until individually verified.",
    }
    save_yaml(segment_path, segments_data)
    save_yaml(semantic_path, semantic_data)
    save_yaml(metadata_path, metadata)
    return {"anchor_count": len(anchors), "refined": refined}


def refine_passages(current_dir: Path) -> dict[str, Any]:
    annotation_path = current_dir / "navigation_annotations.yaml"
    data = load_yaml(annotation_path)
    changes: dict[str, Any] = {}
    for passage in data.get("narrow_passages", []):
        zones = {zone.get("side"): zone for zone in passage.get("alignment_zones", [])}
        if "entry" not in zones or "exit" not in zones:
            continue
        entry_center = polygon_center(zones["entry"].get("polygon") or [])
        exit_center = polygon_center(zones["exit"].get("polygon") or [])
        forward_yaw = math.atan2((exit_center - entry_center).imag, (exit_center - entry_center).real)
        reverse_yaw = math.atan2((entry_center - exit_center).imag, (entry_center - exit_center).real)

        passage["entry"] = {
            "x": round(entry_center.real, 4),
            "y": round(entry_center.imag, 4),
            "yaw": forward_yaw,
        }
        passage["exit"] = {
            "x": round(exit_center.real, 4),
            "y": round(exit_center.imag, 4),
            "yaw": forward_yaw,
        }
        passage["centerline"] = [
            {"x": round(entry_center.real, 4), "y": round(entry_center.imag, 4)},
            {"x": round(exit_center.real, 4), "y": round(exit_center.imag, 4)},
        ]
        zones["entry"]["pose"] = {
            "x": round(entry_center.real, 4),
            "y": round(entry_center.imag, 4),
            "yaw": forward_yaw,
        }
        zones["exit"]["pose"] = {
            "x": round(exit_center.real, 4),
            "y": round(exit_center.imag, 4),
            "yaw": reverse_yaw,
        }
        passage["pose_source"] = "alignment_zone_polygon_centers"
        changes[passage["id"]] = {
            "entry_center": [round(entry_center.real, 4), round(entry_center.imag, 4)],
            "exit_center": [round(exit_center.real, 4), round(exit_center.imag, 4)],
            "forward_yaw": round(forward_yaw, 6),
        }
    save_yaml(annotation_path, data)
    return changes


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--current", type=Path, required=True)
    parser.add_argument("--neighbors", type=int, default=4)
    args = parser.parse_args()
    result = refine_semantics(args.reference.resolve(), args.current.resolve(), args.neighbors)
    result["passages"] = refine_passages(args.current.resolve())
    print(yaml.safe_dump(result, sort_keys=False).strip())


if __name__ == "__main__":
    main()

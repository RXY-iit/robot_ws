#!/usr/bin/env python3
"""Merge sim_real_lab_001 semantics into the L402 real-map bundle.

Existing real-map annotations always win. Remaining reference entries are
transformed by a similarity fit computed from fixed landmarks and are kept
non-navigation-ready until they are calibrated on the real map.
"""

from __future__ import annotations

import argparse
import cmath
import copy
import math
from pathlib import Path
from typing import Any

import yaml


MANUAL_TO_REFERENCE = {
    "front-door": "front_door",
    "back-door": "back_door",
    "sofa": "sofa",
    "tool-box": "tool_box",
    "rxy-table": "desk_rxy",
}

# tool_box is intentionally excluded: it is movable and its two annotations do
# not describe the same physical location closely enough for frame fitting.
FIT_LANDMARKS = {
    "front-door": "front_door",
    "back-door": "back_door",
    "sofa": "sofa",
    "rxy-table": "desk_rxy",
}


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def save_yaml(path: Path, data: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)


def centroid(segment: dict[str, Any]) -> complex:
    polygon = segment.get("map_polygon") or []
    if not polygon:
        pose = segment.get("map_pose") or {}
        return complex(float(pose["x"]), float(pose["y"]))
    return sum(complex(float(p["x"]), float(p["y"])) for p in polygon) / len(polygon)


def fit_similarity(
    current_segments: dict[str, dict[str, Any]],
    reference_segments: dict[str, dict[str, Any]],
) -> tuple[complex, complex, dict[str, float]]:
    source = [centroid(reference_segments[ref_id]) for ref_id in FIT_LANDMARKS.values()]
    target = [centroid(current_segments[cur_id]) for cur_id in FIT_LANDMARKS]
    source_mean = sum(source) / len(source)
    target_mean = sum(target) / len(target)
    denominator = sum(abs(value - source_mean) ** 2 for value in source)
    coefficient = sum(
        (value - source_mean).conjugate() * (expected - target_mean)
        for value, expected in zip(source, target)
    ) / denominator
    offset = target_mean - coefficient * source_mean
    errors = [abs(coefficient * value + offset - expected) for value, expected in zip(source, target)]
    diagnostics = {
        "scale": abs(coefficient),
        "yaw_deg": math.degrees(cmath.phase(coefficient)),
        "translation_x": offset.real,
        "translation_y": offset.imag,
        "landmark_rms_error_m": math.sqrt(sum(error * error for error in errors) / len(errors)),
        "landmark_max_error_m": max(errors),
    }
    return coefficient, offset, diagnostics


def transform_xy(point: dict[str, Any], coefficient: complex, offset: complex) -> dict[str, Any]:
    transformed = coefficient * complex(float(point["x"]), float(point["y"])) + offset
    result = copy.deepcopy(point)
    result["x"] = round(transformed.real, 4)
    result["y"] = round(transformed.imag, 4)
    return result


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def transform_pose(pose: dict[str, Any] | None, coefficient: complex, offset: complex) -> dict[str, Any] | None:
    if not pose:
        return pose
    result = transform_xy(pose, coefficient, offset)
    if "yaw" in result:
        result["yaw"] = normalize_angle(float(result["yaw"]) + cmath.phase(coefficient))
    return result


def transform_polygon(points: list[dict[str, Any]], coefficient: complex, offset: complex) -> list[dict[str, Any]]:
    return [transform_xy(point, coefficient, offset) for point in points]


def transform_side(side: str | None, coefficient: complex) -> str:
    vectors = {
        "east": complex(1.0, 0.0),
        "west": complex(-1.0, 0.0),
        "north": complex(0.0, 1.0),
        "south": complex(0.0, -1.0),
    }
    if side not in vectors:
        return side or "west"
    rotated = coefficient / abs(coefficient) * vectors[side]
    return max(vectors, key=lambda name: (rotated.real * vectors[name].real + rotated.imag * vectors[name].imag))


def provisional_note(existing: str | None, source_id: str) -> str:
    prefix = (
        f"Provisional transform from sim_real_lab_001/{source_id}; "
        "adjust polygon and approach side on the real L402 map before navigation."
    )
    return f"{prefix} {existing or ''}".strip()


def transform_segment(
    segment: dict[str, Any], coefficient: complex, offset: complex
) -> dict[str, Any]:
    result = copy.deepcopy(segment)
    result["map_polygon"] = transform_polygon(result.get("map_polygon") or [], coefficient, offset)
    result["map_pose"] = transform_pose(result.get("map_pose"), coefficient, offset)
    result["calibrated"] = False
    result["navigation_ready"] = False
    result["notes"] = provisional_note(result.get("notes"), result["id"])
    tags = list(result.get("tags") or [])
    for tag in ("transformed_from_sim", "needs_real_calibration"):
        if tag not in tags:
            tags.append(tag)
    result["tags"] = tags
    annotation = result.setdefault("annotation", {})
    annotation["color"] = "imported_provisional"
    policy = result.get("approach_policy") or {}
    if "preferred_side" in policy:
        policy["preferred_side"] = transform_side(policy["preferred_side"], coefficient)
    if "yaw" in policy and policy.get("yaw_mode") == "fixed":
        policy["yaw"] = normalize_angle(float(policy["yaw"]) + cmath.phase(coefficient))
    result["approach_policy"] = policy
    return result


def transform_object(
    item: dict[str, Any], coefficient: complex, offset: complex
) -> dict[str, Any]:
    result = copy.deepcopy(item)
    result["navigation_ready"] = False
    result["confidence"] = min(float(result.get("confidence", 0.3)), 0.2)
    result["note"] = provisional_note(result.get("note"), result["id"])
    if result.get("map_region"):
        region = result["map_region"]
        region["polygon"] = transform_polygon(region.get("polygon") or [], coefficient, offset)
        if "front_side" in region:
            region["front_side"] = transform_side(region["front_side"], coefficient)
    if result.get("pose"):
        result["pose"] = transform_pose(result["pose"], coefficient, offset)
    for approach in result.get("approach_poses") or []:
        if "pose" in approach:
            approach["pose"] = transform_pose(approach["pose"], coefficient, offset)
        elif "x" in approach and "y" in approach:
            transformed = transform_pose(approach, coefficient, offset)
            approach.clear()
            approach.update(transformed or {})
    policy = result.get("approach_policy") or {}
    if "preferred_side" in policy:
        policy["preferred_side"] = transform_side(policy["preferred_side"], coefficient)
    if "yaw" in policy and policy.get("yaw_mode") == "fixed":
        policy["yaw"] = normalize_angle(float(policy["yaw"]) + cmath.phase(coefficient))
    result["approach_policy"] = policy
    return result


def transform_passage(
    passage: dict[str, Any], coefficient: complex, offset: complex
) -> dict[str, Any]:
    result = copy.deepcopy(passage)
    result["enabled"] = False
    result["calibrated"] = False
    result["description"] = provisional_note(result.get("description"), result["id"])
    result["entry"] = transform_pose(result.get("entry"), coefficient, offset)
    result["exit"] = transform_pose(result.get("exit"), coefficient, offset)
    result["centerline"] = transform_polygon(result.get("centerline") or [], coefficient, offset)
    for zone in result.get("alignment_zones") or []:
        zone["enabled"] = False
        zone["polygon"] = transform_polygon(zone.get("polygon") or [], coefficient, offset)
        zone["pose"] = transform_pose(zone.get("pose"), coefficient, offset)
    if "expected_width_m" in result:
        result["expected_width_m"] = round(float(result["expected_width_m"]) * abs(coefficient), 3)
    return result


def merge(reference_dir: Path, current_dir: Path) -> dict[str, Any]:
    current_map_path = current_dir / "semantic_map.yaml"
    current_segments_path = current_dir / "semantic_segments.yaml"
    current_annotations_path = current_dir / "navigation_annotations.yaml"

    current_map = load_yaml(current_map_path)
    current_segments = load_yaml(current_segments_path)
    reference_map = load_yaml(reference_dir / "semantic_map.yaml")
    reference_segments = load_yaml(reference_dir / "semantic_segments.yaml")
    reference_annotations = load_yaml(reference_dir / "navigation_annotations.yaml")

    current_segment_by_id = {item["id"]: item for item in current_segments.get("segments", [])}
    reference_segment_by_id = {item["id"]: item for item in reference_segments.get("segments", [])}
    coefficient, offset, diagnostics = fit_similarity(current_segment_by_id, reference_segment_by_id)

    replaced_reference_ids = set(MANUAL_TO_REFERENCE.values())
    imported_ids = [
        segment["id"]
        for segment in reference_segments.get("segments", [])
        if segment["id"] not in replaced_reference_ids
    ]
    for segment in reference_segments.get("segments", []):
        if segment["id"] in replaced_reference_ids or segment["id"] in current_segment_by_id:
            continue
        transformed = transform_segment(segment, coefficient, offset)
        current_segments.setdefault("segments", []).append(transformed)
        current_segment_by_id[transformed["id"]] = transformed

    existing_object_ids = {item["id"] for item in current_map.get("objects", [])}
    for item in reference_map.get("objects", []):
        if item["id"] in replaced_reference_ids or item["id"] in existing_object_ids:
            continue
        transformed = transform_object(item, coefficient, offset)
        current_map.setdefault("objects", []).append(transformed)
        existing_object_ids.add(transformed["id"])

    existing_place_ids = {item["id"] for item in current_map.get("places", [])}
    for place in reference_map.get("places", []):
        if place["id"] in existing_place_ids:
            continue
        transformed = copy.deepcopy(place)
        transformed["navigation_ready"] = False
        transformed["confidence"] = min(float(transformed.get("confidence", 0.3)), 0.2)
        transformed["note"] = provisional_note(transformed.get("note"), transformed["id"])
        if transformed.get("pose"):
            transformed["pose"] = transform_pose(transformed["pose"], coefficient, offset)
        current_map.setdefault("places", []).append(transformed)

    current_map["status"] = "partial_manual_plus_provisional_transform"
    current_segments["source"] = {
        "type": "manual_plus_transformed_reference",
        "nav2_map": "maps/l402_2d_map_clean_0509.yaml",
        "reference_map": "maps/sim_real_lab_001",
        "note": "Five manual real-map regions are authoritative; imported regions require real-map calibration.",
    }
    annotations = {
        "map_id": current_map["map_id"],
        "frame_id": current_map.get("frame_id", "map"),
        "schema_version": reference_annotations.get("schema_version", 1),
        "status": "provisional_transformed_from_sim_disabled",
        "narrow_passages": [
            transform_passage(passage, coefficient, offset)
            for passage in reference_annotations.get("narrow_passages", [])
        ],
    }

    save_yaml(current_map_path, current_map)
    save_yaml(current_segments_path, current_segments)
    save_yaml(current_annotations_path, annotations)

    metadata_path = current_dir / "metadata.yaml"
    metadata = load_yaml(metadata_path)
    metadata["status"] = "semantic_partial_calibration"
    metadata["semantic_import"] = {
        "reference_map": "sim_real_lab_001",
        "method": "2d_similarity_from_manual_fixed_landmarks",
        "manual_landmarks": list(FIT_LANDMARKS),
        "manual_objects_preserved": list(MANUAL_TO_REFERENCE),
        "provisional_objects_imported": imported_ids,
        **{key: round(value, 6) for key, value in diagnostics.items()},
    }
    metadata["notes"] = [
        "The existing flat Nav2 map remains the navigation baseline.",
        "Five user-authored real-map objects are authoritative and navigation-ready.",
        "Remaining semantic objects were transformed from sim_real_lab_001 and are disabled for navigation until calibrated.",
        "Narrow-passage annotations were transformed but remain disabled until their entry, exit, and alignment zones are verified.",
    ]
    save_yaml(metadata_path, metadata)
    return {**diagnostics, "imported_ids": imported_ids}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--current", type=Path, required=True)
    args = parser.parse_args()
    result = merge(args.reference.resolve(), args.current.resolve())
    print(yaml.safe_dump(result, sort_keys=False).strip())


if __name__ == "__main__":
    main()

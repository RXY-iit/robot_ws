"""Validate capability declarations; expose one normalized registry to consumers."""
from copy import deepcopy
from pathlib import Path

import yaml

from robot_skills.registry import package_config_path


def normalize_targets(data):
    data = deepcopy(data)
    if data.get("schema_version", 1) not in {1, 2}:
        raise ValueError("Unsupported perception target schema version")
    if data.get("schema_version") != 2:
        return data  # Existing external v1 configurations retain their contract.
    classes = data.get("detector_classes", [])
    if not classes or len(classes) != len(set(classes)):
        raise ValueError("detector_classes must be nonempty and unique")
    objects = {}
    for name, item in data.get("object_aliases", {}).items():
        if not item.get("classes") or set(item["classes"]) - set(classes) or item.get("fallback_classes"):
            raise ValueError(f"Object alias {name} claims unsupported detector classes")
        objects[name] = item
    for name, item in data.get("unsupported_objects", {}).items():
        if name in objects or name in classes:
            raise ValueError(f"Conflicting supported/unsupported object: {name}")
        objects[name] = {**item, "classes": [], "supported": False}
    for name, item in data.get("colors", {}).items():
        if not item.get("hsv_ranges"):
            raise ValueError(f"No HSV mask for {name}")
        for bounds in item["hsv_ranges"]:
            low, high = bounds["lower"], bounds["upper"]
            if len(low) != 3 or len(high) != 3 or any(not 0 <= lo <= hi <= limit for lo, hi, limit in zip(low, high, [180, 255, 255])):
                raise ValueError(f"Invalid HSV range for {name}")
    # Compatibility is internal only: YAML distinguishes capabilities from names.
    data["objects"] = objects
    data["yolo_open_vocab_classes"] = classes
    return data


def load_targets(path=None):
    return normalize_targets(yaml.safe_load(Path(path or package_config_path("perception_targets.yaml")).read_text(encoding="utf-8")))


def question_capabilities(config):
    return {"detector_classes": config.get("detector_classes", config.get("yolo_open_vocab_classes", [])),
            "object_aliases": config.get("object_aliases", {}),
            "unsupported_objects": config.get("unsupported_objects", {}),
            "hsv_colors": list(config.get("colors", {})),
            "color_contract": config.get("color_contract", {}),
            "text_contract": config.get("text_contract", {})}

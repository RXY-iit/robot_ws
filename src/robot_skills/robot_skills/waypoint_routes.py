"""Frozen map-frame waypoint routes shared by Mission and the standalone Nav2 runner."""
import hashlib
import math
from pathlib import Path
import re

import yaml


def read_route(path, expected_map_id=None):
    path = Path(path)
    raw = path.read_bytes()
    data = yaml.safe_load(raw)
    if not isinstance(data, dict) or data.get("frame_id") != "map":
        raise ValueError("Route must specify frame_id: map")
    if expected_map_id is not None and data.get("map_id") != expected_map_id:
        raise ValueError(f"Route map {data.get('map_id')!r} does not match active map {expected_map_id!r}")
    points = data.get("waypoints")
    if not isinstance(points, list) or not points:
        raise ValueError("Route requires a nonempty waypoint list")
    poses = []
    for point in points:
        if not isinstance(point, dict) or not all(k in point for k in ("x", "y", "yaw")):
            raise ValueError("Each waypoint requires x/y/yaw")
        pose = {k: point[k] for k in ("x", "y", "yaw")}
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in pose.values()):
            raise ValueError("All x/y/yaw values must be finite numbers; yaw is in radians")
        poses.append(pose)
    return {**data, "poses": poses, "sha256": hashlib.sha256(raw).hexdigest(), "path": str(path.resolve())}


def named_route(name, map_id):
    if not isinstance(name, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", name.lower()):
        raise ValueError("Waypoint list name must be a registered identifier")
    filename = f"waypoint_list_{name.lower()}.yaml"
    path = Path(__file__).resolve().parents[1] / "config" / filename
    if not path.exists():
        from robot_skills.registry import package_config_path
        path = Path(package_config_path(filename))
    if not map_id:
        raise ValueError("Active semantic map must declare map_id")
    return read_route(path, expected_map_id=map_id)

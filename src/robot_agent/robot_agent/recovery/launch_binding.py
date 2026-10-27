"""Allowlisted Quick Start settings, never executable commands from settings."""
import json
import os
from pathlib import Path


def quickstart_context():
    raw = os.environ.get("RECOVERY_LAUNCH_SETTINGS")
    source = "Quick Start launch snapshot"
    if not raw:
        path = Path.home() / ".config/robot-sim-launcher/settings.json"
        try:
            raw = path.read_text()
        except OSError:
            return {"source": "Recovery defaults (no Quick Start context)"}
        source = "Quick Start saved settings (legacy fallback)"
    try:
        data = json.loads(raw)
        if (not isinstance(data, dict) or
                str(data.get("ros_domain_id")) != os.environ.get("ROS_DOMAIN_ID", "0") or
                data.get("ros_mode") != os.environ.get("RECOVERY_PROFILE", "nav2") or
                not isinstance(data.get("motion_policy"), bool)):
            return {"source": "Recovery defaults (Quick Start context mismatch)"}
    except (TypeError, ValueError):
        return {"source": "Recovery defaults (invalid Quick Start context)"}
    keys = ("ros_domain_id", "ros_mode", "robot_map_id", "map", "ply", "motion_policy",
            "initial_x", "initial_y", "initial_yaw")
    return {"source": source, **{k: data[k] for k in keys if k in data}}

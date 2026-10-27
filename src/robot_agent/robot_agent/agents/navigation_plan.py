"""Grounded navigation contracts. No generated coordinates or motion authority."""
from datetime import datetime, timezone
import hashlib
import json
import math


def choice(instructions, criteria):
    return {"type": "choice", "instructions": instructions, "criteria": criteria}


def applicable_rules(semantic, target_id, now=None):
    now = now or datetime.now(timezone.utc)
    active, errors = [], []
    for raw in semantic.data.get("navigation_rules", []):
        if not isinstance(raw, dict):
            errors.append("Invalid navigation rule")
            continue
        scope = raw.get("target_ids", [])
        if not isinstance(scope, list):
            errors.append("Rule target_ids must be a list")
            continue
        if scope and target_id not in scope:
            continue
        try:
            for key in ("valid_from", "valid_until"):
                if raw.get(key):
                    timestamp = datetime.fromisoformat(str(raw[key]).replace("Z", "+00:00"))
                    if timestamp.tzinfo is None:
                        raise ValueError("Rule timestamps must include timezone")
                    if (key == "valid_from" and now < timestamp) or (key == "valid_until" and now >= timestamp):
                        break
            else:
                if not raw.get("id") or not raw.get("revision"):
                    raise ValueError("Rule id/revision missing")
                if raw.get("effect") not in {"deny_destination", "require_visual_confirmation", "advisory"}:
                    raise ValueError("Unsupported rule requires an executable enforcement policy")
                active.append({k: raw[k] for k in ("id", "revision", "effect", "message", "target_ids",
                                                  "valid_from", "valid_until") if k in raw})
        except (ValueError, TypeError) as exc:
            errors.append(f"{raw.get('id', 'unknown_rule')}: {exc}")
    return active, errors


def path_evidence(path, target, annotation):
    points = [{"x": float(p["x"]), "y": float(p["y"])} for p in path]
    if not points or any(not math.isfinite(v) for p in points for v in p.values()):
        raise ValueError("Missing or invalid Nav2 path")
    fingerprint = hashlib.sha256(json.dumps(points, sort_keys=True).encode()).hexdigest()[:20]
    indices = sorted({round(i * (len(points) - 1) / 31) for i in range(32)})
    return {"path_id": fingerprint, "point_count": len(points), "frame_id": target["frame_id"],
            "length_m": sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(points, points[1:])),
            "sampled_points": [{k: round(v, 3) for k, v in points[i].items()} for i in indices], "target_pose": target,
            "annotation": {k: annotation.get(k) for k in ("passage_id", "description", "direction", "expected_width_m",
                                                         "waypoint_pose", "alignment_goal_pose", "motion_mode") if k in annotation},
            "scope": "Computed global path only; not proof of clearance or a constraint on future Nav2 replans"}

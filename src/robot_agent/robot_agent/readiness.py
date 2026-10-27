"""Read-only startup evidence, separate from fault diagnosis and motion authority."""
import math


class ReadinessTracker:
    def __init__(self):
        self.history = {}

    def update(self, domains, *, sim, now):
        items = {item["id"]: item for domain in domains for item in domain.get("items", [])}
        common = ["agent_heartbeat", "delivery_robot_mode", "delivery_safety", "safety"]
        if sim:
            clock = (items.get("sim_rtf") or {}).get("value") or {}
            ages = [clock.get("clock_receive_age_sec"), clock.get("not_advancing_sec")]
            valid = all(isinstance(v, (int, float)) and math.isfinite(v) and v >= 0 for v in ages)
            status = "UNKNOWN" if not valid else "OK" if max(ages) <= 2 else "BLOCKED"
            items["clock_advancing"] = {"status": status, "summary": "Clock delivery and advancement (wall age <= 2s)", "id": "clock_advancing"}
            common += ["clock_advancing"]
        definitions = [
            ("base", "Communication and safety", common, ["MANUAL is not a startup failure. Readiness does not authorize motion."]),
            ("navigation", "Navigation baseline", common + ["lidar_device_status", "gicp_score", "gicp_pose",
                "localization_consistency", "lifecycle_controller_server", "lifecycle_planner_server",
                "lifecycle_bt_navigator", "lifecycle_local_costmap"] + (["sim_time_consistency"] if sim else []),
             ["Goal, TF, odometry and AUTO still require execution prechecks. This does not validate a route."]),
            ("perception", "Visual acquisition", ["agent_heartbeat", "camera_color_diagnostics", "camera_pan", "camera_tilt"]
                + (["sim_camera_model", "timing:sim_camera_color"] if sim else []),
             ["Acquisition only; model loading and inference must be verified per task. RGB OCR does not require depth pairing."]),
            ("arm", "Arm control interface", common + ["so101_arm_gateway_delivery", "so101_arm_visual_joint_states"]
                + ([] if sim else ["so101_arm_hardware_status"]), ["Sim feedback does not prove real hardware execution. Each action requires limit and completion checks." if sim
                else "Each action requires limit and completion checks."]),
        ]
        groups = []
        for key, title, dependencies, notes in definitions:
            checks = [{"id": name, "status": (items.get(name) or {}).get("status", "UNKNOWN"),
                       "summary": (items.get(name) or {}).get("summary", "No monitoring evidence yet")}
                      for name in dependencies]
            bad = [c for c in checks if c["status"] in {"BLOCKED", "ERROR", "STALE"}]
            unknown = [c for c in checks if c["status"] not in {"OK", "OBSERVED", "AVAILABLE", "RECOVERED",
                       "WARN", "DEGRADED", "BLOCKED", "ERROR", "STALE"}]
            warnings = [c for c in checks if c["status"] in {"WARN", "DEGRADED"}]
            status = "BLOCKED" if bad else "WAITING" if unknown else "DEGRADED" if warnings else "READY"
            history = self.history.setdefault(key, {"first_ready_at": None, "ready_since": None,
                                                    "status": None, "status_since": now})
            if history["status"] != status:
                history["status"], history["status_since"] = status, now
                history["ready_since"] = now if status == "READY" else None
            if status == "READY" and history["first_ready_at"] is None:
                history["first_ready_at"] = now
            reasons = bad + unknown + warnings
            groups.append({"id": key, "title": title, **history, "checks": checks,
                           "summary": "; ".join(f"{c['id']}: {c['summary']}" for c in reasons[:2])
                           if reasons else "Monitored prerequisites are currently satisfied",
                           "notes": notes})
        # These are advisory/context only; do not manufacture a model-ready signal from an old success.
        supplemental = [items[name] for name in ("sim_rtf", "host_resources", "sim_sync:rgb_depth", "perception_runtime") if name in items]
        return {"generated_at": now, "scope": "monitored_prerequisites_only", "groups": groups,
                "supplemental": [{"id": i["id"], "status": i["status"], "summary": i["summary"]} for i in supplemental]}

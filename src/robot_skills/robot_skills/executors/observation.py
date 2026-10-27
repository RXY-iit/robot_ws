"""Bounded composition of existing skills; no ROS transport or language parsing."""
import math
import time
from dataclasses import replace
from uuid import uuid4

import yaml

from robot_skills import result_codes as codes
from robot_skills.executors.base import make_result
from robot_skills.registry import package_config_path


def observation_plan(name, args):
    if name == "move_camera_and_capture" or args.get("profile") == "directional":
        poses = [{"id": args.get("scan_band", "requested"), "pan_deg": args["pan_deg"], "tilt_deg": args["tilt_deg"]}]
        home = None
    else:
        with open(package_config_path("camera_scan_profiles.yaml"), encoding="utf-8") as stream:
            profiles = yaml.safe_load(stream)["profiles"]
        profile = profiles[args.get("profile", "shelf_label")]
        poses, home = profile["poses"], profile.get("return_pose")
    if not 1 <= len(poses) <= 12:
        raise ValueError("Invalid bounded view count")
    plan = []
    for pose in poses + ([home] if home else []):
        pan, tilt = float(pose["pan_deg"]), float(pose["tilt_deg"])
        if not all(math.isfinite(x) for x in (pan, tilt)) or not (-180 <= pan <= 180 and -90 <= tilt <= 132):
            raise ValueError("Invalid camera target")
        plan.append(("await_robot_stationary", {"settle_sec": 0.5, "timeout_sec": 10.0}))
        plan.append(("set_camera_pose", {"pan_deg": pan, "tilt_deg": tilt, "scan_band": pose["id"],
                                          "require_feedback": True, "wait_timeout_sec": 5.0}))
        if pose is not home:
            plan.append(("capture_observation", {"view_id": pose["id"]}))
    if name == "search_at_place":
        checks = args.get("checks", [{"query": args.get("query"), "provider": args.get("provider")}])
        if not isinstance(checks, list) or not 1 <= len(checks) <= 12:
            raise ValueError("Search requires 1..12 bounded detection checks")
        if args.get("aggregation", "all") not in {"all", "any"}:
            raise ValueError("Invalid detection aggregation")
        for check in checks:
            if not isinstance(check, dict) or not str(check.get("query") or "").strip() or check.get("provider") not in {"yolo11s", "yolo11s_ocr"}:
                raise ValueError("Search requires explicit query and a configured provider")
            plan.append(("detect_known_object", {"query": check["query"], "provider": check["provider"],
                                                  "min_confidence": args.get("min_confidence", 0.5)}))
    return plan, len(poses)


def execute_observation(client, name, args, context):
    try:
        plan, expected = observation_plan(name, args)
    except (KeyError, ValueError, TypeError) as exc:
        return make_result(name, False, codes.FAILED_SCHEMA, str(exc))
    deadline = time.monotonic() + context.max_duration_sec
    evidence_id = f"{context.evidence_id or context.run_id}:obs-{uuid4().hex[:12]}"
    children, observations, artifacts, check_results = [], [], [], []
    camera_time = None
    for index, (skill, payload) in enumerate(plan):
        if client.is_cancelled(context.run_id):
            return make_result(name, False, codes.CANCELLED, "Observation cancelled.",
                               metrics={"children": children, "required_views": expected, "usable_views": len(observations),
                                        "check_results": check_results, "unresolved_requirements": args.get("unresolved_requirements", [])},
                               observations=observations, artifacts=list(dict.fromkeys(artifacts)))
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return make_result(name, False, codes.FAILED_TIMEOUT, "Composite total budget exhausted.",
                               metrics={"children": children, "required_views": expected, "usable_views": len(observations),
                                        "check_results": check_results, "unresolved_requirements": args.get("unresolved_requirements", [])},
                               observations=observations, artifacts=list(dict.fromkeys(artifacts)))
        child_context = replace(context, max_duration_sec=remaining, evidence_id=evidence_id)
        child_context.cancel_check = lambda: client.is_cancelled(context.run_id)
        if skill == "capture_observation" and camera_time is not None:
            payload["min_image_time_monotonic"] = camera_time
        result = client.execute_skill(skill, payload, child_context)
        children.append({"index": index, "child_step_id": f"{evidence_id}:s{index + 1}",
                         "skill": skill, "code": result.code, "success": result.success})
        seen = {item.get("image_path") for item in observations}
        fresh = [item for item in result.observations if item.get("image_path") and item.get("image_path") not in seen]
        if skill == "capture_observation":
            for item in fresh:
                item.setdefault("view_id", payload.get("view_id"))
            if result.success and result.code == codes.OK and not fresh:
                result.success, result.code = False, codes.FAILED_PERCEPTION
                result.message = "Capture returned no unique image evidence for this view."
                children[-1].update(success=False, code=result.code)
        observations.extend(fresh)
        artifacts.extend(result.artifacts)
        if skill == "detect_known_object":
            check_results.append({"query": payload["query"], "provider": payload["provider"],
                                  "success": result.success, "code": result.code, "metrics": dict(result.metrics)})
        if not result.success or result.code not in {codes.OK, codes.DRY_RUN_OK}:
            result.metrics.update(children=children, required_views=expected, usable_views=len(observations),
                                  check_results=check_results, failed_child=skill,
                                  unresolved_requirements=args.get("unresolved_requirements", []),
                                  annotated_images=[img for x in check_results for img in x["metrics"].get("annotated_images", [])])
            result.skill_name = name
            result.observations = observations
            result.artifacts = list(dict.fromkeys(artifacts))
            return result
        if skill == "set_camera_pose":
            camera_time = result.metrics.get("completed_time_monotonic")
    result.metrics.update(children=children, required_views=expected, usable_views=len(observations))
    if check_results:
        found = [x["metrics"].get("goal_satisfied") is True for x in check_results]
        unresolved = args.get("unresolved_requirements", [])
        satisfied = (any(found) if args.get("aggregation") == "any" else all(found)) and not unresolved
        # Do not promote a supported fragment to proof of a complete compound goal.
        result.metrics.update(check_results=check_results, unresolved_requirements=unresolved,
                              query=args.get("query"), aggregation=args.get("aggregation", "all"),
                              inference_completed=True, goal_satisfied=satisfied,
                              outcome="FOUND" if satisfied else "PARTIAL" if unresolved or any(found) else "NOT_FOUND",
                              annotated_images=[img for x in check_results for img in x["metrics"].get("annotated_images", [])],
                              detections=[item for x in check_results for item in x["metrics"].get("detections", [])])
        result.message = f"Perception {result.metrics['outcome']}: {len(check_results)} checks on {len(observations)}/{expected} shared views."
    result.skill_name = name
    result.observations = observations
    result.artifacts = list(dict.fromkeys(artifacts))
    return result

"""Read-only, operation-specific evidence. Unknown never means available."""
import importlib.util
import time


def capability_snapshot(client, domain, args):
    operation = args.get("operation", "")
    evidence = {}
    supported = True
    available = ready = None
    if domain == "navigation":
        nav = client.navigation
        if operation in {"relative_motion", "rotate_relative", "move_relative"}:
            available = True
            pose = nav._current_wheel_odom_pose()
            evidence["wheel_odom"] = pose
            ready = bool(pose) and client.system.robot_mode == "AUTO"
        elif operation in {"", "global_navigation"}:
            server = getattr(nav, "_action_client", None)
            available = server.server_is_ready() if server else None
            evidence["localization_healthy"] = client.system.localization_is_healthy()
            ready = bool(available and evidence["localization_healthy"] and client.system.robot_mode == "AUTO")
        else:
            supported = False
    elif domain == "perception":
        p = client.perception
        query = args.get("query")
        age = time.monotonic() - p._latest_image_time
        evidence.update(image_age_sec=age if p._latest_image_time else None, camera_topic=p.color_topic)
        ready = p._latest_image is not None and 0 <= age <= 3.0
        plan = {}
        if query:
            plan = p._build_detection_plan(query)
            evidence["query_plan"] = plan
            supported = plan.get("type") not in {"abstract", "unsupported"}
        provider = args.get("provider")
        if provider:
            cfg = p.config.get("providers", {}).get(provider)
            supported = supported and bool(cfg) and provider != "stub"
            if cfg and plan.get("type") in {"object", "composite"}:
                weights = p._workspace_path(cfg.get("weights", ""))
                evidence["weights_present"] = weights.is_file()
                evidence["ultralytics_installed"] = importlib.util.find_spec("ultralytics") is not None
                available = False if not all((evidence["weights_present"], evidence["ultralytics_installed"])) else None
            elif cfg and plan.get("type") in {"color", "text"}:
                # HSV and OCR do not need a YOLO checkpoint. Actual OCR loading
                # is checked at execution; absence of evidence is not readiness.
                evidence["backend"] = "opencv_hsv" if plan["type"] == "color" else "ocr"
                available = None
        else:
            available = True if ready else None
    elif domain == "arm":
        if operation in {"grasp", "place", "release", "manipulate_object"}:
            supported, available, ready = False, False, False
            evidence["reason"] = "No verified manipulation policy configured"
        elif operation in {"", "set_arm_pose", "touch", "press", "move_lift"}:
            arm = client.arm
            age = time.monotonic() - getattr(arm, "_state_received_at", 0)
            evidence.update(gateway_age_sec=age if getattr(arm, "_state_received_at", 0) else None,
                            physical_completion_verified=operation == "set_arm_pose")
            available = True if age < 3.0 else None
        else:
            supported = False
    else:
        supported = False
    return {"domain": domain, "operation": operation, "supported": supported,
            "available": available, "ready": ready, "evidence": evidence,
            "observed_at_monotonic": time.monotonic(), "valid_for_sec": 1.0,
            "safety_gate_required_at_execution": True}

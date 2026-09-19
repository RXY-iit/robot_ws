#!/usr/bin/env python3
"""Audit whether a Touch regression bag contains frame-accurate debug evidence."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
from typing import Any

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import String
import yaml


CORE_TOPICS = (
    "/arm_camera/color/image_raw",
    "/arm_camera/color/camera_info",
    "/joint_states",
    "/arm/dashboard_state",
    "/arm/dashboard_command",
    "/arm/hardware_status",
    "/arm/visual_servo_command",
    "/arm/visual_servo_state",
    "/arm/mount_transform",
    "/arm_controller/controller_state",
    "/arm_controller/follow_joint_trajectory/_action/feedback",
    "/arm_controller/follow_joint_trajectory/_action/status",
    "/touch_anything/state",
    "/touch_anything/command",
    "/tf",
    "/tf_static",
    "/rosout",
)
PER_TICK_DEBUG_TOPICS = (
    "/touch_anything/tracker_debug",
    "/touch_anything/control_debug",
)


def percentile(values: list[float], q: float) -> float | None:
    return None if not values else float(np.percentile(np.asarray(values), q))


def metadata_counts(bag: Path) -> dict[str, int]:
    metadata = yaml.safe_load(
        (bag / "metadata.yaml").read_text(encoding="utf-8")
    )["rosbag2_bagfile_information"]
    return {
        str(item["topic_metadata"]["name"]): int(item["message_count"])
        for item in metadata.get("topics_with_message_count", [])
    }


def image_stamp(message: Image) -> float:
    return (
        float(message.header.stamp.sec)
        + float(message.header.stamp.nanosec) * 1.0e-9
    )


def read_selected(bag: Path, topics: list[str]):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while reader.has_next():
        yield reader.read_next()


def audit_bag(bag: Path, *, allow_legacy_debug: bool = False) -> dict[str, Any]:
    bag = bag.resolve()
    counts = metadata_counts(bag)
    missing_core = [topic for topic in CORE_TOPICS if counts.get(topic, 0) <= 0]
    missing_debug = [
        topic for topic in PER_TICK_DEBUG_TOPICS if counts.get(topic, 0) <= 0
    ]
    arm_image_stamps: list[float] = []
    tracker_payloads: list[tuple[float, dict[str, Any]]] = []
    control_payloads: list[tuple[float, dict[str, Any]]] = []
    dashboard_payloads: list[tuple[float, dict[str, Any]]] = []
    visual_servo_payloads: list[tuple[float, dict[str, Any]]] = []
    visual_servo_commands: list[tuple[float, dict[str, Any]]] = []
    selected = [
        "/arm_camera/color/image_raw",
        "/touch_anything/tracker_debug",
        "/touch_anything/control_debug",
        "/arm/dashboard_state",
        "/arm/visual_servo_state",
        "/arm/visual_servo_command",
    ]
    for topic, serialized, record_ns in read_selected(bag, selected):
        if topic == "/arm_camera/color/image_raw":
            arm_image_stamps.append(
                image_stamp(deserialize_message(serialized, Image))
            )
            continue
        try:
            payload = json.loads(deserialize_message(serialized, String).data)
        except (json.JSONDecodeError, TypeError, ValueError):
            continue
        item = (float(record_ns) / 1.0e9, payload)
        if topic == "/touch_anything/tracker_debug":
            tracker_payloads.append(item)
        elif topic == "/touch_anything/control_debug":
            control_payloads.append(item)
        elif topic == "/arm/dashboard_state":
            dashboard_payloads.append(item)
        elif topic == "/arm/visual_servo_state":
            visual_servo_payloads.append(item)
        elif topic == "/arm/visual_servo_command":
            visual_servo_commands.append(item)

    image_gaps = np.diff(arm_image_stamps).tolist()
    image_duration = (
        arm_image_stamps[-1] - arm_image_stamps[0]
        if len(arm_image_stamps) >= 2 else 0.0
    )
    image_rate = (
        (len(arm_image_stamps) - 1) / image_duration
        if image_duration > 0.0 else 0.0
    )

    tracker_results = [item[1].get("result", {}) for item in tracker_payloads]
    tracker_image_stamps = [
        float(item[1]["image_stamp"])
        for item in tracker_payloads
        if item[1].get("image_stamp") is not None
    ]
    tracker_record_stamps = [item[0] for item in tracker_payloads]
    tracker_gaps = np.diff(tracker_image_stamps).tolist()
    tracker_durations = [
        float(item[1]["callback_duration_ms"])
        for item in tracker_payloads
        if item[1].get("callback_duration_ms") is not None
    ]
    tracker_statuses: dict[str, int] = {}
    for result in tracker_results:
        status = str(result.get("status", "UNKNOWN"))
        tracker_statuses[status] = tracker_statuses.get(status, 0) + 1

    control_durations = [
        float(item[1]["callback_duration_ms"])
        for item in control_payloads
        if item[1].get("callback_duration_ms") is not None
    ]
    control_sequences = [
        int(item[1]["sequence"])
        for item in control_payloads if item[1].get("sequence") is not None
    ]
    sequence_gaps = [
        right - left
        for left, right in zip(control_sequences, control_sequences[1:])
        if right - left != 1
    ]
    rolling_ticks = sum(
        bool(item[1].get("rolling_target_control_implemented"))
        for item in control_payloads
    )
    active_updates = sum(
        bool(item[1].get("active_goal_target_updated_this_tick"))
        for item in control_payloads
    )
    maximum_drop_count = max(
        [
            int(item[1].get("queue_dropped_total", 0))
            for item in tracker_payloads + control_payloads
        ],
        default=0,
    )

    motion_windows: dict[str, dict[str, Any]] = {}
    for record_stamp, payload in dashboard_payloads:
        execution = payload.get("execution") or {}
        request_id = str(
            execution.get("owner_request_id")
            or execution.get("last_request_id", "")
        )
        if not request_id.startswith("touch-approach-execute-"):
            continue
        state = str(execution.get("state", ""))
        try:
            event_stamp = float(execution.get("event_ros_sec", record_stamp))
        except (TypeError, ValueError):
            event_stamp = record_stamp
        window = motion_windows.setdefault(request_id, {
            "request_id": request_id,
            "session_id": None,
            "start": None,
            "end": None,
            "terminal": None,
        })
        if execution.get("visual_servo_session_id"):
            window["session_id"] = str(execution["visual_servo_session_id"])
        if state == "EXECUTING" and window["start"] is None:
            window["start"] = event_stamp
        if state in {"SUCCEEDED", "FAILED", "CANCELLED", "BLOCKED"} and window["end"] is None:
            window["end"] = event_stamp
            window["terminal"] = state
    motion_reports = []
    for window in motion_windows.values():
        if window["start"] is None or window["end"] is None:
            continue
        tracker_during = [
            stamp for stamp in tracker_record_stamps
            if float(window["start"]) < stamp <= float(window["end"])
        ]
        if window.get("session_id"):
            control_during = [
                payload for _, payload in control_payloads
                if str(payload.get("rolling_visual_servo_session_id"))
                == str(window["session_id"])
            ]
        else:
            control_during = [
                payload for record_stamp, payload in control_payloads
                if float(window["start"]) < record_stamp <= float(window["end"])
            ]
        updates_during = sum(
            bool(payload.get("active_goal_target_updated_this_tick"))
            for payload in control_during
        )
        if window.get("session_id"):
            server_during = [
                payload for _, payload in visual_servo_payloads
                if str(payload.get("session_id")) == str(window["session_id"])
            ]
        else:
            server_during = [
                payload for record_stamp, payload in visual_servo_payloads
                if float(window["start"]) < record_stamp <= float(window["end"])
            ]
        server_update_count = max(
            [int(payload.get("update_count", 0)) for payload in server_during],
            default=0,
        )
        server_sequences = [
            int(payload.get("sequence_applied", -1)) for payload in server_during
        ]
        server_fault_states = sorted({
            str(payload.get("state")) for payload in server_during
            if str(payload.get("state")) in {"WATCHDOG_HOLD", "FAULT", "REJECTED"}
        })
        stop_reasons = sorted({
            str(payload.get("rolling_visual_servo_stop_reason"))
            for payload in control_during
            if payload.get("rolling_visual_servo_stop_reason")
        })
        unexpected_stop_reasons = [
            reason for reason in stop_reasons
            if reason not in {
                "PRESS_CONFIRMATION_READY",
                "MAX_PRESS_TRAVEL_REACHED",
                "TOUCH_SUCCESS_CONFIRMED",
                "segment duration completed",
            }
        ]
        motion_reports.append({
            **window,
            "tracker_ticks": len(tracker_during),
            "control_ticks": len(control_during),
            "active_goal_target_updates": updates_during,
            "server_update_count": server_update_count,
            "server_sequence_min": min(server_sequences, default=None),
            "server_sequence_max": max(server_sequences, default=None),
            "server_fault_states": server_fault_states,
            "stop_reasons": stop_reasons,
            "unexpected_stop_reasons": unexpected_stop_reasons,
            "continuous_tracking_observed": len(tracker_during) >= 3,
            "realtime_correction_observed": bool(
                updates_during >= 3
                and server_update_count >= 3
                and max(server_sequences, default=-1) >= 3
                and not server_fault_states
            ),
            "completed_without_control_failure": not unexpected_stop_reasons,
        })

    debug_complete = not missing_debug and maximum_drop_count == 0
    rolling_commands = [
        payload for _, payload in visual_servo_commands
        if str(payload.get("op")) in {"START", "UPDATE"}
    ]
    horizon_commands = [
        payload for payload in rolling_commands
        if isinstance(payload.get("action_horizon"), list)
    ]
    valid_horizon_commands = 0
    for payload in horizon_commands:
        horizon = payload["action_horizon"]
        terminal = payload.get("positions")
        if (
            len(horizon) == 10
            and abs(float(payload.get("horizon_step_period_sec", 0.0)) - 0.05)
            <= 1.0e-9
            and isinstance(terminal, dict)
            and isinstance(horizon[-1], dict)
            and set(horizon[-1]) == set(terminal)
            and all(
                abs(float(horizon[-1][name]) - float(terminal[name])) <= 1.0e-6
                for name in terminal
            )
        ):
            valid_horizon_commands += 1
    horizon_indices = sorted({
        int(payload.get("action_horizon_index", -1))
        for _, payload in visual_servo_payloads
        if int(payload.get("action_horizon_index", -1)) >= 0
    })
    horizon_replans = max(
        (
            int(payload.get("action_horizon_replan_count", 0))
            for _, payload in visual_servo_payloads
        ),
        default=0,
    )
    action_horizon_observed = bool(
        len(horizon_commands) >= 3
        and valid_horizon_commands == len(rolling_commands)
        and horizon_replans >= len(horizon_commands)
        and len(horizon_indices) >= 2
    )
    report = {
        "schema_version": 1,
        "bag": str(bag),
        "topic_counts": counts,
        "missing_core_topics": missing_core,
        "missing_per_tick_debug_topics": missing_debug,
        "arm_camera": {
            "frames": len(arm_image_stamps),
            "header_stamps_monotonic": all(
                right > left
                for left, right in zip(arm_image_stamps, arm_image_stamps[1:])
            ),
            "rate_hz": image_rate,
            "maximum_gap_sec": max(image_gaps, default=None),
        },
        "online_tracker": {
            "ticks": len(tracker_payloads),
            "statuses": tracker_statuses,
            "maximum_image_gap_sec": max(tracker_gaps, default=None),
            "callback_ms_median": (
                statistics.median(tracker_durations) if tracker_durations else None
            ),
            "callback_ms_p95": percentile(tracker_durations, 95.0),
        },
        "online_control": {
            "ticks": len(control_payloads),
            "sequence_discontinuities": sequence_gaps,
            "callback_ms_median": (
                statistics.median(control_durations) if control_durations else None
            ),
            "callback_ms_p95": percentile(control_durations, 95.0),
            "rolling_target_ticks": rolling_ticks,
            "active_goal_target_update_ticks": active_updates,
        },
        "debug_queue_maximum_dropped_total": maximum_drop_count,
        "motion_windows": motion_reports,
        "action_horizon": {
            "rolling_commands": len(rolling_commands),
            "commands_with_horizon": len(horizon_commands),
            "valid_10_step_commands": valid_horizon_commands,
            "server_replans_observed": horizon_replans,
            "server_indices_observed": horizon_indices,
        },
        "gates": {
            "recording_core_complete": not missing_core,
            "per_tick_debug_complete": debug_complete,
            "motion_tracking_observed": bool(
                motion_reports
                and all(item["continuous_tracking_observed"] for item in motion_reports)
            ),
            "realtime_motion_correction_observed": bool(
                motion_reports
                and all(item["realtime_correction_observed"] for item in motion_reports)
            ),
            "delay_aware_action_horizon_observed": action_horizon_observed,
            "next_real_motion_test_ready": bool(
                not missing_core
                and debug_complete
                and action_horizon_observed
                and motion_reports
                and all(
                    item["continuous_tracking_observed"]
                    and item["realtime_correction_observed"]
                    and item["completed_without_control_failure"]
                    for item in motion_reports
                )
            ),
        },
        "legacy_debug_allowed": allow_legacy_debug,
    }
    report["passed_recording_audit"] = bool(
        not missing_core and (debug_complete or allow_legacy_debug)
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--json-output", type=Path)
    parser.add_argument("--allow-legacy-debug", action="store_true")
    args = parser.parse_args()
    report = audit_bag(args.bag, allow_legacy_debug=args.allow_legacy_debug)
    output = json.dumps(report, indent=2, ensure_ascii=False)
    print(output)
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(output + "\n", encoding="utf-8")
    return 0 if report["passed_recording_audit"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

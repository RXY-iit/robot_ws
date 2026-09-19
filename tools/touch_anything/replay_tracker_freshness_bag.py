#!/usr/bin/env python3
"""Replay rolling tracker freshness decisions from per-tick rosbag telemetry."""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


WORKSPACE = Path(__file__).resolve().parents[2]
GEOMETRY_PATH = WORKSPACE / "src/touch_anything/scripts/approach_geometry.py"
SPEC = importlib.util.spec_from_file_location("approach_geometry", GEOMETRY_PATH)
GEOMETRY = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(GEOMETRY)

HEALTHY = {"INITIALIZED", "TRACKING", "RECOVERED"}


def read_debug(bag: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag.resolve()), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[
        "/touch_anything/tracker_debug",
        "/touch_anything/control_debug",
    ]))
    trackers: list[dict[str, Any]] = []
    controls: list[dict[str, Any]] = []
    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        try:
            payload = json.loads(deserialize_message(serialized, String).data)
        except (json.JSONDecodeError, TypeError, ValueError):
            continue
        payload["_record_ros_sec"] = float(record_ns) * 1.0e-9
        if topic.endswith("tracker_debug"):
            trackers.append(payload)
        else:
            controls.append(payload)
    return trackers, controls


def replay(
    bag: Path, *, maximum_completed_age_sec: float,
    maximum_inflight_processing_sec: float,
) -> dict[str, Any]:
    trackers, controls = read_debug(bag)
    intervals = []
    completions = []
    for payload in trackers:
        start = payload.get("frame_claimed_monotonic_sec")
        if start is None:
            start = payload.get("callback_started_monotonic_sec")
        end = payload.get("callback_completed_monotonic_sec")
        if start is None or end is None:
            continue
        result = payload.get("result") or {}
        item = {
            "start": float(start),
            "end": float(end),
            "status": str(result.get("status", "UNKNOWN")),
            "input_age_at_start_sec": float(
                payload.get("input_receive_age_ms_at_start") or 0.0
            ) * 1.0e-3,
            "image_stamp": payload.get("image_stamp"),
        }
        intervals.append(item)
        if item["status"] in HEALTHY:
            completions.append(item)
    intervals.sort(key=lambda item: item["start"])
    completions.sort(key=lambda item: item["end"])

    session_controls = [
        payload for payload in controls
        if str(payload.get("rolling_visual_servo_session_id") or "").startswith(
            "touch-visual-servo-"
        )
    ]
    decisions = []
    for payload in session_controls:
        control_time = float(payload["callback_started_monotonic_sec"])
        completed = [item for item in completions if item["end"] <= control_time]
        latest = completed[-1] if completed else None
        completed_age = (
            math.inf if latest is None else max(0.0, control_time - latest["end"])
        )
        prior_status = (
            str((payload.get("tracker_result") or {}).get("status", "UNKNOWN"))
        )
        inflight_candidates = [
            item for item in intervals
            if item["start"] <= control_time < item["end"]
        ]
        inflight = inflight_candidates[-1] if inflight_candidates else None
        processing_age = (
            None if inflight is None
            else max(0.0, control_time - inflight["start"])
        )
        input_age = (
            None if inflight is None
            else inflight["input_age_at_start_sec"] + float(processing_age)
        )
        legacy_decision = (
            "FRESH"
            if prior_status in HEALTHY
            and completed_age <= maximum_completed_age_sec
            else "STOP"
        )
        new_gate = GEOMETRY.rolling_tracker_freshness_gate(
            status=prior_status,
            completed_age_sec=completed_age,
            maximum_completed_age_sec=maximum_completed_age_sec,
            inflight_processing_age_sec=processing_age,
            inflight_input_age_sec=input_age,
            maximum_inflight_processing_sec=maximum_inflight_processing_sec,
        )
        decisions.append({
            "control_sequence": payload.get("sequence"),
            "rolling_sequence": payload.get("rolling_visual_servo_sequence"),
            "control_time_monotonic_sec": control_time,
            "recorded_update": bool(
                payload.get("active_goal_target_updated_this_tick")
            ),
            "recorded_stop_reason": payload.get(
                "rolling_visual_servo_stop_reason"
            ),
            "legacy_decision": legacy_decision,
            "new_decision": new_gate["decision"],
            "completed_age_sec": (
                None if not math.isfinite(completed_age) else completed_age
            ),
            "inflight_processing_age_sec": processing_age,
            "inflight_input_age_sec": input_age,
            "inflight_completion_after_control_sec": (
                None if inflight is None
                else inflight["end"] - control_time
            ),
            "inflight_eventual_status": (
                None if inflight is None else inflight["status"]
            ),
        })

    false_stops = [
        item for item in decisions
        if item["legacy_decision"] == "STOP"
        and item["new_decision"] == "WAIT_INFLIGHT"
        and str(item.get("recorded_stop_reason") or "").startswith(
            "tracking unavailable during motion"
        )
    ]
    recovered = [
        item for item in false_stops
        if item["inflight_eventual_status"] in HEALTHY
        and item["inflight_completion_after_control_sec"] is not None
        and 0.0 <= item["inflight_completion_after_control_sec"]
        <= maximum_inflight_processing_sec
    ]
    passed = bool(false_stops and len(recovered) == len(false_stops))
    return {
        "schema_version": 1,
        "bag": str(bag.resolve()),
        "configuration": {
            "maximum_completed_age_sec": maximum_completed_age_sec,
            "maximum_inflight_processing_sec": maximum_inflight_processing_sec,
        },
        "tracker_intervals": len(intervals),
        "rolling_control_ticks": len(session_controls),
        "legacy_false_stop_ticks": len(false_stops),
        "bounded_inflight_recoveries": len(recovered),
        "false_stop_eliminated": passed,
        "offline_tracker_freshness_gate_passed": passed,
        "decisions": decisions,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--maximum-completed-age-sec", type=float, default=0.20)
    parser.add_argument("--maximum-inflight-processing-sec", type=float, default=0.15)
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()
    report = replay(
        args.bag,
        maximum_completed_age_sec=args.maximum_completed_age_sec,
        maximum_inflight_processing_sec=args.maximum_inflight_processing_sec,
    )
    output = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(output + "\n", encoding="utf-8")
    print(output)
    return 0 if report["offline_tracker_freshness_gate_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

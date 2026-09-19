#!/usr/bin/env python3
"""Measure requested-vs-feedback Cartesian motion in a Touch rosbag."""
from __future__ import annotations

import argparse
import bisect
import importlib.util
import json
from pathlib import Path
import sys

from geometry_msgs.msg import TransformStamped
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import String


WORKSPACE = Path(__file__).resolve().parents[2]
KINEMATICS_PATH = (
    WORKSPACE / "src/touch_anything/scripts/touch_kinematics.py"
)
SPEC = importlib.util.spec_from_file_location("touch_kinematics", KINEMATICS_PATH)
KINEMATICS = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(KINEMATICS)
REACH_JOINT_NAMES = tuple(KINEMATICS.JOINT_NAMES)


def quaternion_rotation(transform: TransformStamped) -> np.ndarray:
    q = transform.transform.rotation
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = np.linalg.norm([x, y, z, w])
    if norm <= 1.0e-12:
        raise ValueError("arm mount quaternion is invalid")
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def tip_base(positions: dict[str, float], rotation: np.ndarray) -> np.ndarray:
    values = [float(positions[name]) for name in REACH_JOINT_NAMES]
    return rotation @ np.asarray(
        KINEMATICS.tip_transform(values)[:3, 3], dtype=np.float64
    )


def audit(bag: Path) -> dict:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag.resolve()), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = [
        "/arm/dashboard_command", "/arm/visual_servo_state",
        "/joint_states", "/arm/mount_transform", "/touch_anything/control_debug",
    ]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    commands: list[tuple[float, dict]] = []
    servo_states: list[tuple[float, dict]] = []
    joint_states: list[tuple[float, dict[str, float]]] = []
    mount: TransformStamped | None = None
    control_ticks: list[tuple[float, dict]] = []
    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        stamp = float(record_ns) * 1.0e-9
        if topic == "/joint_states":
            message = deserialize_message(serialized, JointState)
            values = {
                str(name): float(value)
                for name, value in zip(message.name, message.position)
            }
            if all(name in values for name in REACH_JOINT_NAMES):
                joint_states.append((stamp, values))
        elif topic == "/arm/mount_transform":
            mount = deserialize_message(serialized, TransformStamped)
        else:
            try:
                payload = json.loads(deserialize_message(serialized, String).data)
            except (json.JSONDecodeError, TypeError):
                continue
            if topic == "/arm/dashboard_command":
                if str(payload.get("command")) in {
                    "visual_servo_start", "visual_servo_update", "visual_servo_stop"
                }:
                    commands.append((stamp, payload))
            elif topic == "/touch_anything/control_debug":
                control_ticks.append((stamp, payload))
            else:
                servo_states.append((stamp, payload))
    if mount is None or not joint_states:
        raise RuntimeError("bag lacks mount transform or joint feedback")
    sessions = sorted({
        str(payload.get("session_id")) for _, payload in commands
        if payload.get("session_id")
    })
    reports = []
    state_times = [item[0] for item in joint_states]
    rotation = quaternion_rotation(mount)
    for session_id in sessions:
        session_commands = [
            (stamp, payload) for stamp, payload in commands
            if str(payload.get("session_id")) == session_id
            and str(payload.get("command")) in {
                "visual_servo_start", "visual_servo_update"
            }
        ]
        if not session_commands:
            continue
        terminal_times = [
            stamp for stamp, payload in servo_states
            if str(payload.get("session_id")) == session_id
            and str(payload.get("state")) in {
                "HOLDING", "WATCHDOG_HOLD", "FAULT", "REJECTED"
            }
        ]
        start_time = session_commands[0][0]
        end_time = terminal_times[0] if terminal_times else session_commands[-1][0]
        stop_times = [
            stamp for stamp, payload in commands
            if str(payload.get("session_id")) == session_id
            and str(payload.get("command")) == "visual_servo_stop"
        ]

        def nearest_actual(stamp: float) -> tuple[float, dict[str, float]]:
            index = bisect.bisect_left(state_times, stamp)
            candidates = [max(0, index - 1), min(len(state_times) - 1, index)]
            chosen = min(candidates, key=lambda item: abs(state_times[item] - stamp))
            return joint_states[chosen]

        def actual_at_or_before(stamp: float) -> tuple[float, dict[str, float]]:
            index = max(0, bisect.bisect_right(state_times, stamp) - 1)
            return joint_states[index]

        def actual_at_or_after(stamp: float) -> tuple[float, dict[str, float]]:
            index = min(len(state_times) - 1, bisect.bisect_left(state_times, stamp))
            return joint_states[index]

        _, start_positions = nearest_actual(start_time)
        _, end_positions = actual_at_or_after(end_time)
        start_tip = tip_base(start_positions, rotation)
        end_tip = tip_base(end_positions, rotation)
        stop_tip = None
        stop_feedback_skew = None
        if stop_times:
            stop_feedback_stamp, stop_positions = actual_at_or_before(stop_times[0])
            stop_tip = tip_base(stop_positions, rotation)
            stop_feedback_skew = stop_feedback_stamp - stop_times[0]
        samples = []
        for stamp, payload in session_commands:
            positions = payload.get("positions")
            if not isinstance(positions, dict):
                continue
            actual_stamp, actual_positions = nearest_actual(stamp)
            requested_tip = tip_base(positions, rotation)
            actual_tip = tip_base(actual_positions, rotation)
            samples.append({
                "time_from_start_sec": stamp - start_time,
                "feedback_skew_sec": actual_stamp - stamp,
                "requested_tip_base_m": requested_tip.tolist(),
                "actual_tip_base_m": actual_tip.tolist(),
                "instantaneous_error_m": (requested_tip - actual_tip).tolist(),
                "sequence": int(payload.get("sequence", 0)),
            })
        z_errors = [sample["instantaneous_error_m"][2] for sample in samples]
        requested_last = np.asarray(samples[-1]["requested_tip_base_m"])
        tracker_centroids = [
            payload.get("tracker_result", {}).get("centroid_uv")
            for stamp, payload in control_ticks
            if start_time <= stamp <= end_time
            and isinstance(payload.get("tracker_result"), dict)
            and payload.get("tracker_result", {}).get("centroid_uv") is not None
        ]
        reports.append({
            "session_id": session_id,
            "duration_sec": end_time - start_time,
            "command_samples": len(samples),
            "start_tip_base_m": start_tip.tolist(),
            "end_tip_base_m": end_tip.tolist(),
            "actual_delta_base_m": (end_tip - start_tip).tolist(),
            "last_requested_delta_from_start_m": (requested_last - start_tip).tolist(),
            "terminal_error_to_last_request_m": (requested_last - end_tip).tolist(),
            "stop_command_time_from_start_sec": (
                None if not stop_times else stop_times[0] - start_time
            ),
            "stop_feedback_skew_sec": stop_feedback_skew,
            "stop_tip_base_m": None if stop_tip is None else stop_tip.tolist(),
            "stop_to_terminal_actual_delta_base_m": (
                None if stop_tip is None else (end_tip - stop_tip).tolist()
            ),
            "stop_to_terminal_actual_distance_m": (
                None if stop_tip is None
                else float(np.linalg.norm(end_tip - stop_tip))
            ),
            "instantaneous_z_error_min_m": min(z_errors),
            "instantaneous_z_error_max_m": max(z_errors),
            "instantaneous_z_error_last_m": z_errors[-1],
            "tracker_centroid_first_uv": (
                tracker_centroids[0] if tracker_centroids else None
            ),
            "tracker_centroid_last_uv": (
                tracker_centroids[-1] if tracker_centroids else None
            ),
            "samples": samples,
        })
    return {"schema_version": 1, "bag": str(bag.resolve()), "sessions": reports}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--json-output", type=Path)
    parser.add_argument("--include-samples", action="store_true")
    args = parser.parse_args()
    report = audit(args.bag)
    if not args.include_samples:
        for session in report["sessions"]:
            session.pop("samples", None)
    output = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    if args.json_output is not None:
        args.json_output.write_text(output + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

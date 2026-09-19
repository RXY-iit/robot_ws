#!/usr/bin/env python3
"""Counterfactual replay of the 10-step rolling action-chunk policy.

This tool is read-only with respect to ROS.  It takes the real joint feedback,
rolling Dashboard targets, and tracker ages recorded in a regression bag and
reconstructs the action horizon that the new runtime would have issued at each
recorded command time.
"""
from __future__ import annotations

import argparse
import bisect
import importlib.util
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
GEOMETRY_PATH = WORKSPACE / "src/touch_anything/scripts/approach_geometry.py"
SPEC = importlib.util.spec_from_file_location("approach_geometry", GEOMETRY_PATH)
GEOMETRY = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(GEOMETRY)


def _read_bag(
    bag: Path, joint_names: set[str]
) -> tuple[list[tuple[float, dict]], list[dict], list[dict]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag.resolve()), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[
        "/joint_states",
        "/arm/dashboard_command",
        "/touch_anything/control_debug",
    ]))
    joints: list[tuple[float, dict]] = []
    commands: list[dict] = []
    controls: list[dict] = []
    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        stamp = float(record_ns) * 1.0e-9
        if topic == "/joint_states":
            message = deserialize_message(serialized, JointState)
            positions = {
                str(name): float(value)
                for name, value in zip(message.name, message.position)
            }
            velocities = {
                str(name): float(value)
                for name, value in zip(message.name, message.velocity)
                if math.isfinite(float(value))
            }
            if joint_names.issubset(positions):
                joints.append((stamp, {
                    "positions": positions,
                    "velocities": velocities,
                }))
            continue
        try:
            payload = json.loads(deserialize_message(serialized, String).data)
        except (json.JSONDecodeError, TypeError, ValueError):
            continue
        payload["_record_ros_sec"] = stamp
        if topic == "/arm/dashboard_command":
            if str(payload.get("command")) in {
                "visual_servo_start", "visual_servo_update"
            }:
                commands.append(payload)
        else:
            controls.append(payload)
    return joints, commands, controls


def replay(
    bag: Path,
    *,
    steps: int = 10,
    step_period_sec: float = 0.05,
    execution_lead_sec: float = 0.08,
    maximum_prediction_lead_sec: float = 0.25,
    maximum_prediction_velocity_rad_s: float = 1.0,
) -> dict[str, Any]:
    config = yaml.safe_load((
        WORKSPACE / "src/so101_arm_control/config/arm_real_dashboard_test.yaml"
    ).read_text(encoding="utf-8"))
    bounds = {
        str(item["name"]): (float(item["lower"]), float(item["upper"]))
        for item in config["joints"]
    }
    watchdog_sec = float(config["visual_servo"]["watchdog_sec"])
    joints, commands, controls = _read_bag(bag, set(bounds))
    if not joints or not commands:
        raise RuntimeError("bag lacks rolling commands or joint feedback")
    joint_times = [item[0] for item in joints]
    control_times = [float(item["_record_ros_sec"]) for item in controls]

    def nearest_index(times: list[float], stamp: float) -> int:
        index = bisect.bisect_left(times, stamp)
        choices = [max(0, index - 1), min(len(times) - 1, index)]
        return min(choices, key=lambda candidate: abs(times[candidate] - stamp))

    samples: list[dict[str, Any]] = []
    for command in commands:
        target_raw = command.get("positions")
        if not isinstance(target_raw, dict) or set(target_raw) != set(bounds):
            continue
        stamp = float(command["_record_ros_sec"])
        joint_index = nearest_index(joint_times, stamp)
        feedback_stamp, feedback = joints[joint_index]
        current = {}
        for name, (lower, upper) in bounds.items():
            measured = float(feedback["positions"][name])
            current[name] = float(np.clip(measured, lower, upper))
        velocities = {
            name: float(feedback["velocities"].get(name, 0.0))
            for name in bounds
        }
        if not any(abs(value) > 1.0e-6 for value in velocities.values()):
            before = max(0, joint_index - 1)
            after = min(len(joints) - 1, joint_index + 1)
            dt = joints[after][0] - joints[before][0]
            if dt > 1.0e-4:
                velocities = {
                    name: (
                        float(joints[after][1]["positions"][name])
                        - float(joints[before][1]["positions"][name])
                    ) / dt
                    for name in bounds
                }
        tracker_age_sec = 0.0
        if controls:
            control = controls[nearest_index(control_times, stamp)]
            raw_age_ms = control.get("tracker_age_ms")
            if raw_age_ms is not None and math.isfinite(float(raw_age_ms)):
                tracker_age_sec = max(0.0, float(raw_age_ms) * 1.0e-3)
        prediction_lead = min(
            maximum_prediction_lead_sec,
            tracker_age_sec + execution_lead_sec,
        )
        target = {name: float(target_raw[name]) for name in bounds}
        plan = GEOMETRY.receding_joint_action_horizon(
            current_positions=current,
            target_positions=target,
            current_velocities=velocities,
            joint_bounds=bounds,
            steps=steps,
            step_period_sec=step_period_sec,
            prediction_lead_sec=prediction_lead,
            maximum_prediction_velocity_rad_s=maximum_prediction_velocity_rad_s,
        )
        points = plan["positions"]
        terminal_exact = all(
            abs(float(points[-1][name]) - target[name]) <= 1.0e-9
            for name in bounds
        )
        within_bounds = all(
            bounds[name][0] <= float(point[name]) <= bounds[name][1]
            for point in points for name in bounds
        )
        monotonic_to_target = all(
            all(
                abs(float(points[index + 1][name]) - target[name])
                <= abs(float(points[index][name]) - target[name]) + 1.0e-12
                for index in range(len(points) - 1)
            )
            for name in bounds
        )
        predicted_shift = max(
            abs(float(plan["predicted_start_positions"][name]) - current[name])
            for name in bounds
        )
        samples.append({
            "sequence": int(command.get("sequence", 0)),
            "command": str(command.get("command")),
            "record_ros_sec": stamp,
            "feedback_skew_sec": feedback_stamp - stamp,
            "tracker_age_sec": tracker_age_sec,
            "prediction_lead_sec": prediction_lead,
            "maximum_predicted_start_shift_rad": predicted_shift,
            "maximum_abs_feedback_velocity_rad_s": max(
                abs(value) for value in velocities.values()
            ),
            "horizon_steps": len(points),
            "horizon_duration_sec": plan["duration_sec"],
            "terminal_exact": terminal_exact,
            "within_bounds": within_bounds,
            "monotonic_to_target": monotonic_to_target,
            "predicted_start_positions": plan["predicted_start_positions"],
            "terminal_positions": plan["terminal_positions"],
        })

    command_times = [float(item["record_ros_sec"]) for item in samples]
    replan_intervals = [
        later - earlier
        for earlier, later in zip(command_times, command_times[1:])
        if later >= earlier
    ]
    max_replan_interval = max(replan_intervals) if replan_intervals else math.inf
    horizon_duration = steps * step_period_sec
    all_valid = bool(samples) and all(
        sample["horizon_steps"] == steps
        and sample["terminal_exact"]
        and sample["within_bounds"]
        and sample["monotonic_to_target"]
        for sample in samples
    )
    prediction_observed = any(
        sample["maximum_abs_feedback_velocity_rad_s"] > 1.0e-4
        and sample["maximum_predicted_start_shift_rad"] > 1.0e-6
        for sample in samples
    )
    cadence_safe = bool(replan_intervals) and max_replan_interval < watchdog_sec
    watchdog_preempts_abandoned_tail = watchdog_sec < horizon_duration
    passed = bool(
        len(samples) >= 3
        and all_valid
        and prediction_observed
        and cadence_safe
        and watchdog_preempts_abandoned_tail
    )
    return {
        "schema_version": 1,
        "bag": str(bag.resolve()),
        "policy": "DELAY_AWARE_RECEDING_10_STEP_ACTION_HORIZON",
        "configuration": {
            "steps": steps,
            "step_period_sec": step_period_sec,
            "horizon_duration_sec": horizon_duration,
            "execution_lead_sec": execution_lead_sec,
            "maximum_prediction_lead_sec": maximum_prediction_lead_sec,
            "maximum_prediction_velocity_rad_s": (
                maximum_prediction_velocity_rad_s
            ),
            "actuator_watchdog_sec": watchdog_sec,
        },
        "commands_replayed": len(samples),
        "maximum_recorded_replan_interval_sec": (
            None if not math.isfinite(max_replan_interval)
            else max_replan_interval
        ),
        "all_horizons_valid": all_valid,
        "latency_prediction_observed": prediction_observed,
        "recorded_replan_cadence_below_watchdog": cadence_safe,
        "watchdog_preempts_abandoned_horizon": watchdog_preempts_abandoned_tail,
        "offline_receding_horizon_gate_passed": passed,
        "scope": (
            "Counterfactual scheduling/geometry validation using recorded real "
            "feedback; it does not claim new real actuator response."
        ),
        "samples": samples,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()
    report = replay(args.bag)
    output = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(output + "\n", encoding="utf-8")
    print(output)
    return 0 if report["offline_receding_horizon_gate_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

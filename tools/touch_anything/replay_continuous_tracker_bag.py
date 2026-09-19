#!/usr/bin/env python3
"""Read-only full-rate replay of the wrist-camera tracker from a rosbag2 bag.

The replay starts from a saved, human-confirmed Arm Camera RGB/SAM-mask pair,
then feeds every later ``/arm_camera/color/image_raw`` frame through the real
ContinuousTargetTracker.  It publishes nothing and cannot command hardware.
"""
from __future__ import annotations

import argparse
import bisect
import importlib.util
import json
import math
from pathlib import Path
import statistics
import sys
import time
from typing import Any

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_EVIDENCE_ROOT = WORKSPACE / "debug-output" / "touch-anything"
ARM_TOPIC = "/arm_camera/color/image_raw"
ARM_STATE_TOPIC = "/arm/dashboard_state"
TF_TOPICS = ("/tf", "/tf_static")


def load_tracker_module():
    path = WORKSPACE / "src" / "touch_anything" / "scripts" / "target_tracker.py"
    spec = importlib.util.spec_from_file_location("bag_target_tracker", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import tracker from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_geometry_module():
    path = WORKSPACE / "src" / "touch_anything" / "scripts" / "approach_geometry.py"
    spec = importlib.util.spec_from_file_location("bag_approach_geometry", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import approach geometry from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_kinematics_module():
    path = WORKSPACE / "src" / "touch_anything" / "scripts" / "touch_kinematics.py"
    spec = importlib.util.spec_from_file_location("bag_touch_kinematics", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import touch kinematics from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def bag_time_range(bag: Path) -> tuple[float, float]:
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))
    info = metadata["rosbag2_bagfile_information"]
    start = float(info["starting_time"]["nanoseconds_since_epoch"]) / 1.0e9
    end = start + float(info["duration"]["nanoseconds"]) / 1.0e9
    return start, end


def find_initial_record(bag: Path, explicit: Path | None) -> Path:
    if explicit is not None:
        return explicit.resolve()
    start, end = bag_time_range(bag)
    candidates: list[tuple[float, Path]] = []
    for path in DEFAULT_EVIDENCE_ROOT.glob("observation_*/arm_observation_*.json"):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            stamp = float(data["image_stamp"])
            target = data.get("target", {})
            if (
                start <= stamp <= end
                and bool(target.get("operator_confirmed_same_object"))
                and isinstance(data.get("files"), dict)
            ):
                candidates.append((stamp, path))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            continue
    if not candidates:
        raise RuntimeError("no human-confirmed Arm Camera evidence lies inside bag")
    # The earliest confirmation preserves the longest continuous test interval.
    return min(candidates, key=lambda item: item[0])[1]


def load_initial(record_path: Path) -> tuple[dict[str, Any], np.ndarray, np.ndarray]:
    record = json.loads(record_path.read_text(encoding="utf-8"))
    files = record["files"]
    bgr = cv2.imread(str(record_path.parent / files["rgb"]), cv2.IMREAD_COLOR)
    mask = cv2.imread(str(record_path.parent / files["mask"]), cv2.IMREAD_GRAYSCALE)
    if bgr is None or mask is None:
        raise RuntimeError(f"cannot read RGB/mask referenced by {record_path}")
    return record, cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), (mask > 0).astype(np.uint8)


def image_stamp(message: Image) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1.0e9


def image_rgb(message: Image) -> np.ndarray:
    if message.encoding not in {"rgb8", "bgr8"}:
        raise ValueError(f"unsupported Arm Camera encoding: {message.encoding}")
    row = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    image = row[:, : message.width * 3].reshape(message.height, message.width, 3).copy()
    if message.encoding == "bgr8":
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image


def percentile(values: list[float], q: float) -> float | None:
    return None if not values else float(np.percentile(np.asarray(values), q))


def transform_stamp(transform) -> float:
    stamp = transform.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1.0e9


def transform_matrix(transform) -> np.ndarray:
    translation = transform.transform.translation
    quaternion = transform.transform.rotation
    x, y, z, w = map(float, (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        raise ValueError("TF quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    rotation = np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def resolve_transform(
    edges: dict[tuple[str, str], np.ndarray], source: str, target: str
) -> np.ndarray | None:
    """Return source_T_target from the latest directed TF edge snapshot."""
    queue: list[tuple[str, np.ndarray]] = [(source, np.eye(4, dtype=np.float64))]
    visited = {source}
    while queue:
        frame, source_from_frame = queue.pop(0)
        if frame == target:
            return source_from_frame
        for (parent, child), parent_from_child in edges.items():
            if frame == parent and child not in visited:
                visited.add(child)
                queue.append((child, source_from_frame @ parent_from_child))
            elif frame == child and parent not in visited:
                visited.add(parent)
                queue.append((parent, source_from_frame @ np.linalg.inv(parent_from_child)))
    return None


def load_arm_camera_tf_trace(bag: Path) -> list[tuple[float, np.ndarray]]:
    """Reconstruct recorded base_link<-arm-camera rotations at Arm joint rate."""
    edges: dict[tuple[str, str], np.ndarray] = {}
    static_reader = rosbag2_py.SequentialReader()
    static_reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    static_reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf_static"]))
    while static_reader.has_next():
        topic, serialized, _ = static_reader.read_next()
        if topic != "/tf_static":
            continue
        for transform in deserialize_message(serialized, TFMessage).transforms:
            edges[(str(transform.header.frame_id), str(transform.child_frame_id))] = (
                transform_matrix(transform)
            )

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf"]))
    trace: list[tuple[float, np.ndarray]] = []
    arm_dynamic_children = {
        "arm/shoulder_link", "arm/upper_arm_link", "arm/lower_arm_link",
        "arm/wrist_link", "arm/gripper_link",
    }
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic != "/tf":
            continue
        message = deserialize_message(serialized, TFMessage)
        sample_stamp = None
        arm_joint_update = False
        for transform in message.transforms:
            parent = str(transform.header.frame_id)
            child = str(transform.child_frame_id)
            edges[(parent, child)] = transform_matrix(transform)
            stamp = transform_stamp(transform)
            sample_stamp = stamp if sample_stamp is None else max(sample_stamp, stamp)
            arm_joint_update = arm_joint_update or child in arm_dynamic_children
        if not arm_joint_update or sample_stamp is None:
            continue
        base_from_camera = resolve_transform(
            edges, "base_link", "arm_camera_color_optical_frame"
        )
        if base_from_camera is not None:
            trace.append((sample_stamp, base_from_camera[:3, :3].copy()))
    # The same robot_state_publisher batch can contain several Arm edges with
    # one stamp. Retain one fully composed snapshot per stamp.
    unique: dict[float, np.ndarray] = {stamp: rotation for stamp, rotation in trace}
    return sorted(unique.items(), key=lambda item: item[0])


def load_approach_windows(bag: Path) -> list[dict[str, Any]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[ARM_STATE_TOPIC]))
    windows: dict[str, dict[str, Any]] = {}
    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        if topic != ARM_STATE_TOPIC:
            continue
        payload = json.loads(deserialize_message(serialized, String).data)
        execution = payload.get("execution", {})
        request_id = str(execution.get("last_request_id", ""))
        if not request_id.startswith("touch-approach-execute-"):
            continue
        state = str(execution.get("state", ""))
        event_stamp = execution.get("event_ros_sec")
        try:
            stamp = float(event_stamp)
        except (TypeError, ValueError):
            stamp = float(record_ns) / 1.0e9
        window = windows.setdefault(request_id, {
            "request_id": request_id,
            "start_stamp": None,
            "end_stamp": None,
            "terminal_state": None,
        })
        if state == "EXECUTING" and window["start_stamp"] is None:
            window["start_stamp"] = stamp
        if (
            state in {"SUCCEEDED", "FAILED", "CANCELLED", "BLOCKED"}
            and window["end_stamp"] is None
        ):
            window["end_stamp"] = stamp
            window["terminal_state"] = state
    return [
        window for window in windows.values()
        if window["start_stamp"] is not None and window["end_stamp"] is not None
    ]


def simulate_shadow_control(
    trace: list[dict[str, Any]],
    record: dict[str, Any],
    tf_trace: list[tuple[float, np.ndarray]],
    control_hz: float,
) -> dict[str, Any]:
    """Run the real image-Jacobian control calculation on a fixed 10 Hz clock."""
    if not trace or control_hz <= 0.0:
        return {"ready": False, "reason": "tracker trace or control rate is empty"}
    geometry = load_geometry_module()
    camera_info = record.get("camera_info") or {}
    target = record.get("target") or {}
    k = np.asarray(camera_info.get("k", []), dtype=np.float64)
    distortion = np.asarray(camera_info.get("d", []), dtype=np.float64)
    range_scale = float(target.get("tracking_range_m", 0.0))
    if k.size != 9 or not math.isfinite(range_scale) or range_scale <= 0.02:
        return {"ready": False, "reason": "initial camera geometry/range is incomplete"}
    width = 640
    height = 480
    image_center = ((width - 1) * 0.5, (height - 1) * 0.5)
    period = 1.0 / float(control_hz)
    observations = [
        item for item in trace
        if item.get("status") in {"TRACKING", "RECOVERED"}
        and item.get("centroid_uv") is not None
    ]
    ready_times = [
        float(item["stamp"]) + float(item.get("processing_ms") or 0.0) / 1000.0
        for item in observations
    ]
    tf_stamps = [item[0] for item in tf_trace]
    if not observations or not tf_trace:
        return {"ready": False, "reason": "accepted observations or Arm-camera TF are absent"}
    tick = ready_times[0]
    end = float(observations[-1]["stamp"])
    samples: list[dict[str, Any]] = []
    compute_ms: list[float] = []
    observation_ages: list[float] = []
    tf_ages: list[float] = []
    output_jumps: list[float] = []
    previous_output: np.ndarray | None = None
    missed = 0
    stale = 0
    nonfinite = 0
    repeated_observation_ticks = 0
    previous_observation_stamp = None
    while tick <= end + 1.0e-9:
        observation_index = bisect.bisect_right(ready_times, tick) - 1
        tf_index = bisect.bisect_right(tf_stamps, tick) - 1
        if observation_index < 0 or tf_index < 0:
            missed += 1
            tick += period
            continue
        observation = observations[observation_index]
        observation_stamp = float(observation["stamp"])
        observation_age = tick - observation_stamp
        tf_stamp, rotation = tf_trace[tf_index]
        tf_age = tick - float(tf_stamp)
        if observation_age > 0.20 or tf_age > 0.75:
            stale += 1
            tick += period
            continue
        started = time.perf_counter()
        try:
            solution = geometry.base_yz_pixel_alignment_delta(
                tuple(observation["centroid_uv"]), image_center,
                k.reshape(3, 3), distortion, rotation, range_scale,
            )
            raw = np.asarray(solution["delta_base_yz_m"], dtype=np.float64)
            norm = float(np.linalg.norm(raw))
            bounded = raw * min(1.0, 0.005 / max(norm, 1.0e-12))
            if not np.all(np.isfinite(bounded)):
                raise ValueError("control output is non-finite")
        except Exception as exc:
            nonfinite += 1
            samples.append({"tick": tick, "error": f"{type(exc).__name__}: {exc}"})
            tick += period
            continue
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        compute_ms.append(elapsed_ms)
        observation_ages.append(observation_age)
        tf_ages.append(tf_age)
        if previous_output is not None:
            output_jumps.append(float(np.linalg.norm(bounded - previous_output)))
        if previous_observation_stamp == observation_stamp:
            repeated_observation_ticks += 1
        previous_observation_stamp = observation_stamp
        previous_output = bounded
        samples.append({
            "tick": tick,
            "observation_stamp": observation_stamp,
            "observation_age_sec": observation_age,
            "tf_stamp": float(tf_stamp),
            "tf_age_sec": tf_age,
            "pixel_error_norm": float(solution["pixel_error_norm"]),
            "raw_delta_yz_m": raw.tolist(),
            "bounded_delta_yz_m": bounded.tolist(),
            "compute_ms": elapsed_ms,
        })
        tick += period
    expected_ticks = int(math.floor((end - ready_times[0]) / period)) + 1
    successful = len(compute_ms)
    ready_fraction = successful / max(expected_ticks, 1)
    compute_p95 = percentile(compute_ms, 95.0)
    observation_age_p95 = percentile(observation_ages, 95.0)
    tf_age_p95 = percentile(tf_ages, 95.0)
    stable = bool(
        ready_fraction >= 0.99
        and nonfinite == 0
        and compute_p95 is not None and compute_p95 <= period * 1000.0 * 0.25
        and observation_age_p95 is not None and observation_age_p95 <= 0.15
        and tf_age_p95 is not None and tf_age_p95 <= 0.30
    )
    return {
        "ready": stable,
        "control_hz": control_hz,
        "period_ms": period * 1000.0,
        "expected_ticks": expected_ticks,
        "successful_ticks": successful,
        "ready_fraction": ready_fraction,
        "missed_ticks": missed,
        "stale_input_ticks": stale,
        "nonfinite_output_ticks": nonfinite,
        "repeated_observation_ticks": repeated_observation_ticks,
        "observation_age_ms_median": (
            statistics.median(observation_ages) * 1000.0 if observation_ages else None
        ),
        "observation_age_ms_p95": (
            observation_age_p95 * 1000.0 if observation_age_p95 is not None else None
        ),
        "tf_age_ms_p95": tf_age_p95 * 1000.0 if tf_age_p95 is not None else None,
        "compute_ms_median": statistics.median(compute_ms) if compute_ms else None,
        "compute_ms_p95": compute_p95,
        "compute_ms_max": max(compute_ms, default=None),
        "bounded_output_jump_m_p95": percentile(output_jumps, 95.0),
        "reference": "ARM_CAMERA_IMAGE_CENTER",
        "range_policy": "INITIAL_D435_ANCHOR_RANGE_SCALE_ONLY",
        "trace": samples,
    }


def benchmark_recorded_ik(record_path: Path, repetitions: int = 20) -> dict[str, Any]:
    """Benchmark the real coarse IK on both recorded approach waypoints."""
    evidence_path = record_path.parent / "motor_evidence.json"
    if not evidence_path.exists():
        return {"ready": False, "reason": f"missing {evidence_path}"}
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    kinematics = load_kinematics_module()
    reports: list[dict[str, Any]] = []
    all_latencies: list[float] = []
    for index, step in enumerate(evidence.get("approach_steps", [])):
        plan = step.get("planned_approach", {})
        feedback = step.get("joint_feedback_rad_at_completion", {})
        try:
            current = [float(feedback[name]) for name in kinematics.JOINT_NAMES]
            target = np.asarray(plan["waypoint_arm_base_m"], dtype=np.float64)
            tool_axis = np.asarray(plan["ik"]["tool_axis"], dtype=np.float64)
            reference_id = str(plan["ik"]["prepare_reference_id"])
        except (KeyError, TypeError, ValueError):
            continue
        latencies: list[float] = []
        successes = 0
        maximum_position_error = 0.0
        for _ in range(max(1, repetitions)):
            started = time.perf_counter()
            result = kinematics.solve_pre_touch(
                current, target, tool_axis, locked_reference_id=reference_id
            )
            latencies.append((time.perf_counter() - started) * 1000.0)
            successes += int(bool(result.get("success")))
            maximum_position_error = max(
                maximum_position_error, float(result.get("position_error_m", math.inf))
            )
        all_latencies.extend(latencies)
        reports.append({
            "step_index": index,
            "repetitions": len(latencies),
            "successes": successes,
            "latency_ms_median": statistics.median(latencies),
            "latency_ms_p95": percentile(latencies, 95.0),
            "latency_ms_max": max(latencies),
            "maximum_position_error_m": maximum_position_error,
        })
    overall_p95 = percentile(all_latencies, 95.0)
    ready = bool(
        reports
        and all(item["successes"] == item["repetitions"] for item in reports)
        and overall_p95 is not None and overall_p95 <= 25.0
    )
    return {
        "ready": ready,
        "budget_ms_for_10_hz_cycle": 25.0,
        "latency_ms_p95": overall_p95,
        "latency_ms_max": max(all_latencies, default=None),
        "steps": reports,
    }


def replay(
    bag: Path, record_path: Path, duration: float, control_hz: float = 10.0
) -> dict[str, Any]:
    record, initial_rgb, initial_mask = load_initial(record_path)
    tracker_module = load_tracker_module()
    tracker = tracker_module.ContinuousTargetTracker()
    height, width = initial_rgb.shape[:2]
    fixed_tip = (0.496 * (width - 1), 0.735 * (height - 1))
    initial = tracker.initialize(initial_rgb, initial_mask, excluded_tip_uv=fixed_tip)
    start_stamp = float(record["image_stamp"])
    end_stamp = float("inf") if duration <= 0.0 else start_stamp + duration

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[ARM_TOPIC]))
    trace: list[dict[str, Any]] = []
    input_stamps: list[float] = []
    accepted_stamps: list[float] = []
    methods: dict[str, int] = {}
    statuses: dict[str, int] = {}
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic != ARM_TOPIC:
            continue
        message = deserialize_message(serialized, Image)
        stamp = image_stamp(message)
        if stamp <= start_stamp:
            continue
        if stamp > end_stamp:
            break
        input_stamps.append(stamp)
        result = tracker.update(image_rgb(message), excluded_tip_uv=fixed_tip)
        status = str(result["status"])
        method = str(result["method"])
        statuses[status] = statuses.get(status, 0) + 1
        methods[method] = methods.get(method, 0) + 1
        if status in {"TRACKING", "RECOVERED"}:
            accepted_stamps.append(stamp)
        trace.append({
            "stamp": stamp,
            "status": status,
            "method": method,
            "centroid_uv": result.get("centroid_uv"),
            "mask_fraction": result.get("mask_fraction"),
            "valid_features": result.get("tracked_points"),
            "inlier_ratio": result.get("inlier_ratio"),
            "processing_ms": result.get("processing_ms"),
            "reason": result.get("reason", ""),
        })

    elapsed = max(input_stamps[-1] - input_stamps[0], 0.0) if len(input_stamps) > 1 else 0.0
    gaps = np.diff(accepted_stamps).tolist() if len(accepted_stamps) > 1 else []
    processing = [float(item["processing_ms"]) for item in trace]
    features = [
        int(item["valid_features"])
        for item in trace if item["status"] in {"TRACKING", "RECOVERED"}
    ]
    accepted_fraction = len(accepted_stamps) / max(len(input_stamps), 1)
    input_rate = (len(input_stamps) - 1) / elapsed if elapsed > 0.0 else 0.0
    accepted_rate = (len(accepted_stamps) - 1) / elapsed if elapsed > 0.0 else 0.0
    p95_ms = percentile(processing, 95.0)
    perception_ready = bool(
        input_rate >= 10.0
        and accepted_rate >= 5.0
        and accepted_fraction >= 0.90
        and p95_ms is not None
        and p95_ms <= 100.0
    )
    tf_trace = load_arm_camera_tf_trace(bag)
    shadow_control = simulate_shadow_control(
        trace, record, tf_trace, control_hz
    )
    ik_benchmark = benchmark_recorded_ik(record_path)
    approach_windows = load_approach_windows(bag)
    motion_reports: list[dict[str, Any]] = []
    control_trace = shadow_control.get("trace", [])
    for window in approach_windows:
        start = float(window["start_stamp"])
        end = float(window["end_stamp"])
        replay_start = float(trace[0]["stamp"]) if trace else math.inf
        replay_end = float(trace[-1]["stamp"]) if trace else -math.inf
        fully_covered = bool(start >= replay_start and end <= replay_end)
        motion_frames = [
            item for item in trace if start < float(item["stamp"]) <= end
        ]
        successful_stamps = [
            float(item["stamp"]) for item in motion_frames
            if item.get("status") in {"TRACKING", "RECOVERED"}
        ]
        boundaries = [start, *successful_stamps, end]
        motion_gaps = [
            boundaries[index + 1] - boundaries[index]
            for index in range(len(boundaries) - 1)
        ]
        centroids = [
            np.asarray(item["centroid_uv"], dtype=np.float64)
            for item in motion_frames if item.get("centroid_uv") is not None
        ]
        motion_control_ticks = [
            item for item in control_trace
            if "tick" in item and start < float(item["tick"]) <= end
        ]
        successful_control_ticks = [
            item for item in motion_control_ticks if "bounded_delta_yz_m" in item
        ]
        motion_reports.append({
            **window,
            "fully_covered_by_replay": fully_covered,
            "duration_sec": end - start,
            "tracker_frames": len(motion_frames),
            "successful_tracker_frames": len(successful_stamps),
            "tracker_acceptance_fraction": (
                len(successful_stamps) / max(len(motion_frames), 1)
            ),
            "maximum_tracker_gap_sec_including_boundaries": max(
                motion_gaps, default=None
            ),
            "centroid_start_uv": centroids[0].tolist() if centroids else None,
            "centroid_end_uv": centroids[-1].tolist() if centroids else None,
            "centroid_displacement_px": (
                float(np.linalg.norm(centroids[-1] - centroids[0]))
                if len(centroids) >= 2 else None
            ),
            "control_ticks": len(motion_control_ticks),
            "successful_control_ticks": len(successful_control_ticks),
            "tracking_during_motion_ready": bool(
                fully_covered
                and len(successful_stamps) >= 3
                and max(motion_gaps, default=math.inf) <= 0.50
            ),
        })
    same_cycle_ready = bool(
        perception_ready
        and shadow_control.get("ready")
        and ik_benchmark.get("ready")
    )
    return {
        "schema_version": 1,
        "mode": "READ_ONLY_FULL_RATE_TRACKER_SHADOW_REPLAY",
        "bag": str(bag),
        "initial_human_confirmation": str(record_path),
        "initial": initial,
        "interval": {
            "start_stamp": start_stamp,
            "requested_duration_sec": duration,
            "observed_duration_sec": elapsed,
            "input_frames": len(input_stamps),
        },
        "tracking": {
            "statuses": statuses,
            "methods": methods,
            "accepted_fraction": accepted_fraction,
            "camera_input_hz": input_rate,
            "accepted_tracking_hz": accepted_rate,
            "largest_accepted_gap_sec": max(gaps, default=None),
            "processing_ms_median": statistics.median(processing) if processing else None,
            "processing_ms_p95": p95_ms,
            "processing_ms_max": max(processing, default=None),
            "valid_features_min": min(features, default=None),
            "valid_features_median": statistics.median(features) if features else None,
        },
        "shadow_control": shadow_control,
        "recorded_ik_benchmark": ik_benchmark,
        "recorded_approach_windows": motion_reports,
        "gates": {
            "perception_ready": perception_ready,
            "tracking_and_control_same_cycle_ready": same_cycle_ready,
            "all_covered_motion_windows_trackable": bool(
                any(item["fully_covered_by_replay"] for item in motion_reports)
                and all(
                    item["tracking_during_motion_ready"]
                    for item in motion_reports if item["fully_covered_by_replay"]
                )
            ),
            "real_continuous_arm_motion_verified": False,
        },
        "interpretation": {
            "proves": [
                "the real tracker can consume recorded Arm Camera frames at full rate",
                "the real image-Jacobian control calculation can consume latest tracker results on a fixed control clock",
                "feature count, inlier ratio and processing latency remain observable",
            ],
            "does_not_prove": [
                "real actuator command streaming",
                "mechanical tracking accuracy, contact or press success",
                "fresh TF availability during a new live run",
            ],
        },
        "trace": trace,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--initial-arm-observation", type=Path)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--control-hz", type=float, default=10.0)
    parser.add_argument("--json-output", type=Path)
    parser.add_argument(
        "--print-trace", action="store_true",
        help="also print the per-frame trace (always retained in --json-output)",
    )
    args = parser.parse_args()
    bag = args.bag.resolve()
    record = find_initial_record(bag, args.initial_arm_observation)
    report = replay(bag, record, args.duration, args.control_hz)
    output = json.dumps(report, indent=2, ensure_ascii=False)
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(output + "\n", encoding="utf-8")
    if args.print_trace:
        printable = report
    else:
        printable = {key: value for key, value in report.items() if key != "trace"}
        printable["shadow_control"] = {
            key: value for key, value in report["shadow_control"].items()
            if key != "trace"
        }
    print(json.dumps(printable, indent=2, ensure_ascii=False))
    return 0 if report["gates"]["tracking_and_control_same_cycle_ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

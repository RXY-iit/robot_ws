#!/usr/bin/env python3

"""Guarded real-hardware FollowJointTrajectory server for SO101 baseline tests."""

from __future__ import annotations

import json
import fcntl
import math
import os
from pathlib import Path
import threading
import time
from typing import Any
from datetime import datetime, timezone
import uuid

import yaml
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from so101_arm_control.read_only_probe import (
    EXPECTED_CALIBRATION,
    EXPECTED_CALIBRATION_SHA256,
    EXPECTED_PORT,
    MOTOR_NAMES,
    SafetyViolation,
    load_and_validate_calibration,
    validate_port,
)
from so101_arm_control.real_read_only_joint_state_publisher import raw_positions_to_urdf
from so101_arm_control.visual_servo import (
    PROTOCOL as VISUAL_SERVO_PROTOCOL,
    select_action_horizon_target,
    validate_command as validate_visual_servo_command,
    validate_sequence as validate_visual_servo_sequence,
    watchdog_expired,
)


CONFIG_PATH = Path(
    "/home/matsunaga-h/robot_ws/src/so101_arm_control/config/arm_real_dashboard_test.yaml"
)
CHANGE_EPS_RAD = 0.002
CONTROL_MODES = {"real2sim", "code2real&sim", "teleoperate"}
COMMAND_CONTROL_MODE = "code2real&sim"
TELEOP_CONTROL_MODE = "teleoperate"
POWERED_CONTROL_MODES = {COMMAND_CONTROL_MODE, TELEOP_CONTROL_MODE}
SERIAL_OWNER_LOCK = Path("/tmp/so101_follower_5AE6054086.lock")


class GoalToleranceViolation(SafetyViolation):
    """A safe, retryable failure to settle close enough to the requested pose."""


class SafeRecoveryRequired(SafetyViolation):
    """The current pose is outside the HOLD inset and the goal does not recover it."""


class ProtectionTrip(SafetyViolation):
    """A latched current, temperature, resistance, or HOLD protection event."""


def trajectory_errors(joint_specs: list[dict], trajectory, max_duration_sec: float) -> list[str]:
    expected = [str(spec["name"]) for spec in joint_specs]
    if list(trajectory.joint_names) != expected:
        return ["joint_names must exactly match configured order"]
    if len(trajectory.points) != 1:
        return ["real baseline accepts exactly one trajectory point"]
    point = trajectory.points[0]
    if len(point.positions) != len(expected):
        return [f"expected {len(expected)} positions"]
    duration = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1e9
    errors: list[str] = []
    if duration < 0.5 or duration > max_duration_sec:
        errors.append(f"duration must be within [0.5, {max_duration_sec}] seconds")
    for spec, value in zip(joint_specs, point.positions):
        lower = float(spec["lower"])
        upper = float(spec["upper"])
        if not math.isfinite(value) or value < lower or value > upper:
            errors.append(f"{spec['name']}: outside [{lower}, {upper}]")
    return errors


def changed_joint_indices(actual: list[float], target: list[float]) -> list[int]:
    changed = [
        index for index, (start, finish) in enumerate(zip(actual, target))
        if abs(finish - start) > CHANGE_EPS_RAD
    ]
    if not changed:
        raise SafetyViolation("At least one joint must change")
    return changed


def classify_hold_inset_recovery(
    starts: dict[str, int],
    targets: dict[str, int],
    safe_limits: dict[str, tuple[int, int]],
) -> tuple[dict[str, dict[str, Any]], dict[str, dict[str, Any]]]:
    """Split out-of-inset starts into recoverable and still-unsafe goals."""
    recovering: dict[str, dict[str, Any]] = {}
    required: dict[str, dict[str, Any]] = {}
    for motor_name, start in starts.items():
        safe_min, safe_max = safe_limits[motor_name]
        if safe_min <= start <= safe_max:
            continue
        details = {
            "actual": start,
            "target": targets[motor_name],
            "safe": [safe_min, safe_max],
        }
        if safe_min <= targets[motor_name] <= safe_max:
            recovering[motor_name] = details
        else:
            required[motor_name] = details
    return recovering, required


def position_recovery_details(
    raw_positions: dict[str, int | float],
    calibration: dict[str, dict[str, int]],
    safe_limits: dict[str, tuple[int, int]],
    tolerance_ticks: int = 0,
) -> dict[str, dict[str, Any]]:
    """Describe measured joints materially outside the command envelope.

    The tolerance applies only to observed-state classification.  It never
    expands command limits or calibration bounds.
    """
    details: dict[str, dict[str, Any]] = {}
    for motor_name, value in raw_positions.items():
        raw = int(value)
        safe_min, safe_max = safe_limits[motor_name]
        if safe_min - tolerance_ticks <= raw <= safe_max + tolerance_ticks:
            continue
        calibration_min = int(calibration[motor_name]["range_min"])
        calibration_max = int(calibration[motor_name]["range_max"])
        details[motor_name] = {
            "raw": raw,
            "safe": [safe_min, safe_max],
            "calibration": [calibration_min, calibration_max],
            "calibration_outside_ticks": max(
                calibration_min - raw, raw - calibration_max, 0
            ),
            "required_direction": "increase" if raw < safe_min else "decrease",
        }
    return details


def minimum_motion_duration(
    actual: list[float],
    target: list[float],
    motor_names: list[str],
    max_velocity_rad_s: dict[str, float],
) -> float:
    durations = []
    for start, finish, motor_name in zip(actual, target, motor_names):
        velocity = float(max_velocity_rad_s[motor_name])
        if not math.isfinite(velocity) or velocity <= 0.0:
            raise SafetyViolation(f"Invalid max velocity for {motor_name}: {velocity}")
        durations.append(abs(finish - start) / velocity)
    return max(durations, default=0.0)


def goal_tolerance_violations(
    errors: dict[str, int], tolerances: dict[str, int]
) -> dict[str, dict[str, int]]:
    """Return motors whose absolute final error exceeds their HOLD-entry tolerance."""
    return {
        name: {"error": int(error), "tolerance": int(tolerances[name])}
        for name, error in errors.items()
        if abs(int(error)) > int(tolerances[name])
    }


def near_goal_settle_extension_allowed(
    violations: dict[str, dict[str, int]],
    initial_errors: dict[str, int],
    hold_position_error_ticks: int,
) -> bool:
    """Allow one extra settle window only for a near-goal, converging endpoint.

    This does not admit the goal.  It merely keeps the already-commanded target
    active for another short observation window while all normal protection
    monitors remain armed.
    """
    if not violations:
        return False
    return all(
        abs(int(item["error"])) < int(hold_position_error_ticks)
        and name in initial_errors
        and abs(int(item["error"])) < abs(int(initial_errors[name]))
        for name, item in violations.items()
    )


class CurrentProtectionMonitor:
    """Detect an immediate hard over-current or a sustained resistance/stall."""

    def __init__(
        self,
        *,
        hard_current_raw: int,
        resistance_current_raw: dict[str, int],
        resistance_samples: int,
        following_error_ticks: int,
        max_progress_ticks: int,
        grace_sec: float,
    ) -> None:
        self.hard_current_raw = hard_current_raw
        self.resistance_current_raw = resistance_current_raw
        self.resistance_samples = resistance_samples
        self.following_error_ticks = following_error_ticks
        self.max_progress_ticks = max_progress_ticks
        self.grace_sec = grace_sec
        self._counts: dict[str, int] = {}

    def update(
        self,
        motor_name: str,
        *,
        current_raw: int,
        present_raw: int,
        previous_raw: int,
        commanded_raw: int,
        elapsed_sec: float,
    ) -> str | None:
        current = abs(int(current_raw))
        if current >= self.hard_current_raw:
            return "HARD_OVERCURRENT"
        if elapsed_sec < self.grace_sec:
            self._counts[motor_name] = 0
            return None
        threshold = int(self.resistance_current_raw[motor_name])
        resisted = (
            current >= threshold
            and abs(commanded_raw - present_raw) >= self.following_error_ticks
            and abs(present_raw - previous_raw) <= self.max_progress_ticks
        )
        self._counts[motor_name] = self._counts.get(motor_name, 0) + 1 if resisted else 0
        if self._counts[motor_name] >= self.resistance_samples:
            return "RESISTANCE_STALL"
        return None


def hold_sample_update(
    *,
    torque_enabled: int,
    temperature_c: int,
    max_temperature_c: int,
    current_raw: int,
    present_raw: int,
    target_raw: int,
    hard_current_raw: int,
    sustained_current_raw: int,
    current_count: int,
    current_samples_required: int,
    position_error_ticks: int,
    position_count: int,
    position_samples_required: int,
) -> tuple[str | None, int, int]:
    """Classify one sampled HOLD motor state and update debounce counters."""
    if torque_enabled != 1:
        return "HOLD_TORQUE_LOST", current_count, position_count
    if temperature_c > max_temperature_c:
        return "HOLD_OVERTEMPERATURE", current_count, position_count
    current = abs(current_raw)
    if current >= hard_current_raw:
        return "HOLD_HARD_OVERCURRENT", current_count, position_count
    next_current_count = current_count + 1 if current >= sustained_current_raw else 0
    if next_current_count >= current_samples_required:
        return "HOLD_SUSTAINED_CURRENT", next_current_count, position_count
    next_position_count = (
        position_count + 1
        if abs(present_raw - target_raw) > position_error_ticks
        else 0
    )
    if next_position_count >= position_samples_required:
        return "HOLD_POSITION_DISTURBANCE", next_current_count, next_position_count
    return None, next_current_count, next_position_count


def temperature_sample_update(
    *,
    temperature_c: int,
    max_temperature_c: int,
    consecutive_count: int,
    samples_required: int,
) -> tuple[bool, int]:
    """Debounce a temperature limit before an independent register check."""
    next_count = consecutive_count + 1 if temperature_c > max_temperature_c else 0
    return next_count >= samples_required, next_count


def leader_relative_targets_raw(
    *,
    entry_leader_norm: dict[str, float],
    current_leader_norm: dict[str, float],
    entry_follower_raw: dict[str, int],
    calibration: dict[str, dict[str, int]],
    safe_limits: dict[str, tuple[int, int]],
    deadband_norm: float,
) -> tuple[dict[str, int], dict[str, dict[str, int]]]:
    """Map leader displacement since mode entry into safe follower raw targets."""
    expected = set(MOTOR_NAMES)
    if (
        set(entry_leader_norm) != expected
        or set(current_leader_norm) != expected
        or set(entry_follower_raw) != expected
    ):
        raise SafetyViolation("Teleoperate mapping requires all six motors")
    targets: dict[str, int] = {}
    clamped: dict[str, dict[str, int]] = {}
    for name in MOTOR_NAMES:
        start_norm = float(entry_leader_norm[name])
        current_norm = float(current_leader_norm[name])
        if not math.isfinite(start_norm) or not math.isfinite(current_norm):
            raise SafetyViolation(f"Non-finite leader feedback for {name}")
        delta_norm = current_norm - start_norm
        if abs(delta_norm) < deadband_norm:
            delta_norm = 0.0
        norm_span = 100.0 if name == "gripper" else 200.0
        calibration_span = (
            int(calibration[name]["range_max"])
            - int(calibration[name]["range_min"])
        )
        requested = int(
            round(int(entry_follower_raw[name]) + delta_norm * calibration_span / norm_span)
        )
        safe_min, safe_max = safe_limits[name]
        bounded = min(safe_max, max(safe_min, requested))
        targets[name] = bounded
        if bounded != requested:
            clamped[name] = {"requested": requested, "bounded": bounded}
    return targets, clamped


def leader_absolute_targets_raw(
    *,
    leader_norm: dict[str, float],
    calibration: dict[str, dict[str, int]],
    safe_limits: dict[str, tuple[int, int]],
) -> tuple[dict[str, int], dict[str, dict[str, int]]]:
    """Map a calibrated Leader pose to the equivalent Follower raw pose."""
    if set(leader_norm) != set(MOTOR_NAMES):
        raise SafetyViolation("Absolute teleoperate mapping requires all six motors")
    targets: dict[str, int] = {}
    clamped: dict[str, dict[str, int]] = {}
    for name in MOTOR_NAMES:
        value = float(leader_norm[name])
        if not math.isfinite(value):
            raise SafetyViolation(f"Non-finite leader feedback for {name}")
        if name == "wrist_roll":
            while value > 100.0:
                value -= 200.0
            while value < -100.0:
                value += 200.0
        lower, upper = (0.0, 100.0) if name == "gripper" else (-100.0, 100.0)
        if not lower - 1.0 <= value <= upper + 1.0:
            raise SafetyViolation(f"Leader absolute position is invalid for {name}: {value}")
        value = min(upper, max(lower, value))
        fraction = (value - lower) / (upper - lower)
        minimum = int(calibration[name]["range_min"])
        maximum = int(calibration[name]["range_max"])
        requested = int(round(minimum + fraction * (maximum - minimum)))
        safe_min, safe_max = safe_limits[name]
        bounded = min(safe_max, max(safe_min, requested))
        targets[name] = bounded
        if bounded != requested:
            clamped[name] = {"requested": requested, "bounded": bounded}
    return targets, clamped


def urdf_radians_to_raw(
    motor_name: str,
    radians: float,
    joint_spec: dict,
    calibration: dict[str, dict[str, int]],
    margin_ticks: int,
    learned_raw_min: int | None = None,
    learned_raw_max: int | None = None,
) -> int:
    lower = float(joint_spec["lower"])
    upper = float(joint_spec["upper"])
    if not math.isfinite(radians) or not lower <= radians <= upper:
        raise SafetyViolation(f"{joint_spec['name']} target outside URDF limits")
    minimum = int(calibration[motor_name]["range_min"])
    maximum = int(calibration[motor_name]["range_max"])
    fraction = (radians - lower) / (upper - lower)
    raw = int(round(minimum + fraction * (maximum - minimum)))
    safe_minimum = max(minimum + margin_ticks, learned_raw_min or minimum)
    safe_maximum = min(maximum - margin_ticks, learned_raw_max or maximum)
    if safe_minimum >= safe_maximum:
        raise SafetyViolation(f"{motor_name} learned soft limits are invalid")
    if not safe_minimum <= raw <= safe_maximum:
        raise SafetyViolation(
            f"{motor_name} raw target {raw} lacks {margin_ticks}-tick limit margin "
            f"[{safe_minimum}, {safe_maximum}]"
        )
    return raw


def _make_guarded_bus(port: Path, calibration_data: dict[str, dict[str, int]]) -> Any:
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    class GuardedDashboardBus(FeetechMotorsBus):
        active_motors: set[str] = set()

        def write(self, data_name, motor, value, *, normalize=True, num_retry=0):
            if motor not in self.active_motors or data_name not in {"Torque_Enable", "Lock"}:
                raise SafetyViolation(f"Blocked write: {data_name} on {motor}")
            return super().write(
                data_name, motor, value, normalize=normalize, num_retry=num_retry
            )

        def sync_write(self, data_name, values, *, normalize=True, num_retry=0):
            if (
                data_name != "Goal_Position"
                or not self.active_motors
                or not isinstance(values, dict)
                or set(values) != self.active_motors
            ):
                names = tuple(values) if isinstance(values, dict) else ("ALL",)
                raise SafetyViolation(f"Blocked sync write: {data_name} on {names}")
            return super().sync_write(
                data_name, values, normalize=normalize, num_retry=num_retry
            )

        def configure_motors(self, *_args, **_kwargs):
            raise SafetyViolation("Motor configuration is forbidden")

        def write_calibration(self, *_args, **_kwargs):
            raise SafetyViolation("Calibration writes are forbidden")

    calibration = {
        name: MotorCalibration(**calibration_data[name])
        for name in MOTOR_NAMES
    }
    motors = {
        name: Motor(
            calibration_data[name]["id"],
            "sts3215",
            MotorNormMode.RANGE_0_100 if name == "gripper" else MotorNormMode.RANGE_M100_100,
        )
        for name in MOTOR_NAMES
    }
    return GuardedDashboardBus(port=str(port), motors=motors, calibration=calibration)


class RealTrajectoryServer(Node):
    def __init__(self) -> None:
        super().__init__("so101_real_trajectory_server")
        self._serial_owner_lock = SERIAL_OWNER_LOCK.open("a+", encoding="utf-8")
        try:
            fcntl.flock(
                self._serial_owner_lock.fileno(),
                fcntl.LOCK_EX | fcntl.LOCK_NB,
            )
        except BlockingIOError as exc:
            self._serial_owner_lock.close()
            raise SafetyViolation(
                "Another SO101 follower bus owner is already running"
            ) from exc
        self._serial_owner_lock.seek(0)
        self._serial_owner_lock.truncate()
        self._serial_owner_lock.write(f"pid={os.getpid()}\n")
        self._serial_owner_lock.flush()
        config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
        if config.get("mode") != "REAL_DASHBOARD_TEST":
            raise SafetyViolation("Real server requires REAL_DASHBOARD_TEST config")
        if not config.get("hardware", {}).get("real_hardware_enabled"):
            raise SafetyViolation("Real hardware flag is not enabled")
        if config.get("calibration", {}).get("sha256") != EXPECTED_CALIBRATION_SHA256:
            raise SafetyViolation("Config calibration hash mismatch")

        validate_port(EXPECTED_PORT)
        self._calibration = load_and_validate_calibration(
            EXPECTED_CALIBRATION, EXPECTED_CALIBRATION_SHA256
        )
        self._config = config
        self._joint_specs = list(config["joints"])
        self._joint_names = [str(spec["name"]) for spec in self._joint_specs]
        self._motor_names = [str(spec["motor_name"]) for spec in self._joint_specs]
        safety = config["safety"]
        self._max_duration = float(safety["max_duration_sec"])
        self._max_velocity = {
            str(name): float(value)
            for name, value in safety["max_joint_velocity_rad_s"].items()
        }
        self._outside_ticks = int(safety["max_outside_calibration_ticks"])
        self._recovery_outside_ticks = int(
            safety["max_recovery_outside_calibration_ticks"]
        )
        if self._recovery_outside_ticks < self._outside_ticks:
            raise SafetyViolation(
                "max_recovery_outside_calibration_ticks must be at least "
                "max_outside_calibration_ticks"
            )
        self._hard_current = int(safety["hard_current_raw"])
        self._resistance_current = {
            str(name): int(value)
            for name, value in safety["resistance_current_raw"].items()
        }
        self._resistance_samples = int(safety["resistance_samples"])
        self._following_error_ticks = int(safety["resistance_following_error_ticks"])
        self._max_stall_progress_ticks = int(safety["resistance_max_progress_ticks"])
        self._current_grace_sec = float(safety["current_grace_sec"])
        self._max_temperature = int(safety["max_temperature_c"])
        self._temperature_consecutive_samples = int(
            safety["temperature_consecutive_samples"]
        )
        self._temperature_confirmation_reads = int(
            safety["temperature_confirmation_reads"]
        )
        self._temperature_confirmation_min_hot_reads = int(
            safety["temperature_confirmation_min_hot_reads"]
        )
        if self._temperature_consecutive_samples < 2:
            raise SafetyViolation("temperature_consecutive_samples must be at least 2")
        if self._temperature_confirmation_reads < 1:
            raise SafetyViolation("temperature_confirmation_reads must be positive")
        if not (
            1
            <= self._temperature_confirmation_min_hot_reads
            <= self._temperature_confirmation_reads
        ):
            raise SafetyViolation(
                "temperature_confirmation_min_hot_reads must be within confirmation reads"
            )
        self._voltage_warning_min = int(safety["voltage_warning_min_deci_v"])
        self._voltage_warning_max = int(safety["voltage_warning_max_deci_v"])
        self._target_margin = int(safety["target_limit_margin_ticks"])
        self._monitor_rate_hz = float(safety["monitor_rate_hz"])
        self._settle_timeout_sec = float(safety["settle_timeout_sec"])
        self._near_goal_settle_extension_sec = float(
            safety.get("near_goal_settle_extension_sec", 0.0)
        )
        self._goal_tolerance_ticks = {
            str(name): int(value)
            for name, value in safety["goal_acceptance_tolerance_ticks"].items()
        }
        self._boundary_window_ticks = int(safety["boundary_learning_window_ticks"])
        self._boundary_backoff_ticks = int(safety["boundary_backoff_ticks"])
        self._soft_limit_path = Path(str(safety["soft_limit_state_file"]))
        self._hold_monitor_rate_hz = float(safety["hold_monitor_rate_hz"])
        self._hold_current_samples_required = int(safety["hold_current_samples"])
        self._hold_position_error_ticks = int(safety["hold_position_error_ticks"])
        self._hold_position_error_samples_required = int(
            safety["hold_position_error_samples"]
        )
        teleoperation = dict(config.get("teleoperation", {}))
        self._teleop_feedback_timeout_sec = float(
            teleoperation.get("feedback_timeout_sec", 0.35)
        )
        self._teleop_command_rate_hz = float(
            teleoperation.get("command_rate_hz", 20.0)
        )
        self._teleop_deadband_norm = float(
            teleoperation.get("leader_deadband_norm", 0.20)
        )
        self._teleop_position_error_ticks = int(
            teleoperation.get("position_error_ticks", 250)
        )
        self._teleop_position_error_samples = int(
            teleoperation.get("position_error_samples", 10)
        )
        self._teleop_alignment_tolerance_ticks = int(
            teleoperation.get("alignment_tolerance_ticks", 45)
        )
        self._teleop_alignment_max_velocity = {
            str(name): float(value)
            for name, value in teleoperation[
                "alignment_max_joint_velocity_rad_s"
            ].items()
        }
        self._teleop_follow_max_velocity = {
            str(name): float(value)
            for name, value in teleoperation[
                "follow_max_joint_velocity_rad_s"
            ].items()
        }
        visual_servo = dict(config.get("visual_servo", {}))
        if str(visual_servo.get("protocol")) != VISUAL_SERVO_PROTOCOL:
            raise SafetyViolation("visual_servo protocol is missing or unsupported")
        self._visual_servo_rate_hz = float(visual_servo["command_rate_hz"])
        self._visual_servo_watchdog_sec = float(visual_servo["watchdog_sec"])
        self._visual_servo_max_session_sec = float(visual_servo["max_session_sec"])
        self._visual_servo_position_error_ticks = int(
            visual_servo["position_error_ticks"]
        )
        self._visual_servo_position_error_samples = int(
            visual_servo["position_error_samples"]
        )
        self._visual_servo_max_velocity = {
            str(name): float(value)
            for name, value in visual_servo["max_joint_velocity_rad_s"].items()
        }
        if str(teleoperation.get("mapping")) != "absolute_align_then_follow":
            raise SafetyViolation("Only absolute_align_then_follow teleoperation is audited")
        if self._teleop_feedback_timeout_sec <= 0.0:
            raise SafetyViolation("teleoperation feedback timeout must be positive")
        if self._teleop_command_rate_hz <= 0.0:
            raise SafetyViolation("teleoperation command rate must be positive")
        if self._teleop_position_error_ticks <= self._hold_position_error_ticks:
            raise SafetyViolation(
                "teleoperation position error must exceed the rigid HOLD threshold"
            )
        if self._teleop_position_error_samples < self._hold_position_error_samples_required:
            raise SafetyViolation(
                "teleoperation position error samples must be at least the rigid HOLD samples"
            )
        if self._teleop_alignment_tolerance_ticks < 1:
            raise SafetyViolation("teleoperation alignment tolerance must be positive")
        expected_motors = set(self._motor_names)
        if set(self._max_velocity) != expected_motors:
            raise SafetyViolation("max_joint_velocity_rad_s must cover every configured motor")
        if set(self._teleop_alignment_max_velocity) != expected_motors:
            raise SafetyViolation(
                "teleoperation alignment velocity limits must cover every configured motor"
            )
        if set(self._teleop_follow_max_velocity) != expected_motors:
            raise SafetyViolation(
                "teleoperation follow velocity limits must cover every configured motor"
            )
        if set(self._visual_servo_max_velocity) != expected_motors:
            raise SafetyViolation(
                "visual-servo velocity limits must cover every configured motor"
            )
        invalid_teleop_velocities = {
            f"{profile}.{name}": value
            for profile, limits in (
                ("alignment", self._teleop_alignment_max_velocity),
                ("follow", self._teleop_follow_max_velocity),
            )
            for name, value in limits.items()
            if value <= 0.0 or not math.isfinite(value)
        }
        if invalid_teleop_velocities:
            raise SafetyViolation(
                f"teleoperation velocity limits must be finite and positive: "
                f"{invalid_teleop_velocities}"
            )
        invalid_visual_servo_velocities = {
            name: value for name, value in self._visual_servo_max_velocity.items()
            if value <= 0.0 or not math.isfinite(value)
        }
        if invalid_visual_servo_velocities:
            raise SafetyViolation(
                "visual-servo velocity limits must be finite and positive: "
                f"{invalid_visual_servo_velocities}"
            )
        if not 10.0 <= self._visual_servo_rate_hz <= 50.0:
            raise SafetyViolation("visual-servo command rate must be within [10, 50] Hz")
        if not 0.15 <= self._visual_servo_watchdog_sec <= 0.75:
            raise SafetyViolation("visual-servo watchdog must be within [0.15, 0.75] s")
        if not 0.5 <= self._visual_servo_max_session_sec <= 60.0:
            raise SafetyViolation("visual-servo session limit must be within [0.5, 10] s")
        if self._visual_servo_position_error_ticks <= self._hold_position_error_ticks:
            raise SafetyViolation(
                "visual-servo position error must exceed rigid HOLD threshold"
            )
        if self._visual_servo_position_error_samples < self._hold_position_error_samples_required:
            raise SafetyViolation(
                "visual-servo position error samples must be at least HOLD samples"
            )
        if set(self._resistance_current) != expected_motors:
            raise SafetyViolation("resistance_current_raw must cover every configured motor")
        if set(self._goal_tolerance_ticks) != expected_motors:
            raise SafetyViolation(
                "goal_acceptance_tolerance_ticks must cover every configured motor"
            )
        invalid_tolerances = {
            name: value
            for name, value in self._goal_tolerance_ticks.items()
            if value < 1 or value >= self._hold_position_error_ticks
        }
        if invalid_tolerances:
            raise SafetyViolation(
                "goal acceptance tolerances must be positive and below the HOLD "
                f"disturbance threshold: {invalid_tolerances}"
            )
        if (
            not math.isfinite(self._near_goal_settle_extension_sec)
            or self._near_goal_settle_extension_sec < 0.0
        ):
            raise SafetyViolation(
                "near_goal_settle_extension_sec must be finite and non-negative"
            )
        self._soft_limit_state = self._load_soft_limit_state()
        previous_events = self._soft_limit_state.get("events", [])
        self._last_protection_trip = previous_events[-1] if previous_events else None

        self._state_lock = threading.Lock()
        self._bus_lock = threading.Lock()
        self._active = False
        self._active_motors: set[str] = set()
        self._holding = False
        self._hold_state = "RELEASED"
        self._hold_reason = "startup torque-off"
        self._hold_targets_raw: dict[str, int] = {}
        self._hold_started_at: str | None = None
        self._hold_current_counts = {name: 0 for name in MOTOR_NAMES}
        self._hold_position_counts = {name: 0 for name in MOTOR_NAMES}
        self._hold_temperature_counts = {name: 0 for name in MOTOR_NAMES}
        self._temperature_rejected_spike_count = 0
        self._last_temperature_rejection: dict[str, Any] | None = None
        self._last_motion_report: dict[str, Any] | None = None
        self._leader_status: dict[str, Any] = {}
        self._last_leader_update: float | None = None
        self._teleop_entry_leader_norm: dict[str, float] = {}
        self._teleop_entry_follower_raw: dict[str, int] = {}
        self._teleop_last_target_raw: dict[str, int] = {}
        self._teleop_last_command_at: float | None = None
        self._teleop_clamped: dict[str, dict[str, int]] = {}
        self._teleop_aligning = False
        self._teleop_initial_target_raw: dict[str, int] = {}
        self._visual_servo_session_id: str | None = None
        self._visual_servo_last_session_id: str | None = None
        self._visual_servo_sequence = -1
        self._visual_servo_requested_rad: dict[str, float] = {}
        self._visual_servo_applied_raw: dict[str, int] = {}
        self._visual_servo_last_update: float | None = None
        self._visual_servo_last_apply: float | None = None
        self._visual_servo_started: float | None = None
        self._visual_servo_state = "INACTIVE"
        self._visual_servo_reason = "no session"
        self._visual_servo_update_count = 0
        self._visual_servo_action_horizon: list[dict[str, float]] = []
        self._visual_servo_horizon_step_period_sec = 0.0
        self._visual_servo_horizon_started: float | None = None
        self._visual_servo_horizon_plan_id = ""
        self._visual_servo_horizon_index = -1
        self._visual_servo_horizon_replan_count = 0
        self._visual_servo_observation_age_sec = 0.0
        self._visual_servo_prediction_lead_sec = 0.0
        self._visual_servo_lock = threading.RLock()
        self._control_mode = str(config.get("default_control_mode", "real2sim"))
        if self._control_mode not in CONTROL_MODES:
            raise SafetyViolation(f"Invalid default control mode: {self._control_mode}")
        self._positions = [0.0] * len(self._joint_names)
        self._desired = list(self._positions)
        self._raw_positions: dict[str, int | float] = {}
        self._position_recovery: dict[str, Any] = {
            "required": False,
            "joints": {},
            "feedback_clamped": {},
        }
        self._sample_count = 0
        self._bus = _make_guarded_bus(EXPECTED_PORT, self._calibration)
        self._connected = False

        try:
            try:
                self._bus.connect(handshake=True)
            except RuntimeError as exc:
                message = str(exc)
                if "Missing motor IDs" in message and "Full found motor list" in message:
                    raise SafetyViolation(
                        "Follower USB controller is present, but one or more servo IDs did not "
                        "respond. If all six IDs are missing, verify the follower 12 V supply, "
                        "power switch, and servo-bus cable before retrying. No torque command "
                        "was sent. Original handshake result: "
                        + message
                    ) from exc
                raise
            self._connected = True
            torque = self._bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
            enabled = {name: int(value) for name, value in torque.items() if int(value)}
            if enabled:
                raise SafetyViolation(
                    f"Real server requires all torque disabled at startup: {enabled}"
                )
            self._refresh_positions()
            self._desired = list(self._positions)
        except Exception:
            if self._bus.is_connected:
                self._bus.disconnect(disable_torque=False)
            self._connected = False
            raise

        self._joint_pub = self.create_publisher(JointState, "/joint_states", 20)
        self._controller_pub = self.create_publisher(
            JointTrajectoryControllerState, "/arm_controller/controller_state", 20
        )
        self._health_pub = self.create_publisher(String, "/arm/hardware_status", 10)
        self._visual_servo_state_pub = self.create_publisher(
            String, "/arm/visual_servo_state", 20
        )
        self.create_subscription(String, "/arm/control_mode", self._on_control_mode, 10)
        self.create_subscription(String, "/arm/hold_command", self._on_hold_command, 10)
        self.create_subscription(
            String, "/arm/visual_servo_command",
            self._on_visual_servo_command, 50,
        )
        self.create_subscription(
            String, "/arm/leader_status", self._on_leader_status, 20
        )
        callbacks = ReentrantCallbackGroup()
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=callbacks,
        )
        self.create_timer(0.2, self._publish_state, callback_group=callbacks)
        self.create_timer(1.0, self._publish_health, callback_group=callbacks)
        self.create_timer(
            1.0 / self._hold_monitor_rate_hz,
            self._monitor_hold,
            callback_group=callbacks,
        )
        self.create_timer(
            1.0 / self._teleop_command_rate_hz,
            self._teleop_tick,
            callback_group=callbacks,
        )
        self.create_timer(
            1.0 / self._visual_servo_rate_hz,
            self._visual_servo_tick,
            callback_group=callbacks,
        )
        self.get_logger().warning(
            "REAL DASHBOARD TEST: multi-joint guarded motion; sampled current/stall protection; "
            "successful code2 goals enter monitored six-joint HOLD"
        )

    def _load_soft_limit_state(self) -> dict[str, Any]:
        empty = {
            "schema_version": 1,
            "calibration_sha256": EXPECTED_CALIBRATION_SHA256,
            "limits": {},
            "events": [],
        }
        if not self._soft_limit_path.exists():
            return empty
        try:
            state = json.loads(self._soft_limit_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise SafetyViolation(f"Cannot read learned soft limits: {exc}") from exc
        if not isinstance(state, dict):
            raise SafetyViolation("Learned soft-limit state must be a JSON object")
        if state.get("calibration_sha256") != EXPECTED_CALIBRATION_SHA256:
            raise SafetyViolation("Learned soft limits belong to a different calibration")
        if not isinstance(state.get("limits", {}), dict) or not isinstance(
            state.get("events", []), list
        ):
            raise SafetyViolation("Learned soft-limit state has invalid limits/events")
        return state

    def _save_soft_limit_state(self) -> None:
        self._soft_limit_path.parent.mkdir(parents=True, exist_ok=True)
        self._soft_limit_state["updated_at"] = datetime.now(timezone.utc).isoformat()
        temporary = self._soft_limit_path.with_suffix(self._soft_limit_path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(self._soft_limit_state, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self._soft_limit_path)

    def _learned_bounds(self, motor_name: str) -> tuple[int | None, int | None]:
        item = self._soft_limit_state.get("limits", {}).get(motor_name, {})
        learned_min = item.get("raw_min")
        learned_max = item.get("raw_max")
        return (
            int(learned_min) if learned_min is not None else None,
            int(learned_max) if learned_max is not None else None,
        )

    def _effective_raw_limits(self, motor_name: str) -> tuple[int, int]:
        calibration = self._calibration[motor_name]
        minimum = int(calibration["range_min"]) + self._target_margin
        maximum = int(calibration["range_max"]) - self._target_margin
        learned_min, learned_max = self._learned_bounds(motor_name)
        if learned_min is not None:
            minimum = max(minimum, learned_min)
        if learned_max is not None:
            maximum = min(maximum, learned_max)
        if minimum >= maximum:
            raise SafetyViolation(f"Invalid effective soft limits for {motor_name}")
        return minimum, maximum

    def _command_limits_rad(self) -> dict[str, dict[str, Any]]:
        limits: dict[str, dict[str, Any]] = {}
        for spec, motor_name in zip(self._joint_specs, self._motor_names):
            raw_min, raw_max = self._effective_raw_limits(motor_name)
            calibration = self._calibration[motor_name]
            range_min = int(calibration["range_min"])
            range_max = int(calibration["range_max"])
            lower = float(spec["lower"])
            upper = float(spec["upper"])

            def to_rad(raw: int) -> float:
                fraction = (raw - range_min) / (range_max - range_min)
                return lower + fraction * (upper - lower)

            learned_min, learned_max = self._learned_bounds(motor_name)
            limits[str(spec["name"])] = {
                "lower": to_rad(raw_min),
                "upper": to_rad(raw_max),
                "raw_min": raw_min,
                "raw_max": raw_max,
                "learned_min": learned_min is not None,
                "learned_max": learned_max is not None,
            }
        return limits

    def _record_protection_trip(
        self,
        *,
        reason: str,
        tripped_motor: str,
        currents: dict[str, int],
        temperatures: dict[str, int],
        positions: dict[str, int],
        start_raw: dict[str, int],
        target_raw: dict[str, int],
        temperature_confirmation_c: list[int] | None = None,
    ) -> dict[str, Any]:
        present = int(positions[tripped_motor])
        direction = 1 if target_raw[tripped_motor] > start_raw[tripped_motor] else -1
        calibration = self._calibration[tripped_motor]
        range_min = int(calibration["range_min"])
        range_max = int(calibration["range_max"])
        current_or_resistance = reason in {"HARD_OVERCURRENT", "RESISTANCE_STALL"}
        near_endpoint = current_or_resistance and (
            (direction < 0 and present - range_min <= self._boundary_window_ticks)
            or (direction > 0 and range_max - present <= self._boundary_window_ticks)
        )
        boundary_update: dict[str, Any] | None = None
        if near_endpoint:
            limits = self._soft_limit_state.setdefault("limits", {})
            item = limits.setdefault(tripped_motor, {})
            if direction < 0:
                learned = min(
                    range_max - self._target_margin,
                    present + self._boundary_backoff_ticks,
                )
                previous = item.get("raw_min")
                item["raw_min"] = max(int(previous), learned) if previous is not None else learned
                boundary_update = {"side": "min", "raw": item["raw_min"]}
            else:
                learned = max(
                    range_min + self._target_margin,
                    present - self._boundary_backoff_ticks,
                )
                previous = item.get("raw_max")
                item["raw_max"] = min(int(previous), learned) if previous is not None else learned
                boundary_update = {"side": "max", "raw": item["raw_max"]}
            item["source"] = "sampled_current_or_resistance_trip"

        event = {
            "id": str(uuid.uuid4()),
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "reason": reason,
            "tripped_motor": tripped_motor,
            "current_raw": {name: int(value) for name, value in currents.items()},
            "temperature_c": {
                name: int(value) for name, value in temperatures.items()
            },
            "present_raw": {name: int(value) for name, value in positions.items()},
            "start_raw": start_raw,
            "target_raw": target_raw,
            "temperature_confirmation_c": temperature_confirmation_c,
            "direction": direction,
            "near_calibrated_endpoint": near_endpoint,
            "boundary_update": boundary_update,
        }
        events = self._soft_limit_state.setdefault("events", [])
        events.append(event)
        del events[:-200]
        self._last_protection_trip = event
        self._save_soft_limit_state()
        return event

    def _disable_motors(self, motors: set[str]) -> list[str]:
        errors: list[str] = []
        remaining = set(motors)
        with self._bus_lock:
            self._bus.active_motors = set(motors)
            for motor_name in sorted(motors):
                try:
                    self._bus.disable_torque(motor_name, num_retry=3)
                    remaining.discard(motor_name)
                except Exception as exc:
                    errors.append(f"{motor_name}: {exc}")
            self._bus.active_motors = remaining
        with self._state_lock:
            self._active_motors = remaining
        return errors

    def _confirm_motor_overtemperature(
        self, motor_name: str
    ) -> tuple[bool, list[int]]:
        """Confirm a debounced sync-read alarm via direct single-motor reads."""
        readings: list[int] = []
        try:
            with self._bus_lock:
                for _ in range(self._temperature_confirmation_reads):
                    readings.append(
                        int(
                            self._bus.read(
                                "Present_Temperature",
                                motor_name,
                                normalize=False,
                                num_retry=2,
                            )
                        )
                    )
        except Exception as exc:
            raise ProtectionTrip(
                f"Temperature confirmation read failed on {motor_name}: {exc}"
            ) from exc
        hot_reads = sum(value > self._max_temperature for value in readings)
        return hot_reads >= self._temperature_confirmation_min_hot_reads, readings

    def _record_rejected_temperature_spike(
        self,
        *,
        context: str,
        motor_name: str,
        sync_temperature_c: int,
        confirmation_reads_c: list[int],
    ) -> None:
        rejection = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "context": context,
            "motor": motor_name,
            "sync_temperature_c": int(sync_temperature_c),
            "confirmation_reads_c": [int(value) for value in confirmation_reads_c],
            "threshold_c": self._max_temperature,
        }
        with self._state_lock:
            self._temperature_rejected_spike_count += 1
            self._last_temperature_rejection = rejection
        self.get_logger().warning(
            "Rejected unconfirmed temperature spike: "
            f"{motor_name} sync={sync_temperature_c} C, "
            f"direct={confirmation_reads_c} C, context={context}"
        )

    def _engage_code_hold(self, reason: str) -> bool:
        """Safely seed and enable all six motors at the measured pose.

        Entering command mode is itself an explicit request for a rigid arm.  The
        goal register is therefore seeded before torque is enabled, so entering
        the mode cannot make the arm jump toward a stale goal.
        """
        motors = set(MOTOR_NAMES)
        rejected_temperature_spikes: list[tuple[str, int, list[int]]] = []
        try:
            with self._state_lock:
                if self._active:
                    raise SafetyViolation("Cannot engage code HOLD during a trajectory")
                if self._control_mode not in POWERED_CONTROL_MODES:
                    raise SafetyViolation(
                        f"Cannot engage powered HOLD in {self._control_mode} mode"
                    )
                if self._hold_state == "FAULT":
                    raise SafetyViolation(
                        "Cannot engage code HOLD while a protection fault is latched"
                    )
                self._holding = False
                self._hold_state = "TRANSITION"
                self._hold_reason = "seeding measured pose before six-axis torque enable"

            with self._bus_lock:
                positions_all = self._bus.sync_read(
                    "Present_Position", normalize=False, num_retry=2
                )
                currents_all = self._bus.sync_read(
                    "Present_Current", normalize=False, num_retry=2
                )
                temperatures_all = self._bus.sync_read(
                    "Present_Temperature", normalize=False, num_retry=2
                )
                positions = {
                    name: int(positions_all[name]) for name in MOTOR_NAMES
                }
                hot_candidates = {
                    name: int(temperatures_all[name])
                    for name in MOTOR_NAMES
                    if int(temperatures_all[name]) > self._max_temperature
                }
                hot: dict[str, dict[str, Any]] = {}
                for motor_name, sync_temperature in hot_candidates.items():
                    direct_reads = [
                        int(
                            self._bus.read(
                                "Present_Temperature",
                                motor_name,
                                normalize=False,
                                num_retry=2,
                            )
                        )
                        for _ in range(self._temperature_confirmation_reads)
                    ]
                    hot_reads = sum(
                        value > self._max_temperature for value in direct_reads
                    )
                    if hot_reads >= self._temperature_confirmation_min_hot_reads:
                        hot[motor_name] = {
                            "sync": sync_temperature,
                            "direct": direct_reads,
                        }
                    else:
                        rejected_temperature_spikes.append(
                            (motor_name, sync_temperature, direct_reads)
                        )
                overcurrent = {
                    name: int(currents_all[name])
                    for name in MOTOR_NAMES
                    if abs(int(currents_all[name])) >= self._hard_current
                }
                if hot or overcurrent:
                    raise ProtectionTrip(
                        "Powered torque enable blocked by hardware protection: "
                        f"hot={hot}, overcurrent={overcurrent}"
                    )

                self._bus.active_motors = set(motors)
                with self._state_lock:
                    self._active_motors = set(motors)
                # Goal_Position may itself enable torque on Feetech hardware.
                # Seed every motor to its measured position before explicitly
                # enabling any motor.
                self._bus.sync_write(
                    "Goal_Position", positions, normalize=False, num_retry=2
                )
                seeded = self._bus.sync_read(
                    "Goal_Position", normalize=False, num_retry=2
                )
                bad_seed = {
                    name: {
                        "present": positions[name],
                        "readback": int(seeded[name]),
                    }
                    for name in MOTOR_NAMES
                    if abs(int(seeded[name]) - positions[name]) > 1
                }
                if bad_seed:
                    raise SafetyViolation(
                        f"Powered hold seed verification failed: {bad_seed}"
                    )
                torque = self._bus.sync_read(
                    "Torque_Enable", normalize=False, num_retry=2
                )
                for motor_name in MOTOR_NAMES:
                    value = int(torque[motor_name])
                    if value == 0:
                        self._bus.enable_torque(motor_name, num_retry=2)
                    elif value != 1:
                        raise SafetyViolation(
                            f"Unexpected torque state on {motor_name}: {value}"
                        )
                torque_verified = self._bus.sync_read(
                    "Torque_Enable", normalize=False, num_retry=2
                )
                incomplete = {
                    name: int(torque_verified[name])
                    for name in MOTOR_NAMES
                    if int(torque_verified[name]) != 1
                }
                if incomplete:
                    raise SafetyViolation(
                        f"Powered six-axis torque verification failed: {incomplete}"
                    )

            for motor_name, sync_temperature, direct_reads in rejected_temperature_spikes:
                self._record_rejected_temperature_spike(
                    context=f"{self._control_mode}_ENABLE",
                    motor_name=motor_name,
                    sync_temperature_c=sync_temperature,
                    confirmation_reads_c=direct_reads,
                )

            with self._state_lock:
                self._holding = True
                self._hold_state = "HOLDING"
                self._hold_reason = reason
                self._hold_targets_raw = dict(positions)
                self._hold_started_at = datetime.now(timezone.utc).isoformat()
                self._hold_current_counts = {name: 0 for name in MOTOR_NAMES}
                self._hold_position_counts = {name: 0 for name in MOTOR_NAMES}
                self._hold_temperature_counts = {name: 0 for name in MOTOR_NAMES}
                self._active_motors = set(motors)
            self.get_logger().warning(
                "Powered HOLD engaged: all six follower torques verified ON at the measured pose"
            )
            return True
        except Exception as exc:
            cleanup_errors = self._disable_motors(motors)
            with self._state_lock:
                self._holding = False
                self._hold_state = "FAULT"
                self._hold_reason = (
                    f"Powered torque enable safety fault: {exc}"
                    + (f"; cleanup errors={cleanup_errors}" if cleanup_errors else "")
                )
                self._hold_targets_raw = {}
                self._hold_started_at = None
            self.get_logger().error(self._hold_reason)
            return False

    def _continue_hold_at_current_pose(self, reason: str) -> dict[str, int]:
        """Stop a non-safety trajectory outcome without dropping torque."""
        rejected_temperature_spikes: list[tuple[str, int, list[int]]] = []
        with self._bus_lock:
            positions_all = self._bus.sync_read(
                "Present_Position", normalize=False, num_retry=2
            )
            currents_all = self._bus.sync_read(
                "Present_Current", normalize=False, num_retry=2
            )
            temperatures_all = self._bus.sync_read(
                "Present_Temperature", normalize=False, num_retry=2
            )
            torque = self._bus.sync_read(
                "Torque_Enable", normalize=False, num_retry=2
            )
            positions = {name: int(positions_all[name]) for name in MOTOR_NAMES}
            hot_candidates = {
                name: int(temperatures_all[name])
                for name in MOTOR_NAMES
                if int(temperatures_all[name]) > self._max_temperature
            }
            hot: dict[str, dict[str, Any]] = {}
            for motor_name, sync_temperature in hot_candidates.items():
                direct_reads = [
                    int(
                        self._bus.read(
                            "Present_Temperature",
                            motor_name,
                            normalize=False,
                            num_retry=2,
                        )
                    )
                    for _ in range(self._temperature_confirmation_reads)
                ]
                hot_reads = sum(
                    value > self._max_temperature for value in direct_reads
                )
                if hot_reads >= self._temperature_confirmation_min_hot_reads:
                    hot[motor_name] = {
                        "sync": sync_temperature,
                        "direct": direct_reads,
                    }
                else:
                    rejected_temperature_spikes.append(
                        (motor_name, sync_temperature, direct_reads)
                    )
            overcurrent = {
                name: int(currents_all[name])
                for name in MOTOR_NAMES
                if abs(int(currents_all[name])) >= self._hard_current
            }
            incomplete = {
                name: int(torque[name])
                for name in MOTOR_NAMES
                if int(torque[name]) != 1
            }
            if hot or overcurrent or incomplete:
                raise ProtectionTrip(
                    "Cannot continue CODE2 HOLD after motion: "
                    f"hot={hot}, overcurrent={overcurrent}, torque={incomplete}"
                )
            self._bus.active_motors = set(MOTOR_NAMES)
            self._bus.sync_write(
                "Goal_Position", positions, normalize=False, num_retry=2
            )
            seeded = self._bus.sync_read(
                "Goal_Position", normalize=False, num_retry=2
            )
            bad_seed = {
                name: {"present": positions[name], "readback": int(seeded[name])}
                for name in MOTOR_NAMES
                if abs(int(seeded[name]) - positions[name]) > 1
            }
            if bad_seed:
                raise SafetyViolation(
                    f"Current-pose HOLD seed verification failed: {bad_seed}"
                )
        for motor_name, sync_temperature, direct_reads in rejected_temperature_spikes:
            self._record_rejected_temperature_spike(
                context="POST_MOTION_HOLD",
                motor_name=motor_name,
                sync_temperature_c=sync_temperature,
                confirmation_reads_c=direct_reads,
            )
        with self._state_lock:
            self._holding = True
            self._hold_state = "HOLDING"
            self._hold_reason = reason
            self._hold_targets_raw = dict(positions)
            self._hold_started_at = datetime.now(timezone.utc).isoformat()
            self._hold_current_counts = {name: 0 for name in MOTOR_NAMES}
            self._hold_position_counts = {name: 0 for name in MOTOR_NAMES}
            self._hold_temperature_counts = {name: 0 for name in MOTOR_NAMES}
            self._active_motors = set(MOTOR_NAMES)
        return positions

    def _record_hold_trip(
        self,
        *,
        reason: str,
        tripped_motor: str,
        currents: dict[str, int],
        temperatures: dict[str, int],
        positions: dict[str, int],
        targets: dict[str, int],
        temperature_confirmation_c: list[int] | None = None,
    ) -> dict[str, Any]:
        event = {
            "id": str(uuid.uuid4()),
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "reason": reason,
            "tripped_motor": tripped_motor,
            "current_raw": {name: int(value) for name, value in currents.items()},
            "temperature_c": {
                name: int(value) for name, value in temperatures.items()
            },
            "present_raw": {name: int(value) for name, value in positions.items()},
            "target_raw": dict(targets),
            "temperature_confirmation_c": temperature_confirmation_c,
            "near_calibrated_endpoint": False,
            "boundary_update": None,
            "context": "CODE2_HOLD",
        }
        events = self._soft_limit_state.setdefault("events", [])
        events.append(event)
        del events[:-200]
        self._last_protection_trip = event
        self._save_soft_limit_state()
        return event

    def _release_hold(
        self, reason: str, *, fault: bool = False, manual: bool = False
    ) -> list[str]:
        with self._state_lock:
            if self._active:
                return ["trajectory is active"]
            was_faulted = self._hold_state == "FAULT"
            motors = set(self._active_motors) or (
                set(MOTOR_NAMES) if self._holding else set()
            )
        errors = self._disable_motors(motors) if motors else []
        with self._state_lock:
            self._holding = False
            self._hold_state = (
                "FAULT"
                if fault or errors or was_faulted
                else "MANUAL_RELEASED"
                if manual
                else "RELEASED"
            )
            self._hold_reason = reason + (f"; cleanup errors={errors}" if errors else "")
            self._hold_targets_raw = {}
            self._hold_started_at = None
            self._hold_current_counts = {name: 0 for name in MOTOR_NAMES}
            self._hold_position_counts = {name: 0 for name in MOTOR_NAMES}
            self._hold_temperature_counts = {name: 0 for name in MOTOR_NAMES}
            self._teleop_entry_leader_norm = {}
            self._teleop_entry_follower_raw = {}
            self._teleop_last_target_raw = {}
            self._teleop_last_command_at = None
            self._teleop_clamped = {}
            self._teleop_aligning = False
            self._teleop_initial_target_raw = {}
            visual_session = getattr(self, "_visual_servo_session_id", None)
            if visual_session is not None:
                self._visual_servo_last_session_id = visual_session
            self._visual_servo_session_id = None
            self._visual_servo_state = "FAULT" if fault or errors else "INACTIVE"
            self._visual_servo_reason = reason
        return errors

    def _on_hold_command(self, message: String) -> None:
        requested = message.data.strip().lower()
        if requested == "reset_fault":
            self._reset_hold_fault()
            return
        if requested == "safe_shutdown":
            with self._state_lock:
                if self._active:
                    self.get_logger().error(
                        "Safe shutdown rejected while a trajectory is active"
                    )
                    return
                was_faulted = self._hold_state == "FAULT"
            errors = self._disable_motors(set(MOTOR_NAMES))
            with self._state_lock:
                self._holding = False
                self._hold_state = (
                    "FAULT" if errors or was_faulted else "MANUAL_RELEASED"
                )
                self._hold_reason = (
                    "safe shutdown torque cleanup failed: " + "; ".join(errors)
                    if errors
                    else "SAFE TO CLOSE TERMINALS; all six motor torque commands were disabled"
                )
                self._hold_targets_raw = {}
                self._hold_started_at = None
            if errors:
                self.get_logger().error(self._hold_reason)
            else:
                self.get_logger().warning(
                    "SAFE TO CLOSE TERMINALS: all six SO101 torques are disabled"
                )
            return
        if requested != "release":
            self.get_logger().error(f"Ignoring invalid hold command: {requested!r}")
            return
        errors = self._release_hold(
            "operator requested hold release", manual=True
        )
        if errors:
            self.get_logger().error("Hold release incomplete: " + "; ".join(errors))
        else:
            self.get_logger().warning("CODE2 hold released; torque is disabled")

    def _reset_hold_fault(self) -> None:
        """Acknowledge a latched HOLD trip without requiring a control-mode cycle."""
        with self._state_lock:
            if self._active:
                self.get_logger().error("Cannot reset HOLD fault while a trajectory is active")
                return
            if self._holding:
                self.get_logger().error("Cannot reset HOLD fault while HOLD is active")
                return
            if self._hold_state != "FAULT":
                self.get_logger().info("HOLD fault reset ignored: no fault is latched")
                return
        try:
            with self._bus_lock:
                torque = self._bus.sync_read(
                    "Torque_Enable", normalize=False, num_retry=2
                )
                current = self._bus.sync_read(
                    "Present_Current", normalize=False, num_retry=2
                )
                temperature = self._bus.sync_read(
                    "Present_Temperature", normalize=False, num_retry=2
                )
            enabled = {name: int(value) for name, value in torque.items() if int(value)}
            hot_candidates = {
                name: int(value)
                for name, value in temperature.items()
                if int(value) > self._max_temperature
            }
            hot: dict[str, dict[str, Any]] = {}
            for motor_name, sync_temperature in hot_candidates.items():
                confirmed, direct_reads = self._confirm_motor_overtemperature(
                    motor_name
                )
                if confirmed:
                    hot[motor_name] = {
                        "sync": sync_temperature,
                        "direct": direct_reads,
                    }
                else:
                    self._record_rejected_temperature_spike(
                        context="RESET_CHECK",
                        motor_name=motor_name,
                        sync_temperature_c=sync_temperature,
                        confirmation_reads_c=direct_reads,
                    )
            overcurrent = {
                name: int(value)
                for name, value in current.items()
                if abs(int(value)) >= self._hard_current
            }
            if enabled or hot or overcurrent:
                self.get_logger().error(
                    "HOLD fault reset rejected; hardware is not safe: "
                    f"torque={enabled}, hot={hot}, overcurrent={overcurrent}"
                )
                return
            with self._state_lock:
                control_mode = self._control_mode
                self._hold_state = "RELEASED"
                self._hold_reason = "protection fault acknowledged"
                self._hold_targets_raw = {}
                self._hold_started_at = None
                self._hold_current_counts = {name: 0 for name in MOTOR_NAMES}
                self._hold_position_counts = {name: 0 for name in MOTOR_NAMES}
                self._hold_temperature_counts = {name: 0 for name in MOTOR_NAMES}
            if control_mode in POWERED_CONTROL_MODES:
                self._engage_code_hold(
                    "protection fault acknowledged; maintaining current six-joint pose"
                )
                if control_mode == TELEOP_CONTROL_MODE and self._holding:
                    self._start_teleop_anchor()
            else:
                self.get_logger().warning(
                    "HOLD protection fault acknowledged; torque remains disabled outside powered modes"
                )
        except Exception as exc:
            self.get_logger().error(f"HOLD fault reset check failed: {exc}")

    def _monitor_hold(self) -> None:
        with self._state_lock:
            if not self._holding or self._active:
                return
            targets = dict(self._hold_targets_raw)
            control_mode = self._control_mode
            visual_servo_active = self._visual_servo_session_id is not None
        position_error_ticks = (
            self._visual_servo_position_error_ticks
            if visual_servo_active
            else
            self._teleop_position_error_ticks
            if control_mode == TELEOP_CONTROL_MODE
            else self._hold_position_error_ticks
        )
        position_error_samples = (
            self._visual_servo_position_error_samples
            if visual_servo_active
            else
            self._teleop_position_error_samples
            if control_mode == TELEOP_CONTROL_MODE
            else self._hold_position_error_samples_required
        )
        try:
            with self._bus_lock:
                torque_all = self._bus.sync_read(
                    "Torque_Enable", normalize=False, num_retry=2
                )
                positions_all = self._bus.sync_read(
                    "Present_Position", normalize=False, num_retry=2
                )
                currents_all = self._bus.sync_read(
                    "Present_Current", normalize=False, num_retry=2
                )
                temperatures_all = self._bus.sync_read(
                    "Present_Temperature", normalize=False, num_retry=2
                )
            positions = {name: int(positions_all[name]) for name in MOTOR_NAMES}
            currents = {name: int(currents_all[name]) for name in MOTOR_NAMES}
            temperatures = {
                name: int(temperatures_all[name]) for name in MOTOR_NAMES
            }
            with self._state_lock:
                if self._active or not self._holding:
                    return
            trip: tuple[str, str] | None = None
            temperature_confirmation: list[int] | None = None
            for motor_name in MOTOR_NAMES:
                temperature_ready, temperature_count = temperature_sample_update(
                    temperature_c=temperatures[motor_name],
                    max_temperature_c=self._max_temperature,
                    consecutive_count=self._hold_temperature_counts[motor_name],
                    samples_required=self._temperature_consecutive_samples,
                )
                self._hold_temperature_counts[motor_name] = temperature_count
                if temperature_ready:
                    confirmed, direct_reads = self._confirm_motor_overtemperature(
                        motor_name
                    )
                    if confirmed:
                        temperature_confirmation = direct_reads
                        trip = (motor_name, "HOLD_OVERTEMPERATURE")
                        break
                    self._hold_temperature_counts[motor_name] = 0
                    self._record_rejected_temperature_spike(
                        context="CODE2_HOLD",
                        motor_name=motor_name,
                        sync_temperature_c=temperatures[motor_name],
                        confirmation_reads_c=direct_reads,
                    )
                reason, current_count, position_count = hold_sample_update(
                    torque_enabled=int(torque_all[motor_name]),
                    # Temperature has a separate two-stage validation path.
                    temperature_c=min(
                        int(temperatures_all[motor_name]), self._max_temperature
                    ),
                    max_temperature_c=self._max_temperature,
                    current_raw=currents[motor_name],
                    present_raw=positions[motor_name],
                    target_raw=targets[motor_name],
                    hard_current_raw=self._hard_current,
                    sustained_current_raw=self._resistance_current[motor_name],
                    current_count=self._hold_current_counts[motor_name],
                    current_samples_required=self._hold_current_samples_required,
                    position_error_ticks=position_error_ticks,
                    position_count=self._hold_position_counts[motor_name],
                    position_samples_required=position_error_samples,
                )
                self._hold_current_counts[motor_name] = current_count
                self._hold_position_counts[motor_name] = position_count
                if reason is not None:
                    trip = (motor_name, reason)
                    break
            if trip is None:
                return
            tripped_motor, reason = trip
            cleanup_errors = self._release_hold(reason, fault=True)
            event = self._record_hold_trip(
                reason=reason,
                tripped_motor=tripped_motor,
                currents=currents,
                temperatures=temperatures,
                positions=positions,
                targets=targets,
                temperature_confirmation_c=temperature_confirmation,
            )
            self.get_logger().error(
                f"Hold protection trip {event['id']}: {reason} on {tripped_motor}, "
                f"temperature={temperatures[tripped_motor]} C, "
                f"direct confirmation={temperature_confirmation}, "
                f"current={currents[tripped_motor]} raw; "
                f"cleanup errors={cleanup_errors}"
            )
        except Exception as exc:
            self.get_logger().error(f"Hold monitor failed; releasing torque: {exc}")
            self._release_hold(f"hold monitor exception: {exc}", fault=True)

    def _visual_servo_target_raw(
        self, requested: dict[str, float]
    ) -> dict[str, int]:
        targets: dict[str, int] = {}
        for spec, motor_name in zip(self._joint_specs, self._motor_names):
            learned_min, learned_max = self._learned_bounds(motor_name)
            targets[motor_name] = urdf_radians_to_raw(
                motor_name,
                float(requested[str(spec["name"])]),
                spec,
                self._calibration,
                self._target_margin,
                learned_min,
                learned_max,
            )
        return targets

    def _on_visual_servo_command(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Visual-servo command has invalid JSON: {exc}")
            return
        command, errors = validate_visual_servo_command(payload, self._joint_specs)
        if errors:
            self.get_logger().error(
                "Visual-servo command rejected: " + "; ".join(errors)
            )
            return
        with self._visual_servo_lock:
            with self._state_lock:
                active_session = self._visual_servo_session_id
                last_sequence = self._visual_servo_sequence
                trajectory_active = self._active
                control_mode = self._control_mode
                hold_ready = self._holding and self._hold_state == "HOLDING"
                hold_targets = dict(self._hold_targets_raw)
            sequence_error = validate_visual_servo_sequence(
                op=command["op"],
                session_id=command["session_id"],
                sequence=command["sequence"],
                active_session_id=active_session,
                last_sequence=last_sequence,
            )
            if sequence_error is None and trajectory_active:
                sequence_error = "FollowJointTrajectory is active"
            if sequence_error is None and control_mode != COMMAND_CONTROL_MODE:
                sequence_error = f"control mode is {control_mode}"
            if sequence_error is None and not hold_ready:
                sequence_error = "verified six-axis HOLD is not ready"
            if sequence_error is not None:
                if active_session is not None:
                    self.get_logger().error(
                        "Visual-servo active-session command rejected; entering HOLD: "
                        + sequence_error
                    )
                    self._visual_servo_watchdog_hold(
                        "active-session command rejected: " + sequence_error
                    )
                    return
                with self._state_lock:
                    self._visual_servo_state = "REJECTED"
                    self._visual_servo_reason = sequence_error
                self.get_logger().error(
                    f"Visual-servo command rejected: {sequence_error}"
                )
                self._publish_visual_servo_state()
                return
            now = time.monotonic()
            if command["op"] in {"START", "UPDATE"}:
                try:
                    self._visual_servo_target_raw(command["positions"])
                except Exception as exc:
                    if active_session is not None:
                        self.get_logger().error(
                            "Visual-servo target conversion failed during active "
                            f"session; entering HOLD: {exc}"
                        )
                        self._visual_servo_watchdog_hold(
                            f"active target rejected: {exc}"
                        )
                        return
                    with self._state_lock:
                        self._visual_servo_state = "REJECTED"
                        self._visual_servo_reason = str(exc)
                    self._publish_visual_servo_state()
                    return
            stop_hold_targets: dict[str, int] = {}
            if command["op"] == "STOP":
                try:
                    # STOP is a safety boundary, not permission to finish the
                    # last unconsumed action tail. Seed Goal_Position from the
                    # measured pose before acknowledging HOLDING so motion
                    # cannot continue toward a now-invalid visual target.
                    stop_hold_targets = self._continue_hold_at_current_pose(
                        "visual-servo STOP; current-pose HOLD"
                    )
                except Exception as exc:
                    self.get_logger().error(
                        "Visual-servo STOP could not establish current-pose "
                        f"HOLD: {exc}"
                    )
                    self._visual_servo_watchdog_hold(
                        f"STOP current-pose HOLD failed: {exc}"
                    )
                    return
            with self._state_lock:
                if command["op"] == "START":
                    self._visual_servo_session_id = command["session_id"]
                    self._visual_servo_last_session_id = command["session_id"]
                    self._visual_servo_started = now
                    self._visual_servo_last_apply = now
                    self._visual_servo_applied_raw = dict(hold_targets)
                    self._visual_servo_update_count = 0
                    self._visual_servo_state = "ACTIVE"
                    self._visual_servo_reason = "visual-servo session started"
                    self._hold_reason = (
                        "visual-servo active; guarded targets update inside monitored HOLD"
                    )
                elif command["op"] == "STOP":
                    self._visual_servo_session_id = None
                    self._visual_servo_state = "HOLDING"
                    self._visual_servo_reason = command["reason"] or "normal stop"
                    self._hold_reason = (
                        "visual-servo stopped; current measured pose is held"
                    )
                    self._visual_servo_applied_raw = dict(stop_hold_targets)
                if command["op"] in {"START", "UPDATE"}:
                    self._visual_servo_requested_rad = dict(command["positions"])
                    self._visual_servo_last_update = now
                    self._visual_servo_update_count += 1
                    self._visual_servo_action_horizon = [
                        dict(point) for point in command["action_horizon"]
                    ]
                    self._visual_servo_horizon_step_period_sec = float(
                        command["horizon_step_period_sec"]
                    )
                    self._visual_servo_horizon_plan_id = str(
                        command["horizon_plan_id"]
                    )
                    self._visual_servo_observation_age_sec = float(
                        command["observation_age_sec"]
                    )
                    self._visual_servo_prediction_lead_sec = float(
                        command["prediction_lead_sec"]
                    )
                    if self._visual_servo_action_horizon:
                        ros_now = (
                            float(self.get_clock().now().nanoseconds) * 1.0e-9
                        )
                        duration = (
                            len(self._visual_servo_action_horizon)
                            * self._visual_servo_horizon_step_period_sec
                        )
                        transport_age = min(
                            duration,
                            max(0.0, ros_now - float(command["issued_ros_sec"])),
                        )
                        self._visual_servo_horizon_started = now - transport_age
                        self._visual_servo_horizon_index = 0
                        self._visual_servo_horizon_replan_count += 1
                    else:
                        self._visual_servo_horizon_started = None
                        self._visual_servo_horizon_index = -1
                self._visual_servo_sequence = int(command["sequence"])
        self._publish_visual_servo_state()

    def _visual_servo_watchdog_hold(self, reason: str) -> None:
        try:
            held = self._continue_hold_at_current_pose(
                "visual-servo safety stop: " + reason
            )
            state = "WATCHDOG_HOLD"
        except Exception as exc:
            self.get_logger().error(
                f"Visual-servo watchdog could not hold current pose: {exc}"
            )
            self._release_hold(
                f"VISUAL_SERVO_WATCHDOG_FAULT: {reason}; {exc}", fault=True
            )
            held = {}
            state = "FAULT"
            reason = f"{reason}; current-pose HOLD failed: {exc}"
        with self._state_lock:
            self._visual_servo_session_id = None
            self._visual_servo_state = state
            self._visual_servo_reason = reason
            if held:
                self._visual_servo_applied_raw = dict(held)
        self._publish_visual_servo_state()

    def _visual_servo_tick(self) -> None:
        with self._visual_servo_lock:
            with self._state_lock:
                session_id = self._visual_servo_session_id
                requested = dict(self._visual_servo_requested_rad)
                previous = dict(self._visual_servo_applied_raw)
                last_update = self._visual_servo_last_update
                last_apply = self._visual_servo_last_apply
                started = self._visual_servo_started
                sequence = self._visual_servo_sequence
                action_horizon = [
                    dict(point) for point in self._visual_servo_action_horizon
                ]
                horizon_step_period = self._visual_servo_horizon_step_period_sec
                horizon_started = self._visual_servo_horizon_started
                control_mode = self._control_mode
                hold_ready = self._holding and self._hold_state == "HOLDING"
            if session_id is None:
                self._publish_visual_servo_state()
                return
            now = time.monotonic()
            stop_reason = None
            if control_mode != COMMAND_CONTROL_MODE:
                stop_reason = f"control mode changed to {control_mode}"
            elif not hold_ready:
                stop_reason = "monitored HOLD became unavailable"
            elif watchdog_expired(
                last_update, self._visual_servo_watchdog_sec,
                now_monotonic=now,
            ):
                stop_reason = "command watchdog expired"
            elif started is None or now - started > self._visual_servo_max_session_sec:
                stop_reason = "maximum session duration exceeded"
            if stop_reason is not None:
                self._visual_servo_watchdog_hold(stop_reason)
                return
            try:
                horizon_index = -1
                if action_horizon and horizon_started is not None:
                    requested, horizon_index = select_action_horizon_target(
                        action_horizon,
                        elapsed_since_issue_sec=max(0.0, now - horizon_started),
                        step_period_sec=horizon_step_period,
                    )
                requested_raw = self._visual_servo_target_raw(requested)
                nominal_period = 1.0 / self._visual_servo_rate_hz
                elapsed = nominal_period if last_apply is None else now - last_apply
                elapsed = min(0.2, max(nominal_period * 0.5, elapsed))
                targets: dict[str, int] = {}
                for spec, motor_name in zip(self._joint_specs, self._motor_names):
                    raw_span = (
                        int(self._calibration[motor_name]["range_max"])
                        - int(self._calibration[motor_name]["range_min"])
                    )
                    rad_span = float(spec["upper"]) - float(spec["lower"])
                    max_step = max(1, int(math.ceil(
                        self._visual_servo_max_velocity[motor_name]
                        * elapsed * raw_span / rad_span
                    )))
                    old = int(previous[motor_name])
                    targets[motor_name] = min(
                        old + max_step,
                        max(old - max_step, int(requested_raw[motor_name])),
                    )
                with self._bus_lock:
                    self._bus.active_motors = set(MOTOR_NAMES)
                    self._bus.sync_write(
                        "Goal_Position", targets, normalize=False, num_retry=2
                    )
                _names, desired, _clipped = raw_positions_to_urdf(
                    targets, self._calibration, 0
                )
                with self._state_lock:
                    if (
                        self._visual_servo_session_id != session_id
                        or self._visual_servo_sequence != sequence
                    ):
                        return
                    self._visual_servo_applied_raw = dict(targets)
                    self._visual_servo_horizon_index = horizon_index
                    self._visual_servo_last_apply = now
                    self._hold_targets_raw = dict(targets)
                    self._desired = desired
                    self._visual_servo_state = "ACTIVE"
                    self._visual_servo_reason = (
                        "target applied under velocity and learned-limit guards"
                    )
            except Exception as exc:
                self.get_logger().error(f"Visual-servo command failed: {exc}")
                self._visual_servo_watchdog_hold(f"command failure: {exc}")
                return
        self._publish_visual_servo_state()

    def _publish_visual_servo_state(self) -> None:
        with self._state_lock:
            payload = {
                "schema_version": 1,
                "protocol": VISUAL_SERVO_PROTOCOL,
                "capable": True,
                "mode": "REAL_DASHBOARD_TEST",
                "state": self._visual_servo_state,
                "reason": self._visual_servo_reason,
                "session_id": (
                    self._visual_servo_session_id
                    or self._visual_servo_last_session_id
                ),
                "sequence_applied": self._visual_servo_sequence,
                "update_count": self._visual_servo_update_count,
                "requested_positions": dict(self._visual_servo_requested_rad),
                "applied_target_raw": dict(self._visual_servo_applied_raw),
                "watchdog_sec": self._visual_servo_watchdog_sec,
                "command_rate_hz": self._visual_servo_rate_hz,
                "max_session_sec": self._visual_servo_max_session_sec,
                "active": self._visual_servo_session_id is not None,
                "action_horizon_plan_id": self._visual_servo_horizon_plan_id,
                "action_horizon_length": len(self._visual_servo_action_horizon),
                "action_horizon_index": self._visual_servo_horizon_index,
                "action_horizon_replan_count": self._visual_servo_horizon_replan_count,
                "action_horizon_step_period_sec": self._visual_servo_horizon_step_period_sec,
                "observation_age_sec": self._visual_servo_observation_age_sec,
                "prediction_lead_sec": self._visual_servo_prediction_lead_sec,
            }
        self._visual_servo_state_pub.publish(
            String(data=json.dumps(payload, sort_keys=True))
        )

    def _on_leader_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Ignoring malformed leader status: {exc}")
            return
        if not isinstance(payload, dict):
            return
        positions = payload.get("positions_norm")
        if isinstance(positions, dict):
            try:
                normalized: dict[str, float] = {}
                if set(positions) != set(MOTOR_NAMES):
                    raise ValueError("feedback does not contain exactly six motors")
                for name in MOTOR_NAMES:
                    value = float(positions[name])
                    lower, upper = (0.0, 100.0) if name == "gripper" else (-100.0, 100.0)
                    cyclic_wrist_valid = name == "wrist_roll" and abs(value) <= 100000.0
                    if not math.isfinite(value) or not (
                        cyclic_wrist_valid or lower - 1.0 <= value <= upper + 1.0
                    ):
                        raise ValueError(f"{name} normalized value {value} is invalid")
                    normalized[name] = (
                        value if name == "wrist_roll" else min(upper, max(lower, value))
                    )
                payload["positions_norm"] = normalized
            except (TypeError, ValueError) as exc:
                payload["healthy"] = False
                payload["reason"] = f"invalid leader feedback: {exc}"
                payload.pop("positions_norm", None)
        with self._state_lock:
            self._leader_status = payload
            self._last_leader_update = time.monotonic()

    def _leader_readiness(self) -> tuple[bool, str, dict[str, Any], float | None]:
        now = time.monotonic()
        with self._state_lock:
            status = dict(self._leader_status)
            last_update = self._last_leader_update
        age = None if last_update is None else now - last_update
        if age is None:
            return False, "no leader feedback received", status, age
        if age > self._teleop_feedback_timeout_sec:
            return False, f"leader feedback is stale ({age:.3f} s)", status, age
        if not bool(status.get("connected")):
            return False, str(status.get("reason", "leader is disconnected")), status, age
        if not bool(status.get("healthy")) or not bool(status.get("torque_free")):
            return False, str(status.get("reason", "leader health is not ready")), status, age
        if set(status.get("positions_norm", {})) != set(MOTOR_NAMES):
            return False, "leader position feedback is incomplete", status, age
        configured_hash = str(
            self._config.get("leader", {}).get("calibration", {}).get("sha256", "")
        )
        if str(status.get("calibration_sha256", "")) != configured_hash:
            return False, "leader calibration identity mismatch", status, age
        return True, "fresh calibrated torque-free leader", status, age

    def _start_teleop_anchor(self) -> bool:
        ready, reason, status, _age = self._leader_readiness()
        if not ready:
            self.get_logger().error(f"Cannot start teleoperate: {reason}")
            return False
        safe_limits = {
            name: self._effective_raw_limits(name) for name in MOTOR_NAMES
        }
        absolute_targets, clamped = leader_absolute_targets_raw(
            leader_norm=status["positions_norm"],
            calibration=self._calibration,
            safe_limits=safe_limits,
        )
        with self._state_lock:
            if not self._holding or self._hold_state != "HOLDING":
                self.get_logger().error(
                    "Cannot start teleoperate without verified six-axis follower HOLD"
                )
                return False
            follower_raw = dict(self._hold_targets_raw)
            self._teleop_entry_leader_norm = {
                name: float(status["positions_norm"][name]) for name in MOTOR_NAMES
            }
            # Anchoring the relative continuation at the absolute calibrated
            # pose makes the first phase converge to Leader pose while keeping
            # later wrist-roll motion continuous across its cyclic boundary.
            self._teleop_entry_follower_raw = dict(absolute_targets)
            self._teleop_last_target_raw = follower_raw
            self._teleop_last_command_at = time.monotonic()
            self._teleop_clamped = clamped
            self._teleop_aligning = True
            self._teleop_initial_target_raw = dict(absolute_targets)
            self._hold_reason = (
                "teleoperate aligning follower to the calibrated absolute leader pose"
            )
        self.get_logger().warning(
            "TELEOPERATE ALIGNING: leader torque OFF; follower six-axis torque ON; "
            "moving from measured pose to calibrated Leader pose under velocity limits"
        )
        return True

    def _teleop_tick(self) -> None:
        with self._state_lock:
            if self._control_mode != TELEOP_CONTROL_MODE:
                return
            if self._hold_state in {"FAULT", "MANUAL_RELEASED"}:
                return
            if not self._holding or not self._teleop_entry_leader_norm:
                return
            entry_leader = dict(self._teleop_entry_leader_norm)
            entry_follower = dict(self._teleop_entry_follower_raw)
            previous_targets = dict(self._teleop_last_target_raw)
            last_command_at = self._teleop_last_command_at
            aligning = self._teleop_aligning
        ready, reason, status, _age = self._leader_readiness()
        if not ready:
            self.get_logger().error(
                f"Teleoperate safety stop: {reason}; disabling follower torque"
            )
            self._release_hold(f"TELEOP_LEADER_UNAVAILABLE: {reason}", fault=True)
            return
        try:
            safe_limits = {
                name: self._effective_raw_limits(name) for name in MOTOR_NAMES
            }
            requested, clamped = leader_relative_targets_raw(
                entry_leader_norm=entry_leader,
                current_leader_norm=status["positions_norm"],
                entry_follower_raw=entry_follower,
                calibration=self._calibration,
                safe_limits=safe_limits,
                deadband_norm=self._teleop_deadband_norm,
            )
            now = time.monotonic()
            nominal_period = 1.0 / self._teleop_command_rate_hz
            elapsed = nominal_period if last_command_at is None else now - last_command_at
            elapsed = min(0.2, max(nominal_period * 0.5, elapsed))
            velocity_limits = (
                self._teleop_alignment_max_velocity
                if aligning
                else self._teleop_follow_max_velocity
            )
            targets: dict[str, int] = {}
            for spec, motor_name in zip(self._joint_specs, self._motor_names):
                raw_span = (
                    int(self._calibration[motor_name]["range_max"])
                    - int(self._calibration[motor_name]["range_min"])
                )
                rad_span = float(spec["upper"]) - float(spec["lower"])
                max_step = max(
                    1,
                    int(
                        math.ceil(
                            velocity_limits[motor_name]
                            * elapsed
                            * raw_span
                            / rad_span
                        )
                    ),
                )
                previous = int(previous_targets[motor_name])
                targets[motor_name] = min(
                    previous + max_step,
                    max(previous - max_step, int(requested[motor_name])),
                )
            with self._bus_lock:
                self._bus.active_motors = set(MOTOR_NAMES)
                self._bus.sync_write(
                    "Goal_Position", targets, normalize=False, num_retry=2
                )
            _names, desired, _clipped = raw_positions_to_urdf(
                targets, self._calibration, 0
            )
            with self._state_lock:
                if self._control_mode != TELEOP_CONTROL_MODE or not self._holding:
                    return
                self._teleop_last_target_raw = targets
                self._teleop_last_command_at = now
                self._teleop_clamped = clamped
                self._hold_targets_raw = targets
                self._desired = desired
                if aligning:
                    present = {
                        name: int(self._raw_positions[name]) for name in MOTOR_NAMES
                    }
                    aligned = all(
                        abs(present[name] - int(requested[name]))
                        <= self._teleop_alignment_tolerance_ticks
                        for name in MOTOR_NAMES
                    )
                    if aligned:
                        self._teleop_aligning = False
                        self._hold_reason = (
                            "teleoperate active; follower aligned to calibrated Leader pose"
                        )
                        self.get_logger().warning(
                            "TELEOPERATE ACTIVE: initial absolute pose alignment completed"
                        )
        except Exception as exc:
            self.get_logger().error(
                f"Teleoperate command failed; disabling follower torque: {exc}"
            )
            self._release_hold(f"TELEOP_COMMAND_FAULT: {exc}", fault=True)

    def _on_control_mode(self, message: String) -> None:
        requested = message.data.strip()
        if requested not in CONTROL_MODES:
            self.get_logger().error(f"Ignoring invalid control mode: {requested!r}")
            return
        if requested == TELEOP_CONTROL_MODE:
            ready, reason, _status, _age = self._leader_readiness()
            if not ready:
                self.get_logger().error(
                    f"Ignoring teleoperate mode request: {reason}"
                )
                return
        release_hold = False
        engage_hold = False
        preserve_fault = False
        with self._state_lock:
            if self._active and requested == self._control_mode:
                return
            if self._active:
                self.get_logger().warning(
                    f"Ignoring control mode switch to {requested}: trajectory is active"
                )
                return
            changed = requested != self._control_mode
            self._control_mode = requested
            self._desired = list(self._positions)
            preserve_fault = self._hold_state == "FAULT"
            release_hold = (
                changed
                and requested not in POWERED_CONTROL_MODES
                and (self._holding or bool(self._active_motors))
            )
            engage_hold = (
                requested in POWERED_CONTROL_MODES
                and self._hold_state != "FAULT"
                and not self._holding
                and self._hold_state != "MANUAL_RELEASED"
            )
        if release_hold:
            self._release_hold(
                f"control mode changed to {requested}", fault=preserve_fault
            )
        elif engage_hold:
            self._engage_code_hold(
                f"{requested} selected; maintaining current six-joint pose"
            )
        if changed and requested == TELEOP_CONTROL_MODE and self._holding:
            self._start_teleop_anchor()
        elif changed and requested not in POWERED_CONTROL_MODES:
            with self._state_lock:
                if self._hold_state == "MANUAL_RELEASED":
                    self._hold_state = "RELEASED"
                    self._hold_reason = f"torque remains off in {requested} mode"
        if changed:
            self.get_logger().warning(
                f"Control mode changed to {requested}; hold state={self._hold_state}"
            )

    def _refresh_positions(self) -> tuple[dict[str, int | float], list[float]]:
        with self._bus_lock:
            raw = self._bus.sync_read("Present_Position", normalize=False, num_retry=2)
        _, positions, clipped = raw_positions_to_urdf(
            raw, self._calibration, self._recovery_outside_ticks
        )
        safe_limits = {
            motor_name: self._effective_raw_limits(motor_name)
            for motor_name in self._motor_names
        }
        recovery_joints = position_recovery_details(
            raw, self._calibration, safe_limits, self._outside_ticks
        )
        with self._state_lock:
            self._raw_positions = raw
            self._positions = positions
            self._position_recovery = {
                "required": bool(recovery_joints),
                "joints": recovery_joints,
                "feedback_clamped": {
                    name: int(value) for name, value in clipped.items()
                },
                "normal_outside_tolerance_ticks": self._outside_ticks,
                "hard_read_limit_ticks": self._recovery_outside_ticks,
            }
            self._sample_count += 1
        return raw, positions

    def _goal(self, request: FollowJointTrajectory.Goal) -> GoalResponse:
        errors = trajectory_errors(self._joint_specs, request.trajectory, self._max_duration)
        with self._state_lock:
            busy = self._active or self._visual_servo_session_id is not None
            control_mode = self._control_mode
            hold_fault = self._hold_state == "FAULT"
            hold_ready = self._holding or self._hold_state == "MANUAL_RELEASED"
        if control_mode != COMMAND_CONTROL_MODE:
            self.get_logger().warning(
                f"Rejected real trajectory: control mode is {control_mode}, not {COMMAND_CONTROL_MODE}"
            )
            return GoalResponse.REJECT
        if busy or hold_fault or not hold_ready or errors:
            self.get_logger().warning(
                "Rejected real trajectory: "
                + (
                    "busy"
                    if busy
                    else "hold protection is faulted; acknowledge it before retry"
                    if hold_fault
                    else "six-axis code HOLD is not ready"
                    if not hold_ready
                    else "; ".join(errors)
                )
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        active_motors: set[str] = set()
        starts: dict[str, int] = {}
        targets: dict[str, int] = {}
        peak_currents: dict[str, int] = {}
        final_positions: dict[str, int] = {}
        requested_duration: float | None = None
        motion_started_at = datetime.now(timezone.utc).isoformat()
        cleanup_required = False
        was_holding = False
        previous_hold_state = "RELEASED"
        try:
            with self._state_lock:
                if self._active:
                    raise SafetyViolation("Another real trajectory became active")
                if self._control_mode != COMMAND_CONTROL_MODE:
                    raise SafetyViolation(
                        f"Control mode changed to {self._control_mode}; real goal is blocked"
                    )
                if self._hold_state == "FAULT":
                    raise SafetyViolation(
                        "Hold protection is faulted; acknowledge it before retry"
                    )
                previous_hold_state = self._hold_state
                was_holding = self._holding
                self._holding = False
                self._hold_state = "TRANSITION"
                self._hold_reason = "trajectory is taking ownership of held motors"
                self._active = True
                if was_holding:
                    active_motors = set(MOTOR_NAMES)
                    cleanup_required = True

            raw_start, actual = self._refresh_positions()
            target = [float(value) for value in goal_handle.request.trajectory.points[0].positions]
            indices = changed_joint_indices(actual, target)
            active_motors = set(MOTOR_NAMES)
            starts = {motor_name: int(raw_start[motor_name]) for motor_name in active_motors}
            peak_currents = {motor_name: 0 for motor_name in active_motors}
            targets = dict(starts)
            for index in indices:
                motor_name = self._motor_names[index]
                learned_min, learned_max = self._learned_bounds(motor_name)
                targets[motor_name] = urdf_radians_to_raw(
                    motor_name,
                    target[index],
                    self._joint_specs[index],
                    self._calibration,
                    self._target_margin,
                    learned_min,
                    learned_max,
                )
            # A prior tolerance/protection stop may leave the measured pose a
            # few ticks outside the HOLD inset while still inside calibration.
            # Permit a guarded goal that brings every such motor back inside;
            # only goals that leave an out-of-inset motor outside are blocked.
            safe_limits = {
                motor_name: self._effective_raw_limits(motor_name)
                for motor_name in active_motors
            }
            recovering, recovery_required = classify_hold_inset_recovery(
                starts, targets, safe_limits
            )
            if recovery_required:
                raise SafeRecoveryRequired(
                    "Current pose is outside HOLD-safe limits; set every listed joint "
                    f"to an inward target and Preview again: {recovery_required}"
                )
            if recovering:
                self.get_logger().warning(
                    f"Allowing guarded inward recovery from HOLD inset: {recovering}"
                )
            duration_point = goal_handle.request.trajectory.points[0].time_from_start
            duration = float(duration_point.sec) + float(duration_point.nanosec) / 1e9
            requested_duration = duration
            minimum_duration = minimum_motion_duration(
                actual, target, self._motor_names, self._max_velocity
            )
            if duration + 1e-6 < minimum_duration:
                raise SafetyViolation(
                    f"Requested duration {duration:.3f}s is below guarded minimum "
                    f"{minimum_duration:.3f}s"
                )

            with self._bus_lock:
                torque = self._bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
                enabled = {name: int(value) for name, value in torque.items() if int(value)}
                if was_holding:
                    if set(enabled) != set(MOTOR_NAMES) or any(value != 1 for value in enabled.values()):
                        raise SafetyViolation(f"Hold torque state was incomplete before goal: {enabled}")
                elif enabled:
                    raise SafetyViolation(f"Torque was not clear before goal: {enabled}")
                self._bus.active_motors = set(active_motors)
                with self._state_lock:
                    self._active_motors = set(active_motors)
                cleanup_required = True
                # A Goal_Position write may auto-enable Feetech torque. Seed the
                # measured position first, with cleanup already armed.
                self._bus.sync_write(
                    "Goal_Position", starts, normalize=False, num_retry=2
                )
                seeded = self._bus.sync_read("Goal_Position", normalize=False, num_retry=2)
                bad_seed = {
                    name: {"start": starts[name], "readback": int(seeded[name])}
                    for name in active_motors
                    if abs(int(seeded[name]) - starts[name]) > 1
                }
                if bad_seed:
                    raise SafetyViolation(f"Goal seed verification failed: {bad_seed}")
                torque_after_seed = self._bus.sync_read(
                    "Torque_Enable", normalize=False, num_retry=2
                )
                for motor_name in sorted(active_motors):
                    torque_value = int(torque_after_seed[motor_name])
                    if torque_value == 0:
                        self._bus.enable_torque(motor_name, num_retry=2)
                    elif torque_value != 1:
                        raise SafetyViolation(
                            f"Unexpected torque state after seed on {motor_name}: {torque_value}"
                        )

            monitor = CurrentProtectionMonitor(
                hard_current_raw=self._hard_current,
                resistance_current_raw=self._resistance_current,
                resistance_samples=self._resistance_samples,
                following_error_ticks=self._following_error_ticks,
                max_progress_ticks=self._max_stall_progress_ticks,
                grace_sec=self._current_grace_sec,
            )
            steps = max(2, int(math.ceil(duration * self._monitor_rate_hz)))
            started = time.monotonic()
            previous_positions = dict(starts)
            motion_temperature_counts = {name: 0 for name in MOTOR_NAMES}
            for step in range(1, steps + 1):
                if goal_handle.is_cancel_requested:
                    final_positions = self._continue_hold_at_current_pose(
                        "trajectory cancelled; maintaining current six-joint pose"
                    )
                    cleanup_required = False
                    goal_handle.canceled()
                    with self._state_lock:
                        self._last_motion_report = {
                            "outcome": "CANCELLED",
                            "started_at": motion_started_at,
                            "finished_at": datetime.now(timezone.utc).isoformat(),
                            "motors": sorted(active_motors),
                            "duration_sec": requested_duration,
                            "start_raw": starts,
                            "target_raw": targets,
                            "hold_raw": final_positions,
                            "peak_current_raw": peak_currents,
                        }
                    result.error_code = result.SUCCESSFUL
                    result.error_string = (
                        "Real multi-joint goal cancelled; six-axis torque remains ON "
                        "at the current pose"
                    )
                    return result
                fraction = step / steps
                intermediate = {
                    name: int(round(starts[name] + fraction * (targets[name] - starts[name])))
                    for name in active_motors
                }
                with self._bus_lock:
                    self._bus.sync_write(
                        "Goal_Position", intermediate, normalize=False, num_retry=2
                    )
                    positions_all = self._bus.sync_read(
                        "Present_Position", normalize=False, num_retry=2
                    )
                    currents_all = self._bus.sync_read(
                        "Present_Current", normalize=False, num_retry=2
                    )
                    temperatures_all = self._bus.sync_read(
                        "Present_Temperature", normalize=False, num_retry=2
                    )
                positions = {name: int(positions_all[name]) for name in active_motors}
                currents = {name: int(currents_all[name]) for name in active_motors}
                for motor_name in active_motors:
                    peak_currents[motor_name] = max(
                        peak_currents[motor_name], abs(currents[motor_name])
                    )
                elapsed = time.monotonic() - started
                trip: tuple[str, str] | None = None
                temperature_confirmation: list[int] | None = None
                for motor_name in sorted(active_motors):
                    temperature = int(temperatures_all[motor_name])
                    temperature_ready, temperature_count = temperature_sample_update(
                        temperature_c=temperature,
                        max_temperature_c=self._max_temperature,
                        consecutive_count=motion_temperature_counts[motor_name],
                        samples_required=self._temperature_consecutive_samples,
                    )
                    motion_temperature_counts[motor_name] = temperature_count
                    if temperature_ready:
                        confirmed, direct_reads = self._confirm_motor_overtemperature(
                            motor_name
                        )
                        if confirmed:
                            temperature_confirmation = direct_reads
                            trip = (motor_name, "OVERTEMPERATURE")
                            break
                        motion_temperature_counts[motor_name] = 0
                        self._record_rejected_temperature_spike(
                            context="MOTION",
                            motor_name=motor_name,
                            sync_temperature_c=temperature,
                            confirmation_reads_c=direct_reads,
                        )
                    reason = monitor.update(
                        motor_name,
                        current_raw=currents[motor_name],
                        present_raw=positions[motor_name],
                        previous_raw=previous_positions[motor_name],
                        commanded_raw=intermediate[motor_name],
                        elapsed_sec=elapsed,
                    )
                    if reason is not None:
                        trip = (motor_name, reason)
                        break
                if trip is not None:
                    tripped_motor, reason = trip
                    stop_errors = self._disable_motors(active_motors)
                    cleanup_required = bool(stop_errors)
                    event = self._record_protection_trip(
                        reason=reason,
                        tripped_motor=tripped_motor,
                        currents=currents,
                        temperatures={
                            name: int(temperatures_all[name]) for name in active_motors
                        },
                        positions=positions,
                        start_raw=starts,
                        target_raw=targets,
                        temperature_confirmation_c=temperature_confirmation,
                    )
                    details = f"; torque cleanup errors={stop_errors}" if stop_errors else ""
                    raise ProtectionTrip(
                        f"Protection trip {event['id']}: {reason} on {tripped_motor}, "
                        f"temperature={int(temperatures_all[tripped_motor])} C, "
                        f"direct confirmation={temperature_confirmation}, "
                        f"current={currents[tripped_motor]} raw{details}"
                    )
                previous_positions = positions
                self._refresh_positions()
                with self._state_lock:
                    self._desired = list(target)
                self._publish_feedback(goal_handle)
                remaining = started + fraction * duration - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)

            settle_deadline = time.monotonic() + self._settle_timeout_sec
            settle_initial_errors: dict[str, int] | None = None
            near_goal_extension_used = False
            while True:
                with self._bus_lock:
                    final_all = self._bus.sync_read(
                        "Present_Position", normalize=False, num_retry=2
                    )
                    current_all = self._bus.sync_read(
                        "Present_Current", normalize=False, num_retry=2
                    )
                    temperature_all = self._bus.sync_read(
                        "Present_Temperature", normalize=False, num_retry=2
                    )
                final_positions = {name: int(final_all[name]) for name in active_motors}
                currents = {name: int(current_all[name]) for name in active_motors}
                for motor_name in active_motors:
                    peak_currents[motor_name] = max(
                        peak_currents[motor_name], abs(currents[motor_name])
                    )
                trip = None
                temperature_confirmation = None
                for motor_name in sorted(active_motors):
                    temperature = int(temperature_all[motor_name])
                    temperature_ready, temperature_count = temperature_sample_update(
                        temperature_c=temperature,
                        max_temperature_c=self._max_temperature,
                        consecutive_count=motion_temperature_counts[motor_name],
                        samples_required=self._temperature_consecutive_samples,
                    )
                    motion_temperature_counts[motor_name] = temperature_count
                    if temperature_ready:
                        confirmed, direct_reads = self._confirm_motor_overtemperature(
                            motor_name
                        )
                        if confirmed:
                            temperature_confirmation = direct_reads
                            trip = (motor_name, "OVERTEMPERATURE")
                            break
                        motion_temperature_counts[motor_name] = 0
                        self._record_rejected_temperature_spike(
                            context="SETTLE",
                            motor_name=motor_name,
                            sync_temperature_c=temperature,
                            confirmation_reads_c=direct_reads,
                        )
                    reason = monitor.update(
                        motor_name,
                        current_raw=currents[motor_name],
                        present_raw=final_positions[motor_name],
                        previous_raw=previous_positions[motor_name],
                        commanded_raw=targets[motor_name],
                        elapsed_sec=time.monotonic() - started,
                    )
                    if reason is not None:
                        trip = (motor_name, reason)
                        break
                if trip is not None:
                    tripped_motor, reason = trip
                    stop_errors = self._disable_motors(active_motors)
                    cleanup_required = bool(stop_errors)
                    event = self._record_protection_trip(
                        reason=reason,
                        tripped_motor=tripped_motor,
                        currents=currents,
                        temperatures={
                            name: int(temperature_all[name]) for name in active_motors
                        },
                        positions=final_positions,
                        start_raw=starts,
                        target_raw=targets,
                        temperature_confirmation_c=temperature_confirmation,
                    )
                    details = f"; torque cleanup errors={stop_errors}" if stop_errors else ""
                    raise ProtectionTrip(
                        f"Protection trip {event['id']}: {reason} on {tripped_motor}, "
                        f"temperature={int(temperature_all[tripped_motor])} C, "
                        f"direct confirmation={temperature_confirmation}, "
                        f"current={currents[tripped_motor]} raw{details}"
                    )
                errors = {
                    name: targets[name] - final_positions[name] for name in active_motors
                }
                if settle_initial_errors is None:
                    settle_initial_errors = dict(errors)
                violations = goal_tolerance_violations(
                    errors, self._goal_tolerance_ticks
                )
                if not violations:
                    break
                if time.monotonic() >= settle_deadline:
                    if (
                        not near_goal_extension_used
                        and self._near_goal_settle_extension_sec > 0.0
                        and near_goal_settle_extension_allowed(
                            violations,
                            settle_initial_errors,
                            self._hold_position_error_ticks,
                        )
                    ):
                        near_goal_extension_used = True
                        settle_deadline = (
                            time.monotonic() + self._near_goal_settle_extension_sec
                        )
                        self.get_logger().info(
                            "Endpoint remains below the HOLD disturbance threshold and "
                            f"is still converging; retaining the existing target for one "
                            f"additional {self._near_goal_settle_extension_sec:.2f}s "
                            f"settle window: {violations}"
                        )
                        previous_positions = final_positions
                        time.sleep(1.0 / self._monitor_rate_hz)
                        continue
                    raise GoalToleranceViolation(
                        "Final error exceeds per-motor HOLD-entry tolerance: "
                        f"{violations}; all errors={errors}"
                    )
                previous_positions = final_positions
                time.sleep(1.0 / self._monitor_rate_hz)

            with self._state_lock:
                self._holding = True
                self._hold_state = "HOLDING"
                self._hold_reason = "trajectory succeeded; maintaining six-joint pose"
                self._hold_targets_raw = dict(targets)
                self._hold_started_at = datetime.now(timezone.utc).isoformat()
                self._hold_current_counts = {name: 0 for name in MOTOR_NAMES}
                self._hold_position_counts = {name: 0 for name in MOTOR_NAMES}
                self._hold_temperature_counts = {name: 0 for name in MOTOR_NAMES}
                self._active_motors = set(MOTOR_NAMES)
            cleanup_required = False
            # Publish the measured terminal JointState before completing the
            # action. During execution the periodic publisher can be waiting
            # behind the servo-bus lock; without this explicit sample, tf2 may
            # still expose only the pre-motion arm/touch_tip pose when clients
            # receive the action result.
            self._publish_cached_state_best_effort("successful trajectory completion")
            goal_handle.succeed()
            with self._state_lock:
                self._last_motion_report = {
                    "outcome": "SUCCEEDED",
                    "started_at": motion_started_at,
                    "finished_at": datetime.now(timezone.utc).isoformat(),
                    "motors": sorted(active_motors),
                    "duration_sec": requested_duration,
                    "start_raw": starts,
                    "target_raw": targets,
                    "final_raw_at_hold_entry": final_positions,
                    "final_error_ticks": {
                        name: targets[name] - final_positions[name] for name in active_motors
                    },
                    "peak_current_raw": peak_currents,
                }
            result.error_code = result.SUCCESSFUL
            result.error_string = (
                f"Real multi-joint goal completed: motors={sorted(active_motors)}, "
                f"start={starts}, actual={final_positions}, target={targets}"
            )
            return result
        except Exception as exc:
            self.get_logger().error(f"Real trajectory aborted: {exc}")
            retryable = isinstance(
                exc, (GoalToleranceViolation, SafeRecoveryRequired)
            ) or (
                was_holding
                and cleanup_required
                and not isinstance(exc, ProtectionTrip)
            )
            hold_after_failure: dict[str, int] | None = None
            if retryable and cleanup_required:
                try:
                    hold_after_failure = self._continue_hold_at_current_pose(
                        "goal not reached; maintaining current pose; retry is allowed: "
                        f"{exc}"
                    )
                    cleanup_required = False
                except Exception as hold_exc:
                    exc = ProtectionTrip(
                        f"Retryable goal failure could not preserve CODE2 HOLD: {hold_exc}"
                    )
                    retryable = False
            with self._state_lock:
                if retryable and hold_after_failure is None:
                    # A validation/recovery rejection can occur before torque is
                    # touched. Preserve an operator-requested torque release.
                    self._holding = False
                    self._hold_state = previous_hold_state
                    self._hold_reason = (
                        f"goal rejected before torque change; retry is allowed: {exc}"
                    )
                elif not retryable:
                    self._holding = False
                    self._hold_state = "FAULT"
                    self._hold_reason = f"trajectory safety fault: {exc}"
                self._last_motion_report = {
                    "outcome": "ABORTED",
                    "started_at": motion_started_at,
                    "finished_at": datetime.now(timezone.utc).isoformat(),
                    "motors": sorted(active_motors),
                    "duration_sec": requested_duration,
                    "start_raw": starts,
                    "target_raw": targets,
                    "last_raw": final_positions or None,
                    "hold_raw": hold_after_failure,
                    "peak_current_raw": peak_currents,
                    "reason": str(exc),
                }
            # A retryable motion can also end while the periodic JointState
            # timer is delayed. Make its measured HOLD pose visible before the
            # action result so Cartesian recovery starts from reality.
            self._publish_cached_state_best_effort("aborted trajectory completion")
            goal_handle.abort()
            result.error_code = result.PATH_TOLERANCE_VIOLATED
            result.error_string = str(exc)
            return result
        finally:
            if cleanup_required and active_motors and self._bus.is_connected:
                cleanup_errors = self._disable_motors(active_motors)
                if cleanup_errors:
                    self.get_logger().error(
                        "Torque cleanup failed after real trajectory: " + "; ".join(cleanup_errors)
                    )
                    with self._state_lock:
                        self._holding = False
                        self._hold_state = "FAULT"
                        self._hold_reason = (
                            "torque cleanup failed after trajectory: "
                            + "; ".join(cleanup_errors)
                        )
            with self._state_lock:
                self._active = False
            try:
                self._refresh_positions()
            except Exception as exc:
                self.get_logger().error(f"Post-goal read failed: {exc}")

    def _publish_feedback(self, goal_handle) -> None:
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(self._joint_names)
        with self._state_lock:
            actual = list(self._positions)
            desired = list(self._desired)
        feedback.actual.positions = actual
        feedback.desired.positions = desired
        feedback.error.positions = [want - have for want, have in zip(desired, actual)]
        goal_handle.publish_feedback(feedback)

    def _publish_cached_state(self) -> None:
        """Publish the latest measured state without another servo-bus read."""
        with self._state_lock:
            positions = list(self._positions)
            desired = list(self._desired)
        stamp = self.get_clock().now().to_msg()
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = list(self._joint_names)
        joint.position = positions
        self._joint_pub.publish(joint)
        controller = JointTrajectoryControllerState()
        controller.header.stamp = stamp
        controller.joint_names = list(self._joint_names)
        controller.actual.positions = positions
        controller.desired.positions = desired
        controller.error.positions = [
            want - have for want, have in zip(desired, positions)
        ]
        self._controller_pub.publish(controller)

    def _publish_cached_state_best_effort(self, context: str) -> None:
        """Expose terminal feedback without changing the action outcome."""
        try:
            self._publish_cached_state()
        except Exception as exc:
            self.get_logger().error(
                f"Terminal feedback publish failed after {context}: {exc}"
            )

    def _publish_state(self) -> None:
        try:
            self._refresh_positions()
            self._publish_cached_state()
        except Exception as exc:
            self.get_logger().error(f"Real feedback publish failed: {exc}")

    def _publish_health(self) -> None:
        try:
            with self._bus_lock:
                torque = self._bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
                voltage = self._bus.sync_read("Present_Voltage", normalize=False, num_retry=2)
                temperature = self._bus.sync_read("Present_Temperature", normalize=False, num_retry=2)
                current = self._bus.sync_read("Present_Current", normalize=False, num_retry=2)
            with self._state_lock:
                active = self._active
                active_motors = set(self._active_motors)
                holding = self._holding
                hold_state = self._hold_state
                hold_reason = self._hold_reason
                hold_targets = dict(self._hold_targets_raw)
                hold_started_at = self._hold_started_at
                control_mode = self._control_mode
                samples = self._sample_count
                raw = dict(self._raw_positions)
                recovery = dict(self._position_recovery)
                temperature_rejected_spike_count = (
                    self._temperature_rejected_spike_count
                )
                last_temperature_rejection = self._last_temperature_rejection
                teleop_targets = dict(self._teleop_last_target_raw)
                teleop_clamped = dict(self._teleop_clamped)
                teleop_aligning = self._teleop_aligning
                teleop_initial_target = dict(self._teleop_initial_target_raw)
                visual_servo_health = {
                    "protocol": VISUAL_SERVO_PROTOCOL,
                    "capable": True,
                    "state": self._visual_servo_state,
                    "reason": self._visual_servo_reason,
                    "active": self._visual_servo_session_id is not None,
                    "session_id": (
                        self._visual_servo_session_id
                        or self._visual_servo_last_session_id
                    ),
                    "sequence_applied": self._visual_servo_sequence,
                    "update_count": self._visual_servo_update_count,
                    "watchdog_sec": self._visual_servo_watchdog_sec,
                    "command_rate_hz": self._visual_servo_rate_hz,
                }
            allowed_torque = active_motors if active or holding else set()
            unexpected = {
                name: int(value)
                for name, value in torque.items()
                if int(value) and name not in allowed_torque
            }
            powered_torque_required = (
                control_mode in POWERED_CONTROL_MODES
                and hold_state not in {"FAULT", "MANUAL_RELEASED"}
            )
            required_torque_missing = {
                name: int(torque[name])
                for name in MOTOR_NAMES
                if powered_torque_required and int(torque[name]) != 1
            }
            leader_ready, leader_reason, leader_status, leader_age = (
                self._leader_readiness()
            )
            payload = {
                "mode": "REAL_DASHBOARD_TEST",
                "control_mode": control_mode,
                "healthy": (
                    not unexpected
                    and not required_torque_missing
                    and hold_state != "FAULT"
                    and not recovery.get("required", False)
                    and (control_mode != TELEOP_CONTROL_MODE or leader_ready)
                ),
                "writes_guarded": True,
                "active_motor": next(iter(active_motors)) if len(active_motors) == 1 else None,
                "active_motors": sorted(active_motors),
                "unexpected_torque": unexpected,
                "code_torque_required": powered_torque_required,
                "powered_torque_required": powered_torque_required,
                "required_torque_missing": required_torque_missing,
                "torque_enable": torque,
                "raw_position": raw,
                "voltage_deci_v": voltage,
                "temperature_c": temperature,
                "current_raw": current,
                "samples": samples,
                "recovery": recovery,
                "calibration_sha256": EXPECTED_CALIBRATION_SHA256,
                "hold": {
                    "state": hold_state,
                    "holding": holding,
                    "reason": hold_reason,
                    "motors": sorted(active_motors) if holding else [],
                    "target_raw": hold_targets,
                    "started_at": hold_started_at,
                    "monitor_rate_hz": self._hold_monitor_rate_hz,
                    "current_trip_samples": self._hold_current_samples_required,
                    "position_error_ticks": self._hold_position_error_ticks,
                    "teleop_position_error_ticks": self._teleop_position_error_ticks,
                    "teleop_position_error_samples": self._teleop_position_error_samples,
                    "temperature_consecutive_counts": dict(
                        self._hold_temperature_counts
                    ),
                    "goal_acceptance_tolerance_ticks": self._goal_tolerance_ticks,
                },
                "visual_servo": visual_servo_health,
                "teleoperation": {
                    "ready": leader_ready,
                    "active": bool(
                        control_mode == TELEOP_CONTROL_MODE
                        and holding
                        and hold_state == "HOLDING"
                        and leader_ready
                        and not teleop_aligning
                    ),
                    "aligning": bool(
                        control_mode == TELEOP_CONTROL_MODE
                        and holding
                        and hold_state == "HOLDING"
                        and leader_ready
                        and teleop_aligning
                    ),
                    "phase": (
                        "ALIGNING" if teleop_aligning else "ACTIVE"
                    ) if control_mode == TELEOP_CONTROL_MODE and holding else "INACTIVE",
                    "reason": leader_reason,
                    "mapping": "absolute_align_then_follow",
                    "feedback_age_sec": leader_age,
                    "leader": {
                        "connected": bool(leader_status.get("connected")),
                        "healthy": bool(leader_status.get("healthy")),
                        "torque_free": bool(leader_status.get("torque_free")),
                        "state": leader_status.get("state", "UNKNOWN"),
                        "serial": leader_status.get("serial"),
                        "calibration_id": leader_status.get("calibration_id"),
                        "positions_norm": leader_status.get("positions_norm", {}),
                        "voltage_deci_v": leader_status.get("voltage_deci_v", {}),
                        "temperature_c": leader_status.get("temperature_c", {}),
                    },
                    "follower_target_raw": teleop_targets,
                    "initial_alignment_target_raw": teleop_initial_target,
                    "clamped_to_safe_limits": teleop_clamped,
                    "command_rate_hz": self._teleop_command_rate_hz,
                    "feedback_timeout_sec": self._teleop_feedback_timeout_sec,
                    "alignment_tolerance_ticks": self._teleop_alignment_tolerance_ticks,
                    "alignment_max_joint_velocity_rad_s": (
                        self._teleop_alignment_max_velocity
                    ),
                    "follow_max_joint_velocity_rad_s": self._teleop_follow_max_velocity,
                    "active_max_joint_velocity_rad_s": (
                        self._teleop_alignment_max_velocity
                        if teleop_aligning
                        else self._teleop_follow_max_velocity
                    ),
                },
                "protection": {
                    "armed": True,
                    "monitor_rate_hz": self._monitor_rate_hz,
                    "hard_current_raw": self._hard_current,
                    "resistance_current_raw": self._resistance_current,
                    "max_temperature_c": self._max_temperature,
                    "temperature_warning_c": max(0, self._max_temperature - 5),
                    "temperature_consecutive_samples": (
                        self._temperature_consecutive_samples
                    ),
                    "temperature_confirmation_reads": (
                        self._temperature_confirmation_reads
                    ),
                    "temperature_confirmation_min_hot_reads": (
                        self._temperature_confirmation_min_hot_reads
                    ),
                    "temperature_rejected_spike_count": (
                        temperature_rejected_spike_count
                    ),
                    "last_temperature_rejection": last_temperature_rejection,
                    "voltage_warning_min_deci_v": self._voltage_warning_min,
                    "voltage_warning_max_deci_v": self._voltage_warning_max,
                    "resistance_samples": self._resistance_samples,
                    "command_limits_rad": self._command_limits_rad(),
                    "soft_limit_state_file": str(self._soft_limit_path),
                    "last_trip": self._last_protection_trip,
                    "last_motion": self._last_motion_report,
                },
            }
            self._health_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        except Exception as exc:
            self.get_logger().error(f"Real health publish failed: {exc}")

    def close(self) -> None:
        if self._connected and self._bus.is_connected:
            cleanup_errors = self._disable_motors(set(MOTOR_NAMES))
            if cleanup_errors:
                self.get_logger().error(
                    "Shutdown torque cleanup failed: " + "; ".join(cleanup_errors)
                )
            with self._bus_lock:
                self._bus.disconnect(disable_torque=False)
                self._connected = False
        if not self._serial_owner_lock.closed:
            fcntl.flock(self._serial_owner_lock.fileno(), fcntl.LOCK_UN)
            self._serial_owner_lock.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealTrajectoryServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

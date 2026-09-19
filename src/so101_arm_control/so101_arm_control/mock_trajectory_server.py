from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from .visual_servo import (
    PROTOCOL as VISUAL_SERVO_PROTOCOL,
    select_action_horizon_target,
    validate_command as validate_visual_servo_command,
    validate_sequence as validate_visual_servo_sequence,
    velocity_limited_positions,
    watchdog_expired,
)


def trajectory_errors(joint_specs: list[dict], trajectory) -> list[str]:
    expected = [str(spec["name"]) for spec in joint_specs]
    if list(trajectory.joint_names) != expected:
        return ["joint_names must exactly match configured order"]
    if not trajectory.points:
        return ["trajectory requires at least one point"]
    errors: list[str] = []
    previous_time = 0.0
    for index, point in enumerate(trajectory.points):
        if len(point.positions) != len(expected):
            errors.append(f"point {index}: expected {len(expected)} positions")
            continue
        stamp = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1e9
        if stamp <= previous_time:
            errors.append(f"point {index}: time_from_start must increase and be positive")
        previous_time = stamp
        for spec, value in zip(joint_specs, point.positions):
            lower = float(spec["lower"])
            upper = float(spec["upper"])
            if not math.isfinite(value) or value < lower or value > upper:
                errors.append(f"point {index} {spec['name']}: outside [{lower}, {upper}]")
    if previous_time > 30.0:
        errors.append("trajectory duration exceeds 30 seconds")
    return errors


class MockTrajectoryServer(Node):
    """Deterministic mock arm implementing the standard trajectory action.

    This exists because the installed Humble controller-manager/JTC packages do
    not forward read-only controller parameters correctly. It contains no USB,
    Feetech, torque, or real-hardware code.
    """

    def __init__(self) -> None:
        super().__init__("so101_mock_trajectory_server")
        default_config = Path(get_package_share_directory("so101_arm_control")) / "config" / "arm_mock.yaml"
        self.declare_parameter("config_file", str(default_config))
        config_path = Path(str(self.get_parameter("config_file").value))
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        if config.get("mode") != "MOCK" or config.get("hardware", {}).get("real_hardware_enabled"):
            raise RuntimeError("mock trajectory server refuses non-MOCK configuration")
        self._joint_specs = list(config["joints"])
        self._names = [str(spec["name"]) for spec in self._joint_specs]
        self._positions = [0.85 if name == "gripper_joint" else 0.0 for name in self._names]
        self._velocities = [0.0] * len(self._names)
        self._desired = list(self._positions)
        self._lock = threading.RLock()
        self._active = False
        visual_servo = config["visual_servo"]
        self._visual_servo_rate_hz = float(visual_servo["command_rate_hz"])
        self._visual_servo_watchdog_sec = float(visual_servo["watchdog_sec"])
        self._visual_servo_max_session_sec = float(visual_servo["max_session_sec"])
        self._visual_servo_max_velocity = {
            str(name): float(value)
            for name, value in visual_servo["max_joint_velocity_rad_s"].items()
        }
        self._visual_servo_session_id: str | None = None
        self._visual_servo_last_session_id: str | None = None
        self._visual_servo_sequence = -1
        self._visual_servo_requested: dict[str, float] = {}
        self._visual_servo_applied: dict[str, float] = {}
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

        self._joint_pub = self.create_publisher(JointState, "/joint_states", 20)
        self._state_pub = self.create_publisher(
            JointTrajectoryControllerState, "/arm_controller/controller_state", 20
        )
        self._visual_servo_state_pub = self.create_publisher(
            String, "/arm/visual_servo_state", 20
        )
        self.create_subscription(
            String, "/arm/visual_servo_command", self._on_visual_servo_command, 50
        )
        callback_group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=callback_group,
        )
        self.create_timer(0.02, self._publish_state, callback_group=callback_group)
        self.create_timer(
            1.0 / self._visual_servo_rate_hz,
            self._visual_servo_tick,
            callback_group=callback_group,
        )
        self.get_logger().warning("standard FollowJointTrajectory server is MOCK ONLY; no USB is opened")

    def _goal(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        errors = trajectory_errors(self._joint_specs, goal_request.trajectory)
        with self._lock:
            busy = self._active or self._visual_servo_session_id is not None
        if busy:
            self.get_logger().warning("rejecting trajectory: another goal is active")
            return GoalResponse.REJECT
        if errors:
            self.get_logger().warning("rejecting trajectory: " + "; ".join(errors))
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_visual_servo_command(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self._visual_servo_state = "REJECTED"
            self._visual_servo_reason = f"invalid JSON: {exc}"
            return
        command, errors = validate_visual_servo_command(payload, self._joint_specs)
        if errors:
            self._visual_servo_state = "REJECTED"
            self._visual_servo_reason = "; ".join(errors)
            return
        with self._lock:
            sequence_error = validate_visual_servo_sequence(
                op=command["op"],
                session_id=command["session_id"],
                sequence=command["sequence"],
                active_session_id=self._visual_servo_session_id,
                last_sequence=self._visual_servo_sequence,
            )
            if self._active:
                sequence_error = "FollowJointTrajectory is active"
            if sequence_error is not None:
                if self._visual_servo_session_id is not None:
                    self._visual_servo_state = "WATCHDOG_HOLD"
                    self._visual_servo_reason = (
                        "active-session command rejected: " + sequence_error
                    )
                    self._visual_servo_session_id = None
                    self._velocities = [0.0] * len(self._names)
                else:
                    self._visual_servo_state = "REJECTED"
                    self._visual_servo_reason = sequence_error
                return
            now = time.monotonic()
            if command["op"] == "START":
                self._visual_servo_session_id = command["session_id"]
                self._visual_servo_last_session_id = command["session_id"]
                self._visual_servo_started = now
                self._visual_servo_last_apply = now
                self._visual_servo_applied = dict(zip(self._names, self._positions))
                self._visual_servo_update_count = 0
                self._visual_servo_state = "ACTIVE"
                self._visual_servo_reason = "mock visual-servo session started"
            elif command["op"] == "STOP":
                self._visual_servo_state = "HOLDING"
                self._visual_servo_reason = command["reason"] or "normal stop"
                self._visual_servo_session_id = None
                self._visual_servo_sequence = int(command["sequence"])
            if command["op"] in {"START", "UPDATE"}:
                self._visual_servo_requested = dict(command["positions"])
                self._visual_servo_last_update = now
                self._visual_servo_sequence = int(command["sequence"])
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
                    ros_now = float(self.get_clock().now().nanoseconds) * 1.0e-9
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

    def _visual_servo_tick(self) -> None:
        with self._lock:
            session_id = self._visual_servo_session_id
            if session_id is None:
                self._publish_visual_servo_state()
                return
            now = time.monotonic()
            if watchdog_expired(
                self._visual_servo_last_update,
                self._visual_servo_watchdog_sec,
                now_monotonic=now,
            ):
                self._visual_servo_state = "WATCHDOG_HOLD"
                self._visual_servo_reason = "command watchdog expired"
                self._visual_servo_session_id = None
                self._velocities = [0.0] * len(self._names)
                self._publish_visual_servo_state()
                return
            if (
                self._visual_servo_started is None
                or now - self._visual_servo_started > self._visual_servo_max_session_sec
            ):
                self._visual_servo_state = "WATCHDOG_HOLD"
                self._visual_servo_reason = "maximum session duration exceeded"
                self._visual_servo_session_id = None
                self._velocities = [0.0] * len(self._names)
                self._publish_visual_servo_state()
                return
            elapsed = max(
                1.0 / self._visual_servo_rate_hz,
                now - (self._visual_servo_last_apply or now),
            )
            requested = self._visual_servo_requested
            if (
                self._visual_servo_action_horizon
                and self._visual_servo_horizon_started is not None
            ):
                requested, horizon_index = select_action_horizon_target(
                    self._visual_servo_action_horizon,
                    elapsed_since_issue_sec=max(
                        0.0, now - self._visual_servo_horizon_started
                    ),
                    step_period_sec=self._visual_servo_horizon_step_period_sec,
                )
                self._visual_servo_horizon_index = horizon_index
            applied = velocity_limited_positions(
                self._visual_servo_applied,
                requested,
                self._joint_specs,
                self._visual_servo_max_velocity,
                elapsed,
            )
            old = dict(zip(self._names, self._positions))
            self._positions = [applied[name] for name in self._names]
            self._velocities = [
                (applied[name] - old[name]) / elapsed for name in self._names
            ]
            self._desired = [
                requested[name] for name in self._names
            ]
            self._visual_servo_applied = applied
            self._visual_servo_last_apply = now
            self._visual_servo_state = "ACTIVE"
            self._visual_servo_reason = "target applied under velocity limits"
        self._publish_visual_servo_state()

    def _publish_visual_servo_state(self) -> None:
        with self._lock:
            payload = {
                "schema_version": 1,
                "protocol": VISUAL_SERVO_PROTOCOL,
                "capable": True,
                "mode": "MOCK",
                "state": self._visual_servo_state,
                "reason": self._visual_servo_reason,
                "session_id": (
                    self._visual_servo_session_id
                    or self._visual_servo_last_session_id
                ),
                "sequence_applied": self._visual_servo_sequence,
                "update_count": self._visual_servo_update_count,
                "requested_positions": dict(self._visual_servo_requested),
                "applied_positions": dict(self._visual_servo_applied),
                "watchdog_sec": self._visual_servo_watchdog_sec,
                "command_rate_hz": self._visual_servo_rate_hz,
                "action_horizon_plan_id": self._visual_servo_horizon_plan_id,
                "action_horizon_length": len(self._visual_servo_action_horizon),
                "action_horizon_index": self._visual_servo_horizon_index,
                "action_horizon_replan_count": self._visual_servo_horizon_replan_count,
                "action_horizon_step_period_sec": self._visual_servo_horizon_step_period_sec,
                "observation_age_sec": self._visual_servo_observation_age_sec,
                "prediction_lead_sec": self._visual_servo_prediction_lead_sec,
            }
        self._visual_servo_state_pub.publish(String(data=json.dumps(payload)))

    @staticmethod
    def _cancel(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        with self._lock:
            if self._active:
                result.error_code = result.INVALID_GOAL
                result.error_string = "another mock goal became active"
                goal_handle.abort()
                return result
            self._active = True
            segment_start = list(self._positions)

        elapsed_total = 0.0
        try:
            for point in goal_handle.request.trajectory.points:
                segment_end_time = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1e9
                duration = segment_end_time - elapsed_total
                target = [float(value) for value in point.positions]
                started = time.monotonic()
                while True:
                    if goal_handle.is_cancel_requested:
                        with self._lock:
                            self._velocities = [0.0] * len(self._names)
                        goal_handle.canceled()
                        result.error_code = result.SUCCESSFUL
                        result.error_string = "mock trajectory cancelled and held"
                        return result
                    fraction = min(1.0, (time.monotonic() - started) / duration)
                    positions = [
                        start + fraction * (finish - start)
                        for start, finish in zip(segment_start, target)
                    ]
                    velocities = [
                        (finish - start) / duration if fraction < 1.0 else 0.0
                        for start, finish in zip(segment_start, target)
                    ]
                    with self._lock:
                        self._positions = positions
                        self._velocities = velocities
                        self._desired = target
                    self._publish_feedback(goal_handle)
                    if fraction >= 1.0:
                        break
                    time.sleep(0.02)
                segment_start = target
                elapsed_total = segment_end_time
            goal_handle.succeed()
            result.error_code = result.SUCCESSFUL
            result.error_string = "mock trajectory completed"
            return result
        finally:
            with self._lock:
                self._active = False
                self._velocities = [0.0] * len(self._names)

    def _publish_feedback(self, goal_handle) -> None:
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(self._names)
        with self._lock:
            actual = list(self._positions)
            velocities = list(self._velocities)
            desired = list(self._desired)
        feedback.actual.positions = actual
        feedback.actual.velocities = velocities
        feedback.desired.positions = desired
        feedback.error.positions = [want - have for want, have in zip(desired, actual)]
        goal_handle.publish_feedback(feedback)

    def _publish_state(self) -> None:
        now = self.get_clock().now().to_msg()
        with self._lock:
            positions = list(self._positions)
            velocities = list(self._velocities)
            desired = list(self._desired)

        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name = list(self._names)
        joint_state.position = positions
        joint_state.velocity = velocities
        self._joint_pub.publish(joint_state)

        controller_state = JointTrajectoryControllerState()
        controller_state.header.stamp = now
        controller_state.joint_names = list(self._names)
        controller_state.actual.positions = positions
        controller_state.actual.velocities = velocities
        controller_state.desired.positions = desired
        controller_state.error.positions = [want - have for want, have in zip(desired, positions)]
        self._state_pub.publish(controller_state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockTrajectoryServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

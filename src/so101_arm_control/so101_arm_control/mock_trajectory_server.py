from __future__ import annotations

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
from trajectory_msgs.msg import JointTrajectoryPoint


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
        self._lock = threading.Lock()
        self._active = False

        self._joint_pub = self.create_publisher(JointState, "/joint_states", 20)
        self._state_pub = self.create_publisher(
            JointTrajectoryControllerState, "/arm_controller/controller_state", 20
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
        self.get_logger().warning("standard FollowJointTrajectory server is MOCK ONLY; no USB is opened")

    def _goal(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        errors = trajectory_errors(self._joint_specs, goal_request.trajectory)
        with self._lock:
            busy = self._active
        if busy:
            self.get_logger().warning("rejecting trajectory: another goal is active")
            return GoalResponse.REJECT
        if errors:
            self.get_logger().warning("rejecting trajectory: " + "; ".join(errors))
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

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

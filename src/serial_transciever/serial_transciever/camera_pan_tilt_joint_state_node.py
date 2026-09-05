"""Publish calibrated URDF joint angles from direct Pan/Tilt feedback.

The mechanical axes and offsets live in robot.urdf.xacro.  This node only
maps the deployed OpenRB raw angle feedback to the two zero-at-reference URDF
joint angles.  It never commands either motor.
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


def calibrated_angle(raw: float, reference: float, linear: float, quadratic: float) -> float:
    delta = raw - reference
    return linear * delta + quadratic * delta * delta


class CameraPanTiltJointStateNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_pan_tilt_joint_state_node")
        self.pan_reference = float(self.declare_parameter("pan_reference_raw", 267.1).value)
        self.tilt_reference = float(self.declare_parameter("tilt_reference_raw", 101.8).value)
        self.pan_linear = float(self.declare_parameter("pan_linear_rad_per_raw", 0.0170137853).value)
        self.pan_quadratic = float(
            self.declare_parameter("pan_quadratic_rad_per_raw2", -0.00000617291927).value
        )
        self.tilt_linear = float(self.declare_parameter("tilt_linear_rad_per_raw", 0.0160077655).value)
        self.tilt_quadratic = float(
            self.declare_parameter("tilt_quadratic_rad_per_raw2", -0.00000578161438).value
        )
        self.feedback_timeout_sec = float(
            self.declare_parameter("feedback_timeout_sec", 1.0).value
        )
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 10.0).value)
        if self.feedback_timeout_sec <= 0.0 or self.publish_rate_hz <= 0.0:
            raise ValueError("feedback_timeout_sec and publish_rate_hz must be positive")

        self._pan: tuple[float, float] | None = None
        self._tilt: tuple[float, float] | None = None
        self._publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(Float32, "/chokudomotor/angle", self._on_pan, 10)
        self.create_subscription(Float32, "/cameraswingmotor/angle", self._on_tilt, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish)
        self.get_logger().info(
            "Read-only calibrated Pan/Tilt JointState publisher ready; reference raw %.1f/%.1f"
            % (self.pan_reference, self.tilt_reference)
        )

    def _on_pan(self, message: Float32) -> None:
        if math.isfinite(message.data):
            self._pan = (float(message.data), time.monotonic())

    def _on_tilt(self, message: Float32) -> None:
        if math.isfinite(message.data):
            self._tilt = (float(message.data), time.monotonic())

    def _publish(self) -> None:
        if self._pan is None or self._tilt is None:
            return
        now = time.monotonic()
        if now - self._pan[1] > self.feedback_timeout_sec:
            return
        if now - self._tilt[1] > self.feedback_timeout_sec:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["camera_pan_joint", "camera_tilt_joint"]
        message.position = [
            calibrated_angle(
                self._pan[0], self.pan_reference, self.pan_linear, self.pan_quadratic
            ),
            calibrated_angle(
                self._tilt[0], self.tilt_reference, self.tilt_linear, self.tilt_quadratic
            ),
        ]
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPanTiltJointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Preflight checks for real-robot Test-F calibration runs."""

import math
import time

import rclpy
from rclpy.node import Node

from my_messages.msg import DriveMotor, SteerMotor
from nav_msgs.msg import Odometry


class TestFPreflight(Node):
    def __init__(self):
        super().__init__("test_f_preflight")
        self.samples = {
            "/drive_odom": None,
            "/steer_odom": None,
            "/wheel_odom": None,
        }
        self.drive_pub = self.create_publisher(DriveMotor, "/drive_vel", 10)
        self.steer_pub = self.create_publisher(SteerMotor, "/steer_angle", 10)
        self.create_subscription(DriveMotor, "/drive_odom", self._drive_cb, 10)
        self.create_subscription(SteerMotor, "/steer_odom", self._steer_cb, 10)
        self.create_subscription(Odometry, "/wheel_odom", self._wheel_cb, 10)

    def _drive_cb(self, msg):
        self.samples["/drive_odom"] = (time.monotonic(), msg)

    def _steer_cb(self, msg):
        self.samples["/steer_odom"] = (time.monotonic(), msg)

    def _wheel_cb(self, msg):
        self.samples["/wheel_odom"] = (time.monotonic(), msg)

    def publish_zero_commands(self):
        now = self.get_clock().now().to_msg()
        drive = DriveMotor()
        drive.header.stamp = now
        steer = SteerMotor()
        steer.header.stamp = now
        self.drive_pub.publish(drive)
        self.steer_pub.publish(steer)

    def fresh_topics(self, max_age_sec):
        now = time.monotonic()
        missing = []
        for topic, sample in self.samples.items():
            if sample is None or now - sample[0] > max_age_sec:
                missing.append(topic)
        return missing

    def steer_error(self):
        sample = self.samples["/steer_odom"]
        if sample is None:
            return math.inf
        msg = sample[1]
        return max(abs(msg.phi1), abs(msg.phi2), abs(msg.phi3))


def main():
    rclpy.init()
    node = TestFPreflight()
    deadline = time.monotonic() + 8.0
    ok = False

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            node.publish_zero_commands()
            rclpy.spin_once(node, timeout_sec=0.05)
            missing = node.fresh_topics(max_age_sec=0.5)
            steer_error = node.steer_error()
            if not missing and steer_error <= 0.08:
                ok = True
                break
            time.sleep(0.05)

        missing = node.fresh_topics(max_age_sec=0.5)
        steer_error = node.steer_error()
        if not ok:
            node.get_logger().error(
                "Preflight failed. Missing/stale topics=%s, max steer error=%.3f rad. "
                "Check drive_motor, steer_motor_node, robot_odom_node, Dynamixel power, "
                "BLV-R power, and joystick/manual mode before running the bag."
                % (missing, steer_error)
            )
            raise SystemExit(1)

        node.get_logger().info(
            "Preflight OK: /drive_odom, /steer_odom, /wheel_odom are fresh; "
            "steer wheels are near home."
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Minimal simulated Livox IMU publisher for GLIM bringup.

This node is intentionally simple: it publishes a steady IMU message in
``livox_frame`` so GLIM can be exercised when the Isaac IMU publisher is not
enabled yet. Prefer the Isaac IMU before judging final localization quality.
"""

import rclpy
from rclpy.clock import Clock as RclpyClock
from rclpy.clock import ClockType
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import Imu


class SimLivoxImuNode(Node):
    def __init__(self):
        super().__init__("sim_livox_imu_node")
        self.declare_parameter("topic", "/livox/imu")
        self.declare_parameter("frame_id", "livox_frame")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("accel_z", 9.80665)
        self.declare_parameter("accel_z_g", 0.0)
        self.declare_parameter("use_clock_interpolation", True)
        self.declare_parameter("max_backfill_samples", 80)
        self.declare_parameter("clock_reset_threshold_sec", 0.5)
        self.declare_parameter("cmd_vel_topic", "/sim/cmd_vel")
        self.declare_parameter("livox_pitch_rad", 0.5236)

        topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.accel_z = float(self.get_parameter("accel_z").value)
        legacy_accel_z_g = float(self.get_parameter("accel_z_g").value)
        if legacy_accel_z_g > 0.0 and self.accel_z == 9.80665:
            self.accel_z = legacy_accel_z_g * 9.80665
        self.livox_pitch = float(self.get_parameter("livox_pitch_rad").value)
        self.rate = float(self.get_parameter("publish_rate").value)
        self.use_clock_interpolation = bool(
            self.get_parameter("use_clock_interpolation").value
        )
        self.max_backfill_samples = int(self.get_parameter("max_backfill_samples").value)
        self.clock_reset_threshold_ns = int(
            float(self.get_parameter("clock_reset_threshold_sec").value) * 1e9
        )
        if self.rate <= 0.0:
            raise ValueError("publish_rate must be positive")

        self.pub = self.create_publisher(Imu, topic, 10)
        self.cmd_vel = Twist()
        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self.cmd_vel_cb,
            10,
        )
        self.period_ns = int(1e9 / self.rate)
        self.last_clock_ns = None
        self.latest_clock = None
        self.latest_clock_wall_ns = None
        self.last_published_stamp_ns = None
        self.steady_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
        if self.use_clock_interpolation:
            self.create_subscription(ClockMsg, "/clock", self.clock_cb, 10)
        else:
            self.create_subscription(ClockMsg, "/clock", self.latest_clock_cb, 10)
            self.create_timer(
                1.0 / self.rate,
                self.publish_imu,
                clock=self.steady_clock,
            )
        self.get_logger().warn(
            "Publishing fake /livox/imu for sim GLIM bringup only. "
            "Use an Isaac IMU publisher before judging final localization quality."
        )

    def cmd_vel_cb(self, msg: Twist):
        self.cmd_vel = msg

    def clock_cb(self, msg: ClockMsg):
        self.latest_clock = msg.clock
        now_ns = _time_to_ns(msg.clock)
        if self.last_clock_ns is None:
            self.last_clock_ns = now_ns
            self.publish_imu(msg.clock)
            return
        if now_ns <= self.last_clock_ns:
            if self.last_clock_ns - now_ns < self.clock_reset_threshold_ns:
                return

            self.last_clock_ns = now_ns
            self.last_published_stamp_ns = None
            self.publish_imu(msg.clock)
            return

        stamps = []
        stamp_ns = self.last_clock_ns + self.period_ns
        while stamp_ns < now_ns and len(stamps) < self.max_backfill_samples:
            stamps.append(stamp_ns)
            stamp_ns += self.period_ns
        stamps.append(now_ns)

        for stamp in stamps:
            self.publish_imu(_ns_to_time(stamp))
        self.last_clock_ns = now_ns

    def latest_clock_cb(self, msg: ClockMsg):
        self.latest_clock = msg.clock
        self.latest_clock_wall_ns = self.steady_clock.now().nanoseconds

    def publish_imu(self, stamp=None):
        msg = Imu()
        if stamp is None:
            stamp = self._estimated_sim_stamp()
        stamp = self._monotonic_stamp(stamp)
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        # Orientation is unknown. Covariance[0] = -1 is the standard ROS marker
        # for "do not use this orientation estimate".
        msg.orientation_covariance[0] = -1.0

        # Approximate base yaw-rate in the pitched livox_frame. This is still a
        # bringup aid, but it avoids the worst contradiction where the robot
        # visibly turns while the IMU reports zero gyro.
        yaw_rate = float(self.cmd_vel.angular.z)
        msg.angular_velocity.x = -_sin(self.livox_pitch) * yaw_rate
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = _cos(self.livox_pitch) * yaw_rate
        msg.angular_velocity_covariance[0] = 0.05
        msg.angular_velocity_covariance[4] = 0.05
        msg.angular_velocity_covariance[8] = 0.05

        # Publish ROS-standard acceleration in m/s^2. The LiDAR/IMU frame is
        # pitched relative to base_link, so stationary gravity must be
        # projected into livox_frame.
        msg.linear_acceleration.x = -_sin(self.livox_pitch) * self.accel_z
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = _cos(self.livox_pitch) * self.accel_z
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1
        self.pub.publish(msg)

    def _estimated_sim_stamp(self):
        if self.latest_clock is None or self.latest_clock_wall_ns is None:
            return self.get_clock().now().to_msg()

        elapsed_ns = self.steady_clock.now().nanoseconds - self.latest_clock_wall_ns
        if elapsed_ns < 0:
            elapsed_ns = 0
        estimated_ns = _time_to_ns(self.latest_clock) + elapsed_ns
        if self.last_published_stamp_ns is not None:
            max_next_ns = self.last_published_stamp_ns + max(1, self.period_ns)
            estimated_ns = min(estimated_ns, max_next_ns)
        return _ns_to_time(estimated_ns)

    def _monotonic_stamp(self, stamp: Time):
        stamp_ns = _time_to_ns(stamp)
        if self.last_published_stamp_ns is not None:
            min_next_ns = self.last_published_stamp_ns + max(1, self.period_ns)
            if stamp_ns < min_next_ns:
                stamp_ns = min_next_ns
        self.last_published_stamp_ns = stamp_ns
        return _ns_to_time(stamp_ns)


def _time_to_ns(stamp: Time):
    return stamp.sec * 1000000000 + stamp.nanosec


def _ns_to_time(stamp_ns: int):
    msg = Time()
    msg.sec = int(stamp_ns // 1000000000)
    msg.nanosec = int(stamp_ns % 1000000000)
    return msg


def _sin(value):
    import math

    return math.sin(value)


def _cos(value):
    import math

    return math.cos(value)


def main(args=None):
    rclpy.init(args=args)
    node = SimLivoxImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

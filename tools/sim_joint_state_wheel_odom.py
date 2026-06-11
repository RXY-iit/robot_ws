#!/usr/bin/env python3
"""Publish /wheel_odom from Isaac Sim /joint_states.

This mirrors the real robot_odom_node kinematics, using simulated joint_states
as encoder feedback. Drive joint velocity is rad/s in Isaac, so it is converted
to wheel linear speed with the URDF wheel radius.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


WHEEL_RADIUS_M = 0.1125
WHEEL_DIST = [0.32330, 0.32330, 0.38500]
WHEEL_PHI = [-0.883978, 0.883978, math.pi]  # right, left, back

STEER_JOINTS = ["wheel_right_steer", "wheel_left_steer", "wheel_back_steer"]
DRIVE_JOINTS = ["wheel_right_drive", "wheel_left_drive", "wheel_back_drive"]


def _build_r_inv():
    rows = []
    for dist, phi in zip(WHEEL_DIST, WHEEL_PHI):
        rx = dist * math.cos(phi)
        ry = dist * math.sin(phi)
        rows.append([1.0, 0.0, -ry])
        rows.append([0.0, 1.0, rx])
    r = np.array(rows, dtype=float)
    return np.linalg.inv(r.T @ r) @ r.T


R_INV = _build_r_inv()


def _yaw_to_quat(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class SimJointStateWheelOdom(Node):
    def __init__(self):
        super().__init__("sim_joint_state_wheel_odom")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("wheel_odom_topic", "/wheel_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("wheel_radius", WHEEL_RADIUS_M)

        self._joint_topic = self.get_parameter("joint_states_topic").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._wheel_radius = float(self.get_parameter("wheel_radius").value)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_stamp = None

        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter("wheel_odom_topic").value, 10
        )
        self._tf = TransformBroadcaster(self)
        self.create_subscription(JointState, self._joint_topic, self._joint_cb, 10)
        self.get_logger().info(
            f"Publishing /wheel_odom from {self._joint_topic} "
            f"({self._odom_frame}->{self._base_frame})"
        )

    def _joint_cb(self, msg):
        by_name = {name: i for i, name in enumerate(msg.name)}
        if any(name not in by_name for name in STEER_JOINTS + DRIVE_JOINTS):
            missing = [name for name in STEER_JOINTS + DRIVE_JOINTS if name not in by_name]
            self.get_logger().warn(f"Missing joints in /joint_states: {missing}", throttle_duration_sec=2.0)
            return

        if len(msg.position) <= max(by_name[name] for name in STEER_JOINTS):
            return
        if len(msg.velocity) <= max(by_name[name] for name in DRIVE_JOINTS):
            return

        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if self._last_stamp is None:
            self._last_stamp = stamp
            return

        dt = (stamp - self._last_stamp).nanoseconds * 1e-9
        self._last_stamp = stamp
        if dt <= 0.0 or dt > 1.0:
            return

        steer_phi = [float(msg.position[by_name[name]]) for name in STEER_JOINTS]
        drive_vel = [
            float(msg.velocity[by_name[name]]) * self._wheel_radius
            for name in DRIVE_JOINTS
        ]

        v_wheel = []
        for speed, phi in zip(drive_vel, steer_phi):
            v_wheel.append(speed * math.cos(phi))
            v_wheel.append(speed * math.sin(phi))

        vx, vy, wz = (R_INV @ np.array(v_wheel, dtype=float).reshape(6, 1)).flatten()

        dx = (vx * math.cos(self._yaw) - vy * math.sin(self._yaw)) * dt
        dy = (vx * math.sin(self._yaw) + vy * math.cos(self._yaw)) * dt
        self._x += dx
        self._y += dy
        self._yaw += wz * dt

        self._publish(msg.header.stamp, float(vx), float(vy), float(wz))

    def _publish(self, stamp, vx, vy, wz):
        qx, qy, qz, qw = _yaw_to_quat(self._yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation = odom.pose.pose.orientation
        self._tf.sendTransform(tf)


def main():
    rclpy.init()
    node = SimJointStateWheelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Publish a stationary /wheel_odom for Nav2 Phase 1 logic checks.

Use only when the real robot_odom_node is not running.  This keeps Nav2's
odom subscription fresh while the robot is intentionally not allowed to move.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class FakeWheelOdom(Node):
    def __init__(self):
        super().__init__("fake_wheel_odom")
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self.pub = self.create_publisher(Odometry, "/wheel_odom", 10)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None
        self.create_timer(0.05, self._timer_cb)
        self.get_logger().info(
            f"Publishing stationary /wheel_odom {self.odom_frame}->{self.base_frame}, "
            f"publish_tf={self.publish_tf}"
        )

    def _timer_cb(self):
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.orientation.w = 1.0
        self.pub.publish(odom)

        if self.tf_pub is not None:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.rotation.w = 1.0
            self.tf_pub.sendTransform(tf)


def main():
    rclpy.init()
    node = FakeWheelOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

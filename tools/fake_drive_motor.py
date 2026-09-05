#!/usr/bin/env python3
"""Provide stationary drive feedback when Oriental Motor hardware is disabled.

This test-only node never mirrors /drive_vel and never claims physical motion.
It publishes zero three-wheel feedback so the real steering/odometry pipeline
can remain alive, plus an explicit latched marker for monitoring and controls.
"""

import rclpy
from my_messages.msg import DriveMotor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class FakeDriveMotor(Node):
    def __init__(self) -> None:
        super().__init__("fake_drive_motor")
        self._drive_pub = self.create_publisher(DriveMotor, "/drive_odom", 10)
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._mode_pub = self.create_publisher(Bool, "/drive/virtual_mode", marker_qos)
        self.create_timer(0.1, self._publish)
        self.get_logger().warning(
            "TEST MODE: Oriental Motor/Modbus is disabled; publishing stationary "
            "virtual /drive_odom only"
        )

    def _publish(self) -> None:
        message = DriveMotor()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.vel1 = 0.0
        message.vel2 = 0.0
        message.vel3 = 0.0
        self._drive_pub.publish(message)
        self._mode_pub.publish(Bool(data=True))


def main() -> None:
    rclpy.init()
    node = FakeDriveMotor()
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

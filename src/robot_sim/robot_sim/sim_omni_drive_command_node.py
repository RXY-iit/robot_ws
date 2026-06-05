#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import nan

from geometry_msgs.msg import Twist
from my_messages.msg import DriveMotor, SteerMotor
from sensor_msgs.msg import JointState

from robot_sim.omni_kinematics import cmd_vel_to_wheels, linear_to_wheel_rad_s


STEER_JOINTS = [
    'wheel_right_steer',
    'wheel_left_steer',
    'wheel_back_steer',
]

DRIVE_JOINTS = [
    'wheel_right_drive',
    'wheel_left_drive',
    'wheel_back_drive',
]


class SimOmniDriveCommandNode(Node):
    def __init__(self):
        super().__init__('sim_omni_drive_command_node')
        self.declare_parameter('cmd_vel_timeout', 1.0)
        self.declare_parameter('publish_rate', 60.0)
        self.declare_parameter('wheel_radius', 0.1125)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self._timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self._rate = float(self.get_parameter('publish_rate').value)
        self._wheel_radius = float(self.get_parameter('wheel_radius').value)
        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self._last_cmd_time = self.get_clock().now()
        self._has_cmd = False
        self._drive_vel = [0.0, 0.0, 0.0]
        self._steer_phi = [0.0, 0.0, 0.0]

        self.create_subscription(Twist, self._cmd_vel_topic, self._cmd_vel_cb, 10)
        self._joint_pub = self.create_publisher(JointState, '/sim/target_joint_states', 10)
        self._drive_pub = self.create_publisher(DriveMotor, '/sim/drive_target', 10)
        self._steer_pub = self.create_publisher(SteerMotor, '/sim/steer_target', 10)

        period = 1.0 / max(self._rate, 1.0)
        self.create_timer(period, self._publish_targets)
        self.get_logger().info(
            f'SimOmniDriveCommandNode ready: {self._cmd_vel_topic} -> '
            '/sim/target_joint_states')

    def _cmd_vel_cb(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()
        self._has_cmd = True
        self._drive_vel, self._steer_phi = cmd_vel_to_wheels(
            msg.linear.x, msg.linear.y, msg.angular.z)

    def _publish_targets(self):
        if (not self._has_cmd or
                (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9 > self._timeout):
            self._drive_vel = [0.0, 0.0, 0.0]

        stamp = self.get_clock().now().to_msg()

        joint = JointState()
        joint.header.stamp = stamp
        joint.name = STEER_JOINTS + DRIVE_JOINTS
        # Isaac's Articulation Controller treats non-NaN entries as commands.
        # Steer joints are position controlled; wheel joints are velocity
        # controlled, so unused command modes must be NaN rather than 0.0.
        joint.position = list(self._steer_phi) + [nan, nan, nan]
        joint.velocity = [nan, nan, nan] + [
            linear_to_wheel_rad_s(v, self._wheel_radius) for v in self._drive_vel
        ]
        self._joint_pub.publish(joint)

        drive = DriveMotor()
        drive.header.stamp = stamp
        drive.vel1, drive.vel2, drive.vel3 = self._drive_vel
        self._drive_pub.publish(drive)

        steer = SteerMotor()
        steer.header.stamp = stamp
        steer.phi1, steer.phi2, steer.phi3 = self._steer_phi
        self._steer_pub.publish(steer)


def main(args=None):
    rclpy.init(args=args)
    node = SimOmniDriveCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

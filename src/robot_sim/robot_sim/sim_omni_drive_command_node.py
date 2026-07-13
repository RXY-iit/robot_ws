#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import atan2, hypot, nan, pi

from geometry_msgs.msg import Twist
from my_messages.msg import DriveMotor, SteerMotor
from sensor_msgs.msg import JointState

from robot_sim.omni_kinematics import R_X, R_Y, cmd_vel_to_wheels, linear_to_wheel_rad_s


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
        self.declare_parameter('steer_angle_limit_rad', 0.71 * pi)
        self.declare_parameter('steer_rate_limit_rad_s', 1.2)
        self.declare_parameter('wheel_speed_deadband_m_s', 0.035)
        self.declare_parameter('drive_disable_steer_error_rad', 0.35)
        self.declare_parameter('steer_neutral_after_stop_sec', 0.4)

        self._timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self._rate = float(self.get_parameter('publish_rate').value)
        self._wheel_radius = float(self.get_parameter('wheel_radius').value)
        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._steer_limit = float(self.get_parameter('steer_angle_limit_rad').value)
        self._steer_rate_limit = float(self.get_parameter('steer_rate_limit_rad_s').value)
        self._wheel_speed_deadband = float(self.get_parameter('wheel_speed_deadband_m_s').value)
        self._drive_disable_steer_error = float(
            self.get_parameter('drive_disable_steer_error_rad').value)
        self._steer_neutral_after_stop = float(
            self.get_parameter('steer_neutral_after_stop_sec').value)

        self._last_cmd_time = self.get_clock().now()
        self._last_motion_cmd_time = self.get_clock().now()
        self._last_publish_time = self.get_clock().now()
        self._has_cmd = False
        self._drive_vel = [0.0, 0.0, 0.0]
        self._steer_phi = [0.0, 0.0, 0.0]
        self._target_drive_vel = [0.0, 0.0, 0.0]
        self._target_steer_phi = [0.0, 0.0, 0.0]

        self.create_subscription(Twist, self._cmd_vel_topic, self._cmd_vel_cb, 10)
        self._joint_pub = self.create_publisher(JointState, '/sim/target_joint_states', 10)
        self._drive_pub = self.create_publisher(DriveMotor, '/sim/drive_target', 10)
        self._steer_pub = self.create_publisher(SteerMotor, '/sim/steer_target', 10)

        period = 1.0 / max(self._rate, 1.0)
        self.create_timer(period, self._publish_targets)
        self.get_logger().info(
            f'SimOmniDriveCommandNode ready: {self._cmd_vel_topic} -> '
            f'/sim/target_joint_states, steer_limit={self._steer_limit:.3f} rad, '
            f'steer_rate_limit={self._steer_rate_limit:.3f} rad/s')

    def _cmd_vel_cb(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()
        self._has_cmd = True
        drive_vel, steer_phi = self._nearest_limited_wheel_targets(
            msg.linear.x, msg.linear.y, msg.angular.z)
        if max(abs(v) for v in drive_vel) < self._wheel_speed_deadband:
            self._target_drive_vel = [0.0, 0.0, 0.0]
            self._target_steer_phi = list(self._steer_phi)
            return
        self._last_motion_cmd_time = self.get_clock().now()
        self._target_drive_vel = drive_vel
        self._target_steer_phi = steer_phi

    def _publish_targets(self):
        now = self.get_clock().now()
        dt = (now - self._last_publish_time).nanoseconds * 1e-9
        self._last_publish_time = now
        dt = min(max(dt, 1.0 / max(self._rate, 1.0)), 0.1)

        if (not self._has_cmd or
                (now - self._last_cmd_time).nanoseconds * 1e-9 > self._timeout):
            self._target_drive_vel = [0.0, 0.0, 0.0]
            self._target_steer_phi = self._neutral_steer_target(now)
        elif max(abs(v) for v in self._target_drive_vel) < self._wheel_speed_deadband:
            self._target_steer_phi = self._neutral_steer_target(now)

        steer_error = [
            target - current
            for target, current in zip(self._target_steer_phi, self._steer_phi)
        ]
        max_step = max(0.0, self._steer_rate_limit * dt)
        self._steer_phi = [
            current + self._clamp(error, -max_step, max_step)
            for current, error in zip(self._steer_phi, steer_error)
        ]
        if max(abs(e) for e in steer_error) > self._drive_disable_steer_error:
            self._drive_vel = [0.0, 0.0, 0.0]
        else:
            self._drive_vel = list(self._target_drive_vel)

        stamp = now.to_msg()

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

    def _nearest_limited_wheel_targets(self, vx, vy, wz):
        drive_vel = []
        steer_phi = []
        for i, (rx, ry) in enumerate(zip(R_X, R_Y)):
            wheel_x = vx - ry * wz
            wheel_y = vy + rx * wz
            speed = hypot(wheel_x, wheel_y)
            raw_angle = atan2(wheel_y, wheel_x)
            candidates = []
            for k in range(-2, 3):
                angle = self._normalize_angle(raw_angle + k * pi)
                if abs(angle) <= self._steer_limit:
                    signed_speed = speed * (-1.0 if abs(k) % 2 else 1.0)
                    candidates.append((abs(angle - self._steer_phi[i]), signed_speed, angle))
            if candidates:
                _, signed_speed, angle = min(candidates, key=lambda item: item[0])
                drive_vel.append(signed_speed)
                steer_phi.append(angle)
            else:
                fallback_drive, fallback_steer = cmd_vel_to_wheels(vx, vy, wz)
                return fallback_drive, fallback_steer
        return drive_vel, steer_phi

    def _neutral_steer_target(self, now):
        stopped_for = (now - self._last_motion_cmd_time).nanoseconds * 1e-9
        if stopped_for >= self._steer_neutral_after_stop:
            return [0.0, 0.0, 0.0]
        return list(self._steer_phi)

    @staticmethod
    def _normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))


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

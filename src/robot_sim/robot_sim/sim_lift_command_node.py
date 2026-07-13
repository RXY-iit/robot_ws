#!/usr/bin/env python3
"""sim_lift_command_node — Sim bridge for lift + camera holder joints.

Real-robot side:
  lift_joy_node      → /lift/command              (std_msgs/String: UP/DOWN/STOP/HOME)
  camera_motor_joy   → /cameraswingmotor/target_angle (std_msgs/Float32, Dynamixel degrees)

This node subscribes to those same topics and publishes to /sim/arm_joint_states
(sensor_msgs/JointState), which the Isaac Sim arm_articulation_controller reads.

Joint mapping
  lift_joint         — PhysicsPrismaticJoint, Z axis, unit = metres
                        0.0-0.3 m travel for the future arm base
                        0.0 m = 25 cm above camera, 0.3 m = 55 cm above camera
  camera_pan_joint   — PhysicsRevoluteJoint, Z axis, unit = radians
                        0 rad = forward, ±pi/2 default yaw limit
  camera_swing_joint — PhysicsRevoluteJoint, Y axis, unit = radians
                        Dynamixel neutral maps to 42 deg downward pitch
                        default range = -90°..132°

Parameters
  publish_rate           float  Hz for /sim/arm_joint_states publish (default 30)
  lift_speed_m_s         float  Lift speed [m/s] (default 0.06 matching real AZD-KD JOG)
  lift_lower_m           float  Lower joint limit [m] (default 0.0)
  lift_upper_m           float  Upper joint limit [m] (default 0.3)
  camera_pan_neutral_deg float  Pan angle that maps to joint 0 (default 0.0)
  camera_pan_lower_rad   float  Lower pan limit [rad] (default -π/2)
  camera_pan_upper_rad   float  Upper pan limit [rad] (default +π/2)
  camera_neutral_deg     float  Dynamixel angle that maps to joint 0 (default 67.0)
  camera_down_offset_deg float  Isaac joint pitch for neutral camera angle (default 42.0)
  camera_lower_rad       float  Lower swing limit [rad] (default -90°)
  camera_upper_rad       float  Upper swing limit [rad] (default 132°)
  robot_cmd_vel_topic    str    Final velocity topic used to lock camera while moving
  robot_mode_topic       str    Mode topic used to lock camera during AUTO navigation
  lock_camera_in_auto    bool   If true, force camera to neutral whenever robot is AUTO.
  stop_linear_threshold  float  Robot is stopped below this XY speed
  stop_angular_threshold float  Robot is stopped below this yaw speed
  cmd_vel_timeout_sec    float  Treat stale cmd_vel as stopped
"""

import math
from math import nan

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String


JOINT_LIFT = 'lift_joint'
JOINT_PAN = 'camera_pan_joint'
JOINT_SWING = 'camera_swing_joint'


class SimLiftCommandNode(Node):
    def __init__(self):
        super().__init__('sim_lift_command_node')

        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('lift_speed_m_s', 0.06)
        self.declare_parameter('lift_lower_m', 0.0)
        self.declare_parameter('lift_upper_m', 0.3)
        self.declare_parameter('camera_pan_neutral_deg', 0.0)
        self.declare_parameter('camera_pan_lower_rad', -math.pi / 2)
        self.declare_parameter('camera_pan_upper_rad', math.pi / 2)
        self.declare_parameter('camera_neutral_deg', 67.0)
        self.declare_parameter('camera_down_offset_deg', 42.0)
        self.declare_parameter('camera_lower_rad', math.radians(-90.0))
        self.declare_parameter('camera_upper_rad', math.radians(132.0))
        self.declare_parameter('robot_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('robot_mode_topic', '/robot_mode')
        self.declare_parameter('lock_camera_in_auto', False)
        self.declare_parameter('stop_linear_threshold', 0.01)
        self.declare_parameter('stop_angular_threshold', 0.02)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)

        rate = float(self.get_parameter('publish_rate').value)
        self._lift_speed = float(self.get_parameter('lift_speed_m_s').value)
        self._lift_lo = float(self.get_parameter('lift_lower_m').value)
        self._lift_hi = float(self.get_parameter('lift_upper_m').value)
        self._pan_neutral = float(self.get_parameter('camera_pan_neutral_deg').value)
        self._pan_lo = float(self.get_parameter('camera_pan_lower_rad').value)
        self._pan_hi = float(self.get_parameter('camera_pan_upper_rad').value)
        self._cam_neutral = float(self.get_parameter('camera_neutral_deg').value)
        self._cam_offset = math.radians(float(self.get_parameter('camera_down_offset_deg').value))
        self._cam_lo = float(self.get_parameter('camera_lower_rad').value)
        self._cam_hi = float(self.get_parameter('camera_upper_rad').value)
        self._cmd_vel_topic = str(self.get_parameter('robot_cmd_vel_topic').value)
        self._robot_mode_topic = str(self.get_parameter('robot_mode_topic').value)
        self._lock_camera_in_auto = bool(self.get_parameter('lock_camera_in_auto').value)
        self._stop_linear_threshold = float(self.get_parameter('stop_linear_threshold').value)
        self._stop_angular_threshold = float(self.get_parameter('stop_angular_threshold').value)
        self._cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout_sec').value)

        self._lift_pos = self._lift_lo
        self._lift_cmd = 'STOP'     # UP / DOWN / STOP
        self._pan_rad = 0.0         # radians, 0 = forward
        self._swing_rad = self._cam_offset
        self._last_cmd_vel_time = None
        self._last_cmd_vel = Twist()
        self._robot_mode = 'MANUAL'

        self.create_subscription(String, '/lift/command', self._lift_cb, 10)
        self.create_subscription(Float32, '/chokudomotor/target_angle', self._pan_cb, 10)
        self.create_subscription(Float32, '/cameraswingmotor/target_angle', self._swing_cb, 10)
        self.create_subscription(Twist, self._cmd_vel_topic, self._cmd_vel_cb, 10)
        self.create_subscription(String, self._robot_mode_topic, self._robot_mode_cb, 10)

        self._pub = self.create_publisher(JointState, '/sim/arm_joint_states', 10)
        self._pan_feedback_pub = self.create_publisher(Float32, '/chokudomotor/angle', 10)
        self._swing_feedback_pub = self.create_publisher(Float32, '/cameraswingmotor/angle', 10)

        dt = 1.0 / max(rate, 1.0)
        self._dt = dt
        self.create_timer(dt, self._publish)

        self.get_logger().info(
            f'SimLiftCommandNode ready '
            f'(lift {self._lift_lo:.3f}..{self._lift_hi:.3f} m @ {self._lift_speed} m/s, '
            f'pan ±{math.degrees(self._pan_hi):.0f}°, '
            f'swing {math.degrees(self._cam_lo):.0f}..{math.degrees(self._cam_hi):.0f}°, '
            f'offset={math.degrees(self._cam_offset):.0f}°, '
            f'neutral={self._cam_neutral}°, '
            f'camera_lock_cmd_vel={self._cmd_vel_topic}, '
            f'robot_mode_topic={self._robot_mode_topic}, '
            f'lock_camera_in_auto={self._lock_camera_in_auto})'
        )

    # ---------------------------------------------------------------- callbacks

    def _lift_cb(self, msg: String):
        self._lift_cmd = msg.data.upper().strip()

    def _pan_cb(self, msg: Float32):
        if self._camera_locked():
            return
        raw_rad = (float(msg.data) - self._pan_neutral) * math.pi / 180.0
        self._pan_rad = max(self._pan_lo, min(self._pan_hi, raw_rad))

    def _swing_cb(self, msg: Float32):
        if self._camera_locked():
            return
        raw_rad = self._cam_offset + (float(msg.data) - self._cam_neutral) * math.pi / 180.0
        self._swing_rad = max(self._cam_lo, min(self._cam_hi, raw_rad))

    def _cmd_vel_cb(self, msg: Twist):
        self._last_cmd_vel = msg
        self._last_cmd_vel_time = self.get_clock().now()

    def _robot_mode_cb(self, msg: String):
        self._robot_mode = msg.data.upper().strip()

    def _camera_locked(self):
        return (self._lock_camera_in_auto and self._robot_mode == 'AUTO') or self._robot_is_moving()

    def _robot_is_moving(self):
        if self._last_cmd_vel_time is None:
            return False
        age = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds * 1e-9
        if age > self._cmd_vel_timeout:
            return False
        linear = math.hypot(self._last_cmd_vel.linear.x, self._last_cmd_vel.linear.y)
        angular = abs(self._last_cmd_vel.angular.z)
        return linear > self._stop_linear_threshold or angular > self._stop_angular_threshold

    # ---------------------------------------------------------------- publish

    def _publish(self):
        # Advance lift position according to current command
        if self._lift_cmd == 'UP':
            self._lift_pos = min(self._lift_hi, self._lift_pos + self._lift_speed * self._dt)
        elif self._lift_cmd == 'DOWN':
            self._lift_pos = max(self._lift_lo, self._lift_pos - self._lift_speed * self._dt)
        elif self._lift_cmd == 'HOME':
            self._lift_pos = self._lift_lo
            self._lift_cmd = 'STOP'

        if self._camera_locked():
            pan_rad = 0.0
            swing_rad = self._cam_offset
        else:
            pan_rad = self._pan_rad
            swing_rad = self._swing_rad

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [JOINT_LIFT, JOINT_PAN, JOINT_SWING]
        msg.position = [self._lift_pos, pan_rad, swing_rad]
        msg.velocity = [nan, nan, nan]
        msg.effort = [nan, nan, nan]
        self._pub.publish(msg)

        self._pan_feedback_pub.publish(Float32(data=self._pan_deg_from_rad(pan_rad)))
        self._swing_feedback_pub.publish(Float32(data=self._swing_deg_from_rad(swing_rad)))

    def _pan_deg_from_rad(self, pan_rad):
        return self._pan_neutral + math.degrees(pan_rad)

    def _swing_deg_from_rad(self, swing_rad):
        return self._cam_neutral + math.degrees(swing_rad - self._cam_offset)


def main(args=None):
    rclpy.init(args=args)
    node = SimLiftCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

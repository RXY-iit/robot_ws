#!/usr/bin/env python3
"""
nav_mode_switch_node.py — A/B mode switch for Nav2.

Subscribes to /joy and optional /nav_mode/command.
Pressing B forces MANUAL mode. Pressing A forces AUTO mode by default.

  MANUAL (default): /teleop/cmd_vel is relayed to /cmd_vel_raw.
  AUTO:             /nav2/cmd_vel (from Nav2 controller_server) is relayed to /cmd_vel_raw.
                    cmd_vel_safety_node clamps /cmd_vel_raw and publishes /cmd_vel.
                    Pressing B cancels the active Nav2 goal, publishes zero vel,
                    and returns to MANUAL.

Topic map:
  Sub:  /joy              (sensor_msgs/Joy)
  Sub:  /nav_mode/command (std_msgs/String: AUTO/MANUAL)
  Sub:  /teleop/cmd_vel   (geometry_msgs/Twist) — from teleop_twist_joy
  Sub:  /nav2/cmd_vel     (geometry_msgs/Twist) — from controller_server remap
  Pub:  /cmd_vel_raw      (geometry_msgs/Twist) — to cmd_vel_safety_node
  Pub:  /robot_mode       (std_msgs/String)     — "AUTO" or "MANUAL"

Joy button mapping (Linux jsX driver):
  index 0 = A  ← AUTO
  index 1 = B  ← MANUAL + cancel Nav2 goal
  index 4 = L1 (dead-man for teleop_twist_joy)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import String
from action_msgs.srv import CancelGoal
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import math


class NavModeSwitchNode(Node):
    MODE_AUTO = "AUTO"
    MODE_MANUAL = "MANUAL"

    def __init__(self):
        super().__init__('nav_mode_switch_node')

        self.mode = self.MODE_MANUAL
        self.prev_buttons = []

        self.declare_parameter('auto_button', 0)
        self.declare_parameter('manual_button', 1)
        self.declare_parameter('mode_command_topic', '/nav_mode/command')
        self.declare_parameter('enable_motion_policy', True)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('align_angle_rad', 0.75)
        self.declare_parameter('align_distance_m', 0.35)
        self.declare_parameter('finish_distance_m', 0.18)
        self.declare_parameter('finish_yaw_gate_rad', 0.45)
        self.declare_parameter('finish_max_vx', 0.08)
        self.declare_parameter('finish_max_wz', 0.25)
        self.declare_parameter('reverse_distance_m', 0.45)
        self.declare_parameter('reverse_lateral_window_m', 0.35)
        self.declare_parameter('policy_lookahead_m', 0.8)
        self.declare_parameter('lateral_scale', 0.25)
        self.declare_parameter('lateral_to_yaw_gain', 1.0)
        self.declare_parameter('min_align_wz', 0.25)
        self.declare_parameter('max_align_wz', 0.8)
        self.declare_parameter('max_reverse_vx', 0.12)
        self.declare_parameter('reverse_yaw_gain', 0.8)
        self.auto_button = int(self.get_parameter('auto_button').value)
        self.manual_button = int(self.get_parameter('manual_button').value)
        self.mode_command_topic = str(self.get_parameter('mode_command_topic').value)
        self.enable_motion_policy = self._param_bool(
            self.get_parameter('enable_motion_policy').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.plan_topic = str(self.get_parameter('plan_topic').value)
        self.align_angle = float(self.get_parameter('align_angle_rad').value)
        self.align_distance = float(self.get_parameter('align_distance_m').value)
        self.finish_distance = float(self.get_parameter('finish_distance_m').value)
        self.finish_yaw_gate = float(self.get_parameter('finish_yaw_gate_rad').value)
        self.finish_max_vx = float(self.get_parameter('finish_max_vx').value)
        self.finish_max_wz = float(self.get_parameter('finish_max_wz').value)
        self.reverse_distance = float(self.get_parameter('reverse_distance_m').value)
        self.reverse_lateral_window = float(self.get_parameter('reverse_lateral_window_m').value)
        self.policy_lookahead = float(self.get_parameter('policy_lookahead_m').value)
        self.lateral_scale = float(self.get_parameter('lateral_scale').value)
        self.lateral_to_yaw_gain = float(self.get_parameter('lateral_to_yaw_gain').value)
        self.min_align_wz = float(self.get_parameter('min_align_wz').value)
        self.max_align_wz = float(self.get_parameter('max_align_wz').value)
        self.max_reverse_vx = float(self.get_parameter('max_reverse_vx').value)
        self.reverse_yaw_gain = float(self.get_parameter('reverse_yaw_gain').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_path = None

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(String, self.mode_command_topic, self._mode_command_cb, 10)
        self.create_subscription(Twist, '/teleop/cmd_vel', self._teleop_cmd_vel_cb, 10)
        self.create_subscription(Twist, '/nav2/cmd_vel', self._nav2_cmd_vel_cb, 10)
        self.create_subscription(Path, self.plan_topic, self._plan_cb, 10)

        # Publish to /cmd_vel_raw; cmd_vel_safety_node applies watchdog/clamp → /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.policy_pub = self.create_publisher(String, '/motion_policy/status', 10)

        self._cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.create_timer(2.0, self._publish_mode)
        self.get_logger().info(
            f'NavModeSwitchNode started. Mode: {self.mode}. '
            f'auto_button={self.auto_button}, manual_button={self.manual_button}, '
            f'mode_command_topic={self.mode_command_topic}, '
            f'enable_motion_policy={self.enable_motion_policy}.')

    def _joy_cb(self, msg: Joy):
        if len(self.prev_buttons) != len(msg.buttons):
            self.prev_buttons = [0] * len(msg.buttons)

        if self.auto_button >= 0 and len(msg.buttons) > self.auto_button:
            if msg.buttons[self.auto_button] == 1 and self.prev_buttons[self.auto_button] == 0:
                self._set_auto()

        if self.manual_button >= 0 and len(msg.buttons) > self.manual_button:
            if msg.buttons[self.manual_button] == 1 and self.prev_buttons[self.manual_button] == 0:
                self._set_manual()

        self.prev_buttons = list(msg.buttons)

    def _mode_command_cb(self, msg: String):
        command = msg.data.upper().strip()
        if command == self.MODE_AUTO:
            self._set_auto()
        elif command == self.MODE_MANUAL:
            self._set_manual()
        else:
            self.get_logger().warn(
                f'Ignoring unknown {self.mode_command_topic} command: {msg.data!r}')

    def _set_auto(self):
        if self.mode != self.MODE_AUTO:
            self.mode = self.MODE_AUTO
            self.get_logger().info('Button A: MANUAL → AUTO (Nav2 cmd_vel relay active)')
        else:
            self.get_logger().info('Button A: already AUTO')
        self._publish_mode()

    def _set_manual(self):
        if self.mode != self.MODE_MANUAL:
            self.get_logger().info('Button B: AUTO → MANUAL (Nav2 goals cancelled)')
        else:
            self.get_logger().info('Button B: already MANUAL (Nav2 goals cancelled)')
        self.mode = self.MODE_MANUAL
        self._stop_robot()
        self._cancel_nav2_goals()
        self._publish_mode()

    def _stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def _cancel_nav2_goals(self):
        if self._cancel_client.service_is_ready():
            req = CancelGoal.Request()
            self._cancel_client.call_async(req)
        else:
            self.get_logger().warn('Nav2 cancel service not ready — goals may still be active')

    def _nav2_cmd_vel_cb(self, msg: Twist):
        if self.mode == self.MODE_AUTO:
            try:
                out, status = self._apply_motion_policy(msg)
            except Exception as exc:
                self.get_logger().error(
                    f'Motion policy failed; passing Nav2 cmd_vel through: {exc!r}')
                out, status = msg, 'PASS_THROUGH_POLICY_ERROR'
            self.cmd_vel_pub.publish(out)
            self._publish_policy_status(status)

    def _teleop_cmd_vel_cb(self, msg: Twist):
        if self.mode == self.MODE_MANUAL:
            self.cmd_vel_pub.publish(msg)

    def _publish_mode(self):
        out = String()
        out.data = self.mode
        self.mode_pub.publish(out)

    def _plan_cb(self, msg: Path):
        if msg.poses:
            self.latest_path = msg

    def _apply_motion_policy(self, msg: Twist):
        if not self.enable_motion_policy:
            return msg, 'PASS_THROUGH_DISABLED'

        path_target = self._path_target_in_base_frame()
        if path_target is None:
            return self._reduce_lateral_motion(msg), 'PASS_THROUGH_NO_GOAL_TF'

        target_x, target_y, target_distance, goal_x, goal_y, goal_distance, goal_yaw = path_target
        target_angle = math.atan2(target_y, target_x)

        if (
            goal_distance <= self.reverse_distance
            and goal_x < -0.05
            and abs(goal_y) <= self.reverse_lateral_window
        ):
            out = Twist()
            out.linear.x = -min(self.max_reverse_vx, max(0.03, goal_distance * 0.5))
            out.angular.z = self._clamp(-self.reverse_yaw_gain * goal_y,
                                        -self.max_align_wz, self.max_align_wz)
            return out, 'SHORT_REVERSE'

        if goal_distance <= self.finish_distance:
            out = self._reduce_lateral_motion(msg)
            out.linear.x = self._clamp(out.linear.x, -self.finish_max_vx, self.finish_max_vx)
            out.linear.y = 0.0
            if abs(goal_yaw) > self.finish_yaw_gate:
                out.linear.x = 0.0
                out.angular.z = self._clamp(0.5 * goal_yaw,
                                            -self.finish_max_wz, self.finish_max_wz)
                return out, 'FINAL_YAW_ALIGN'
            out.angular.z = self._clamp(out.angular.z, -self.finish_max_wz, self.finish_max_wz)
            return out, 'FINAL_APPROACH'

        if target_distance >= self.align_distance and abs(target_angle) >= self.align_angle:
            out = Twist()
            out.angular.z = math.copysign(
                min(self.max_align_wz, max(self.min_align_wz, abs(target_angle) * 0.8)),
                target_angle,
            )
            return out, 'ALIGN_THEN_FORWARD'

        return self._reduce_lateral_motion(msg), 'FORWARD_TRACK'

    def _path_target_in_base_frame(self):
        if self.latest_path is None or not self.latest_path.poses:
            return None

        path_frame = self.latest_path.header.frame_id or self.latest_path.poses[-1].header.frame_id or 'map'
        try:
            tf = self.tf_buffer.lookup_transform(
                path_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return None

        base = tf.transform.translation
        yaw = self._yaw_from_quaternion(tf.transform.rotation)

        nearest_i = min(
            range(len(self.latest_path.poses)),
            key=lambda i: self._distance_xy(self.latest_path.poses[i].pose.position, base),
        )
        target_i = nearest_i
        walked = 0.0
        for i in range(nearest_i + 1, len(self.latest_path.poses)):
            prev = self.latest_path.poses[i - 1].pose.position
            curr = self.latest_path.poses[i].pose.position
            walked += math.hypot(curr.x - prev.x, curr.y - prev.y)
            target_i = i
            if walked >= self.policy_lookahead:
                break

        target = self.latest_path.poses[target_i].pose.position
        goal_pose = self.latest_path.poses[-1].pose
        goal = goal_pose.position

        target_x, target_y = self._map_delta_to_base(target.x - base.x, target.y - base.y, yaw)
        goal_x, goal_y = self._map_delta_to_base(goal.x - base.x, goal.y - base.y, yaw)
        goal_yaw = self._normalize_angle(self._yaw_from_quaternion(goal_pose.orientation) - yaw)
        return (
            target_x,
            target_y,
            math.hypot(target_x, target_y),
            goal_x,
            goal_y,
            math.hypot(goal_x, goal_y),
            goal_yaw,
        )

    @staticmethod
    def _map_delta_to_base(dx, dy, yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
        )

    @staticmethod
    def _distance_xy(point, base):
        return math.hypot(point.x - base.x, point.y - base.y)

    def _reduce_lateral_motion(self, msg: Twist):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y * self.lateral_scale
        out.angular.z = msg.angular.z + msg.linear.y * self.lateral_to_yaw_gain
        return out

    def _publish_policy_status(self, status: str):
        msg = String()
        msg.data = status
        self.policy_pub.publish(msg)

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))

    @staticmethod
    def _param_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ('1', 'true', 'yes', 'on')
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    node = NavModeSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

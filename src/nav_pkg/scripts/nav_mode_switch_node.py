#!/usr/bin/env python3
"""
nav_mode_switch_node.py — A/B mode switch for Nav2.

Subscribes to /joy and monitors button index 1 (B button on Joy-Con / Xbox).
Pressing B forces MANUAL mode. Pressing A forces AUTO mode.

  MANUAL (default): /teleop/cmd_vel is relayed to /cmd_vel_raw.
  AUTO:             /nav2/cmd_vel (from Nav2 controller_server) is relayed to /cmd_vel_raw.
                    cmd_vel_safety_node clamps /cmd_vel_raw and publishes /cmd_vel.
                    Pressing B cancels the active Nav2 goal, publishes zero vel,
                    and returns to MANUAL.

Topic map:
  Sub:  /joy              (sensor_msgs/Joy)
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
from std_msgs.msg import String
from action_msgs.srv import CancelGoal


class NavModeSwitchNode(Node):
    MODE_AUTO = "AUTO"
    MODE_MANUAL = "MANUAL"

    def __init__(self):
        super().__init__('nav_mode_switch_node')

        self.mode = self.MODE_MANUAL
        self.prev_buttons = []

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(Twist, '/teleop/cmd_vel', self._teleop_cmd_vel_cb, 10)
        self.create_subscription(Twist, '/nav2/cmd_vel', self._nav2_cmd_vel_cb, 10)

        # Publish to /cmd_vel_raw; cmd_vel_safety_node applies watchdog/clamp → /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)

        self._cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.create_timer(2.0, self._publish_mode)
        self.get_logger().info(
            f'NavModeSwitchNode started. Mode: {self.mode}. '
            'Press A (Joy index 0) for AUTO, B (Joy index 1) for MANUAL.')

    def _joy_cb(self, msg: Joy):
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons)

        if len(msg.buttons) > 0:
            if msg.buttons[0] == 1 and self.prev_buttons[0] == 0:
                self._set_auto()

        if len(msg.buttons) > 1:
            if msg.buttons[1] == 1 and self.prev_buttons[1] == 0:
                self._set_manual()

        self.prev_buttons = list(msg.buttons)

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
            self.cmd_vel_pub.publish(msg)

    def _teleop_cmd_vel_cb(self, msg: Twist):
        if self.mode == self.MODE_MANUAL:
            self.cmd_vel_pub.publish(msg)

    def _publish_mode(self):
        out = String()
        out.data = self.mode
        self.mode_pub.publish(out)


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

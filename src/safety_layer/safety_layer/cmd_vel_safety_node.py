#!/usr/bin/env python3
"""
cmd_vel_safety_node.py — Phase 3 safety layer between mode-switch and motors.

Topic flow:
  /cmd_vel_raw  (nav_mode_switch_node output)
    → this node applies: watchdog + speed clamp + emergency stop
  /cmd_vel      (omni_base_driver input)

Emergency stop sources:
  /emergency_stop  (std_msgs/Bool, True = stop)
  Y button (Joy index 3)

Safety status published on /safety_status (std_msgs/String):
  "OK", "WATCHDOG", "ESTOP", "MANUAL_STOP"

Parameters (ROS):
  watchdog_timeout:  float = 0.5   seconds without input → zero output
  max_vx:            float = 0.15  m/s
  max_vy:            float = 0.10  m/s
  max_wz:            float = 0.5   rad/s
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Joy
import time


class CmdVelSafetyNode(Node):
    STATUS_OK = "OK"
    STATUS_WATCHDOG = "WATCHDOG"
    STATUS_ESTOP = "ESTOP"

    def __init__(self):
        super().__init__('cmd_vel_safety_node')

        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('max_vx', 0.15)
        self.declare_parameter('max_vy', 0.10)
        self.declare_parameter('max_wz', 0.5)

        self._timeout = self.get_parameter('watchdog_timeout').value
        self._max_vx = self.get_parameter('max_vx').value
        self._max_vy = self.get_parameter('max_vy').value
        self._max_wz = self.get_parameter('max_wz').value

        self._last_input_time = time.time()
        self._estop = False
        self._last_status = self.STATUS_OK
        self._prev_joy_y = 0  # for rising-edge detection on Y button

        self.create_subscription(Twist, '/cmd_vel_raw', self._raw_cb, 10)
        self.create_subscription(Bool, '/emergency_stop', self._estop_cb, 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._status_pub = self.create_publisher(String, '/safety_status', 10)

        # Watchdog timer: fires at 2x the timeout rate
        period = max(0.05, self._timeout / 2.0)
        self.create_timer(period, self._watchdog_tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'CmdVelSafetyNode ready. '
            f'watchdog={self._timeout}s '
            f'max vx={self._max_vx} vy={self._max_vy} wz={self._max_wz}'
        )

    def _raw_cb(self, msg: Twist):
        self._last_input_time = time.time()
        if self._estop:
            self._publish_zero()
            return
        out = Twist()
        out.linear.x = self._clamp(msg.linear.x, -self._max_vx, self._max_vx)
        out.linear.y = self._clamp(msg.linear.y, -self._max_vy, self._max_vy)
        out.angular.z = self._clamp(msg.angular.z, -self._max_wz, self._max_wz)
        self._cmd_pub.publish(out)
        self._set_status(self.STATUS_OK)

    def _estop_cb(self, msg: Bool):
        if msg.data and not self._estop:
            self.get_logger().warn('Emergency stop activated via /emergency_stop topic.')
        elif not msg.data and self._estop:
            self.get_logger().info('Emergency stop cleared via /emergency_stop topic.')
        self._estop = msg.data
        if self._estop:
            self._publish_zero()
            self._set_status(self.STATUS_ESTOP)

    def _joy_cb(self, msg: Joy):
        # Y button = index 3 → toggle ESTOP on rising edge only (prev=0, curr=1)
        curr_y = msg.buttons[3] if len(msg.buttons) > 3 else 0
        if curr_y == 1 and self._prev_joy_y == 0:
            self._estop = not self._estop
            if self._estop:
                self.get_logger().warn('Emergency stop activated via Y button.')
                self._publish_zero()
                self._set_status(self.STATUS_ESTOP)
            else:
                self.get_logger().info('Emergency stop cleared via Y button.')
        self._prev_joy_y = curr_y

    def _watchdog_tick(self):
        if self._estop:
            return
        elapsed = time.time() - self._last_input_time
        if elapsed > self._timeout:
            self._publish_zero()
            if self._last_status != self.STATUS_WATCHDOG:
                self.get_logger().warn(
                    f'Watchdog: no /cmd_vel_raw for {elapsed:.2f}s — publishing zero.')
            self._set_status(self.STATUS_WATCHDOG)

    def _publish_zero(self):
        self._cmd_pub.publish(Twist())

    def _set_status(self, s: str):
        self._last_status = s

    def _publish_status(self):
        msg = String()
        msg.data = self._last_status
        self._status_pub.publish(msg)

    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

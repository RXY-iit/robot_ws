#!/usr/bin/env python3
"""
cmd_vel_safety_node.py — Phase 3 safety layer between mode-switch and motors.

Topic flow:
  /cmd_vel_raw  (nav_mode_switch_node output)
    → this node applies: watchdog + speed clamp + slew/jerk limit + emergency stop
  /cmd_vel      (omni_base_driver input)

Emergency stop sources:
  /emergency_stop  (std_msgs/Bool, True = stop)
  Joy button (default index 3 = Y; set joy_estop_button=-1 to disable)

Safety status published on /safety_status (std_msgs/String):
  "OK", "WATCHDOG", "ESTOP", "MANUAL_STOP"

Parameters (ROS):
  watchdog_timeout:  float = 0.5   seconds without input → zero output
  max_vx:            float = 0.15  m/s
  max_vy:            float = 0.10  m/s
  max_wz:            float = 0.5   rad/s
  max_accel_vx:      float = 0.30  m/s^2
  max_accel_vy:      float = 0.20  m/s^2
  max_accel_wz:      float = 0.80  rad/s^2
  max_jerk_vx:       float = 1.00  m/s^3
  max_jerk_vy:       float = 0.60  m/s^3
  max_jerk_wz:       float = 2.00  rad/s^3
  joy_estop_button:  int   = 3     Joy button index for ESTOP toggle; -1 disables
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
        self.declare_parameter('max_accel_vx', 0.30)
        self.declare_parameter('max_accel_vy', 0.20)
        self.declare_parameter('max_accel_wz', 0.80)
        self.declare_parameter('max_jerk_vx', 1.00)
        self.declare_parameter('max_jerk_vy', 0.60)
        self.declare_parameter('max_jerk_wz', 2.00)
        self.declare_parameter('joy_estop_button', 3)

        self._timeout = self.get_parameter('watchdog_timeout').value
        self._max_vx = self.get_parameter('max_vx').value
        self._max_vy = self.get_parameter('max_vy').value
        self._max_wz = self.get_parameter('max_wz').value
        self._max_accel = {
            'vx': float(self.get_parameter('max_accel_vx').value),
            'vy': float(self.get_parameter('max_accel_vy').value),
            'wz': float(self.get_parameter('max_accel_wz').value),
        }
        self._max_jerk = {
            'vx': float(self.get_parameter('max_jerk_vx').value),
            'vy': float(self.get_parameter('max_jerk_vy').value),
            'wz': float(self.get_parameter('max_jerk_wz').value),
        }
        self._joy_estop_button = int(self.get_parameter('joy_estop_button').value)

        self._last_input_time = time.time()
        self._last_output_time = self._last_input_time
        self._last_output = Twist()
        self._last_accel = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
        self._estop = False
        self._last_status = self.STATUS_OK
        self._prev_joy_estop = 0  # for rising-edge detection

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
            f'max vx={self._max_vx} vy={self._max_vy} wz={self._max_wz} '
            f'accel vx={self._max_accel["vx"]} vy={self._max_accel["vy"]} wz={self._max_accel["wz"]} '
            f'jerk vx={self._max_jerk["vx"]} vy={self._max_jerk["vy"]} wz={self._max_jerk["wz"]} '
            f'joy_estop_button={self._joy_estop_button}'
        )

    def _raw_cb(self, msg: Twist):
        now = time.time()
        self._last_input_time = now
        if self._estop:
            self._publish_zero()
            return
        target = Twist()
        target.linear.x = self._clamp(msg.linear.x, -self._max_vx, self._max_vx)
        target.linear.y = self._clamp(msg.linear.y, -self._max_vy, self._max_vy)
        target.angular.z = self._clamp(msg.angular.z, -self._max_wz, self._max_wz)

        dt = self._dt(now)
        out = Twist()
        out.linear.x = self._slew_component('vx', self._last_output.linear.x, target.linear.x, dt)
        out.linear.y = self._slew_component('vy', self._last_output.linear.y, target.linear.y, dt)
        out.angular.z = self._slew_component('wz', self._last_output.angular.z, target.angular.z, dt)
        self._last_output = out
        self._last_output_time = now
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
        if self._joy_estop_button < 0:
            return
        # Toggle ESTOP on rising edge only (prev=0, curr=1).
        curr = msg.buttons[self._joy_estop_button] if len(msg.buttons) > self._joy_estop_button else 0
        if curr == 1 and self._prev_joy_estop == 0:
            self._estop = not self._estop
            if self._estop:
                self.get_logger().warn('Emergency stop activated via Joy button.')
                self._publish_zero()
                self._set_status(self.STATUS_ESTOP)
            else:
                self.get_logger().info('Emergency stop cleared via Joy button.')
        self._prev_joy_estop = curr

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
        out = Twist()
        self._last_output = out
        self._last_output_time = time.time()
        self._last_accel = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
        self._cmd_pub.publish(out)

    def _set_status(self, s: str):
        self._last_status = s

    def _publish_status(self):
        msg = String()
        msg.data = self._last_status
        self._status_pub.publish(msg)

    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))

    def _dt(self, now):
        return min(max(now - self._last_output_time, 0.01), 0.2)

    def _slew_component(self, key, current, target, dt):
        if dt <= 0.0:
            return current
        desired_accel = (target - current) / dt
        prev_accel = self._last_accel[key]
        jerk_step = max(0.0, self._max_jerk[key] * dt)
        accel = self._clamp(desired_accel, prev_accel - jerk_step, prev_accel + jerk_step)
        accel = self._clamp(accel, -self._max_accel[key], self._max_accel[key])
        next_value = current + accel * dt

        # Avoid tiny overshoot oscillations when ramping toward a fixed target.
        if (target - current) * (target - next_value) <= 0.0:
            next_value = target
            accel = (next_value - current) / dt

        self._last_accel[key] = accel
        return next_value


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

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
  "OK", "WATCHDOG", "ESTOP", "MANUAL_STOP", "TOUCH_LOCK"

Parameters (ROS):
  watchdog_timeout:  float = 0.30  seconds without input → zero output
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
  auto_output_frequency: float = 50.0  AUTO interpolation/output frequency (Hz)
"""
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


class CmdVelSafetyNode(Node):
    STATUS_OK = "OK"
    STATUS_WATCHDOG = "WATCHDOG"
    STATUS_ESTOP = "ESTOP"
    STATUS_TOUCH_LOCK = "TOUCH_LOCK"

    def __init__(self):
        super().__init__('cmd_vel_safety_node')

        self.declare_parameter('watchdog_timeout', 0.30)
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
        self.declare_parameter('manual_bypass_slew', True)
        self.declare_parameter('auto_output_frequency', 50.0)

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
        self._manual_bypass_slew = bool(
            self.get_parameter('manual_bypass_slew').value)
        self._auto_output_frequency = max(
            10.0,
            float(self.get_parameter('auto_output_frequency').value),
        )

        self._last_input_time = time.time()
        self._last_output_time = self._last_input_time
        self._last_output = Twist()
        self._last_accel = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
        self._auto_target = Twist()
        self._auto_target_active = False
        self._estop = False
        self._last_status = self.STATUS_OK
        self._prev_joy_estop = 0  # for rising-edge detection
        self._robot_mode = 'MANUAL'
        self._touch_lock = False

        self.create_subscription(Twist, '/cmd_vel_raw', self._raw_cb, 10)
        self.create_subscription(Bool, '/emergency_stop', self._estop_cb, 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(String, '/robot_mode', self._mode_cb, 10)
        touch_qos = QoSProfile(depth=1)
        touch_qos.reliability = ReliabilityPolicy.RELIABLE
        touch_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, '/touch_anything/base_lock', self._touch_lock_cb, touch_qos)

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._status_pub = self.create_publisher(String, '/safety_status', 10)

        # Watchdog timer: fires at 2x the timeout rate
        period = max(0.05, self._timeout / 2.0)
        self.create_timer(period, self._watchdog_tick)
        self.create_timer(
            1.0 / self._auto_output_frequency,
            self._auto_output_tick,
        )
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'CmdVelSafetyNode ready. '
            f'watchdog={self._timeout}s '
            f'max vx={self._max_vx} vy={self._max_vy} wz={self._max_wz} '
            f'accel vx={self._max_accel["vx"]} '
            f'vy={self._max_accel["vy"]} wz={self._max_accel["wz"]} '
            f'jerk vx={self._max_jerk["vx"]} '
            f'vy={self._max_jerk["vy"]} wz={self._max_jerk["wz"]} '
            f'joy_estop_button={self._joy_estop_button} '
            f'manual_bypass_slew={self._manual_bypass_slew} '
            f'auto_output_frequency={self._auto_output_frequency}Hz'
        )

    def _raw_cb(self, msg: Twist):
        now = time.time()
        self._last_input_time = now
        if self._estop:
            self._publish_zero()
            return
        if self._touch_lock:
            self._publish_zero()
            self._set_status(self.STATUS_TOUCH_LOCK)
            return

        # A zero Twist is a stop request (Joy dead-man released, mode switch,
        # Nav2 stop, etc.).  Never slew-limit a stop: ramping the previous
        # command down made the real robot continue moving after the operator
        # released L1/the sticks.  Acceleration and jerk limits are retained
        # only while changing between non-zero motion commands.
        if self._is_zero_twist(msg):
            self._publish_zero()
            self._set_status(self.STATUS_OK)
            return

        target = Twist()
        target.linear.x = self._clamp(msg.linear.x, -self._max_vx, self._max_vx)
        target.linear.y = self._clamp(msg.linear.y, -self._max_vy, self._max_vy)
        target.angular.z = self._clamp(msg.angular.z, -self._max_wz, self._max_wz)

        # Human teleoperation should follow the stick directly after clamping.
        # AUTO keeps the acceleration/jerk limiter as a final safety guard.
        if self._robot_mode == 'MANUAL' and self._manual_bypass_slew:
            self._last_output = target
            self._last_output_time = now
            self._last_accel = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
            self._cmd_pub.publish(target)
            self._set_status(self.STATUS_OK)
            return

        # Nav2 normally updates at about 10 Hz and occasionally has 0.15--0.24 s
        # gaps on this machine.  Retain its newest target; a 50 Hz timer below
        # advances the limiter continuously.  Stop requests and the watchdog
        # still bypass this interpolation and publish zero immediately.
        self._auto_target = target
        self._auto_target_active = True
        self._set_status(self.STATUS_OK)

    def _auto_output_tick(self):
        if (
            self._robot_mode != 'AUTO'
            or self._estop
            or not self._auto_target_active
        ):
            return

        now = time.time()
        if now - self._last_input_time > self._timeout:
            self._publish_zero()
            self._set_status(self.STATUS_WATCHDOG)
            return

        target = self._auto_target
        dt = self._dt(now)
        out = Twist()
        out.linear.x = self._slew_component('vx', self._last_output.linear.x, target.linear.x, dt)
        out.linear.y = self._slew_component('vy', self._last_output.linear.y, target.linear.y, dt)
        out.angular.z = self._slew_component(
            'wz', self._last_output.angular.z, target.angular.z, dt)
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

    def _touch_lock_cb(self, msg: Bool):
        requested = bool(msg.data)
        if requested == self._touch_lock:
            return
        self._touch_lock = requested
        self._publish_zero()
        if requested:
            self.get_logger().warn(
                'Touch Mode base lock active: all /cmd_vel_raw motion is rejected.')
            if not self._estop:
                self._set_status(self.STATUS_TOUCH_LOCK)
        else:
            self.get_logger().info('Touch Mode base lock released.')
            if not self._estop:
                self._set_status(self.STATUS_OK)

    def _joy_cb(self, msg: Joy):
        if self._joy_estop_button < 0:
            return
        # Toggle ESTOP on rising edge only (prev=0, curr=1).
        curr = (
            msg.buttons[self._joy_estop_button]
            if len(msg.buttons) > self._joy_estop_button
            else 0
        )
        if curr == 1 and self._prev_joy_estop == 0:
            self._estop = not self._estop
            if self._estop:
                self.get_logger().warn('Emergency stop activated via Joy button.')
                self._publish_zero()
                self._set_status(self.STATUS_ESTOP)
            else:
                self.get_logger().info('Emergency stop cleared via Joy button.')
        self._prev_joy_estop = curr

    def _mode_cb(self, msg: String):
        mode = msg.data.upper().strip()
        if mode not in ('MANUAL', 'AUTO'):
            return
        if mode != self._robot_mode:
            # Never carry velocity or acceleration state across mode changes.
            self._publish_zero()
        self._robot_mode = mode

    def _watchdog_tick(self):
        if self._estop:
            return
        if self._touch_lock:
            self._publish_zero()
            self._set_status(self.STATUS_TOUCH_LOCK)
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
        self._auto_target = Twist()
        self._auto_target_active = False
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

    @staticmethod
    def _is_zero_twist(msg, epsilon=1e-6):
        return (
            abs(msg.linear.x) <= epsilon
            and abs(msg.linear.y) <= epsilon
            and abs(msg.linear.z) <= epsilon
            and abs(msg.angular.x) <= epsilon
            and abs(msg.angular.y) <= epsilon
            and abs(msg.angular.z) <= epsilon
        )

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

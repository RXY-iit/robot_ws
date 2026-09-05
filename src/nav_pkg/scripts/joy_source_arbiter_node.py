#!/usr/bin/env python3
"""
Select exactly one Joy source before commands reach robot consumers.

Physical input is the default.  Dashboard input requires an explicit acquire
command and is immediately pre-empted by any non-neutral physical input.
"""

from __future__ import annotations

import json
import math
import time
from typing import Iterable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class JoySourceArbiterCore:
    PHYSICAL = 'PHYSICAL'
    DASHBOARD = 'DASHBOARD'
    # Linux joy reports trigger axes 2 and 5 as +1.0 at rest and toward -1.0
    # when pressed on the current controller. Treat a trigger as activity only
    # below the same threshold used by lift_joy_node; otherwise an electrically
    # neutral controller would permanently deny Dashboard acquisition.
    ACTIVE_AXIS_INDICES = (0, 1, 3, 6, 7)
    TRIGGER_AXIS_INDICES = (2, 5)
    TRIGGER_ACTIVE_BELOW = -0.5

    def __init__(
        self,
        *,
        dashboard_timeout_sec: float = 0.30,
        physical_active_hold_sec: float = 0.50,
        axis_deadzone: float = 0.05,
    ) -> None:
        self.dashboard_timeout_sec = dashboard_timeout_sec
        self.physical_active_hold_sec = physical_active_hold_sec
        self.axis_deadzone = axis_deadzone
        self.source = self.PHYSICAL
        self.dashboard_acquired = False
        self.last_dashboard_time: float | None = None
        self.last_physical_active_time: float | None = None
        self.reason = 'physical source is the default'

    def is_active(self, axes: Iterable[float], buttons: Iterable[int]) -> bool:
        axis_values = list(axes)
        regular_axis_active = any(
            index < len(axis_values)
            and math.isfinite(float(axis_values[index]))
            and abs(float(axis_values[index])) > self.axis_deadzone
            for index in self.ACTIVE_AXIS_INDICES
        )
        trigger_active = any(
            index < len(axis_values)
            and math.isfinite(float(axis_values[index]))
            and float(axis_values[index]) < self.TRIGGER_ACTIVE_BELOW
            for index in self.TRIGGER_AXIS_INDICES
        )
        return regular_axis_active or trigger_active or any(
            int(value) != 0 for value in buttons
        )

    def acquire_dashboard(self, now: float) -> tuple[bool, bool]:
        if (
            self.last_physical_active_time is not None
            and now - self.last_physical_active_time < self.physical_active_hold_sec
        ):
            self.source = self.PHYSICAL
            self.dashboard_acquired = False
            self.reason = 'dashboard acquire denied: physical input is active'
            return False, False
        changed = self.source != self.DASHBOARD or not self.dashboard_acquired
        self.source = self.DASHBOARD
        self.dashboard_acquired = True
        self.last_dashboard_time = now
        self.reason = 'dashboard control explicitly acquired'
        return True, changed

    def release_dashboard(self) -> bool:
        changed = self.source == self.DASHBOARD or self.dashboard_acquired
        self.source = self.PHYSICAL
        self.dashboard_acquired = False
        self.last_dashboard_time = None
        self.reason = 'dashboard control released'
        return changed

    def physical_message(
        self, axes: Iterable[float], buttons: Iterable[int], now: float
    ) -> tuple[bool, bool]:
        active = self.is_active(axes, buttons)
        preempted = False
        if active:
            self.last_physical_active_time = now
            if self.source == self.DASHBOARD:
                preempted = True
                self.source = self.PHYSICAL
                self.dashboard_acquired = False
                self.last_dashboard_time = None
                self.reason = 'physical input pre-empted dashboard control'
        return self.source == self.PHYSICAL, preempted

    def dashboard_message(self, now: float) -> bool:
        if self.source != self.DASHBOARD or not self.dashboard_acquired:
            return False
        self.last_dashboard_time = now
        return True

    def watchdog(self, now: float) -> bool:
        if self.source != self.DASHBOARD or not self.dashboard_acquired:
            return False
        if self.last_dashboard_time is None:
            return False
        if now - self.last_dashboard_time <= self.dashboard_timeout_sec:
            return False
        self.source = self.PHYSICAL
        self.dashboard_acquired = False
        self.last_dashboard_time = None
        self.reason = 'dashboard source timed out; returned to physical'
        return True

    def status(self, now: float) -> dict:
        dashboard_age = (
            None
            if self.last_dashboard_time is None
            else max(0.0, now - self.last_dashboard_time)
        )
        physical_active_age = (
            None
            if self.last_physical_active_time is None
            else max(0.0, now - self.last_physical_active_time)
        )
        return {
            'source': self.source,
            'dashboard_acquired': self.dashboard_acquired,
            'reason': self.reason,
            'dashboard_age_sec': dashboard_age,
            'physical_active_age_sec': physical_active_age,
        }


class JoySourceArbiterNode(Node):

    def __init__(self) -> None:
        super().__init__('joy_source_arbiter_node')
        self.declare_parameter('dashboard_timeout_sec', 0.30)
        self.declare_parameter('physical_active_hold_sec', 0.50)
        self.declare_parameter('axis_deadzone', 0.05)
        self.core = JoySourceArbiterCore(
            dashboard_timeout_sec=float(
                self.get_parameter('dashboard_timeout_sec').value
            ),
            physical_active_hold_sec=float(
                self.get_parameter('physical_active_hold_sec').value
            ),
            axis_deadzone=float(self.get_parameter('axis_deadzone').value),
        )
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.status_pub = self.create_publisher(String, '/joy/source_status', 10)
        self.create_subscription(Joy, '/joy/physical', self._physical_cb, 10)
        self.create_subscription(Joy, '/joy/dashboard', self._dashboard_cb, 10)
        self.create_subscription(
            String, '/joy/source_command', self._source_command_cb, 10
        )
        self.create_timer(0.05, self._watchdog_tick)
        self.create_timer(0.5, self._publish_status)
        self._publish_status()
        self.get_logger().info(
            'Joy source arbiter ready: PHYSICAL default; Dashboard requires '
            'ACQUIRE_DASHBOARD; non-neutral physical input pre-empts Dashboard.'
        )

    def _physical_cb(self, msg: Joy) -> None:
        forward, preempted = self.core.physical_message(
            msg.axes, msg.buttons, time.monotonic()
        )
        if preempted:
            self._publish_neutral()
            self._publish_status()
        if forward:
            self.joy_pub.publish(msg)

    def _dashboard_cb(self, msg: Joy) -> None:
        if self.core.dashboard_message(time.monotonic()):
            self.joy_pub.publish(msg)

    def _source_command_cb(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command == 'ACQUIRE_DASHBOARD':
            acquired, changed = self.core.acquire_dashboard(time.monotonic())
            if changed:
                self._publish_neutral()
            if not acquired:
                self.get_logger().warn(self.core.reason)
        elif command == 'RELEASE_DASHBOARD':
            if self.core.release_dashboard():
                self._publish_neutral()
        else:
            self.get_logger().warn(f'Ignoring unknown Joy source command: {msg.data!r}')
            return
        self._publish_status()

    def _watchdog_tick(self) -> None:
        if self.core.watchdog(time.monotonic()):
            self._publish_neutral()
            self._publish_status()

    def _publish_neutral(self) -> None:
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [0.0] * 8
        msg.buttons = [0] * 8
        self.joy_pub.publish(msg)

    def _publish_status(self) -> None:
        payload = self.core.status(time.monotonic())
        payload['generated_at'] = time.time()
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoySourceArbiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

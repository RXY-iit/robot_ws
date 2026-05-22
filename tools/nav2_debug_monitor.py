#!/usr/bin/env python3
"""Compact Nav2 Phase 1 debug monitor.

This answers the practical question: when /nav2/cmd_vel is zero, did Nav2
reach the goal, fail, wait, cancel, lose odom, or simply not receive a plan?
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

try:
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
except Exception:  # pragma: no cover - only for broken environments
    NavigateToPose = None
    NavigateToPose_FeedbackMessage = None


STATUS_NAMES = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}


def yaw_from_pose(pose) -> float:
    q = pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def fmt_age(node: Node, stamp) -> str:
    if stamp is None:
        return "never"
    age = (node.get_clock().now() - stamp).nanoseconds / 1e9
    return f"{age:.1f}s ago"


def nonzero_twist(t: Optional[Twist]) -> bool:
    if t is None:
        return False
    return (
        abs(t.linear.x) > 1e-4
        or abs(t.linear.y) > 1e-4
        or abs(t.angular.z) > 1e-4
    )


class Nav2DebugMonitor(Node):
    def __init__(self):
        super().__init__("nav2_debug_monitor")

        self.goal: Optional[PoseStamped] = None
        self.goal_stamp = None
        self.plan: Optional[Path] = None
        self.plan_stamp = None
        self.nav_cmd: Optional[Twist] = None
        self.nav_cmd_stamp = None
        self.cmd: Optional[Twist] = None
        self.cmd_stamp = None
        self.odom: Optional[Odometry] = None
        self.odom_stamp = None
        self.mode: Optional[str] = None
        self.status_text = "no status"
        self.feedback_text = "no feedback"

        self.create_subscription(PoseStamped, "/goal_pose", self._goal_cb, 10)
        self.create_subscription(Path, "/plan", self._plan_cb, 10)
        self.create_subscription(Twist, "/nav2/cmd_vel", self._nav_cmd_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(Odometry, "/wheel_odom", self._odom_cb, 10)
        self.create_subscription(String, "/robot_mode", self._mode_cb, 10)
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._status_cb,
            10,
        )
        if NavigateToPose_FeedbackMessage is not None:
            self.create_subscription(
                NavigateToPose_FeedbackMessage,
                "/navigate_to_pose/_action/feedback",
                self._feedback_cb,
                10,
            )

        self.create_timer(1.0, self._print_summary)

    def _goal_cb(self, msg):
        self.goal = msg
        self.goal_stamp = self.get_clock().now()

    def _plan_cb(self, msg):
        self.plan = msg
        self.plan_stamp = self.get_clock().now()

    def _nav_cmd_cb(self, msg):
        self.nav_cmd = msg
        self.nav_cmd_stamp = self.get_clock().now()

    def _cmd_cb(self, msg):
        self.cmd = msg
        self.cmd_stamp = self.get_clock().now()

    def _odom_cb(self, msg):
        self.odom = msg
        self.odom_stamp = self.get_clock().now()

    def _mode_cb(self, msg):
        self.mode = msg.data

    def _status_cb(self, msg):
        if not msg.status_list:
            self.status_text = "empty"
            return
        latest = msg.status_list[-1]
        self.status_text = STATUS_NAMES.get(latest.status, str(latest.status))

    def _feedback_cb(self, msg):
        fb = msg.feedback
        self.feedback_text = (
            f"dist={fb.distance_remaining:.2f}m "
            f"recoveries={fb.number_of_recoveries} "
            f"time={fb.navigation_time.sec}s"
        )

    def _print_summary(self):
        lines = []
        lines.append("\n=== Nav2 debug ===")
        lines.append(f"mode={self.mode or 'unknown'} action={self.status_text} feedback={self.feedback_text}")

        if self.goal is None:
            lines.append("goal: none")
        else:
            p = self.goal.pose.position
            yaw = yaw_from_pose(self.goal.pose)
            lines.append(
                f"goal: age={fmt_age(self, self.goal_stamp)} "
                f"xy=({p.x:.2f},{p.y:.2f}) yaw={yaw:.2f}"
            )

        if self.plan is None:
            lines.append("plan: none")
        else:
            n = len(self.plan.poses)
            if n:
                s = self.plan.poses[0].pose.position
                e = self.plan.poses[-1].pose.position
                lines.append(
                    f"plan: age={fmt_age(self, self.plan_stamp)} poses={n} "
                    f"start=({s.x:.2f},{s.y:.2f}) end=({e.x:.2f},{e.y:.2f})"
                )
            else:
                lines.append(f"plan: age={fmt_age(self, self.plan_stamp)} poses=0")

        if self.odom is None:
            lines.append("odom: none")
        else:
            p = self.odom.pose.pose.position
            v = self.odom.twist.twist
            lines.append(
                f"odom: age={fmt_age(self, self.odom_stamp)} "
                f"pose=({p.x:.2f},{p.y:.2f}) "
                f"twist=({v.linear.x:.2f},{v.linear.y:.2f},{v.angular.z:.2f})"
            )

        nav = self.nav_cmd
        if nav is None:
            lines.append("nav2/cmd_vel: none")
        else:
            reason = "NONZERO" if nonzero_twist(nav) else "ZERO"
            lines.append(
                f"nav2/cmd_vel: {reason} age={fmt_age(self, self.nav_cmd_stamp)} "
                f"x={nav.linear.x:.3f} y={nav.linear.y:.3f} z={nav.angular.z:.3f}"
            )

        cmd = self.cmd
        if cmd is None:
            lines.append("cmd_vel: none")
        else:
            reason = "NONZERO" if nonzero_twist(cmd) else "ZERO"
            lines.append(
                f"cmd_vel: {reason} age={fmt_age(self, self.cmd_stamp)} "
                f"x={cmd.linear.x:.3f} y={cmd.linear.y:.3f} z={cmd.angular.z:.3f}"
            )

        hints = []
        if self.goal is None:
            hints.append("no goal received yet")
        elif self.status_text == "SUCCEEDED":
            hints.append("goal reached: cmd_vel zero is expected")
        elif self.status_text in ("ABORTED", "CANCELED"):
            hints.append(f"action {self.status_text.lower()}: check BT/controller logs")
        elif self.plan is None and self.goal is not None:
            hints.append("goal received but no plan: check planner, map, TF, start/goal cost")
        elif self.plan is not None and self.nav_cmd is not None and not nonzero_twist(self.nav_cmd):
            hints.append("plan exists but nav cmd is zero: check goal tolerance, controller failure, collision, or wait BT")
        if hints:
            lines.append("hint: " + " | ".join(hints))

        print("\n".join(lines), flush=True)


def main():
    rclpy.init()
    node = Nav2DebugMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

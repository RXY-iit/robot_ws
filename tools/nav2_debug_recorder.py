#!/usr/bin/env python3
"""Record Nav2 Phase 1 debug context to debug-output.

The recorder writes:
  - events.jsonl: timestamped topic/action/mode/user-operation events
  - latest_state.md: compact current state for sharing with Codex

It is intentionally lightweight and records summaries, not large sensor data.
Use rosbag separately when raw LiDAR/camera data is needed.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path as FsPath
from typing import Any, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import Joy
from std_msgs.msg import String

try:
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
except Exception:  # pragma: no cover
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


def now_stamp(node: Node) -> float:
    return node.get_clock().now().nanoseconds / 1e9


def yaw_from_pose(pose) -> float:
    q = pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def twist_dict(msg: Twist) -> dict[str, float]:
    return {
        "linear_x": float(msg.linear.x),
        "linear_y": float(msg.linear.y),
        "linear_z": float(msg.linear.z),
        "angular_x": float(msg.angular.x),
        "angular_y": float(msg.angular.y),
        "angular_z": float(msg.angular.z),
    }


def pose_dict(msg: PoseStamped) -> dict[str, float]:
    p = msg.pose.position
    return {
        "x": float(p.x),
        "y": float(p.y),
        "z": float(p.z),
        "yaw": float(yaw_from_pose(msg.pose)),
    }


def odom_dict(msg: Odometry) -> dict[str, Any]:
    p = msg.pose.pose.position
    t = msg.twist.twist
    return {
        "pose_x": float(p.x),
        "pose_y": float(p.y),
        "pose_yaw": float(yaw_from_pose(msg.pose.pose)),
        "twist_linear_x": float(t.linear.x),
        "twist_linear_y": float(t.linear.y),
        "twist_angular_z": float(t.angular.z),
    }


def nonzero(t: Optional[Twist]) -> bool:
    if t is None:
        return False
    return abs(t.linear.x) > 1e-4 or abs(t.linear.y) > 1e-4 or abs(t.angular.z) > 1e-4


class Nav2DebugRecorder(Node):
    def __init__(self, output_dir: FsPath):
        super().__init__("nav2_debug_recorder")
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.events_path = self.output_dir / "events.jsonl"
        self.latest_path = self.output_dir / "latest_state.md"
        self.run_info_path = self.output_dir / "run_info.md"
        self.events_file = self.events_path.open("a", buffering=1)

        self.prev_buttons: list[int] = []
        self.mode: Optional[str] = None
        self.goal: Optional[dict[str, Any]] = None
        self.plan: Optional[dict[str, Any]] = None
        self.nav_cmd: Optional[Twist] = None
        self.cmd: Optional[Twist] = None
        self.teleop_cmd: Optional[Twist] = None
        self.odom: Optional[dict[str, Any]] = None
        self.status = "no_status"
        self.feedback: Optional[dict[str, Any]] = None
        self.last_hint = "waiting for goal"

        self.create_subscription(String, "/robot_mode", self._mode_cb, 10)
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._goal_cb, 10)
        self.create_subscription(NavPath, "/plan", self._plan_cb, 10)
        self.create_subscription(Twist, "/nav2/cmd_vel", self._nav_cmd_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(Twist, "/teleop/cmd_vel", self._teleop_cmd_cb, 10)
        self.create_subscription(Odometry, "/wheel_odom", self._odom_cb, 10)
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

        self.create_timer(1.0, self._snapshot_cb)
        self._write_run_info()
        self._event("recorder_started", {"output_dir": str(self.output_dir)})
        self.get_logger().info(f"Recording Nav2 debug output to {self.output_dir}")

    def destroy_node(self):
        self._event("recorder_stopped", {})
        try:
            self.events_file.close()
        finally:
            super().destroy_node()

    def _write_run_info(self):
        text = [
            "# Nav2 Debug Run",
            "",
            f"- output_dir: `{self.output_dir}`",
            f"- pid: `{os.getpid()}`",
            "- files:",
            "  - `events.jsonl`: timestamped events",
            "  - `latest_state.md`: current compact state",
            "",
            "Use this folder when asking Codex to debug a run.",
            "",
        ]
        self.run_info_path.write_text("\n".join(text), encoding="utf-8")

    def _event(self, event: str, data: dict[str, Any]):
        record = {
            "t": now_stamp(self),
            "event": event,
            "data": data,
        }
        self.events_file.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _mode_cb(self, msg: String):
        if msg.data != self.mode:
            old = self.mode
            self.mode = msg.data
            self._event("mode_changed", {"old": old, "new": self.mode})

    def _joy_cb(self, msg: Joy):
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons)
        labels = {0: "A_AUTO", 1: "B_MANUAL", 4: "L1_DEADMAN"}
        for idx, label in labels.items():
            if idx < len(msg.buttons):
                old = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
                new = msg.buttons[idx]
                if new == 1 and old == 0:
                    self._event("joy_button_pressed", {"button": idx, "label": label})
                if new == 0 and old == 1 and idx == 4:
                    self._event("joy_button_released", {"button": idx, "label": label})
        self.prev_buttons = list(msg.buttons)

    def _goal_cb(self, msg: PoseStamped):
        self.goal = pose_dict(msg)
        self.goal["frame_id"] = msg.header.frame_id
        self._event("goal_pose", self.goal)

    def _plan_cb(self, msg: Path):
        n = len(msg.poses)
        data: dict[str, Any] = {"poses": n, "frame_id": msg.header.frame_id}
        if n:
            data["start"] = pose_dict(msg.poses[0])
            data["end"] = pose_dict(msg.poses[-1])
        self.plan = data
        self._event("plan", data)

    def _nav_cmd_cb(self, msg: Twist):
        was_zero = not nonzero(self.nav_cmd)
        self.nav_cmd = msg
        is_nonzero = nonzero(msg)
        if is_nonzero or not was_zero:
            self._event("nav2_cmd_vel", twist_dict(msg))

    def _cmd_cb(self, msg: Twist):
        self.cmd = msg
        if nonzero(msg):
            self._event("cmd_vel", twist_dict(msg))

    def _teleop_cmd_cb(self, msg: Twist):
        self.teleop_cmd = msg
        if nonzero(msg):
            self._event("teleop_cmd_vel", twist_dict(msg))

    def _odom_cb(self, msg: Odometry):
        self.odom = odom_dict(msg)

    def _status_cb(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        latest = msg.status_list[-1]
        new_status = STATUS_NAMES.get(latest.status, str(latest.status))
        if new_status != self.status:
            old = self.status
            self.status = new_status
            self._event("action_status", {"old": old, "new": new_status})

    def _feedback_cb(self, msg):
        fb = msg.feedback
        self.feedback = {
            "distance_remaining": float(fb.distance_remaining),
            "number_of_recoveries": int(fb.number_of_recoveries),
            "navigation_time_sec": int(fb.navigation_time.sec),
        }

    def _compute_hint(self) -> str:
        if self.goal is None:
            return "no goal received after recorder started; set a 2D Goal Pose in RViz"
        if self.status == "SUCCEEDED":
            return "goal succeeded; zero cmd_vel is expected"
        if self.status in ("ABORTED", "CANCELED"):
            return f"action {self.status.lower()}; check nav2 terminal logs"
        if self.goal and not self.plan:
            return "goal received but no plan; check planner/map/TF/start-goal cost"
        if self.plan and self.nav_cmd is not None and not nonzero(self.nav_cmd):
            return "plan exists but nav2 cmd is zero; check tolerance/controller/collision/wait BT"
        if self.mode == "AUTO" and self.nav_cmd is not None and nonzero(self.nav_cmd) and self.cmd is not None and not nonzero(self.cmd):
            return "nav2 cmd nonzero but /cmd_vel zero; check nav_mode_switch relay"
        if self.mode == "MANUAL" and self.nav_cmd is not None and nonzero(self.nav_cmd):
            return "nav2 wants motion but mode is MANUAL; relay is intentionally blocked"
        return "no obvious issue"

    def _snapshot_cb(self):
        self.last_hint = self._compute_hint()
        snapshot = {
            "t": now_stamp(self),
            "mode": self.mode,
            "action_status": self.status,
            "feedback": self.feedback,
            "goal": self.goal,
            "plan": self.plan,
            "odom": self.odom,
            "nav2_cmd_vel": twist_dict(self.nav_cmd) if self.nav_cmd else None,
            "teleop_cmd_vel": twist_dict(self.teleop_cmd) if self.teleop_cmd else None,
            "cmd_vel": twist_dict(self.cmd) if self.cmd else None,
            "hint": self.last_hint,
        }
        self._event("snapshot", snapshot)
        self._write_latest(snapshot)

    def _write_latest(self, snapshot: dict[str, Any]):
        lines = [
            "# Latest Nav2 Debug State",
            "",
            f"- mode: `{snapshot['mode']}`",
            f"- action_status: `{snapshot['action_status']}`",
            f"- hint: {snapshot['hint']}",
            f"- feedback: `{snapshot['feedback']}`",
            f"- goal: `{snapshot['goal']}`",
            f"- plan: `{snapshot['plan']}`",
            f"- odom: `{snapshot['odom']}`",
            f"- nav2_cmd_vel: `{snapshot['nav2_cmd_vel']}`",
            f"- teleop_cmd_vel: `{snapshot['teleop_cmd_vel']}`",
            f"- cmd_vel: `{snapshot['cmd_vel']}`",
            "",
            "Recent event log is in `events.jsonl`.",
            "",
        ]
        self.latest_path.write_text("\n".join(lines), encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Nav2DebugRecorder(FsPath(args.output_dir))
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

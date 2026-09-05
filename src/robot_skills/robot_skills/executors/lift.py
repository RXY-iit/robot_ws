from __future__ import annotations

import time

from std_msgs.msg import Float32, String

from robot_skills import result_codes as codes
from robot_skills.executors.base import make_result, now
from robot_skills.schemas import SkillContext, SkillResult


class LiftSkills:
    def __init__(self, node):
        self.node = node
        self.cmd_pub = node.create_publisher(String, "/lift/command", 10)
        self._stop_timer = None
        self.lift_speed_m_s = 0.06
        self._position_mm: float | None = None
        self._position_feedback_time = 0.0
        self._state = "UNKNOWN"
        node.create_subscription(Float32, "/lift/position", self._position_cb, 10)
        node.create_subscription(String, "/lift/state", self._state_cb, 10)

    def _position_cb(self, msg: Float32):
        self._position_mm = float(msg.data)
        self._position_feedback_time = time.monotonic()

    def _state_cb(self, msg: String):
        self._state = str(msg.data)

    def move_lift_bounded(
        self,
        context: SkillContext,
        direction: str,
        duration_sec: float | None = None,
        distance_m: float | None = None,
    ) -> SkillResult:
        started = now()
        command = direction.upper()
        if command not in {"UP", "DOWN"}:
            return make_result("move_lift_bounded", False, codes.FAILED_SCHEMA, "direction must be UP or DOWN.", started)
        distance = float(distance_m) if distance_m is not None else None
        if distance is not None and (distance <= 0.0 or distance > 0.30):
            return make_result(
                "move_lift_bounded",
                False,
                codes.FAILED_SCHEMA,
                "distance_m must be in (0, 0.30].",
                started,
                {"distance_m": distance},
            )
        if duration_sec is None:
            if distance is None:
                distance = 0.05
            duration = distance / self.lift_speed_m_s
        else:
            duration = float(duration_sec)
            if distance is None:
                distance = duration * self.lift_speed_m_s
        if duration <= 0.0 or duration > 5.0:
            return make_result(
                "move_lift_bounded",
                False,
                codes.FAILED_SCHEMA,
                "duration_sec must be in (0, 5.0].",
                started,
                {"duration_sec": duration, "distance_m": distance},
            )
        if context.mode == "dry_run":
            return make_result(
                "move_lift_bounded",
                True,
                codes.DRY_RUN_OK,
                "Dry-run bounded lift command accepted.",
                started,
                {
                    "direction": command,
                    "distance_m": distance,
                    "duration_sec": duration,
                    "lift_speed_m_s": self.lift_speed_m_s,
                },
            )
        if context.mode == "real":
            if context.robot_mode != "MANUAL":
                return make_result(
                    "move_lift_bounded",
                    False,
                    codes.NEEDS_HUMAN,
                    "Real lift motion requires MANUAL mode. Press B and retry.",
                    started,
                    {"robot_mode": context.robot_mode},
                )
            if self.cmd_pub.get_subscription_count() < 1:
                return make_result(
                    "move_lift_bounded",
                    False,
                    codes.FAILED_PRECHECK,
                    "Real lift driver is not subscribed to /lift/command.",
                    started,
                )
            feedback_age = (
                time.monotonic() - self._position_feedback_time
                if self._position_feedback_time
                else float("inf")
            )
            if feedback_age > 1.0:
                return make_result(
                    "move_lift_bounded",
                    False,
                    codes.FAILED_PRECHECK,
                    "Real lift position feedback is missing or stale.",
                    started,
                    {"feedback_age_sec": feedback_age},
                )
        if self._stop_timer is not None:
            self._stop_timer.cancel()
            self._stop_timer = None
        self.cmd_pub.publish(String(data=command))

        def publish_stop_once():
            self.cmd_pub.publish(String(data="STOP"))
            if self._stop_timer is not None:
                self._stop_timer.cancel()
                self._stop_timer = None

        self._stop_timer = self.node.create_timer(float(duration), publish_stop_once)
        return make_result(
            "move_lift_bounded",
            True,
            codes.COMMAND_ACCEPTED,
            "Bounded lift command accepted; STOP will be published by timer.",
            started,
            {
                "direction": command,
                "distance_m": distance,
                "duration_sec": duration,
                "lift_speed_m_s": self.lift_speed_m_s,
                "stop_timer_active": True,
                "position_before_mm": self._position_mm,
                "lift_state_before": self._state,
            },
        )

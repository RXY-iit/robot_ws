from __future__ import annotations

import time
import math

from robot_skills import result_codes as codes
from robot_skills.executors.base import make_result, now
from robot_skills.schemas import SkillContext, SkillResult


class SystemSkills:
    def __init__(self, node, mode: str = "sim"):
        self.node = node
        self.mode = mode
        self.robot_mode = "UNKNOWN"
        self.safety_status = "UNKNOWN"
        self._last_robot_mode_time = 0.0
        self._last_safety_time = 0.0
        self._last_cmd_vel_time = 0.0
        self._last_cmd_speed = 0.0
        self._last_nav2_cmd_vel_time = 0.0
        self._last_nav2_cmd_speed = 0.0
        self._local_planner_required = False
        self._local_planner_required_reason = "idle"
        self._local_planner_state_label = "UNKNOWN"
        self._local_planner_state_id = None
        self._last_local_planner_state_time = 0.0
        self._local_planner_state_stale_sec = 5.0
        self._stationary_pose = None
        self._stationary_pose_time = 0.0
        self._latest_odom_pose = None
        self._stationary_drift_m = 0.0
        self._stationary_drift_yaw_rad = 0.0
        self._localization_warning = "UNKNOWN_ODOM"
        self._last_trusted_pose = None
        self._last_trusted_pose_time = 0.0
        self._gicp_score = None
        self._last_gicp_score_time = 0.0
        if mode == "real":
            self._gicp_warn_threshold = 0.25
            self._gicp_bad_threshold = 0.5
            self._stationary_drift_warn_m = 0.05
            self._stationary_drift_warn_yaw_rad = 0.10
        else:
            self._gicp_warn_threshold = 0.5
            self._gicp_bad_threshold = 0.65
            self._stationary_drift_warn_m = 0.15
            self._stationary_drift_warn_yaw_rad = 0.20
        self._last_initialpose_time = 0.0
        self._initialpose_settle_sec = 10.0

    def update_robot_mode(self, value: str):
        self.robot_mode = value
        self._last_robot_mode_time = time.time()

    def update_safety_status(self, value: str):
        self.safety_status = value
        self._last_safety_time = time.time()

    def update_gicp_score(self, score: float):
        self._gicp_score = float(score)
        self._last_gicp_score_time = time.time()
        if self._gicp_score >= self._gicp_bad_threshold:
            self._localization_warning = "GICP_SCORE_BAD"
        elif self._gicp_score >= self._gicp_warn_threshold:
            self._localization_warning = "GICP_SCORE_WARN"

    def update_initialpose(self):
        self._last_initialpose_time = time.time()
        self._stationary_pose = None
        self._stationary_drift_m = 0.0
        self._stationary_drift_yaw_rad = 0.0
        self._localization_warning = "MANUAL_INITIALPOSE_SETTLING"

    def update_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
        cmd_speed = abs(linear_x) + abs(linear_y) + abs(angular_z)
        self._last_cmd_speed = cmd_speed
        if cmd_speed > 0.02:
            self._last_cmd_vel_time = time.time()
            self._stationary_pose = None
            self._stationary_drift_m = 0.0
            self._stationary_drift_yaw_rad = 0.0
            if self._score_warning() is None:
                self._localization_warning = "MOVING"

    def update_nav2_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
        cmd_speed = abs(linear_x) + abs(linear_y) + abs(angular_z)
        self._last_nav2_cmd_speed = cmd_speed
        self._last_nav2_cmd_vel_time = time.time()

    def update_local_planner_state(self, label: str, state_id: int | None = None):
        self._local_planner_state_label = str(label or "UNKNOWN")
        self._local_planner_state_id = state_id
        self._last_local_planner_state_time = time.time()

    def set_local_planner_required(self, required: bool, reason: str = ""):
        self._local_planner_required = bool(required)
        self._local_planner_required_reason = reason or ("navigation_active" if required else "idle")

    def update_odom_pose(self, x: float, y: float, yaw: float):
        now_time = time.time()
        self._latest_odom_pose = (x, y, yaw)
        idle = self._last_cmd_speed <= 0.02 and (
            self._last_cmd_vel_time == 0.0 or now_time - self._last_cmd_vel_time > 0.5
        )
        if not idle:
            return
        if self._stationary_pose is None:
            self._stationary_pose = (x, y, yaw)
            self._stationary_pose_time = now_time
            self._stationary_drift_m = 0.0
            self._stationary_drift_yaw_rad = 0.0
            self._localization_warning = self._score_warning() or "OK"
            if self.localization_is_healthy():
                self._record_trusted_pose(x, y, yaw, now_time)
            return
        sx, sy, syaw = self._stationary_pose
        self._stationary_drift_m = math.hypot(x - sx, y - sy)
        self._stationary_drift_yaw_rad = abs(self._angle_diff(yaw, syaw))
        if now_time - self._stationary_pose_time >= 3.0 and (
            self._stationary_drift_m > self._stationary_drift_warn_m
            or self._stationary_drift_yaw_rad > self._stationary_drift_warn_yaw_rad
        ):
            self._localization_warning = "POSSIBLE_STATIONARY_DRIFT"
        elif self._gicp_score is not None and self._gicp_score >= self._gicp_bad_threshold:
            self._localization_warning = "GICP_SCORE_BAD"
        elif self._gicp_score is not None and self._gicp_score >= self._gicp_warn_threshold:
            self._localization_warning = "GICP_SCORE_WARN"
        else:
            self._localization_warning = "OK"
        if self.localization_is_healthy():
            self._record_trusted_pose(x, y, yaw, now_time)

    def get_robot_status(self, context: SkillContext | None = None) -> SkillResult:
        started = now()
        metrics = self.localization_metrics(context)
        metrics.update({
            "robot_mode": self.robot_mode,
            "safety_status": self.safety_status,
            "robot_mode_age_sec": time.time() - self._last_robot_mode_time if self._last_robot_mode_time else None,
            "safety_status_age_sec": time.time() - self._last_safety_time if self._last_safety_time else None,
            "mode": context.mode if context else "unknown",
        })
        return make_result("get_robot_status", True, codes.OK, "Robot status collected.", started, metrics)

    def localization_is_healthy(self) -> bool:
        if self._latest_odom_pose is None:
            return False
        if self.initialpose_is_settling():
            return False
        if self._localization_warning not in {"OK", "MOVING"}:
            return False
        if self._gicp_score is not None and self._gicp_score >= self._gicp_warn_threshold:
            return False
        return True

    def localization_metrics(self, context: SkillContext | None = None) -> dict:
        initialpose_age = time.time() - self._last_initialpose_time if self._last_initialpose_time else None
        return {
            "latest_odom_pose": self._latest_odom_pose,
            "stationary_drift_m": self._stationary_drift_m,
            "stationary_drift_yaw_rad": self._stationary_drift_yaw_rad,
            "stationary_drift_warn_m": self._stationary_drift_warn_m,
            "stationary_drift_warn_yaw_rad": self._stationary_drift_warn_yaw_rad,
            "localization_warning": self._localization_warning,
            "localization_healthy": self.localization_is_healthy(),
            "gicp_score": self._gicp_score,
            "gicp_score_age_sec": time.time() - self._last_gicp_score_time if self._last_gicp_score_time else None,
            "last_trusted_pose": self._last_trusted_pose,
            "last_trusted_pose_age_sec": time.time() - self._last_trusted_pose_time if self._last_trusted_pose_time else None,
            "initialpose_age_sec": initialpose_age,
            "initialpose_settling": self.initialpose_is_settling(),
            "initialpose_settle_sec": self._initialpose_settle_sec,
            "mode": context.mode if context else "unknown",
        }

    def health_summary(self, context: SkillContext | None = None) -> dict:
        localization = self.localization_metrics(context)
        local_planner = self.local_planner_metrics()
        safety_level = self._level_from_safety()
        mode_level = "OK" if self.robot_mode == "AUTO" else ("WARN" if self.robot_mode in {"MANUAL", "UNKNOWN"} else "WARN")
        localization_level = self._level_from_localization(localization)
        local_planner_level = self._level_from_local_planner(local_planner)
        components = {
            "mode": {
                "level": mode_level,
                "summary": self.robot_mode or "UNKNOWN",
                "details": {"robot_mode_age_sec": time.time() - self._last_robot_mode_time if self._last_robot_mode_time else None},
            },
            "safety": {
                "level": safety_level,
                "summary": self.safety_status or "UNKNOWN",
                "details": {"safety_status_age_sec": time.time() - self._last_safety_time if self._last_safety_time else None},
            },
            "localization": {
                "level": localization_level,
                "summary": localization.get("localization_warning", "UNKNOWN"),
                "details": localization,
            },
            "local_planner": {
                "level": local_planner_level,
                "summary": local_planner.get("summary", "UNKNOWN"),
                "details": local_planner,
            },
        }
        overall = self._worst_level([item["level"] for item in components.values()])
        recommended_action = {
            "OK": "continue",
            "WARN": "continue_with_caution",
            "BAD": "pause_or_recover",
            "UNKNOWN": "check_system",
        }.get(overall, "check_system")
        return {
            "overall": overall,
            "components": components,
            "recommended_action": recommended_action,
            "timestamp": now(),
        }

    def local_planner_metrics(self) -> dict:
        now_time = time.time()
        state_age = now_time - self._last_local_planner_state_time if self._last_local_planner_state_time else None
        nav2_cmd_age = now_time - self._last_nav2_cmd_vel_time if self._last_nav2_cmd_vel_time else None
        label = self._local_planner_state_label or "UNKNOWN"
        normalized = label.lower()
        state_recent = state_age is not None and state_age <= self._local_planner_state_stale_sec
        if not self._local_planner_required and not state_recent:
            summary = "standby"
        elif not state_recent:
            summary = "controller_state_unknown"
        elif normalized == "active":
            summary = "active"
        else:
            summary = f"controller_{normalized}"
        return {
            "required": self._local_planner_required,
            "required_reason": self._local_planner_required_reason,
            "controller_state": label,
            "controller_state_id": self._local_planner_state_id,
            "controller_state_age_sec": state_age,
            "controller_state_stale_sec": self._local_planner_state_stale_sec,
            "nav2_cmd_age_sec": nav2_cmd_age,
            "nav2_cmd_speed": self._last_nav2_cmd_speed,
            "summary": summary,
        }

    def last_trusted_pose(self):
        return self._last_trusted_pose

    def initialpose_is_settling(self) -> bool:
        return bool(self._last_initialpose_time and time.time() - self._last_initialpose_time < self._initialpose_settle_sec)

    def _level_from_safety(self) -> str:
        normalized = str(self.safety_status or "").upper()
        if normalized in {"OK", "WATCHDOG", "WATCHDOG_OK", "SAFE"}:
            return "OK"
        if normalized in {"ESTOP", "E_STOP", "ERROR", "FAULT"}:
            return "BAD"
        if normalized in {"UNKNOWN", ""}:
            return "UNKNOWN"
        return "WARN"

    def _level_from_localization(self, metrics: dict) -> str:
        score = metrics.get("gicp_score")
        if isinstance(score, (int, float)) and score >= self._gicp_bad_threshold:
            return "BAD"
        if isinstance(score, (int, float)) and score >= self._gicp_warn_threshold:
            return "BAD"
        if metrics.get("initialpose_settling"):
            return "WARN"
        warning = str(metrics.get("localization_warning") or "UNKNOWN")
        if warning == "OK":
            return "OK"
        if warning == "MOVING":
            return "OK"
        if warning in {"GICP_SCORE_BAD", "GICP_SCORE_WARN", "POSSIBLE_STATIONARY_DRIFT", "UNKNOWN_ODOM"}:
            return "BAD"
        if warning in {"MANUAL_INITIALPOSE_SETTLING"}:
            return "WARN"
        return "UNKNOWN"

    def _level_from_local_planner(self, metrics: dict) -> str:
        required = bool(metrics.get("required"))
        label = str(metrics.get("controller_state") or "UNKNOWN").lower()
        age = metrics.get("controller_state_age_sec")
        state_recent = isinstance(age, (int, float)) and age <= float(metrics.get("controller_state_stale_sec") or 5.0)
        if not required and not state_recent:
            return "OK"
        if state_recent and label == "active":
            return "OK"
        if required:
            return "BAD"
        if state_recent and label not in {"unknown", ""}:
            return "WARN"
        return "UNKNOWN"

    @staticmethod
    def _worst_level(levels: list[str]) -> str:
        rank = {"OK": 0, "WARN": 1, "UNKNOWN": 2, "BAD": 3}
        return max(levels or ["UNKNOWN"], key=lambda item: rank.get(item, 2))

    def _record_trusted_pose(self, x: float, y: float, yaw: float, timestamp: float):
        self._last_trusted_pose = (float(x), float(y), float(yaw))
        self._last_trusted_pose_time = timestamp

    def _score_warning(self) -> str | None:
        if self._gicp_score is not None and self._gicp_score >= self._gicp_bad_threshold:
            return "GICP_SCORE_BAD"
        if self._gicp_score is not None and self._gicp_score >= self._gicp_warn_threshold:
            return "GICP_SCORE_WARN"
        if self.initialpose_is_settling():
            return "MANUAL_INITIALPOSE_SETTLING"
        return None

    def await_robot_auto_mode(self, context: SkillContext | None = None, timeout_sec: float = 0.1) -> SkillResult:
        started = now()
        deadline = time.time() + max(0.0, timeout_sec)
        while time.time() <= deadline:
            if self.robot_mode == "AUTO":
                return make_result(
                    "await_robot_auto_mode",
                    True,
                    codes.OK,
                    "Robot is in AUTO mode.",
                    started,
                    {"robot_mode": self.robot_mode},
                )
            time.sleep(0.05)

        return make_result(
            "await_robot_auto_mode",
            False,
            codes.NEEDS_HUMAN,
            "Navigation requires AUTO mode. Press A / confirm AUTO, then resend or continue.",
            started,
            {"robot_mode": self.robot_mode},
        )

    def await_robot_stationary(
        self,
        context: SkillContext | None = None,
        settle_sec: float = 1.0,
        timeout_sec: float = 180.0,
    ) -> SkillResult:
        started = now()
        deadline = time.time() + max(0.0, timeout_sec)
        settle_started_at: float | None = None
        while time.time() <= deadline:
            moving = self._last_cmd_speed > 0.02 or (
                self._last_cmd_vel_time > 0.0 and time.time() - self._last_cmd_vel_time < 0.5
            )
            if moving:
                settle_started_at = None
            else:
                if settle_started_at is None:
                    settle_started_at = time.time()
                if time.time() - settle_started_at >= max(0.0, settle_sec):
                    return make_result(
                        "await_robot_stationary",
                        True,
                        codes.OK,
                        "Robot is stationary.",
                        started,
                        {
                            "settle_sec": settle_sec,
                            "last_cmd_speed": self._last_cmd_speed,
                            "last_cmd_vel_age_sec": time.time() - self._last_cmd_vel_time if self._last_cmd_vel_time else None,
                        },
                    )
            time.sleep(0.05)

        return make_result(
            "await_robot_stationary",
            False,
            codes.FAILED_TIMEOUT,
            "Robot did not become stationary before perception timeout.",
            started,
            {
                "settle_sec": settle_sec,
                "timeout_sec": timeout_sec,
                "last_cmd_speed": self._last_cmd_speed,
                "last_cmd_vel_age_sec": time.time() - self._last_cmd_vel_time if self._last_cmd_vel_time else None,
            },
        )

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

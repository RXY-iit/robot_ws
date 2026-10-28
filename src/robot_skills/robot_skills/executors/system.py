from __future__ import annotations

import time
import math

from robot_skills import result_codes as codes
from robot_skills.localization_reference import LocalizationReference
from robot_skills.executors.base import make_result, now
from robot_skills.schemas import SkillContext, SkillResult


class SystemSkills:
    # Receipt ages use monotonic wall time; sensor stamps stay in the ROS clock.
    def __init__(self, node, mode: str = "sim"):
        self.node = node
        self.mode = mode
        self._reference = LocalizationReference()
        self._reference_observation_verified = False
        self._manual_alignment_pending = False
        self._initialpose_source = "none"
        self.robot_mode = "UNKNOWN"
        self.safety_status = "UNKNOWN"
        self._last_robot_mode_time = 0.0
        self._last_safety_time = 0.0
        self._last_cmd_vel_time = 0.0
        self._last_cmd_receipt_time = 0.0
        self._last_cmd_speed = 0.0
        self._last_nav2_cmd_vel_time = 0.0
        self._last_nav2_cmd_speed = 0.0
        self._latest_wheel_odom_pose = None
        self._last_wheel_odom_time = 0.0
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
        self._wheel_odom_pose_at_trust = None
        self._last_trusted_pose_time = 0.0
        self._trusted_pose_stamp = None
        self._wheel_odom_stamp = None
        self._evidence_stamp = None
        self._evidence_odom_anchor = None
        self._recovery_accumulating = False
        self._recovery_status_received = 0.0
        self._gicp_score = None
        self._last_gicp_score_time = 0.0
        self._last_localization_pose_time = 0.0
        self._gicp_good_samples_after_initialpose = 0
        self._gicp_min_good_samples = 3
        self._localization_data_stale_sec = 6.0 if mode == "sim" else 3.0
        self._localization_source_stale_sec = 2.0
        if mode == "real":
            self._gicp_warn_threshold = 0.25
            self._gicp_bad_threshold = 0.5
            self._stationary_drift_warn_m = 0.05
            self._stationary_drift_warn_yaw_rad = 0.10
            self._trusted_pose_delta_warn_m = 0.12
            self._trusted_pose_delta_warn_yaw_rad = 0.20
        else:
            # Preserve the legacy task emergency gate. HealthMonitor owns the
            # debounced .16/.20 policy; recovery reference qualification is <.15.
            self._gicp_warn_threshold = 0.5
            self._gicp_bad_threshold = 0.65
            self._stationary_drift_warn_m = 0.15
            self._stationary_drift_warn_yaw_rad = 0.20
            self._trusted_pose_delta_warn_m = 0.25
            self._trusted_pose_delta_warn_yaw_rad = 0.35
        self._last_initialpose_time = 0.0
        self._initialpose_settle_sec = 10.0
        self._stationary_cmd_speed_threshold = 0.02
        self._stationary_cmd_fresh_sec = 0.5

    def update_robot_mode(self, value: str):
        self.robot_mode = value
        self._last_robot_mode_time = time.monotonic()

    def update_safety_status(self, value: str):
        self.safety_status = value
        self._last_safety_time = time.monotonic()

    def update_gicp_score(self, score: float):
        self._gicp_score = float(score)
        self._last_gicp_score_time = time.monotonic()
        if self.mode == "sim" and (not math.isfinite(score) or score >= self._reference.score_limit):
            self._reference.disturb("score_not_qualified")
        if not math.isfinite(self._gicp_score) or self._gicp_score >= self._gicp_bad_threshold:
            self._gicp_good_samples_after_initialpose = 0
            self._localization_warning = "GICP_SCORE_BAD"
        elif self._gicp_score >= self._gicp_warn_threshold:
            self._gicp_good_samples_after_initialpose = 0
            self._localization_warning = "GICP_SCORE_WARN"
        else:
            self._gicp_good_samples_after_initialpose += 1

    def update_initialpose(self, *, operator_alignment=False, source="unknown"):
        self._initialpose_source = source
        if self.mode == "sim":
            if self._manual_alignment_pending and not operator_alignment:
                self._reference.invalidate("operator_alignment_superseded")
            self._reference.disturb("initialpose_pending_verification")
            self._reference_observation_verified = False
            self._manual_alignment_pending = bool(operator_alignment)
            if operator_alignment:
                # Authorization comes from the explicit operator channel with
                # stopped-motion evidence, not the AUTO/MANUAL drive mode.
                self._reference.anchor = None
                self._reference.frozen = False
                self._reference.reset_required = False
                self._last_trusted_pose = None
                self._wheel_odom_pose_at_trust = None
                self._trusted_pose_stamp = None
                self._last_trusted_pose_time = 0.0
        self._last_initialpose_time = time.monotonic()
        self._stationary_pose = None
        self._stationary_drift_m = 0.0
        self._stationary_drift_yaw_rad = 0.0
        self._latest_odom_pose = None
        # initialpose resets the map estimate, not the physical odometry frame.
        # Keep the previously verified map/odom anchor for bounded recovery.
        # Clock rollback and odometry jumps still invalidate it in the odom callback.
        self._last_localization_pose_time = 0.0
        self._gicp_score = None
        self._last_gicp_score_time = 0.0
        self._gicp_good_samples_after_initialpose = 0
        self._localization_warning = "MANUAL_INITIALPOSE_SETTLING"

    def update_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
        self._last_cmd_receipt_time = time.monotonic()
        cmd_speed = abs(linear_x) + abs(linear_y) + abs(angular_z)
        self._last_cmd_speed = cmd_speed
        if cmd_speed > 0.02:
            self._last_cmd_vel_time = time.monotonic()
            self._stationary_pose = None
            self._stationary_drift_m = 0.0
            self._stationary_drift_yaw_rad = 0.0
            if self._score_warning() is None:
                self._localization_warning = "MOVING"

    def update_nav2_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
        cmd_speed = abs(linear_x) + abs(linear_y) + abs(angular_z)
        self._last_nav2_cmd_speed = cmd_speed
        self._last_nav2_cmd_vel_time = time.monotonic()

    def update_wheel_odom_pose(self, x: float, y: float, yaw: float, stamp=None):
        if self.mode == "sim" and stamp is not None:
            self._reference.odom((x, y, yaw), stamp)
        if stamp is not None and self._wheel_odom_stamp is not None:
            dt = stamp - self._wheel_odom_stamp
            previous = self._latest_wheel_odom_pose
            if dt < 0 or (previous is not None and dt > 0 and (
                math.hypot(x - previous[0], y - previous[1]) > 2 * dt + 0.15
                or abs(self._angle_diff(yaw, previous[2])) > 3 * dt + 0.15)):
                self.invalidate_localization_anchor()
        self._wheel_odom_stamp = stamp
        self._latest_wheel_odom_pose = (float(x), float(y), float(yaw))
        self._last_wheel_odom_time = time.monotonic()

    def update_local_planner_state(self, label: str, state_id: int | None = None):
        self._local_planner_state_label = str(label or "UNKNOWN")
        self._local_planner_state_id = state_id
        self._last_local_planner_state_time = time.monotonic()

    def set_local_planner_required(self, required: bool, reason: str = ""):
        self._local_planner_required = bool(required)
        self._local_planner_required_reason = reason or ("navigation_active" if required else "idle")

    def invalidate_localization_anchor(self):
        self._reference.invalidate("odom_or_clock_reset")
        self._last_trusted_pose = None
        self._wheel_odom_pose_at_trust = None
        self._trusted_pose_stamp = None
        self._evidence_stamp = None
        self._evidence_odom_anchor = None
        self._gicp_good_samples_after_initialpose = 0
        self._localization_warning = "GICP_POSE_STALE"

    def update_odom_pose(self, x: float, y: float, yaw: float, stamp=None, odom_anchor=None):
        old_reference = self._reference.anchor
        self._update_odom_pose_checks(x, y, yaw, stamp, odom_anchor)
        candidate = self._reference.anchor
        if (self.mode == "sim" and stamp is not None and candidate is not old_reference
                and candidate is not None and self._trusted_pose_stamp != candidate['stamp']):
            # Qualification is not a shortcut around the existing consistency
            # checks. Commit both references together or retain the old one.
            self._reference.anchor = old_reference
            self._reference.disturb("consistency_not_qualified")

    def _update_odom_pose_checks(self, x, y, yaw, stamp, odom_anchor):
        self._evidence_stamp = stamp
        self._evidence_odom_anchor = odom_anchor
        now_time = time.monotonic()
        self._latest_odom_pose = (x, y, yaw)
        self._last_localization_pose_time = now_time
        strict_reference = self.mode == "sim" and stamp is not None
        if strict_reference:
            was_frozen = self._reference.frozen
            self._reference_observation_verified = self._reference.observe(
                (x, y, yaw), odom_anchor, stamp, now_time,
                self._gicp_score if self._gicp_score is not None else float('inf'),
                self.initialpose_is_settling())
            if was_frozen and self._reference_observation_verified:
                # Stable clean evidence agrees with the pre-fault propagated
                # reference: only now rebuild the stationary comparison origin.
                self._stationary_pose = None
                self._localization_warning = "OK"
            if self._reference_observation_verified:
                self._manual_alignment_pending = False
            if self._manual_alignment_pending or self._reference.reset_required:
                # A human seed permits requalification, never immediate health.
                self._localization_warning = self._score_warning() or "RECOVERY_REFERENCE_UNVERIFIED"
                return
            if self._reference.frozen:
                severe = self._reference.reason in {
                    "reference_disagreement", "inconsistent_map_odom",
                    "reference_unavailable_manual_alignment_required", "odom_discontinuity", "nonfinite_odom"}
                if severe or self.initialpose_is_settling():
                    self._localization_warning = self._score_warning() or "RECOVERY_REFERENCE_UNVERIFIED"
                    return
        nav2_motion_requested = (
            self._last_nav2_cmd_speed > 0.02
            and self._last_nav2_cmd_vel_time > 0.0
            and now_time - self._last_nav2_cmd_vel_time <= 0.5
        )
        idle = self._last_cmd_speed <= 0.02 and (
            self._last_cmd_vel_time == 0.0 or now_time - self._last_cmd_vel_time > 0.5
        ) and not nav2_motion_requested
        if not idle:
            if self.localization_is_healthy():
                self._record_trusted_pose(x, y, yaw, now_time)
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
            "robot_mode_age_sec": time.monotonic() - self._last_robot_mode_time if self._last_robot_mode_time else None,
            "safety_status_age_sec": time.monotonic() - self._last_safety_time if self._last_safety_time else None,
            "mode": context.mode if context else "unknown",
        })
        return make_result("get_robot_status", True, codes.OK, "Robot status collected.", started, metrics)

    def localization_is_healthy(self) -> bool:
        now_time = time.monotonic()
        if self._latest_odom_pose is None:
            return False
        if self._gicp_score is None:
            return False
        if self._localization_receipt_stale(self._last_gicp_score_time, now_time):
            return False
        if self._localization_receipt_stale(self._last_localization_pose_time, now_time):
            return False
        if self._gicp_good_samples_after_initialpose < self._gicp_min_good_samples:
            return False
        if self.initialpose_is_settling():
            return False
        if self._localization_warning not in {"OK", "MOVING"}:
            return False
        if self._gicp_score is not None and self._gicp_score >= self._gicp_warn_threshold:
            return False
        return True

    def localization_has_fresh_good_evidence(self, context: SkillContext | None = None) -> bool:
        return self._localization_fresh_good_evidence_from_warning(self._score_warning() or self._localization_warning)

    def localization_requires_recovery(self, context: SkillContext | None = None) -> bool:
        metrics = self.localization_metrics(context)
        return bool(metrics.get("localization_requires_recovery"))

    def localization_metrics(self, context: SkillContext | None = None) -> dict:
        initialpose_age = time.monotonic() - self._last_initialpose_time if self._last_initialpose_time else None
        trusted_delta_m, trusted_delta_yaw = self._trusted_pose_delta()
        trusted_delta_warning = (
            trusted_delta_m is not None
            and (
                trusted_delta_m > self._trusted_pose_delta_warn_m
                or (trusted_delta_yaw is not None and trusted_delta_yaw > self._trusted_pose_delta_warn_yaw_rad)
            )
        )
        score_warning = self._score_warning()
        effective_warning = score_warning or self._localization_warning
        if trusted_delta_warning and effective_warning in {
            "GICP_POSE_STALE",
            "GICP_SCORE_STALE",
            "GICP_SCORE_WARN",
        }:
            effective_warning = "GICP_TRUSTED_POSE_DIVERGENCE"
        return {
            **self._reference.metrics(time.monotonic()),
            "initialpose_source": self._initialpose_source,
            "manual_alignment_pending": self._manual_alignment_pending,
            "latest_odom_pose": self._latest_odom_pose,
            "latest_wheel_odom_pose": self._latest_wheel_odom_pose,
            "stationary_drift_m": self._stationary_drift_m,
            "stationary_drift_yaw_rad": self._stationary_drift_yaw_rad,
            "stationary_drift_warn_m": self._stationary_drift_warn_m,
            "stationary_drift_warn_yaw_rad": self._stationary_drift_warn_yaw_rad,
            "trusted_pose_delta_m": trusted_delta_m,
            "trusted_pose_delta_kind": "wheel_motion_since_trusted_anchor",
            "trusted_pose_delta_yaw_rad": trusted_delta_yaw,
            "trusted_pose_delta_warn_m": self._trusted_pose_delta_warn_m,
            "trusted_pose_delta_warn_yaw_rad": self._trusted_pose_delta_warn_yaw_rad,
            "localization_warning": effective_warning,
            "localization_healthy": self.localization_is_healthy(),
            "gicp_score": self._gicp_score,
            "gicp_score_age_sec": time.monotonic() - self._last_gicp_score_time if self._last_gicp_score_time else None,
            "localization_pose_age_sec": time.monotonic() - self._last_localization_pose_time if self._last_localization_pose_time else None,
            "gicp_good_samples_after_initialpose": self._gicp_good_samples_after_initialpose,
            "gicp_min_good_samples": self._gicp_min_good_samples,
            "localization_data_stale_sec": self._localization_data_stale_sec,
            "localization_source_stale_sec": self._localization_source_stale_sec if self.mode == "sim" else None,
            "last_trusted_pose": self._last_trusted_pose,
            "trusted_pose_frame": "base_footprint" if self._trusted_pose_stamp is not None else None,
            "trusted_pose_stamp_sec": self._trusted_pose_stamp,
            "trusted_odom_anchor": self._wheel_odom_pose_at_trust,
            "wheel_odom_stamp_sec": self._wheel_odom_stamp,
            "pose_source_age_sec": self._source_age(self._evidence_stamp),
            "wheel_odom_source_age_sec": self._source_age(self._wheel_odom_stamp),
            "odom_propagated_trusted_pose": self.odom_propagated_trusted_pose(),
            "trusted_pose_has_odom_anchor": self._wheel_odom_pose_at_trust is not None,
            "wheel_odom_age_sec": time.monotonic() - self._last_wheel_odom_time if self._last_wheel_odom_time else None,
            "last_trusted_pose_age_sec": time.monotonic() - self._last_trusted_pose_time if self._last_trusted_pose_time else None,
            "initialpose_age_sec": initialpose_age,
            "initialpose_settling": self.initialpose_is_settling(),
            "initialpose_settle_sec": self._initialpose_settle_sec,
            "localization_fresh_good_evidence": self._localization_fresh_good_evidence_from_warning(effective_warning),
            "localization_requires_recovery": self._localization_requires_recovery_from_warning(effective_warning),
            "mode": context.mode if context else "unknown",
        }

    def health_summary(self, context: SkillContext | None = None) -> dict:
        localization = self.localization_metrics(context)
        local_planner = self.local_planner_metrics()
        safety_level = self._level_from_safety()
        mode_level = "OK" if self.robot_mode == "AUTO" else ("WARN" if self.robot_mode in {"MANUAL", "UNKNOWN"} else "WARN")
        wall_now = time.monotonic()
        mode_age = wall_now - self._last_robot_mode_time if self._last_robot_mode_time else None
        safety_age = wall_now - self._last_safety_time if self._last_safety_time else None
        mode_fresh = mode_age is not None and 0 <= mode_age <= 5.0
        safety_fresh = safety_age is not None and 0 <= safety_age <= 5.0
        localization_level = self._level_from_localization(localization)
        local_planner_level = self._level_from_local_planner(local_planner)
        components = {
            "mode": {
                "level": mode_level if mode_fresh else "BAD",
                "summary": (self.robot_mode or "UNKNOWN") if mode_fresh else "STALE",
                "details": {"robot_mode_age_sec": mode_age, "last_value": self.robot_mode},
            },
            "safety": {
                "level": safety_level if safety_fresh else "BAD",
                "summary": (self.safety_status or "UNKNOWN") if safety_fresh else "STALE",
                "details": {"safety_status_age_sec": safety_age, "last_value": self.safety_status},
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
        now_time = time.monotonic()
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

    def odom_propagated_trusted_pose(self) -> tuple[float, float, float] | None:
        if self._last_trusted_pose is None:
            return None
        if self._latest_wheel_odom_pose is None or self._wheel_odom_pose_at_trust is None:
            return self._last_trusted_pose

        if self.mode == "sim" and self._trusted_pose_stamp is not None:
            if self._reference.prediction(self._latest_wheel_odom_pose, time.monotonic()) is None:
                return None
        mx, my, myaw = self._last_trusted_pose
        ox0, oy0, oyaw0 = self._wheel_odom_pose_at_trust
        ox1, oy1, oyaw1 = self._latest_wheel_odom_pose
        dx_odom = ox1 - ox0
        dy_odom = oy1 - oy0
        dyaw = self._angle_diff(oyaw1, oyaw0)

        # Apply the wheel-odom relative motion in the trusted map pose frame.
        heading_offset = self._angle_diff(myaw, oyaw0)
        c = math.cos(heading_offset)
        s = math.sin(heading_offset)
        return (
            mx + c * dx_odom - s * dy_odom,
            my + s * dx_odom + c * dy_odom,
            self._angle_diff(myaw + dyaw, 0.0),
        )

    def initialpose_is_settling(self) -> bool:
        return bool((self._recovery_accumulating and time.monotonic() - self._recovery_status_received < 3)
                    or (self._last_initialpose_time and time.monotonic() - self._last_initialpose_time < self._initialpose_settle_sec))

    def _localization_receipt_stale(self, received: float, now_time: float) -> bool:
        # A slow simulation can deliver a fresh accepted match after >3 wall-s.
        # Keep a bounded wall watchdog AND a ROS-stamp budget; a paused clock
        # must never make evidence fresh indefinitely. Real robot keeps 3s.
        limit = self._localization_data_stale_sec
        if self.mode == "sim":
            age = self._source_age(self._evidence_stamp)
            if age is None:
                limit = min(limit, 3.0)
            elif not 0 <= age <= self._localization_source_stale_sec:
                return True
        return now_time - received > limit

    def _source_age(self, stamp):
        if stamp is None or self.node is None:
            return None
        return self.node.get_clock().now().nanoseconds / 1e9 - stamp

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
        if metrics.get("localization_healthy"):
            return "OK"
        if metrics.get("localization_fresh_good_evidence"):
            return "WARN"
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
        if warning in {
            "GICP_SCORE_BAD",
            "GICP_SCORE_WARN",
            "GICP_SCORE_MISSING",
            "GICP_SCORE_STALE",
            "GICP_POSE_STALE",
            "GICP_TRUSTED_POSE_DIVERGENCE",
            "POSSIBLE_STATIONARY_DRIFT",
            "RECOVERY_REFERENCE_UNVERIFIED",
            "UNKNOWN_ODOM",
        }:
            return "BAD"
        if warning in {"MANUAL_INITIALPOSE_SETTLING"}:
            return "WARN"
        return "UNKNOWN"

    def _localization_fresh_good_evidence_from_warning(self, warning: str) -> bool:
        now_time = time.monotonic()
        if self._latest_odom_pose is None or self._gicp_score is None:
            return False
        if self._localization_receipt_stale(self._last_gicp_score_time, now_time):
            return False
        if self._localization_receipt_stale(self._last_localization_pose_time, now_time):
            return False
        if self._gicp_score >= self._gicp_warn_threshold:
            return False
        return str(warning or "UNKNOWN") in {"OK", "MOVING", "MANUAL_INITIALPOSE_SETTLING"}

    def _localization_requires_recovery_from_warning(self, warning: str) -> bool:
        if self._gicp_score is not None and self._gicp_score >= self._gicp_bad_threshold:
            return True
        return str(warning or "UNKNOWN") in {
            "GICP_SCORE_BAD",
            "GICP_TRUSTED_POSE_DIVERGENCE",
            "POSSIBLE_STATIONARY_DRIFT",
            "RECOVERY_REFERENCE_UNVERIFIED",
        }

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
        if self.mode == "sim" and self._evidence_stamp is not None and not self._reference_observation_verified:
            return
        if self._evidence_stamp is not None and self._evidence_odom_anchor is None:
            return
        self._last_trusted_pose = (float(x), float(y), float(yaw))
        self._wheel_odom_pose_at_trust = self._evidence_odom_anchor if self._evidence_stamp is not None else self._latest_wheel_odom_pose
        self._trusted_pose_stamp = self._evidence_stamp
        self._last_trusted_pose_time = timestamp

    def _trusted_pose_delta(self) -> tuple[float | None, float | None]:
        if self._latest_wheel_odom_pose is None or self._wheel_odom_pose_at_trust is None:
            return None, None
        x, y, yaw = self._latest_wheel_odom_pose
        tx, ty, tyaw = self._wheel_odom_pose_at_trust
        return math.hypot(x - tx, y - ty), abs(self._angle_diff(yaw, tyaw))

    def _score_warning(self) -> str | None:
        now_time = time.monotonic()
        if self._gicp_score is None:
            return "MANUAL_INITIALPOSE_SETTLING" if self.initialpose_is_settling() else "GICP_SCORE_MISSING"
        if self._localization_receipt_stale(self._last_gicp_score_time, now_time):
            return "GICP_SCORE_STALE"
        # A fresh rejected/bad match explains why the last accepted GICP pose
        # became stale. Report the root cause instead of the downstream symptom.
        if self._gicp_score is not None and self._gicp_score >= self._gicp_bad_threshold:
            return "GICP_SCORE_BAD"
        if self._gicp_score is not None and self._gicp_score >= self._gicp_warn_threshold:
            return "GICP_SCORE_WARN"
        if (
            self._last_localization_pose_time
            and self._localization_receipt_stale(self._last_localization_pose_time, now_time)
        ):
            return "GICP_POSE_STALE"
        if self.initialpose_is_settling():
            return "MANUAL_INITIALPOSE_SETTLING"
        return None

    def await_robot_auto_mode(self, context: SkillContext | None = None, timeout_sec: float = 0.1) -> SkillResult:
        started = now()
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() <= deadline:
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
        deadline = time.monotonic() + max(0.0, timeout_sec)
        settle_started_at: float | None = None
        while time.monotonic() <= deadline:
            if context is not None and getattr(context, "cancel_check", lambda: False)():
                return make_result("await_robot_stationary", False, codes.CANCELLED,
                                   "Stationary wait cancelled.", started)
            received = self._last_cmd_receipt_time
            command_fresh = received > 0 and time.monotonic() - received <= self._stationary_cmd_fresh_sec
            cmd_speed_threshold = self._stationary_cmd_speed_threshold
            moving = self._last_cmd_speed > cmd_speed_threshold or (
                self._last_cmd_vel_time > 0.0
                and time.monotonic() - self._last_cmd_vel_time < self._stationary_cmd_fresh_sec
                and self._last_cmd_speed > 0.02
                and self.mode == "real"
            )
            if moving or (self.mode != "dry_run" and not command_fresh):
                settle_started_at = None
            else:
                if settle_started_at is None:
                    settle_started_at = time.monotonic()
                if time.monotonic() - settle_started_at >= max(0.0, settle_sec):
                    return make_result(
                        "await_robot_stationary",
                        True,
                        codes.OK,
                        "Robot is stationary.",
                        started,
                        {
                            "settle_sec": settle_sec,
                            "last_cmd_speed": self._last_cmd_speed,
                            "cmd_speed_threshold": cmd_speed_threshold,
                            "last_cmd_vel_age_sec": time.monotonic() - self._last_cmd_vel_time if self._last_cmd_vel_time else None,
                            "cmd_vel_receipt_age_sec": time.monotonic() - received if received else None,
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
                "cmd_speed_threshold": self._stationary_cmd_speed_threshold,
                "last_cmd_vel_age_sec": time.monotonic() - self._last_cmd_vel_time if self._last_cmd_vel_time else None,
                "cmd_vel_receipt_age_sec": time.monotonic() - self._last_cmd_receipt_time if self._last_cmd_receipt_time else None,
            },
        )

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

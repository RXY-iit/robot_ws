from __future__ import annotations

import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

from robot_skills import result_codes as codes
from robot_skills.executors.base import dry_run_result, make_result, now
from robot_skills.schemas import SkillContext, SkillResult


class LocalizationSkills:
    def __init__(self, node, system):
        self.node = node
        self.system = system
        self._cmd_vel_pub = node.create_publisher(Twist, "/cmd_vel_raw", 10)
        self._initialpose_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def check_localization_health(
        self,
        context: SkillContext,
        settle_timeout_sec: float = 2.0,
        allow_warmup_pass: bool = False,
    ) -> SkillResult:
        started = now()
        metrics = self.system.localization_metrics(context)
        if context.mode == "dry_run":
            return dry_run_result("check_localization_health", "Dry-run localization check accepted.", metrics)
        deadline = time.time() + max(0.0, float(settle_timeout_sec))
        while not self.system.localization_is_healthy() and time.time() < deadline:
            if allow_warmup_pass and self.system.localization_has_fresh_good_evidence(context):
                metrics = self.system.localization_metrics(context)
                metrics["warmup_pass"] = True
                return make_result(
                    "check_localization_health",
                    True,
                    codes.OK,
                    "Localization evidence is fresh and good; continuing while warm-up gates settle.",
                    started,
                    metrics,
                )
            if self.system.localization_requires_recovery(context):
                break
            time.sleep(0.05)
            metrics = self.system.localization_metrics(context)
        if self.system.localization_is_healthy():
            return make_result(
                "check_localization_health",
                True,
                codes.OK,
                "Localization health is OK.",
                started,
                metrics,
            )
        return make_result(
            "check_localization_health",
            False,
            codes.FAILED_LOCALIZATION,
            "Localization is not healthy enough for base motion.",
            started,
            metrics,
        )

    def recover_localization_spin(
        self,
        context: SkillContext,
        timeout_sec: float = 24.0,
        angular_z: float = 0.25,
    ) -> SkillResult:
        started = now()
        timeout_sec = max(12.0, min(float(timeout_sec), 45.0))
        angular_z = max(-0.5, min(float(angular_z), 0.5))
        if context.mode == "dry_run":
            return dry_run_result(
                "recover_localization_spin",
                "Dry-run localization recovery accepted.",
                {"timeout_sec": timeout_sec, "angular_z": angular_z, "motion": "stationary"},
            )
        if context.robot_mode != "AUTO":
            return make_result(
                "recover_localization_spin",
                False,
                codes.NEEDS_HUMAN,
                "Localization recovery requires AUTO mode. Press A / confirm AUTO, then retry.",
                started,
                self.system.localization_metrics(context),
            )
        recovery_pose = self.system.odom_propagated_trusted_pose()
        if recovery_pose is None:
            return make_result(
                "recover_localization_spin",
                False,
                codes.NEEDS_HUMAN,
                "No trusted localization pose is available for reset. Please relocalize manually.",
                started,
                self.system.localization_metrics(context),
            )

        self._publish_stop(repeat=3)
        self._publish_initialpose(recovery_pose)
        settle_cap_sec = 3.0 if context.mode == "sim" else 6.0
        settle_until = time.time() + min(settle_cap_sec, timeout_sec * 0.25)
        while time.time() < settle_until:
            self._publish_stop(repeat=1)
            if self.system.localization_is_healthy():
                break
            time.sleep(0.2)
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            self._publish_stop(repeat=1)
            time.sleep(0.2)
            if self.system.localization_is_healthy():
                self._publish_stop(repeat=3)
                metrics = self.system.localization_metrics(context)
                metrics["recovery_action"] = "initialpose_reset_stationary_pointcloud_settle"
                metrics["recovery_initialpose"] = recovery_pose
                return make_result(
                    "recover_localization_spin",
                    True,
                    codes.OK,
                    "Localization recovered after initialpose reset and stationary point-cloud settling.",
                    started,
                    metrics,
                )

        self._publish_stop(repeat=3)
        metrics = self.system.localization_metrics(context)
        metrics["recovery_action"] = "initialpose_reset_stationary_pointcloud_settle"
        metrics["recovery_initialpose"] = recovery_pose
        return make_result(
            "recover_localization_spin",
            False,
            codes.NEEDS_HUMAN,
            "Localization did not recover within timeout. Please confirm robot pose manually.",
            started,
            metrics,
        )

    def _publish_stop(self, repeat: int = 1):
        stop = Twist()
        for _ in range(max(1, repeat)):
            self._cmd_vel_pub.publish(stop)
            time.sleep(0.05)

    def _publish_initialpose(self, pose: tuple[float, float, float]):
        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        msg.pose.covariance[0] = 0.04
        msg.pose.covariance[7] = 0.04
        msg.pose.covariance[35] = 0.09
        self._initialpose_pub.publish(msg)

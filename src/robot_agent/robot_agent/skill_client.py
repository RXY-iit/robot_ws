from __future__ import annotations

from typing import Any
from dataclasses import replace
import threading

from robot_skills.executors.camera import CameraSkills
from robot_skills.executors.arm import ArmSkills
from robot_skills.executors.base import make_result
from robot_skills.executors.lift import LiftSkills
from robot_skills.executors.localization import LocalizationSkills
from robot_skills.executors.navigation import NavigationSkills
from robot_skills.executors.perception import PerceptionSkills
from robot_skills.executors.semantic import SemanticSkills
from robot_skills.executors.system import SystemSkills
from robot_skills.registry import SkillRegistry
from robot_skills import result_codes as codes
from robot_skills.schemas import Pose2D, SkillContext, SkillResult
from robot_skills.extensions import load_extensions, resolve_alias
from robot_agent.capabilities import capability_snapshot


class SkillClient:
    def __init__(self, node, mode: str = "sim"):
        self.node = node
        self.mode = mode
        self.registry = SkillRegistry()
        self.extensions = load_extensions()
        self._cancelled_runs = set()
        self.stopping = False
        self._cancel_lock = threading.Lock()
        self.system = SystemSkills(node, mode=mode)
        self.semantic = SemanticSkills()
        self.navigation = NavigationSkills(node, localization_metrics=self.system.localization_metrics)
        self.perception = PerceptionSkills(node)
        self.camera = CameraSkills(node)
        self.arm = ArmSkills(node)
        self.lift = LiftSkills(node)
        self.localization = LocalizationSkills(node, self.system)

    def list_skills(self) -> list[str]:
        return self.registry.list_names()

    def update_robot_mode(self, value: str):
        self.system.update_robot_mode(value)

    def update_safety_status(self, value: str):
        self.system.update_safety_status(value)

    def update_gicp_score(self, score: float):
        self.system.update_gicp_score(score)

    def update_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
        self.system.update_cmd_vel(linear_x, linear_y, angular_z)

    def update_nav2_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
        self.system.update_nav2_cmd_vel(linear_x, linear_y, angular_z)

    def update_local_planner_state(self, label: str, state_id: int | None = None):
        self.system.update_local_planner_state(label, state_id)

    def set_local_planner_required(self, required: bool, reason: str = ""):
        self.system.set_local_planner_required(required, reason)

    def update_odom_pose(self, x: float, y: float, yaw: float):
        self.system.update_odom_pose(x, y, yaw)

    def update_initialpose(self, *, operator_alignment=False, source="unknown"):
        self.system.update_initialpose(operator_alignment=operator_alignment, source=source)

    def context(self, run_id: str, max_duration_sec: float = 120.0) -> SkillContext:
        context = SkillContext(
            run_id=run_id,
            mode=self.mode,
            robot_mode=self.system.robot_mode,
            max_duration_sec=max_duration_sec,
        )
        context.cancel_check = lambda: self.is_cancelled(run_id)
        return context

    def is_cancelled(self, run_id):
        if getattr(self, "stopping", False):
            return True
        with self._cancel_lock:
            return run_id in self._cancelled_runs

    def capability_snapshot(self, domain, args):
        return capability_snapshot(self, domain, args)

    def execute_skill(self, skill_name: str, payload: dict[str, Any], context: SkillContext) -> SkillResult:
        if skill_name in self.extensions["aliases"]:
            target, payload = resolve_alias(self.extensions, skill_name, payload)
            result = self.execute_skill(target, payload, context)
            result.metrics["implementation_skill"] = target
            result.skill_name = skill_name
            return result
        if self.is_cancelled(context.run_id) and skill_name != "cancel_motion":
            return make_result(skill_name, False, codes.CANCELLED, "Execution was cancelled.")
        definition = self.registry.get(skill_name)
        if definition is None:
            return make_result(
                skill_name,
                False,
                codes.FAILED_UNSUPPORTED,
                f"Skill is not registered: {skill_name}",
            )
        allowed_modes = definition.get("allowed_modes", [])
        support_key = f"{context.mode}_supported"
        if (allowed_modes and context.mode not in allowed_modes) or definition.get(support_key) is False:
            return make_result(
                skill_name,
                False,
                codes.FAILED_UNSUPPORTED,
                f"Skill {skill_name} is disabled in {context.mode} mode.",
                metrics={"mode": context.mode, "support_key": support_key},
            )
        if skill_name in self.extensions["checks"]:
            domain = skill_name.removeprefix("check_").removesuffix("_capability")
            snapshot = self.capability_snapshot(domain, payload)
            return make_result(skill_name, True, codes.OK, "Read-only capability check completed; inspect supported/available/ready.", metrics=snapshot)
        if skill_name in self.extensions.get("composites", {}):
            from robot_skills.executors.observation import execute_observation
            return execute_observation(self, skill_name, payload, context)
        if skill_name == "get_robot_status":
            return self.system.get_robot_status(context)
        if skill_name == "await_robot_auto_mode":
            return self.system.await_robot_auto_mode(context, payload.get("timeout_sec", 0.1))
        if skill_name == "await_robot_stationary":
            return self.system.await_robot_stationary(
                context,
                payload.get("settle_sec", 1.0),
                payload.get("timeout_sec", context.max_duration_sec),
            )
        if skill_name == "check_localization_health":
            return self.localization.check_localization_health(
                context,
                payload.get("settle_timeout_sec", 2.0),
                payload.get("allow_warmup_pass", False),
            )
        if skill_name == "recover_localization_spin":
            return self.localization.recover_localization_spin(
                context,
                payload.get("timeout_sec", 24.0),
                payload.get("angular_z", 0.25),
            )
        if skill_name == "resolve_semantic_target":
            return self.semantic.resolve_semantic_target(
                context,
                target_name=payload["target_name"],
                relation=payload.get("relation", "near"),
            )
        if skill_name == "list_known_targets":
            return self.semantic.list_known_targets(context)
        if skill_name == "goto_pose":
            pose_data = payload["pose"]
            pose = pose_data if isinstance(pose_data, Pose2D) else Pose2D.from_dict(pose_data)
            return self.navigation.goto_pose(
                context,
                pose,
                payload.get("timeout_sec", 120.0),
                payload.get("xy_goal_tolerance_m"),
                payload.get("yaw_goal_tolerance_rad"),
                payload.get("tolerance_profile", "default"),
                payload.get("dynamic_timeout", False),
                payload.get("max_timeout_sec", 360.0),
                payload.get("recovery_policy"),
                apply_navigation_annotations=payload.get("apply_navigation_annotations", True),
            )
        if skill_name == "move_relative":
            return self.navigation.move_relative(
                context,
                payload.get("direction", "forward"),
                payload.get("distance_m", 0.5),
                payload.get("timeout_sec", 30.0),
            )
        if skill_name == "move_relative_xy":
            return self.navigation.move_relative_xy(
                context,
                payload.get("dx_m", 0.0),
                payload.get("dy_m", 0.0),
                payload.get("timeout_sec", 30.0),
            )
        if skill_name == "rotate_relative":
            return self.navigation.rotate_relative(
                context,
                payload.get("yaw_deg", 0.0),
                payload.get("timeout_sec", 30.0),
            )
        if skill_name == "cancel_motion":
            return self.navigation.cancel_navigation(context)
        if skill_name == "capture_observation":
            result = self.perception.capture_observation(
                self._perception_context(context),
                min_image_time_monotonic=payload.get("min_image_time_monotonic"),
            )
            for item in result.observations:
                item["view_id"] = payload.get("view_id", payload.get("scan_band"))
            return result
        if skill_name == "detect_known_object":
            return self.perception.detect_known_object(
                self._perception_context(context),
                payload.get("query", ""),
                payload.get("min_confidence", 0.5),
                payload.get("provider", "stub"),
            )
        if skill_name == "set_camera_pose":
            return self.camera.set_camera_pose(
                context,
                payload["pan_deg"],
                payload["tilt_deg"],
                payload.get("wait_timeout_sec", 1.0),
                payload.get("tolerance_deg", 2.0),
                payload.get("scan_band"),
                payload.get("require_feedback", False),
            )
        if skill_name == "move_lift_bounded":
            return self.lift.move_lift_bounded(
                context,
                payload["direction"],
                payload.get("duration_sec"),
                payload.get("distance_m"),
            )
        if skill_name == "set_pose":
            return self.arm.set_pose(
                context,
                payload.get("name", ""),
                payload.get("repeat", 1),
            )
        if skill_name == "touch":
            return self.arm.touch(
                context,
                payload.get("target", ""),
                payload.get("mode", "touch"),
                payload.get("detection_result"),
            )
        if skill_name == "grasp":
            return self.arm.grasp(
                context,
                payload.get("target", ""),
                payload.get("destination"),
            )
        raise ValueError(f"Unknown skill: {skill_name}")

    @staticmethod
    def _perception_context(context):
        scoped = replace(context, run_id=context.evidence_id or context.run_id)
        scoped.cancel_check = getattr(context, "cancel_check", lambda: False)
        return scoped

    def cancel_all(self, run_id: str) -> SkillResult:
        with self._cancel_lock:
            self._cancelled_runs.add(run_id)
        return self.stop_all(run_id)

    def stop_all(self, run_id: str) -> SkillResult:
        self.arm.cancel(self.context(run_id))
        return self.execute_skill("cancel_motion", {}, self.context(run_id))

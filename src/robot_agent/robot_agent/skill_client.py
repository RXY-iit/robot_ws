from __future__ import annotations

from typing import Any

from robot_skills.executors.camera import CameraSkills
from robot_skills.executors.lift import LiftSkills
from robot_skills.executors.localization import LocalizationSkills
from robot_skills.executors.navigation import NavigationSkills
from robot_skills.executors.perception import PerceptionSkills
from robot_skills.executors.semantic import SemanticSkills
from robot_skills.executors.system import SystemSkills
from robot_skills.registry import SkillRegistry
from robot_skills.schemas import Pose2D, SkillContext, SkillResult


class SkillClient:
    def __init__(self, node, mode: str = "sim"):
        self.node = node
        self.mode = mode
        self.registry = SkillRegistry()
        self.system = SystemSkills(node, mode=mode)
        self.semantic = SemanticSkills()
        self.navigation = NavigationSkills(node)
        self.perception = PerceptionSkills(node)
        self.camera = CameraSkills(node)
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

    def update_initialpose(self):
        self.system.update_initialpose()

    def context(self, run_id: str, max_duration_sec: float = 120.0) -> SkillContext:
        return SkillContext(
            run_id=run_id,
            mode=self.mode,
            robot_mode=self.system.robot_mode,
            max_duration_sec=max_duration_sec,
        )

    def execute_skill(self, skill_name: str, payload: dict[str, Any], context: SkillContext) -> SkillResult:
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
            return self.localization.check_localization_health(context)
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
            return self.perception.capture_observation(
                context,
                min_image_time_monotonic=payload.get("min_image_time_monotonic"),
            )
        if skill_name == "detect_known_object":
            return self.perception.detect_known_object(
                context,
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
            )
        if skill_name == "move_lift_bounded":
            return self.lift.move_lift_bounded(
                context,
                payload["direction"],
                payload.get("duration_sec"),
                payload.get("distance_m"),
            )
        raise ValueError(f"Unknown skill: {skill_name}")

    def cancel_all(self, run_id: str) -> SkillResult:
        return self.execute_skill("cancel_motion", {}, self.context(run_id))

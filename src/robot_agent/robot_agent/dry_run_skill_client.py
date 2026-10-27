from __future__ import annotations

from typing import Any

from robot_skills import result_codes as codes
from robot_skills.executors.base import dry_run_result, make_result, now
from robot_skills.executors.semantic import SemanticSkills
from robot_skills.registry import SkillRegistry
from robot_skills.schemas import Pose2D, SkillContext, SkillResult
from robot_skills.extensions import load_extensions, resolve_alias


class DryRunSkillClient:
    def __init__(self):
        self.mode = "dry_run"
        self.registry = SkillRegistry()
        self.semantic = SemanticSkills()
        self.robot_mode = "AUTO"
        self.safety_status = "OK"
        self.extensions = load_extensions()
        self._cancelled_runs = set()

    def list_skills(self) -> list[str]:
        return self.registry.list_names()

    def context(self, run_id: str, max_duration_sec: float = 120.0) -> SkillContext:
        return SkillContext(
            run_id=run_id,
            mode="dry_run",
            robot_mode=self.robot_mode,
            max_duration_sec=max_duration_sec,
        )

    def execute_skill(self, skill_name: str, payload: dict[str, Any], context: SkillContext) -> SkillResult:
        if self.is_cancelled(context.run_id) and skill_name != "cancel_motion":
            return make_result(skill_name, False, codes.CANCELLED, "Dry-run cancelled.")
        if skill_name in self.extensions["aliases"]:
            target, args = resolve_alias(self.extensions, skill_name, payload)
            result = self.execute_skill(target, args, context)
            result.metrics["implementation_skill"] = target
            result.skill_name = skill_name
            return result
        if skill_name in self.extensions["checks"]:
            return dry_run_result(skill_name, "Capability not checked against hardware.",
                                  {"supported": None, "available": None, "ready": None})
        if skill_name in self.extensions.get("composites", {}):
            from robot_skills.executors.observation import execute_observation
            return execute_observation(self, skill_name, payload, context)
        started = now()
        if skill_name == "get_robot_status":
            return make_result(
                "get_robot_status",
                True,
                codes.OK,
                "Dry-run robot status collected.",
                started,
                {"robot_mode": self.robot_mode, "safety_status": self.safety_status, "mode": "dry_run"},
            )
        if skill_name == "await_robot_auto_mode":
            return make_result(
                "await_robot_auto_mode",
                True,
                codes.OK,
                "Dry-run robot is AUTO.",
                started,
                {"robot_mode": self.robot_mode},
            )
        if skill_name == "await_robot_stationary":
            return make_result(
                "await_robot_stationary",
                True,
                codes.OK,
                "Dry-run robot is stationary.",
                started,
                {"settle_sec": payload.get("settle_sec", 1.0), "timeout_sec": payload.get("timeout_sec", 180.0)},
            )
        if skill_name == "check_localization_health":
            return dry_run_result(
                "check_localization_health",
                "Dry-run localization health accepted.",
                {"localization_healthy": True, "localization_warning": "OK"},
            )
        if skill_name == "recover_localization_spin":
            return dry_run_result(
                "recover_localization_spin",
                "Dry-run localization recovery accepted.",
                {"timeout_sec": payload.get("timeout_sec", 24.0), "angular_z": payload.get("angular_z", 0.25)},
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
            return dry_run_result(
                "goto_pose",
                "Dry-run navigation accepted.",
                {
                    "pose": pose.to_dict(),
                    "timeout_policy": "dynamic" if payload.get("dynamic_timeout") else "fixed",
                    "requested_timeout_sec": payload.get("timeout_sec", 120.0),
                    "dynamic_timeout": bool(payload.get("dynamic_timeout")),
                    "recovery_policy": payload.get("recovery_policy"),
                },
            )
        if skill_name == "move_relative":
            direction = str(payload.get("direction", "")).lower()
            distance_m = float(payload.get("distance_m", 0.0))
            if direction not in {"forward", "back", "left", "right"}:
                return make_result(
                    "move_relative",
                    False,
                    codes.FAILED_SCHEMA,
                    "direction must be one of: forward, back, left, right.",
                    started,
                    {"direction": direction},
                )
            if distance_m <= 0.0 or distance_m > 2.0:
                return make_result(
                    "move_relative",
                    False,
                    codes.FAILED_SCHEMA,
                    "distance_m must be in (0, 2.0].",
                    started,
                    {"distance_m": distance_m},
                )
            return dry_run_result(
                "move_relative",
                "Dry-run relative motion accepted.",
                {"direction": direction, "distance_m": distance_m},
            )
        if skill_name == "move_relative_xy":
            dx_m = float(payload.get("dx_m", 0.0))
            dy_m = float(payload.get("dy_m", 0.0))
            distance_m = (dx_m * dx_m + dy_m * dy_m) ** 0.5
            if distance_m <= 0.0 or distance_m > 2.0:
                return make_result(
                    "move_relative_xy",
                    False,
                    codes.FAILED_SCHEMA,
                    "relative xy distance must be in (0, 2.0].",
                    started,
                    {"dx_m": dx_m, "dy_m": dy_m, "distance_m": distance_m},
                )
            return dry_run_result(
                "move_relative_xy",
                "Dry-run relative xy motion accepted.",
                {"dx_m": dx_m, "dy_m": dy_m, "distance_m": distance_m},
            )
        if skill_name == "rotate_relative":
            yaw_deg = float(payload.get("yaw_deg", 0.0))
            if abs(yaw_deg) <= 0.0 or abs(yaw_deg) > 180.0:
                return make_result(
                    "rotate_relative",
                    False,
                    codes.FAILED_SCHEMA,
                    "yaw_deg must be in [-180, 180] and non-zero.",
                    started,
                    {"yaw_deg": yaw_deg},
                )
            return dry_run_result(
                "rotate_relative",
                "Dry-run relative rotation accepted.",
                {"yaw_deg": yaw_deg},
            )
        if skill_name == "cancel_motion":
            return make_result("cancel_motion", True, codes.OK, "Dry-run cancel accepted; no active goal tracked.", started)
        if skill_name == "capture_observation":
            return dry_run_result("capture_observation", "Dry-run observation capture accepted.", {"stub": True})
        if skill_name == "detect_known_object":
            return make_result(
                "detect_known_object",
                True,
                codes.PERCEPTION_STUB,
                f'Dry-run detector stub for "{payload.get("query", "")}".',
                started,
                {
                    "query": payload.get("query", ""),
                    "provider": payload.get("provider", "stub"),
                    "stub": True,
                },
            )
        if skill_name == "set_camera_pose":
            pan_deg = float(payload.get("pan_deg", 0.0))
            tilt_deg = float(payload.get("tilt_deg", 0.0))
            if not (-180.0 <= pan_deg <= 180.0 and -90.0 <= tilt_deg <= 132.0):
                return make_result(
                    "set_camera_pose",
                    False,
                    codes.FAILED_SCHEMA,
                    "Camera target angle is out of configured bounds.",
                    started,
                    {"pan_deg": pan_deg, "tilt_deg": tilt_deg},
                )
            return dry_run_result(skill_name, f"Dry-run {skill_name} accepted.", payload)
        if skill_name == "set_pose":
            return dry_run_result(
                "set_pose",
                f"Dry-run arm pose accepted: {payload.get('name', 'unknown')}.",
                {"name": payload.get("name"), "repeat": payload.get("repeat", 1)},
            )
        if skill_name == "touch":
            return dry_run_result(
                "touch",
                f"Dry-run touch accepted: {payload.get('target', 'target')}.",
                {"target": payload.get("target"), "mode": payload.get("mode", "touch")},
            )
        if skill_name == "grasp":
            return make_result(
                "grasp",
                False,
                codes.NEEDS_HUMAN,
                "Dry-run grasp requires a configured VLA endpoint.",
                started,
                {"target": payload.get("target"), "destination": payload.get("destination")},
            )
        if skill_name == "move_lift_bounded":
            direction = str(payload.get("direction", "")).upper()
            distance_m = payload.get("distance_m")
            if distance_m is not None:
                distance_m = float(distance_m)
                duration_sec = distance_m / 0.06
            else:
                duration_sec = float(payload.get("duration_sec", 0.0))
                distance_m = duration_sec * 0.06
            if direction not in {"UP", "DOWN"}:
                return make_result(
                    "move_lift_bounded",
                    False,
                    codes.FAILED_SCHEMA,
                    "direction must be UP or DOWN.",
                    started,
                    {"direction": direction},
                )
            if distance_m <= 0.0 or distance_m > 0.30 or duration_sec <= 0.0 or duration_sec > 5.0:
                return make_result(
                    "move_lift_bounded",
                    False,
                    codes.FAILED_SCHEMA,
                    "distance_m must be in (0, 0.30] and duration_sec must be in (0, 5.0].",
                    started,
                    {"distance_m": distance_m, "duration_sec": duration_sec},
                )
            metrics = dict(payload)
            metrics.update({"distance_m": distance_m, "duration_sec": duration_sec, "lift_speed_m_s": 0.06})
            return dry_run_result(skill_name, f"Dry-run {skill_name} accepted.", metrics)
        raise ValueError(f"Unknown skill: {skill_name}")

    def cancel_all(self, run_id: str) -> SkillResult:
        self._cancelled_runs.add(run_id)
        return self.stop_all(run_id)

    def is_cancelled(self, run_id):
        return run_id in self._cancelled_runs

    def stop_all(self, run_id):
        return self.execute_skill("cancel_motion", {}, self.context(run_id))

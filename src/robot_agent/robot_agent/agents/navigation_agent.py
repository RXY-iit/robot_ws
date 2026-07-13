from __future__ import annotations

from robot_agent.schemas import MissionIntent, SkillCall


class NavigationAgent:
    def build_plan(self, intent: MissionIntent) -> list[SkillCall]:
        if intent.target is None:
            return []
        return [
            SkillCall(
                name="resolve_semantic_target",
                payload={"target_name": intent.target, "relation": intent.relation or "near"},
                reason="Resolve natural-language target to map-frame pose.",
                timeout_sec=1.0,
            ),
            SkillCall(
                name="await_robot_auto_mode",
                payload={"timeout_sec": 0.1},
                reason="Human must press A / confirm AUTO before navigation.",
                timeout_sec=0.1,
                requires_human_confirm=True,
            ),
            SkillCall(
                name="check_localization_health",
                reason="Verify localization before sending a Nav2 goal.",
                timeout_sec=2.0,
            ),
            SkillCall(
                name="goto_pose",
                payload={
                    "pose": "__resolved_pose__",
                    "pose_candidates": "__resolved_pose_candidates__",
                    "recovery_policy": "__resolved_recovery_policy__",
                    "timeout_sec": 60.0,
                    "dynamic_timeout": True,
                    "max_timeout_sec": 60.0,
                    "xy_goal_tolerance_m": 0.25,
                    "yaw_goal_tolerance_rad": 0.45,
                    "tolerance_profile": "semantic_near",
                },
                reason="Navigate to resolved approach pose using Nav2.",
                timeout_sec=60.0,
            ),
        ]

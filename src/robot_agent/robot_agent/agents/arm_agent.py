from __future__ import annotations

from robot_agent.schemas import MissionIntent, SkillCall
from robot_agent.agents.domain import TaskRefinement
from robot_agent.task_catalog import TaskCatalog


class ArmAgent:
    def refine(self, intent, client, jev=None, mode="classic"):
        if intent.task_type == "manipulate_object":
            return TaskRefinement(question="Grasp/place/release policy and physical completion verification are not configured.",
                                  outcome="UNSUPPORTED")
        if intent.task_type == "interact_with_object":
            # Existing touch queues commands without an end-to-end result contract.
            # Do not promise a completed interaction or use the base RGB bbox as arm-camera evidence.
            return TaskRefinement(question="Touch/press requires the supervised Touch Anything workflow; autonomous task completion is not yet verified.",
                                  outcome="UNSUPPORTED")
        if intent.task_type == "move_lift":
            return TaskRefinement(question="Lift currently acknowledges a timed command, not verified completion. Use the supervised lift control.",
                                  outcome="UNSUPPORTED")
        legacy = TaskCatalog().legacy(intent)
        plan = self.build_plan(legacy)
        if not plan:
            return TaskRefinement(question="Specify a configured arm pose.")
        arm = getattr(client, "arm", None)
        if arm is not None and getattr(client, "mode", "sim") != "dry_run":
            error = arm.validate_pose(plan[0].payload["name"], getattr(client, "mode", "sim"))
            if error:
                return TaskRefinement(question=error, outcome="UNSUPPORTED")
        traces = []
        if jev is not None and mode in {"jev_primary", "jev_shadow"}:
            decision, trace = jev.domain_review("arm_task_review", intent,
                {"execute": "Use only this supported named-pose plan; the arm gateway still validates limits and completion.",
                 "ask_human": "The requested pose or repetition is ambiguous; ask the operator."}, "execute",
                {"supported_task": "set_arm_pose", "proposed_steps": [step.to_dict() for step in plan],
                 "unsupported": ["autonomous_grasp", "autonomous_place", "autonomous_touch"]})
            traces.append(trace)
            if mode == "jev_primary" and decision != "execute":
                return TaskRefinement(question="Please confirm the requested arm pose and repetition.", traces=traces)
        return TaskRefinement(plan=[SkillCall("await_robot_stationary", {"settle_sec": 1.0, "timeout_sec": 180.0})] + plan, traces=traces)

    def build_plan(self, intent: MissionIntent) -> list[SkillCall]:
        if intent.task_type == "set_pose":
            pose_name = intent.target or str(intent.constraints.get("pose", "")).strip()
            if not pose_name:
                return []
            return [
                SkillCall(
                    name="set_pose",
                    payload={
                        "name": self._normalize_pose_name(pose_name),
                        "repeat": int(intent.constraints.get("repeat", 1)),
                    },
                    reason="Play requested arm pose or gesture.",
                    timeout_sec=15.0,
                )
            ]

        if intent.task_type in {"touch_object", "press_object"}:
            target = intent.object_query or intent.target
            if not target:
                return []
            mode = "press" if intent.task_type == "press_object" else "touch"
            return self._touch_plan(target, mode)

        if intent.task_type in {"touch_object_near_place", "press_object_near_place"}:
            target = intent.object_query
            if not target:
                return []
            mode = "press" if intent.task_type == "press_object_near_place" else "touch"
            return self._touch_plan(target, mode)

        if intent.task_type in {"grasp_object", "grasp_object_near_place"}:
            target = intent.object_query or intent.target
            if not target:
                return []
            return [
                SkillCall(
                    name="grasp",
                    payload={
                        "target": target,
                        "destination": intent.constraints.get("destination"),
                    },
                    reason="Grasp requested target through the configured VLA policy endpoint.",
                    timeout_sec=60.0,
                    requires_human_confirm=True,
                )
            ]

        return []

    @staticmethod
    def _touch_plan(target: str, mode: str) -> list[SkillCall]:
        return [
            SkillCall(
                name="set_pose",
                payload={"name": "see_front", "repeat": 1},
                reason="Move arm camera to a front observation pose before touch target detection.",
                timeout_sec=15.0,
            ),
            SkillCall(
                name="capture_observation",
                payload={},
                reason="Capture an arm-camera observation for touch target detection.",
                timeout_sec=3.0,
            ),
            SkillCall(
                name="detect_known_object",
                payload={"query": target, "min_confidence": 0.5, "provider": "yolo11s_ocr"},
                reason="Detect target and choose a prompt point for Touch Anything.",
                timeout_sec=8.0,
            ),
            SkillCall(
                name="touch",
                payload={"target": target, "mode": mode},
                reason=f"Run Touch Anything {mode} sequence for the selected target.",
                timeout_sec=45.0,
                requires_human_confirm=True,
            ),
        ]

    @staticmethod
    def _normalize_pose_name(value: str) -> str:
        normalized = value.strip().lower().replace("-", "_").replace(" ", "_")
        aliases = {
            "arm_home": "home",
            "home_pose": "home",
            "check_ground": "see_ground",
            "look_ground": "see_ground",
            "ground": "see_ground",
            "look_front": "see_front",
            "front": "see_front",
            "look_left": "see_left",
            "left": "see_left",
            "look_right": "see_right",
            "right": "see_right",
            "wave_hello": "wave",
            "hello": "wave",
        }
        return aliases.get(normalized, normalized)

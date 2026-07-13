from __future__ import annotations

import json
import os
import re
import threading
import time
from typing import Callable

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from robot_agent.agents.navigation_agent import NavigationAgent
from robot_agent.agents.perception_agent import PerceptionAgent
from robot_agent.parsers.hybrid_parser import HybridParser
from robot_agent.schemas import MissionState, SkillCall, utc_now
from robot_skills import result_codes as codes
from robot_skills.executors.base import make_result


EventCallback = Callable[[dict], None]
StateCallback = Callable[[MissionState], None]


class MissionAgent:
    BASE_MOTION_SKILLS = {"goto_pose", "move_relative", "move_relative_xy", "rotate_relative"}
    HEALTH_CONTROL_SKILLS = {"check_localization_health", "recover_localization_spin", "cancel_motion", "get_robot_status"}

    def __init__(
        self,
        skill_client,
        event_cb: EventCallback | None = None,
        state_cb: StateCallback | None = None,
    ):
        self.skill_client = skill_client
        self.parser = HybridParser()
        self.navigation_agent = NavigationAgent()
        self.perception_agent = PerceptionAgent()
        self.tolerance_policy = self._load_tolerance_policy()
        self.event_cb = event_cb or (lambda event: None)
        self.state_cb = state_cb or (lambda state: None)
        self._cancel_lock = threading.Lock()
        self._cancelled_mission_ids: set[str] = set()

    def run(self, text: str, mode: str = "sim") -> MissionState:
        state = MissionState(user_text=text, mode=mode)
        self._state(state)
        self._event(state, "mission_received", {"text": text})
        state.status = "PARSE"
        state.intent = self.parser.parse(text)
        state.parser_trace = dict(getattr(self.parser, "last_metrics", {}) or {})
        self._state(state)
        self._event(
            state,
            "intent_parsed",
            {"intent": state.intent.to_dict(), "parser": state.parser_trace},
        )
        if state.intent.task_type == "ask_human":
            ambiguity = (state.intent.constraints or {}).get("ambiguity") if state.intent else None
            state.status = "NEEDS_HUMAN"
            if ambiguity and ambiguity.get("reason"):
                state.question_for_human = f"コマンドの意味を一意に決められません: {ambiguity.get('reason')}"
            else:
                state.question_for_human = "このコマンドを実行可能なタスクに変換できませんでした。別の言い方で入力してください。"
            state.final_message = state.question_for_human
            self._state(state)
            self._event(
                state,
                "mission_needs_human",
                {
                    "question": state.question_for_human,
                    "ambiguity": ambiguity,
                },
            )
            return state

        state.status = "PLAN"
        state.plan = self._build_plan(state)
        self._state(state)
        self._event(state, "plan_created", {"plan": [item.to_dict() for item in state.plan]})

        if not state.plan:
            state.status = "NEEDS_HUMAN"
            state.question_for_human = "このコマンドを実行可能なタスクに変換できませんでした。別の言い方で入力してください。"
            state.final_message = state.question_for_human
            self._state(state)
            self._event(state, "mission_needs_human", {"question": state.question_for_human})
            return state

        resolved_pose = None
        resolved_pose_candidates = None
        resolved_recovery_policy = None
        last_camera_completed_time = None
        state.status = "EXECUTE_STEP"
        self._state(state)
        index = 0
        recovery_retry_counts: dict[int, int] = {}
        tolerance_retry_counts: dict[int, int] = {}
        navigation_failed_pose_keys: dict[int, set[str]] = {}
        while index < len(state.plan):
            if self._is_cancelled(state.mission_id):
                return self._finish_cancelled(state)
            call = state.plan[index]
            state.current_step_index = index
            payload = dict(call.payload)
            if payload.get("pose") == "__resolved_pose__":
                if resolved_pose is None:
                    state.status = "FAILED_SKILL"
                    state.final_message = "ナビゲーション目標を解決できませんでした。"
                    self._state(state)
                    return state
                payload["pose"] = self._select_resolved_pose(
                    resolved_pose,
                    resolved_pose_candidates,
                    state,
                    exclude_pose_keys=navigation_failed_pose_keys.get(index),
                )
            if payload.get("pose_candidates") == "__resolved_pose_candidates__":
                if resolved_pose_candidates:
                    payload["pose_candidates"] = resolved_pose_candidates
                else:
                    payload.pop("pose_candidates", None)
            if payload.get("recovery_policy") == "__resolved_recovery_policy__":
                if resolved_recovery_policy:
                    payload["recovery_policy"] = resolved_recovery_policy
                else:
                    payload.pop("recovery_policy", None)
            if call.name == "capture_observation" and last_camera_completed_time is not None:
                payload.setdefault("min_image_time_monotonic", last_camera_completed_time)

            if not self._handle_health_before_step(state, call):
                return state
            state.status = "EXECUTE_STEP"

            context = self.skill_client.context(state.mission_id, call.timeout_sec)
            self._state(state)
            agent_name = self._agent_for_skill(call.name)
            self._event(
                state,
                "skill_started",
                {
                    "skill": call.name,
                    "agent": agent_name,
                    "step_index": index,
                    "payload": payload,
                    "reason": call.reason,
                },
            )
            try:
                result = self.skill_client.execute_skill(call.name, payload, context)
            except Exception as exc:
                state.status = "FAILED_SKILL"
                state.final_message = f"{call.name} の実行中に例外が発生しました: {exc!r}"
                self._state(state)
                self._event(state, "skill_exception", {"skill": call.name, "error": repr(exc)})
                return state

            result_dict = result.to_dict()
            result_dict["agent"] = agent_name
            result_dict["step_index"] = index
            state.skill_results.append(result_dict)
            self._state(state)
            self._event(state, "skill_finished", result_dict)

            if self._is_cancelled(state.mission_id):
                return self._finish_cancelled(state)

            if call.name == "resolve_semantic_target" and result.success:
                resolved_pose = result.metrics.get("pose")
                resolved_pose_candidates = result.metrics.get("approach_candidates")
                resolved_recovery_policy = result.metrics.get("approach_policy")
            if call.name == "set_camera_pose" and result.success:
                last_camera_completed_time = result.metrics.get("completed_time_monotonic")

            if result.code == codes.FAILED_LOCALIZATION:
                if self._run_health_recovery(state):
                    index += 1
                    continue
                return state

            if (
                call.name in {"goto_pose", "move_relative", "move_relative_xy", "rotate_relative"}
                and result.code in {codes.FAILED_NAVIGATION, codes.FAILED_TIMEOUT, codes.CANCELLED}
            ):
                if (
                    call.name == "goto_pose"
                    and result.code in {codes.FAILED_NAVIGATION, codes.FAILED_TIMEOUT}
                    and resolved_pose_candidates
                    and self._navigation_policy_allows(resolved_recovery_policy, "allow_alternate_goal", True)
                    and payload.get("pose")
                    and self._try_next_navigation_candidate(
                        state,
                        index,
                        payload.get("pose"),
                        resolved_pose,
                        resolved_pose_candidates,
                        navigation_failed_pose_keys,
                        result,
                    )
                ):
                    continue

                if (
                    call.name == "goto_pose"
                    and result.code == codes.FAILED_TIMEOUT
                    and tolerance_retry_counts.get(index, 0) < 1
                    and self._navigation_policy_allows(resolved_recovery_policy, "allow_relaxed_tolerance", True)
                    and not self._health_is_bad()
                ):
                    if self._try_adjust_navigation_tolerance(state, index, call, result):
                        tolerance_retry_counts[index] = tolerance_retry_counts.get(index, 0) + 1
                        continue

                if not self._health_is_bad():
                    state.status = "NEEDS_HUMAN" if tolerance_retry_counts.get(index, 0) >= 1 else "FAILED_SKILL"
                    if state.status == "NEEDS_HUMAN":
                        state.question_for_human = (
                            "Navigation timed out again after tolerance adjustment. Please inspect the robot or goal."
                        )
                    state.final_message = state.question_for_human or result.message
                    self._state(state)
                    event_type = "mission_needs_human" if state.status == "NEEDS_HUMAN" else "mission_failed"
                    payload_key = "question" if state.status == "NEEDS_HUMAN" else "reason"
                    self._event(
                        state,
                        event_type,
                        {
                            payload_key: state.final_message,
                            "code": result.code,
                            "health": self._health_summary(),
                            "navigation_metrics": result.metrics,
                        },
                    )
                    return state
                health = self._health_summary()
                if self._component_is_bad(health, "local_planner"):
                    if recovery_retry_counts.get(index, 0) < 2 and self._run_local_planner_recovery(state):
                        recovery_retry_counts[index] = recovery_retry_counts.get(index, 0) + 1
                        continue
                    return state
                if recovery_retry_counts.get(index, 0) < 1 and self._run_health_recovery(state):
                    recovery_retry_counts[index] = recovery_retry_counts.get(index, 0) + 1
                    continue
                return state

            if result.code == codes.NEEDS_HUMAN:
                state.status = "NEEDS_HUMAN"
                state.question_for_human = result.message
                state.final_message = result.message
                self._state(state)
                self._event(state, "mission_needs_human", {"question": result.message})
                return state

            if result.code == codes.PERCEPTION_STUB:
                state.status = "DONE_WITH_STUB"
                state.final_message = result.message
                self._state(state)
                self._event(
                    state,
                    "mission_done_with_stub",
                    {"message": result.message, "code": result.code},
                )
                return state

            if not result.success:
                state.status = "FAILED_SKILL"
                state.final_message = result.message
                self._state(state)
                self._event(state, "mission_failed", {"reason": result.message, "code": result.code})
                return state

            index += 1

        state.status = "DONE"
        state.final_message = self._final_message(state)
        state.updated_at = utc_now()
        self._state(state)
        self._event(state, "mission_done", {"message": state.final_message})
        return state

    def cancel(self, run_id: str):
        with self._cancel_lock:
            self._cancelled_mission_ids.add(run_id)
        return self.skill_client.cancel_all(run_id)

    def cancel_motion_only(self, run_id: str):
        return self.skill_client.cancel_all(run_id)

    def _is_cancelled(self, mission_id: str) -> bool:
        with self._cancel_lock:
            return mission_id in self._cancelled_mission_ids

    def _finish_cancelled(self, state: MissionState) -> MissionState:
        with self._cancel_lock:
            self._cancelled_mission_ids.discard(state.mission_id)
        state.status = "CANCELLED"
        state.final_message = "ユーザー操作によりタスクをキャンセルしました。"
        state.updated_at = utc_now()
        self._state(state)
        self._event(state, "mission_cancelled", {"message": state.final_message})
        return state

    def _handle_health_before_step(self, state: MissionState, call: SkillCall) -> bool:
        health = self._health_summary()
        if not health or call.name in self.HEALTH_CONTROL_SKILLS:
            return True
        if health.get("overall") != "BAD":
            return True
        components = health.get("components") or {}
        safety_bad = (components.get("safety") or {}).get("level") == "BAD"
        localization_bad = (components.get("localization") or {}).get("level") == "BAD"
        local_planner_bad = (components.get("local_planner") or {}).get("level") == "BAD"

        if safety_bad:
            self._cancel_for_health(state, health)
            state.status = "NEEDS_HUMAN"
            state.question_for_human = "Health check failed: safety is BAD. Please inspect the robot before continuing."
            state.final_message = state.question_for_human
            self._state(state)
            self._event(state, "mission_needs_human", {"question": state.question_for_human, "health": health})
            return False

        if localization_bad and call.name in self.BASE_MOTION_SKILLS:
            self._cancel_for_health(state, health)
            return self._run_health_recovery(state)

        if local_planner_bad and call.name in self.BASE_MOTION_SKILLS:
            self._cancel_for_health(state, health)
            return self._run_local_planner_recovery(state)

        if call.name in self.BASE_MOTION_SKILLS:
            self._cancel_for_health(state, health)
            state.status = "NEEDS_HUMAN"
            state.question_for_human = "Health check failed before base motion. Please inspect robot health."
            state.final_message = state.question_for_human
            self._state(state)
            self._event(state, "mission_needs_human", {"question": state.question_for_human, "health": health})
            return False
        return True

    def _run_health_recovery(self, state: MissionState) -> bool:
        state.status = "PAUSED_HEALTH"
        state.final_message = "Mission paused because robot health is BAD."
        self._state(state)
        self._event(state, "mission_paused_health", {"message": state.final_message, "health": self._health_summary()})

        state.status = "RECOVERING"
        state.final_message = "Running localization recovery."
        self._state(state)
        recovery_result = self._run_localization_recovery(state)
        if recovery_result.success:
            state.status = "RESUMING"
            state.final_message = "Health recovery completed; resuming mission."
            self._state(state)
            self._event(state, "mission_resuming", {"message": state.final_message})
            return True

        state.status = "NEEDS_HUMAN" if recovery_result.code == codes.NEEDS_HUMAN else "FAILED_SKILL"
        state.question_for_human = recovery_result.message if recovery_result.code == codes.NEEDS_HUMAN else None
        state.final_message = recovery_result.message
        self._state(state)
        event_type = "mission_needs_human" if recovery_result.code == codes.NEEDS_HUMAN else "mission_failed"
        self._event(state, event_type, {"reason": recovery_result.message, "code": recovery_result.code})
        return False

    def _run_local_planner_recovery(self, state: MissionState, timeout_sec: float = 30.0) -> bool:
        state.status = "PAUSED_HEALTH"
        state.final_message = "Mission paused because local planner health is BAD."
        self._state(state)
        self._event(state, "mission_paused_health", {"message": state.final_message, "health": self._health_summary()})

        deadline = time.monotonic() + max(1.0, timeout_sec)
        while time.monotonic() < deadline:
            if self._is_cancelled(state.mission_id):
                return False
            health = self._health_summary()
            local_planner = ((health or {}).get("components") or {}).get("local_planner") or {}
            if local_planner.get("level") != "BAD":
                state.status = "RESUMING"
                state.final_message = "Local planner recovered; retrying navigation step."
                self._state(state)
                self._event(
                    state,
                    "mission_resuming",
                    {
                        "message": state.final_message,
                        "recovery": "local_planner",
                        "local_planner": local_planner,
                    },
                )
                state.status = "EXECUTE_STEP"
                return True
            self._event(
                state,
                "local_planner_recovery_wait",
                {
                    "agent": "Health / Recovery Agent",
                    "local_planner": local_planner,
                    "timeout_sec": timeout_sec,
                },
            )
            time.sleep(1.0)

        health = self._health_summary()
        state.status = "NEEDS_HUMAN"
        state.question_for_human = "Local planner is still unavailable. Please restart or inspect Nav2 controller_server."
        state.final_message = state.question_for_human
        self._state(state)
        self._event(state, "mission_needs_human", {"question": state.question_for_human, "health": health})
        return False

    def _cancel_for_health(self, state: MissionState, health: dict):
        result = self.cancel_motion_only(state.mission_id)
        result_dict = result.to_dict()
        result_dict["agent"] = "Health / Recovery Agent"
        state.skill_results.append(result_dict)
        self._state(state)
        self._event(
            state,
            "health_cancel_requested",
            {"health": health, "cancel_result": result_dict},
        )

    def _health_summary(self) -> dict | None:
        system = getattr(self.skill_client, "system", None)
        if system is None or not hasattr(system, "health_summary"):
            return None
        return system.health_summary(self.skill_client.context("health_check", 1.0))

    def _health_is_bad(self) -> bool:
        health = self._health_summary()
        return bool(health and health.get("overall") == "BAD")

    @staticmethod
    def _component_is_bad(health: dict | None, component_name: str) -> bool:
        component = ((health or {}).get("components") or {}).get(component_name) or {}
        return component.get("level") == "BAD"

    def _build_plan(self, state: MissionState) -> list[SkillCall]:
        intent = state.intent
        if intent is None:
            return []
        if intent.task_type == "cancel":
            return [SkillCall(name="cancel_motion", reason="Cancel active motion.", timeout_sec=2.0)]
        if intent.task_type == "robot_status":
            return [SkillCall(name="get_robot_status", reason="Report robot status.", timeout_sec=3.0)]
        if intent.task_type == "list_known_targets":
            return [SkillCall(name="list_known_targets", reason="List semantic map targets.", timeout_sec=1.0)]
        if intent.task_type == "move_lift_bounded":
            return [
                SkillCall(
                    name="move_lift_bounded",
                    payload={
                        "direction": intent.constraints.get("direction", "UP"),
                        "distance_m": intent.constraints.get("distance_m", 0.05),
                        "duration_sec": intent.constraints.get("duration_sec"),
                    },
                    reason="Run bounded lift motion by requested distance with timer-based STOP.",
                    timeout_sec=6.0,
                )
            ]
        if intent.task_type == "move_relative":
            return self._relative_motion_preflight() + [
                SkillCall(
                    name="move_relative",
                    payload={
                        "direction": intent.constraints.get("direction", "forward"),
                        "distance_m": intent.constraints.get("distance_m", 0.5),
                        "timeout_sec": 30.0,
                    },
                    reason="Move relative to current base frame through Nav2.",
                    timeout_sec=30.0,
                ),
            ]
        if intent.task_type == "move_relative_xy":
            return self._relative_motion_preflight() + [
                SkillCall(
                    name="move_relative_xy",
                    payload={
                        "dx_m": intent.constraints.get("dx_m", 0.0),
                        "dy_m": intent.constraints.get("dy_m", 0.0),
                        "timeout_sec": 30.0,
                    },
                    reason="Move relative xy to current base frame through Nav2.",
                    timeout_sec=30.0,
                ),
            ]
        if intent.task_type == "rotate_relative":
            return self._relative_motion_preflight() + [
                SkillCall(
                    name="rotate_relative",
                    payload={
                        "yaw_deg": intent.constraints.get("yaw_deg", 0.0),
                        "timeout_sec": 30.0,
                    },
                    reason="Rotate relative to current map pose through Nav2.",
                    timeout_sec=30.0,
                ),
            ]
        if intent.task_type == "look_direction":
            return self._perception_preflight() + self.perception_agent.build_plan(intent)
        if intent.task_type in {"navigate_to_place", "navigate_to_object"}:
            plan = self.navigation_agent.build_plan(intent)
            if self._needs_visual_arrival_confirmation(intent.target):
                plan += self.perception_agent.build_arrival_confirmation_plan(intent.target)
            return plan
        if intent.task_type == "inspect_place":
            plan = self.navigation_agent.build_plan(intent)
            return plan + self.perception_agent.build_plan(intent)
        if intent.task_type == "search_object_near_place":
            plan = self.navigation_agent.build_plan(intent)
            return plan + self.perception_agent.build_plan(intent)
        if intent.task_type == "search_object_here":
            return self._perception_preflight() + self.perception_agent.build_search_plan(intent.object_query)
        return []

    @staticmethod
    def _relative_motion_preflight() -> list[SkillCall]:
        return [
            SkillCall(
                name="await_robot_auto_mode",
                payload={"timeout_sec": 0.1},
                reason="Human must press A / confirm AUTO before relative motion.",
                timeout_sec=0.1,
                requires_human_confirm=True,
            ),
            SkillCall(
                name="check_localization_health",
                reason="Verify localization before relative motion.",
                timeout_sec=2.0,
            ),
        ]

    @staticmethod
    def _perception_preflight() -> list[SkillCall]:
        return [
            SkillCall(
                name="await_robot_stationary",
                payload={"settle_sec": 1.0, "timeout_sec": 180.0},
                reason="Wait until base motion has stopped before moving the camera or capturing perception data.",
                timeout_sec=180.0,
            )
        ]

    @staticmethod
    def _needs_visual_arrival_confirmation(target: str | None) -> bool:
        if not target:
            return False
        normalized = target.lower().replace(" ", "")
        return any(
            token in normalized
            for token in [
                "shelf",
                "棚",
                "06",
                "05",
                "04",
                "03",
                "02",
                "01",
                "6号",
                "1号",
            ]
        )

    def _try_adjust_navigation_tolerance(self, state: MissionState, index: int, call: SkillCall, result) -> bool:
        selected = self._select_navigation_tolerance_profile(state, result.metrics or {})
        if selected is None:
            self._event(
                state,
                "navigation_tolerance_not_adjusted",
                {
                    "agent": "Health / Recovery Agent",
                    "reason": "No tolerance profile fits this timeout.",
                    "navigation_metrics": result.metrics,
                },
            )
            return False
        profile_name = selected["name"]
        if profile_name == "fine":
            return False
        xy_tolerance = selected.get("xy_goal_tolerance_m")
        yaw_tolerance = selected.get("yaw_goal_tolerance_rad")
        if not self._navigation_error_fits_profile(result.metrics or {}, xy_tolerance, yaw_tolerance):
            self._event(
                state,
                "navigation_tolerance_not_adjusted",
                {
                    "agent": "Health / Recovery Agent",
                    "reason": "Final navigation error is outside the selected profile.",
                    "selected_profile": selected,
                    "navigation_metrics": result.metrics,
                },
            )
            return False

        call.payload.update(
            {
                "tolerance_profile": profile_name,
                "xy_goal_tolerance_m": xy_tolerance,
                "yaw_goal_tolerance_rad": yaw_tolerance,
            }
        )
        if f"tolerance profile={profile_name}" not in call.reason:
            call.reason += f" Retry with tolerance profile={profile_name} after first timeout."
        state.status = "RECOVERING"
        state.final_message = f"Health / Recovery Agent adjusted navigation tolerance to {profile_name}; retrying step."
        self._state(state)
        self._event(
            state,
            "navigation_tolerance_adjusted",
            {
                "agent": "Health / Recovery Agent",
                "step_index": index,
                "skill": call.name,
                "selected_profile": selected,
                "navigation_metrics": result.metrics,
                "updated_payload": call.payload,
            },
        )
        state.status = "EXECUTE_STEP"
        return True

    def _select_navigation_tolerance_profile(self, state: MissionState, metrics: dict) -> dict | None:
        profiles = self.tolerance_policy.get("profiles") or {}
        intent_dict = state.intent.to_dict() if state.intent else {}
        target = str(intent_dict.get("target") or "")
        task_type = str(intent_dict.get("task_type") or "")
        for rule in self.tolerance_policy.get("rules") or []:
            if task_type not in set(rule.get("task_types") or []):
                continue
            if not self._target_matches_patterns(target, rule.get("target_patterns") or []):
                continue
            profile_name = str(rule.get("profile") or "")
            profile = profiles.get(profile_name)
            if isinstance(profile, dict):
                return {"name": profile_name, "source": "policy_db", "rule_id": rule.get("id"), **profile}

        llama_choice = self.parser.llama.choose_tolerance_profile(
            command=state.user_text,
            intent=intent_dict,
            navigation_metrics=metrics,
            profiles={key: profiles.get(key, {}) for key in ["fine", "medium", "loose"]},
        )
        if not llama_choice:
            return None
        profile_name = llama_choice["profile"]
        profile = profiles.get(profile_name)
        if not isinstance(profile, dict):
            return None
        return {"name": profile_name, "source": "llm_fallback", "llm_reason": llama_choice.get("reason"), **profile}

    @staticmethod
    def _target_matches_patterns(target: str, patterns: list[str]) -> bool:
        if not target:
            return False
        for pattern in patterns:
            try:
                if re.search(str(pattern), target, re.IGNORECASE):
                    return True
            except re.error:
                if str(pattern).lower() in target.lower():
                    return True
        return False

    @staticmethod
    def _navigation_error_fits_profile(
        metrics: dict,
        xy_goal_tolerance_m: float | None,
        yaw_goal_tolerance_rad: float | None,
    ) -> bool:
        xy_error = metrics.get("final_xy_error_m")
        yaw_error = metrics.get("final_yaw_error_rad")
        if xy_goal_tolerance_m is None and yaw_goal_tolerance_rad is None:
            return False
        if xy_goal_tolerance_m is not None:
            if xy_error is None or float(xy_error) > float(xy_goal_tolerance_m):
                return False
        if yaw_goal_tolerance_rad is not None:
            if yaw_error is None or float(yaw_error) > float(yaw_goal_tolerance_rad):
                return False
        return True

    @staticmethod
    def _load_tolerance_policy() -> dict:
        fallback_dirs = [
            os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "config")),
            os.path.abspath(os.path.join(os.getcwd(), "src", "robot_agent", "config")),
        ]
        try:
            config_dir = os.path.join(get_package_share_directory("robot_agent"), "config")
        except PackageNotFoundError:
            config_dir = fallback_dirs[0]
        path = os.path.join(config_dir, "navigation_tolerance_profiles.yaml")
        if not os.path.exists(path):
            for fallback_dir in fallback_dirs:
                candidate = os.path.join(fallback_dir, "navigation_tolerance_profiles.yaml")
                if os.path.exists(candidate):
                    path = candidate
                    break
        if not os.path.exists(path):
            return {"profiles": {}, "rules": []}
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {"profiles": {}, "rules": []}

    @staticmethod
    def _agent_for_skill(skill_name: str) -> str:
        if skill_name in {"resolve_semantic_target", "await_robot_auto_mode", "await_robot_stationary", "goto_pose", "cancel_motion"}:
            return "Navigation Agent"
        if skill_name in {"move_relative", "move_relative_xy", "rotate_relative"}:
            return "Navigation Agent"
        if skill_name in {"check_localization_health", "recover_localization_spin"}:
            return "Health / Recovery Agent"
        if skill_name in {"set_camera_pose", "capture_observation", "detect_known_object"}:
            return "Perception Agent"
        return "Mission Agent"

    def _run_localization_recovery(self, state: MissionState):
        call = SkillCall(
            name="recover_localization_spin",
            payload={"timeout_sec": 24.0, "angular_z": 0.25},
            reason="Recover localization before continuing mission.",
            timeout_sec=28.0,
        )
        context = self.skill_client.context(state.mission_id, call.timeout_sec)
        agent_name = self._agent_for_skill(call.name)
        self._event(
            state,
            "skill_started",
            {"skill": call.name, "agent": agent_name, "payload": call.payload, "reason": call.reason},
        )
        try:
            result = self.skill_client.execute_skill(call.name, call.payload, context)
        except Exception as exc:
            result = make_result(
                call.name,
                False,
                codes.FAILED_LOCALIZATION,
                f"{call.name} の実行中に例外が発生しました: {exc!r}",
            )
        result_dict = result.to_dict()
        result_dict["agent"] = agent_name
        state.skill_results.append(result_dict)
        self._state(state)
        self._event(state, "skill_finished", result_dict)
        return result

    def _event(self, state: MissionState, event_type: str, payload: dict):
        event = {
            "time": utc_now(),
            "mission_id": state.mission_id,
            "status": state.status,
            "type": event_type,
            "payload": payload,
        }
        self.event_cb(event)

    def _state(self, state: MissionState):
        state.updated_at = utc_now()
        self.state_cb(state)

    def _try_next_navigation_candidate(
        self,
        state: MissionState,
        index: int,
        attempted_pose,
        resolved_pose,
        candidates,
        failed_pose_keys: dict[int, set[str]],
        result,
    ) -> bool:
        attempted_key = self._pose_key(attempted_pose)
        if attempted_key is not None:
            failed_pose_keys.setdefault(index, set()).add(attempted_key)
        next_pose = self._select_resolved_pose(
            resolved_pose,
            candidates,
            state,
            exclude_pose_keys=failed_pose_keys.get(index),
        )
        next_key = self._pose_key(next_pose)
        if next_key is None or next_key == attempted_key:
            return False

        state.status = "RECOVERING"
        state.final_message = "Navigation failed near this candidate; retrying another semantic approach pose."
        self._state(state)
        self._event(
            state,
            "navigation_retry_alternate_pose",
            {
                "agent": "Navigation Agent",
                "step_index": index,
                "attempted_pose": attempted_pose,
                "next_pose": next_pose,
                "failed_pose_keys": sorted(failed_pose_keys.get(index, set())),
                "navigation_metrics": result.metrics,
            },
        )
        state.status = "EXECUTE_STEP"
        return True

    @staticmethod
    def _navigation_policy_allows(policy, key: str, default: bool) -> bool:
        if not isinstance(policy, dict):
            return default
        if str(policy.get("recovery_mode", "")).lower() == "fixed_point":
            return False
        if key not in policy:
            return default
        return bool(policy.get(key))

    @staticmethod
    def _select_resolved_pose(resolved_pose, candidates, state: MissionState, exclude_pose_keys: set[str] | None = None):
        if not candidates:
            return resolved_pose
        exclude_pose_keys = exclude_pose_keys or set()
        current_pose = MissionAgent._latest_pose_from_state(state)
        valid_candidates = []
        for candidate in candidates:
            pose = candidate.get("pose") if isinstance(candidate, dict) else None
            if not isinstance(pose, dict):
                continue
            pose_key = MissionAgent._pose_key(pose)
            if pose_key is not None and pose_key in exclude_pose_keys:
                continue
            distance = 0.0
            if current_pose is not None:
                try:
                    distance = ((float(pose["x"]) - current_pose[0]) ** 2 + (float(pose["y"]) - current_pose[1]) ** 2) ** 0.5
                except (KeyError, TypeError, ValueError):
                    continue
            valid_candidates.append((0 if candidate.get("preferred") else 1, distance, candidate))
        if not valid_candidates:
            return resolved_pose
        valid_candidates.sort(key=lambda item: (item[0], item[1]))
        return valid_candidates[0][2]["pose"]

    @staticmethod
    def _pose_key(pose) -> str | None:
        if not isinstance(pose, dict):
            return None
        try:
            return f"{float(pose['x']):.3f},{float(pose['y']):.3f},{float(pose.get('yaw', 0.0)):.3f}"
        except (KeyError, TypeError, ValueError):
            return None

    @staticmethod
    def _latest_pose_from_state(state: MissionState) -> tuple[float, float, float] | None:
        for result in reversed(state.skill_results):
            metrics = result.get("metrics") or {}
            pose = metrics.get("latest_odom_pose") or metrics.get("last_trusted_pose")
            if isinstance(pose, list) and len(pose) >= 3:
                try:
                    return float(pose[0]), float(pose[1]), float(pose[2])
                except (TypeError, ValueError):
                    return None
        return None

    @staticmethod
    def _final_message(state: MissionState) -> str:
        if state.intent and state.intent.task_type == "robot_status" and state.skill_results:
            return json.dumps(state.skill_results[-1].get("metrics", {}), ensure_ascii=False)
        if state.intent and state.intent.task_type == "list_known_targets" and state.skill_results:
            return json.dumps(state.skill_results[-1].get("metrics", {}), ensure_ascii=False)
        return "タスクが完了しました。"

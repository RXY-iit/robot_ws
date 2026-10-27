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
from robot_agent.agents.arm_agent import ArmAgent
from robot_agent.jev_agent import AGENT_MODES, JevStrengthenAgent, JevTrace
from robot_agent.parsers.hybrid_parser import HybridParser
from robot_agent.schemas import MissionIntent, MissionState, SkillCall, utc_now
from robot_agent.task_catalog import TaskCatalog
from robot_agent.recovery_supervision import RecoverySupervision
from robot_skills import result_codes as codes
from robot_skills.executors.base import make_result
from robot_skills.skill_catalog import SkillCatalog


EventCallback = Callable[[dict], None]
StateCallback = Callable[[MissionState], None]


class MissionAgent:
    BASE_MOTION_SKILLS = {"goto_pose", "move_relative", "move_relative_xy", "rotate_relative"}
    LOCALIZATION_DEPENDENT_MOTION_SKILLS = {"goto_pose"}
    HEALTH_CONTROL_SKILLS = {"check_localization_health", "recover_localization_spin", "cancel_motion", "get_robot_status"}

    def __init__(
        self,
        skill_client,
        event_cb: EventCallback | None = None,
        state_cb: StateCallback | None = None,
    ):
        self.skill_client = skill_client
        self.task_catalog = TaskCatalog()
        self.parser = HybridParser(defer_target_resolution=True)
        self.jev_agent = JevStrengthenAgent()
        self.agent_mode = "classic"
        self.navigation_agent = NavigationAgent()
        self.perception_agent = PerceptionAgent()
        self.arm_agent = ArmAgent()
        self.skill_catalog = SkillCatalog()
        self.skill_catalog_errors = self.skill_catalog.validate_registry()
        self.tolerance_policy = self._load_tolerance_policy()
        self.event_cb = event_cb or (lambda event: None)
        self.state_cb = state_cb or (lambda state: None)
        self._cancel_lock = threading.Lock()
        self._cancelled_mission_ids: set[str] = set()
        self._recovery_lock = threading.Condition()
        self._recovery_responses: dict[str, list[dict]] = {}
        self._recovery_pending: dict[str, str] = {}
        self._recovery_request_pub = None
        self.recovery_supervision = RecoverySupervision()
        self.enable_human_resume = False
        self.request_auto_mode = None
        self._human_condition = threading.Condition()
        self._human_wait = None
        self._human_action = None
        self._init_recovery_transport()

    def set_agent_mode(self, mode: str) -> str:
        normalized = str(mode or "").strip().lower()
        if normalized not in AGENT_MODES:
            normalized = "classic"
        self.agent_mode = normalized
        return self.agent_mode

    def run(self, text: str, mode: str = "sim") -> MissionState:
        state = MissionState(user_text=text, mode=mode)
        state.agent_mode = self.agent_mode
        jev_trace = JevTrace(agent_mode=self.agent_mode)
        self._state(state)
        self._event(state, "mission_received", {"text": text})
        state.status = "PARSE"
        self._state(state)
        comparison_parser = getattr(self.parser, "deterministic", self.parser) if state.agent_mode == "jev_primary" else self.parser
        classic_intent = self.task_catalog.normalize(comparison_parser.parse(text))
        classic_parser_trace = dict(getattr(self.parser, "last_metrics", {}) or {})
        if classic_intent.task_type == "follow_waypoint_list":
            # Exact named-route commands bind to registered data, never generated poses.
            state.intent = classic_intent
            state.parser_trace = {"source": "registered_waypoint_command"}
        elif state.agent_mode == "jev_primary":
            parse_decision = self.jev_agent.parse_command(text, clarify_task=lambda ordinal, candidates, item:
                self._clarify_task_interpretation(state, ordinal, candidates, item),
                is_cancelled=lambda: self._is_cancelled(state.mission_id))
            if state.status == "CANCELLED":
                return state
            state.intent = parse_decision.intent
            jev_trace.add(parse_decision.trace)
            for item in getattr(parse_decision, "additional_traces", []):
                jev_trace.add(item)
            state.parser_trace = {
                "source": "jev_primary",
                "accepted_layer": "JEV",
                "classic_task_type_for_plan_review": classic_intent.task_type,
                "classic_comparison_scope": "deterministic_only",
                "route_steps": [
                    {
                        "layer": "JEV",
                        "status": "accepted" if state.intent.task_type != "ask_human" else "needs_human",
                        "task_type": state.intent.task_type,
                        "reason": ([parse_decision.trace, *getattr(parse_decision, "additional_traces", [])][-1]).adopted_decision,
                    }
                ],
            }
        else:
            state.intent = classic_intent
            state.parser_trace = classic_parser_trace
            if state.agent_mode == "jev_shadow":
                parse_decision = self.jev_agent.parse_command(text)
                jev_trace.add(parse_decision.trace)
                for item in getattr(parse_decision, "additional_traces", []):
                    jev_trace.add(item)
                state.parser_trace["jev_shadow_task_type"] = parse_decision.intent.task_type
        state.parsed_intent = state.intent.to_dict() if state.intent else None
        state.jev_trace = jev_trace.to_dict()
        self._state(state)
        self._event(
            state,
            "intent_parsed",
            {"intent": state.intent.to_dict(), "parser": state.parser_trace},
        )
        if self._is_cancelled(state.mission_id):
            return self._finish_cancelled(state)
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
        self._state(state)
        state.plan = self._build_plan(state)
        self._annotate_plan_with_catalog(state.plan, self._health_summary())
        if state.agent_mode in {"jev_primary", "jev_shadow"}:
            classic_plan = []
            if state.agent_mode == "jev_primary":
                classic_plan = self._build_plan_for_intent(classic_intent, state)
                self._annotate_plan_with_catalog(classic_plan, self._health_summary())
            plan_review_trace = self.jev_agent.plan_review(
                text,
                state.intent,
                state.plan,
                self._health_summary(),
                classic_intent=classic_intent if state.agent_mode == "jev_primary" else None,
                classic_plan=classic_plan,
            )
            jev_trace.add(plan_review_trace)
            if self._is_cancelled(state.mission_id):
                state.jev_trace = jev_trace.to_dict()
                return self._finish_cancelled(state)
            if state.agent_mode == "jev_primary":
                decision = plan_review_trace.adopted_decision
                if decision == "use_classic_plan":
                    state.intent = classic_intent
                    state.plan = classic_plan
                    state.parser_trace["plan_review_selected"] = "classic"
                elif decision in {"ask_human", "ask_thinking_llm", "reject"}:
                    state.jev_trace = jev_trace.to_dict()
                    state.status = "NEEDS_HUMAN" if decision != "reject" else "FAILED_SKILL"
                    state.question_for_human = (
                        "JEV plan_review did not approve a robot plan. Please clarify or use Classic mode."
                        if decision != "reject"
                        else None
                    )
                    state.final_message = state.question_for_human or "JEV plan_review rejected the plan as unsafe or unsupported."
                    self._state(state)
                    event_type = "mission_needs_human" if state.question_for_human else "mission_failed"
                    self._event(
                        state,
                        event_type,
                        {"decision": decision, "jev_trace": plan_review_trace.to_dict()},
                    )
                    return state
                else:
                    state.parser_trace["plan_review_selected"] = "jev"
            state.jev_trace = jev_trace.to_dict()
        state.initial_plan = [item.to_dict() for item in state.plan]
        self._state(state)
        self._event(state, "plan_created", {"plan": state.initial_plan})

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
        last_detection_result = None
        state.status = "EXECUTE_STEP"
        self._state(state)
        index = 0
        recovery_retry_counts: dict[int, int] = {}
        supervision_pause_counts: dict[int, int] = {}
        jev_retry_counts: dict[int, int] = {}
        tolerance_retry_counts: dict[int, int] = {}
        navigation_failed_pose_keys: dict[int, set[str]] = {}
        while index < len(state.plan):
            if self._is_cancelled(state.mission_id):
                return self._finish_cancelled(state)
            call = state.plan[index]
            state.current_step_index = index
            if call.kind == "task":
                if not self._expand_task(state, index, jev_trace):
                    return state
                # Each task gets its own evidence epoch, even within one Mission.
                resolved_pose = resolved_pose_candidates = resolved_recovery_policy = None
                last_camera_completed_time = last_detection_result = None
                continue
            if call.kind == "task_end":
                self._complete_task(state, call.task_id)
                index += 1
                continue
            self._annotate_call_with_catalog(call, self._health_summary())
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
                    None if call.catalog.get("navigation_contract", {}).get("relation", "near") != "near" else resolved_pose_candidates,
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
            if call.catalog.get("navigation_contract", {}).get("relation", "near") != "near":
                payload.pop("pose_candidates", None)
            if call.name == "capture_observation" and last_camera_completed_time is not None:
                payload.setdefault("min_image_time_monotonic", last_camera_completed_time)
            if call.name == "touch" and last_detection_result is not None:
                payload.setdefault("detection_result", last_detection_result)

            if not self._wait_for_capability_recovery(state, call):
                return state
            if not self._handle_health_before_step(state, call):
                if state.status == "NEEDS_HUMAN" and self._wait_for_step_repair(state, call, recovery_resume=True):
                    continue
                return state
            state.status = "EXECUTE_STEP"

            context = self.skill_client.context(state.mission_id, call.timeout_sec)
            context.evidence_id = call.task_id or state.mission_id
            recovery_interruptions = []
            if call.name == "goto_pose":
                def recovery_check(interruptions=recovery_interruptions):
                    blockers = self.recovery_supervision.blockers(["auto_navigation"])
                    if blockers:
                        interruptions[:] = blockers
                    return bool(blockers)
                context.recovery_check = recovery_check
            if call.name == "goto_pose" and call.catalog.get("navigation_contract"):
                context.navigation_review = self._navigation_review_callback(state, call, jev_trace)
            self._state(state)
            agent_name = {"mission": "Mission Agent", "navigation": "Navigation Agent",
                          "perception": "Perception Agent", "arm": "Arm Agent"}.get(call.owner, self._agent_for_skill(call.name))
            self._event(
                state,
                "skill_started",
                {
                    "skill": call.name,
                    "agent": agent_name,
                    "step_index": index,
                    "payload": payload,
                    "reason": call.reason,
                    "task_id": call.task_id,
                    "step_id": call.step_id,
                    "owner": call.owner,
                },
            )
            try:
                result = self.skill_client.execute_skill(call.name, payload, context)
            except Exception as exc:
                if call.name == "goto_pose":
                    # An exception can occur after Nav2 accepted the goal.
                    # A failed Mission must not leave that goal running silently.
                    try:
                        cancel_result = self.skill_client.execute_skill("cancel_motion", {}, context)
                        self._event(state, "navigation_exception_cancel_requested", {
                            "result": cancel_result.to_dict(), "error": repr(exc)})
                    except Exception as cancel_exc:
                        self._event(state, "navigation_exception_cancel_failed", {"error": repr(cancel_exc)})
                if self._is_cancelled(state.mission_id):
                    return self._finish_cancelled(state)
                state.status = "FAILED_SKILL"
                state.final_message = f"{call.name} の実行中に例外が発生しました: {exc!r}"
                self._state(state)
                self._event(state, "skill_exception", {"skill": call.name, "error": repr(exc)})
                return state

            result_dict = result.to_dict()
            result_dict["agent"] = agent_name
            result_dict["step_index"] = index
            result_dict.update(task_id=call.task_id, step_id=call.step_id, owner=call.owner)
            state.skill_results.append(result_dict)
            if call.owner == "navigation" and call.name == "goto_pose":
                record = next((t for t in state.tasks if t["task_id"] == call.task_id), None)
                if record is not None:
                    record["navigation_arrival"] = self.navigation_agent.arrival_feedback(result_dict)
            self._state(state)
            self._event(state, "skill_finished", result_dict)
            if self._is_cancelled(state.mission_id):
                return self._finish_cancelled(state)
            if recovery_interruptions and not result.success:
                # Recovery interrupted the step before a task failure was needed.
                # Do not classify this as a planning failure or change its goal.
                self._event(state, "recovery_task_interrupted", {
                    "step_index": index, "skill": call.name, "constraints": recovery_interruptions})
                resume_allowed = supervision_pause_counts.get(index, 0) < 2
                supervision_pause_counts[index] = supervision_pause_counts.get(index, 0) + 1
                if not self._wait_for_capability_recovery(state, call, interrupted=recovery_interruptions,
                                                          resume_allowed=resume_allowed):
                    return state
                continue
            # A bounded Navigation retry is handled locally before fault escalation.
            if (call.name == "goto_pose" and result.code in {codes.FAILED_NAVIGATION, codes.FAILED_TIMEOUT}
                    and resolved_pose_candidates
                    and self._navigation_policy_allows(payload.get("recovery_policy"), "allow_alternate_goal", True)
                    and payload.get("pose")
                    and self._try_next_navigation_candidate(state, index, payload["pose"], resolved_pose,
                        resolved_pose_candidates, navigation_failed_pose_keys, result)):
                self._event(state, "step_supervision_skipped", {"step_index": index,
                            "reason": "navigation_retry_alternate_pose", "handled_by": "navigation_policy"})
                continue
            if (call.name == "goto_pose" and result.code == codes.FAILED_TIMEOUT
                    and tolerance_retry_counts.get(index, 0) < 1
                    and self._navigation_policy_allows(payload.get("recovery_policy"), "allow_relaxed_tolerance", True)
                    and not self._health_is_bad()
                    and self._try_adjust_navigation_tolerance(state, index, call, result)):
                tolerance_retry_counts[index] = tolerance_retry_counts.get(index, 0) + 1
                self._event(state, "step_supervision_skipped", {"step_index": index,
                            "reason": "navigation_tolerance_retry", "handled_by": "navigation_policy"})
                continue
            supervision = None
            if state.agent_mode in {"jev_primary", "jev_shadow"} and self._should_call_jev_supervision(result_dict):
                domain = {"perception": self.perception_agent, "navigation": self.navigation_agent}.get(call.owner)
                supervisor = (lambda **kw: domain.supervise_step(self.jev_agent, **kw)) if domain else self.jev_agent.supervise_step
                supervision = supervisor(
                    user_text=state.user_text,
                    intent=state.intent,
                    plan=state.plan,
                    step_index=index,
                    result=result_dict,
                    health=self._health_summary(),
                )
                supervision.trace.task_id = call.task_id
                supervision.trace.step_id = call.step_id
                supervision.trace.step_index = index
                supervision.trace.skill_name = call.name
                jev_trace.add(supervision.trace)
                state.jev_trace = jev_trace.to_dict()
                self._event(
                    state,
                    "jev_supervision_decision",
                    {
                        "step_index": index,
                        "skill": call.name,
                        "action": supervision.action,
                        "decision_scope": "proposal_pending_execution_policy" if state.agent_mode == "jev_primary" else "shadow_only",
                        "message": supervision.message,
                        "human_choices": supervision.human_choices,
                        "trace": supervision.trace.to_dict(),
                    },
                )
            self._state(state)

            if self._is_cancelled(state.mission_id):
                return self._finish_cancelled(state)

            if result.code == codes.NEEDS_HUMAN and isinstance(result.metrics.get("navigation_review"), dict):
                # A deliberate pre-dispatch hold is not an execution failure. Recheck
                # the same goal; its arrival-confirmation child cannot run yet.
                state.status = "NEEDS_HUMAN"
                state.question_for_human = result.message + " Recheck this navigation step after resolving the issue, or cancel all."
                state.final_message = state.question_for_human
                self._state(state)
                self._event(state, "mission_needs_human", {"question": state.question_for_human,
                            "navigation_review": result.metrics["navigation_review"]})
                if self._wait_for_step_repair(state, call):
                    continue
                return state

            if (result.code == codes.NEEDS_HUMAN and call.name == "await_robot_auto_mode"
                    and self.enable_human_resume):
                state.question_for_human = "AUTO mode is required. Change to AUTO here or with the controller; this mission will continue after verification."
                if self._wait_for_step_repair(state, call, automatic_auto=True):
                    continue
                return state

            if call.name == "resolve_semantic_target" and result.success:
                resolved_pose = result.metrics.get("pose")
                resolved_pose_candidates = result.metrics.get("approach_candidates")
                resolved_recovery_policy = result.metrics.get("approach_policy")
            if call.name == "set_camera_pose" and result.success:
                last_camera_completed_time = result.metrics.get("completed_time_monotonic")
            if call.name == "detect_known_object" and result.success:
                last_detection_result = result.metrics

            if result.code == codes.FAILED_LOCALIZATION:
                if recovery_retry_counts.get(index, 0) >= 1 or call.name not in {"goto_pose", "check_localization_health"}:
                    state.status = "NEEDS_HUMAN"
                    state.question_for_human = "Localization remains unresolved or this action cannot be safely replayed."
                    state.final_message = state.question_for_human
                    self._state(state)
                    if call.name in {"goto_pose", "check_localization_health"} and self._wait_for_step_repair(state, call):
                        continue
                    return state
                if state.agent_mode == "jev_primary" and supervision is not None and supervision.action == "call_recovery_agent":
                    if self._run_recovery_for_supervision(state, supervision, call, result_dict, index):
                        recovery_retry_counts[index] = recovery_retry_counts.get(index, 0) + 1
                        continue
                    if state.status == "NEEDS_HUMAN" and self._wait_for_step_repair(state, call, recovery_resume=True):
                        continue
                    return state
                if self._localization_requires_recovery_from_metrics(result.metrics):
                    if self._run_health_recovery(state, call=call, result=result_dict):
                        recovery_retry_counts[index] = recovery_retry_counts.get(index, 0) + 1
                        continue
                    if state.status == "NEEDS_HUMAN" and self._wait_for_step_repair(state, call, recovery_resume=True):
                        continue
                    return state
                state.status = "NEEDS_HUMAN"
                state.question_for_human = (
                    "Localization is not ready for base motion, but no hard drift/bad-score recovery condition was detected. "
                    "Wait for fresh GICP pose/score evidence or set the pose manually, then retry."
                )
                state.final_message = state.question_for_human
                self._state(state)
                self._event(
                    state,
                    "mission_needs_human",
                    {
                        "question": state.question_for_human,
                        "code": result.code,
                        "localization_metrics": result.metrics,
                    },
                )
                if self._wait_for_step_repair(state, call, recovery_resume=True):
                    continue
                return state

            if (not result.success and result.code != codes.PERCEPTION_STUB
                    and state.agent_mode == "jev_primary" and supervision is not None):
                action = self._apply_primary_failure_decision(
                    state, call, result_dict, index, supervision, jev_retry_counts, recovery_retry_counts)
                if action == "retry":
                    continue
                if action == "next":
                    result_dict["continued_after_failure"] = True
                    index = self._after_failed_task(state, index)
                    continue
                return state

            if (
                call.name in {"goto_pose", "move_relative", "move_relative_xy", "rotate_relative"}
                and result.code in {codes.FAILED_NAVIGATION, codes.FAILED_TIMEOUT, codes.CANCELLED}
            ):
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
                if self._explicit_cleanup_after_not_found(state, index, result_dict):
                    result_dict["continued_after_failure"] = True
                    self._event(state, "subgoal_not_found_continue_cleanup", {"step_index": index,
                                "reason": "Explicit user instruction; detection completed, target not confirmed"})
                    index = self._after_failed_task(state, index)
                    continue
                state.status = "NEEDS_HUMAN"
                state.question_for_human = result.message
                state.final_message = result.message
                self._state(state)
                self._event(state, "mission_needs_human", {"question": result.message})
                if call.name in {"await_robot_stationary", "check_localization_health", "check_capability"}:
                    if self._wait_for_step_repair(state, call):
                        continue
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
        if any(t.get("goal_satisfied") is False and t.get("condition_result") != "not_found" for t in state.tasks):
            state.outcome = "PARTIAL"
            state.final_message = "Execution ended with unmet task goals; inspect task results."
        else:
            state.outcome = "SATISFIED"
            state.final_message = ("Target not found; conditional task skipped (stay-still branch)."
                                   if any(t.get("outcome") == "CONDITION_FALSE" for t in state.tasks)
                                   else self._final_message(state))
        state.updated_at = utc_now()
        self._state(state)
        self._event(state, "mission_done", {"message": state.final_message})
        return state

    def cancel(self, run_id: str):
        with self._cancel_lock:
            self._cancelled_mission_ids.add(run_id)
        with self._human_condition:
            self._human_condition.notify_all()
        return self.skill_client.cancel_all(run_id)

    def human_action(self, command: dict) -> bool:
        with self._human_condition:
            wait = self._human_wait
            if (not wait or command.get("mission_id") != wait["mission_id"]
                    or command.get("wait_id") != wait["wait_id"]
                    or command.get("action") not in wait["actions"]
                    or self._is_cancelled(wait["mission_id"])):
                return False
            self._human_action = command["action"]
            self._human_condition.notify_all()
            return True

    def _apply_primary_failure_decision(self, state, call, result, index, supervision,
                                       retry_counts, recovery_counts):
        action = supervision.action
        message = supervision.message or result.get("message") or "Step failed; choose how to proceed."
        # Relative motion may already have travelled part of its target. Replaying
        # the original payload is not resuming the remaining distance/angle.
        replayable = call.name not in {"move_relative", "move_relative_xy", "rotate_relative"}
        next_call = self.jev_agent._next_meaningful_call(state.plan, index)
        independent_next = (next_call is not None
                            and self.jev_agent._next_step_still_meaningful(state.intent, next_call))
        if result.get("code") == codes.CANCELLED:
            action = "ask_human_continue_retry_stop"
        if action in {"continue_next_step", "skip_failed_subgoal_continue_independent_goal"}:
            if independent_next or self._explicit_cleanup_after_not_found(state, index, result):
                self._event(state, "jev_supervision_applied", {"step_id": call.step_id, "action": "continue_independent_task"})
                return "next"
            action = "ask_human_continue_retry_stop"
        if action == "call_recovery_agent":
            if recovery_counts.get(index, 0) < 1:
                recovery_counts[index] = recovery_counts.get(index, 0) + 1
                recovered = self._run_recovery_for_supervision(state, supervision, call, result, index)
                if not recovered:
                    message = state.final_message or message
                if not recovered and state.status != "NEEDS_HUMAN":
                    return "stop"
                if self._is_cancelled(state.mission_id):
                    self._finish_cancelled(state)
                    return "stop"
                if recovered and replayable:
                    self._event(state, "jev_supervision_applied", {"step_id": call.step_id, "action": "recheck_after_recovery"})
                    return "retry"
            if recovery_counts.get(index, 0) >= 1 and replayable:
                message += " Automatic recovery budget is exhausted; inspect the fault before explicitly retrying."
            action = "ask_human_continue_retry_stop"
        if action == "retry_with_alternate_view":
            message += " No alternate-view retry plan is configured; choose an explicit recheck or cancel."
            action = "ask_human_continue_retry_stop"
        if action in {"retry_same_step", "retry_with_alternate_view"}:
            if replayable and retry_counts.get(index, 0) < 1:
                retry_counts[index] = retry_counts.get(index, 0) + 1
                self._event(state, "jev_supervision_applied", {"step_id": call.step_id, "action": "retry_same_step"})
                return "retry"
            action = "ask_human_continue_retry_stop"
        if action in {"ask_human_continue_retry_stop", "ask_human_enable_auto", "record_design_gap"}:
            state.status = "NEEDS_HUMAN"
            state.question_for_human = state.final_message = (
                message)
            if not replayable:
                state.question_for_human = state.final_message = (
                    "Relative motion stopped and may be partially complete. The original motion will not be replayed. "
                    "Cancel all or explicitly continue an independent task.")
            self._state(state)
            self._event(state, "mission_needs_human", {"question": state.question_for_human,
                        "jev_action": supervision.action, "step_id": call.step_id})
            choices = []
            if replayable:
                choices.append({"action": "retry_failed_step", "label": "Recheck / retry current step"})
            if independent_next:
                choices.append({"action": "continue_after_failed_task",
                                "label": f"Leave this task unmet; continue {next_call.name} with safety checks"})
            if self.enable_human_resume and choices:
                selected = self._wait_for_domain_choice(state, call, choices)
                if selected and not self._is_cancelled(state.mission_id):
                    state.status = "EXECUTE_STEP"
                    state.question_for_human = state.final_message = None
                    for task in state.tasks:
                        if task["state"] == "WAITING_HUMAN":
                            task.update(state="RUNNING", goal_satisfied=None, outcome=None)
                    if selected["action"] == "continue_after_failed_task" and next_call is not None:
                        next_call.catalog["failure_continuation_approval"] = {
                            "mission_id": state.mission_id, "failed_task_id": call.task_id}
                    self._state(state)
                    return "retry" if selected["action"] == "retry_failed_step" else "next"
            return "stop"
        state.status = "FAILED_SKILL"
        state.final_message = result.get("message") or "Step supervision stopped this mission."
        self._state(state)
        self._event(state, "mission_failed", {"reason": state.final_message, "code": result.get("code")})
        return "stop"

    def _wait_for_step_repair(self, state, call, automatic_auto=False, recovery_resume=False):
        recovery_resume = (recovery_resume and call.name in {"goto_pose", "check_localization_health"}
                           and call.catalog.get("recovery_verified_resume_count", 0) < 2)
        if not self.enable_human_resume:
            return False
        waiting_since = time.monotonic()
        verified_since = None
        from uuid import uuid4
        wait = {"mission_id": state.mission_id, "wait_id": uuid4().hex,
                "step_id": call.step_id, "skill_name": call.name,
                "actions": ["set_auto"] if automatic_auto and self.request_auto_mode else ["resume"],
                "auto_resume": automatic_auto}
        wait["recovery_resume_allowed"] = recovery_resume
        state.status = "NEEDS_HUMAN"
        state.human_wait = wait
        state.final_message = state.question_for_human
        with self._human_condition:
            self._human_wait, self._human_action = wait, None
        self._state(state)
        self._event(state, "mission_waiting_for_repair", dict(wait))
        try:
            while not self._is_cancelled(state.mission_id):
                with self._human_condition:
                    action, self._human_action = self._human_action, None
                if action:
                    self._event(state, "mission_human_action", {**wait, "action": action})
                if action == "set_auto":
                    # Explicit operator approval only. Publishing is not evidence of AUTO.
                    try:
                        with self._cancel_lock:
                            if state.mission_id in self._cancelled_mission_ids:
                                break
                            self.request_auto_mode()
                    except Exception as exc:
                        self._event(state, "mission_control_failed", {"action": action, "error": str(exc)})
                system = getattr(self.skill_client, "system", None)
                mode = getattr(system, "robot_mode", None)
                mode_stamp = getattr(system, "_last_robot_mode_time", 0)
                fresh_auto = mode == "AUTO" and mode_stamp > 0 and time.monotonic() - mode_stamp < 3.0
                recovery_authorized = False
                if recovery_resume:
                    verdict = self.recovery_supervision.resume_verified(["auto_navigation"], waiting_since)
                    local_health = self._health_summary()
                    locally_blocked = any(self._component_is_bad(local_health, c)
                                          for c in ("localization", "safety", "local_planner"))
                    navigation = getattr(self.skill_client, "navigation", None)
                    stopped = bool(navigation and navigation.recovery_motion_stopped())
                    if verdict and fresh_auto and stopped and not locally_blocked:
                        verified_since = verified_since if verified_since is not None else time.monotonic()
                        recovery_authorized = time.monotonic() - verified_since >= 2.0
                    else:
                        verified_since = None
                if (automatic_auto and fresh_auto) or (not automatic_auto and action == "resume") or recovery_authorized:
                    if self._is_cancelled(state.mission_id):
                        break
                    state.status = "EXECUTE_STEP"
                    state.question_for_human = state.final_message = None
                    state.human_wait = None
                    if recovery_authorized and action != "resume":
                        call.catalog["recovery_verified_resume_count"] = call.catalog.get("recovery_verified_resume_count", 0) + 1
                        self._event(state, "mission_recovery_resume_authorized", {
                            "step_id": call.step_id, "source": "/agent/recovery_state.task_supervision",
                            "verified_capabilities": ["auto_navigation"], "motion_stopped": True,
                            "attempt": call.catalog["recovery_verified_resume_count"]})
                    for task in state.tasks:
                        if task["state"] == "WAITING_HUMAN":
                            task.update(state="RUNNING", goal_satisfied=None, outcome=None, message=None)
                    self._state(state)
                    self._event(state, "mission_resuming_step", {"step_id": call.step_id,
                                "skill": call.name, "evidence": ("robot_mode_AUTO" if automatic_auto else
                                    "operator_requested_recheck" if action == "resume" else "recovery_verified_capability")})
                    return True
                with self._human_condition:
                    self._human_condition.wait(timeout=0.2)
            self._finish_cancelled(state)
            return False
        finally:
            with self._human_condition:
                self._human_wait = self._human_action = None
            state.human_wait = None

    @staticmethod
    def _explicit_cleanup_after_not_found(state, index, result):
        if not (state.intent and state.intent.constraints.get("explicit_continue_after_not_found") is True):
            return False
        metrics = result.get("metrics") or {}
        if result.get("skill_name") not in {"detect_known_object", "search_at_place"} or metrics.get("inference_completed") is not True:
            return False
        remaining = [call for call in state.plan[index + 1:] if call.name != "await_robot_stationary" and call.kind != "task_end"]
        return bool(remaining) and all((call.name == "set_pose" and call.payload.get("name") == "home") or
                                      (call.kind == "task" and call.name == "set_arm_pose" and call.payload.get("target") == "home")
                                      for call in remaining)

    def cancel_motion_only(self, run_id: str):
        return self.skill_client.stop_all(run_id)

    def _is_cancelled(self, mission_id: str) -> bool:
        with self._cancel_lock:
            return mission_id in self._cancelled_mission_ids

    def _finish_cancelled(self, state: MissionState) -> MissionState:
        with self._cancel_lock:
            self._cancelled_mission_ids.discard(state.mission_id)
        state.status = "CANCELLED"
        state.outcome = "CANCELLED"
        state.question_for_human = None
        state.human_wait = None
        state.recovery_pause = None
        state.final_message = "ユーザー操作によりタスクをキャンセルしました。"
        state.updated_at = utc_now()
        self._state(state)
        self._event(state, "mission_cancelled", {"message": state.final_message})
        return state

    def _handle_health_before_step(self, state: MissionState, call: SkillCall) -> bool:
        health = self._health_summary()
        self._annotate_call_with_catalog(call, health)
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

        if localization_bad and call.name in self.LOCALIZATION_DEPENDENT_MOTION_SKILLS:
            self._cancel_for_health(state, health)
            if self._localization_requires_recovery_from_health(health):
                return self._run_health_recovery(state)
            state.status = "NEEDS_HUMAN"
            state.question_for_human = (
                "Localization is not fresh enough for base motion. Waiting is safer than resetting pose because no hard "
                "drift/bad-score recovery condition was detected."
            )
            state.final_message = state.question_for_human
            self._state(state)
            self._event(state, "mission_needs_human", {"question": state.question_for_human, "health": health})
            return False

        if local_planner_bad and call.name in self.BASE_MOTION_SKILLS:
            self._cancel_for_health(state, health)
            return self._run_local_planner_recovery(state)

        if call.name in self.BASE_MOTION_SKILLS:
            bad_components = [
                name
                for name, summary in components.items()
                if isinstance(summary, dict) and summary.get("level") == "BAD"
            ]
            if call.name not in self.LOCALIZATION_DEPENDENT_MOTION_SKILLS and bad_components == ["localization"]:
                return True
            self._cancel_for_health(state, health)
            state.status = "NEEDS_HUMAN"
            state.question_for_human = "Health check failed before base motion. Please inspect robot health."
            state.final_message = state.question_for_human
            self._state(state)
            self._event(state, "mission_needs_human", {"question": state.question_for_human, "health": health})
            return False
        return True

    @staticmethod
    def _should_call_jev_supervision(result: dict) -> bool:
        if result.get("code") == codes.NEEDS_HUMAN and isinstance((result.get("metrics") or {}).get("navigation_review"), dict):
            return False
        if not bool(result.get("success")):
            return True
        code = str(result.get("code") or "")
        return code not in {codes.OK, codes.DRY_RUN_OK}

    def _annotate_plan_with_catalog(self, plan: list[SkillCall], health: dict | None) -> None:
        for call in plan:
            if call.kind != "skill":
                continue
            self._annotate_call_with_catalog(call, health)

    def _annotate_call_with_catalog(self, call: SkillCall, health: dict | None) -> None:
        definition = self.skill_catalog.get(call.name) or {}
        dashboard = definition.get("dashboard") if isinstance(definition.get("dashboard"), dict) else {}
        call.catalog = {
            **call.catalog,
            "group": definition.get("group"),
            "intent_tags": definition.get("intent_tags") or [],
            "display_name": dashboard.get("display_name") or call.name,
            "risk_label": dashboard.get("risk_label") or "unknown",
            "precondition_review": self.skill_catalog.review_skill_preconditions(call.name, health).to_dict(),
            "catalog_errors": self.skill_catalog_errors,
        }

    def _init_recovery_transport(self) -> None:
        node = getattr(self.skill_client, "node", None)
        if node is None or not hasattr(node, "create_publisher"):
            return
        try:
            from std_msgs.msg import String

            self._recovery_request_pub = node.create_publisher(String, "/agent/recovery_request", 10)
            node.create_subscription(String, "/agent/recovery_response", self._on_recovery_response, 10)
            node.create_subscription(String, "/agent/recovery_state", self._on_recovery_state, 10)
        except Exception:
            self._recovery_request_pub = None

    def _on_recovery_state(self, msg) -> None:
        try:
            state = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(state, dict):
            self.recovery_supervision.update(state)

    def _wait_for_capability_recovery(self, state, call, interrupted=None, timeout_sec=180.0, resume_allowed=True):
        supervision = getattr(self, "recovery_supervision", None)
        if supervision is None:
            return True
        required = self._required_capabilities_for_skill(call.name)
        blockers = supervision.blockers(required)
        if not blockers and not interrupted:
            return True
        state.status = "PAUSED_HEALTH"
        state.final_message = "Recovery Agent paused this step until its required capability is verified."
        state.recovery_pause = {"constraints": blockers or interrupted, "motion_stopped": call.name != "goto_pose"}
        self._state(state)
        self._event(state, "recovery_task_paused", state.recovery_pause)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._is_cancelled(state.mission_id):
                self._finish_cancelled(state)
                return False
            if call.name == "goto_pose":
                navigation = getattr(self.skill_client, "navigation", None)
                stopped = bool(navigation and navigation.recovery_motion_stopped())
                if state.recovery_pause["motion_stopped"] != stopped:
                    state.recovery_pause["motion_stopped"] = stopped
                    self._state(state)
            else:
                stopped = True
            if stopped and supervision.cleared(required):
                state.recovery_pause = None
                if not resume_allowed:
                    state.status = "NEEDS_HUMAN"
                    state.question_for_human = "Recovery repeatedly interrupted this navigation step; automatic resume budget is exhausted."
                    state.final_message = state.question_for_human
                    self._state(state)
                    self._event(state, "mission_needs_human", {"question": state.question_for_human})
                    return False
                state.status = "RESUMING"
                state.final_message = "Recovery capability constraint cleared; rechecking the original step."
                self._state(state)
                self._event(state, "recovery_task_released", {"step_index": state.current_step_index})
                return True
            with supervision.condition:
                supervision.condition.wait(timeout=min(.2, max(0, deadline - time.monotonic())))
        state.status = "NEEDS_HUMAN"
        state.question_for_human = "Recovery capability verification or motion-stop acknowledgement timed out. Inspect before retrying."
        state.final_message = state.question_for_human
        self._state(state)
        self._event(state, "mission_needs_human", {"question": state.question_for_human})
        return False

    def _on_recovery_response(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        request_id = str(data.get("request_id") or "")
        if not request_id:
            return
        with self._recovery_lock:
            mission_id = self._recovery_pending.get(request_id)
            if mission_id is None or data.get("mission_id") not in {None, mission_id}:
                return
            queue = self._recovery_responses.setdefault(request_id, [])
            if data not in queue:
                queue.append(data)
                del queue[:-32]
            self._recovery_lock.notify_all()

    def _publish_recovery_request_and_wait(self, state: MissionState, request: dict, timeout_sec: float = 180.0) -> dict | None:
        request_id = str(request.get("request_id") or f"rr-{int(time.time() * 1000)}")
        request["request_id"] = request_id
        self._event(state, "recovery_request", request)
        pub = self._recovery_request_pub
        if pub is None:
            self._event(state, "recovery_request_not_published", {"request_id": request_id, "reason": "no_ros_transport"})
            return None
        with self._recovery_lock:
            self._recovery_pending[request_id] = state.mission_id
        try:
            from std_msgs.msg import String

            pub.publish(String(data=json.dumps(request, ensure_ascii=False, default=str)))
        except Exception as exc:
            with self._recovery_lock:
                self._recovery_pending.pop(request_id, None)
                self._recovery_responses.pop(request_id, None)
            self._event(state, "recovery_request_not_published", {"request_id": request_id, "error": repr(exc)})
            return None
        try:
            return self._wait_recovery_final(state, request_id, timeout_sec)
        finally:
            with self._recovery_lock:
                self._recovery_pending.pop(request_id, None)
                self._recovery_responses.pop(request_id, None)

    def _wait_recovery_final(self, state, request_id, timeout_sec):
        deadline = time.monotonic() + max(0.1, timeout_sec)
        with self._recovery_lock:
            while time.monotonic() < deadline:
                if self._is_cancelled(state.mission_id):
                    return {"request_id": request_id, "mission_id": state.mission_id,
                            "status": "CANCELLED", "final": True, "advice": "stop"}
                queue = self._recovery_responses.get(request_id) or []
                response = queue.pop(0) if queue else None
                if queue:
                    self._recovery_responses[request_id] = queue
                else:
                    self._recovery_responses.pop(request_id, None)
                if response is not None:
                    self._event(state, "recovery_response", response)
                    status = str(response.get("status") or "")
                    final = response.get("final")
                    if final is False or (final is None and status == "IN_PROGRESS"):
                        state.status = "RECOVERING"
                        state.final_message = str(
                            response.get("summary")
                            or response.get("message")
                            or "Recovery Agent is still working."
                        )
                        self._state(state)
                        self._event(
                            state,
                            "recovery_response_waiting_final",
                            {
                                "request_id": request_id,
                                "status": status,
                                "summary": state.final_message,
                            },
                        )
                        continue
                    return response
                self._recovery_lock.wait(timeout=min(0.2, deadline - time.monotonic()))
        self._event(state, "recovery_response_timeout", {"request_id": request_id, "timeout_sec": timeout_sec})
        return {
            "request_id": request_id,
            "mission_id": state.mission_id,
            "status": "NEEDS_HUMAN",
            "final": True,
            "advice": "pause",
            "summary": f"Recovery Agent did not send a final response within {timeout_sec:.0f}s.",
        }

    def _build_recovery_request(
        self,
        state: MissionState,
        *,
        trigger: str,
        call: SkillCall | None = None,
        result: dict | None = None,
        suspected_components: list[str] | None = None,
        required_capabilities: list[str] | None = None,
        supervision_action: str | None = None,
    ) -> dict:
        step = {
            "index": state.current_step_index,
            "skill": call.name if call else None,
            "payload": dict(call.payload) if call else {},
        }
        return {
            "request_id": f"rr-{state.mission_id}-{int(time.time() * 1000)}",
            "caller": "mission_agent",
            "mission_id": state.mission_id,
            "trigger": trigger,
            "step": step,
            "result": self._compact_result_for_recovery(result or {}),
            "required_capabilities": required_capabilities or self._required_capabilities_for_skill(call.name if call else ""),
            "suspected_components": suspected_components or self._suspected_components_from_result(result or {}),
            "urgency": "blocking",
            "resources_held": self._resources_for_skill(call.name if call else ""),
            "user_context": state.user_text,
            "agent_context": {
                "agent_mode": state.agent_mode,
                "intent": state.intent.to_dict() if state.intent else None,
                "supervision_action": supervision_action,
            },
        }

    @staticmethod
    def _compact_result_for_recovery(result: dict) -> dict:
        metrics = result.get("metrics") if isinstance(result.get("metrics"), dict) else {}
        return {
            "code": result.get("code"),
            "success": result.get("success"),
            "message": str(result.get("message") or "")[:300],
            "metrics": {
                key: value
                for key, value in metrics.items()
                if key
                in {
                    "localization_warning",
                    "localization_requires_recovery",
                    "gicp_score",
                    "trusted_pose_delta_m",
                    "trusted_pose_delta_yaw_rad",
                    "stationary_drift_m",
                    "final_xy_error_m",
                    "final_yaw_error_rad",
                    "provider",
                    "image_topic",
                    "timeout_reason", "wheel_odom_rejection", "control_mode", "travelled_m",
                    "signed_accumulated_yaw_rad", "image_age_sec", "failed_child", "provider_errors",
                    "feedback_fresh", "feedback_age_sec", "failure_category",
                    "pan_feedback", "tilt_feedback", "hardware_pan_deg", "hardware_tilt_deg",
                }
            },
        }

    @staticmethod
    def _required_capabilities_for_skill(skill_name: str) -> list[str]:
        return {
            "goto_pose": ["auto_navigation"],
            "move_relative": ["relative_motion"],
            "move_relative_xy": ["relative_motion"],
            "rotate_relative": ["relative_motion"],
            "check_localization_health": ["localization"],
            "set_camera_pose": ["perception"],
            "capture_observation": ["perception"],
            "detect_known_object": ["perception"],
            "set_pose": ["arm"],
            "touch": ["arm"],
            "grasp": ["arm"],
        }.get(skill_name, [])

    @staticmethod
    def _resources_for_skill(skill_name: str) -> list[str]:
        if skill_name in {"goto_pose", "move_relative", "move_relative_xy", "rotate_relative"}:
            return ["base"]
        if skill_name in {"set_camera_pose", "capture_observation", "detect_known_object"}:
            return ["camera"]
        if skill_name in {"set_pose", "touch", "grasp"}:
            return ["arm"]
        return []

    @staticmethod
    def _suspected_components_from_result(result: dict) -> list[str]:
        code = str(result.get("code") or "")
        skill = str((result.get("metrics") or {}).get("failed_child") or result.get("skill_name") or "")
        if code == codes.FAILED_LOCALIZATION:
            return ["localization"]
        if skill == "set_camera_pose":
            return ["camera_control"]
        if skill in {"capture_observation", "detect_known_object"}:
            return ["camera"]
        if skill in {"set_pose", "touch", "grasp"}:
            return ["arm_gateway"]
        if skill in {"move_relative", "move_relative_xy", "rotate_relative"}:
            return ["drive"]
        if code in {codes.FAILED_NAVIGATION, codes.FAILED_TIMEOUT}:
            return ["local_planner"]
        return []

    def _handle_recovery_response(self, state: MissionState, response: dict | None, fallback_target: str | None = None) -> bool:
        if self._is_cancelled(state.mission_id) or (response and response.get("status") == "CANCELLED"):
            self._finish_cancelled(state)
            return False
        if response is None:
            if fallback_target == "local_planner":
                return self._run_local_planner_recovery(state)
            if fallback_target == "localization":
                recovery_result = self._run_localization_recovery(state)
                if recovery_result.success:
                    state.status = "RESUMING"
                    state.final_message = "Compatibility localization recovery completed; resuming mission."
                    self._state(state)
                    self._event(state, "mission_resuming", {"message": state.final_message})
                    return True
            state.status = "NEEDS_HUMAN"
            state.question_for_human = "Recovery Agent did not respond. Please inspect robot health before continuing."
            state.final_message = state.question_for_human
            self._state(state)
            self._event(state, "mission_needs_human", {"question": state.question_for_human})
            return False
        status = str(response.get("status") or "")
        advice = str(response.get("advice") or "")
        if status == "RESOLVED" or advice in {"resume_from_step", "retry_step"}:
            state.status = "RESUMING"
            state.final_message = f"Recovery Agent response: {status or advice}; resuming mission logic."
            self._state(state)
            self._event(state, "mission_resuming", {"message": state.final_message, "recovery_response": response})
            return True
        if status in {"NO_FAULT_FOUND", "NOT_RELATED"} or advice == "handle_as_task_failure":
            state.status = "NEEDS_HUMAN"
            state.question_for_human = str(
                response.get("summary")
                or "Recovery Agent found no related system fault; this should be handled as a task/plan failure."
            )
            state.final_message = state.question_for_human
            self._state(state)
            self._event(state, "mission_needs_human", {"question": state.question_for_human, "recovery_response": response})
            return False
        state.status = "NEEDS_HUMAN"
        state.question_for_human = str(response.get("summary") or "Recovery Agent needs human help before continuing.")
        state.final_message = state.question_for_human
        self._state(state)
        self._event(state, "mission_needs_human", {"question": state.question_for_human, "recovery_response": response})
        return False

    def _run_health_recovery(self, state: MissionState, call: SkillCall | None = None, result: dict | None = None) -> bool:
        if call is None and 0 <= state.current_step_index < len(state.plan):
            call = state.plan[state.current_step_index]
        if result is None:
            health = self._health_summary() or {}
            result = {
                "code": codes.FAILED_LOCALIZATION,
                "success": False,
                "message": "Localization precheck failed",
                "metrics": ((health.get("components") or {}).get("localization") or {}).get("details") or {},
            }
        state.status = "PAUSED_HEALTH"
        state.final_message = "Mission paused because robot health is BAD."
        self._state(state)
        self._event(state, "mission_paused_health", {"message": state.final_message, "health": self._health_summary()})

        state.status = "RECOVERING"
        state.final_message = "Calling Recovery Agent for health recovery."
        self._state(state)
        request = self._build_recovery_request(
            state,
            trigger="precheck_failed",
            call=call,
            result=result,
            suspected_components=["localization"],
            required_capabilities=["localization"],
        )
        response = self._publish_recovery_request_and_wait(state, request)
        if response is not None:
            return self._handle_recovery_response(state, response, fallback_target="localization")

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

    def _run_recovery_for_supervision(
        self,
        state: MissionState,
        supervision,
        call: SkillCall,
        result: dict,
        step_index: int,
    ) -> bool:
        state.current_step_index = step_index
        health = self._health_summary()
        target = self._recovery_target_from_jev(supervision)
        failed_skill = (result.get("metrics") or {}).get("failed_child") or call.name
        if failed_skill == "set_camera_pose" and target in {None, "none", "camera"}:
            target = "camera_control"
        suspected = [target] if target and target != "none" else self._suspected_components_from_result(result)
        state.status = "RECOVERING"
        state.final_message = f"Calling Recovery Agent for {target or 'system'} recovery."
        self._state(state)
        request = self._build_recovery_request(
            state,
            trigger="jev_step_supervision",
            call=call,
            result=result,
            suspected_components=suspected,
            supervision_action=getattr(supervision, "action", None),
        )
        response = self._publish_recovery_request_and_wait(state, request)
        if response is not None:
            return self._handle_recovery_response(state, response, fallback_target=target)
        if target == "local_planner" or self._component_is_bad(health, "local_planner"):
            return self._run_local_planner_recovery(state)
        if target == "localization" or self._component_is_bad(health, "localization"):
            return self._run_health_recovery(state)
        state.status = "NEEDS_HUMAN"
        state.question_for_human = (
            "JEV judged that recovery is needed, but no implemented automatic recovery target is available yet. "
            "Please inspect robot health or retry after fixing the component."
        )
        state.final_message = state.question_for_human
        self._state(state)
        self._event(state, "mission_needs_human", {"question": state.question_for_human, "health": health})
        return False

    @staticmethod
    def _recovery_target_from_jev(supervision) -> str | None:
        trace = getattr(supervision, "trace", None)
        for question in getattr(trace, "questions", []) or []:
            if isinstance(question, dict) and question.get("id") == "recovery_target":
                value = str(question.get("result") or "")
                return value if value else None
        return None

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

    @staticmethod
    def _localization_requires_recovery_from_health(health: dict | None) -> bool:
        localization = (((health or {}).get("components") or {}).get("localization") or {}).get("details") or {}
        return MissionAgent._localization_requires_recovery_from_metrics(localization)

    @staticmethod
    def _localization_requires_recovery_from_metrics(metrics: dict | None) -> bool:
        if not isinstance(metrics, dict):
            return True
        if metrics.get("localization_requires_recovery") is True:
            return True
        warning = str(metrics.get("localization_warning") or "")
        return warning in {
            "GICP_SCORE_BAD",
            "GICP_TRUSTED_POSE_DIVERGENCE",
            "POSSIBLE_STATIONARY_DRIFT",
        }

    def _build_plan_for_intent(self, intent: MissionIntent | None, state: MissionState) -> list[SkillCall]:
        if intent is None:
            return []
        temp = MissionState(user_text=state.user_text, mode=state.mode, mission_id=state.mission_id)
        temp.intent = intent
        return self._build_plan(temp)

    def _build_plan(self, state: MissionState) -> list[SkillCall]:
        if state.intent is None:
            return []
        intents = self.task_catalog.flatten(state.intent)
        if not intents or len(intents) > 6 or any(i.task_type not in self.task_catalog.tasks for i in intents):
            return []
        return [SkillCall(i.task_type, payload=i.to_dict(), kind="task",
                          owner=self.task_catalog.tasks[i.task_type]["owner"],
                          task_id=f"{state.mission_id}:t{n + 1}") for n, i in enumerate(intents)]

    def _hold_pending_task(self, state, call, message, outcome="CONDITION_UNKNOWN",
                           reason="condition_evidence_incomplete"):
        record = next((t for t in state.tasks if t["task_id"] == call.task_id), None)
        if record is None:
            record = {"task_id": call.task_id, "parent_task_id": call.catalog.get("parent_task_id"),
                      "task_type": call.name, "owner": call.owner, "goal": call.payload}
            state.tasks.append(record)
        record.update(state="WAITING_HUMAN", goal_satisfied=None,
                      outcome=outcome, message=message)
        state.status = "NEEDS_HUMAN"
        state.question_for_human = state.final_message = message
        self._event(state, "task_feedback", dict(record))
        self._event(state, "mission_needs_human", {"question": message, "task_id": call.task_id,
                                                  "reason": reason})
        self._state(state)
        return False

    def _expand_task(self, state, index, trace):
        from robot_agent.agents.domain import TaskRefinement
        call = state.plan[index]
        intent = MissionIntent(**call.payload)
        # References remain tied to the original task, not the most recently
        # completed task (which may be an unrelated cleanup operation).
        if intent.constraints.get("depends_on") and not intent.constraints.get("run_if"):
            for ordinal in intent.constraints["depends_on"]:
                source = next((t for t in state.tasks if
                    t.get("parent_task_id") == call.catalog.get("parent_task_id") and
                    (t.get("goal", {}).get("constraints") or {}).get("task_ordinal") == ordinal), None)
                if not source or source.get("state") != "COMPLETED" or source.get("goal_satisfied") is not True:
                    return self._hold_pending_task(state, call, f"Task requires successful completion of task {ordinal}; evidence is missing or goal unmet.",
                                                   outcome="DEPENDENCY_UNCONFIRMED", reason="required_task_unsatisfied")
        condition = intent.constraints.get("run_if")
        if condition is not None:
            prior = [t for t in state.tasks if t.get("parent_task_id") == call.catalog.get("parent_task_id")
                     and t.get("task_id") != call.task_id]
            refs = intent.constraints.get("depends_on", [])
            source = (next((t for t in prior if (t.get("goal", {}).get("constraints") or {}).get("task_ordinal") == refs[0]), {})
                      if len(refs) == 1 else prior[-1] if not refs and prior else {})
            evidence = [r for r in state.skill_results if r.get("task_id") == source.get("task_id")
                        and r.get("skill_name") == "search_at_place"]
            result = evidence[-1] if evidence else {}
            metrics = result.get("metrics") or {}
            complete = (source.get("state") == "COMPLETED" and source.get("task_type") == "search_object"
                        and result.get("success") is True and metrics.get("inference_completed") is True
                        and metrics.get("outcome") in {"FOUND", "NOT_FOUND"})
            if condition != "previous_search_found" or not complete:
                return self._hold_pending_task(state, call,
                    "Conditional action is blocked: complete detection evidence is unavailable. PARTIAL or failed detection is not NOT_FOUND.")
            if metrics["outcome"] == "NOT_FOUND" and metrics.get("goal_satisfied") is False:
                source["condition_result"] = "not_found"
                record = {"task_id": call.task_id, "parent_task_id": call.catalog.get("parent_task_id"),
                          "task_type": call.name, "owner": call.owner, "goal": call.payload,
                          "state": "SKIPPED", "goal_satisfied": None, "outcome": "CONDITION_FALSE",
                          "message": "Target not found; remaining stationary as requested."}
                state.tasks.append(record)
                state.plan.pop(index)
                self._event(state, "task_feedback", dict(record))
                self._state(state)
                return True
            if metrics["outcome"] != "FOUND" or source.get("goal_satisfied") is not True:
                return self._hold_pending_task(state, call,
                    "Conditional detection evidence is inconsistent; dependent task remains blocked.")
        if not self._allow_task_after_unmet_goal(state, call, intent, trace):
            return False
        record = next((t for t in state.tasks if t["task_id"] == call.task_id), None)
        first_delegation = record is None or record.get("outcome") in {"CONDITION_UNKNOWN", "DEPENDENCY_UNCONFIRMED"}
        if record is None:
            record = {"task_id": call.task_id, "parent_task_id": call.catalog.get("parent_task_id"),
                  "task_type": call.name, "owner": call.owner, "goal": call.payload,
                  "state": "REFINING", "goal_satisfied": None, "outcome": None}
            state.tasks.append(record)
        record.update(state="REFINING", goal_satisfied=None, outcome=None, message=None)
        self._state(state)
        if first_delegation:
            self._event(state, "task_delegated", dict(record))
        if self._is_cancelled(state.mission_id):
            self._finish_cancelled(state)
            return False
        if call.owner == "mission":
            temp = MissionState(user_text=intent.raw_text, mode=state.mode,
                                intent=self.task_catalog.legacy(intent))
            refinement = TaskRefinement(plan=self._build_legacy_plan(temp))
        else:
            owner = {"navigation": self.navigation_agent, "perception": self.perception_agent,
                     "arm": self.arm_agent}[call.owner]
            try:
                refinement = owner.refine(intent, self.skill_client, self.jev_agent, state.agent_mode)
            except Exception as exc:
                refinement = TaskRefinement(question=f"Domain refinement failed: {exc}", outcome="REFINEMENT_ERROR")
        for item in refinement.traces:
            item.task_id = call.task_id
            trace.add(item)
            self._event(state, "domain_decision", {"task_id": call.task_id, "owner": call.owner,
                                                  "trace": item.to_dict()})
        state.jev_trace = trace.to_dict()
        if self._is_cancelled(state.mission_id):
            self._finish_cancelled(state)
            return False
        if refinement.question or not refinement.plan:
            record.update(state="WAITING_HUMAN",
                          outcome=refinement.outcome if refinement.outcome != "READY" else "NEEDS_HUMAN", goal_satisfied=False,
                          message=refinement.question or "No executable task plan.")
            state.status = "NEEDS_HUMAN"
            state.question_for_human = refinement.question or "No executable task plan."
            state.final_message = state.question_for_human
            if refinement.choices:
                state.intent.constraints["ambiguity"] = {"reason": state.question_for_human,
                                                           "choices": refinement.choices}
            self._state(state)
            self._event(state, "task_feedback", dict(record))
            self._event(state, "mission_needs_human", {"question": state.question_for_human,
                                                        "task_id": call.task_id})
            scoped = [c for c in refinement.choices if c.get("action") and isinstance(c.get("constraints"), dict)]
            if scoped and self.enable_human_resume:
                selected = self._wait_for_domain_choice(state, call, scoped)
                if selected:
                    call.payload["constraints"].update(selected["constraints"])
                    if state.intent is not None:
                        state.intent.constraints.pop("ambiguity", None)
                    state.question_for_human = state.final_message = None
                    state.status = "EXECUTE_STEP"
                    # Recheck only this delegated task, not previous Mission work.
                    return self._expand_task(state, index, trace)
            return False
        for n, step in enumerate(refinement.plan):
            if step.kind == "task":
                step.task_id = f"{call.task_id}:t{n + 1}"
                step.catalog["parent_task_id"] = call.task_id
            else:
                step.task_id, step.owner = call.task_id, call.owner
                step.step_id = f"{call.task_id}:s{n + 1}"
        record["state"] = "RUNNING"
        self._annotate_plan_with_catalog(refinement.plan, self._health_summary())
        state.plan[index:index + 1] = refinement.plan + [SkillCall(call.name, kind="task_end", task_id=call.task_id,
            owner=call.owner, catalog={"parent_task_id": call.catalog.get("parent_task_id")})]
        self._state(state)
        self._event(state, "task_plan_created", {"task_id": call.task_id,
                    "plan": [x.to_dict() for x in refinement.plan]})
        return True

    def _clarify_task_interpretation(self, state, ordinal, candidates, parse_trace):
        state.status = "NEEDS_HUMAN"
        state.intent = MissionIntent("ask_human", raw_text=state.user_text, requires_confirmation=True,
            constraints={"ambiguity": {"reason": "task_interpretation_ambiguous", "task_ordinal": ordinal,
                                        "interpretations": candidates}})
        state.parsed_intent = state.intent.to_dict()
        state.question_for_human = state.final_message = (
            f"Task {ordinal} has more than one plausible meaning. Which action did you intend? "
            "The pose or map target will still be validated after selection.")
        pending_trace = JevTrace(agent_mode=state.agent_mode)
        pending_trace.add(parse_trace)
        state.jev_trace = pending_trace.to_dict()
        self._state(state)
        self._event(state, "mission_needs_human", {"question": state.question_for_human,
                    "task_ordinal": ordinal, "interpretations": candidates})
        if not self.enable_human_resume:
            return None
        choices = [{**c, "action": "interpret_as_" + c["task_type"]} for c in candidates]
        call = SkillCall("command_parse", step_id=f"{state.mission_id}:parse:{ordinal}")
        selected = self._wait_for_domain_choice(state, call, choices)
        if selected is None or self._is_cancelled(state.mission_id):
            return None
        self._event(state, "task_interpretation_confirmed", {"task_ordinal": ordinal, **selected})
        state.status = "PARSE"
        state.question_for_human = state.final_message = None
        self._state(state)
        return selected["task_type"]

    def _wait_for_domain_choice(self, state, call, choices):
        from uuid import uuid4
        wait = {"mission_id": state.mission_id, "wait_id": uuid4().hex,
                "step_id": call.step_id, "skill_name": call.name, "auto_resume": False,
                "actions": [c["action"] for c in choices],
                "action_labels": {c["action"]: c["label"] for c in choices}}
        state.human_wait = wait
        with self._human_condition:
            self._human_wait, self._human_action = wait, None
        self._state(state)
        self._event(state, "mission_waiting_for_domain_choice", dict(wait))
        try:
            while not self._is_cancelled(state.mission_id):
                with self._human_condition:
                    action, self._human_action = self._human_action, None
                    if action is None:
                        self._human_condition.wait(timeout=.2)
                if action is not None and not self._is_cancelled(state.mission_id):
                    selected = next(c for c in choices if c["action"] == action)
                    self._event(state, "mission_human_action", {**wait, "action": action, "selection": selected})
                    return selected
            self._finish_cancelled(state)
            return None
        finally:
            with self._human_condition:
                self._human_wait = self._human_action = None
            state.human_wait = None

    def _navigation_review_callback(self, state, call, trace):
        intent = MissionIntent(**call.catalog["navigation_intent"])
        def review_route(evidence):
            if self._is_cancelled(state.mission_id):
                return {"decision": "cancelled"}
            decision, items = self.navigation_agent.review_path(
                intent, call.catalog["navigation_contract"], evidence,
                self.skill_client, self.jev_agent, state.agent_mode)
            for item in items:
                item.task_id, item.step_id = call.task_id, call.step_id
                item.skill_name = call.name
                trace.add(item)
                self._event(state, "domain_decision", {"task_id": call.task_id, "owner": "navigation",
                                                      "trace": item.to_dict()})
            state.jev_trace = trace.to_dict()
            self._state(state)
            if self._is_cancelled(state.mission_id):
                return {"decision": "cancelled"}
            # Re-read current control/safety evidence after a potentially slow API request.
            health = self._health_summary()
            if health and not self.skill_catalog.review_skill_preconditions("goto_pose", health).allowed:
                return {"decision": "ask_human", "reason": "Navigation prerequisites changed during review"}
            if health:
                components = health.get("components", {})
                mode = components.get("mode", {})
                mode_age = mode.get("details", {}).get("robot_mode_age_sec")
                safety_age = components.get("safety", {}).get("details", {}).get("safety_status_age_sec")
                if (mode.get("summary") != "AUTO" or mode_age is None or safety_age is None
                        or not 0 <= mode_age <= 5.0 or not 0 <= safety_age <= 5.0):
                    return {"decision": "ask_human", "reason": "Fresh AUTO and safety feedback required after route review"}
            self._event(state, "navigation_path_review", {"task_id": call.task_id, "step_id": call.step_id, **decision})
            return decision
        return review_route

    def _complete_task(self, state, task_id):
        record = next(t for t in state.tasks if t["task_id"] == task_id)
        results = [x for x in state.skill_results if x.get("task_id") == task_id]
        children = [t for t in state.tasks if t.get("parent_task_id") == task_id]
        # Retain every attempt in history, but completion uses the last attempt per step.
        latest = {x.get("step_id") or f"legacy:{n}": x for n, x in enumerate(results)}
        okay = bool(latest or children) and all(
            x.get("success") and x.get("code") in {codes.OK, codes.DRY_RUN_OK}
            and (x.get("metrics") or {}).get("goal_satisfied") is not False for x in latest.values())
        okay = okay and all(t.get("goal_satisfied") is True for t in children)
        record.update(state="COMPLETED", goal_satisfied=okay, outcome="SATISFIED" if okay else "UNMET",
                      result_step_ids=[x.get("step_id") for x in latest.values()])
        if record.get("owner") == "navigation":
            arrival = next((x for x in reversed(list(latest.values())) if x.get("skill_name") == "goto_pose"), None)
            record["navigation_arrival"] = self.navigation_agent.arrival_feedback(arrival or {})
            record["visual_confirmation"] = ("NOT_REQUIRED" if not children else
                                              "CONFIRMED" if all(t.get("goal_satisfied") for t in children) else "UNCONFIRMED")
        self._event(state, "task_feedback", dict(record))
        self._state(state)

    def _allow_task_after_unmet_goal(self, state, call, intent, trace):
        if intent.constraints.get("depends_on"):
            # _expand_task already verified these exact references. An unrelated
            # intervening unmet goal must not replace that explicit dependency.
            return True
        previous = [t for t in state.tasks if t.get("state") == "COMPLETED"
                    and t.get("parent_task_id") == call.catalog.get("parent_task_id")]
        if not previous or previous[-1].get("goal_satisfied") is not False:
            return True
        constraints = state.intent.constraints if state.intent else {}
        policy = intent.constraints.get("dependency_policy", constraints.get("dependency_policy", "ambiguous_dependency"))
        approval = call.catalog.pop("failure_continuation_approval", None)
        if (policy != "required_condition" and approval == {
                "mission_id": state.mission_id, "failed_task_id": previous[-1]["task_id"]}):
            return True
        independent = policy in {"independent_goals", "ordered_but_independent", "cleanup_after_attempt"}
        # A parser's independence label is not operator approval to move after
        # an unconfirmed perception goal (M13). This policy is task-owned.
        if (previous[-1].get("task_type") == "search_object"
                and self.task_catalog.tasks.get(intent.task_type, {}).get("unmet_perception_policy") == "ask_before_motion"):
            independent = False
        cleanup = (intent.constraints.get("explicit_after_attempt") is True or
                   constraints.get("explicit_continue_after_not_found") is True
                   and intent.task_type == "set_arm_pose" and intent.target == "home")
        decision = "continue_independent" if (independent or cleanup) and policy != "required_condition" else "ask_human"
        if state.agent_mode in {"jev_primary", "jev_shadow"} and not cleanup:
            choices = {"ask_human": "Prior goal was unmet; clarify whether this next task is still wanted."}
            # Dependency checks bound the model's authority, including on API fallback.
            if independent and policy != "required_condition":
                choices["continue_independent"] = "Continue this explicitly independent task; preserve the prior unmet result."
            chosen, item = self.jev_agent.domain_review("task_outcome_review", intent, choices, decision,
                {"previous_task": {k: previous[-1].get(k) for k in ("task_type", "outcome", "goal_satisfied")},
                 "dependency_policy": policy, "execution_fault": False})
            item.task_id = call.task_id
            trace.add(item)
            state.jev_trace = trace.to_dict()
            self._event(state, "task_outcome_decision", {"task_id": call.task_id, "trace": item.to_dict()})
            if state.agent_mode == "jev_primary":
                decision = chosen
        if self._is_cancelled(state.mission_id):
            self._finish_cancelled(state)
            return False
        if decision == "continue_independent":
            return True
        self._hold_pending_task(state, call,
            "Previous task completed without confirming its target. Should the next task continue?",
            outcome="DEPENDENCY_UNCONFIRMED", reason="unmet_goal_dependency")
        if self.enable_human_resume and policy != "required_condition":
            selected = self._wait_for_domain_choice(state, call, [{
                "action": "continue_unmet_goal", "label": f"Continue {call.name}: previous target is unconfirmed",
                "constraints": {}}])
            if selected and not self._is_cancelled(state.mission_id):
                state.status = "EXECUTE_STEP"
                state.question_for_human = state.final_message = None
                self._event(state, "unmet_goal_continue_approved", {"task_id": call.task_id,
                            "previous_task_id": previous[-1]["task_id"], "safety_checks_required": True})
                self._state(state)
                return True
        return False

    def _after_failed_task(self, state, index):
        task_id = state.plan[index].task_id
        if not task_id:
            return index + 1
        # Skip the rest of this goal, never just the failed prerequisite.
        for n in range(index + 1, len(state.plan)):
            if state.plan[n].kind == "task_end" and state.plan[n].task_id == task_id:
                return n
        return len(state.plan)

    def _build_legacy_plan(self, state: MissionState) -> list[SkillCall]:
        intent = state.intent
        if intent is None:
            return []
        if intent.task_type == "sequence":
            return self._build_sequence_plan(state)
        if intent.task_type == "cancel":
            return [SkillCall(name="cancel_motion", reason="Cancel active motion.", timeout_sec=2.0)]
        if intent.task_type == "robot_status":
            return [SkillCall(name="get_robot_status", reason="Report robot status.", timeout_sec=3.0)]
        if intent.task_type == "list_known_targets":
            return [SkillCall(name="list_known_targets", reason="List semantic map targets.", timeout_sec=1.0)]
        if intent.task_type == "move_lift_bounded":
            return [
                SkillCall(
                    name="await_robot_stationary",
                    payload={"settle_sec": 1.0, "timeout_sec": 30.0},
                    reason="Require the base to be stationary before lift motion.",
                    timeout_sec=30.0,
                ),
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
                    reason="Bounded direct base motion using fresh wheel odometry; no global localization dependency.",
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
                    reason="Rotate relative to current base heading.",
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
        if intent.task_type == "set_pose":
            return self._arm_preflight() + self.arm_agent.build_plan(intent)
        if intent.task_type in {"touch_object", "press_object", "grasp_object"}:
            return self._arm_preflight() + self.arm_agent.build_plan(intent)
        if intent.task_type in {"touch_object_near_place", "press_object_near_place", "grasp_object_near_place"}:
            plan = self.navigation_agent.build_plan(intent)
            return plan + self._arm_preflight() + self.arm_agent.build_plan(intent)
        return []

    def _build_sequence_plan(self, state: MissionState) -> list[SkillCall]:
        steps = state.intent.constraints.get("steps", []) if state.intent else []
        plan: list[SkillCall] = []
        if not isinstance(steps, list):
            return plan
        if not 1 <= len(steps) <= 3:
            return []
        for item in steps:
            if not isinstance(item, dict):
                return []
            sub_intent = MissionIntent(
                task_type=str(item.get("task_type") or "ask_human"),
                target=item.get("target"),
                relation=item.get("relation"),
                object_query=item.get("object_query"),
                constraints=item.get("constraints") if isinstance(item.get("constraints"), dict) else {},
                requires_confirmation=bool(item.get("requires_confirmation", False)),
                raw_text=str(item.get("raw_text") or ""),
            )
            sub_state = MissionState(user_text=sub_intent.raw_text or state.user_text, mode=state.mode)
            sub_state.intent = sub_intent
            sub_plan = self._build_plan(sub_state)
            if not sub_plan:
                return []
            plan.extend(sub_plan)
        return plan

    @staticmethod
    def _intent_has_perception_stage(intent: MissionIntent | None) -> bool:
        if intent is None:
            return False
        perception_tasks = {"look_direction", "search_object_here", "search_object_near_place", "inspect_place"}
        if intent.task_type in perception_tasks:
            return True
        if intent.task_type != "sequence":
            return False
        steps = intent.constraints.get("steps") if isinstance(intent.constraints, dict) else None
        if not isinstance(steps, list):
            return False
        return any(isinstance(step, dict) and str(step.get("task_type") or "") in perception_tasks for step in steps)

    @staticmethod
    def _relative_motion_preflight() -> list[SkillCall]:
        return [
            SkillCall(
                name="await_robot_auto_mode",
                payload={"timeout_sec": 0.1},
                reason="Human must press A / confirm AUTO before relative motion.",
                timeout_sec=0.1,
                requires_human_confirm=True,
            )
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
    def _arm_preflight() -> list[SkillCall]:
        return [
            SkillCall(
                name="await_robot_stationary",
                payload={"settle_sec": 1.0, "timeout_sec": 180.0},
                reason="Wait until base motion has stopped before arm motion.",
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
        if skill_name in {"set_pose", "touch", "grasp"}:
            return "Arm Agent"
        return "Mission Agent"

    def _run_localization_recovery(self, state: MissionState):
        # Recovery owns reseeding. A missing/late response must not launch a
        # second, uncoordinated reset while that agent may still be working.
        result = make_result(
            "localization_recovery_handoff", False, codes.NEEDS_HUMAN,
            "Recovery Agent did not finish. Keep the robot stopped, inspect its recovery status, "
            "and use RViz 2D Pose Estimate if needed. No automatic fallback reset was issued.",
        )
        self._event(state, "localization_recovery_unavailable", result.to_dict())
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
        if state.status == "FAILED_SKILL":
            state.outcome = "FAILED"
        if state.status in {"CANCELLED", "FAILED_SKILL", "NEEDS_HUMAN", "DONE_WITH_STUB"}:
            for task in state.tasks:
                if task["state"] in {"REFINING", "RUNNING"} or (state.status != "NEEDS_HUMAN" and task["state"] == "WAITING_HUMAN"):
                    task.update(state="WAITING_HUMAN" if state.status == "NEEDS_HUMAN" else state.status,
                                goal_satisfied=False, outcome=state.status, message=state.final_message)
                    self._event(state, "task_feedback", dict(task))
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
        if next_key is None or next_key in failed_pose_keys.get(index, set()):
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
        if any(result.get("continued_after_failure") for result in state.skill_results):
            return "Some goals remain unmet; independent steps continued under the continuation policy. Inspect failed step results."
        if state.intent and state.intent.task_type in {"robot_status", "get_robot_status"} and state.skill_results:
            return json.dumps(state.skill_results[-1].get("metrics", {}), ensure_ascii=False)
        if state.intent and state.intent.task_type == "list_known_targets" and state.skill_results:
            return json.dumps(state.skill_results[-1].get("metrics", {}), ensure_ascii=False)
        return "タスクが完了しました。"

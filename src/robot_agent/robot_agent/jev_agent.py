from __future__ import annotations

import json
import hashlib
import os
import re
import time
import urllib.error
import urllib.request
from copy import deepcopy
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Any

from robot_agent.candidates.skill_candidate_generator import CommandCandidate, SkillCandidateGenerator
from robot_agent.parsers.deterministic_parser import DeterministicParser, command_guard_reason, STEP_SEPARATOR, conditional_parts, bind_detection_condition
from robot_agent.schemas import MissionIntent, SkillCall
from robot_agent.task_catalog import TaskCatalog
from robot_skills import result_codes as codes
from robot_skills.skill_catalog import SkillCatalog


AGENT_MODES = {"classic", "jev_shadow", "jev_primary"}
JEV_SCHEMA_VERSION = "jev_robot_agent.v2"


@dataclass
class JevCallTrace:
    call_id: str
    stage: str
    provider: str = "local"
    model: str = "jev-local-v0"
    question_count: int = 0
    started_at_monotonic: float = 0.0
    ended_at_monotonic: float = 0.0
    latency_ms: float = 0.0
    input_tokens_est: int = 0
    input_cost_usd: float = 0.0
    output_cost_usd: float = 0.0
    questions: list[dict[str, Any]] = field(default_factory=list)
    adopted_decision: str = ""
    fallback_used: bool = False
    task_id: str | None = None
    step_id: str | None = None
    step_index: int | None = None
    skill_name: str | None = None
    execution_gate: str | None = None

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data.pop("started_at_monotonic", None)
        data.pop("ended_at_monotonic", None)
        return data


@dataclass
class JevTrace:
    agent_mode: str = "classic"
    calls: list[JevCallTrace] = field(default_factory=list)

    def add(self, call: JevCallTrace) -> None:
        self.calls.append(call)

    def to_dict(self) -> dict[str, Any]:
        input_tokens = sum(call.input_tokens_est for call in self.calls)
        input_cost = sum(call.input_cost_usd for call in self.calls)
        output_cost = sum(call.output_cost_usd for call in self.calls)
        latency_ms = sum(call.latency_ms for call in self.calls)
        return {
            "agent_mode": self.agent_mode,
            "total": {
                "calls": len(self.calls),
                "latency_ms": round(latency_ms, 3),
                "input_tokens_est": input_tokens,
                "input_cost_usd": round(input_cost, 8),
                "output_cost_usd": round(output_cost, 8),
                "estimated_total_cost_usd": round(input_cost + output_cost, 8),
            },
            "calls": [call.to_dict() for call in self.calls],
        }


@dataclass
class JevParseDecision:
    intent: MissionIntent
    trace: JevCallTrace
    additional_traces: list[JevCallTrace] = field(default_factory=list)


@dataclass
class JevSupervisionDecision:
    action: str
    trace: JevCallTrace
    human_choices: list[dict[str, str]] = field(default_factory=list)
    message: str = ""


class JevStrengthenAgent:
    """Typed-decision adapter for the JEV agent path.

    If JEV_API_KEY is set, calls are sent to the configured JEV endpoint.
    Otherwise a deterministic local adapter keeps the same trace shape for
    offline testing and dashboard development.
    """

    PRICE_PER_MILLION_INPUT_TOKENS_USD = 0.042

    def __init__(self):
        self.task_catalog = TaskCatalog()
        self.parser = DeterministicParser()
        self.skill_catalog = SkillCatalog()
        self.candidate_generator = SkillCandidateGenerator(self.parser, self.skill_catalog)
        self._call_seq = 0
        self.api_key = os.environ.get("JEV_API_KEY", "").strip()
        self.api_url = os.environ.get("JEV_API_URL", "").strip()
        self.model = os.environ.get("JEV_MODEL", "").strip()
        if self.api_key and not self.api_url:
            self.api_url = (
                "https://openrouter.ai/api/v1/systemone"
                if self.api_key.startswith("sk-or-")
                else "https://api.typesafe.ai/v1/systemone"
            )
        if not self.model:
            self.model = "typesafe/jev-1.13" if self.api_url.startswith("https://openrouter.ai/") else "jev-latest"
        try:
            self.timeout_sec = float(os.environ.get("JEV_TIMEOUT_SEC", "3.0"))
        except ValueError:
            self.timeout_sec = 3.0
        self.api_configured = bool(self.api_key and self.api_url)
        self.api_available = False
        self.api_status = "not_configured"
        self.api_error = ""
        self.call_log_path = self._resolve_call_log_path()
        if self.api_configured:
            self._probe_api()

    def is_live_enabled(self) -> bool:
        return self.api_configured and self.api_available

    def provider_label(self) -> str:
        return "live" if self.is_live_enabled() else "local"

    def availability_summary(self) -> dict[str, Any]:
        return {
            "configured": self.api_configured,
            "available": self.api_available,
            "status": self.api_status,
            "error": self.api_error,
            "provider": self.provider_label(),
            "model": self.model if self.api_configured else "jev-local-v0",
        }

    def parse_command(self, text: str, robot_state: dict[str, Any] | None = None, *, clarify_task=None, is_cancelled=None) -> JevParseDecision:
        from robot_agent.command_grounding import tokens, structure_questions, binding_questions, compile_tasks, interpretation_candidates, disambiguation_questions
        started = time.monotonic()
        self._call_seq += 1
        questions = structure_questions(self.task_catalog)
        words = tokens(text)
        response = self._decide("command_parse", {
            "stage_goal": "Identify goal-level tasks and control flow independently of Classic. "
                          "Do not assess a plan that has not been constructed. Domain agents resolve their own parameters/capabilities.",
            "user_command": text,
        }, questions) if 0 < len(words) <= 96 else {"_error": "command_token_limit"}
        if response is None and not self.api_configured:
            return self._parse_command_local(text, robot_state)
        traces = []

        def trace(stage, begin, qs, result, decision):
            entries = [{"id": key, "type": value["type"]} for key, value in qs.items()]
            return self._trace(stage, begin, text, self._questions_from_response(result or {}, entries),
                               decision, api_response=result or {"_error": "api_unavailable"})

        def values(result, qs):
            if not result or result.get("_error"):
                raise ValueError("jev_api_unavailable")
            answers = {key: self._answer_value(result, key) for key in qs}
            if any(value not in qs[key]["criteria"] for key, value in answers.items()):
                raise ValueError("jev_missing_or_invalid_answer")
            return answers

        binding_started, binding, bound_response = None, None, None
        try:
            if is_cancelled and is_cancelled():
                raise ValueError("parse_cancelled")
            parsed = values(response, questions)
            count = {"one": 1, "two": 2, "three": 3}.get(parsed["task_count"])
            flow = parsed["command_shape"]
            if flow not in {"linear", "if_found_else_hold"} or count is None:
                raise ValueError("unsupported_or_unclear_task_structure")
            task_types = [parsed[f"task_{i}_type"] for i in range(1, count + 1)]
            if any(task not in self.task_catalog.tasks for task in task_types):
                raise ValueError("unknown_task_type")
            if any(parsed[f"task_{i}_type"] != "absent" for i in range(count + 1, 4)):
                raise ValueError("task_count_type_disagreement")
            traces.append(trace("command_parse", started, questions, response, "structure_identified"))
            duplicates = [i for i, task in enumerate(task_types, 1) if parsed[f"task_{i}_alternative_type"] == task]
            if duplicates:
                follow_started = time.monotonic()
                self._call_seq += 1
                follow_questions = disambiguation_questions(self.task_catalog, task_types, duplicates)
                follow_response = self._decide("command_disambiguation", {
                    "stage_goal": "Resolve inconsistent alternative answers with primary types now explicit.",
                    "user_command": text, "primary_task_types": task_types}, follow_questions)
                traces.append(trace("command_disambiguation", follow_started, follow_questions, follow_response, "alternatives_reviewed"))
                if is_cancelled and is_cancelled():
                    raise ValueError("parse_cancelled")
                resolved = values(follow_response, follow_questions)
                if "unclear" in resolved.values():
                    raise ValueError("task_interpretation_unclear")
                parsed.update(resolved)
            confirmed = {}
            for i, task in enumerate(task_types, 1):
                alternatives = interpretation_candidates(self.task_catalog, i, task,
                    parsed[f"task_{i}_alternative_type"], response)
                if not alternatives:
                    continue
                traces[0].adopted_decision = "ask_human:task_interpretation_ambiguous"
                traces[-1].adopted_decision = "ask_human:task_interpretation_ambiguous"
                selected = clarify_task(i, alternatives, traces[-1]) if clarify_task is not None else None
                if selected not in {c["task_type"] for c in alternatives}:
                    intent = self._ask_human_intent(text, "task_interpretation_ambiguous")
                    intent.constraints["ambiguity"].update(task_ordinal=i, interpretations=alternatives)
                    return JevParseDecision(intent, traces[0], traces[1:])
                task_types[i - 1] = selected
                confirmed[str(i)] = selected
                traces[0].adopted_decision = "structure_human_clarified"
                traces[-1].adopted_decision = "structure_human_clarified"
            if is_cancelled and is_cancelled():
                raise ValueError("parse_cancelled")
            binding_started = time.monotonic()
            self._call_seq += 1
            binding = binding_questions(task_types, words)
            bound_response = self._decide("command_bind", {
                "stage_goal": "Ground classified tasks in exact source spans and per-task dependencies. "
                              "No Classic result, no model/preset selection; never invent an argument.",
                "user_command": text, "source_words": words, "task_types": task_types, "control_flow": flow,
                **({"human_confirmed_task_types": confirmed} if confirmed else {}),
            }, binding)
            answers = values(bound_response, binding)
            intent = compile_tasks(self, text, words, task_types, flow, answers)
            if confirmed:
                intent.constraints["human_confirmed_task_types"] = confirmed
                children = intent.constraints.get("steps", []) if intent.task_type == "sequence" else [intent.to_dict()]
                for i, child in enumerate(children, 1):
                    if str(i) in confirmed:
                        child["constraints"]["human_confirmed_task_type"] = confirmed[str(i)]
                if intent.task_type != "sequence":
                    intent.constraints["human_confirmed_task_type"] = confirmed["1"]
            traces.append(trace("command_bind", binding_started, binding, bound_response, "task_graph_grounded"))
        except ValueError as exc:
            intent = self._ask_human_intent(text, str(exc))
            if binding_started is not None:
                traces.append(trace("command_bind", binding_started, binding, bound_response, "ask_human:" + str(exc)))
            else:
                if traces:
                    traces[-1].adopted_decision = "ask_human:" + str(exc)
                else:
                    traces.append(trace("command_parse", started, questions, response, "ask_human:" + str(exc)))
        return JevParseDecision(intent, traces[0], traces[1:])

    def _parse_command_local(self, text: str, robot_state: dict[str, Any] | None = None) -> JevParseDecision:
        # Explicit offline mode: this is deterministic fallback, never live JEV.
        started = time.monotonic()
        intent = self._fallback_parse(text)
        questions = [{"id": "local_task_type", "type": "local", "result": intent.task_type}]
        trace = self._trace("command_parse", started, text, questions,
                            "ask_human" if intent.task_type == "ask_human" else "local_fallback")
        return JevParseDecision(intent, trace)

    def plan_review(
        self,
        text: str,
        intent: MissionIntent | None,
        plan: list[SkillCall],
        health: dict[str, Any] | None,
        *,
        classic_intent: MissionIntent | None = None,
        classic_plan: list[SkillCall] | None = None,
    ) -> JevCallTrace:
        started = time.monotonic()
        self._call_seq += 1
        del health
        classic_plan = classic_plan or []
        covers_all = bool(plan) and intent is not None and intent.task_type != "ask_human"
        classic_covers_all = bool(classic_plan) and classic_intent is not None and classic_intent.task_type != "ask_human"
        def known(call):
            return call.name in self.task_catalog.tasks if call.kind == "task" else bool(self.skill_catalog.get(call.name))
        supported = bool(plan) and all(known(call) for call in plan)
        classic_supported = bool(classic_plan) and all(known(call) for call in classic_plan)
        defaults_ok = self._plan_slot_defaults_reasonable(intent, plan)
        classic_defaults_ok = self._plan_slot_defaults_reasonable(classic_intent, classic_plan)
        confirmations = (intent.constraints or {}).get("human_confirmed_task_types", {}) if intent else {}
        if confirmations:
            def respects_confirmation(candidate):
                tasks = self.task_catalog.flatten(candidate) if candidate else []
                return all(0 < int(i) <= len(tasks) and tasks[int(i) - 1].task_type == kind
                           for i, kind in confirmations.items())
            defaults_ok = defaults_ok and respects_confirmation(intent)
            classic_defaults_ok = classic_defaults_ok and respects_confirmation(classic_intent)
        slots_need_human = bool(intent and intent.requires_confirmation)
        selected_source = "use_jev_plan" if covers_all and supported and defaults_ok and not slots_need_human else (
            "use_classic_plan" if classic_covers_all and classic_supported and classic_defaults_ok else "ask_human"
        )
        questions = [{"id": key, "type": "choice", "result": None, "answer_source": "not_evaluated"}
                     for key in ("jev_graph_faithful", "classic_graph_faithful")]
        questions.append({"id": "plan_source_decision", "type": "choice", "result": selected_source,
                          "answer_source": "local_structural_validation"})
        adopted = selected_source
        api_response = self._decide(
            "plan_review",
            {
                "stage_goal": "Review plans before any robot action. Compare the JEV Primary plan with the Classic parser plan only in this stage, then choose JEV, Classic, ask a human/LLM, or reject.",
                "user_command": text,
                "jev_task_graph": self._task_graph_from_intent(intent),
                "classic_task_graph": self._task_graph_from_intent(classic_intent),
                **({"human_confirmed_task_types": confirmations} if confirmations else {}),
                "structural_validation": {
                    "jev": {"present": covers_all, "registered_types": supported, "parameters_valid": defaults_ok},
                    "classic": {"present": classic_covers_all, "registered_types": classic_supported, "parameters_valid": classic_defaults_ok}},
                # Task graphs include arguments. Only legacy callers also need a skill-level plan.
                **({"jev_legacy_steps": self._compact_plan_for_jev(plan)} if any(c.kind != "task" for c in plan) else {}),
                **({"classic_legacy_steps": self._compact_plan_for_jev(classic_plan)} if any(c.kind != "task" for c in classic_plan) else {}),
            },
            self._plan_review_questions(),
        )
        if api_response:
            questions = self._questions_from_response(api_response, questions)
            start_action = self._answer_value(api_response, "plan_source_decision")
            if start_action in {"use_jev_plan", "use_classic_plan", "ask_human", "ask_thinking_llm", "reject"}:
                adopted = str(start_action)
            else:
                adopted = "ask_human"
            if adopted in {"use_jev_plan", "use_classic_plan"}:
                key = "jev_graph_faithful" if adopted == "use_jev_plan" else "classic_graph_faithful"
                if self._answer_value(api_response, key) != "faithful":
                    adopted = "ask_human"
        elif self.api_configured:
            adopted = "ask_human"
        if adopted == "use_jev_plan" and not (covers_all and supported and defaults_ok and not slots_need_human):
            adopted = "ask_human"
        if adopted == "use_classic_plan" and not (classic_covers_all and classic_supported and classic_defaults_ok
                                                 and not classic_intent.requires_confirmation):
            adopted = "ask_human"
        return self._trace("plan_review", started, text, questions, adopted, api_response=api_response)

    @classmethod
    def _plan_review_questions(cls):
        return {
                "jev_graph_faithful": cls._choice_question(
                    "Compare the concrete JEV task graph with the original command. Check every goal, exact source/argument ownership, "
                    "interpreting ambiguous text using human_confirmed_task_types when present. "
                    "order, per-task success dependencies, if-found true branch and stay-still false branch. No omitted qualifiers, "
                    "unrequested actions or invented success dependencies: temporal ordering alone does not require the earlier goal to succeed. "
                    "Deferred domain capability checks are not missing goals. Unknown model/preset availability is not "
                    "a Mission parsing failure. A source span selecting only part of a compound target is incomplete.",
                    {"faithful": "All goals and control flow are preserved", "incorrect": "An omission, wrong parameter or wrong branch exists", "unclear": "Cannot verify"}),
                "classic_graph_faithful": cls._choice_question(
                    "Compare the concrete Classic task graph with the original command, independently of JEV. Are all goals, parameters, "
                    "and the operator's human_confirmed_task_types preserved? "
                    "ordering and conditions preserved? Temporal ordering alone does not require success. "
                    "A missing/null plan or first-task-only parse of a multi-goal command is incorrect.",
                    {"faithful": "Complete and faithful", "incorrect": "Missing or incorrect plan", "unclear": "Cannot verify"}),
                "plan_source_decision": cls._choice_question(
                    "Compare the actual plans independently of other question answers. Choose a structurally valid, semantically "
                    "complete plan preserving all goals, argument roles and success conditions. Prefer JEV when both are equally faithful. "
                    "Respect the operator's human_confirmed_task_types as authoritative clarification of the original text; "
                    "do not reinterpret that choice or treat the resolved ambiguity as still open. "
                    "If neither is faithful, ask human or reject; never choose a first-goal-only plan for a multi-goal request.",
                    {
                        "use_jev_plan": "Execute the JEV Primary plan.",
                        "use_classic_plan": "Execute the Classic parser plan because it is better.",
                        "ask_human": "Ask the human before starting.",
                        "ask_thinking_llm": "Use slower LLM second opinion before starting.",
                        "reject": "Reject because it is unsafe or unsupported.",
                    },
                ),
            }

    def domain_questions(self, stage, intent, questions, defaults, evidence):
        """Answer bounded domain questions; malformed live answers never become defaults."""
        started = time.monotonic()
        self._call_seq += 1
        response = self._decide(stage, {"stage_goal": evidence.get("stage_goal", stage),
                                       "task": intent.to_dict(), "evidence": evidence}, questions)
        answers = {}
        for key, question in questions.items():
            value = self._answer_value(response, key) if response is not None else (
                None if self.api_configured else defaults.get(key))
            answers[key] = value if value in question.get("criteria", {}) else "unclear"
        trace = self._trace(stage, started, intent.raw_text,
                            [{"id": key, "type": "choice", "result": value} for key, value in answers.items()],
                            str(answers.get("decision", answers.get("coverage", "reviewed"))), api_response=response)
        return answers, trace

    def domain_review(self, stage, intent, choices, default, evidence):
        started = time.monotonic()
        self._call_seq += 1
        questions = {"decision": self._choice_question(
            "Refine only this domain task. Preserve the requested goal. Choose only an offered policy; "
            "unknown, conflicting or missing evidence requires ask_human. Never invent coordinates or models.", choices)}
        response = self._decide(stage, {"stage_goal": stage, "task": intent.to_dict(),
                                       "evidence": evidence}, questions)
        value = self._answer_value(response, "decision") if response else (None if self.api_configured else default)
        if value not in choices:
            value = "ask_human"
        trace = self._trace(stage, started, intent.raw_text,
                            [{"id": "decision", "type": "choice", "result": value}],
                            value, api_response=response)
        return value, trace

    def perception_plan_review(
        self,
        *,
        intent: MissionIntent | None,
        plan: list[SkillCall],
        health: dict[str, Any] | None,
    ) -> JevCallTrace:
        started = time.monotonic()
        self._call_seq += 1
        provider = self._perception_provider_from_plan(plan)
        scan_policy = self._scan_policy_from_plan(plan)
        task_kind = self._perception_task_kind(intent, plan)
        stationary = self._robot_stationary_from_health(health)
        start_decision = "start" if stationary is not False and task_kind != "unsupported" else "ask_human"
        questions = [
            {"id": "perception_task_kind", "type": "choice", "result": task_kind, "probabilities": {task_kind: 0.84, "unsupported": 0.08, "capture_only": 0.08}},
            {"id": "provider_choice", "type": "choice", "result": provider, "probabilities": {provider: 0.84, "stub": 0.08, "none_capture_only": 0.08}},
            {"id": "scan_policy", "type": "choice", "result": scan_policy, "probabilities": {scan_policy: 0.84, "current_view_only": 0.08, "no_camera_motion": 0.08}},
            {"id": "perception_plan_start_decision", "type": "choice", "result": start_decision, "probabilities": {start_decision: 0.84, "ask_human": 0.12, "reject": 0.04}},
        ]
        api_response = self._decide(
            "perception_plan_review",
            {
                "stage_goal": "Review the perception plan before camera/capture/detect execution.",
                "task_type": intent.task_type if intent else None,
                "object_query": intent.object_query if intent else None,
                "robot_stationary": stationary,
                "perception_capability_summary": self._perception_capability_summary(),
                "candidate_perception_plan": self._compact_plan_for_jev(plan),
            },
            {
                "perception_task_kind": self._choice_question(
                    "Classify the perception task kind.",
                    {
                        "capture_only": "User only asked to look/observe.",
                        "detect_in_current_view": "Run detector on current or requested view.",
                        "bounded_search_scan": "Move camera through bounded scan and detect.",
                        "arrival_confirmation": "Detect label/object to confirm arrived place.",
                        "unsupported": "Cannot form a meaningful perception plan.",
                    },
                ),
                "provider_choice": self._choice_question(
                    "Choose perception provider.",
                    {
                        "yolo11s": "Object/color detection without OCR requirement.",
                        "yolo11s_ocr": "Object detection plus OCR/text/label support.",
                        "stub": "Only stub is available; low confidence.",
                        "none_capture_only": "No detector provider needed.",
                    },
                ),
                "scan_policy": self._choice_question(
                    "Choose camera scan policy.",
                    {
                        "current_view_only": "Do not move camera; capture/detect current view.",
                        "directional_view": "Move camera to requested direction then capture.",
                        "three_view_middle": "Short left-center-right middle scan.",
                        "nine_pose_bounded": "Full bounded scan.",
                        "no_camera_motion": "Do not move camera.",
                    },
                ),
                "perception_plan_start_decision": self._choice_question(
                    "Choose perception plan action.",
                    {
                        "start": "Start perception plan.",
                        "ask_human": "Ask human to clarify target/query.",
                        "capture_only": "Capture observations but skip detection.",
                        "reject": "Reject unsupported perception request.",
                    },
                ),
            },
        )
        adopted = start_decision
        if api_response:
            questions = self._questions_from_response(api_response, questions)
            live_action = self._answer_value(api_response, "perception_plan_start_decision")
            if live_action in {"start", "ask_human", "capture_only", "reject"}:
                adopted = str(live_action)
        return self._trace(
            "perception_plan_review",
            started,
            intent.raw_text if intent else "",
            questions,
            adopted,
            api_response=api_response,
        )

    def supervise_step(
        self,
        *,
        user_text: str,
        intent: MissionIntent | None,
        plan: list[SkillCall],
        step_index: int,
        result: dict[str, Any],
        health: dict[str, Any] | None,
    ) -> JevSupervisionDecision:
        started = time.monotonic()
        self._call_seq += 1
        skill = str(result.get("skill_name") or "")
        owner = result.get("owner")
        stage = f"{owner}_step_supervision" if owner in {"perception", "navigation"} else "step_supervision"
        failed_child = (result.get("metrics") or {}).get("failed_child")
        contract = self.skill_catalog.compact_skill(failed_child or skill)
        skill_questions = (contract.get("supervision") or {}).get("questions", {})
        code = str(result.get("code") or "")
        success = bool(result.get("success"))
        next_call = self._next_meaningful_call(plan, step_index)
        category = "none_success" if success else self._failure_category(skill, code, health)
        needs_recovery = self._needs_recovery_agent(category, health)
        recovery_target = self._recovery_target_for_category(category)
        if (failed_child or skill) == "set_camera_pose" and category == "camera_or_perception_fault":
            recovery_target = "camera_control"
        blocks_goal = self._failure_blocks_goal(intent, skill, code)
        next_meaningful = next_call is not None and self._next_step_still_meaningful(intent, next_call)
        need_human = (not success) and next_meaningful and self._next_step_needs_human_after_failure(next_call, category)
        if success:
            action = "continue_next_step"
        elif skill == "await_robot_auto_mode" and code == codes.NEEDS_HUMAN:
            action = "ask_human_enable_auto"
        elif needs_recovery:
            action = "call_recovery_agent"
        elif need_human:
            action = "ask_human_continue_retry_stop"
        elif next_meaningful and not blocks_goal:
            action = "continue_next_step"
        else:
            action = "stop_failed"
        questions = [
            {"id": "step_completed_normally", "type": "noul", "result": success, "probabilities": {"true": 0.88 if success else 0.08, "false": 0.08 if success else 0.86, "null": 0.04 if success else 0.06}},
            {"id": "step_result_reasonable", "type": "noul", "result": success or category in {"target_not_found", "ambiguous_detection", "unsupported_query"}, "probabilities": {"true": 0.78 if success or category in {"target_not_found", "ambiguous_detection", "unsupported_query"} else 0.22, "false": 0.14 if success or category in {"target_not_found", "ambiguous_detection", "unsupported_query"} else 0.68, "null": 0.08 if success else 0.10}},
            {"id": "failure_category", "type": "choice", "result": category, "probabilities": {category: 0.86, "unknown": 0.14}},
            {"id": "needs_recovery_agent", "type": "noul", "result": needs_recovery, "probabilities": {"true": 0.78 if needs_recovery else 0.12, "false": 0.14 if needs_recovery else 0.82, "null": 0.08 if needs_recovery else 0.06}},
            {"id": "recovery_target", "type": "choice", "result": recovery_target, "probabilities": {recovery_target: 0.82, "unknown": 0.18}},
            {"id": "failure_blocks_current_subgoal", "type": "noul", "result": blocks_goal, "probabilities": {"true": 0.78 if blocks_goal else 0.18, "false": 0.16 if blocks_goal else 0.74, "null": 0.06 if blocks_goal else 0.08}},
            {"id": "next_subgoal_still_meaningful", "type": "noul", "result": next_meaningful, "probabilities": {"true": 0.76 if next_meaningful else 0.12, "false": 0.16 if next_meaningful else 0.82, "null": 0.08 if next_meaningful else 0.06}},
            {"id": "continue_requires_human", "type": "noul", "result": need_human, "probabilities": {"true": 0.82 if need_human else 0.10, "false": 0.12 if need_human else 0.82, "null": 0.06 if need_human else 0.08}},
            {"id": "supervision_action", "type": "choice", "result": action, "probabilities": {action: 0.80, "stop_failed": 0.12, "continue_next_step": 0.08}},
        ]
        questions.extend({"id": f"skill_{key}", "type": "noul", "result": None} for key in skill_questions)
        api_questions = deepcopy(self.skill_catalog.data["common_supervision_questions"])
        if skill != "await_robot_auto_mode" or code != codes.NEEDS_HUMAN:
            api_questions["supervision_action"]["criteria"].pop("ask_human_enable_auto", None)
        api_questions.update({f"skill_{key}": {"type": "noul", "instructions": question +
                              " Answer null if evidence is missing."} for key, question in skill_questions.items()})
        api_response = self._decide(
            stage,
            {
                "stage_goal": "Supervise the current robot skill result and decide whether to continue, recover, ask human, retry, or stop.",
                "execution_policy": {
                    "retry_replays_original_payload": True,
                    "automatic_replay_allowed": skill not in {"move_relative", "move_relative_xy", "rotate_relative"},
                    "recovery_success_requires_step_recheck": True,
                    "relative_motion_uses": "direct drive and wheel odometry; not Nav2 or global localization",
                    "health_fault_policy": "Missing required actuator or sensor feedback must be investigated by Recovery before abandoning a still-requested goal. Independent later goals remain pending. Normal target-not-found is not a health fault.",
                },
                "task_graph": self._task_graph_from_intent(intent),
                "plan_window": self._compact_plan_window_for_jev(plan, step_index),
                "current_step": {
                    "index": step_index,
                    "skill": skill,
                    "position": f"{step_index + 1}/{len(plan)}",
                    "skill_contract": contract,
                    "failed_child": failed_child,
                },
                "skill_result": self._compact_skill_result_for_jev(result, (contract.get("supervision") or {}).get("evidence_fields", [])),
                "next_step_group": self._next_step_group(next_call),
                "safety_summary": self._safety_summary(health),
            },
            api_questions,
        )
        if api_response:
            questions = self._questions_from_response(api_response, questions)
            live_action = self._answer_value(api_response, "supervision_action")
            if live_action in {
                "continue_next_step",
                "ask_human_continue_retry_stop",
                "ask_human_enable_auto",
                "retry_same_step",
                "retry_with_alternate_view",
                "skip_failed_subgoal_continue_independent_goal",
                "call_recovery_agent",
                "stop_failed",
                "record_design_gap",
            }:
                action = str(live_action)
            elif not api_response.get("_error"):
                action = "ask_human_continue_retry_stop"
        if action == "ask_human_enable_auto" and (skill != "await_robot_auto_mode" or code != codes.NEEDS_HUMAN):
            action = "ask_human_continue_retry_stop"
        execution_gate = None
        if action in {"continue_next_step", "skip_failed_subgoal_continue_independent_goal"} and not success:
            if code == codes.CANCELLED:
                action = "stop_failed"
            elif needs_recovery:
                action = "call_recovery_agent"
                execution_gate = "Required system feedback failed; request Recovery before continuing or abandoning the goal."
            elif not next_meaningful:
                action = "stop_failed"
        trace = self._trace(stage, started, user_text, questions, action, api_response=api_response)
        trace.execution_gate = execution_gate
        return JevSupervisionDecision(
            action=action,
            trace=trace,
            human_choices=self._human_choices_for_next(next_call if next_meaningful else None, user_text, category) if action == "ask_human_continue_retry_stop" else [],
            message=self._message_for_action(action, result, next_call if next_meaningful else None),
        )

    def _decide(self, stage: str, state: dict[str, Any], questions: dict[str, Any]) -> dict[str, Any] | None:
        if not self.is_live_enabled():
            return None
        started = time.monotonic()
        body = {
            "model": self.model,
            "state": {
                "schema_version": JEV_SCHEMA_VERSION,
                "stage": stage,
                **state,
            },
            "questions": questions,
        }
        response = self._post_decision(body)
        self._write_call_log(stage, body, response, (time.monotonic() - started) * 1000.0)
        if response.get("_error"):
            self.api_available = False
            self.api_status = "request_error"
            self.api_error = str(response.get("_error") or "")
        return response

    def _probe_api(self) -> None:
        probe_bodies = self._representative_probe_bodies()
        errors = []
        for body in probe_bodies:
            started = time.monotonic()
            response = self._post_decision(body)
            valid = bool(response and not response.get("_error") and isinstance(response.get("answers"), dict))
            if valid:
                valid = all(self._answer_value(response, key) in question["criteria"]
                            for key, question in body["questions"].items())
            if not valid:
                errors.append(str((response or {}).get("_error") or "missing_or_invalid_probe_answers"))
            self._write_call_log(body["state"]["stage"], body,
                                 response or {"_error": "empty_probe_response"},
                                 (time.monotonic() - started) * 1000, purpose="probe")
        if not errors:
            self.api_available = True
            self.api_status = "available"
            self.api_error = ""
            return
        self.api_available = False
        self.api_status = "unavailable"
        self.api_error = " | ".join(errors)

    def _representative_probe_bodies(self) -> list[dict[str, Any]]:
        from robot_agent.command_grounding import tokens, structure_questions, binding_questions
        command = "go back 20cm"
        intent = MissionIntent("move_relative", raw_text=command,
                               constraints={"direction": "back", "distance_m": .2})
        graph = self._task_graph_from_intent(intent)
        stages = [
            ("command_parse", "Identify goal-level tasks and control flow independently of Classic.",
             {}, structure_questions(self.task_catalog)),
            ("command_bind", "Ground classified tasks in exact source spans and per-task dependencies.",
             {"source_words": tokens(command), "task_types": ["move_relative"], "control_flow": "linear"},
             binding_questions(["move_relative"], tokens(command))),
            ("plan_review", "Review plans before action; this probe never executes a plan.",
             {"jev_task_graph": graph, "classic_task_graph": graph,
              "structural_validation": {source: {"present": True, "registered_types": True, "parameters_valid": True}
                                        for source in ("jev", "classic")}}, self._plan_review_questions()),
        ]
        return [{"model": self.model, "state": {"schema_version": JEV_SCHEMA_VERSION,
                 "stage": stage, "stage_goal": goal, "user_command": command, **state}, "questions": questions}
                for stage, goal, state, questions in stages]

    @staticmethod
    def _choice_question(instructions: str, criteria: dict[str, str]) -> dict[str, Any]:
        return {"type": "choice", "instructions": instructions, "criteria": criteria}

    @staticmethod
    def _score_question(instructions: str, criteria: list[str]) -> dict[str, Any]:
        return {"type": "score", "instructions": instructions, "criteria": criteria}

    def _post_decision(self, body: dict[str, Any]) -> dict[str, Any]:
        data = json.dumps(body, ensure_ascii=False).encode("utf-8")
        request = urllib.request.Request(
            self.api_url,
            data=data,
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                payload = response.read().decode("utf-8")
            parsed = json.loads(payload)
            if isinstance(parsed, dict):
                return parsed
            return {"_error": "jev_response_not_object"}
        except urllib.error.HTTPError as exc:
            try:
                detail = exc.read().decode("utf-8", errors="replace")
            except Exception:
                detail = ""
            message = f"HTTPError: HTTP Error {exc.code}: {exc.reason}"
            if detail:
                message = f"{message}; body={detail[:500]}"
            return {"_error": message}
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            return {"_error": f"{type(exc).__name__}: {exc}"}

    def _trace(
        self,
        stage: str,
        started: float,
        prompt_seed: str,
        questions: list[dict[str, Any]],
        decision: str,
        *,
        api_response: dict[str, Any] | None = None,
    ) -> JevCallTrace:
        ended = time.monotonic()
        usage = api_response.get("usage", {}) if isinstance(api_response, dict) else {}
        live_answer = bool(api_response and not api_response.get("_error") and isinstance(api_response.get("answers"), dict))
        input_tokens = int(usage.get("input_tokens") or usage.get("prompt_tokens") or (
            max(1, len(prompt_seed) // 4 + len(questions) * 32) if live_answer else 0))
        input_cost = input_tokens / 1_000_000.0 * self.PRICE_PER_MILLION_INPUT_TOKENS_USD
        fallback_used = bool(api_response and api_response.get("_error"))
        if fallback_used:
            questions = [
                *questions,
                {
                    "id": "jev_api_error",
                    "type": "error",
                    "result": api_response.get("_error"),
                },
            ]
        return JevCallTrace(
            call_id=f"jev_{self._call_seq:04d}_{stage}",
            stage=stage,
            provider="live" if live_answer else "error" if fallback_used else "local",
            model=self.model if live_answer or fallback_used else "jev-local-v0",
            question_count=len(questions),
            started_at_monotonic=started,
            ended_at_monotonic=ended,
            latency_ms=(ended - started) * 1000.0,
            input_tokens_est=input_tokens,
            input_cost_usd=input_cost,
            questions=questions,
            adopted_decision=decision,
            fallback_used=fallback_used,
        )

    @staticmethod
    def _questions_from_response(api_response: dict[str, Any], fallback: list[dict[str, Any]]) -> list[dict[str, Any]]:
        if api_response.get("_error"):
            return [{"id": item.get("id"), "type": item.get("type"), "result": None,
                     "answer_source": "api_error"} for item in fallback]
        answers = api_response.get("answers")
        if not isinstance(answers, dict):
            answers = {}
        # A live answer must not inherit invented local confidence/probabilities.
        by_id = {item.get("id"): {"id": item.get("id"), "type": item.get("type"),
                                  "result": None, "answer_source": "missing"} for item in fallback}
        ordered_ids = [item.get("id") for item in fallback]
        for question_id, answer in answers.items():
            if not isinstance(answer, dict):
                continue
            item = by_id.get(question_id, {"id": question_id})
            item["answer_source"] = "api"
            item["type"] = item.get("type") or answer.get("type") or "unknown"
            if "choice" in answer:
                item["result"] = answer.get("choice")
            elif "score" in answer:
                item["result"] = answer.get("score")
            elif "noul" in answer:
                item["result"] = answer.get("noul")
            elif "answer" in answer:
                item["result"] = answer.get("answer")
            if isinstance(answer.get("probabilities"), dict):
                item["probabilities"] = answer.get("probabilities")
            if "confidence" in answer:
                item["confidence"] = answer.get("confidence")
            if "legend" in answer:
                item["legend"] = answer["legend"]
            by_id[question_id] = item
            if question_id not in ordered_ids:
                ordered_ids.append(question_id)
        return [by_id[item_id] for item_id in ordered_ids if item_id in by_id]

    @staticmethod
    def _answer_value(api_response: dict[str, Any], question_id: str) -> Any:
        answers = api_response.get("answers") if isinstance(api_response, dict) else None
        answer = answers.get(question_id) if isinstance(answers, dict) else None
        if not isinstance(answer, dict):
            return None
        for key in ("choice", "score", "noul", "answer"):
            if key in answer:
                value = answer[key]
                return value if value is None or isinstance(value, (str, int, float, bool)) else None
        return None

    @staticmethod
    def _as_float(value: Any, default: float = 0.0) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @classmethod
    def _as_bool_probability(cls, value: Any) -> float:
        if isinstance(value, bool):
            return 1.0 if value else 0.0
        if value is None:
            return 0.5
        return cls._as_float(value, 0.5)

    @staticmethod
    def _has_sequence_cue(text: str) -> bool:
        return bool(re.search(r"(?:\bthen\b|\bafter\b|\band\b|然后|然後|それから|そして)", str(text), re.IGNORECASE))

    def _fallback_parse(self, text: str) -> MissionIntent:
        guard = command_guard_reason(text)
        if guard:
            return self._ask_human_intent(text, guard)
        local_intent = self.parser.parse(text)
        candidates = self.candidate_generator.generate(text, local_intent=local_intent)
        default_candidate = self._default_candidate(candidates)
        if default_candidate is not None and (
            local_intent.task_type == "ask_human" or self._candidate_is_more_complete(default_candidate, local_intent)
        ):
            return self.task_catalog.normalize(self._intent_from_candidate(default_candidate, text))
        return self.task_catalog.normalize(local_intent)

    def _fallback_step_question_results(self, intent: MissionIntent) -> list[dict[str, Any]]:
        steps = self._intent_steps(intent)
        out: list[dict[str, Any]] = []
        for index in range(1, 4):
            task_type = "not_applicable"
            if index <= len(steps):
                task_type = str(steps[index - 1].get("task_type") or "ask_human")
            out.append(
                {
                    "id": f"step{index}_task_type",
                    "type": "choice",
                    "result": task_type,
                    "probabilities": {task_type: 0.80, "ask_human": 0.12, "not_applicable": 0.08},
                }
            )
        return out


    def _intent_for_jev_task(
        self, task_type, raw_text, api_response, step_index,
    ) -> MissionIntent:
        catalog = self.task_catalog
        if self._answer_value(api_response, f"step{step_index}_slot_resolution_action") == "ask_human_for_slots":
            return self._ask_human_intent(raw_text, "jev_required_slots_unclear")
        canonical = catalog.aliases.get(task_type, task_type)
        if canonical == "set_arm_pose":
            raw_text = re.sub(r"\s+regardless of whether it is found\s*[.!]?\s*$", "", raw_text, flags=re.IGNORECASE)
        if canonical not in catalog.tasks:
            return self._ask_human_intent(raw_text, "unknown_task_type")
        # Numeric extraction remains code-owned, after JEV selects the intent.
        parsed = catalog.normalize(self.parser.parse(raw_text))
        if parsed.task_type == canonical:
            return parsed
        legacy = catalog.tasks[canonical]["legacy"]
        if canonical == "interact_with_object":
            return self._ask_human_intent(raw_text, "interaction_target_or_operation_unclear")
        if canonical == "manipulate_object":
            state = "released" if re.search(r"release|松开|放开", raw_text, re.I) else "placed_at" if re.search(r"place|put|放到", raw_text, re.I) else "held"
            return MissionIntent(canonical, object_query=raw_text.strip(), raw_text=raw_text,
                                 constraints={"desired_end_state": state, "unresolved_target": True})
        result = self._legacy_intent_for_jev_task(legacy, raw_text, api_response, step_index)
        return catalog.normalize(result)

    def _legacy_intent_for_jev_task(
        self,
        task_type: str,
        raw_text: str,
        api_response: dict[str, Any],
        step_index: int,
    ) -> MissionIntent:
        if task_type == "set_pose":
            raw_text = re.sub(r"\s+regardless of whether it is found\s*[.!]?\s*$", "", raw_text, flags=re.IGNORECASE)
        code_intent = self.parser.parse(raw_text)
        if code_intent.task_type == task_type and self._slots_complete_or_defaultable(code_intent):
            return code_intent
        candidates = self.candidate_generator.generate(raw_text, local_intent=code_intent)
        candidate = self._candidate_for_task(candidates, task_type)
        if candidate is not None:
            return self._intent_from_candidate(candidate, raw_text)
        prefix = f"step{step_index}"
        if task_type == "move_relative":
            direction = self._answer_value(api_response, f"{prefix}_move_direction")
            if direction in {"ask_human", "not_applicable", None, ""}:
                direction = self._extract_move_direction(raw_text)
            distance_policy = str(self._answer_value(api_response, f"{prefix}_distance_policy") or "")
            distance = self._extract_distance_m(raw_text)
            if distance is None:
                distance = 0.2 if distance_policy == "small_default_0_2m" or self._is_small_distance_phrase(raw_text) else 0.5
            if not direction:
                return self._ask_human_intent(raw_text, "jev_move_direction_unclear")
            return MissionIntent(task_type="move_relative", constraints={"direction": str(direction), "distance_m": distance}, raw_text=raw_text)
        if task_type == "rotate_relative":
            yaw = self._extract_yaw_deg(raw_text)
            if yaw is None:
                policy = str(self._answer_value(api_response, f"{prefix}_turn_policy") or "")
                yaw = {"left_90deg": 90, "right_90deg": -90, "around_180deg": 180}.get(policy)
            if yaw is None:
                return self._ask_human_intent(raw_text, "jev_turn_angle_unclear")
            return MissionIntent(task_type="rotate_relative", constraints={"yaw_deg": yaw}, raw_text=raw_text)
        if task_type == "look_direction":
            direction = self._answer_value(api_response, f"{prefix}_look_direction")
            if direction in {"ask_human", "not_applicable", None, ""}:
                direction = self._extract_look_direction(raw_text)
            if not direction:
                return self._ask_human_intent(raw_text, "jev_look_direction_unclear")
            return MissionIntent(task_type="look_direction", constraints={"direction": str(direction), "band": "middle"}, raw_text=raw_text)
        if task_type == "set_pose":
            pose = self._answer_value(api_response, f"{prefix}_pose_name")
            if pose in {"ask_human", "not_applicable", None, ""}:
                pose = "home" if re.search(r"\bhome\b|ホーム", raw_text, re.IGNORECASE) else None
            if not pose:
                return self._ask_human_intent(raw_text, "jev_pose_unclear")
            return MissionIntent(task_type="set_pose", target=str(pose), raw_text=raw_text)
        if task_type in {"search_object_here", "search_object_near_place"}:
            query = self._extract_object_query(raw_text)
            if not query:
                return self._ask_human_intent(raw_text, "jev_object_query_unclear")
            return MissionIntent(task_type=task_type, object_query=query, raw_text=raw_text)
        if task_type in {"navigate_to_place", "navigate_to_object", "inspect_place"}:
            target = self._extract_navigation_target(raw_text) or raw_text.strip()
            return MissionIntent(task_type=task_type, target=target, raw_text=raw_text)
        if task_type in {"cancel", "robot_status", "list_known_targets"}:
            return MissionIntent(task_type=task_type, raw_text=raw_text)
        return self._ask_human_intent(raw_text, f"jev_unsupported_task_type:{task_type}")

    @staticmethod
    def _ask_human_intent(text: str, reason: str) -> MissionIntent:
        return MissionIntent(
            task_type="ask_human",
            raw_text=text,
            requires_confirmation=True,
            constraints={"ambiguity": {"reason": reason, "choices": [{"label": "Cancel", "command": "cancel"}]}},
        )

    @staticmethod
    def _split_command_steps(text: str, desired_count: int) -> list[str]:
        branches = conditional_parts(text)
        if branches:
            return branches if desired_count == 2 else []
        explicit_parts = [p.strip(" ,，.。") for p in STEP_SEPARATOR.split(text) if p.strip(" ,，.。")]
        if len(explicit_parts) > desired_count:
            return []
        if len(explicit_parts) == desired_count:
            return explicit_parts
        parts = [
            part.strip(" ,，.。")
            for part in SkillCandidateGenerator._SEQUENCE_SPLIT_RE.split(str(text or ""))
            if part.strip(" ,，.。")
        ]
        if len(parts) > desired_count:
            return []
        if len(parts) == desired_count:
            return parts
        return [str(text or "").strip()] if desired_count == 1 else []

    @staticmethod
    def _intent_steps(intent: MissionIntent | None) -> list[dict[str, Any]]:
        if intent is None:
            return []
        if intent.task_type == "sequence":
            steps = intent.constraints.get("steps") if isinstance(intent.constraints, dict) else None
            return [step for step in steps if isinstance(step, dict)] if isinstance(steps, list) else []
        return [intent.to_dict()]

    @staticmethod
    def _extract_move_direction(text: str) -> str | None:
        lowered = text.lower()
        if re.search(r"\bback\b|\bbackward\b|\bback\s+up\b|後ろ|戻", lowered):
            return "back"
        if re.search(r"\bforward\b|\bahead\b|前進|前へ", lowered):
            return "forward"
        if re.search(r"\bleft\b|左", lowered):
            return "left"
        if re.search(r"\bright\b|右", lowered):
            return "right"
        return None

    @staticmethod
    def _extract_look_direction(text: str) -> str | None:
        lowered = text.lower()
        if re.search(r"\bleft\b|左", lowered):
            return "left"
        if re.search(r"\bright\b|右", lowered):
            return "right"
        if re.search(r"\bfront\b|\bforward\b|前", lowered):
            return "front"
        return None

    @staticmethod
    def _extract_distance_m(text: str) -> float | None:
        match = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|meter|meters|メートル)", text, re.IGNORECASE)
        if not match:
            return None
        try:
            return float(match.group(1))
        except ValueError:
            return None

    @staticmethod
    def _is_small_distance_phrase(text: str) -> bool:
        return bool(re.search(r"\ba\s+little\b|\blittle\b|少し|ちょっと", text, re.IGNORECASE))

    @staticmethod
    def _extract_yaw_deg(text: str) -> float | None:
        match = re.search(r"(-?\d+(?:\.\d+)?)\s*(?:deg|degree|degrees|度)", text, re.IGNORECASE)
        if match:
            try:
                return float(match.group(1))
            except ValueError:
                return None
        lowered = text.lower()
        if re.search(r"turn\s+around|rotate\s+around|180|後ろを向", lowered):
            return 180.0
        if re.search(r"\bleft\b|左", lowered):
            return 90.0
        if re.search(r"\bright\b|右", lowered):
            return -90.0
        return None

    @staticmethod
    def _extract_object_query(text: str) -> str | None:
        lowered = text.lower().strip()
        match = re.search(r"(?:find|search|check|detect|verify|confirm|探し|探す)\s+(?:the\s+)?(.+)", lowered)
        if match:
            query = re.split(r"\b(?:then|after|and)\b|然后|それから", match.group(1), maxsplit=1)[0].strip(" ,.")
            return query or None
        color_obj = re.search(r"\b(red|blue|green|yellow|orange|black|white)\s+([a-z0-9_-]+)", lowered)
        if color_obj:
            return f"{color_obj.group(1)} {color_obj.group(2)}"
        return None

    @staticmethod
    def _extract_navigation_target(text: str) -> str | None:
        match = re.search(r"(?:go\s+to|navigate\s+to|移動|行って?)\s+(.+)", text, re.IGNORECASE)
        if not match:
            return None
        target = re.split(r"\b(?:then|after|and)\b|然后|それから", match.group(1), maxsplit=1)[0].strip(" ,.")
        return target or None

    @staticmethod
    def _default_candidate(candidates: list[CommandCandidate]) -> CommandCandidate | None:
        for candidate in candidates:
            if candidate.candidate_id != "ask_human" and len(candidate.steps) > 1:
                return candidate
        for candidate in candidates:
            if candidate.candidate_id != "ask_human":
                return candidate
        return candidates[0] if candidates else None

    @staticmethod
    def _default_sequence_candidate(candidates: list[CommandCandidate]) -> CommandCandidate | None:
        for candidate in candidates:
            if candidate.candidate_id != "ask_human" and len(candidate.steps) > 1:
                return candidate
        return None

    @staticmethod
    def _candidate_for_task(candidates: list[CommandCandidate], task_type: str) -> CommandCandidate | None:
        if not task_type or task_type in {"not_applicable", "ask_human"}:
            return None
        for candidate in candidates:
            if candidate.candidate_id == "ask_human" or len(candidate.steps) != 1:
                continue
            step_type = str(candidate.steps[0].get("task_type") or candidate.task_type)
            if step_type == task_type:
                return candidate
        return None

    @staticmethod
    def _candidate_by_id(candidates: list[CommandCandidate], candidate_id: str) -> CommandCandidate | None:
        for candidate in candidates:
            if candidate.candidate_id == candidate_id:
                return candidate
        return None

    @staticmethod
    def _candidate_is_more_complete(candidate: CommandCandidate, intent: MissionIntent) -> bool:
        return len(candidate.steps) > 1 and intent.task_type != "sequence"

    @staticmethod
    def _candidate_summary(candidate: CommandCandidate) -> str:
        if candidate.candidate_id == "ask_human":
            return "Ask the user to clarify; do not execute robot skills."
        steps = " -> ".join(str(step.get("task_type")) for step in candidate.steps)
        return f"{steps}; risk={candidate.risk}; source={','.join(candidate.source)}"

    @staticmethod
    def _command_shape_for_intent(intent: MissionIntent | None) -> str:
        if intent is None:
            return "ambiguous_or_unsupported"
        if intent.task_type == "sequence":
            return "ordered_sequence"
        if intent.task_type in {"robot_status", "cancel", "list_known_targets"}:
            return "status_or_cancel"
        if intent.task_type == "ask_human":
            return "ambiguous_or_unsupported"
        return "single_step"

    @staticmethod
    def _sequence_step_count(intent: MissionIntent | None) -> str:
        if intent is None or intent.task_type != "sequence":
            return "one"
        steps = intent.constraints.get("steps")
        count = len(steps) if isinstance(steps, list) else 0
        if count == 1:
            return "one"
        if count == 2:
            return "two"
        if count == 3:
            return "three"
        if count > 3:
            return "more_than_three"
        return "unclear"

    @staticmethod
    def _dependency_policy_for_intent(intent: MissionIntent | None) -> str:
        if intent is None:
            return "ambiguous_dependency"
        if isinstance(intent.constraints, dict) and intent.constraints.get("dependency_policy"):
            return str(intent.constraints.get("dependency_policy"))
        if intent.task_type != "sequence":
            return "independent_goals"
        steps = intent.constraints.get("steps")
        if not isinstance(steps, list) or len(steps) < 2:
            return "ambiguous_dependency"
        task_types = [str(step.get("task_type") or "") for step in steps if isinstance(step, dict)]
        if task_types and task_types[-1] == "set_pose":
            return "cleanup_after_attempt"
        return "ordered_but_independent"

    @staticmethod
    def _slots_complete_or_defaultable(intent: MissionIntent | None) -> bool:
        if intent is not None and intent.task_type == "observe_scene" and intent.constraints.get("profile") in {"current_view", "surroundings"}:
            return True
        if intent is not None and intent.task_type != "sequence":
            intent = TaskCatalog().legacy(intent)
        if intent is None or intent.task_type == "ask_human":
            return False
        if intent.task_type == "sequence":
            steps = intent.constraints.get("steps")
            if not isinstance(steps, list) or not steps:
                return False
            return all(
                JevStrengthenAgent._slots_complete_or_defaultable(
                    MissionIntent(
                        task_type=str(step.get("task_type") or "ask_human"),
                        target=step.get("target"),
                        object_query=step.get("object_query"),
                        constraints=step.get("constraints") if isinstance(step.get("constraints"), dict) else {},
                    )
                )
                for step in steps
                if isinstance(step, dict)
            )
        if intent.task_type == "move_relative":
            return bool(intent.constraints.get("direction")) and "distance_m" in intent.constraints
        if intent.task_type == "move_relative_xy":
            return "dx_m" in intent.constraints and "dy_m" in intent.constraints
        if intent.task_type == "rotate_relative":
            return "yaw_deg" in intent.constraints
        if intent.task_type in {"search_object_here", "search_object_near_place"}:
            return bool(intent.object_query)
        if intent.task_type == "look_direction":
            return bool(intent.constraints.get("direction") or intent.constraints.get("unresolved_view"))
        if intent.task_type == "set_pose":
            return bool(intent.target)
        if intent.task_type in {"navigate_to_place", "navigate_to_object", "inspect_place"}:
            return bool(intent.target)
        return True

    @staticmethod
    def _slot_resolution_action(intent: MissionIntent | None) -> str:
        if intent is None:
            return "ask_human_for_slots"
        if intent.task_type == "sequence":
            return "not_applicable"
        if not JevStrengthenAgent._slots_complete_or_defaultable(intent):
            return "ask_human_for_slots"
        defaults = {"move_relative", "move_relative_xy", "rotate_relative", "look_direction"}
        if intent.task_type in defaults:
            return "use_catalog_defaults"
        return "use_extracted_slots"

    @staticmethod
    def _task_graph_from_intent(intent: MissionIntent | None) -> dict[str, Any] | None:
        if intent is None:
            return None
        if intent.task_type == "sequence":
            steps = []
            for index, step in enumerate(intent.constraints.get("steps", []) if isinstance(intent.constraints, dict) else []):
                if not isinstance(step, dict):
                    continue
                steps.append(
                    {
                        "step_id": f"s{index + 1}",
                        "task_type": step.get("task_type"),
                        "target": step.get("target"),
                        "object_query": step.get("object_query"),
                        "source_text": step.get("raw_text"),
                        "slots": step.get("constraints") if isinstance(step.get("constraints"), dict) else {},
                        "dependency_role": ("condition_source" if index == 0 and intent.constraints.get("else_action") == "stay_still"
                                            else "conditional_on_previous_search_found" if step.get("constraints", {}).get("run_if") == "previous_search_found"
                                            else step.get("constraints", {}).get("dependency_policy", "independent_goal" if index == 0 else JevStrengthenAgent._dependency_policy_for_intent(intent))),
                    }
                )
            return {
                "steps": steps,
                "connector": "if_found" if intent.constraints.get("else_action") == "stay_still" else "then" if steps else None,
                "else_action": intent.constraints.get("else_action"),
                "dependency_policy": JevStrengthenAgent._dependency_policy_for_intent(intent),
            }
        return {
            "steps": [
                {
                    "step_id": "s1",
                    "task_type": intent.task_type,
                    "target": intent.target,
                    "object_query": intent.object_query,
                    "source_text": intent.raw_text,
                    "slots": intent.constraints if isinstance(intent.constraints, dict) else {},
                    "dependency_role": "independent_goal",
                }
            ],
            "connector": None,
            "dependency_policy": "independent_goals",
        }

    @staticmethod
    def _supported_task_type_criteria() -> dict[str, str]:
        criteria = {name: spec["description"] for name, spec in TaskCatalog().tasks.items()}
        return {**criteria, "ask_human": "Clarification required.", "not_applicable": "No such task."}

    @classmethod
    def _supported_task_types(cls) -> list[dict[str, Any]]:
        return [{"task_type": name, "description": spec["description"], "owner": spec["owner"],
                 "slots": spec["slots"]} for name, spec in TaskCatalog().tasks.items()]

    @staticmethod
    def _task_type_slots(task_type: str) -> dict[str, Any]:
        return TaskCatalog().tasks.get(task_type, {}).get("slots", {})

    @staticmethod
    def _default_policy_summary() -> dict[str, Any]:
        return {
            "move_relative": {
                "distance_words": {"a little": 0.2, "少し": 0.2},
                "missing_distance": "ask_human_or_safe_small_default_by_policy",
            },
            "rotate_relative": {
                "left": 90,
                "right": -90,
                "turn around": 180,
                "missing_angle": "catalog_default_if_direction_word_is_clear",
            },
            "look_direction": {"missing_band": "middle"},
        }

    @staticmethod
    def _slot_resolution_summary(intent: MissionIntent | None, plan: list[SkillCall]) -> dict[str, Any]:
        graph = JevStrengthenAgent._task_graph_from_intent(intent) or {}
        summary: dict[str, Any] = {"task_graph_slots": graph.get("steps") or [], "plan_args": {}}
        for index, call in enumerate(plan):
            summary["plan_args"][f"{index}:{call.name}"] = {
                key: {"source": "code_or_catalog_default", "value": value}
                for key, value in call.payload.items()
            }
        return summary

    def _compact_plan_for_jev(self, plan: list[SkillCall]) -> list[dict[str, Any]]:
        return [self._compact_call_for_jev(index, call) for index, call in enumerate(plan or [])]

    def _compact_plan_window_for_jev(self, plan: list[SkillCall], step_index: int) -> dict[str, Any]:
        start = max(0, step_index - 1)
        end = min(len(plan), step_index + 3)
        return {
            "total_steps": sum(call.kind != "task_end" for call in plan),
            "current_index": step_index,
            "window": [self._compact_call_for_jev(index, plan[index]) for index in range(start, end)
                       if plan[index].kind != "task_end"],
        }

    def _compact_call_for_jev(self, index: int, call: SkillCall) -> dict[str, Any]:
        if call.kind == "task":
            return {"index": index, "task_type": call.name, "owner": call.owner,
                    "goal": call.payload, "kind": "task"}
        payload = {
            key: value
            for key, value in (call.payload or {}).items()
            if key
            in {
                "direction",
                "distance_m",
                "dx_m",
                "dy_m",
                "yaw_deg",
                "target_name",
                "relation",
                "query",
                "provider",
                "pose",
                "pose_candidates",
                "name",
                "pan_deg",
                "tilt_deg",
                "scan_band",
                "timeout_sec",
            }
        }
        return {
            "index": index,
            "name": call.name,
            "payload": payload,
            "timeout_sec": call.timeout_sec,
            "reason": call.reason[:160],
            "risk_label": ((call.catalog or {}).get("risk_label") if isinstance(call.catalog, dict) else None),
        }

    @staticmethod
    def _compact_skill_result_for_jev(result: dict[str, Any], evidence_fields=()) -> dict[str, Any]:
        metrics = result.get("metrics") if isinstance(result.get("metrics"), dict) else {}
        keep_metrics = {
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
                "detections_count",
                "objects_count",
                "image_topic",
                "provider",
                "target_found",
                "completed_time_monotonic",
                "inference_completed", "goal_satisfied", "outcome", "required_views", "usable_views",
                "failed_child", "provider_errors", "provider_warnings", "unresolved_requirements",
            }
            or key in evidence_fields
        }
        artifacts = result.get("artifacts")
        for key in ("provider_errors", "provider_warnings"):
            if isinstance(keep_metrics.get(key), list):
                keep_metrics[key] = [{"backend": x.get("backend"), "error": str(x.get("error", ""))[:160]}
                                     for x in keep_metrics[key][:3] if isinstance(x, dict)]
        keep_metrics["recent_children"] = metrics.get("children", [])[-6:]
        keep_metrics["check_results"] = [{"query": x.get("query"), "code": x.get("code"),
                                         "goal_satisfied": (x.get("metrics") or {}).get("goal_satisfied"),
                                         "outcome": (x.get("metrics") or {}).get("outcome")}
                                        for x in metrics.get("check_results", [])[:12]]
        artifact_summary = None
        if isinstance(artifacts, list):
            artifact_summary = [{"type": item.get("type"), "topic": item.get("topic")} for item in artifacts[:5] if isinstance(item, dict)]
        return {
            "skill_name": result.get("skill_name"),
            "success": result.get("success"),
            "code": result.get("code"),
            "message": str(result.get("message") or "")[:240],
            "metrics": keep_metrics,
            "artifact_summary": artifact_summary,
        }

    @staticmethod
    def _plan_slot_defaults_reasonable(intent: MissionIntent | None, plan: list[SkillCall]) -> bool:
        if intent is None or not plan:
            return False
        for call in plan:
            if call.kind == "task":
                task = MissionIntent(**call.payload)
                if not JevStrengthenAgent._slots_complete_or_defaultable(task):
                    return False
                args = task.constraints
                if call.name == "move_relative":
                    values = [args[k] for k in ("distance_m", "dx_m", "dy_m") if k in args]
                    if not values or any(not isinstance(v, (int, float)) or not -3 <= v <= 3 for v in values):
                        return False
                if call.name == "rotate_relative" and not -360 <= args.get("yaw_deg", 1000) <= 360:
                    return False
                continue
            if call.name == "move_relative":
                distance = call.payload.get("distance_m")
                if not isinstance(distance, (int, float)) or float(distance) <= 0 or float(distance) > 2.0:
                    return False
            if call.name == "rotate_relative":
                yaw = call.payload.get("yaw_deg")
                if not isinstance(yaw, (int, float)) or abs(float(yaw)) > 360.0:
                    return False
            if call.name == "detect_known_object" and not call.payload.get("query"):
                return False
        return True

    @staticmethod
    def _perception_provider_from_plan(plan: list[SkillCall]) -> str:
        for call in plan:
            if call.name == "detect_known_object":
                provider = str(call.payload.get("provider") or "")
                if provider:
                    return provider
        return "none_capture_only"

    @staticmethod
    def _scan_policy_from_plan(plan: list[SkillCall]) -> str:
        poses = [call for call in plan if call.name == "set_camera_pose"]
        detections = [call for call in plan if call.name == "detect_known_object"]
        if not poses:
            return "current_view_only" if detections else "no_camera_motion"
        non_return = [
            call
            for call in poses
            if str(call.payload.get("scan_band") or "") != "return_home"
        ]
        count = len(non_return)
        if count >= 9:
            return "nine_pose_bounded"
        if count >= 3:
            return "three_view_middle"
        if count == 1:
            return "directional_view"
        return "current_view_only"

    @staticmethod
    def _perception_task_kind(intent: MissionIntent | None, plan: list[SkillCall]) -> str:
        if not plan:
            return "unsupported"
        has_detect = any(call.name == "detect_known_object" for call in plan)
        if intent and intent.task_type == "inspect_place":
            return "arrival_confirmation"
        if has_detect and JevStrengthenAgent._scan_policy_from_plan(plan) in {"three_view_middle", "nine_pose_bounded"}:
            return "bounded_search_scan"
        if has_detect:
            return "detect_in_current_view"
        return "capture_only"

    @staticmethod
    def _robot_stationary_from_health(health: dict[str, Any] | None) -> bool | None:
        motion = ((health or {}).get("components") or {}).get("motion") or {}
        if not isinstance(motion, dict):
            return None
        summary = str(motion.get("summary") or "").lower()
        if "moving" in summary:
            return False
        if motion.get("level") in {"OK", "IDLE"}:
            return True
        return None

    @staticmethod
    def _perception_capability_summary() -> dict[str, Any]:
        return {
            "providers": ["yolo11s", "yolo11s_ocr", "stub"],
            "known_colors": ["red", "blue", "green", "yellow", "orange", "black", "white"],
            "known_objects_examples": ["box", "bottle", "car", "person", "chair"],
            "ocr_supported": True,
            "shelf_label_supported": True,
        }

    @staticmethod
    def _intent_from_candidate(candidate: CommandCandidate, raw_text: str) -> MissionIntent:
        if candidate.candidate_id == "ask_human":
            return MissionIntent(task_type="ask_human", raw_text=raw_text, requires_confirmation=True)
        if len(candidate.steps) > 1 or candidate.task_type == "sequence":
            steps = []
            for step in candidate.steps:
                steps.append(
                    {
                        "task_type": step.get("task_type"),
                        "target": step.get("target"),
                        "object_query": step.get("object_query"),
                        "constraints": step.get("constraints") or {},
                        "raw_text": step.get("raw_text") or raw_text,
                    }
                )
            return MissionIntent(task_type="sequence", constraints={"steps": steps}, raw_text=raw_text)
        step = candidate.steps[0] if candidate.steps else {}
        return MissionIntent(
            task_type=str(step.get("task_type") or candidate.task_type),
            target=step.get("target"),
            object_query=step.get("object_query"),
            constraints=step.get("constraints") if isinstance(step.get("constraints"), dict) else {},
            raw_text=raw_text,
        )

    @staticmethod
    def _risk_score(plan: list[SkillCall]) -> float:
        names = {call.name for call in plan}
        if names & {"touch", "grasp", "move_lift_bounded"}:
            return 0.65
        if names & {"goto_pose", "move_relative", "move_relative_xy", "rotate_relative"}:
            return 0.45
        if names & {"set_pose"}:
            return 0.35
        return 0.15

    @staticmethod
    def _failure_category(skill: str, code: str, health: dict[str, Any] | None) -> str:
        if skill == "search_at_place":
            skill = "detect_known_object"
        if skill == "detect_known_object" and code == codes.FAILED_PERCEPTION:
            return "camera_or_perception_fault"
        if skill == "detect_known_object" and code == codes.NEEDS_HUMAN:
            return "target_not_found"
        if code == codes.PERCEPTION_STUB:
            return "camera_or_perception_fault"
        if skill in {"capture_observation", "set_camera_pose"}:
            return "camera_or_perception_fault"
        if skill in {"move_relative", "move_relative_xy", "rotate_relative"} and code == codes.CANCELLED:
            components = (health or {}).get("components") or {}
            localization_bad = (components.get("localization") or {}).get("level") == "BAD"
            return "localization_unrelated" if localization_bad else "control_mode_block"
        if code == codes.FAILED_LOCALIZATION:
            return "localization_related"
        if code == codes.FAILED_NAVIGATION:
            if skill in {"move_relative", "move_relative_xy", "rotate_relative"}:
                return "relative_motion_fault"
            return "local_planner_fault"
        if code == codes.FAILED_TIMEOUT:
            return "timeout_or_load"
        return "unknown"

    @staticmethod
    def _failure_blocks_goal(intent: MissionIntent | None, skill: str, code: str) -> bool:
        if skill == "detect_known_object" and code == codes.NEEDS_HUMAN:
            return True
        if intent and intent.task_type == "look_direction" and skill == "detect_known_object":
            return False
        return code not in {codes.OK, codes.DRY_RUN_OK}

    @staticmethod
    def _is_safe_independent_next_step(call: SkillCall | None) -> bool:
        if call is None:
            return False
        return call.name in {
            "await_robot_stationary",
            "await_robot_auto_mode",
            "set_pose",
            "set_camera_pose",
            "capture_observation",
            "cancel_motion",
            "move_relative",
            "move_relative_xy",
            "rotate_relative",
        }

    @staticmethod
    def _next_meaningful_call(plan: list[SkillCall], step_index: int) -> SkillCall | None:
        task_id = plan[step_index].task_id
        parents = {c.task_id: c.catalog.get("parent_task_id") for c in plan if c.task_id and c.catalog.get("parent_task_id")}
        def within_failed_task(candidate_id):
            visited = set()
            while candidate_id and candidate_id not in visited:
                if candidate_id == task_id:
                    return True
                visited.add(candidate_id)
                candidate_id = parents.get(candidate_id)
            return False
        for call in plan[step_index + 1 :]:
            if call.kind == "task_end" or (task_id and within_failed_task(call.task_id)):
                continue
            if call.name in {"await_robot_stationary", "await_robot_auto_mode", "check_localization_health"}:
                continue
            return call
        return None

    @staticmethod
    def _next_step_still_meaningful(intent: MissionIntent | None, call: SkillCall | None) -> bool:
        if call is None:
            return False
        if call.kind == "task":
            constraints = call.payload.get("constraints") or {}
            if constraints.get("depends_on") or constraints.get("run_if") or constraints.get("dependency_policy") == "required_condition":
                return False
            if constraints.get("dependency_policy") in {"independent_goals", "ordered_but_independent", "cleanup_after_attempt"}:
                return True
        if intent and intent.task_type == "sequence":
            return JevStrengthenAgent._dependency_policy_for_intent(intent) in {
                "independent_goals",
                "ordered_but_independent",
                "cleanup_after_attempt",
            }
        return JevStrengthenAgent._is_safe_independent_next_step(call)

    @staticmethod
    def _next_step_needs_human_after_failure(call: SkillCall | None, category: str) -> bool:
        if call is None:
            return False
        if category in {"safety_block", "localization_related", "local_planner_fault", "camera_or_perception_fault", "timeout_or_load"}:
            return True
        return call.name in {"move_relative", "move_relative_xy", "rotate_relative", "goto_pose", "touch", "grasp", "set_pose"}

    @staticmethod
    def _needs_recovery_agent(category: str, health: dict[str, Any] | None) -> bool:
        if category in {"target_not_found", "ambiguous_detection", "unsupported_query", "logic_or_parse_error", "unknown"}:
            return False
        if category in {"localization_related", "local_planner_fault", "relative_motion_fault", "camera_or_perception_fault", "arm_fault", "timeout_or_load"}:
            return True
        if category == "control_mode_block":
            return False
        if category == "localization_unrelated":
            return False
        components = (health or {}).get("components") or {}
        return any(isinstance(item, dict) and item.get("level") == "BAD" for item in components.values())

    @staticmethod
    def _recovery_target_for_category(category: str) -> str:
        return {
            "localization_related": "localization",
            "local_planner_fault": "local_planner",
            "relative_motion_fault": "drive",
            "camera_or_perception_fault": "camera",
            "arm_fault": "arm_gateway",
            "control_mode_block": "control_mode",
            "timeout_or_load": "system_load",
        }.get(category, "none")

    @staticmethod
    def _next_step_group(call: SkillCall | None) -> dict[str, Any] | None:
        if call is None:
            return None
        return {
            "skill": call.name,
            "args": call.payload,
            "kind": call.kind,
            "risk": "physical_motion" if call.name in {"navigate_to", "set_arm_pose", "interact_with_object", "manipulate_object", "goto_pose", "move_relative", "move_relative_xy", "rotate_relative", "set_pose", "touch", "grasp"} else "read_or_perception",
        }

    @staticmethod
    def _safety_summary(health: dict[str, Any] | None) -> dict[str, Any]:
        components = (health or {}).get("components") or {}
        safety = components.get("safety") if isinstance(components, dict) else {}
        return {
            "hard_safety_block": isinstance(safety, dict) and safety.get("level") == "BAD",
            "safety_status": (safety or {}).get("summary") if isinstance(safety, dict) else "UNKNOWN",
        }

    @staticmethod
    def _human_choices_for_next(next_call: SkillCall | None, user_text: str, category: str) -> list[dict[str, str]]:
        choices = []
        if next_call and next_call.name == "set_pose":
            pose = str(next_call.payload.get("name") or "home")
            choices.append({
                "label": f"Continue: set pose {pose}",
                "command": f"set pose {pose}",
            })
        elif next_call and next_call.name == "move_relative":
            direction = str(next_call.payload.get("direction") or "forward")
            distance = next_call.payload.get("distance_m")
            label = f"Continue: move {direction}"
            command = f"move {direction}"
            if isinstance(distance, (int, float)):
                label += f" {float(distance):.2f} m"
                command += f" {float(distance):.2f}m"
            choices.append({"label": label, "command": command})
        elif next_call and next_call.name == "rotate_relative":
            yaw = next_call.payload.get("yaw_deg")
            label = f"Continue: rotate {yaw} deg" if yaw is not None else "Continue: rotate"
            command = f"turn {yaw} degrees" if yaw is not None else "turn"
            choices.append({"label": label, "command": command})
        if category == "target_not_found":
            choices.append({"label": "Retry perception from another view", "command": user_text})
        choices.append({"label": "Retry original command", "command": user_text})
        choices.append({"label": "Cancel", "command": "cancel"})
        return choices

    @staticmethod
    def _message_for_action(action: str, result: dict[str, Any], next_call: SkillCall | None) -> str:
        if action == "ask_human_continue_retry_stop":
            options = f"Continue with {next_call.name}, retry, or stop?" if next_call else "Retry or cancel the mission; dependent tasks cannot run before this step succeeds."
            return f"{result.get('message') or 'Step failed'} {options}"
        return str(result.get("message") or action)

    @staticmethod
    def _resolve_call_log_path() -> str:
        explicit = os.environ.get("JEV_CALL_LOG_PATH", "").strip()
        if explicit:
            os.makedirs(os.path.dirname(os.path.abspath(explicit)), exist_ok=True)
            return os.path.abspath(explicit)
        if os.environ.get("ROBOT_AGENT_SESSION_ID"):
            from pathlib import Path
            from robot_agent.session import session_dir
            directory = session_dir(Path(os.environ.get("ROBOT_WS", Path.cwd()))) / "agent"
            directory.mkdir(parents=True, exist_ok=True)
            return str(directory / "jev_calls.jsonl")
        root = os.environ.get("JEV_LOG_ROOT", "debug-output/agent")
        os.makedirs(root, exist_ok=True)
        latest = None
        try:
            dirs = [
                os.path.join(root, item)
                for item in os.listdir(root)
                if os.path.isdir(os.path.join(root, item))
            ]
            dirs.sort(key=lambda path: os.path.getmtime(path), reverse=True)
            latest = dirs[0] if dirs else None
        except OSError:
            latest = None
        if latest is None:
            latest = os.path.join(root, datetime.now().strftime("%Y%m%d_%H%M%S"))
            os.makedirs(latest, exist_ok=True)
        return os.path.abspath(os.path.join(latest, "jev_calls.jsonl"))

    def _write_call_log(self, stage: str, request_body: dict[str, Any], response: dict[str, Any], latency_ms: float, *, purpose="decision") -> None:
        record = {
            "time": datetime.utcnow().isoformat() + "Z",
            "stage": stage,
            "purpose": purpose,
            "schema_version": request_body.get("state", {}).get("schema_version"),
            "questions_sha256": hashlib.sha256(json.dumps(request_body.get("questions", {}),
                sort_keys=True, ensure_ascii=False, separators=(",", ":")).encode("utf-8")).hexdigest(),
            "provider": "api_error" if response.get("_error") else "live" if self.api_configured else "local",
            "model": self.model if self.api_configured else "jev-local-v0",
            "request": self._redact_for_log(request_body),
            "response_raw": self._redact_for_log(response),
            "latency_ms": round(latency_ms, 3),
            "redaction": {
                "api_key_removed": True,
                "large_images_removed": True,
                "max_string_chars": 2000,
            },
        }
        try:
            os.makedirs(os.path.dirname(self.call_log_path), exist_ok=True)
            with open(self.call_log_path, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError:
            return

    @classmethod
    def _redact_for_log(cls, value: Any, *, depth: int = 0) -> Any:
        if depth > 8:
            return "<max_depth>"
        if isinstance(value, dict):
            out = {}
            for key, item in value.items():
                key_str = str(key)
                lowered = key_str.lower()
                if (any(token in lowered for token in ["api_key", "authorization", "secret"])
                        or ("token" in lowered and lowered not in {
                            "input_tokens", "output_tokens", "total_tokens", "cached_tokens",
                            "required_tokens", "matched_tokens", "token_count"})):
                    out[key_str] = "<redacted>"
                elif any(token in lowered for token in ["image", "jpeg", "png", "base64"]) and isinstance(item, str) and len(item) > 200:
                    out[key_str] = "<large_image_or_blob_redacted>"
                else:
                    out[key_str] = cls._redact_for_log(item, depth=depth + 1)
            return out
        if isinstance(value, list):
            if len(value) > 80:
                return [cls._redact_for_log(item, depth=depth + 1) for item in value[:80]] + ["<truncated_list>"]
            return [cls._redact_for_log(item, depth=depth + 1) for item in value]
        if isinstance(value, str):
            return value if len(value) <= 2000 else value[:2000] + "<truncated>"
        return value

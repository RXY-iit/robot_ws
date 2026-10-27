from __future__ import annotations

import re
from dataclasses import asdict, dataclass, field
from typing import Any

from robot_agent.schemas import MissionIntent
from robot_agent.parsers.deterministic_parser import DeterministicParser
from robot_skills.skill_catalog import SkillCatalog


@dataclass
class CommandCandidate:
    candidate_id: str
    task_type: str
    steps: list[dict[str, Any]]
    source: list[str] = field(default_factory=list)
    risk: str = "unknown"
    confidence_hint: float = 0.5
    notes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class SkillCandidateGenerator:
    """Conservative typed candidate generator for JEV command_parse.

    This is intentionally not a planner. It proposes schema-shaped candidates
    from deterministic parsing, safe typo repair, and sequence splitting; JEV
    can then choose among these candidates.
    """

    _SEQUENCE_SPLIT_RE = re.compile(
        r"\s*(?:,?\s*(?:then|and\s+then|after(?:\s+that)?|and)\s+|然后|然後|それから|そして)\s*",
        re.IGNORECASE,
    )

    def __init__(self, parser: DeterministicParser | None = None, catalog: SkillCatalog | None = None):
        self.parser = parser or DeterministicParser()
        self.catalog = catalog or SkillCatalog()

    def generate(self, text: str, *, local_intent: MissionIntent | None = None, max_candidates: int = 5) -> list[CommandCandidate]:
        raw = str(text or "").strip()
        if not raw:
            return [self._ask_human(raw, ["empty command"])]
        repaired, repair_notes = self._repair_typos(raw)
        candidates: list[CommandCandidate] = []

        intent = local_intent or self.parser.parse(raw)
        if intent.task_type != "ask_human":
            candidates.append(self._from_intent("deterministic_full", intent, ["deterministic"], 0.9, repair_notes))

        if repaired != raw:
            repaired_intent = self.parser.parse(repaired)
            if repaired_intent.task_type != "ask_human":
                candidates.append(
                    self._from_intent("typo_repaired_full", repaired_intent, ["typo_normalizer", "deterministic"], 0.82, repair_notes)
                )

        for base_text, source in [(raw, "sequence_splitter"), (repaired, "typo_normalizer_sequence")]:
            sequence_candidate = self._sequence_candidate(base_text, source, repair_notes)
            if sequence_candidate is not None:
                candidates.append(sequence_candidate)

        heuristic = self._heuristic_candidate(repaired, repair_notes)
        if heuristic is not None:
            candidates.append(heuristic)

        candidates.append(self._ask_human(raw, ["fallback candidate"]))
        return self._dedupe(candidates)[:max(1, max_candidates)]

    def _sequence_candidate(self, text: str, source: str, notes: list[str]) -> CommandCandidate | None:
        parts = [part.strip(" ,，.。") for part in self._SEQUENCE_SPLIT_RE.split(text) if part.strip(" ,，.。")]
        if len(parts) < 2:
            return None
        steps: list[dict[str, Any]] = []
        for part in parts:
            intent = self.parser.parse(part)
            if intent.task_type == "ask_human":
                fallback = self._heuristic_intent(part)
                if fallback is None:
                    return None
                intent = fallback
            if intent.task_type == "sequence":
                return None
            steps.append(self._intent_step(intent))
        risk = self._risk_for_steps(steps)
        return CommandCandidate(
            candidate_id=self._candidate_id_from_steps(steps),
            task_type="sequence",
            steps=steps,
            source=[source],
            risk=risk,
            confidence_hint=0.78,
            notes=notes,
        )

    def _heuristic_candidate(self, text: str, notes: list[str]) -> CommandCandidate | None:
        intent = self._heuristic_intent(text)
        if intent is None:
            return None
        return self._from_intent(f"heuristic_{intent.task_type}", intent, ["skill_catalog_alias", "heuristic"], 0.68, notes)

    @staticmethod
    def _heuristic_intent(text: str) -> MissionIntent | None:
        lowered = text.lower().strip()
        if re.search(r"\b(?:turn|rotate)\s+left\b", lowered):
            return MissionIntent(task_type="rotate_relative", constraints={"yaw_deg": 90}, raw_text=text)
        if re.search(r"\b(?:turn|rotate)\s+right\b", lowered):
            return MissionIntent(task_type="rotate_relative", constraints={"yaw_deg": -90}, raw_text=text)
        if re.search(r"\b(?:go|move|back up)\b.*\bback\b|\bback up\b", lowered):
            return MissionIntent(task_type="move_relative", constraints={"direction": "back", "distance_m": 0.2}, raw_text=text)
        if re.search(r"\b(?:go|move|前進)\b.*\b(?:forward|front|ahead)\b|前進", lowered):
            return MissionIntent(task_type="move_relative", constraints={"direction": "forward", "distance_m": 0.2}, raw_text=text)
        if re.search(r"\b(?:look|see)\s+left\b|左を見|左を見る", lowered):
            return MissionIntent(task_type="look_direction", constraints={"direction": "left", "band": "middle"}, raw_text=text)
        if re.search(r"\b(?:look|see)\s+right\b|右を見|右を見る", lowered):
            return MissionIntent(task_type="look_direction", constraints={"direction": "right", "band": "middle"}, raw_text=text)
        if re.search(r"\bset\s+pose\s+home\b|\bpose\s+home\b", lowered):
            return MissionIntent(task_type="set_pose", target="home", raw_text=text)
        return None

    def _from_intent(
        self,
        candidate_id: str,
        intent: MissionIntent,
        source: list[str],
        confidence_hint: float,
        notes: list[str],
    ) -> CommandCandidate:
        if intent.task_type == "sequence":
            steps = []
            for item in intent.constraints.get("steps", []):
                if isinstance(item, dict):
                    steps.append(self._intent_dict_step(item))
        else:
            steps = [self._intent_step(intent)]
        return CommandCandidate(
            candidate_id=candidate_id if intent.task_type != "sequence" else self._candidate_id_from_steps(steps),
            task_type=intent.task_type,
            steps=steps,
            source=source,
            risk=self._risk_for_steps(steps),
            confidence_hint=confidence_hint,
            notes=notes,
        )

    def _intent_step(self, intent: MissionIntent) -> dict[str, Any]:
        return self._intent_dict_step(intent.to_dict())

    def _intent_dict_step(self, data: dict[str, Any]) -> dict[str, Any]:
        task_type = str(data.get("task_type") or "ask_human")
        skills = self._robot_skills_for_task(task_type)
        return {
            "task_type": task_type,
            "target": data.get("target"),
            "object_query": data.get("object_query"),
            "constraints": data.get("constraints") if isinstance(data.get("constraints"), dict) else {},
            "raw_text": data.get("raw_text") or "",
            "robot_skills": [self.catalog.compact_skill(name) for name in skills],
        }

    @staticmethod
    def _robot_skills_for_task(task_type: str) -> list[str]:
        return {
            "cancel": ["cancel_motion"],
            "robot_status": ["get_robot_status"],
            "list_known_targets": ["list_known_targets"],
            "move_relative": ["await_robot_auto_mode", "move_relative"],
            "move_relative_xy": ["await_robot_auto_mode", "move_relative_xy"],
            "rotate_relative": ["await_robot_auto_mode", "rotate_relative"],
            "look_direction": ["await_robot_stationary", "set_camera_pose", "capture_observation", "set_camera_pose"],
            "search_object_here": ["await_robot_stationary", "set_camera_pose", "capture_observation", "detect_known_object"],
            "set_pose": ["await_robot_stationary", "set_pose"],
            "navigate_to_place": ["resolve_semantic_target", "await_robot_auto_mode", "check_localization_health", "goto_pose"],
            "navigate_to_object": ["resolve_semantic_target", "await_robot_auto_mode", "check_localization_health", "goto_pose"],
        }.get(task_type, [])

    @staticmethod
    def _repair_typos(text: str) -> tuple[str, list[str]]:
        repairs = {
            r"\blittlt\b": "little",
            r"\blittel\b": "little",
            r"\bforword\b": "forward",
            r"\bbackword\b": "backward",
        }
        out = text
        notes: list[str] = []
        for pattern, replacement in repairs.items():
            new = re.sub(pattern, replacement, out, flags=re.IGNORECASE)
            if new != out:
                notes.append(f"typo repair: {pattern} -> {replacement}")
                out = new
        return out, notes

    @staticmethod
    def _risk_for_steps(steps: list[dict[str, Any]]) -> str:
        skill_names = {
            skill.get("name")
            for step in steps
            for skill in step.get("robot_skills", [])
            if isinstance(skill, dict)
        }
        if skill_names & {"touch", "grasp"}:
            return "contact_arm_motion"
        if skill_names & {"set_pose", "move_lift_bounded"}:
            return "arm_or_lift_motion"
        if skill_names & {"goto_pose", "move_relative", "move_relative_xy", "rotate_relative"}:
            return "base_motion"
        if skill_names & {"set_camera_pose"}:
            return "camera_motion"
        return "read_or_perception"

    @staticmethod
    def _candidate_id_from_steps(steps: list[dict[str, Any]]) -> str:
        names = [str(step.get("task_type") or "step") for step in steps]
        return "_then_".join(names) if names else "empty_candidate"

    @staticmethod
    def _ask_human(text: str, notes: list[str]) -> CommandCandidate:
        return CommandCandidate(
            candidate_id="ask_human",
            task_type="ask_human",
            steps=[],
            source=["safe_fallback"],
            risk="none",
            confidence_hint=0.0,
            notes=[*notes, f"raw={text}"],
        )

    @staticmethod
    def _dedupe(candidates: list[CommandCandidate]) -> list[CommandCandidate]:
        seen: set[str] = set()
        out: list[CommandCandidate] = []
        for candidate in candidates:
            key = candidate.candidate_id
            if key in seen:
                continue
            seen.add(key)
            out.append(candidate)
        return out

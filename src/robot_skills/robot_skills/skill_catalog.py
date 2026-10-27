from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import yaml

from robot_skills.registry import SkillRegistry, package_config_path


@dataclass(frozen=True)
class SkillPreconditionReview:
    skill_name: str
    required: list[str]
    irrelevant: list[str]
    satisfied: list[str]
    missing: list[str]
    hard_blocks: list[str]
    warnings: list[str]
    ignored_bad_components: list[str]
    notes: list[str]

    @property
    def allowed(self) -> bool:
        return not self.hard_blocks

    def to_dict(self) -> dict[str, Any]:
        return {
            "skill_name": self.skill_name,
            "required": self.required,
            "irrelevant": self.irrelevant,
            "satisfied": self.satisfied,
            "missing": self.missing,
            "hard_blocks": self.hard_blocks,
            "warnings": self.warnings,
            "ignored_bad_components": self.ignored_bad_components,
            "notes": self.notes,
            "allowed": self.allowed,
        }


class SkillCatalog:
    def __init__(self, path: str | None = None):
        self.path = path or package_config_path("skill_catalog.v2.yaml")
        self.data = self._load(self.path)
        self.schema_version = str(self.data.get("schema_version") or "")
        self.precondition_atoms = self.data.get("precondition_atoms") or {}
        self.robot_skills = {
            str(item.get("name")): item
            for item in self.data.get("robot_skills", [])
            if isinstance(item, dict) and item.get("name")
        }
        if path is None:
            from robot_skills.extensions import extend_definitions
            extend_definitions(self.robot_skills, catalog=True,
                               supervision_profiles=self.data.get("supervision_profiles", {}))

    @staticmethod
    def _load(path: str) -> dict[str, Any]:
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def get(self, name: str) -> dict[str, Any] | None:
        return self.robot_skills.get(name)

    def list_names(self) -> list[str]:
        return sorted(self.robot_skills)

    def validate_registry(self, registry: SkillRegistry | None = None) -> list[str]:
        registry = registry or SkillRegistry()
        errors: list[str] = []
        registry_names = set(registry.list_names())
        catalog_names = set(self.list_names())
        for name in sorted(registry_names - catalog_names):
            errors.append(f"registry skill missing from catalog: {name}")
        for name in sorted(catalog_names - registry_names):
            errors.append(f"catalog skill missing from registry: {name}")
        for name in sorted(registry_names & catalog_names):
            item = self.robot_skills[name]
            if not isinstance(item.get("args_schema"), dict):
                errors.append(f"{name}: missing args_schema")
            if not isinstance(item.get("preconditions"), dict):
                errors.append(f"{name}: missing preconditions")
            if not isinstance(item.get("effects"), dict):
                errors.append(f"{name}: missing effects")
            supervision = item.get("supervision") or {}
            if not supervision.get("questions") or not supervision.get("evidence_fields"):
                errors.append(f"{name}: missing supervision questions/evidence_fields")
            for atom in (item.get("preconditions") or {}).get("required") or []:
                if atom not in self.precondition_atoms:
                    errors.append(f"{name}: unknown required precondition atom {atom}")
            for atom in (item.get("preconditions") or {}).get("irrelevant") or []:
                if atom not in self.precondition_atoms:
                    errors.append(f"{name}: unknown irrelevant precondition atom {atom}")
        return errors

    def compact_skill(self, name: str) -> dict[str, Any]:
        item = self.get(name) or {}
        preconditions = item.get("preconditions") if isinstance(item.get("preconditions"), dict) else {}
        dashboard = item.get("dashboard") if isinstance(item.get("dashboard"), dict) else {}
        jev = item.get("jev") if isinstance(item.get("jev"), dict) else {}
        return {
            "name": name,
            "group": item.get("group"),
            "intent_tags": item.get("intent_tags") or [],
            "required_preconditions": preconditions.get("required") or [],
            "irrelevant_preconditions": preconditions.get("irrelevant") or [],
            "risk_label": dashboard.get("risk_label"),
            "candidate_priority": jev.get("candidate_priority"),
            "supervision": item.get("supervision") or {},
        }

    def compact_skills_for_jev(self, *, command_parse_only: bool = False) -> list[dict[str, Any]]:
        out: list[dict[str, Any]] = []
        for name in self.list_names():
            item = self.get(name) or {}
            jev = item.get("jev") if isinstance(item.get("jev"), dict) else {}
            if command_parse_only and not jev.get("expose_to_command_parse", False):
                continue
            out.append(self.compact_skill(name))
        return out

    def review_skill_preconditions(self, skill_name: str, health: dict[str, Any] | None) -> SkillPreconditionReview:
        item = self.get(skill_name) or {}
        preconditions = item.get("preconditions") if isinstance(item.get("preconditions"), dict) else {}
        required = [str(atom) for atom in preconditions.get("required") or []]
        irrelevant = [str(atom) for atom in preconditions.get("irrelevant") or []]
        notes = [str(note) for note in preconditions.get("notes") or []]
        satisfied: list[str] = []
        missing: list[str] = []
        hard_blocks: list[str] = []
        warnings: list[str] = []
        ignored_bad_components: list[str] = []
        for atom in required:
            ok, reason, hard = self._evaluate_atom(atom, health)
            if ok:
                satisfied.append(atom)
            else:
                missing.append(atom)
                if hard:
                    hard_blocks.append(atom)
                else:
                    warnings.append(f"{atom}: {reason}")
        ignored_bad_components = self._ignored_bad_components(irrelevant, health)
        for component in ignored_bad_components:
            warnings.append(f"{component} is BAD but irrelevant for {skill_name}")
        return SkillPreconditionReview(
            skill_name=skill_name,
            required=required,
            irrelevant=irrelevant,
            satisfied=satisfied,
            missing=missing,
            hard_blocks=hard_blocks,
            warnings=warnings,
            ignored_bad_components=ignored_bad_components,
            notes=notes,
        )

    def review_plan_preconditions(self, plan: list[Any], health: dict[str, Any] | None) -> list[dict[str, Any]]:
        reviews = []
        for call in plan:
            name = str(getattr(call, "name", "") or (call.get("name") if isinstance(call, dict) else ""))
            if name:
                reviews.append(self.review_skill_preconditions(name, health).to_dict())
        return reviews

    def _evaluate_atom(self, atom: str, health: dict[str, Any] | None) -> tuple[bool, str, bool]:
        components = (health or {}).get("components") or {}
        if atom == "safety_ok_or_watchdog":
            safety = components.get("safety") or {}
            summary = str(safety.get("summary") or "").upper()
            level = str(safety.get("level") or "UNKNOWN").upper()
            ok = level in {"OK", "WARN"} and summary not in {"ESTOP", "MANUAL_STOP", "BAD"}
            return ok, summary or level, True
        if atom == "robot_mode_auto":
            mode = components.get("mode") or {}
            summary = str(mode.get("summary") or "").upper()
            return summary == "AUTO", summary or "UNKNOWN", False
        if atom == "global_localization_healthy":
            localization = components.get("localization") or {}
            return localization.get("level") == "OK", str(localization.get("summary") or "UNKNOWN"), False
        if atom == "wheel_odom_fresh":
            details = ((components.get("localization") or {}).get("details") or {})
            return bool(details.get("latest_wheel_odom_pose")), "latest_wheel_odom_pose missing", False
        if atom == "local_planner_active":
            planner = components.get("local_planner") or {}
            details = planner.get("details") or {}
            if not details.get("required", False):
                return True, "not required", False
            return planner.get("level") != "BAD", str(planner.get("summary") or "UNKNOWN"), False
        if atom in {"robot_stationary", "camera_ready", "camera_image_fresh", "arm_gateway_ready", "lift_gateway_ready"}:
            # These are enforced by concrete skills or external dashboards in the
            # current system. Treat as advisory here until richer health sources
            # are wired into the catalog.
            return True, "advisory_only", False
        return True, "unknown_atom_treated_advisory", False

    @staticmethod
    def _ignored_bad_components(irrelevant: list[str], health: dict[str, Any] | None) -> list[str]:
        components = (health or {}).get("components") or {}
        ignored: list[str] = []
        if "global_localization_healthy" in irrelevant:
            localization = components.get("localization") or {}
            if localization.get("level") == "BAD":
                ignored.append("localization")
        if "robot_mode_auto" in irrelevant:
            mode = components.get("mode") or {}
            if mode.get("level") in {"WARN", "BAD"}:
                ignored.append("mode")
        return ignored

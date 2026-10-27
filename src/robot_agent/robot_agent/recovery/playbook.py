from __future__ import annotations

import copy
from dataclasses import dataclass, field, replace
from pathlib import Path
from string import Template
from typing import Any

import yaml

from .environment import EnvProfile, risk_rank
from .primitives import PrimitiveRegistry
from .snapshot import ABNORMAL_CLASSES, classify_status


ENV_STATUSES = {"supported", "injection_only", "n/a"}
# Severity is what the fault means for operation; priority (P0-P3) is only
# the order of handling. blocking: robot cannot do its job; degraded: works
# with reduced capability/confidence; advisory: deviation to know about.
SEVERITIES = ("blocking", "degraded", "advisory")
HANDLING_CLASSES = {"known_recovery", "hardware_intervention", "safety_event", "unknown_investigation"}
STEP_TYPES = {"action", "operator_request", "wait_evidence"}
MISMATCH_ACTIONS = {"handoff", "escalate", "reject"}
# Per-env sections may replace these top-level blocks.
OVERRIDABLE = ("trigger", "mission_trigger", "diagnosis", "supporting_evidence", "preconditions", "recovery", "verify", "handling_class")
# A trigger item of "*" matches any root item (generic playbooks such as
# PB-NODE-DOWN). Probe args may reference ${root_item} and ${producer}.
WILDCARD = "*"


class PlaybookError(ValueError):
    pass


@dataclass
class Playbook:
    """Effective (env-resolved) playbook."""

    id: str
    number: int
    title: str
    version: int
    domain: str
    layer: str
    affects_capabilities: list[str]
    env_status: str
    verified: bool
    note: str
    trigger: list[dict[str, Any]]
    mission_trigger: dict[str, Any] | None
    diagnosis: list[dict[str, Any]]
    preconditions: list[str]
    recovery: list[dict[str, Any]]
    verify: list[dict[str, Any]]
    policy: dict[str, Any]
    fault_injection: dict[str, Any] | None
    operator_text: dict[str, str]
    source: str = ""
    raw: dict[str, Any] = field(default_factory=dict, repr=False)
    supporting_evidence: list[dict[str, Any]] = field(default_factory=list)
    handling_class: str = "known_recovery"

    def bind_components(self, env: EnvProfile, variables: dict[str, str]) -> Playbook:
        steps = copy.deepcopy(self.recovery)
        preconditions = list(self.preconditions)
        verify = copy.deepcopy(self.verify)
        for step in steps:
            if step.get("primitive") != "restart_component":
                continue
            component = Template(str((step.get("args") or {}).get("component", ""))).safe_substitute(variables)
            spec = env.components.get(component)
            if spec is not None:
                step["risk"] = max(step["risk"], spec.restart_risk, key=risk_rank)
                preconditions.extend(p for p in spec.restart_preconditions if p not in preconditions)
                for item in (spec.restart or {}).get("verify_items", []):
                    if not any(v.get("probe") == "health_item" and v.get("args", {}).get("item") == item for v in verify):
                        verify.append({"probe": "health_item", "args": {"item": item},
                                       "within_sec": (spec.restart or {}).get("verify_timeout_sec", 30)})
        return replace(self, recovery=steps, preconditions=preconditions, verify=verify)

    @property
    def severity(self) -> str:
        value = self.raw.get("severity")
        if value:
            return str(value)
        return "blocking" if self.layer == "safety" else "degraded"

    @property
    def max_action_risk(self) -> str:
        """Highest risk among software actions (operator steps excluded): actuator lock."""
        risks = [str(step.get("risk", "A0")) for step in self.recovery if step.get("type") == "action"]
        return max(risks, key=risk_rank) if risks else "A0"

    @property
    def max_risk(self) -> str:
        risks = [str(step.get("risk", "A0")) for step in self.recovery]
        return max(risks, key=risk_rank) if risks else "A0"


    def summary(self) -> dict[str, Any]:
        return {
            "number": self.number,
            "id": self.id,
            "title": self.title,
            "domain": self.domain,
            "layer": self.layer,
            "env_status": self.env_status,
            "verified": self.verified,
            "max_risk": self.max_risk,
            "severity": self.severity,
            "source": self.source,
            "handling_class": self.handling_class,
        }


def trigger_matches(condition: dict[str, Any], status: str | None, abnormal_age: float | None) -> bool:
    if status is None:
        return False
    if "status_in" in condition:
        if status not in condition["status_in"]:
            return False
    elif "class_in" in condition:
        if classify_status(status) not in condition["class_in"]:
            return False
    elif classify_status(status) not in ABNORMAL_CLASSES:
        return False
    for_sec = float(condition.get("for_sec", 0.0))
    return for_sec <= 0.0 or (abnormal_age is not None and abnormal_age >= for_sec)


def _resolve(raw: dict[str, Any], env_kind: str) -> dict[str, Any]:
    envs = raw.get("envs") or {}
    env_spec = envs.get(env_kind) or {"status": "n/a", "note": f"No {env_kind} section"}
    resolved = copy.deepcopy(raw)
    for key in OVERRIDABLE:
        if key in env_spec:
            resolved[key] = copy.deepcopy(env_spec[key])
    resolved["_env"] = env_spec
    return resolved


def _check_probe_ref(pb_id: str, where: str, spec: dict[str, Any], registry: PrimitiveRegistry) -> None:
    name = spec.get("probe")
    prim = registry.get(str(name)) if name else None
    if prim is None or prim.kind != "probe":
        raise PlaybookError(f"{pb_id} {where}: unknown probe {name!r}")


def build_playbook(raw: dict[str, Any], env: EnvProfile, registry: PrimitiveRegistry, source: str = "") -> Playbook:
    pb_id = str(raw.get("id") or "")
    if not pb_id.startswith("PB-"):
        raise PlaybookError(f"{source}: playbook id must start with 'PB-' (got {pb_id!r})")
    try:
        # Not "no:" -- YAML 1.1 reads that key as boolean False.
        number = int(raw["number"])
    except (KeyError, TypeError, ValueError) as exc:
        raise PlaybookError(f"{pb_id}: integer 'number' (playbook number) is required") from exc
    if raw.get("severity") is not None and raw["severity"] not in SEVERITIES:
        raise PlaybookError(f"{pb_id}: severity must be one of {SEVERITIES}")
    data = _resolve(raw, env.kind)
    if data.get("handling_class", "known_recovery") not in HANDLING_CLASSES:
        raise PlaybookError(f"{pb_id}: invalid handling_class")
    env_spec = data["_env"]
    env_status = str(env_spec.get("status", "n/a"))
    if env_status not in ENV_STATUSES:
        raise PlaybookError(f"{pb_id}: envs.{env.kind}.status must be one of {sorted(ENV_STATUSES)}")

    trigger = list((data.get("trigger") or {}).get("any", []) or [])
    mission_trigger = data.get("mission_trigger")
    if env_status != "n/a" and not trigger and not mission_trigger:
        raise PlaybookError(f"{pb_id}: needs trigger.any or mission_trigger for env {env.kind}")
    for cond in trigger:
        if "item" not in cond:
            raise PlaybookError(f"{pb_id}: trigger condition without item: {cond}")
        if cond["item"] != WILDCARD and env_status != "n/a" and cond["item"] not in env.items:
            raise PlaybookError(f"{pb_id}: trigger item {cond['item']!r} is not in envs/{env.kind}.yaml")
    if mission_trigger is not None:
        if not isinstance(mission_trigger, dict):
            raise PlaybookError(f"{pb_id}: mission_trigger must be a mapping")
        if "when" in mission_trigger:
            _check_probe_ref(pb_id, "mission_trigger.when", mission_trigger["when"], registry)

    diagnosis = list(data.get("diagnosis") or [])
    supporting_evidence = list(data.get("supporting_evidence") or [])
    for spec in supporting_evidence:
        _check_probe_ref(pb_id, "supporting_evidence", spec, registry)
    seen_steps: set[str] = set()
    for step in diagnosis:
        step_id = str(step.get("id") or "")
        if not step_id or step_id in seen_steps:
            raise PlaybookError(f"{pb_id}: diagnosis step ids must be unique and non-empty")
        seen_steps.add(step_id)
        _check_probe_ref(pb_id, f"diagnosis.{step_id}", step, registry)
        if str(step.get("expect", "pass")) not in {"pass", "fail"}:
            raise PlaybookError(f"{pb_id} diagnosis.{step_id}: expect must be pass|fail")
        mismatch = step.get("on_mismatch") or {"escalate": "R3"}
        if not isinstance(mismatch, dict) or len(mismatch) != 1 or next(iter(mismatch)) not in MISMATCH_ACTIONS:
            raise PlaybookError(f"{pb_id} diagnosis.{step_id}: on_mismatch must be one of {sorted(MISMATCH_ACTIONS)}")

    preconditions = [str(name) for name in data.get("preconditions", []) or []]
    if env_status != "n/a":
        for name in preconditions:
            if name not in env.preconditions:
                raise PlaybookError(f"{pb_id}: precondition {name!r} has no binding in envs/{env.kind}.yaml")

    recovery = []
    for index, step in enumerate(data.get("recovery") or []):
        step = dict(step)
        step_type = str(step.get("type", ""))
        if step_type not in STEP_TYPES:
            raise PlaybookError(f"{pb_id} recovery[{index}]: type must be one of {sorted(STEP_TYPES)}")
        if step.get("on_fail", "abort") not in {"abort", "next"}:
            raise PlaybookError(f"{pb_id} recovery[{index}]: on_fail must be abort|next")
        if step.get("on_success", "next") not in {"next", "verify"}:
            raise PlaybookError(f"{pb_id} recovery[{index}]: on_success must be next|verify")
        for spec in step.get("success_when", []) or []:
            _check_probe_ref(pb_id, f"recovery[{index}].success_when", spec, registry)
        if step_type == "action":
            prim = registry.get(str(step.get("primitive")))
            if prim is None or prim.kind != "action":
                raise PlaybookError(f"{pb_id} recovery[{index}]: unknown action {step.get('primitive')!r}")
            if env_status != "n/a" and env.kind not in prim.envs and env_status != "injection_only":
                raise PlaybookError(f"{pb_id} recovery[{index}]: action {prim.name} unsupported in {env.kind}")
            # A step may raise but never lower the primitive's declared risk.
            declared = str(step.get("risk", prim.risk)).upper()
            step["risk"] = max(declared, prim.risk, key=risk_rank)
        elif step_type == "operator_request":
            step["risk"] = "A4"
            for spec in step.get("done_when", []) or []:
                _check_probe_ref(pb_id, f"recovery[{index}].done_when", spec, registry)
        else:
            step["risk"] = "A0"
            for spec in step.get("until", []) or []:
                _check_probe_ref(pb_id, f"recovery[{index}].until", spec, registry)
        recovery.append(step)

    verify = list(data.get("verify") or [])
    for spec in verify:
        _check_probe_ref(pb_id, "verify", spec, registry)

    return Playbook(
        id=pb_id,
        handling_class=str(data.get("handling_class", "known_recovery")),
        number=number,
        title=str(data.get("title", pb_id)),
        version=int(data.get("version", 1)),
        domain=str(data.get("domain", "")),
        layer=str(data.get("layer", "")),
        affects_capabilities=[str(cap) for cap in data.get("affects_capabilities", []) or []],
        env_status=env_status,
        verified=bool(env_spec.get("verified", False)),
        note=str(env_spec.get("note", "")),
        trigger=trigger,
        mission_trigger=dict(mission_trigger) if mission_trigger else None,
        diagnosis=diagnosis,
        preconditions=preconditions,
        recovery=recovery,
        verify=verify,
        policy=dict(data.get("policy") or {}),
        fault_injection=(data.get("fault_injection") or {}).get(env.kind),
        operator_text={str(k): str(v) for k, v in (data.get("operator_text") or {}).items()},
        source=source,
        raw=raw,
        supporting_evidence=supporting_evidence,
    )


@dataclass
class PlaybookLibrary:
    env_kind: str
    playbooks: dict[str, Playbook]
    # n/a playbooks are kept for listing/coverage but never matched.
    inactive: dict[str, Playbook]

    def get(self, pb_id: str) -> Playbook | None:
        return self.playbooks.get(pb_id) or self.inactive.get(pb_id)

    def active(self) -> list[Playbook]:
        return list(self.playbooks.values())

    def catalog(self) -> list[dict[str, Any]]:
        return sorted(
            (pb.summary() for pb in [*self.playbooks.values(), *self.inactive.values()]),
            key=lambda entry: entry["number"],
        )


def load_playbooks(playbook_dir: Path, env: EnvProfile, registry: PrimitiveRegistry) -> PlaybookLibrary:
    active: dict[str, Playbook] = {}
    inactive: dict[str, Playbook] = {}
    for path in sorted(playbook_dir.glob("*.yaml")):
        with path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}
        pb = build_playbook(raw, env, registry, source=path.name)
        if pb.id in active or pb.id in inactive:
            raise PlaybookError(f"Duplicate playbook id {pb.id} in {path.name}")
        (inactive if pb.env_status == "n/a" else active)[pb.id] = pb
    numbers: dict[int, str] = {}
    for pb in [*active.values(), *inactive.values()]:
        if pb.number in numbers:
            raise PlaybookError(f"Playbook number {pb.number} used by both {numbers[pb.number]} and {pb.id}")
        numbers[pb.number] = pb.id
    known = set(active) | set(inactive)
    for unit in env.startup_units:
        if any(pb_id not in known for pb_id in unit.get("playbooks", [])):
            raise PlaybookError(f"Startup unit {unit['id']} references unknown playbook")
    for pb in active.values():
        for step in pb.diagnosis:
            target = (step.get("on_mismatch") or {}).get("handoff")
            if target is not None and target not in known:
                raise PlaybookError(f"{pb.id} diagnosis.{step['id']}: handoff to unknown playbook {target}")
    return PlaybookLibrary(env.kind, active, inactive)

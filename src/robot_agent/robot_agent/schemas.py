from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any
from uuid import uuid4


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass
class MissionIntent:
    task_type: str
    target: str | None = None
    relation: str | None = None
    object_query: str | None = None
    constraints: dict[str, Any] = field(default_factory=dict)
    requires_confirmation: bool = False
    raw_text: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class SkillCall:
    name: str
    payload: dict[str, Any] = field(default_factory=dict)
    reason: str = ""
    timeout_sec: float = 120.0
    requires_human_confirm: bool = False
    catalog: dict[str, Any] = field(default_factory=dict)
    kind: str = "skill"
    task_id: str | None = None
    owner: str | None = None
    step_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class HealthComponentState:
    level: str = "UNKNOWN"
    summary: str = "UNKNOWN"
    details: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class HealthState:
    overall: str = "UNKNOWN"
    components: dict[str, HealthComponentState] = field(default_factory=dict)
    recommended_action: str = "check_system"
    active_recovery: str | None = None
    timestamp: str = field(default_factory=utc_now)

    @classmethod
    def from_summary(cls, data: dict[str, Any]) -> "HealthState":
        components = {}
        for name, item in (data.get("components") or {}).items():
            if not isinstance(item, dict):
                continue
            components[str(name)] = HealthComponentState(
                level=str(item.get("level") or "UNKNOWN"),
                summary=str(item.get("summary") or "UNKNOWN"),
                details=item.get("details") if isinstance(item.get("details"), dict) else {},
            )
        return cls(
            overall=str(data.get("overall") or "UNKNOWN"),
            components=components,
            recommended_action=str(data.get("recommended_action") or "check_system"),
            active_recovery=data.get("active_recovery"),
            timestamp=str(data.get("timestamp") or utc_now()),
        )

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["components"] = {key: value.to_dict() for key, value in self.components.items()}
        return data


@dataclass
class MissionState:
    user_text: str
    mode: str = "sim"
    mission_id: str = field(default_factory=lambda: uuid4().hex[:12])
    status: str = "RECEIVED"
    outcome: str | None = None
    intent: MissionIntent | None = None
    parsed_intent: dict[str, Any] | None = None
    initial_plan: list[dict[str, Any]] = field(default_factory=list)
    parser_trace: dict[str, Any] = field(default_factory=dict)
    agent_mode: str = "classic"
    jev_trace: dict[str, Any] = field(default_factory=dict)
    plan: list[SkillCall] = field(default_factory=list)
    tasks: list[dict[str, Any]] = field(default_factory=list)
    current_step_index: int = 0
    skill_results: list[dict[str, Any]] = field(default_factory=list)
    observations: list[dict[str, Any]] = field(default_factory=list)
    question_for_human: str | None = None
    human_wait: dict[str, Any] | None = None
    recovery_pause: dict[str, Any] | None = None
    final_message: str | None = None
    created_at: str = field(default_factory=utc_now)
    updated_at: str = field(default_factory=utc_now)

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        if self.intent is not None:
            data["intent"] = self.intent.to_dict()
        data["plan"] = [item.to_dict() for item in self.plan]
        return data

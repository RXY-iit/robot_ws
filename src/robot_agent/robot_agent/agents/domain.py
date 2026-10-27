"""Small, transport-independent contracts between Mission and domain agents."""
from dataclasses import dataclass, field

from robot_agent.schemas import SkillCall


@dataclass
class TaskRefinement:
    plan: list[SkillCall] = field(default_factory=list)
    traces: list = field(default_factory=list)
    question: str | None = None
    outcome: str = "READY"
    choices: list[dict] = field(default_factory=list)


def review(jev, mode, stage, intent, choices, default, evidence):
    if mode == "classic":
        return default, []
    chosen, trace = jev.domain_review(stage, intent, choices, default, evidence)
    # Shadow produces evidence only; it never changes plans or asks another question.
    return (default if mode == "jev_shadow" else chosen), [trace]

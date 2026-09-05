from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float
    frame_id: str = "map"

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Pose2D":
        return cls(
            x=float(data["x"]),
            y=float(data["y"]),
            yaw=float(data.get("yaw", 0.0)),
            frame_id=str(data.get("frame_id", "map")),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class SkillContext:
    run_id: str
    mode: str = "sim"
    robot_mode: str = "UNKNOWN"
    require_human_confirm: bool = False
    max_duration_sec: float = 120.0


@dataclass
class SkillResult:
    skill_name: str
    success: bool
    code: str
    message: str
    started_at: str = field(default_factory=utc_now)
    ended_at: str = field(default_factory=utc_now)
    metrics: dict[str, Any] = field(default_factory=dict)
    observations: list[dict[str, Any]] = field(default_factory=list)
    artifacts: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class SemanticTarget:
    object_id: str
    canonical_name: str
    category: str
    pose: Pose2D
    confidence: float
    approach_pose_id: str | None = None
    approach_source: str = "configured_pose"
    needs_clarification: bool = False
    alternatives: list[str] = field(default_factory=list)
    region: dict[str, Any] = field(default_factory=dict)
    approach_policy: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["pose"] = self.pose.to_dict()
        return data

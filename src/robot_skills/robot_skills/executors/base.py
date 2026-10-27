from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from robot_skills import result_codes as codes
from robot_skills.schemas import SkillResult


def now() -> str:
    return datetime.now(timezone.utc).isoformat()


def make_result(
    skill_name: str,
    success: bool,
    code: str,
    message: str,
    started_at: str | None = None,
    metrics: dict[str, Any] | None = None,
    observations: list[dict[str, Any]] | None = None,
    artifacts: list[str] | None = None,
) -> SkillResult:
    if skill_name == "goto_pose":
        metrics = dict(metrics or {})
        status = metrics.get("nav2_status")
        degraded = any(v.get("attempted") for v in metrics.get("staged_recovery", {}).values() if isinstance(v, dict))
        degraded = degraded or any(word in str(status) for word in ("relaxed", "micro_adjust", "alternate"))
        degraded = degraded or any(metrics.get(k) for k in ("alternate_goal_result", "obstacle_escape_final_retry_result", "near_goal_recovery"))
        grade = "not_successful"
        if success:
            grade = "degraded" if degraded else ("nav2_strict" if status == 4 else "task_tolerance")
        if code == codes.DRY_RUN_OK:
            grade = "dry_run"
        metrics["completion_class"] = grade
    return SkillResult(
        skill_name=skill_name,
        success=success,
        code=code,
        message=message,
        started_at=started_at or now(),
        ended_at=now(),
        metrics=metrics or {},
        observations=observations or [],
        artifacts=artifacts or [],
    )


def dry_run_result(skill_name: str, message: str, metrics: dict[str, Any] | None = None) -> SkillResult:
    return make_result(skill_name, True, codes.DRY_RUN_OK, message, metrics=metrics)

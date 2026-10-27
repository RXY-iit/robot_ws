"""Environment availability is not a robot fault or proof of recovery."""
from __future__ import annotations

import math
from typing import Any

from .snapshot import HealthSnapshot


def simulation_environment(snapshot: HealthSnapshot, evidence: dict[str, Any], now: float) -> dict[str, Any]:
    clock = (snapshot.item("sim_rtf") or {}).get("value") or {}
    if not isinstance(clock, dict):
        clock = {}
    age = now - snapshot.received_at

    def fresh(value: Any, limit: float = 2.0) -> bool:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and 0 <= value <= limit

    clock_fresh = fresh(age, 5.0) and fresh(clock.get("clock_receive_age_sec"))
    advancing = (clock_fresh and fresh(clock.get("not_advancing_sec"))
                 and isinstance(clock.get("rtf"), (int, float)) and not isinstance(clock["rtf"], bool)
                 and math.isfinite(clock["rtf"]) and clock["rtf"] > 0)
    timeline = evidence.get("timeline")
    process = evidence.get("process_alive")
    timeline_fresh = process is True and fresh(evidence.get("timeline_age_sec"), 3.0)
    if not fresh(age, 5.0):
        state, summary = "UNKNOWN", "Health snapshot is stale; recovery is suspended."
    elif evidence.get("timeline_conflict"):
        state, summary = "UNKNOWN", "Conflicting Isaac timelines in this ROS domain; check duplicate simulator instances."
    elif timeline_fresh and timeline in {"PAUSED", "STOPPED"}:
        state, summary = timeline, f"Isaac timeline is {timeline.lower()}; downstream recovery is suspended."
    elif advancing:
        state, summary = "RUNNING", "Fresh advancing simulation clock. ROS components are diagnosed independently."
    elif process is False:
        state, summary = "OFFLINE", "No local Isaac process in this ROS domain; simulation is unavailable, not a downstream PB fault."
    elif timeline_fresh and timeline == "PLAYING":
        state, summary = "DELIVERY_UNKNOWN", "Isaac reports Play, but fresh advancing ROS clock evidence is missing. Check clock/bridge delivery."
    else:
        state, summary = "UNKNOWN", "Cannot distinguish pause, bridge failure or an unresponsive simulator. No blind restart."
    return {"state": state, "summary": summary, "recovery_allowed": state == "RUNNING",
            "evidence": {**evidence, "health_age_sec": age, "clock": clock}, "suppressed_items": []}

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any


# Item statuses produced by health_monitor, grouped by what they prove.
FAULT_STATES = {"BLOCKED", "ERROR", "STALE"}
DEGRADED_STATES = {"DEGRADED", "WARN"}
UNCERTAIN_STATES = {"UNKNOWN", "NOT_OBSERVED"}
# Not evidence either way: the source is absent by design or idle.
NEUTRAL_STATES = {"NOT_INSTRUMENTED", "IDLE"}

FAULT = "FAULT"
DEGRADED = "DEGRADED"
UNCERTAIN = "UNCERTAIN"
NEUTRAL = "NEUTRAL"
HEALTHY = "HEALTHY"

ABNORMAL_CLASSES = {FAULT, DEGRADED, UNCERTAIN}


def classify_status(status: Any) -> str:
    value = str(status or "UNKNOWN").upper()
    if value in FAULT_STATES:
        return FAULT
    if value in DEGRADED_STATES:
        return DEGRADED
    if value in UNCERTAIN_STATES:
        return UNCERTAIN
    if value in NEUTRAL_STATES:
        return NEUTRAL
    if value in {"OK", "OBSERVED", "AVAILABLE", "ESTIMATED", "REPORTED", "RECOVERED"}:
        return HEALTHY
    return UNCERTAIN


@dataclass
class HealthSnapshot:
    """Flattened view of one /agent/health_dashboard_state message."""

    items: dict[str, dict[str, Any]]
    capabilities: dict[str, dict[str, Any]]
    sequence: int | None = None
    generated_at: str | None = None
    received_at: float = 0.0

    @classmethod
    def from_state(cls, state: dict[str, Any], received_at: float) -> "HealthSnapshot":
        items: dict[str, dict[str, Any]] = {}
        for domain in state.get("domains", []) or []:
            domain_id = str(domain.get("id", ""))
            for item in domain.get("items", []) or []:
                item_id = str(item.get("id", ""))
                if not item_id:
                    continue
                # The monitor intentionally lists some items in several domains
                # (e.g. drive_feedback in H1 and H3); keep one copy.
                entry = items.setdefault(item_id, {**item, "domains": []})
                entry["domains"].append(domain_id)
        capabilities = {
            str(cap.get("id")): cap for cap in state.get("capabilities", []) or [] if cap.get("id")
        }
        return cls(
            items=items,
            capabilities=capabilities,
            sequence=state.get("sequence"),
            generated_at=state.get("generated_at"),
            received_at=received_at,
        )

    def item(self, item_id: str) -> dict[str, Any] | None:
        return self.items.get(item_id)

    def status(self, item_id: str) -> str | None:
        item = self.items.get(item_id)
        return None if item is None else str(item.get("status", "UNKNOWN"))

    def health_class(self, item_id: str) -> str | None:
        status = self.status(item_id)
        return None if status is None else classify_status(status)

    def capabilities_for_item(self, item_id: str) -> list[str]:
        return [
            cap_id
            for cap_id, cap in self.capabilities.items()
            if item_id in (cap.get("dependencies") or [])
        ]


@dataclass
class StatusHistory:
    """Tracks how long each item has been in its current health class.

    Used for debouncing (`for_sec` in playbook triggers) and as compact
    "state change" history for incident context instead of full snapshots.
    """

    max_events: int = 200
    since: dict[str, tuple[str, float]] = field(default_factory=dict)
    # Debounce must not restart when a fault only changes flavour
    # (e.g. STALE -> ERROR), so abnormality is tracked separately.
    abnormal_since: dict[str, float] = field(default_factory=dict)
    events: deque[dict[str, Any]] = field(default_factory=deque)

    def update(self, snapshot: HealthSnapshot) -> list[dict[str, Any]]:
        changes = []
        now = snapshot.received_at
        for item_id, item in snapshot.items.items():
            status = str(item.get("status", "UNKNOWN"))
            if classify_status(status) in ABNORMAL_CLASSES:
                self.abnormal_since.setdefault(item_id, now)
            else:
                self.abnormal_since.pop(item_id, None)
            previous = self.since.get(item_id)
            if previous is None or previous[0] != status:
                self.since[item_id] = (status, now)
                if previous is not None:
                    change = {
                        "t": now,
                        "item": item_id,
                        "from": previous[0],
                        "to": status,
                        "summary": item.get("summary"),
                    }
                    changes.append(change)
                    self.events.append(change)
        while len(self.events) > self.max_events:
            self.events.popleft()
        return changes

    def status_age(self, item_id: str, now: float) -> float:
        entry = self.since.get(item_id)
        return 0.0 if entry is None else max(0.0, now - entry[1])

    def abnormal_age(self, item_id: str, now: float) -> float | None:
        started = self.abnormal_since.get(item_id)
        return None if started is None else max(0.0, now - started)

    def recent(self, window_sec: float, now: float) -> list[dict[str, Any]]:
        return [event for event in self.events if now - event["t"] <= window_sec]

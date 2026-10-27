from __future__ import annotations

import itertools
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

import json
from pathlib import Path

from robot_agent.health_response import robot_summary
from .environment import EnvProfile, LaunchProfile
from .executor import (
    ABORTED,
    ESCALATED,
    FAILED,
    SUCCEEDED,
    SUPERSEDED_PREFIX,
    ActionBackend,
    RecoveryExecutor,
    substitute,
)
from .playbook import WILDCARD, Playbook, PlaybookLibrary, trigger_matches
from .primitives import FAIL, PASS, UNKNOWN, GraphAdapter, PrimitiveRegistry, ProbeContext, SystemAdapter
from .snapshot import ABNORMAL_CLASSES, UNCERTAIN, HealthSnapshot, StatusHistory, classify_status
from .runtime_environment import simulation_environment


# Stage 1 priorities (design.md §4.2). Lower is handled first; nothing is dropped.
P_SAFETY = 0
P_MISSION = 1
P_CAPABILITY = 2
P_OTHER = 3

CONFIRMED = "CONFIRMED"
NEEDS_INVESTIGATION = "NEEDS_INVESTIGATION"
WARNING = "WARNING"
PENDING = "PENDING"  # abnormal, but not yet past debounce

MAX_HANDOFF_DEPTH = 4
# Mission-triggered incidents without a re-checkable `when` probe expire.
MISSION_INCIDENT_TTL_SEC = 120.0
MISSION_KEY_PREFIX = "mission:"
# A key reopened within this window after closing counts as a reopen; at
# FLAPPING_REOPENS the incident is shown as flapping (chronic, intermittent).
REOPEN_WINDOW_SEC = 300.0
FLAPPING_REOPENS = 3


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass
class RecoveryRequest:
    request_id: str
    caller: str = "mission_agent"
    mission_id: str | None = None
    trigger: str = ""
    required_capabilities: list[str] = field(default_factory=list)
    suspected_components: list[str] = field(default_factory=list)
    skill: str | None = None
    result_code: str | None = None
    raw: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RecoveryRequest":
        return cls(
            request_id=str(data.get("request_id") or f"rr-{int(datetime.now().timestamp() * 1000)}"),
            caller=str(data.get("caller", "mission_agent")),
            mission_id=data.get("mission_id"),
            trigger=str(data.get("trigger", "")),
            required_capabilities=[str(c) for c in data.get("required_capabilities", []) or []],
            suspected_components=[str(c) for c in data.get("suspected_components", []) or []],
            skill=(data.get("step") or {}).get("skill"),
            result_code=(data.get("result") or {}).get("code"),
            raw=data,
        )


@dataclass
class RootCandidate:
    key: str
    root_items: list[str]
    consequences: list[str]
    layer: str
    layer_index: int
    priority: int
    capabilities: list[str]
    max_abnormal_age: float | None
    upstream_checks: list[dict[str, Any]]


@dataclass
class Diagnosis:
    candidate: RootCandidate
    outcome: str
    playbook_id: str | None = None
    steps: list[dict[str, Any]] = field(default_factory=list)
    reason: str = ""
    uncertain: bool = False
    plan: dict[str, Any] | None = None
    variables: dict[str, str] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        cand = self.candidate
        return {
            "key": cand.key,
            "outcome": self.outcome,
            "playbook_id": self.playbook_id,
            "reason": self.reason,
            "uncertain": self.uncertain,
            "priority": cand.priority,
            "layer": cand.layer,
            "root_items": cand.root_items,
            "consequences": cand.consequences,
            "capabilities": cand.capabilities,
            "abnormal_for_sec": cand.max_abnormal_age,
            "diagnosis_path": {"upstream_checks": cand.upstream_checks, "playbook_steps": self.steps},
            "plan": self.plan,
        }


@dataclass
class Incident:
    id: str
    key: str
    opened_at: float
    opened_at_utc: str
    updated_at_utc: str
    status: str
    diagnosis: Diagnosis
    triggers: set[str] = field(default_factory=set)
    requests: list[str] = field(default_factory=list)
    resolved_at_utc: str | None = None
    # Hysteresis: time the root was first seen healthy again (None = still faulty).
    healthy_since: float | None = None
    reopen_count: int = 0
    acknowledged: dict[str, Any] | None = None
    execution_blocked: str | None = None

    def to_dict(self, snapshot: HealthSnapshot | None) -> dict[str, Any]:
        cand = self.diagnosis.candidate
        root_summaries = []
        if snapshot is not None:
            for item_id in cand.root_items:
                entry = snapshot.item(item_id) or {}
                root_summaries.append(
                    {"item": item_id, "status": entry.get("status"), "summary": entry.get("summary")}
                )
        return {
            "id": self.id,
            "status": self.status,
            "opened_at": self.opened_at_utc,
            "updated_at": self.updated_at_utc,
            "resolved_at": self.resolved_at_utc,
            "triggers": sorted(self.triggers),
            "requests": self.requests[-5:],
            "reopen_count": self.reopen_count,
            "flapping": self.reopen_count >= FLAPPING_REOPENS,
            "recovering": self.healthy_since is not None,
            "acknowledged": self.acknowledged,
            "execution_blocked": self.execution_blocked,
            "roots": root_summaries,
            **self.diagnosis.to_dict(),
        }


class RecoveryEngine:
    """Detect -> Triage -> Diagnose -> Act -> Verify loop.

    Diagnosis is always on. Execution (Stage 4) runs only with an action
    backend and `execution_enabled`; otherwise plans are dry runs.
    """

    def __init__(
        self,
        env: EnvProfile,
        library: PlaybookLibrary,
        registry: PrimitiveRegistry,
        *,
        graph: GraphAdapter | None = None,
        system: SystemAdapter | None = None,
        launch_profile: LaunchProfile | None = None,
        execution_enabled: bool = False,
        backend: ActionBackend | None = None,
        ack_store: Path | None = None,
        max_closed: int = 20,
    ) -> None:
        self.env = env
        self.library = library
        self.registry = registry
        self.graph = graph
        self.system = system
        self.launch_profile = launch_profile
        self.launch_profile_source = "parameter" if launch_profile else "none (all components expected)"
        self.expected_capabilities = list(launch_profile.capabilities) if launch_profile else []
        self.not_launched = env.items_not_launched(launch_profile)
        self.execution_enabled = execution_enabled
        self.history = StatusHistory()
        self.virtual_devices_missing: set[str] = set()
        self.incidents: dict[str, Incident] = {}
        self.closed: deque[Incident] = deque(maxlen=max_closed)
        self._last_close: dict[str, tuple[float, int]] = {}
        self.last_snapshot: HealthSnapshot | None = None
        self.last_cycle: dict[str, Any] = {}
        self._events: list[dict[str, Any]] = []
        self._ids = itertools.count(1)
        self.executor = RecoveryExecutor(
            env,
            registry,
            backend or _NoBackend(),
            enabled=execution_enabled and backend is not None,
            require_verified=env.require_verified,
            emit=self._emit_raw,
        )
        # Mission requests waiting for a final answer: request_id -> keys
        self._pending_requests: dict[str, dict[str, Any]] = {}
        self._responses: list[dict[str, Any]] = []
        # Operator acknowledgements of known deviations, keyed by incident key.
        self._ack_store = ack_store
        self._acks: dict[str, dict[str, Any]] = self._load_acks()
        self.mission_state: dict[str, Any] = {}
        self._last_tick_at = 0.0
        self.lifecycle = "RUNNING"
        self._draining_handles = []
        self.environment_state: dict[str, Any] = {"state": "UNKNOWN", "recovery_allowed": False,
                                                  "summary": "Waiting for environment evidence", "suppressed_items": []}

    # ------------------------------------------------------------------ entry points

    def on_snapshot(self, snapshot: HealthSnapshot) -> dict[str, Any]:
        """Heartbeat trigger: full-scope diagnosis on every health snapshot."""
        self.history.update(snapshot)
        self.last_snapshot = snapshot
        self._refresh_environment(snapshot.received_at)
        if self.lifecycle != "RUNNING":
            return {"stage_reached": 0, "result": "INTENTIONAL_STOP", "diagnoses": []}
        self._suspend_environment_incidents(snapshot.received_at)
        cycle = self._diagnose(snapshot, request=None)
        self._update_incidents(cycle["diagnoses"], trigger="heartbeat", resolve_missing=True)
        self.last_cycle = self._cycle_summary(cycle)
        self.tick(snapshot.received_at)
        return cycle

    def tick(self, now: float) -> None:
        """Advance executions. Called on every snapshot and by a timer in between."""
        self._last_tick_at = now
        if self.lifecycle != "RUNNING":
            return
        if self.last_snapshot is None:
            return
        self._refresh_environment(now)
        self._suspend_environment_incidents(now)
        ctx = self._ctx(self.last_snapshot)
        awaiting_pause = set()
        for key, incident in list(self.incidents.items()):
            self._apply_ack(key, incident)
            pb = self.library.get(incident.diagnosis.playbook_id or "")
            running = self.executor.active.get(key)
            # A successful spawn invalidates a "node missing" diagnosis before
            # its first output arrives. Finish this already-authorized action's
            # bounded verification instead of aborting it on that transition.
            restart_verifying = running is not None and running.component_recovery_in_flight
            if restart_verifying and not incident.acknowledged:
                incident.execution_blocked = "Authorized component recovery in progress; awaiting output verification"
                continue
            if pb is not None and pb.handling_class == "unknown_investigation":
                self.executor.abort(key, now, "Investigation required; no executable recovery diagnosis")
                incident.execution_blocked = "Investigator not connected; evidence retained"
                continue
            localization_recovery = pb is not None and pb.id == "PB-LOC-LOST"
            if not localization_recovery and key.startswith(MISSION_KEY_PREFIX) and self.mission_state.get("status") in {"DONE", "DONE_WITH_STUB", "FAILED", "FAILED_SKILL", "CANCELLED", "REJECTED"}:
                self.executor.abort(key, now, "Mission ended; mission-only recovery stopped")
                incident.execution_blocked = "Mission ended"
                continue
            if localization_recovery and self._navigation_pause_pending():
                # The incident and its capability constraint are global. Only
                # mutation waits for the dependent task to acknowledge its pause;
                # no task failure or Mission recovery request is required.
                incident.execution_blocked = "Waiting for active navigation to acknowledge Recovery pause"
                awaiting_pause.add(key)
                continue
            if incident.healthy_since is not None:
                running = self.executor.active.get(key)
                if running is None or (running.state != "RUNNING_ACTION" and not running.localization_verification_pending):
                    self.executor.resolved_by_evidence(key, now)
                incident.execution_blocked = "healthy evidence; awaiting stable resolution"
                continue
            if running is not None and (
                incident.status != CONFIRMED or pb is None or running.playbook.id != pb.id or incident.acknowledged
            ):
                # The run no longer matches the diagnosis (e.g. #50 -> #52 once a
                # longer trigger window filled): drop it and its operator request.
                why = "incident acknowledged" if incident.acknowledged else (
                    f"diagnosis changed to {pb.id if pb else incident.status}"
                )
                self.executor.abort(key, now, f"{SUPERSEDED_PREFIX}: {why}")
                self.executor.release(key)
            if incident.status != CONFIRMED or pb is None or incident.acknowledged:
                incident.execution_blocked = (
                    "acknowledged" if incident.acknowledged else None if pb is not None else "no playbook"
                )
                continue
            if key in self.executor.active:
                incident.execution_blocked = None
                continue
            reason = self.executor.blocked_reason(key, pb, now, incident.diagnosis.variables)
            incident.execution_blocked = reason
            if reason is None:
                self.executor.start(key, incident.id, pb, incident.diagnosis.variables, now)
            elif reason.startswith("chronic"):
                self.executor.open_chronic_request(key, pb, reason, now)
        self.executor.tick(ctx, now, blocked_keys=awaiting_pause)
        self._settle_requests()

    def _navigation_pause_pending(self) -> bool:
        state = self.mission_state
        plan, index = state.get("plan") or [], state.get("current_step_index", 0)
        if not isinstance(index, int) or not 0 <= index < len(plan) or plan[index].get("name") != "goto_pose":
            return False
        pause = state.get("recovery_pause")
        if pause and pause.get("motion_stopped") is not True:
            return True
        return state.get("status") == "EXECUTE_STEP"

    def handle_request(self, data: dict[str, Any]) -> dict[str, Any]:
        """Mission trigger: same engine, mission capabilities first, no debounce."""
        if self.lifecycle != "RUNNING":
            return {"request_id": data.get("request_id"), "mission_id": data.get("mission_id"),
                    "status": "CANCELLED", "final": True, "advice": "cancel", "summary": "Agent stack is stopping"}
        if self.env.kind == "sim" and not self.environment_state["recovery_allowed"]:
            return {"request_id": data.get("request_id"), "mission_id": data.get("mission_id"),
                    "status": "NEEDS_HUMAN", "final": True, "advice": "pause",
                    "summary": self.environment_state["summary"], "environment": self.environment_state}
        request = RecoveryRequest.from_dict(data)
        request.required_capabilities = self.env.normalize_capabilities(request.required_capabilities)
        request.suspected_components = self.env.normalize_components(request.suspected_components)
        base = {
            "request_id": request.request_id,
            "mission_id": request.mission_id,
            "generated_at": utc_now(),
            "execution_enabled": self.execution_enabled,
        }
        snapshot = self.last_snapshot
        if snapshot is None:
            return {**base, "status": "UNKNOWN", "advice": "retry_later", "summary": "No health snapshot received yet"}
        cycle = self._diagnose(snapshot, request=request)
        incidents = self._update_incidents(
            cycle["diagnoses"], trigger=f"mission:{request.request_id}", resolve_missing=False
        )
        for incident in incidents:
            incident.requests.append(request.request_id)
        response = {**base, **self._build_response(request, cycle, incidents, snapshot)}
        related_keys = response.pop("_related_keys", [])
        if response["status"] == "IN_PROGRESS":
            self.tick(snapshot.received_at)  # start executions now, not at the next snapshot
            self._pending_requests[request.request_id] = {
                "keys": related_keys,
                "mission_id": request.mission_id,
                "started": utc_now(),
            }
        response["final"] = response["status"] != "IN_PROGRESS"
        return response

    def drain_responses(self) -> list[dict[str, Any]]:
        """Follow-up (final) responses for mission requests answered IN_PROGRESS."""
        responses, self._responses = self._responses, []
        return responses

    def handle_command(self, data: dict[str, Any], now: float) -> dict[str, Any]:
        """Dashboard / operator commands (topic /agent/recovery_command)."""
        if self.lifecycle != "RUNNING":
            return {"command": data, "result": "Agent stack is stopping"}
        self._refresh_environment(now)
        self._suspend_environment_incidents(now)
        kind = str(data.get("type", ""))
        key = str(data.get("incident_key", ""))
        if kind == "set_auto_approve":
            enabled = data.get("enabled")
            if not isinstance(enabled, bool):
                result = "enabled must be boolean"
            elif enabled and (self.env.kind != "sim" or not self.executor.enabled):
                result = "Auto approval requires enabled sim Recovery"
            else:
                self.executor.auto_approve = enabled
                result = "ok"
        elif kind == "operator_reply":
            request_id = str(data.get("request_id", ""))
            reply = str(data.get("reply", ""))
            request = self.executor.requests.get(request_id)
            if request is not None and reply in {"skip", "ignore", "acknowledge"} and self._can_ignore(request.incident_key):
                result = self._acknowledge(request.incident_key, str(data.get("note", "")))
                self.executor.abort(request.incident_key, now, "Warning temporarily ignored by operator")
                self.executor.close_request(request_id)
            elif request is not None and request.kind == "chronic":
                result = "ok"
                if reply == "retry":
                    self.executor.retry(request.incident_key, now)
                elif reply == "acknowledge":
                    result = self._acknowledge(request.incident_key, str(data.get("note", "")))
                if result == "ok":
                    self.executor.close_request(request_id)
            else:
                result = self.executor.operator_reply(request_id, reply, now, str(data.get("note", "")))
        elif kind in {"acknowledge", "ignore"}:
            result = self._acknowledge(key, str(data.get("note", "")))
        elif kind == "unacknowledge":
            result = "ok" if self._acks.pop(key, None) is not None else f"{key} not acknowledged"
            if key in self.incidents:
                self.incidents[key].acknowledged = None
            self.executor.release(key)
            self._save_acks()
        elif kind == "retry":
            self.executor.retry(key, now)
            result = "ok"
        elif kind == "abort":
            self.executor.abort(key, now)
            result = "ok"
        else:
            result = f"unknown command type {kind!r}"
        self.tick(now)
        self._emit_raw("command_handled", {"command": data, "result": result})
        return {"command": data, "result": result}

    def drain_events(self) -> list[dict[str, Any]]:
        events, self._events = self._events, []
        return events

    def prepare_shutdown(self, now: float) -> None:
        self.lifecycle = "STOPPING"
        self.execution_enabled = self.executor.enabled = False
        self._draining_handles.extend(self.executor.draining_handles())
        # Aborting a run does not stop its backend thread. Retain the handles
        # and refuse shutdown until they have actually completed.
        for key, execution in list(self.executor.active.items()):
            if execution.handle is not None:
                self._draining_handles.append(execution.handle)
            self.executor.abort(key, now, "Agent stack shutdown requested")
        for request_id in list(self.executor.requests):
            self.executor.close_request(request_id)
        for request_id, request in self._pending_requests.items():
            self._responses.append({"request_id": request_id, "mission_id": request.get("mission_id"),
                                    "status": "CANCELLED", "final": True, "advice": "cancel",
                                    "summary": "Agent stack is stopping"})
        self._pending_requests.clear()

    def shutdown_ready(self):
        self._draining_handles = [h for h in self._draining_handles if h.poll()[0] == "RUNNING"]
        ready = not self._draining_handles
        if ready:
            self.lifecycle = "INTENTIONALLY_OFFLINE"
        return ready, "Recovery actions drained" if ready else "Waiting for in-flight recovery action to finish"

    def state(self) -> dict[str, Any]:
        snapshot = self.last_snapshot
        now = snapshot.received_at if snapshot is not None else 0.0
        return {
            "schema_version": 1,
            "lifecycle": self.lifecycle,
            "environment": self.environment_state,
            "startup_units": self._startup_unit_state(),
            "generated_at": utc_now(),
            "env": self.env.kind,
            "mode": "execute" if self.executor.enabled else "read_only",
            "execution_enabled": self.executor.enabled,
            "auto_approve": self.executor.auto_approve,
            "health_sequence": snapshot.sequence if snapshot else None,
            "robot_health_summary": robot_summary(snapshot.items if snapshot else {},
                [item for inc in self.incidents.values() if inc.acknowledged
                 for item in inc.diagnosis.candidate.root_items]),
            "launch_profile": self.launch_profile.name if self.launch_profile else None,
            "launch_profile_note": self.launch_profile.note if self.launch_profile else "",
            "launch_profile_source": self.launch_profile_source,
            "expected_capabilities": self.expected_capabilities,
            "not_launched_items": sorted(self.not_launched),
            "last_cycle": self.last_cycle,
            "incidents": [self._incident_dict(inc, snapshot) for inc in self._sorted_incidents()],
            "recently_closed": [self._incident_dict(inc, None) for inc in reversed(self.closed)],
            "recent_changes": self.history.recent(60.0, now) if snapshot else [],
            "ignored_warnings": [{"key": key, **ack, "active": key in self.incidents} for key, ack in self._acks.items()],
            "execution": self.executor.state(now),
            "task_supervision": self._task_supervision(),
            "pending_mission_requests": self._pending_requests,
            "playbooks": {
                "active": len(self.library.playbooks),
                "inactive": len(self.library.inactive),
                "catalog": self.library.catalog(),
            },
        }

    def _task_supervision(self) -> dict[str, Any]:
        constraints = []
        for incident in self._sorted_incidents():
            pb = self.library.get(incident.diagnosis.playbook_id or "")
            running = self.executor.active.get(incident.key)
            pb = pb or (running.playbook if running else None)
            if pb is None or pb.severity != "blocking" or incident.acknowledged:
                continue
            caps = incident.diagnosis.candidate.capabilities or pb.affects_capabilities
            if caps:
                constraints.append({"incident_id": incident.id, "key": incident.key,
                                    "playbook_id": pb.id, "capabilities": list(caps),
                                    "action": "pause_dependent", "reason": incident.diagnosis.reason})
        fresh = self.last_snapshot is not None and 0 <= self._last_tick_at - self.last_snapshot.received_at <= 5.0
        verified = [] if self.last_snapshot is None else [
            cap_id for cap_id, cap in self.last_snapshot.capabilities.items()
            if cap.get("status") == "AVAILABLE" and cap.get("dependencies")
            and all(self.last_snapshot.health_class(item) == "HEALTHY" for item in cap["dependencies"])
        ]
        return {"ready": bool(fresh and self.lifecycle == "RUNNING" and self.environment_state["recovery_allowed"]),
                "constraints": constraints, "verified_capabilities": verified}

    # ------------------------------------------------------------------ stages

    def _startup_unit_state(self) -> list[dict[str, Any]]:
        snapshot = self.last_snapshot
        units = []
        for spec in self.env.startup_units:
            expected = [item for item in spec.get("verify_items", []) if item not in self.not_launched]
            evidence = {item: snapshot.status(item) if snapshot else None for item in expected}
            health_age = (self.environment_state.get("evidence") or {}).get("health_age_sec", 0)
            if not expected:
                status = "NOT_REQUIRED" if spec.get("verify_items") else "UNVERIFIED"
            elif snapshot is None or not 0 <= health_age <= 5:
                status = "UNKNOWN"
            elif not self.environment_state["recovery_allowed"] and spec["id"] != "agent_stack":
                status = "ENVIRONMENT_UNAVAILABLE"
            elif all(classify_status(v) == "HEALTHY" for v in evidence.values()):
                status = "OBSERVED"  # never claim complete group verification from partial checks
            elif all(classify_status(v) in {"HEALTHY", "DEGRADED"} for v in evidence.values()):
                status = "DEGRADED"  # e.g. advancing Isaac clock with low RTF, not offline
            else:
                status = "NOT_READY"
            units.append({**spec, "status": status, "evidence": evidence})
        return units

    def _refresh_environment(self, now: float) -> None:
        if self.env.kind != "sim":
            self.environment_state = {"state": "NOT_APPLICABLE", "summary": "Real hardware uses component evidence.",
                                      "recovery_allowed": True, "suppressed_items": []}
            return
        if self.last_snapshot is None:
            return
        reader = getattr(self.system, "simulation_evidence", None)
        evidence = reader(now) if reader else {"process_alive": None, "source": "no local process observation"}
        previous = self.environment_state
        self.environment_state = simulation_environment(self.last_snapshot, evidence, now)
        self.environment_state["suppressed_items"] = [item for item in self.last_snapshot.items
                                                      if self._environment_suppresses(item)
                                                      and self.last_snapshot.health_class(item) in ABNORMAL_CLASSES]
        if previous["state"] != self.environment_state["state"]:
            self._emit_raw("environment_changed", {"previous": previous["state"], "environment": self.environment_state})

    def _environment_suppresses(self, item: str) -> bool:
        if self.env.kind != "sim" or self.environment_state["recovery_allowed"]:
            return False
        # Host/Agent diagnostics and asserted safety stops remain visible. An
        # absent safety heartbeat is not an asserted physical E-stop.
        if self.env.producer_of(item) in {"host", "agent"} or item.startswith("host_"):
            return False
        if item in self.env.safety_items and self.last_snapshot.status(item) in {"BLOCKED", "ERROR"}:
            return False
        return True

    def _suspend_environment_incidents(self, now: float) -> None:
        for key, incident in list(self.incidents.items()):
            roots = incident.diagnosis.candidate.root_items
            if not roots or not all(self._environment_suppresses(item) for item in roots):
                continue
            reason = "Environment unavailable; suspended, not recovered: " + self.environment_state["state"]
            self.executor.abort(key, now, reason)
            self.executor.release(key)
            incident.status = "ENVIRONMENT_SUSPENDED"
            incident.diagnosis.reason = reason
            incident.updated_at_utc = utc_now()
            self.closed.append(incident)
            del self.incidents[key]
            self._emit("incident_environment_suspended", incident)
        if not self.environment_state["recovery_allowed"]:
            for request_id, pending in list(self._pending_requests.items()):
                self._responses.append({"request_id": request_id, "mission_id": pending.get("mission_id"),
                                        "status": "NEEDS_HUMAN", "final": True, "advice": "pause",
                                        "summary": self.environment_state["summary"]})
                del self._pending_requests[request_id]

    def _ctx(self, snapshot: HealthSnapshot) -> ProbeContext:
        return ProbeContext(
            env=self.env,
            snapshot=snapshot,
            history=self.history,
            graph=self.graph,
            system=self.system,
            virtual_devices_missing=set(self.virtual_devices_missing),
            launch_profile_name=self.launch_profile.name if self.launch_profile else None,
            environment_ready=self.environment_state["recovery_allowed"],
        )

    def _diagnose(self, snapshot: HealthSnapshot, request: RecoveryRequest | None) -> dict[str, Any]:
        ctx = self._ctx(snapshot)
        # Stage 0: global scan of existing evidence only.
        abnormal, ignored = self._abnormal_items(snapshot)
        self.environment_state["suppressed_items"] = [item for item, why in ignored.items() if why == "environment_unavailable"]
        diagnoses: list[Diagnosis] = []
        if abnormal:
            # Stage 1: prioritise (not filter).
            priorities = {item: self._priority(item, snapshot, request) for item in abnormal}

            # Stage 2: walk dependencies upstream; healthy layers are skipped.
            candidates = self._find_roots(abnormal, snapshot, priorities)

            # Stage 3: playbook confirmation, highest priority / most upstream first.
            for cand in sorted(candidates, key=lambda c: (c.priority, c.layer_index, c.key)):
                if request is None and not self._debounced(cand, snapshot):
                    diagnoses.append(Diagnosis(cand, PENDING, reason=f"debounce {self.env.debounce_sec:.1f}s"))
                    continue
                diagnoses.append(self._match_playbook(cand, ctx, ignore_for_sec=request is not None))

        if request is not None:
            # Conditions that are only faults relative to a task (e.g. MANUAL
            # mode while the mission needs auto_navigation) never show up as
            # abnormal health items, so they are checked per request.
            diagnoses.extend(self._mission_trigger_diagnoses(request, ctx, diagnoses))

        for diagnosis in diagnoses:
            if diagnosis.outcome == CONFIRMED:
                diagnosis.plan = self._plan(self.library.get(diagnosis.playbook_id), ctx, diagnosis.variables)
        if not diagnoses:
            return {"stage_reached": 0, "result": "NO_FAULT_FOUND" if self.environment_state["recovery_allowed"] else "ENVIRONMENT_UNAVAILABLE", "diagnoses": [], "ignored": ignored}
        return {"stage_reached": 3, "result": "FAULTS_FOUND", "diagnoses": diagnoses, "ignored": ignored}

    def _mission_trigger_diagnoses(
        self, request: RecoveryRequest, ctx: ProbeContext, health_diagnoses: list[Diagnosis]
    ) -> list[Diagnosis]:
        results = []
        required = set(request.required_capabilities)
        health_related = [
            d for d in health_diagnoses if not required or required.intersection(d.candidate.capabilities)
        ]
        for pb in self.library.active():
            trigger = pb.mission_trigger
            if not trigger:
                continue
            caps = set(trigger.get("capabilities", []) or [])
            if caps and not caps.intersection(required):
                continue
            codes = trigger.get("result_codes")
            if codes and request.result_code not in codes:
                continue
            skills = trigger.get("skills")
            if skills and request.skill not in skills:
                continue
            if trigger.get("only_without_health_fault") and health_related:
                continue
            when = trigger.get("when")
            if when is not None:
                result = self.registry.run_probe(str(when["probe"]), ctx, when.get("args") or {})
                if result.outcome != PASS:
                    continue
            items = [str(item) for item in trigger.get("items", []) or []]
            cand = RootCandidate(
                key=f"{MISSION_KEY_PREFIX}{pb.id}",
                root_items=items,
                consequences=[],
                layer=pb.layer,
                layer_index=self.env.layers.index(pb.layer) if pb.layer in self.env.layers else len(self.env.layers),
                priority=P_MISSION,
                capabilities=sorted(caps),
                max_abnormal_age=None,
                upstream_checks=[],
            )
            variables = {"root_item": items[0] if items else "", "producer": ""}
            outcome, final_pb, steps, reason, uncertain = self._run_diagnosis(pb, ctx, [pb.id], variables)
            if outcome == "REJECTED":
                continue
            results.append(Diagnosis(cand, outcome, final_pb or pb.id, steps, reason, uncertain, variables=variables))
        return results

    def _abnormal_items(self, snapshot: HealthSnapshot) -> tuple[dict[str, str], dict[str, str]]:
        abnormal: dict[str, str] = {}
        ignored: dict[str, str] = {}
        expected = set(self.expected_capabilities)
        for item_id, entry in snapshot.items.items():
            health_class = classify_status(entry.get("status"))
            if health_class not in ABNORMAL_CLASSES:
                continue
            if self._environment_suppresses(item_id):
                ignored[item_id] = "environment_unavailable"
                continue
            if item_id in self.env.ignored_items:
                ignored[item_id] = "ignored_by_env"
                continue
            if item_id in self.not_launched:
                ignored[item_id] = "not_launched_in_profile"
                continue
            if item_id in self.env.event_driven_items and str(entry.get("status")).upper() in {
                "STALE",
                "UNKNOWN",
                "NOT_OBSERVED",
            }:
                ignored[item_id] = "event_driven_idle"
                continue
            caps = snapshot.capabilities_for_item(item_id)
            if expected and caps and not expected.intersection(caps):
                ignored[item_id] = "capability_not_launched"
                continue
            if health_class == UNCERTAIN and not entry.get("critical"):
                ignored[item_id] = "uncertain_non_critical"
                continue
            abnormal[item_id] = health_class
        # Optional in isolation does not mean optional to a failed capability.
        # A freshly restarted monitor has UNKNOWN upstream signals, not STALE.
        for item_id in list(abnormal):
            for upstream in self._ancestors(item_id):
                if ignored.get(upstream) == "uncertain_non_critical":
                    abnormal[upstream] = snapshot.health_class(upstream)
                    del ignored[upstream]
        return abnormal, ignored

    def _priority(self, item: str, snapshot: HealthSnapshot, request: RecoveryRequest | None) -> int:
        if item in self.env.safety_items or self.env.layer_of(item) == "safety":
            return P_SAFETY
        caps = snapshot.capabilities_for_item(item)
        if request is not None:
            suspected = set(request.suspected_components)
            if set(request.required_capabilities).intersection(caps):
                return P_MISSION
            if item in suspected or self.env.producer_of(item) in suspected:
                return P_MISSION
        return P_CAPABILITY if caps else P_OTHER

    def _ancestors(self, item: str) -> list[str]:
        seen: list[str] = []
        queue = list(self.env.upstream(item))
        while queue:
            current = queue.pop(0)
            if current in seen or current == item:
                continue
            seen.append(current)
            queue.extend(self.env.upstream(current))
        return seen

    def _find_roots(
        self, abnormal: dict[str, str], snapshot: HealthSnapshot, priorities: dict[str, int]
    ) -> list[RootCandidate]:
        ancestors = {item: self._ancestors(item) for item in abnormal}
        roots = [item for item in abnormal if not any(a in abnormal for a in ancestors[item])]
        # Topmost abnormal ancestors of each consequence = the roots it hangs off.
        hangs_off = {
            item: [a for a in ancestors[item] if a in roots]
            for item in abnormal
            if item not in roots
        }
        groups: dict[str, list[str]] = {}
        for root in roots:
            groups.setdefault(self.env.producer_of(root) or root, []).append(root)

        candidates = []
        now = snapshot.received_at
        for key, root_items in groups.items():
            consequences = sorted(
                item for item, tops in hangs_off.items() if set(tops).intersection(root_items)
            )
            members = [*root_items, *consequences]
            checks = []
            for root in root_items:
                for upstream in sorted(ancestors[root], key=self.env.layer_index):
                    status = snapshot.status(upstream)
                    checks.append(
                        {
                            "for": root,
                            "upstream": upstream,
                            "layer": self.env.layer_of(upstream),
                            "status": status,
                            "result": ("unobserved" if status is None else "healthy_skipped"
                                       if classify_status(status) == "HEALTHY" else "not_healthy_skipped"),
                        }
                    )
            ages = [self.history.abnormal_age(item, now) for item in root_items]
            ages = [age for age in ages if age is not None]
            top = min(root_items, key=self.env.layer_index)
            candidates.append(
                RootCandidate(
                    key=key,
                    root_items=sorted(root_items),
                    consequences=consequences,
                    layer=self.env.layer_of(top),
                    layer_index=self.env.layer_index(top),
                    priority=min(priorities[item] for item in members),
                    capabilities=sorted({cap for item in members for cap in snapshot.capabilities_for_item(item)}),
                    max_abnormal_age=max(ages) if ages else None,
                    upstream_checks=checks,
                )
            )
        return candidates

    def _debounced(self, cand: RootCandidate, snapshot: HealthSnapshot) -> bool:
        if cand.priority == P_SAFETY and any(snapshot.status(i) == "BLOCKED" for i in cand.root_items):
            return True
        return cand.max_abnormal_age is not None and cand.max_abnormal_age >= self.env.debounce_sec

    def _match_playbook(self, cand: RootCandidate, ctx: ProbeContext, *, ignore_for_sec: bool) -> Diagnosis:
        if all((ctx.snapshot.item(item) or {}).get("warning_kind") == "low_rate" for item in cand.root_items):
            return Diagnosis(cand, WARNING, reason="Low simulation sensor rate; warning only, no automatic recovery")
        if all((ctx.snapshot.item(item) or {}).get('response_policy', {}).get('level') == 'WARN'
               for item in cand.root_items):
            return Diagnosis(cand, WARNING, reason='Robot WARN: inspect item response_policy checks; notify human; no automatic disruptive PB')
        matching: list[tuple[Playbook, str]] = []
        for pb in self.library.active():
            item = self._triggered_item(pb, cand, ctx.snapshot, ignore_for_sec)
            if item is not None:
                matching.append((pb, item))
        matching.sort(key=lambda match: (int(match[0].raw.get("match_priority", 50)), match[0].id))
        # Playbooks whose status condition already holds but whose `for_sec`
        # window is still filling: wait for them instead of opening an
        # unmatched incident that would flap (seen with host_resources).
        matched_ids = {pb.id for pb, _ in matching}
        waiting = []
        if not ignore_for_sec:
            for pb in self.library.active():
                if pb.id in matched_ids:
                    continue
                item = self._triggered_item(pb, cand, ctx.snapshot, True)
                if item is None:
                    continue
                # Only worth waiting for if its (A0) diagnosis would not reject.
                variables = {"root_item": item, "producer": self.env.producer_of(item) or ""}
                if self._run_diagnosis(pb, ctx, [pb.id], variables)[0] != "REJECTED":
                    waiting.append(pb.id)
        if not matching:
            if waiting:
                return Diagnosis(cand, PENDING, reason=f"Waiting for trigger window of {', '.join(waiting)}")
            return Diagnosis(cand, NEEDS_INVESTIGATION, reason="No playbook trigger matched the root items")
        # A more specific playbook (lower match_priority) is still filling its
        # window: decide when it can, instead of acting on the generic one and
        # switching later (live sim: #50 NODE-DOWN at 3 s, then #52 at 30 s).
        best = int(matching[0][0].raw.get("match_priority", 50))
        preferred = [
            pb_id for pb_id in waiting if int(self.library.get(pb_id).raw.get("match_priority", 50)) < best
        ]
        if preferred:
            return Diagnosis(cand, PENDING, reason=f"Waiting for trigger window of {', '.join(preferred)}")

        all_steps: list[dict[str, Any]] = []
        for pb, item in matching:
            variables = {"root_item": item, "producer": self.env.producer_of(item) or ""}
            outcome, final_pb, steps, reason, uncertain = self._run_diagnosis(pb, ctx, [pb.id], variables)
            all_steps.extend(steps)
            if outcome in (CONFIRMED, NEEDS_INVESTIGATION):
                return Diagnosis(
                    cand,
                    outcome,
                    final_pb if outcome == CONFIRMED else pb.id,
                    all_steps,
                    reason,
                    uncertain,
                    variables=variables,
                )
            # REJECTED: this playbook does not apply; try the next one.
        if waiting:
            return Diagnosis(cand, PENDING, None, all_steps, f"Waiting for trigger window of {', '.join(waiting)}")
        return Diagnosis(cand, NEEDS_INVESTIGATION, None, all_steps, "All matching playbooks rejected the evidence")

    def _triggered_item(
        self, pb: Playbook, cand: RootCandidate, snapshot: HealthSnapshot, ignore_for_sec: bool
    ) -> str | None:
        """First root item that satisfies one of the playbook's trigger conditions."""
        for cond in pb.trigger:
            wanted = str(cond["item"])
            effective = {**cond, "for_sec": 0.0} if ignore_for_sec else cond
            for item in cand.root_items:
                if wanted not in (WILDCARD, item):
                    continue
                age = self.history.abnormal_age(item, snapshot.received_at)
                if trigger_matches(effective, snapshot.status(item), age):
                    return item
        return None

    def _run_diagnosis(
        self, pb: Playbook, ctx: ProbeContext, visited: list[str], variables: dict[str, str]
    ) -> tuple[str, str | None, list[dict[str, Any]], str, bool]:
        steps: list[dict[str, Any]] = []
        uncertain = False
        # Positive evidence is mandatory where specified. Missing evidence is
        # not confirmation; keep the abnormal observation without running a PB.
        for index, spec in enumerate(pb.supporting_evidence):
            args = substitute(spec.get("args") or {}, variables)
            result = self.registry.run_probe(str(spec["probe"]), ctx, args)
            steps.append({"playbook": pb.id, "step": f"support:{index + 1}",
                          "probe": spec["probe"], "args": args, "expected": PASS,
                          **result.to_dict(), "result": "match" if result.outcome == PASS else "insufficient_support"})
            if result.outcome != PASS:
                return NEEDS_INVESTIGATION, pb.id, steps, f"Supporting evidence missing: {result.detail}", True
        for step in pb.diagnosis:
            args = substitute(step.get("args") or {}, variables)
            result = self.registry.run_probe(str(step["probe"]), ctx, args)
            expected = PASS if str(step.get("expect", "pass")) == "pass" else FAIL
            record = {
                "playbook": pb.id,
                "step": step["id"],
                "probe": step["probe"],
                "args": args,
                "expected": expected,
                **result.to_dict(),
            }
            steps.append(record)
            if step.get("informational"):
                # Evidence recorded for the operator / investigator only.
                record["result"] = "info"
                continue
            if result.outcome == UNKNOWN:
                uncertain = True
                if str(step.get("on_unknown", "mismatch")) == "continue":
                    record["result"] = "no_evidence_continue"
                    continue
            elif result.outcome == expected:
                record["result"] = "match"
                if step.get("confirm"):
                    return CONFIRMED, pb.id, steps, f"Confirmed at step {step['id']}", uncertain
                continue
            record["result"] = "mismatch"
            action = step.get("on_mismatch") or {"escalate": "R3"}
            if "reject" in action:
                return "REJECTED", None, steps, f"{pb.id} rejected at {step['id']}", uncertain
            if "handoff" in action:
                target_id = str(action["handoff"])
                target = self.library.get(target_id)
                if target is None or target.env_status == "n/a":
                    return NEEDS_INVESTIGATION, None, steps, f"Handoff target {target_id} is n/a in {self.env.kind}", uncertain
                if target_id in visited or len(visited) > MAX_HANDOFF_DEPTH:
                    return NEEDS_INVESTIGATION, None, steps, f"Handoff loop at {target_id}", uncertain
                outcome, final, more, reason, more_uncertain = self._run_diagnosis(
                    target, ctx, [*visited, target_id], variables
                )
                return outcome, final, steps + more, reason, uncertain or more_uncertain
            return NEEDS_INVESTIGATION, None, steps, f"{pb.id} escalated at {step['id']}: {action.get('escalate')}", uncertain
        return CONFIRMED, pb.id, steps, "All diagnosis steps matched", uncertain

    def _plan(self, pb: Playbook | None, ctx: ProbeContext, variables: dict[str, str]) -> dict[str, Any] | None:
        """Stage 4 dry run: what would be done, and is it currently allowed."""
        if pb is None:
            return None
        pb = pb.bind_components(self.env, variables)
        preconditions = []
        for name in pb.preconditions:
            result = self.registry.run_precondition(name, ctx)
            preconditions.append({"name": name, **result.to_dict()})
        steps = []
        for step in pb.recovery:
            risk = str(step["risk"])
            steps.append(
                {
                    "type": step["type"],
                    "primitive": step.get("primitive"),
                    "args": substitute(step.get("args") or {}, variables),
                    "risk": risk,
                    "approval": self.env.action_approval(risk, step.get("primitive")),
                    "template": step.get("template"),
                    "done_when": substitute(step.get("done_when") or [], variables),
                    "until": substitute(step.get("until") or [], variables),
                }
            )
        return {
            "playbook": pb.summary(),
            "preconditions": preconditions,
            "preconditions_met": all(p["outcome"] == PASS for p in preconditions),
            "steps": steps,
            "verify": substitute(pb.verify, variables),
            "policy": pb.policy,
            "operator_text": substitute(pb.operator_text, variables),
            "execution": "enabled" if self.execution_enabled else "disabled_read_only_v1",
        }

    # ------------------------------------------------------------------ incidents

    def _update_incidents(
        self, diagnoses: list[Diagnosis], *, trigger: str, resolve_missing: bool
    ) -> list[Incident]:
        touched: list[Incident] = []
        now = self.last_snapshot.received_at if self.last_snapshot is not None else 0.0
        now_utc = utc_now()
        active_keys = {d.candidate.key for d in diagnoses}
        for diagnosis in diagnoses:
            key = diagnosis.candidate.key
            incident = self.incidents.get(key)
            if incident is not None:
                incident.healthy_since = None
                running = self.executor.active.get(key)
                if (running is not None and running.playbook.id == "PB-LOC-LOST"
                        and running.state in {"WAITING_SUCCESS", "VERIFYING", "WAITING_OPERATOR"}
                        and diagnosis.outcome in {NEEDS_INVESTIGATION, WARNING}
                        and set(diagnosis.candidate.root_items) <= {"gicp_score", "gicp_pose", "localization_consistency"}):
                    # Reseeding clears old pose/score evidence while scans accumulate.
                    # Keep bounded verification after BAD improves to WARN/UNKNOWN.
                    # This does not start a new playbook for a warning.
                    touched.append(incident)
                    continue
            if diagnosis.outcome == PENDING:
                if incident is not None:
                    touched.append(incident)
                continue
            if incident is None:
                last_close = self._last_close.get(key)
                reopen_count = (
                    last_close[1] + 1 if last_close is not None and now - last_close[0] <= REOPEN_WINDOW_SEC else 0
                )
                incident = Incident(
                    id=f"inc-{datetime.now().strftime('%Y%m%d-%H%M%S')}-{next(self._ids)}",
                    key=key,
                    opened_at=now,
                    opened_at_utc=now_utc,
                    updated_at_utc=now_utc,
                    status=diagnosis.outcome,
                    diagnosis=diagnosis,
                    reopen_count=reopen_count,
                )
                self.incidents[key] = incident
                self._emit("incident_reopened" if reopen_count else "incident_opened", incident)
            else:
                changed = (incident.status, incident.diagnosis.playbook_id) != (
                    diagnosis.outcome,
                    diagnosis.playbook_id,
                )
                incident.diagnosis = diagnosis
                incident.status = diagnosis.outcome
                incident.updated_at_utc = now_utc
                if changed:
                    self._emit("incident_updated", incident)
            incident.triggers.add(trigger.split(":")[0])
            touched.append(incident)
        if resolve_missing:
            for key in [k for k in self.incidents if k not in active_keys]:
                incident = self.incidents[key]
                if key.startswith(MISSION_KEY_PREFIX):
                    if not self._mission_incident_active(incident, now):
                        self._close(key, now, now_utc)
                    continue
                parents = [d.candidate.key for d in diagnoses
                           if d.candidate.key != key and d.outcome != PENDING and d.candidate.key in self.incidents
                           and set(incident.diagnosis.candidate.root_items)
                           and set(incident.diagnosis.candidate.root_items) <= set(d.candidate.consequences)]
                if parents:
                    # Archive, not resolve: the downstream fault has not healed.
                    for pending in self._pending_requests.values():
                        if key in pending["keys"]:
                            pending["keys"] = list(dict.fromkeys([k for k in pending["keys"] if k != key] + parents))
                    self.executor.abort(key, now, "Subsumed by upstream fault: " + ", ".join(parents))
                    incident.status = "SUBSUMED"
                    incident.updated_at_utc = now_utc
                    incident.diagnosis.reason = "Historical diagnosis; now covered by " + ", ".join(parents)
                    self.closed.append(incident)
                    del self.incidents[key]
                    self._emit("incident_subsumed", incident)
                    continue
                if self.last_snapshot is None or not all(
                    self.last_snapshot.health_class(item) == "HEALTHY"
                    for item in incident.diagnosis.candidate.root_items
                ):
                    # An upstream fault can hide this candidate without fixing it.
                    incident.healthy_since = None
                    continue
                # Hysteresis: the root must stay healthy for resolve_after_sec.
                if incident.healthy_since is None:
                    incident.healthy_since = now
                if now - incident.healthy_since >= self.env.resolve_after_sec:
                    running = self.executor.active.get(key)
                    if running is None or (not running.component_recovery_in_flight and running.state != "RUNNING_ACTION" and not running.localization_verification_pending):
                        self._close(key, now, now_utc)
        return touched

    def _mission_incident_active(self, incident: Incident, now: float) -> bool:
        """Heartbeat cannot re-run a mission trigger, so re-check its `when` probe or expire it."""
        pb = self.library.get(incident.diagnosis.playbook_id or "")
        when = (pb.mission_trigger or {}).get("when") if pb is not None else None
        if when is None or self.last_snapshot is None:
            return now - incident.opened_at < MISSION_INCIDENT_TTL_SEC
        ctx = self._ctx(self.last_snapshot)
        return self.registry.run_probe(str(when["probe"]), ctx, when.get("args") or {}).outcome == PASS

    def _close(self, key: str, now: float, now_utc: str) -> None:
        self.executor.resolved_by_evidence(key, now)
        incident = self.incidents.pop(key)
        self._last_close[key] = (now, incident.reopen_count)
        incident.status = "RESOLVED"
        incident.resolved_at_utc = now_utc
        self.closed.append(incident)
        self._emit("incident_resolved", incident)

    def _incident_dict(self, incident: Incident, snapshot: HealthSnapshot | None) -> dict[str, Any]:
        data = incident.to_dict(snapshot)
        pb = self.library.get(incident.diagnosis.playbook_id or "")
        data["playbook_no"] = pb.number if pb is not None else None
        data["title"] = pb.title if pb is not None else incident.key
        data["severity"] = pb.severity if pb is not None else ("blocking" if incident.diagnosis.candidate.priority == P_SAFETY else "degraded")
        execution = self.executor.active.get(incident.key)
        data["execution_state"] = execution.state if execution else None
        last = next((ex for ex in reversed(self.executor.finished) if ex.incident_id == incident.id), None)
        data["last_execution"] = ({"state": last.state, "message": last.message,
                                    "finished_at": last.finished_at_utc} if last else None)
        data["handling_class"] = (pb.handling_class if pb is not None and incident.diagnosis.outcome == CONFIRMED
                                  else "unknown_investigation")
        data["investigator_available"] = False
        return data

    def _emit_raw(self, event_type: str, data: dict[str, Any]) -> None:
        self._events.append({"type": event_type, "generated_at": utc_now(), **data})

    # ------------------------------------------------------------------ mission follow-ups

    def _settle_requests(self) -> None:
        """Send the final response once every related incident's execution ended."""
        for request_id, pending in list(self._pending_requests.items()):
            outcomes = []
            for key in pending["keys"]:
                if key in self.executor.active:
                    break
                if key not in self.incidents:
                    closed = next((inc for inc in reversed(self.closed) if inc.key == key), None)
                    if closed is None:
                        break  # not yet diagnosed, or evidence history unavailable
                    outcomes.append(SUCCEEDED if closed.status == "RESOLVED" else ABORTED)
                    continue
                finished = next((ex for ex in reversed(self.executor.finished) if ex.incident_key == key), None)
                if finished is None:
                    break  # not started yet (e.g. waiting for actuator lock)
                outcomes.append(finished.state)
            else:
                del self._pending_requests[request_id]
                resolved = all(state == SUCCEEDED for state in outcomes)
                details = [
                    {"key": ex.incident_key, "playbook_id": ex.playbook.id, "state": ex.state, "message": ex.message}
                    for ex in self.executor.finished
                    if ex.incident_key in pending["keys"]
                ][-len(pending["keys"]):]
                self._responses.append(
                    {
                        "request_id": request_id,
                        "mission_id": pending["mission_id"],
                        "generated_at": utc_now(),
                        "final": True,
                        "status": "RESOLVED" if resolved else ("FAILED" if FAILED in outcomes else "NEEDS_HUMAN"),
                        "advice": "resume_from_step" if resolved else "pause",
                        "summary": (
                            "Recovery completed and verified; mission can resume."
                            if resolved
                            else "Recovery did not complete: " + "; ".join(d["message"] for d in details if d["state"] != SUCCEEDED)
                        ),
                        "executions": details,
                        "escalated": ESCALATED in outcomes,
                    }
                )

    # ------------------------------------------------------------------ acknowledgements

    @staticmethod
    def _ack_signature(incident: Incident) -> list[Any]:
        # The ack holds while the same playbook explains the same root items.
        cand = incident.diagnosis.candidate
        return [incident.diagnosis.playbook_id, sorted(cand.root_items)]

    def _acknowledge(self, key: str, note: str) -> str:
        incident = self.incidents.get(key)
        if incident is None:
            return f"no open incident {key}"
        if not self._can_ignore(key):
            return "Only non-blocking warnings may be temporarily ignored"
        ack = {"at": utc_now(), "note": note, "signature": self._ack_signature(incident),
               "playbook_id": incident.diagnosis.playbook_id,
               "summary": incident.diagnosis.reason}
        self._acks[key] = ack
        incident.acknowledged = ack
        self._save_acks()
        self._emit("incident_acknowledged", incident)
        return "ok"

    def _can_ignore(self, key: str) -> bool:
        incident = self.incidents.get(key)
        if incident is None or self.last_snapshot is None:
            return False
        cand = incident.diagnosis.candidate
        pb = self.library.get(incident.diagnosis.playbook_id or "")
        return (cand.priority != P_SAFETY and (pb is None or pb.severity != "blocking")
                and all(self.last_snapshot.health_class(item) in {"HEALTHY", "DEGRADED"}
                        for item in cand.root_items))

    def _apply_ack(self, key: str, incident: Incident) -> None:
        ack = self._acks.get(key)
        if ack is None:
            incident.acknowledged = None
            return
        if ack.get("signature") != self._ack_signature(incident) or not self._can_ignore(key):
            # Evidence changed (different root / playbook): show it again.
            self._acks.pop(key, None)
            incident.acknowledged = None
            self._save_acks()
            self._emit("incident_ack_cleared", incident)
            return
        incident.acknowledged = ack

    def _load_acks(self) -> dict[str, dict[str, Any]]:
        if self._ack_store is None:
            return {}
        try:
            data = json.loads(self._ack_store.read_text())
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def _save_acks(self) -> None:
        if self._ack_store is None:
            return
        try:
            self._ack_store.parent.mkdir(parents=True, exist_ok=True)
            self._ack_store.write_text(json.dumps(self._acks, ensure_ascii=False, indent=1))
        except OSError:
            pass

    def _emit(self, event_type: str, incident: Incident) -> None:
        self._events.append(
            {
                "type": event_type,
                "generated_at": utc_now(),
                "incident_id": incident.id,
                "key": incident.key,
                "status": incident.status,
                "playbook_id": incident.diagnosis.playbook_id,
                "reopen_count": incident.reopen_count,
                "priority": incident.diagnosis.candidate.priority,
                "root_items": incident.diagnosis.candidate.root_items,
            }
        )

    def _sorted_incidents(self) -> list[Incident]:
        return sorted(
            self.incidents.values(),
            key=lambda inc: (inc.diagnosis.candidate.priority, inc.diagnosis.candidate.layer_index, inc.key),
        )

    def _cycle_summary(self, cycle: dict[str, Any]) -> dict[str, Any]:
        diagnoses: list[Diagnosis] = cycle["diagnoses"]
        return {
            "stage_reached": cycle["stage_reached"],
            "result": cycle["result"],
            "pending": [d.to_dict() for d in diagnoses if d.outcome == PENDING],
            "ignored": cycle["ignored"],
        }

    def _build_response(
        self,
        request: RecoveryRequest,
        cycle: dict[str, Any],
        incidents: list[Incident],
        snapshot: HealthSnapshot,
    ) -> dict[str, Any]:
        required = set(request.required_capabilities)
        diagnoses: list[Diagnosis] = cycle["diagnoses"]
        caps = required or set(snapshot.capabilities)
        capabilities = {
            cap: (snapshot.capabilities.get(cap) or {}).get("status", "UNKNOWN") for cap in sorted(caps)
        }
        suspected_report = []
        for comp in request.suspected_components:
            items = self.env.items_of_component(comp) or ([comp] if snapshot.item(comp) else [])
            suspected_report.append(
                {"component": comp, "items": {item: snapshot.status(item) for item in items}}
            )
        common = {
            "capabilities": capabilities,
            "suspected_components": suspected_report,
            "incidents": [self._incident_dict(inc, snapshot) for inc in incidents],
        }
        if not diagnoses:
            return {
                **common,
                "status": "NO_FAULT_FOUND",
                "advice": "handle_as_task_failure",
                "summary": "Health evidence shows no fault; treat the failure as task-level",
            }
        related = [
            d
            for d in diagnoses
            if d.outcome != WARNING
            and not (self._acks.get(d.candidate.key) and self._can_ignore(d.candidate.key))
            and (d.candidate.priority <= P_MISSION or not required or required.intersection(d.candidate.capabilities))
        ]
        if not related:
            return {
                **common,
                "status": "NOT_RELATED",
                "advice": "handle_as_task_failure",
                "summary": "Faults exist but none affect the requested capabilities; they are handled in the background",
            }
        safety = any(d.candidate.priority == P_SAFETY for d in related)
        top = related[0]
        now = snapshot.received_at
        executable = self.executor.enabled and all(
            d.outcome == CONFIRMED
            and self.library.get(d.playbook_id or "") is not None
            and self.library.get(d.playbook_id or "").handling_class != "unknown_investigation"
            and not self._acks.get(d.candidate.key)
            and (
                d.candidate.key in self.executor.active
                or self.executor.blocked_reason(d.candidate.key, self.library.get(d.playbook_id or ""), now, d.variables) is None
            )
            for d in related
        )
        return {
            **common,
            "_related_keys": [d.candidate.key for d in related],
            "status": "IN_PROGRESS" if executable else "NEEDS_HUMAN",
            "advice": "pause" if safety else "pause_and_wait_for_recovery",
            "safety_blocked": safety,
            "root_cause": {
                "playbook": top.playbook_id,
                "playbook_no": getattr(self.library.get(top.playbook_id or ""), "number", None),
                "outcome": top.outcome,
                "root_items": top.candidate.root_items,
                "reason": top.reason,
            },
            "summary": (
                f"{len(related)} related fault(s); top root {top.candidate.key} "
                f"({top.playbook_id or 'no playbook'}, {top.outcome})"
            ),
        }


class _NoBackend:
    def supports(self, primitive: str, args: dict[str, Any]) -> tuple[bool, str]:
        return False, "no action backend"

    def start(self, primitive: str, args: dict[str, Any]):
        raise RuntimeError("no action backend")

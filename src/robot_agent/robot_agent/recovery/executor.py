from __future__ import annotations

import itertools
import math
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Callable, Protocol
from uuid import uuid4

from .environment import EnvProfile, risk_rank
from .playbook import Playbook
from .primitives import PASS, PrimitiveRegistry, ProbeContext


# Execution states
WAITING_PRECONDITIONS = "WAITING_PRECONDITIONS"
WAITING_CONFIRM = "WAITING_CONFIRM"
RUNNING_ACTION = "RUNNING_ACTION"
WAITING_SUCCESS = "WAITING_SUCCESS"
WAITING_OPERATOR = "WAITING_OPERATOR"
WAITING_EVIDENCE = "WAITING_EVIDENCE"
VERIFYING = "VERIFYING"
SUCCEEDED = "SUCCEEDED"
FAILED = "FAILED"          # verify failed and no retry left -> handed to human
ESCALATED = "ESCALATED"    # operator said "cannot" / forbidden action / R3 needed
ABORTED = "ABORTED"
TERMINAL = {SUCCEEDED, FAILED, ESCALATED, ABORTED}

# Action handle states
RUNNING = "RUNNING"
DONE = "DONE"
ERROR = "ERROR"

DEFAULT_SUCCESS_TIMEOUT_SEC = 20.0
DEFAULT_VERIFY_TIMEOUT_SEC = 20.0
DEFAULT_EVIDENCE_TIMEOUT_SEC = 120.0
# Unmet preconditions are shown to the operator after this long.
PRECONDITION_PROMPT_SEC = 10.0
# Minimum gap between two automatic runs of the same incident when the
# playbook sets no cooldown (stops success -> re-fault -> re-run spinning).
DEFAULT_COOLDOWN_SEC = 30.0

SUPERSEDED_PREFIX = "Superseded"

OPERATOR_REPLIES = {"done", "cannot", "skip", "abort", "approve", "reject"}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class ActionHandle(Protocol):
    def poll(self) -> tuple[str, str]:
        """(RUNNING | DONE | ERROR, detail)."""
        ...


class ActionBackend(Protocol):
    def supports(self, primitive: str, args: dict[str, Any]) -> tuple[bool, str]: ...

    def start(self, primitive: str, args: dict[str, Any]) -> ActionHandle: ...


@dataclass
class OperatorRequest:
    """One question to the operator on the Dashboard (R2 channel)."""

    id: str
    kind: str  # operator_action | confirm_action | precondition | chronic | manual_fallback
    incident_key: str
    playbook_id: str | None
    playbook_number: int | None
    title: str
    text: dict[str, str]
    buttons: list[str]
    created_at_utc: str
    created_at: float
    done_when: list[dict[str, Any]] = field(default_factory=list)
    step: dict[str, Any] | None = None
    last_check: list[dict[str, Any]] = field(default_factory=list)
    reply: str | None = None
    note: str = ""
    action_context: dict[str, Any] = field(default_factory=dict)

    def to_dict(self, now: float) -> dict[str, Any]:
        return {
            "id": self.id,
            "kind": self.kind,
            "incident_key": self.incident_key,
            "playbook_id": self.playbook_id,
            "playbook_number": self.playbook_number,
            "title": self.title,
            "text": self.text,
            "buttons": self.buttons,
            "created_at": self.created_at_utc,
            "waiting_sec": max(0.0, now - self.created_at),
            "done_when": self.done_when,
            "step": self.step,
            "last_check": self.last_check,
            "note": self.note,
            "action_context": self.action_context,
        }


@dataclass
class Execution:
    id: str
    incident_key: str
    incident_id: str
    playbook: Playbook
    variables: dict[str, str]
    started_at: float
    started_at_utc: str
    attempt: int = 1
    state: str = WAITING_PRECONDITIONS
    step_index: int = 0
    state_since: float = 0.0
    handle: ActionHandle | None = None
    request: OperatorRequest | None = None
    message: str = ""
    finished_at_utc: str | None = None
    log: list[dict[str, Any]] = field(default_factory=list)
    # The operator skipped a step in this run: do not retry on our own.
    operator_skipped: bool = False
    evidence_after: float | None = None
    checked_at: float = 0.0

    @property
    def terminal(self) -> bool:
        return self.state in TERMINAL

    @property
    def component_recovery_in_flight(self) -> bool:
        return self.state in {RUNNING_ACTION, WAITING_SUCCESS, VERIFYING} and any(
            entry["event"] == "action_started" and entry["detail"].startswith(("restart_component ", "lifecycle_reset "))
            for entry in self.log
        )

    @property
    def localization_verification_pending(self) -> bool:
        return self.playbook.id == "PB-LOC-LOST" and self.evidence_after is not None

    def to_dict(self, now: float) -> dict[str, Any]:
        step = self.playbook.recovery[self.step_index] if self.step_index < len(self.playbook.recovery) else None
        return {
            "id": self.id,
            "incident_key": self.incident_key,
            "incident_id": self.incident_id,
            "playbook_id": self.playbook.id,
            "playbook_number": self.playbook.number,
            "attempt": self.attempt,
            "state": self.state,
            "state_for_sec": max(0.0, now - self.state_since),
            "step_index": self.step_index,
            "step_count": len(self.playbook.recovery),
            "current_step": _step_summary(step, self.variables) if step else None,
            "message": self.message,
            "started_at": self.started_at_utc,
            "finished_at": self.finished_at_utc,
            "operator_request_id": self.request.id if self.request else None,
            "log": self.log[-30:],
        }


def substitute(value: Any, variables: dict[str, str]) -> Any:
    if isinstance(value, str):
        for name, replacement in variables.items():
            value = value.replace("${" + name + "}", replacement)
        return value
    if isinstance(value, dict):
        return {key: substitute(item, variables) for key, item in value.items()}
    if isinstance(value, list):
        return [substitute(item, variables) for item in value]
    return value


def _step_summary(step: dict[str, Any], variables: dict[str, str]) -> dict[str, Any]:
    return {
        "type": step.get("type"),
        "primitive": step.get("primitive"),
        "template": step.get("template"),
        "args": substitute(step.get("args") or {}, variables),
        "risk": step.get("risk"),
    }


class RecoveryExecutor:
    """Runs confirmed playbooks: preconditions -> steps -> verify, with policy.

    Tick-driven and side-effect free except through the ActionBackend, so the
    whole state machine is unit-testable with fake backends and snapshots.
    """

    def __init__(
        self,
        env: EnvProfile,
        registry: PrimitiveRegistry,
        backend: ActionBackend,
        *,
        enabled: bool,
        require_verified: bool = False,
        emit: Callable[[str, dict[str, Any]], None] | None = None,
    ) -> None:
        self.env = env
        self.registry = registry
        self.backend = backend
        self.enabled = enabled
        self.auto_approve = False
        self.require_verified = require_verified
        self._emit_cb = emit or (lambda _type, _data: None)
        self.active: dict[str, Execution] = {}
        self.finished: deque[Execution] = deque(maxlen=30)
        self.requests: dict[str, OperatorRequest] = {}
        # (key) -> list of execution start times, for chronic detection
        self._history: dict[str, list[float]] = {}
        self._last_end: dict[str, float] = {}
        self._human_only: set[str] = set()
        # After a run ends without success (aborted / operator cannot / verify
        # failed) the incident is NOT re-run automatically: the operator already
        # decided or the playbook already failed. Released by Retry, by the
        # incident resolving, or by the diagnosis changing.
        self._held: dict[str, str] = {}
        # One-shot operator "Run" for playbooks that never auto-start (advisory).
        self._manual_start: set[str] = set()
        self._draining_actions: list[ActionHandle] = []
        self._ids = itertools.count(1)
        self._request_session = uuid4().hex[:12]

    # ------------------------------------------------------------------ policy

    def blocked_reason(self, key: str, playbook: Playbook, now: float, variables: dict[str, str] | None = None) -> str | None:
        """Why a confirmed incident is not (yet) executed automatically, or None."""
        playbook = playbook.bind_components(self.env, variables or {"producer": key})
        if not self.enabled:
            return "execution disabled"
        if key in self.active:
            return "already running"
        if self.require_verified and not playbook.verified:
            return f"{playbook.id} not verified in {self.env.kind}: suggestion only"
        if not playbook.recovery:
            return "playbook has no recovery steps"
        if key in self._held:
            return f"held after {self._held[key]} (press Retry to run again)"
        if key in self._human_only:
            return "chronic: human only (use Retry on the Dashboard)"
        if playbook.severity == "advisory" and key not in self._manual_start:
            return "advisory: suggestion only (press Run to start, or Acknowledge)"
        cooldown = float(playbook.policy.get("cooldown_sec", DEFAULT_COOLDOWN_SEC))
        last = self._last_end.get(key)
        if last is not None and now - last < cooldown:
            return f"cooldown {cooldown - (now - last):.0f}s"
        retry_budget = int(playbook.policy.get("retry_budget", 0))
        chronic = playbook.policy.get("chronic") or {}
        window = float(chronic.get("window_sec", 3600.0))
        recent = [t for t in self._history.get(key, []) if now - t <= window]
        limit = int(chronic.get("count", max(3, retry_budget + 2)))
        if len(recent) >= limit:
            self._human_only.add(key)
            return f"chronic: {len(recent)} runs in {window:.0f}s"
        if risk_rank(playbook.max_action_risk) >= risk_rank("A2"):
            return self._actuator_lock_reason()
        return None

    def _actuator_lock_reason(self, exclude: Execution | None = None) -> str | None:
        if self.draining_handles():
            return "actuator lock held by a finishing backend action"
        for other in self.active.values():
            if (other is not exclude
                    and other.state in {RUNNING_ACTION, WAITING_SUCCESS, VERIFYING}
                    and risk_rank(other.playbook.max_action_risk) >= risk_rank("A2")):
                return f"actuator lock held by {other.playbook.id}"
        return None

    def draining_handles(self):
        self._draining_actions = [handle for handle in self._draining_actions if handle.poll()[0] == RUNNING]
        return list(self._draining_actions)

    def start(self, key: str, incident_id: str, playbook: Playbook, variables: dict[str, str], now: float) -> Execution:
        execution = Execution(
            id=f"exe-{next(self._ids)}",
            incident_key=key,
            incident_id=incident_id,
            playbook=playbook.bind_components(self.env, variables),
            variables=dict(variables),
            started_at=now,
            started_at_utc=utc_now(),
            state_since=now,
        )
        self.active[key] = execution
        self._manual_start.discard(key)
        self._history.setdefault(key, []).append(now)
        self._log(execution, "started", f"{playbook.id} attempt 1")
        self._emit("execution_started", execution)
        return execution

    def retry(self, key: str, now: float | None = None) -> None:
        """Operator override: clear hold/chronic/cooldown and allow one run (also for advisory)."""
        execution = self.active.get(key)
        # A waiting operator run otherwise occupies the key indefinitely: merely
        # clearing its retry budget never starts a new recovery attempt.
        if execution is not None and execution.state == WAITING_OPERATOR:
            self.abort(key, time.monotonic() if now is None else now,
                       "Operator requested a fresh recovery attempt")
        self._held.pop(key, None)
        self._manual_start.add(key)
        self._human_only.discard(key)
        self._last_end.pop(key, None)
        self._history.pop(key, None)

    def abort(self, key: str, now: float, reason: str = "aborted by operator") -> None:
        execution = self.active.get(key)
        if execution is not None:
            self._finish(execution, ABORTED, reason, now)
        elif reason == "aborted by operator":
            # An operator may abort an incident while its execution is deferred.
            self._held[key] = ABORTED

    def resolved_by_evidence(self, key: str, now: float) -> None:
        """The incident closed (root healthy): finish any run and forget holds."""
        execution = self.active.get(key)
        if execution is not None:
            self._finish(execution, SUCCEEDED, "Incident resolved by health evidence", now)
        self.release(key)

    def release(self, key: str) -> None:
        """Forget per-incident holds (incident gone or re-diagnosed)."""
        self._held.pop(key, None)
        self._manual_start.discard(key)
        for request_id in [r.id for r in self.requests.values() if r.incident_key == key and r.kind == "chronic"]:
            self.close_request(request_id)

    def open_chronic_request(self, key: str, playbook: Playbook, reason: str, now: float) -> None:
        if any(r.incident_key == key and r.kind == "chronic" for r in self.requests.values()):
            return
        self._open_request(
            key,
            playbook,
            kind="chronic",
            title=f"#{playbook.number} {playbook.id}: {reason}",
            text={
                "en": "This fault keeps coming back. Inspect the cause; press Retry to allow automatic recovery again.",
                "ja": "この故障が繰り返し発生しています。原因を確認し、自動復旧を再開する場合は Retry を押してください。",
            },
            buttons=["retry", "acknowledge"],
            now=now,
        )

    # ------------------------------------------------------------------ operator

    def operator_reply(self, request_id: str, reply: str, now: float, note: str = "") -> str:
        request = self.requests.get(request_id)
        if request is None:
            return f"unknown request {request_id}"
        reply = reply.lower()
        if reply not in OPERATOR_REPLIES and reply not in {"retry", "acknowledge"}:
            return f"unknown reply {reply}"
        if reply not in request.buttons:
            return f"reply {reply} is not offered for request {request_id}"
        request.reply = reply
        request.note = note
        return "ok"

    # ------------------------------------------------------------------ tick

    def _action_blockers(self, ex: Execution, step: dict, ctx: ProbeContext, now: float) -> list[str]:
        if not ctx.environment_ready and step.get("primitive") != "cancel_motion":
            return ["Environment unavailable; action not authorized"]
        if not 0 <= now - ctx.snapshot.received_at <= 5.0:
            return ["Health snapshot is stale; fresh evidence required"]
        blockers = []
        for name in ex.playbook.preconditions:
            result = self.registry.run_precondition(name, ctx)
            if result.outcome != PASS:
                blockers.append(f"{name}: {result.detail}")
        if risk_rank(str(step["risk"])) >= risk_rank("A2"):
            # Waiting for dependencies/approval must not prevent their recovery.
            # Recheck at dispatch so concurrent approvals cannot bypass the lock.
            lock_reason = self._actuator_lock_reason(exclude=ex)
            if lock_reason:
                blockers.append(lock_reason)
            for item in sorted(self.env.safety_items):
                result = self.registry.run_probe("health_item", ctx, {"item": item})
                if result.outcome != PASS:
                    blockers.append(f"safety not OK ({item}): {result.detail}")
        return blockers

    def tick(self, ctx: ProbeContext, now: float, blocked_keys: set[str] | None = None) -> None:
        for execution in list(self.active.values()):
            if execution.incident_key in (blocked_keys or ()):
                continue
            execution.checked_at = now
            for _ in range(10):  # advance through instantaneous transitions
                before = (execution.state, execution.step_index)
                self._advance(execution, ctx, now)
                if execution.terminal or (execution.state, execution.step_index) == before:
                    break

    def _advance(self, ex: Execution, ctx: ProbeContext, now: float) -> None:
        pb = ex.playbook
        if ex.state == WAITING_PRECONDITIONS:
            unmet = []
            for name in pb.preconditions:
                result = self.registry.run_precondition(name, ctx)
                if result.outcome != PASS:
                    unmet.append({"name": name, **result.to_dict()})
            if not unmet:
                self._close_request(ex)
                self._begin_step(ex, now)
                return
            ex.message = "Waiting for preconditions: " + ", ".join(u["name"] for u in unmet)
            if now - ex.state_since >= PRECONDITION_PROMPT_SEC and ex.request is None:
                ex.request = self._open_request(
                    ex.incident_key,
                    pb,
                    kind="precondition",
                    title=f"#{pb.number} {pb.id}: preconditions not met",
                    text={
                        "en": "Recovery is waiting for: " + ", ".join(u["name"] for u in unmet),
                        "ja": "復旧は次の条件を待っています: " + ", ".join(u["name"] for u in unmet),
                    },
                    buttons=["abort"],
                    now=now,
                )
            if ex.request is not None:
                ex.request.last_check = unmet
                if ex.request.reply == "abort":
                    self._finish(ex, ABORTED, "Aborted while waiting for preconditions", now)
            return

        if ex.state == VERIFYING:
            checks = self._check(pb.verify, ex, ctx)
            if all(check["outcome"] == PASS for check in checks):
                self._finish(ex, SUCCEEDED, "Verified", now)
                return
            deadline = max([float(v.get("within_sec", DEFAULT_VERIFY_TIMEOUT_SEC)) for v in pb.verify] or [0.0])
            if now - ex.state_since >= deadline:
                self._attempt_failed(ex, "Verification failed: " + _failed_text(checks), now)
            else:
                ex.message = "Verifying: " + _failed_text(checks)
            return

        if ex.step_index >= len(pb.recovery):
            self._set_state(ex, VERIFYING, now)
            return
        step = pb.recovery[ex.step_index]
        step_type = step["type"]

        if ex.state == WAITING_CONFIRM:
            reply = ex.request.reply if ex.request else None
            automatic = (reply is None and self.auto_approve and self.env.kind == "sim"
                         and ex.request is not None and ex.request.kind == "confirm_action"
                         and self.env.action_approval(str(step["risk"]), step.get("primitive")) == "confirm")
            if reply == "approve" or automatic:
                blocked = self._action_blockers(ex, step, ctx, now)
                if blocked:
                    ex.request.reply = None
                    ex.request.note = "Not executed; approve again after: " + "; ".join(blocked)
                    ex.message = ex.request.note
                    return
                if automatic:
                    self._log(ex, "action_auto_approved", f"Dashboard session permission: {step.get('primitive')} [{step['risk']}] request={ex.request.id}")
                    self._emit_cb("action_auto_approved", {"execution_id": ex.id,
                        "playbook_id": ex.playbook.id, "request_id": ex.request.id,
                        "primitive": step.get("primitive"), "risk": step["risk"],
                        "source": "dashboard_session_permission"})
                self._close_request(ex)
                self._start_action(ex, step, now)
            elif reply in {"reject", "abort"}:
                self._finish(ex, ABORTED, f"Operator rejected {step.get('primitive')}", now)
            return

        if ex.state == RUNNING_ACTION:
            state, detail = ex.handle.poll() if ex.handle else (ERROR, "no handle")
            if state == RUNNING:
                ex.message = f"Running {step.get('primitive')}: {detail}"
                return
            self._log(ex, "action_" + ("done" if state == DONE else "error"), f"{step.get('primitive')}: {detail}")
            if state == DONE and step.get("success_when"):
                self._set_state(ex, WAITING_SUCCESS, now)
                return
            if state == DONE and step.get("on_success") == "verify":
                self._set_state(ex, VERIFYING, now)
            elif state == DONE:
                self._next_step(ex, now)
            else:
                self._step_failed(ex, step, detail, now)
            return

        if ex.state == WAITING_SUCCESS:
            success_timeout = float(step.get("success_timeout_sec", DEFAULT_SUCCESS_TIMEOUT_SEC))
            if ex.playbook.id == "PB-LOC-LOST":
                success_timeout = max(success_timeout, float(self.env.actions.get("localization_success_timeout_sec", 0)))
            checks = self._check(step.get("success_when") or [], ex, ctx)
            ex.message = "Automatic action finished; waiting for verification: " + _failed_text(checks)
            if all(check["outcome"] == PASS for check in checks):
                self._log(ex, "step_succeeded", _summary_text(step))
                if step.get("on_success") == "verify":
                    self._set_state(ex, VERIFYING, now)
                else:
                    self._next_step(ex, now)
            elif now - ex.state_since >= success_timeout:
                self._step_failed(ex, step, "success_when not met: " + _failed_text(checks), now)
            return

        if ex.state == WAITING_OPERATOR:
            request = ex.request
            checks = self._check(request.done_when if request else [], ex, ctx)
            ex.message = "Waiting for operator / fresh evidence: " + _failed_text(checks)
            if request is not None:
                request.last_check = checks
            reply = request.reply if request else None
            if checks and all(check["outcome"] == PASS for check in checks):
                self._log(ex, "operator_step_done", "evidence confirmed")
                self._close_request(ex)
                self._next_step(ex, now)
            elif reply == "done":
                if checks:
                    # The click is not evidence: re-check and tell the operator.
                    request.reply = None
                    request.note = "Not detected yet: " + _failed_text(checks)
                else:
                    self._log(ex, "operator_step_done", "human attested (no machine check)")
                    self._close_request(ex)
                    self._next_step(ex, now)
            elif reply == "skip":
                ex.operator_skipped = True
                self._log(ex, "operator_step_skipped", "")
                self._close_request(ex)
                self._next_step(ex, now)
            elif reply == "cannot":
                self._finish(ex, ESCALATED, "Operator cannot perform the step", now)
            elif reply == "abort":
                self._finish(ex, ABORTED, "Aborted by operator", now)
            return

        if ex.state == WAITING_EVIDENCE:
            checks = self._check(step.get("until") or [], ex, ctx)
            if all(check["outcome"] == PASS for check in checks):
                self._next_step(ex, now)
            elif now - ex.state_since >= float(step.get("timeout_sec", DEFAULT_EVIDENCE_TIMEOUT_SEC)):
                self._step_failed(ex, step, "evidence wait timed out: " + _failed_text(checks), now)
            else:
                ex.message = "Waiting for evidence: " + _failed_text(checks)
            return

        # Fresh step (state is the generic "step pending" marker).
        if step_type == "action":
            primitive = str(step.get("primitive"))
            args = substitute(step.get("args") or {}, ex.variables)
            supported, why = self.backend.supports(primitive, args)
            if not supported:
                # Graceful degradation: ask the operator to do it by hand.
                self._log(ex, "action_unsupported", f"{primitive}: {why}")
                if step.get("on_fail") == "next":
                    self._next_step(ex, now)
                    return
                ex.request = self._open_request(
                    ex.incident_key,
                    pb,
                    kind="manual_fallback",
                    title=f"#{pb.number} {pb.id}: please do manually — {primitive} {_args_text(args)}",
                    text={
                        "en": f"Automatic {primitive} ({_args_text(args)}) is not available here: {why}. Please do it manually; it completes when the check below passes.",
                        "ja": f"自動 {primitive}（{_args_text(args)}）はこの環境では使えません: {why}。手動で実施してください（下のチェックが通ると完了）。",
                    },
                    buttons=["done", "cannot", "abort"],
                    now=now,
                    step=_step_summary(step, ex.variables),
                    done_when=substitute(pb.verify, ex.variables),
                )
                self._set_state(ex, WAITING_OPERATOR, now)
                return
            blocked = self._action_blockers(ex, step, ctx, now)
            if blocked:
                ex.message = f"Holding {primitive} [{step['risk']}]: " + "; ".join(blocked)
                return
            approval = self.env.action_approval(str(step["risk"]), primitive)
            if approval == "auto":
                self._start_action(ex, step, now)
            elif approval == "confirm":
                component = self.env.components.get(str(args.get("component", "")))
                start_only = primitive == "restart_component" and component and (component.restart or {}).get("start_if_absent")
                detail_en = ("Start the missing node only; an existing process will not be killed or duplicated. "
                             "This does not restore the whole ROS stack. " if start_only else "")
                detail_ja = ("停止中のノードのみ起動します。既存プロセスの終了・重複起動はしません。ROS 全体の復旧ではありません。 " if start_only else "")
                ex.request = self._open_request(
                    ex.incident_key,
                    pb,
                    kind="confirm_action",
                    title=f"#{pb.number} {pb.id}: confirm {primitive} [{step['risk']}]",
                    text={
                        "en": detail_en + f"Recovery wants to run {primitive} ({_args_text(args)}). Approve when it is safe.",
                        "ja": detail_ja + f"復旧のため {primitive}（{_args_text(args)}）を実行します。安全を確認して承認してください。",
                    },
                    buttons=["approve", "reject"],
                    now=now,
                    step=_step_summary(step, ex.variables),
                    done_when=substitute(pb.verify, ex.variables),
                    action_context={
                        "component": args.get("component"),
                        "command_template": (component.restart or {}).get("command", []) if component else [],
                        "preconditions": list(pb.preconditions),
                        "scope": (f"Existing nodes managed by {args.get('manager')}; RESET then STARTUP; no new Terminal"
                                  if primitive == "lifecycle_reset" else "Listed component/group only; not a full ROS restart"),
                        "exclusive_nodes": (component.restart or {}).get("exclusive_nodes", []) if component else [],
                        "node_check": self.registry.run_probe("node_alive", ctx, {"component": args["component"]}).to_dict()
                                      if "component" in args else {},
                        "launch_context": (self.env.actions.get("launch_context", {}) if primitive == "restart_component"
                                           else {"source": "Existing ROS service/action; see operation scope"}),
                        "opens_terminal": primitive == "restart_component" and bool(self.env.actions.get("restart_in_terminal")),
                    },
                )
                self._set_state(ex, WAITING_CONFIRM, now)
            else:
                self._finish(ex, ESCALATED, f"{primitive} [{step['risk']}] is {approval} in {self.env.kind}", now)
            return

        if step_type == "operator_request":
            text = substitute(pb.operator_text, ex.variables)
            if pb.id == "PB-LOC-LOST":
                text = {"en": "Automatic localization recovery did not verify or could not run. Waiting for a manual RViz 2D Pose Estimate (position and heading). Keep stationary while fresh scans accumulate; completion requires accepted GICP evidence.",
                        "ja": "自動復旧は検証未完了または実行不可です。RViz 2D Pose Estimate で位置と向きを設定し、停止したまま点群蓄積と GICP 検証を待ってください。"}
            ex.request = self._open_request(
                ex.incident_key,
                pb,
                kind="operator_action",
                title=f"#{pb.number} {pb.id}: {step.get('template') or 'operator action'}",
                text=text,
                buttons=["done", "cannot", "skip", "abort"],
                now=now,
                step=_step_summary(step, ex.variables),
                done_when=substitute(step.get("done_when") or [], ex.variables),
            )
            self._set_state(ex, WAITING_OPERATOR, now)
            return

        if step_type == "wait_evidence":
            self._set_state(ex, WAITING_EVIDENCE, now)

    # ------------------------------------------------------------------ helpers

    def _begin_step(self, ex: Execution, now: float) -> None:
        ex.step_index = 0
        self._set_state(ex, "STEP", now)

    def _next_step(self, ex: Execution, now: float) -> None:
        ex.step_index += 1
        ex.handle = None
        self._set_state(ex, "STEP", now)

    def _start_action(self, ex: Execution, step: dict[str, Any], now: float) -> None:
        primitive = str(step.get("primitive"))
        if primitive in {"publish_initialpose", "restart_component", "lifecycle_reset"}:
            ex.evidence_after = now
        args = substitute(step.get("args") or {}, ex.variables)
        try:
            ex.handle = self.backend.start(primitive, args)
        except Exception as exc:  # backend failure is a step failure, not a crash
            self._log(ex, "action_error", f"{primitive}: {exc!r}")
            self._step_failed(ex, step, repr(exc), now)
            return
        self._log(ex, "action_started", f"{primitive} {args} [{step['risk']}]")
        self._set_state(ex, RUNNING_ACTION, now)

    def _step_failed(self, ex: Execution, step: dict[str, Any], detail: str, now: float) -> None:
        if step.get("on_fail") == "next":
            self._log(ex, "step_failed_continue", f"{_summary_text(step)}: {detail}")
            self._next_step(ex, now)
            return
        self._attempt_failed(ex, f"Step {ex.step_index + 1} failed: {detail}", now)

    def _attempt_failed(self, ex: Execution, message: str, now: float) -> None:
        retry_budget = int(ex.playbook.policy.get("retry_budget", 0))
        if ex.operator_skipped:
            # Re-running would show the operator the step they just skipped.
            self._finish(ex, FAILED, f"{message} (after operator skip; not retried)", now)
            return
        if ex.attempt <= retry_budget:
            ex.attempt += 1
            self._log(ex, "retry", f"{message}; attempt {ex.attempt}")
            ex.step_index = 0
            ex.handle = None
            self._close_request(ex)
            self._set_state(ex, WAITING_PRECONDITIONS, now)
            return
        self._finish(ex, FAILED, message, now)

    def _finish(self, ex: Execution, state: str, message: str, now: float) -> None:
        if ex.handle is not None and ex.handle.poll()[0] == RUNNING:
            self._draining_actions.append(ex.handle)
        ex.state = state
        ex.state_since = now
        ex.message = message
        ex.finished_at_utc = utc_now()
        self._close_request(ex)
        self._log(ex, state.lower(), message)
        self.active.pop(ex.incident_key, None)
        self.finished.append(ex)
        self._last_end[ex.incident_key] = now
        if state != SUCCEEDED and not message.startswith(SUPERSEDED_PREFIX):
            self._held[ex.incident_key] = state
        self._emit("execution_" + state.lower(), ex)

    def _set_state(self, ex: Execution, state: str, now: float) -> None:
        ex.state = state
        ex.state_since = now
        ex.message = state.replace("_", " ").capitalize()

    def _check(self, specs: list[dict[str, Any]], ex: Execution, ctx: ProbeContext) -> list[dict[str, Any]]:
        results = []
        for spec in specs:
            args = substitute(spec.get("args") or {}, ex.variables)
            result = self.registry.run_probe(str(spec.get("probe")), ctx, args)
            if (ex.playbook.id == "PB-LOC-LOST" or ex.evidence_after is not None) and spec.get("probe") == "health_item" and result.outcome == PASS:
                entry = ctx.snapshot.item(str(args.get("item"))) or {}
                age = entry.get("receive_age_sec")
                if (ex.checked_at - ctx.snapshot.received_at > 4.0
                        or not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0
                        or (ex.evidence_after is not None and ctx.snapshot.received_at - age <= ex.evidence_after)):
                    result.outcome = "UNKNOWN"
                    result.detail = "Waiting for fresh evidence received after the recovery action"
            results.append({"probe": spec.get("probe"), "args": args, **result.to_dict()})
        return results

    def _open_request(
        self,
        key: str,
        pb: Playbook,
        *,
        kind: str,
        title: str,
        text: dict[str, str],
        buttons: list[str],
        now: float,
        step: dict[str, Any] | None = None,
        done_when: list[dict[str, Any]] | None = None,
        action_context: dict[str, Any] | None = None,
    ) -> OperatorRequest:
        request = OperatorRequest(
            id=f"op-{self._request_session}-{next(self._ids)}",
            kind=kind,
            incident_key=key,
            playbook_id=pb.id,
            playbook_number=pb.number,
            title=title,
            text=text,
            buttons=buttons,
            created_at_utc=utc_now(),
            created_at=now,
            done_when=done_when or [],
            step=step,
            action_context=action_context or {},
        )
        self.requests[request.id] = request
        self._emit_cb("operator_request_opened", request.to_dict(now))
        return request

    def close_request(self, request_id: str) -> None:
        request = self.requests.pop(request_id, None)
        if request is not None:
            self._emit_cb("operator_request_closed", {"id": request.id, "reply": request.reply})

    def _close_request(self, ex: Execution) -> None:
        if ex.request is not None:
            self.close_request(ex.request.id)
            ex.request = None

    def _log(self, ex: Execution, event: str, detail: str) -> None:
        ex.log.append({"t": utc_now(), "event": event, "step": ex.step_index, "detail": detail})

    def _emit(self, event_type: str, ex: Execution) -> None:
        self._emit_cb(
            event_type,
            {
                "execution_id": ex.id,
                "incident_key": ex.incident_key,
                "incident_id": ex.incident_id,
                "playbook_id": ex.playbook.id,
                "playbook_number": ex.playbook.number,
                "state": ex.state,
                "message": ex.message,
            },
        )

    def state(self, now: float) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "require_verified": self.require_verified,
            "active": [ex.to_dict(now) for ex in self.active.values()],
            "finished": [ex.to_dict(now) for ex in reversed(self.finished)],
            "operator_requests": [r.to_dict(now) for r in self.requests.values()],
            "human_only": sorted(self._human_only),
            "held": dict(self._held),
        }


def _args_text(args: dict[str, Any]) -> str:
    return " ".join(f"{key}={value}" for key, value in args.items())


def _failed_text(checks: list[dict[str, Any]]) -> str:
    failed = [c for c in checks if c["outcome"] != PASS]
    return "; ".join(f"{c['probe']}({c['detail']})" for c in failed) or "all checks pass"


def _summary_text(step: dict[str, Any]) -> str:
    return f"{step.get('type')} {step.get('primitive') or step.get('template') or ''}".strip()

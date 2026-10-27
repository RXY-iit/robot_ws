from __future__ import annotations

import json
import os
import time
import hashlib
import urllib.error
import urllib.request
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from robot_agent.session import session_dir, session_id


SENSITIVE_KEYS = ("api_key", "apikey", "authorization", "token", "secret", "password", "passwd")
EVIDENCE_TOKEN_KEYS = {"matched_tokens", "required_tokens", "input_tokens", "input_tokens_est", "prompt_tokens", "output_tokens", "total_tokens"}
REPORT_TOPIC = "/agent/human_report"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def redacted(value: Any) -> Any:
    if isinstance(value, dict):
        out: dict[str, Any] = {}
        for key, item in value.items():
            if str(key).lower() not in EVIDENCE_TOKEN_KEYS and any(word in str(key).lower() for word in SENSITIVE_KEYS):
                out[str(key)] = "[REDACTED]"
            else:
                out[str(key)] = redacted(item)
        return out
    if isinstance(value, list):
        return [redacted(item) for item in value]
    return value


def parse_json(data: str) -> Any | None:
    try:
        return json.loads(data)
    except json.JSONDecodeError:
        return None


def append_unique(items: list[Any], item: Any, *, key: str | None = None, cap: int = 80) -> None:
    if key and isinstance(item, dict):
        value = item.get(key)
        if value is not None and any(isinstance(existing, dict) and existing.get(key) == value for existing in items):
            return
    elif item in items:
        return
    items.append(item)
    if len(items) > cap:
        del items[: len(items) - cap]


def upsert_by_key(items: list[Any], item: dict[str, Any], key: str, *, cap: int = 80) -> None:
    value = item.get(key)
    if value is not None:
        for index, existing in enumerate(items):
            if isinstance(existing, dict) and existing.get(key) == value:
                items[index] = item
                return
    items.append(item)
    if len(items) > cap:
        del items[: len(items) - cap]


class HumanReportManager:
    """One immutable-on-completion mission record, also used as LLM evidence."""

    def __init__(self, workspace: Path | None = None) -> None:
        self.workspace = (workspace or Path.cwd()).resolve()
        self.root = session_dir(self.workspace) / "missions"
        self.root.mkdir(parents=True, exist_ok=True)
        self.current: dict[str, Any] | None = None
        self._last_public: dict[str, Any] | None = None
        self._closed_ids: set[str] = set()
        self._health_baseline: dict[str, Any] = {}
        self._last_api_input: str | None = None
        self._last_api_trigger = None
        self._last_recovery_signature = None
        self.api_enabled = os.environ.get("HUMAN_REPORT_API_ENABLED", "").strip().lower() in {"1", "true", "yes", "on"}
        self.api_url = os.environ.get("HUMAN_REPORT_API_URL", "").strip()
        self.api_key = os.environ.get("HUMAN_REPORT_API_KEY", "").strip()
        self.api_model = os.environ.get("HUMAN_REPORT_MODEL", "human-report-summary").strip()
        try:
            self.api_timeout_sec = float(os.environ.get("HUMAN_REPORT_TIMEOUT_SEC", "3.0"))
        except ValueError:
            self.api_timeout_sec = 3.0

    def on_topic(self, topic: str, data: str) -> dict[str, Any] | None:
        obj = parse_json(data)
        if topic == "/agent/mission_events" and isinstance(obj, dict) and obj.get("type") == "agent_heartbeat":
            return None
        if topic == "/agent/health_dashboard_state" and isinstance(obj, dict):
            self._health_baseline = self._compact_health_dashboard(obj)
        incoming_id = obj.get("mission_id") if isinstance(obj, dict) else None
        if topic == "/agent/recovery_response" and incoming_id and self.current and incoming_id != self.current.get("mission_id"):
            return None
        if incoming_id in self._closed_ids and (not self.current or incoming_id != self.current.get("mission_id")):
            return None
        if self.current and self.current.get("closed_at"):
            # ROS topics have independent ordering: a terminal event may precede
            # its richer state. Reconcile only that mission's compatible terminal state.
            status = str(obj.get("status") or "") if isinstance(obj, dict) else ""
            closed_status = self.current.get("status")
            compatible = status == closed_status or (closed_status == "FAILED" and status.startswith("FAILED"))
            if (topic == "/agent/mission_state" and incoming_id == self.current.get("mission_id")
                    and compatible and (status in {"DONE", "DONE_WITH_STUB", "CANCELLED"} or status.startswith("FAILED"))):
                if self.current.get("mission", {}).get("status") == status:
                    return None
                closed_at = self.current["closed_at"]
                self._on_mission_state(obj)
                self.current["closed_at"] = closed_at
                self.current["evidence_refs"]["selection"]["end"] = closed_at
                return self._commit(maybe_call_api=False)
            # Final state and event arrive on different topics. Accept the final
            # event once, but never let background health reopen this mission.
            final_event = (topic == "/agent/mission_events" and isinstance(obj, dict)
                           and incoming_id == self.current.get("mission_id")
                           and obj.get("type") in {"mission_done", "mission_done_with_stub", "mission_failed", "mission_cancelled"})
            if final_event:
                if obj in self.current["mission_events"]:
                    return None
                append_unique(self.current["mission_events"], redacted(obj), cap=2000)
                return self._commit(maybe_call_api=False)
            if not incoming_id or incoming_id == self.current.get("mission_id"):
                return None
        changed = False
        maybe_call_api = False

        if topic == "/agent/mission_events" and isinstance(obj, dict):
            changed, maybe_call_api = self._on_mission_event(obj)
        elif topic == "/agent/mission_state" and isinstance(obj, dict):
            changed = self._on_mission_state(obj)
            maybe_call_api = str(obj.get("status") or "") == "NEEDS_HUMAN"
        elif topic == "/agent/recovery_response" and isinstance(obj, dict):
            changed = self._on_recovery_response(obj)
            maybe_call_api = changed
        elif topic == "/agent/recovery_state" and isinstance(obj, dict):
            changed = self._on_recovery_state(obj)
        elif topic == "/agent/recovery_events" and isinstance(obj, dict):
            changed = self._on_recovery_event(obj)
        elif topic == "/agent/health_dashboard_state" and isinstance(obj, dict):
            changed = self._on_health_dashboard_state(obj)
        elif topic == "/agent/health_state" and isinstance(obj, dict):
            changed = self._on_health_state(obj)
        elif topic == "/agent/human_prompt" and data.strip():
            changed = self._on_human_prompt(data.strip())
            maybe_call_api = changed

        if not changed:
            return None
        trigger = json.dumps([self.current.get("status"), self.current.get("why_call_human"),
                              self.current.get("user_interactions")], sort_keys=True)
        maybe_call_api = maybe_call_api and self.current.get("status") == "NEEDS_HUMAN" and trigger != self._last_api_trigger
        if maybe_call_api:
            self._last_api_trigger = trigger
        return self._commit(maybe_call_api=maybe_call_api)

    def on_dashboard_command(self, kind: str, payload: Any) -> dict[str, Any] | None:
        if self.current is None or self.current.get("closed_at"):
            return None
        now = utc_now()
        entry = {"at": now, "kind": kind, "payload": redacted(payload)}
        append_unique(self.current["user_interactions"], entry, cap=80)
        if kind == "cancel":
            self.current["status"] = "CANCEL_REQUESTED"
        elif kind == "recovery_command" and isinstance(payload, dict):
            reply = payload.get("reply") or payload.get("type")
            if reply:
                self.current["current_situation"]["last_operator_choice"] = str(reply)
        elif kind == "command":
            # The new mission_received event owns this command, not the old report.
            self.current["user_interactions"].pop()
            return None
        return self._commit(maybe_call_api=kind in {"cancel", "recovery_command", "command"})

    def public_state(self) -> dict[str, Any] | None:
        return deepcopy(self._last_public)

    def _start(self, mission_id: str | None, text: str = "") -> None:
        mission = str(mission_id or f"manual-{int(time.time() * 1000)}")
        if not all(c.isalnum() or c in "-_" for c in mission):
            mission = hashlib.sha256(mission.encode()).hexdigest()[:24]
        now = utc_now()
        report_id = f"human-report-{mission}"
        path = self.root / mission / f"{report_id}.json"
        self.current = {
            "schema_version": 2,
            "session_id": session_id(),
            "scope": "mission" if mission_id else "system_incident",
            "closed_at": None,
            "report_id": report_id,
            "mission_id": mission_id,
            "created_at": now,
            "updated_at": now,
            "status": "ACTIVE",
            "json_path": str(path),
            "report_url": f"/report?path={str(path)}",
            "summary_line": "Mission report is collecting evidence.",
            "mission": {
                "user_text": text,
                "status": "RECEIVED",
                "intent": None,
                "current_step_index": None,
                "plan": [],
                "skill_results": [],
            },
            "why_call_human": [],
            "operator_actions": [],
            "current_situation": {},
            "blocking_components": [],
            "evidence": [],
            "recovery_history": {"responses": [], "events": [], "incidents": [], "operator_requests": [], "executions": []},
            "user_interactions": [],
            "mission_events": [],
            "health_summary": {},
            "health_at_start": deepcopy(self._health_baseline),
            "health_transitions": [],
            "evidence_refs": {
                "skill_events": str(self.root.parent / "agent" / "skill_events.jsonl"),
                "jev_calls": str(self.root.parent / "agent" / "jev_calls.jsonl"),
                "sim_health": str(self.root.parent / "sim_health.jsonl"),
                "selection": {"mission_id": mission_id, "start": now, "end": None},
            },
            "api_summary": {
                "enabled": self.api_enabled,
                "called": False,
                "status": "disabled" if not self.api_enabled else "pending",
            },
        }
        self._last_api_input = None
        self._last_api_trigger = None
        self._last_recovery_signature = None

    def _ensure(self, mission_id: str | None = None, text: str = "") -> None:
        if self.current is None:
            self._start(mission_id, text)
            return
        if mission_id and self.current.get("mission_id") != mission_id:
            if not self.current.get("closed_at"):
                self._close("SUPERSEDED")
                self._commit(maybe_call_api=False)
            self._start(mission_id, text)

    def _on_mission_event(self, event: dict[str, Any]) -> tuple[bool, bool]:
        event_type = str(event.get("type") or "")
        if event_type == "agent_heartbeat":
            return False, False
        mission_id = event.get("mission_id")
        payload = event.get("payload") if isinstance(event.get("payload"), dict) else {}
        if event_type == "mission_received":
            if self.current and self.current.get("mission_id") != mission_id and not self.current.get("closed_at"):
                self._close("SUPERSEDED")
                self._commit(maybe_call_api=False)
            self._ensure(str(mission_id) if mission_id else None, str(payload.get("text") or event.get("command") or ""))
            self.current["mission"]["user_text"] = str(payload.get("text") or event.get("command") or "")
        else:
            if not mission_id and self.current is None:
                return False, False
            self._ensure(str(mission_id) if mission_id else None)

        assert self.current is not None
        append_unique(self.current["mission_events"], redacted(event), cap=2000)
        if event_type == "mission_needs_human":
            self.current["status"] = "NEEDS_HUMAN"
            question = str(payload.get("question") or event.get("question") or "").strip()
            if question:
                append_unique(self.current["why_call_human"], question, cap=40)
                self.current["current_situation"]["human_question"] = question
            response = payload.get("recovery_response")
            if isinstance(response, dict):
                self._record_recovery_response(response)
            code = payload.get("code")
            if code:
                append_unique(self.current["evidence"], {"source": "mission_event", "code": code, "detail": question}, cap=80)
            return True, True
        if event_type == "recovery_response":
            response = payload if isinstance(payload, dict) else event
            self._record_recovery_response(response)
            return True, True
        if event_type in {"mission_done", "mission_done_with_stub", "mission_failed", "mission_cancelled"}:
            self._close(event_type.removeprefix("mission_").upper())
        return True, False

    def _close(self, status: str) -> None:
        self.current["status"] = status
        self.current["closed_at"] = utc_now()
        self.current["evidence_refs"]["selection"]["end"] = self.current["closed_at"]
        if self.current.get("mission_id"):
            self._closed_ids.add(self.current["mission_id"])

    def _on_mission_state(self, state: dict[str, Any]) -> bool:
        mission_id = state.get("mission_id")
        if not mission_id:
            return False
        self._ensure(str(mission_id) if mission_id else None, str(state.get("user_text") or ""))
        assert self.current is not None
        mission = self.current["mission"]
        mission.update(
            {
                "user_text": state.get("user_text"),
                "status": state.get("status"),
                "outcome": state.get("outcome"),
                "intent": redacted(state.get("intent")),
                "final_message": state.get("final_message"),
                "question_for_human": state.get("question_for_human"),
                "human_wait": redacted(state.get("human_wait")),
                "current_step_index": state.get("current_step_index"),
                "agent_mode": state.get("agent_mode"),
                "parser_trace": redacted(state.get("parser_trace")),
                "jev_trace_total": redacted((state.get("jev_trace") or {}).get("total") if isinstance(state.get("jev_trace"), dict) else None),
                "plan": self._compact_plan(state.get("plan")),
                "tasks": redacted(state.get("tasks") or []),
                "skill_results": self._compact_skill_results(state.get("skill_results")),
            }
        )
        status = str(state.get("status") or "")
        if status:
            self.current["status"] = status
        if status in {"DONE", "DONE_WITH_STUB", "FAILED", "FAILED_SKILL", "CANCELLED", "REJECTED"}:
            self._close(status)
        question = str(state.get("question_for_human") or "").strip()
        if question:
            append_unique(self.current["why_call_human"], question, cap=40)
            self.current["current_situation"]["human_question"] = question
        return True

    def _on_recovery_response(self, response: dict[str, Any]) -> bool:
        mission_id = response.get("mission_id")
        self._ensure(str(mission_id) if mission_id else None)
        assert self.current is not None
        self._record_recovery_response(response)
        return True

    def _on_recovery_state(self, state: dict[str, Any]) -> bool:
        execution = state.get("execution") if isinstance(state.get("execution"), dict) else {}
        operator_requests = execution.get("operator_requests") if isinstance(execution.get("operator_requests"), list) else []
        incidents = state.get("incidents") if isinstance(state.get("incidents"), list) else []
        if self.current is None and (operator_requests or incidents):
            self._start(None, "system recovery / operator request")
            assert self.current is not None
            self.current["status"] = "NEEDS_HUMAN" if operator_requests else "RECOVERY_INCIDENT"
        if self.current is None:
            return False
        mission_scoped = bool(self.current.get("mission_id"))
        if mission_scoped:
            request_ids = {e.get("payload", {}).get("request_id") for e in self.current["mission_events"]
                           if e.get("type") == "recovery_request"}
            request_ids.update(r.get("request_id") for r in self.current["recovery_history"]["responses"])
            request_ids.discard(None)
            related_keys = {i.get("key") for i in incidents if request_ids.intersection(i.get("requests") or [])}
            related_keys.update(i.get("key") for i in self.current["recovery_history"]["incidents"])
            self.current["current_situation"]["background_incidents"] = [
                {"key": i.get("key"), "playbook_id": i.get("playbook_id"), "severity": i.get("severity")}
                for i in incidents if i.get("key") not in related_keys]
            incidents = [i for i in incidents if i.get("key") in related_keys]
            operator_requests = [r for r in operator_requests if r.get("incident_key") in related_keys]
            execution = {key: [run for run in execution.get(key, []) if run.get("incident_key") in related_keys]
                         for key in ("active", "finished")}
        signature = json.dumps({
            "incidents": [(i.get("id"), i.get("status"), i.get("playbook_id")) for i in incidents],
            "requests": [(r.get("id"), r.get("note")) for r in operator_requests],
            "runs": [(r.get("id"), r.get("state"), r.get("step_index"), r.get("log"))
                     for r in (execution.get("active") or []) + (execution.get("finished") or [])],
            "closed": [(i.get("id"), i.get("status")) for i in state.get("recently_closed", [])],
            "background": self.current["current_situation"].get("background_incidents"),
        }, sort_keys=True)
        if signature == self._last_recovery_signature:
            return False
        self._last_recovery_signature = signature
        recovery = self.current["recovery_history"]
        self.current["current_situation"]["recovery"] = redacted({
            "incidents": incidents,
            "operator_requests": operator_requests,
            "generated_at": state.get("generated_at"),
        })
        active_request_ids = {r.get("id") for r in operator_requests}
        for previous in recovery["operator_requests"]:
            if previous.get("id") not in active_request_ids:
                previous["report_state"] = "CLOSED"
        for request in operator_requests:
            upsert_by_key(recovery["operator_requests"], redacted({**request, "report_state": "OPEN"}), "id", cap=80)
        # Global snapshots include finished runs from earlier missions.
        started = self.current["created_at"]
        for run in (execution.get("active") or []) + (execution.get("finished") or []):
            if not run.get("finished_at") or run["finished_at"] >= started:
                upsert_by_key(recovery["executions"], redacted(run), "id", cap=80)
        for incident in incidents:
            if isinstance(incident, dict):
                self._record_incident(incident)
        for incident in state.get("recently_closed") or []:
            if isinstance(incident, dict) and (not mission_scoped or incident.get("key") in related_keys) and ((incident.get("resolved_at") or "") >= started
                    or any(old.get("id") == incident.get("id") for old in recovery["incidents"])):
                self._record_incident(incident)
        self.current["blocking_components"] = redacted([
            {"component": i.get("key"), "playbook_id": i.get("playbook_id"),
             "severity": i.get("severity"), "roots": i.get("roots", [])}
            for i in incidents if i.get("severity") == "blocking" and not i.get("recovering")
        ])
        if operator_requests and (not mission_scoped or self.current["mission"].get("status") in {"RECOVERING", "PAUSED_HEALTH", "NEEDS_HUMAN"}):
            self.current["status"] = "NEEDS_HUMAN"
        elif not self.current.get("mission_id"):
            self.current["status"] = "RECOVERY_INCIDENT" if incidents else "RESOLVED"
        for request in operator_requests:
            if isinstance(request, dict):
                text = request.get("text") if isinstance(request.get("text"), dict) else {}
                append_unique(self.current["why_call_human"],
                              text.get("ja") or text.get("en") or request.get("title") or "Operator action requested",
                              cap=80)
                append_unique(
                    self.current["operator_actions"],
                    {
                        "source": "recovery_operator_request",
                        "request_id": request.get("id"),
                        "incident_key": request.get("incident_key"),
                        "playbook_id": request.get("playbook_id"),
                        "title": request.get("title"),
                        "text": text.get("ja") or text.get("en") or request.get("note") or "",
                        "buttons": request.get("buttons"),
                    },
                    key="request_id",
                    cap=80,
                )
        return True

    def _on_recovery_event(self, event: dict[str, Any]) -> bool:
        event_type = str(event.get("type") or event.get("event") or "")
        if self.current is None and "operator_request" in event_type:
            self._start(None, "system recovery / operator request")
            assert self.current is not None
            self.current["status"] = "NEEDS_HUMAN"
            append_unique(
                self.current["why_call_human"],
                "Recovery Agent opened an operator request before a mission is active.",
                cap=40,
            )
        if self.current is None:
            return False
        if self.current.get("mission_id"):
            keys = {i.get("key") for i in self.current["recovery_history"]["incidents"]}
            ids = {r.get("id") for r in self.current["recovery_history"]["operator_requests"]}
            if event.get("incident_key", event.get("key")) not in keys and event.get("id") not in ids:
                return False
        append_unique(self.current["recovery_history"]["events"], redacted(event), cap=120)
        return True

    def _on_health_dashboard_state(self, state: dict[str, Any]) -> bool:
        if self.current is None:
            return False
        compact = self._compact_health_dashboard(state)
        previous = self.current["health_summary"].get("dashboard", {})
        signature = lambda s: [(i["name"], i["status"]) for i in s.get("unhealthy", [])]
        if signature(compact) == signature(previous) and previous:
            # Refresh current evidence in memory, but do not write a heartbeat.
            self.current["health_summary"]["dashboard"] = compact
            return False
        old_items = {i["name"]: i for i in previous.get("unhealthy", [])}
        new_items = {i["name"]: i for i in compact["unhealthy"]}
        changed = [i for name, i in new_items.items() if old_items.get(name, {}).get("status") != i["status"]]
        cleared = [name for name in old_items if name not in new_items]
        append_unique(self.current["health_transitions"], {
            "at": utc_now(), "kind": "health_status_delta", "unhealthy": changed,
            "cleared": cleared,
        }, cap=120)
        self.current["health_summary"]["dashboard"] = compact
        return True

    def _on_health_state(self, state: dict[str, Any]) -> bool:
        if self.current is None:
            return False
        components = state.get("components") if isinstance(state.get("components"), dict) else {}
        compact = {}
        for name, value in components.items():
            if not isinstance(value, dict):
                continue
            summary = str(value.get("summary") or "")
            status = str(value.get("level") or value.get("status") or value.get("health") or "")
            if status.upper() not in {"", "OK", "IDLE", "OBSERVED"}:
                compact[name] = {"status": status, "summary": summary}
        if self.current["health_summary"].get("legacy") == redacted(compact):
            return False
        self.current["health_summary"]["legacy"] = redacted(compact)
        return True

    def _on_human_prompt(self, prompt: str) -> bool:
        self._ensure(None)
        assert self.current is not None
        append_unique(self.current["why_call_human"], prompt, cap=40)
        self.current["current_situation"]["human_prompt"] = prompt
        self.current["status"] = "NEEDS_HUMAN"
        return True

    def _record_recovery_response(self, response: dict[str, Any]) -> None:
        assert self.current is not None
        compact = {
            "request_id": response.get("request_id"),
            "status": response.get("status"),
            "advice": response.get("advice"),
            "summary": response.get("summary"),
            "final": response.get("final"),
            "root_cause": redacted(response.get("root_cause")),
            "safety_blocked": response.get("safety_blocked"),
        }
        transitions = self.current["recovery_history"].setdefault("response_transitions", [])
        previous = next((r for r in reversed(transitions) if r.get("request_id") == compact["request_id"]), None)
        if previous is None or {k: previous.get(k) for k in compact} != compact:
            transitions.append({**compact, "recorded_at": utc_now()})
            del transitions[:-120]
        upsert_by_key(self.current["recovery_history"]["responses"], compact, "request_id", cap=60)
        status = str(response.get("status") or "").strip()
        if status:
            self.current["status"] = "RECOVERING" if status == "IN_PROGRESS" else status
        summary = str(response.get("summary") or "").strip()
        if summary and status in {"NEEDS_HUMAN", "FAILED"}:
            append_unique(self.current["why_call_human"], summary, cap=40)
        for incident in response.get("incidents") or []:
            if isinstance(incident, dict):
                self._record_incident(incident)

    def _record_incident(self, incident: dict[str, Any]) -> None:
        assert self.current is not None
        roots = incident.get("roots") if isinstance(incident.get("roots"), list) else []
        compact = {
            "id": incident.get("id"),
            "key": incident.get("key"),
            "status": incident.get("status"),
            "playbook_id": incident.get("playbook_id"),
            "playbook_no": incident.get("playbook_no"),
            "severity": incident.get("severity"),
            "reason": incident.get("reason"),
            "execution_state": incident.get("execution_state"),
            "roots": redacted(roots),
        }
        upsert_by_key(self.current["recovery_history"]["incidents"], compact, "id", cap=80)
        component = {
            "component": incident.get("key"),
            "playbook_id": incident.get("playbook_id"),
            "severity": incident.get("severity"),
            "roots": redacted(roots),
        }
        if incident.get("severity") == "blocking" and incident.get("status") not in {"RESOLVED", "CLOSED", "ABORTED"}:
            upsert_by_key(self.current["blocking_components"], component, "component", cap=60)
        else:
            self.current["blocking_components"] = [c for c in self.current["blocking_components"]
                                                  if c.get("component") != component["component"]]
        for root in roots:
            if isinstance(root, dict):
                append_unique(
                    self.current["evidence"],
                    {
                        "source": "recovery_root",
                        "component": incident.get("key"),
                        "item": root.get("item"),
                        "status": root.get("status"),
                        "detail": root.get("summary"),
                    },
                    cap=120,
                )
        plan = incident.get("plan") if isinstance(incident.get("plan"), dict) else {}
        operator_text = plan.get("operator_text") if isinstance(plan.get("operator_text"), dict) else {}
        text = operator_text.get("ja") or operator_text.get("en")
        if text:
            append_unique(
                self.current["operator_actions"],
                {
                    "source": "recovery_playbook",
                    "incident_key": incident.get("key"),
                    "playbook_id": incident.get("playbook_id"),
                    "text": text,
                },
                cap=80,
            )

    def _commit(self, *, maybe_call_api: bool) -> dict[str, Any] | None:
        if self.current is None:
            return None
        self.current["updated_at"] = utc_now()
        self.current["summary_line"] = self._summary_line()
        if maybe_call_api:
            self._maybe_call_api_summary()
        path = Path(self.current["json_path"])
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(redacted(self.current), ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(path)
        self._last_public = {
            "report_id": self.current["report_id"],
            "mission_id": self.current.get("mission_id"),
            "status": self.current.get("status"),
            "updated_at": self.current.get("updated_at"),
            "summary_line": self.current.get("summary_line"),
            "json_path": self.current.get("json_path"),
            "report_url": self.current.get("report_url"),
            "why_count": len(self.current.get("why_call_human") or []),
            "operator_action_count": len(self.current.get("operator_actions") or []),
            "api_summary": redacted(self.current.get("api_summary")),
        }
        return deepcopy(self._last_public)

    def _summary_line(self) -> str:
        assert self.current is not None
        why = [str(item) for item in self.current.get("why_call_human") or [] if item]
        status = str(self.current.get("status") or "ACTIVE")
        if status == "RESOLVED" and not self.current.get("mission_id"):
            return "RESOLVED: No current recovery incidents; previous requests remain in history."
        recovery = self.current.get("current_situation", {}).get("recovery", {})
        if not self.current.get("mission_id") and recovery:
            return (f"{status}: {len(recovery.get('incidents', []))} current incident(s), "
                    f"{len(recovery.get('operator_requests', []))} pending operator request(s).")
        if why:
            return f"{status}: {why[-1]}"
        mission = self.current.get("mission") if isinstance(self.current.get("mission"), dict) else {}
        final = str(mission.get("final_message") or mission.get("question_for_human") or "").strip()
        if final:
            return f"{status}: {final}"
        text = str(mission.get("user_text") or "").strip()
        return f"{status}: {text or 'Mission report is collecting evidence.'}"

    def _maybe_call_api_summary(self) -> None:
        assert self.current is not None
        api = self.current["api_summary"]
        if not self.api_enabled:
            api.update({"enabled": False, "called": False, "status": "disabled"})
            return
        if not self.api_url:
            api.update({"enabled": True, "called": False, "status": "missing_url"})
            return
        fingerprint = json.dumps(self._api_report_view(), sort_keys=True, ensure_ascii=False)
        if self._last_api_input == fingerprint:
            return
        self._last_api_input = fingerprint
        request_body = {
            "model": self.api_model,
            "task": "Summarize why this robot mission needs human help and what the operator should do next. Use only the provided facts.",
            "report": self._api_report_view(),
        }
        api["request_body_redacted"] = redacted(request_body)
        payload = json.dumps(request_body, ensure_ascii=False).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"
        started = time.monotonic()
        try:
            req = urllib.request.Request(self.api_url, data=payload, headers=headers, method="POST")
            with urllib.request.urlopen(req, timeout=self.api_timeout_sec) as resp:
                raw = resp.read(200_000).decode("utf-8", errors="replace")
            try:
                parsed = json.loads(raw)
            except json.JSONDecodeError:
                parsed = {"text": raw}
            api.update(
                {
                    "enabled": True,
                    "called": True,
                    "status": "ok",
                    "latency_ms": round((time.monotonic() - started) * 1000.0, 1),
                    "source_updated_at": self.current.get("updated_at"),
                    "response": redacted(parsed),
                }
            )
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            api.update(
                {
                    "enabled": True,
                    "called": True,
                    "status": "error",
                    "latency_ms": round((time.monotonic() - started) * 1000.0, 1),
                    "error": str(exc),
                }
            )

    def _api_report_view(self) -> dict[str, Any]:
        assert self.current is not None
        return redacted(
            {
                "mission": self.current.get("mission"),
                "why_call_human": self.current.get("why_call_human"),
                "current_situation": self.current.get("current_situation"),
                "blocking_components": self.current.get("blocking_components"),
                "operator_actions": self.current.get("operator_actions"),
                "recovery_history": self.current.get("recovery_history"),
                "user_interactions": self.current.get("user_interactions"),
                "health_summary": self.current.get("health_summary"),
                "health_at_start": self.current.get("health_at_start"),
                "health_transitions": self.current.get("health_transitions"),
                "evidence_refs": self.current.get("evidence_refs"),
            }
        )

    @staticmethod
    def _compact_plan(plan: Any) -> list[dict[str, Any]]:
        if not isinstance(plan, list):
            return []
        out = []
        for index, call in enumerate(plan):
            if not isinstance(call, dict):
                continue
            out.append(
                {
                    "index": index,
                    "name": call.get("name"),
                    "kind": call.get("kind", "skill"),
                    "task_id": call.get("task_id"),
                    "step_id": call.get("step_id"),
                    "owner": call.get("owner"),
                    "payload": redacted(call.get("payload")),
                    "reason": call.get("reason"),
                    "catalog": redacted(call.get("catalog")),
                }
            )
        return out

    @staticmethod
    def _compact_skill_results(results: Any) -> list[dict[str, Any]]:
        if not isinstance(results, list):
            return []
        out = []
        for result in results:
            if not isinstance(result, dict):
                continue
            out.append(
                {
                    "step_index": result.get("step_index"),
                    "task_id": result.get("task_id"),
                    "step_id": result.get("step_id"),
                    "owner": result.get("owner"),
                    "skill_name": result.get("skill_name"),
                    "success": result.get("success"),
                    "continued_after_failure": result.get("continued_after_failure", False),
                    "code": result.get("code"),
                    "message": result.get("message"),
                    "metrics": redacted(result.get("metrics")),
                    "artifacts": result.get("artifacts"),
                }
            )
        return out

    @staticmethod
    def _compact_health_dashboard(state: dict[str, Any]) -> dict[str, Any]:
        unhealthy: list[dict[str, Any]] = []
        relevant = {}
        evidence_ids = {"gicp_score", "gicp_pose", "localization_consistency", "fast_lio_odom",
                        "lidar_device_status", "sim_clock", "sim_rtf", "camera_color_diagnostics",
                        "robot_mode", "software_safety", "perception_runtime"}

        def visit(obj: Any, path: str = "") -> None:
            if len(unhealthy) >= 80:
                return
            if isinstance(obj, dict):
                status = obj.get("status") or obj.get("health") or obj.get("state")
                name = obj.get("id") or obj.get("name") or path.rstrip(".")
                summary = obj.get("summary") or obj.get("message") or obj.get("detail")
                if isinstance(status, str) and status.upper() in {"BLOCKED", "ERROR", "FAULT", "STALE", "DEGRADED", "WARN", "UNKNOWN", "NOT_OBSERVED"}:
                    unhealthy.append({"path": path.rstrip("."), "name": name, "status": status, "summary": summary})
                for key, value in obj.items():
                    visit(value, f"{path}{key}.")
            elif isinstance(obj, list):
                for index, value in enumerate(obj[:200]):
                    visit(value, f"{path}{index}.")

        # Nested device payloads contain workflow states, not health verdicts.
        for group in ("summaries", "capabilities", "domains"):
            for index, entry in enumerate(state.get(group) or []):
                visit({k: v for k, v in entry.items() if k not in {"items", "value"}}, f"{group}.{index}.")
                for j, item in enumerate(entry.get("items") or []):
                    visit({k: v for k, v in item.items() if k != "value"}, f"{group}.{index}.items.{j}.")
                    if item.get("id") in evidence_ids:
                        compact_item = {k: item.get(k) for k in ("id", "status", "summary", "source", "source_age_sec", "age_sec")}
                        value = item.get("value")
                        if isinstance(value, (int, float, bool)):
                            compact_item["value"] = value
                        elif isinstance(value, dict):
                            compact_item["value"] = {k: v for k, v in list(value.items())[:24]
                                                     if isinstance(v, (int, float, bool)) or v is None}
                        relevant[item["id"]] = compact_item
        return {
            "generated_at": state.get("generated_at"),
            "sequence": state.get("sequence"),
            "unhealthy": redacted(unhealthy[:80]),
            "relevant_evidence": redacted(list(relevant.values())),
        }

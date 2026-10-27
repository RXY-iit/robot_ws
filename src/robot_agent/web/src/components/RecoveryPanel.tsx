import type { RecoveryExecution, RecoveryIncident, RecoveryOperatorRequest, RecoveryState } from "../types";
import { useEffect, useMemo, useState } from "react";

interface Props {
  recovery: RecoveryState | null;
  sendCommand: (command: Record<string, unknown>) => void;
}

const PRIORITY_LABEL = ["P0 safety", "P1 mission", "P2 capability", "P3 other"];
const NUMBERING_LEGEND = "Procedure IDs (legacy): 1x Safety · 2x Platform · 3x Drivers · 4x Sensors · 5x ROS · 6x Capabilities";
const HANDLING_LABEL: Record<string, string> = {
  known_recovery: "Known issue / 既知の問題",
  hardware_intervention: "Hardware inspection / ハードウェア確認",
  safety_event: "Safety event / 安全状態",
  unknown_investigation: "Investigation required / 調査待ち",
};
const BUTTON_LABEL: Record<string, string> = {
  done: "Done / 完了",
  cannot: "Cannot / できない",
  skip: "Skip",
  abort: "Abort",
  approve: "Approve / 承認",
  reject: "Reject / 拒否",
  retry: "Retry",
  acknowledge: "Acknowledge / 承知",
};

function severityClass(incident: RecoveryIncident): string {
  if (incident.acknowledged) return "unknown";
  if (incident.severity === "blocking") return "bad";
  if (incident.severity === "advisory") return "unknown";
  return "warn";
}

function playbookLabel(id: string | null, number: number | null, key: string): string {
  if (id) return number != null ? `#${number} ${id}` : id;
  return `Unmatched → R3 (${key})`;
}

function OperatorRequestCard({
  request,
  sendCommand,
  incident,
}: {
  request: RecoveryOperatorRequest;
  sendCommand: Props["sendCommand"];
  incident?: RecoveryIncident;
}) {
  const text = request.text?.ja || request.text?.en || "";
  const failing = request.last_check.filter((check) => check.outcome !== "PASS");
  return (
    <div className="operator-request">
      <strong>{incident?.handling_class && incident.handling_class !== "known_recovery"
        ? `${HANDLING_LABEL[incident.handling_class]}: ${incident.title ?? incident.key}` : request.title}</strong>
      {text && <p>{text}</p>}
      {request.step?.risk && (
        <small>
          step: {request.step.primitive || request.step.template} [{request.step.risk}]
          {request.step.args && Object.keys(request.step.args).length > 0 && ` ${JSON.stringify(request.step.args)}`}
        </small>
      )}
      {failing.length > 0 && (
        <small className="operator-request-check">
          Waiting for: {failing.map((check) => check.name || check.detail).join("; ")}
        </small>
      )}
      {request.note && <small className="operator-request-note">{request.note}</small>}
      {request.kind === "confirm_action" && <details>
        <summary>Action scope and verification / 操作範囲と検証</summary>
        <p>{request.action_context?.scope}</p>
        <p>{request.action_context?.node_check?.detail}</p>
        {request.action_context?.opens_terminal && <p>Opens a new Terminal / 新しい Terminal で起動</p>}
        <p>Configuration: {String(request.action_context?.launch_context?.source ?? "Recovery profile")}</p>
        <p>Preconditions: {request.action_context?.preconditions?.join(", ") || "See safety checks"}</p>
        <p style={{ overflowWrap: "anywhere" }}>Command: {request.action_context?.command_template?.join(" ") || request.step?.primitive}</p>
        <p>Verify: {request.done_when?.map(check => `${check.probe} ${JSON.stringify(check.args ?? {})}`).join("; ")}</p>
      </details>}
      <div className="operator-request-buttons">
        {request.buttons.map((button) => (
          <button
            key={button}
            type="button"
            className={button === "abort" || button === "reject" || button === "cannot" ? "secondary" : ""}
            onClick={() => sendCommand({ type: "operator_reply", request_id: request.id, reply: button })}
          >
            {BUTTON_LABEL[button] ?? button}
          </button>
        ))}
      </div>
      <small>waiting {Math.round(request.waiting_sec)}s · {request.kind}</small>
    </div>
  );
}

function ExecutionLine({ execution }: { execution: RecoveryExecution }) {
  const step = execution.current_step;
  return (
    <small className="execution-line">
      ▶ {execution.state} · step {Math.min(execution.step_index + 1, execution.step_count)}/{execution.step_count}
      {step && ` ${step.primitive || step.template || step.type} [${step.risk}]`} · attempt {execution.attempt}
      {execution.message && ` — ${execution.message}`}
    </small>
  );
}

function IncidentCard({
  incident,
  execution,
  sendCommand,
}: {
  incident: RecoveryIncident;
  execution: RecoveryExecution | undefined;
  sendCommand: Props["sendCommand"];
}) {
  const plan = incident.plan;
  const operatorText = plan?.operator_text?.ja || plan?.operator_text?.en;
  const cls = severityClass(incident);
  const canIgnore = incident.severity !== "blocking" && incident.priority !== 0 &&
    incident.roots.every((root) => !["ERROR", "BLOCKED", "STALE", "UNKNOWN", "NOT_OBSERVED"].includes(root.status ?? "UNKNOWN"));
  return (
    <div className={`capability-card ${cls}`}>
      <strong>
        <span>{incident.handling_class && incident.handling_class !== "known_recovery"
          ? `${HANDLING_LABEL[incident.handling_class]}: ${incident.title ?? incident.key}`
          : playbookLabel(incident.playbook_id, incident.playbook_no, incident.key)}</span>
        <span className={`health-status ${cls}`}>
          {incident.acknowledged ? "ACKNOWLEDGED" : incident.severity.toUpperCase()}
        </span>
      </strong>
      <p>
        {incident.roots.map((root) => `${root.item}=${root.status}`).join(", ") || incident.root_items.join(", ")}
      </p>
      {incident.roots.filter(root => root.summary).map(root => <div className="recovery-root-evidence" key={root.item}>
        <strong>{root.item}</strong><div>{root.summary}</div>
      </div>)}
      <small>
        {HANDLING_LABEL[incident.handling_class ?? "unknown_investigation"]} ·{" "}
        {incident.status} · {PRIORITY_LABEL[incident.priority] ?? `P${incident.priority}`} · layer {incident.layer}
        {incident.consequences.length > 0 && ` · +${incident.consequences.length} downstream`}
        {incident.uncertain && " · partial evidence"}
        {incident.reopen_count > 0 && ` · reopened ×${incident.reopen_count}`}
        {incident.flapping && " · FLAPPING"}
        {incident.recovering && " · recovering (hysteresis)"}
      </small>
      {incident.handling_class === "unknown_investigation" && <small>Investigator is not connected. Evidence retained; no speculative restart.</small>}
      {execution && <ExecutionLine execution={execution} />}
      {!execution && incident.execution_blocked && (
        <small className="execution-line">auto-recovery: {incident.execution_blocked}</small>
      )}
      {!execution && incident.last_execution && <small className="execution-line">
        Last run: {incident.last_execution.state} · {incident.last_execution.message}
      </small>}
      {incident.acknowledged?.note && <small>ack note: {incident.acknowledged.note}</small>}
      {!incident.acknowledged && operatorText && <p>{operatorText}</p>}
      <div className="operator-request-buttons">
        {incident.acknowledged ? (
          <button type="button" className="secondary" onClick={() => sendCommand({ type: "unacknowledge", incident_key: incident.key })}>
            Un-acknowledge
          </button>
        ) : canIgnore ? (
          <button
            type="button"
            className="secondary"
            onClick={() => {
              sendCommand({ type: "ignore", incident_key: incident.key, note: "Temporarily ignored warning" });
            }}
          >
            Snooze warning
          </button>
        ) : null}
        {execution ? (
          <button type="button" className="secondary" onClick={() => sendCommand({ type: "abort", incident_key: incident.key })}>
            Abort recovery
          </button>
        ) : (
          incident.playbook_id && incident.handling_class !== "unknown_investigation" && (
            <button type="button" className="secondary" onClick={() => sendCommand({ type: "retry", incident_key: incident.key })}>
              {incident.execution_blocked?.startsWith("advisory") ? "Run / 実行" : "Retry / 再試行"}
            </button>
          )
        )}
      </div>
      <details>
        <summary>Diagnosis path / plan</summary>
        <small>{incident.reason}</small>
        <ul>
          {incident.diagnosis_path.upstream_checks.map((check, index) => (
            <li key={`u${index}`}>
              upstream {check.upstream} ({check.layer}) = {check.status ?? "n/a"} → {check.result}
            </li>
          ))}
          {incident.diagnosis_path.playbook_steps.map((step, index) => (
            <li key={`s${index}`}>
              {step.playbook}/{step.step}: {step.probe} → {step.outcome} ({step.result}) — {step.detail}
            </li>
          ))}
        </ul>
        {plan && (
          <>
            <small>
              Plan; preconditions {plan.preconditions_met ? "met" : "not met"}
            </small>
            <ul>
              {plan.preconditions.map((pre) => (
                <li key={pre.name}>
                  pre {pre.name}: {pre.outcome} — {pre.detail}
                </li>
              ))}
              {plan.steps.map((step, index) => (
                <li key={`p${index}`}>
                  {index + 1}. {step.type} {step.primitive || step.template || ""} [{step.risk} / {step.approval}]
                  {Object.keys(step.args || {}).length > 0 && ` ${JSON.stringify(step.args)}`}
                </li>
              ))}
            </ul>
          </>
        )}
        {execution && execution.log.length > 0 && (
          <ul>
            {execution.log.map((entry, index) => (
              <li key={`l${index}`}>
                {entry.t.slice(11, 19)} {entry.event} — {entry.detail}
              </li>
            ))}
          </ul>
        )}
        {incident.consequences.length > 0 && <small>Downstream: {incident.consequences.join(", ")}</small>}
      </details>
    </div>
  );
}

export function RecoveryPanel({ recovery, sendCommand }: Props) {
  const [now, setNow] = useState(Date.now);
  const receivedAt = useMemo(() => Date.now(), [recovery]);
  useEffect(() => {
    const timer = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(timer);
  }, []);
  if (!recovery) {
    return <p>Waiting for /agent/recovery_state</p>;
  }
  const generated = Date.parse(recovery.generated_at);
  const stale = now - receivedAt > 10000 || (Number.isFinite(generated) && (now - generated > 10000 || generated - now > 5000));
  const pending = recovery.last_cycle?.pending ?? [];
  const execution = recovery.execution;
  const activeByKey = new Map((execution?.active ?? []).map((ex) => [ex.incident_key, ex]));
  const open = recovery.incidents.filter((incident) => !incident.acknowledged);
  const acknowledged = recovery.incidents.filter((incident) => incident.acknowledged);
  const ignored = recovery.ignored_warnings ?? acknowledged.map((incident) => ({
    key: incident.key, at: incident.acknowledged!.at, note: incident.acknowledged!.note,
    playbook_id: incident.playbook_id, summary: incident.reason, active: true,
  }));
  return (
    <fieldset className="recovery-panel" disabled={stale} style={{ border: 0, padding: 0, margin: 0, minWidth: 0 }}>
      {stale && <p role="status">Recovery evidence is stale. Controls disabled; waiting for a fresh update.</p>}
      <small>
        env {recovery.env} · profile {recovery.launch_profile ?? "all"}
        {recovery.launch_profile_source && !recovery.launch_profile_source.startsWith("launch parameter") && (
          <span title={recovery.launch_profile_source}> ({recovery.launch_profile_source.startsWith("launcher") ? "from launcher settings" : "not set"})</span>
        )}
        {recovery.not_launched_items?.length ? ` (${recovery.not_launched_items.length} items not launched)` : ""} ·{" "}
        {execution?.enabled ? "Recovery enabled (approval policy applies)" : "diagnose only"}
        {execution?.require_verified && " (verified playbooks only)"} · {recovery.playbooks.active} procedures · last cycle{" "}
        {recovery.last_cycle?.result ?? "—"}
        {pending.length > 0 && ` · ${pending.length} pending`}
      </small>
      {recovery.robot_health_summary && <details>
        <summary>Robot health response: {recovery.robot_health_summary.level}</summary>
        <p>OK: skip · WARN: check / notify · BAD: diagnose / recover. Motion eligibility is assessed by capability and safety checks.</p>
        {recovery.robot_health_summary.items.map(item => <div key={item.item}>
          <strong>{item.item}: {item.level}</strong>{item.ignored && ` (observed ${item.observed_level}; ignored)`}
          <div>{item.action} · {item.checks.join("; ")}</div>
        </div>)}
      </details>}
      <small className="recovery-legend">{NUMBERING_LEGEND}</small>
      {recovery.env === "sim" && <label style={{ display: "block", margin: "8px 0" }}>
        <input type="checkbox" checked={recovery.auto_approve === true}
          disabled={stale || !execution?.enabled || recovery.lifecycle !== "RUNNING"}
          onChange={event => sendCommand({ type: "set_auto_approve", enabled: event.target.checked })} />
        PB 自动批准（本次 Sim 会话）
        <small style={{ display: "block" }}>开启后自动批准当前及后续 PB 动作确认；停稳、前置条件和恢复验证仍生效。人工操作步骤仍需人工完成。重启后关闭。</small>
      </label>}
      {recovery.environment && <section className={`capability-card ${!stale && recovery.environment.recovery_allowed ? "ok" : "warn"}`} aria-label="Environment availability">
        <strong>Environment / 環境状態: {stale ? "UNKNOWN (stale)" : recovery.environment.state}</strong>
        <p>{stale ? "Last received evidence is historical, not current readiness." : recovery.environment.summary}</p>
        <details><summary>Environment evidence ({recovery.environment.suppressed_items.length} downstream signals suspended)</summary>
          <pre style={{ whiteSpace: "pre-wrap", overflowWrap: "anywhere" }}>{JSON.stringify(recovery.environment.evidence, null, 2)}</pre>
          <p>{recovery.environment.suppressed_items.join(", ")}</p>
        </details>
      </section>}
      {!!recovery.startup_units?.length && <details>
        <summary>Startup coverage / 起動項目 ({recovery.startup_units.length})</summary>
        {recovery.startup_units.map(unit => <div className="recovery-root-evidence" key={unit.id}>
          <strong>{unit.label}: {stale ? "UNKNOWN (stale)" : unit.status}</strong>
          <div>{unit.recovery === "authorized_start" ? "Approved start available through the incident card" : "Manual / external startup"}</div>
          <small>{unit.playbooks?.join(", ") || "No software PB"} · {unit.source}</small>
          {!!unit.depends_on?.length && <small>Requires: {unit.depends_on.join(", ")}</small>}
          <small>{Object.entries(unit.evidence).map(([name, status]) => `${name}=${status ?? "UNKNOWN"}`).join(", ")}</small>
          {unit.limitation && <p>{unit.limitation}</p>}
        </div>)}
      </details>}
      {recovery.lifecycle && recovery.lifecycle !== "RUNNING" && <p role="status">{recovery.lifecycle}: automatic recovery is disabled for the requested Agent shutdown.</p>}
      {(execution?.operator_requests ?? []).map((request) => (
        <OperatorRequestCard key={request.id} request={request} sendCommand={sendCommand}
          incident={recovery.incidents.find(incident => incident.key === request.incident_key)} />
      ))}
      {open.length === 0 ? (
        <p>{recovery.environment && !recovery.environment.recovery_allowed ? "Downstream recovery suspended; this does not mean the robot is healthy." : "No open incidents."}</p>
      ) : (
        open.map((incident) => (
          <IncidentCard
            key={incident.id}
            incident={incident}
            execution={activeByKey.get(incident.key)}
            sendCommand={sendCommand}
          />
        ))
      )}
      {ignored.length > 0 && (
        <details className="ignored-warnings" open>
          <summary>Snoozed warnings ({ignored.length})</summary>
          {ignored.map((warning) => (
            <div className="ignored-warning" key={warning.key}>
              <span title={warning.summary}>{warning.playbook_id || warning.key} · {warning.active ? "Still active" : "Not currently triggered"}</span>
              <button type="button" className="secondary" onClick={() => sendCommand({ type: "unacknowledge", incident_key: warning.key })}>Restore alerts</button>
            </div>
          ))}
        </details>
      )}
      {(execution?.finished?.length ?? 0) > 0 && (
        <details>
          <summary>Recovery runs ({execution.finished.length})</summary>
          <ul>
            {execution.finished.map((ex) => (
              <li key={ex.id}>
                {ex.finished_at?.slice(11, 19)} #{ex.playbook_number} {ex.playbook_id} → {ex.state} — {ex.message}
              </li>
            ))}
          </ul>
        </details>
      )}
      {recovery.recently_closed.length > 0 && (
        <details>
          <summary>Incident history ({recovery.recently_closed.length})</summary>
          <ul>
            {recovery.recently_closed.map((incident) => (
              <li key={incident.id}>
                {incident.resolved_at?.slice(11, 19)} {playbookLabel(incident.playbook_id, incident.playbook_no, incident.key)}
                {" "} {incident.status}{incident.status === "SUBSUMED" ? " (covered by upstream fault; not recovered)" : ""}
              </li>
            ))}
          </ul>
        </details>
      )}
    </fieldset>
  );
}

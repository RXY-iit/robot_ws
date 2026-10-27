import { useEffect, useState } from "react";
import type { HealthDashboardState } from "../types";

interface Props { state: HealthDashboardState | null; receivedAt: number | null; connected: boolean }
const labels: Record<string, string> = {READY: "Ready", WAITING: "Awaiting evidence", BLOCKED: "Blocked", DEGRADED: "Degraded", STALE: "Stale"};
const timeText = (value: string | null) => value ? new Date(value).toLocaleTimeString("en-GB", {timeZone: "Asia/Tokyo", hour12: false}) : "Not ready yet";

export function ReadinessPanel({state, receivedAt, connected}: Props) {
  const [now, setNow] = useState(Date.now());
  useEffect(() => {const id = window.setInterval(() => setNow(Date.now()), 1000); return () => window.clearInterval(id);}, []);
  const generatedAge = state?.generated_at ? now - Date.parse(state.generated_at) : NaN;
  const stale = !connected || receivedAt == null || now - receivedAt > 10000
    || !Number.isFinite(generatedAge) || Math.abs(generatedAge) > 10000;
  return <div className="readiness-panel" role="region" aria-label="Readiness by Capability">
    <h2>Readiness by Capability</h2>
    {!state?.readiness ? <p className="small">Waiting for readiness evidence</p> : <>
      {stale && <p role="status">Health data is stale. Historical status does not establish current readiness.</p>}
      {state.readiness.groups.map(group => {
        const status = stale ? "STALE" : group.status;
        return <details key={group.id} className={`readiness-item readiness-${status.toLowerCase()}`}>
          <summary><strong>{group.title}</strong><span>{labels[status] || status}</span></summary>
          <p>{group.summary}</p>
          <small>First ready: {timeText(group.first_ready_at)} · Ready since: {timeText(group.ready_since)} (JST)</small>
          <ul>{group.checks.map(check => <li key={check.id}><b>{check.id}</b> · {check.status}<br />{check.summary}</li>)}</ul>
          {group.notes.map(note => <p className="small" key={note}>{note}</p>)}
        </details>;
      })}
      <details className="readiness-context"><summary>Performance, RGB-D sync and inference history</summary>
        {state.readiness.supplemental.map(item => <p key={item.id}><b>{item.id}</b> · {stale ? "STALE" : item.status}<br />{item.summary}</p>)}
      </details>
    </>}
  </div>;
}

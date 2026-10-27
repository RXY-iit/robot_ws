import { effectiveHealthSummaries } from "../lib/effectiveHealthSummary";
import type { HealthDashboardState, HealthState, RecoveryState } from "../types";
import { levelClass } from "../lib/missionLogic";
import { ReadinessPanel } from "./ReadinessPanel";
import { useEffect, useState } from "react";

interface Props {
  health: HealthState | null;
  recovery: RecoveryState | null;
  dashboardHealth: HealthDashboardState | null;
  receivedAt: number | null;
  connected: boolean;
}

export function HealthPanel({ health, dashboardHealth, receivedAt, connected, recovery }: Props) {
  const [now, setNow] = useState(Date.now());
  useEffect(() => { const id = window.setInterval(() => setNow(Date.now()), 1000); return () => window.clearInterval(id); }, []);
  const generatedAge = dashboardHealth?.generated_at ? now - Date.parse(dashboardHealth.generated_at) : NaN;
  const stale = !connected || receivedAt == null || now - receivedAt > 10000
    || !Number.isFinite(generatedAge) || Math.abs(generatedAge) > 10000;
  const cards: [string, string, string, string | null][] = dashboardHealth?.summaries?.length
    ? effectiveHealthSummaries(dashboardHealth, recovery, now).map((summary) => [
        summary.title,
        summary.status,
        summary.summary,
        summary.reason_domain_id || null,
      ])
    : [
        ["Device & Data Acquisition", "UNKNOWN", "Waiting for /agent/health_dashboard_state", null],
        ["ROS & Platform Delivery", "UNKNOWN", "Waiting for /agent/health_dashboard_state", null],
        [
          "Functional & Behavior",
          health?.overall || "UNKNOWN",
          health ? `Legacy Agent report: ${health.recommended_action || "no action"}` : "Waiting for health evidence",
          null,
        ],
      ];

  return (
    <>
      <div className="health-summary-grid">
        {cards.map(([name, recordedLevel, summary, domain]) => {
          const level = stale ? "STALE" : recordedLevel;
          return (
          <a
            key={name}
            className={`health-card health-summary-card ${levelClass(level)}`}
            href={domain ? `/health#${domain}` : "/health"}
          >
            <strong>
              <span>{name}</span>
              <span>{level}</span>
            </strong>
            <span>{level}</span>
            <small>{stale ? "Health evidence is unavailable or stale. Last report: " : ""}{summary}</small>
          </a>
        ); })}
      </div>
      <div className="panel-link-row">
        <a className="text-link" href="/health">
          Open seven-domain Health Dashboard →
        </a>
      </div>
      <ReadinessPanel state={dashboardHealth} receivedAt={receivedAt} connected={connected} />
    </>
  );
}

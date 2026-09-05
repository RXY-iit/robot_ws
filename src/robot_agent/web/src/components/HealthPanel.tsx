import type { HealthDashboardState, HealthState } from "../types";
import { levelClass } from "../lib/missionLogic";

interface Props {
  health: HealthState | null;
  dashboardHealth: HealthDashboardState | null;
}

export function HealthPanel({ health, dashboardHealth }: Props) {
  const cards: [string, string, string, string | null][] = dashboardHealth?.summaries?.length
    ? dashboardHealth.summaries.map((summary) => [
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
        {cards.map(([name, level, summary, domain]) => (
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
            <small>{summary}</small>
          </a>
        ))}
      </div>
      <div className="panel-link-row">
        <a className="text-link" href="/health">
          Open seven-domain Health Dashboard →
        </a>
      </div>
    </>
  );
}

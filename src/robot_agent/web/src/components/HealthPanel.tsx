import type { HealthState, NavigationProgress } from "../types";
import { levelClass, navigationProgressText } from "../lib/missionLogic";

interface Props {
  health: HealthState | null;
  navigationProgress: NavigationProgress | null;
}

export function HealthPanel({ health, navigationProgress }: Props) {
  const components = health?.components || {};
  const cards: [string, string, string][] = [
    ["Overall", health?.overall || "UNKNOWN", health?.recommended_action || "check_system"],
    ["Mode", components.mode?.level || "UNKNOWN", components.mode?.summary || "UNKNOWN"],
    ["Safety", components.safety?.level || "UNKNOWN", components.safety?.summary || "UNKNOWN"],
    ["Localization", components.localization?.level || "UNKNOWN", components.localization?.summary || "UNKNOWN"],
    ["Local Planner", components.local_planner?.level || "UNKNOWN", components.local_planner?.summary || "UNKNOWN"],
    ["Nav Timeout", navigationProgress ? "OK" : "UNKNOWN", navigationProgressText(navigationProgress)],
  ];

  return (
    <div className="health-grid">
      {cards.map(([name, level, summary]) => (
        <div key={name} className={`health-card ${levelClass(level)}`}>
          <strong>
            <span>{name}</span>
            <span>{level}</span>
          </strong>
          <span>{level}</span>
          <small>{summary}</small>
        </div>
      ))}
    </div>
  );
}

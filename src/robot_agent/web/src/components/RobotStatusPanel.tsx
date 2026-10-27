import { Fragment } from "react";

interface Props {
  robotMode: string;
  safetyStatus: string;
  localizationWarning: string;
  stationaryDriftM: string;
  agentStatus: string;
  humanPrompt: string;
}

function valueClass(value: string): string {
  if (value.startsWith("STALE") || value.startsWith("BLOCKED")) return "value bad";
  if (value.startsWith("UNKNOWN")) return "value warn";
  if (value === "AUTO" || value === "OK") return "value ok";
  if (value === "ESTOP") return "value bad";
  return "value";
}

export function RobotStatusPanel({
  robotMode,
  safetyStatus,
  localizationWarning,
  stationaryDriftM,
  agentStatus,
  humanPrompt,
}: Props) {
  const rows: [string, string][] = [
    ["robot_mode", robotMode],
    ["safety", safetyStatus],
    ["localization", localizationWarning],
    ["drift_m", stationaryDriftM],
    ["agent", agentStatus],
  ];
  return (
    <div className="kv">
      {rows.map(([label, value]) => (
        <Fragment key={label}>
          <div className="label">{label}</div>
          <div className={valueClass(value)}>{value || "UNKNOWN"}</div>
        </Fragment>
      ))}
      <div className="label">prompt</div>
      <div className="value warn">{humanPrompt || "なし"}</div>
    </div>
  );
}

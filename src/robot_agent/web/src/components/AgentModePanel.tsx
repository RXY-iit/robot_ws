import type { AgentModeState, MissionState } from "../types";

interface Props {
  modeState: AgentModeState | null;
  mission: MissionState | null;
  onModeChange: (mode: string) => void;
}

const MODE_LABELS: Record<string, string> = {
  classic: "Classic",
  jev_primary: "JEV Primary",
  jev_shadow: "JEV Shadow",
};

export function AgentModePanel({ modeState, mission, onModeChange }: Props) {
  const active = modeState?.agent_mode || mission?.agent_mode || "jev_primary";
  const modes = modeState?.available_modes?.length ? modeState.available_modes : ["classic", "jev_primary", "jev_shadow"];
  const trace = mission?.jev_trace;
  const total = trace?.total || {};
  const calls = trace?.calls || [];
  const jevStatus = modeState?.jev_status;
  const jevBadgeText =
    modeState?.jev_available === false
      ? jevStatus?.configured
        ? "JEV probe failed"
        : "JEV local"
      : modeState?.jev_provider || "local";
  const jevBadgeTitle = jevStatus?.error
    ? `${jevStatus.status || "unavailable"}: ${jevStatus.error}`
    : jevStatus?.status || "";

  return (
    <div className="agent-mode-panel">
      <div className="section-head compact">
        <h2>Agent Mode</h2>
        <span className={`pill ${modeState?.jev_available === false ? "bad" : "ok"}`} title={jevBadgeTitle}>
          {jevBadgeText}
        </span>
      </div>
      <div className="segmented">
        {modes.map((mode) => (
          <button key={mode} className={mode === active ? "" : "secondary"} onClick={() => onModeChange(mode)}>
            {MODE_LABELS[mode] || mode}
          </button>
        ))}
      </div>
      <div className="jev-trace-card">
        <strong>
          {trace?.agent_mode || active}: {Number(total.calls || 0)} calls · {Number(total.latency_ms || 0).toFixed(1)} ms
        </strong>
        <span>
          input {Number(total.input_tokens_est || 0)} tok · ${Number(total.estimated_total_cost_usd || 0).toFixed(6)}
        </span>
        {calls.slice(-3).map((call, index) => (
          <small key={`${call.call_id || call.stage}-${index}`}>
            {call.stage || "jev"}: {Number(call.latency_ms || 0).toFixed(1)} ms · {call.adopted_decision || "decision"}
          </small>
        ))}
      </div>
    </div>
  );
}

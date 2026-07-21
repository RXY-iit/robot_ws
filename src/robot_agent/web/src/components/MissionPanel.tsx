import type { MissionEvent, MissionState, NavigationProgress, NavigationRouteState, HealthState } from "../types";
import {
  AGENT_ORDER,
  agentForSkill,
  alignPlanResults,
  artifactUrl,
  computeAgentStatus,
  skillExtraLines,
  statusClass,
  statusLabel,
} from "../lib/missionLogic";

interface Props {
  mission: MissionState | null;
  health: HealthState | null;
  navigationProgress: NavigationProgress | null;
  navigationRouteState: NavigationRouteState | null;
  missionEvents: MissionEvent[];
  receivedPending: boolean;
  pendingCommandText: string;
}

export function MissionPanel({
  mission,
  health,
  navigationProgress,
  navigationRouteState,
  missionEvents,
  receivedPending,
  pendingCommandText,
}: Props) {
  if (!mission) {
    return (
      <>
        <h2>ミッション</h2>
        <div className="summary">
          <SummaryCard label="状態" value={receivedPending ? "受信" : "UNKNOWN"} />
          <SummaryCard label="コマンド" value={receivedPending ? pendingCommandText : "なし"} />
          <SummaryCard label="意図" value="解析待ち" />
          <SummaryCard label="結果" value="実行中" />
        </div>
        <div className="section-head">
          <h2>Intent Routing</h2>
          <span className="small">解析待ち</span>
        </div>
        <div className="block-grid">
          <div className="block active">
            <div className="block-title">
              <span>Intent Routing</span>
              <span>active</span>
            </div>
            <div className="block-meta">Deterministic -&gt; Embedding -&gt; LLM Fallback の順に解析しています。</div>
          </div>
        </div>
      </>
    );
  }

  const intent = mission.intent || {};
  const plan = mission.plan || [];
  const current = Number.isInteger(mission.current_step_index) ? (mission.current_step_index as number) : 0;
  const statusText = String(mission.status || "");
  const isDone = statusText.includes("DONE");
  const agentStatus = computeAgentStatus(mission, navigationProgress, navigationRouteState, missionEvents, health);
  const alignedResults = alignPlanResults(plan, mission.skill_results);
  const trace = mission.parser_trace || {};
  const routeSteps = trace.route_steps || [];
  const routingSource = intent.task_type ? `${trace.accepted_layer || trace.source || "未確定"} -> ${intent.task_type}` : "解析待ち";

  return (
    <>
      <h2>ミッション</h2>
      <div className="summary">
        <SummaryCard label="状態" value={statusLabel(mission.status)} />
        <SummaryCard label="コマンド" value={mission.user_text || "なし"} />
        <SummaryCard label="意図" value={intent.task_type ? `${intent.task_type}${intent.target ? " -> " + intent.target : ""}` : "なし"} />
        <SummaryCard label="結果" value={mission.final_message || mission.question_for_human || "実行中"} />
      </div>

      <div className="section-head">
        <h2>Intent Routing</h2>
        <span className="small">{routingSource}</span>
      </div>
      <div className="block-grid">
        {routeSteps.length ? (
          routeSteps.map((step, index) => {
            const cls = step.status === "accepted" ? "done" : step.status === "rejected" ? "waiting" : "active";
            const score = Number.isFinite(step.score) ? ` score=${(step.score as number).toFixed(3)}` : "";
            const meta = [
              step.task_type ? ` task=${step.task_type}` : "",
              score,
              step.engine ? ` engine=${step.engine}` : "",
              step.model ? ` model=${step.model}` : "",
              step.template_id ? ` template=${step.template_id}` : "",
              step.reason ? ` reason=${step.reason}` : "",
            ]
              .join("")
              .trim();
            return (
              <div key={index} className={`block ${cls}`}>
                <div className="block-title">
                  <span>{step.layer || "Parser"}</span>
                  <span>{step.status || "checked"}</span>
                </div>
                <div className="block-meta">{meta || "checked"}</div>
                {step.error && <div className="block-detail">{step.error}</div>}
              </div>
            );
          })
        ) : (
          <div className="block waiting">
            <div className="block-title">
              <span>解析待ち</span>
              <span>waiting</span>
            </div>
            <div className="block-meta">Intent parser の結果がここに表示されます。</div>
          </div>
        )}
      </div>

      <div className="section-head">
        <h2>エージェント</h2>
        <span className="small">実行中の block を強調表示</span>
      </div>
      <div className="block-grid">
        {AGENT_ORDER.map((name) => (
          <div key={name} className={`block ${agentStatus[name].state}`}>
            <div className="block-title">
              <span>{name}</span>
              <span>{agentStatus[name].state || "idle"}</span>
            </div>
            <div className="block-meta">{agentStatus[name].detail}</div>
          </div>
        ))}
      </div>

      <div className="section-head">
        <h2>スキル実行計画</h2>
        <span className="small">
          {plan.length ? `ステップ ${Math.min(current + 1, plan.length)} / ${plan.length}` : "ステップ 0 / 0"}
        </span>
      </div>
      <div className="block-grid">
        {plan.length ? (
          plan.map((call, index) => {
            const result = alignedResults[index];
            let cls = "";
            if (index < current) cls = "done";
            if (index === current && !isDone) cls = "active";
            if (result) cls = statusClass(result.code, result.success) || cls;
            const agent = agentForSkill(call.name);
            const detail = result ? `${result.code}: ${result.message || ""}` : call.reason || "pending";
            const payload = call.payload ? JSON.stringify(call.payload) : "{}";
            const extraLines = skillExtraLines(call, result, missionEvents, mission.current_step_index);
            return (
              <div key={index} className={`block ${cls}`}>
                <div className="block-title">
                  <span>
                    {index + 1}. {call.name}
                  </span>
                  <span>{agent}</span>
                </div>
                <div className="block-meta">{payload}</div>
                <div className="block-detail">{detail}</div>
                {extraLines.map((line, lineIndex) => (
                  <div key={lineIndex} className="block-detail">
                    {line}
                  </div>
                ))}
              </div>
            );
          })
        ) : (
          <div className="block waiting">
            <div className="block-title">
              <span>計画なし</span>
              <span>待機</span>
            </div>
            <div className="block-meta">コマンド待ち。</div>
          </div>
        )}
      </div>

      <div className="section-head">
        <h2>検出結果画像</h2>
        <span className="small">検出に使われた画像</span>
      </div>
      <ResultImages mission={mission} />

      <details>
        <summary>Raw mission state</summary>
        <pre>{JSON.stringify(mission, null, 2)}</pre>
      </details>
    </>
  );
}

function SummaryCard({ label, value }: { label: string; value: string }) {
  return (
    <div className="summary-card">
      <strong>{label}</strong>
      <span>{value}</span>
    </div>
  );
}

function ResultImages({ mission }: { mission: MissionState }) {
  const seen = new Set<string>();
  const matches: { path: string; label: string; text: string; confidence?: number; matched: boolean }[] = [];
  for (const result of mission.skill_results || []) {
    if (result.skill_name !== "detect_known_object") continue;
    for (const item of result.metrics?.annotated_images || []) {
      const path = item.annotated_image_path || item.image_path;
      if (!path || seen.has(path) || !item.matched) continue;
      seen.add(path);
      matches.push({
        path,
        label: item.matched ? `MATCH ${item.top_label || "detection"}` : item.top_label || "candidate",
        text: `${item.query || ""} detections:${item.detection_count || 0} matched:${item.matched_count || 0}`,
        confidence: item.top_confidence,
        matched: Boolean(item.matched),
      });
    }
  }

  if (!matches.length) {
    return (
      <div className="result-images">
        <div className="block waiting">
          <div className="block-title">
            <span>画像なし</span>
            <span>待機</span>
          </div>
          <div className="block-meta">検出に成功すると、該当 observation がここに表示されます。</div>
        </div>
      </div>
    );
  }

  return (
    <div className="result-images">
      {matches.map((item) => (
        <div key={item.path} className={`result-image ${item.matched ? "matched" : ""}`}>
          <img src={artifactUrl(item.path)} alt={item.label} loading="lazy" />
          <div>
            {item.label} {Number.isFinite(item.confidence) ? (item.confidence as number).toFixed(2) : ""}
            <br />
            {item.text || item.path}
          </div>
        </div>
      ))}
    </div>
  );
}

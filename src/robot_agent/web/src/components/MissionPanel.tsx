import type { HumanReportState, MissionEvent, MissionState, NavigationProgress, NavigationRouteState, HealthState } from "../types";
import { artifactUrl, statusLabel } from "../lib/missionLogic";
import { IntentRouting, MissionTaskFlow, DecisionDetails } from "./MissionProgress";

interface Props {
  mission: MissionState | null;
  health: HealthState | null;
  navigationProgress: NavigationProgress | null;
  navigationRouteState: NavigationRouteState | null;
  missionEvents: MissionEvent[];
  humanReport: HumanReportState | null;
  receivedPending: boolean;
  pendingCommandText: string;
}

export function MissionPanel({ mission, missionEvents, humanReport, receivedPending, pendingCommandText }: Props) {
  if (!mission) return <>
    <h2>ミッション</h2>
    <div className="mission-overview">
      <SummaryCard label="状態" value={receivedPending ? "受信 / 解析中" : "待機"} />
      <SummaryCard label="コマンド" value={receivedPending ? pendingCommandText : "なし"} />
    </div>
    <h2>Decision Model Routing</h2><div className="small">{receivedPending ? "解析中" : "待機"}</div>
    <h2>ミッションイベント</h2><div className="small">計画なし</div>
    <HumanReportCard report={humanReport} />
  </>;
  const current = mission.plan?.[mission.current_step_index ?? -1];
  const failed = Boolean(mission.status?.startsWith("FAILED"));
  const blocked = mission.status === "NEEDS_HUMAN" || failed;
  return <>
    <div className="section-head"><h2>ミッション</h2><span className="small">{mission.mission_id}</span></div>
    <div className="mission-overview">
      <SummaryCard label="状態" value={statusLabel(mission.status)} />
      <SummaryCard label="コマンド" value={mission.user_text || "なし"} />
      <SummaryCard label="現在の処理" value={mission.status === "EXECUTE_STEP" && current ? `${current.owner || "mission"} · ${current.name}` : mission.status || "待機"} />
      <SummaryCard label="結果" value={mission.final_message || mission.outcome || "未確定"} />
    </div>
    {blocked && <div className={`mission-blocker ${failed ? "failed" : "waiting"}`} role="status">
      <strong>{mission.status === "NEEDS_HUMAN" ? "確認待ち" : "実行エラー"}</strong>
      <div>{mission.question_for_human || mission.final_message}</div>
    </div>}
    <HumanReportCard report={humanReport} missionId={mission.mission_id} />
    <IntentRouting mission={mission} events={missionEvents} />
    <MissionTaskFlow mission={mission} events={missionEvents} />
    <h2>検出結果画像</h2><ResultImages mission={mission} />
    {!!mission.jev_trace?.calls?.length && <details className="raw-evidence">
      <summary>JEV Decision Trace · {mission.jev_trace.calls.length} calls</summary>
      {mission.jev_trace.calls.map((call, i) => <DecisionDetails key={call.call_id || i} call={call} />)}
    </details>}
    <details className="raw-evidence"><summary>Raw mission state</summary><pre>{JSON.stringify(mission, null, 2)}</pre></details>
  </>;
}

function SummaryCard({ label, value }: { label: string; value: string }) {
  return (
    <div className="summary-card">
      <strong>{label}</strong>
      <span>{value}</span>
    </div>
  );
}

function HumanReportCard({ report, missionId }: { report: HumanReportState | null; missionId?: string }) {
  if (!report?.report_url) return null;
  if (missionId && report.mission_id && report.mission_id !== missionId) return null;
  const api = report.api_summary || {};
  const apiText = api.enabled
    ? `api=${api.status || "pending"}${Number.isFinite(api.latency_ms) ? ` ${api.latency_ms}ms` : ""}`
    : "api=off";
  return (
    <div className="human-report-card">
      <div>
        <strong>Human report</strong>
        <span>{report.summary_line || "Report ready"}</span>
        <small>
          why={report.why_count || 0} actions={report.operator_action_count || 0} {apiText}
        </small>
      </div>
      <a className="button-link secondary" href={report.report_url} target="_blank" rel="noreferrer">
        More
      </a>
    </div>
  );
}

function ResultImages({ mission }: { mission: MissionState }) {
  const seenObservations = new Set<string>();
  const seenDetections = new Set<string>();
  const observations: { path: string; label: string; text: string; matched: boolean }[] = [];
  const matches: { path: string; label: string; text: string; confidence?: number; matched: boolean }[] = [];
  const candidates: { path: string; label: string; text: string; confidence?: number; matched: boolean }[] = [];
  for (const result of mission.skill_results || []) {
    if (result.skill_name === "capture_observation") {
      const metricPath = typeof result.metrics?.image_path === "string" ? result.metrics.image_path : "";
      const artifactImage = (result.artifacts || []).find((path) => /\.(png|jpe?g|webp)$/i.test(path));
      const path = metricPath || artifactImage || "";
      if (path && !seenObservations.has(path)) {
        seenObservations.add(path);
        observations.push({
          path,
          label: `OBS ${observations.length + 1}`,
          text: String(result.metrics?.scan_band || result.metrics?.color_topic || path),
          matched: false,
        });
      }
    }
    for (const observation of result.observations || []) {
      const obsPath = typeof observation.image_path === "string" ? observation.image_path : "";
      if (!obsPath || seenObservations.has(obsPath)) continue;
      seenObservations.add(obsPath);
      observations.push({
        path: obsPath,
        label: `OBS ${observations.length + 1}`,
        text: String(observation.scan_band || observation.color_topic || obsPath),
        matched: false,
      });
    }
    for (const item of result.metrics?.annotated_images || []) {
      const path = item.annotated_image_path || item.image_path;
      if (!path || seenDetections.has(path)) continue;
      seenDetections.add(path);
      const entry = {
        path,
        label: item.matched ? `MATCH ${item.top_label || "detection"}` : item.top_label || "candidate",
        text: `${item.query || ""} detections:${item.detection_count || 0} matched:${item.matched_count || 0}`,
        confidence: item.top_confidence,
        matched: Boolean(item.matched),
      };
      if (entry.matched) matches.push(entry);
      else candidates.push(entry);
    }
  }

  if (!observations.length && !matches.length && !candidates.length) {
    return (
      <div className="result-images">
        <div className="block waiting">
          <div className="block-title">
            <span>画像なし</span>
            <span>待機</span>
          </div>
          <div className="block-meta">画像を取得すると、検出成否に関係なくここに表示されます。</div>
        </div>
      </div>
    );
  }

  return (
    <div className="result-images">
      <ImageAlbum title="Captured observations" items={observations} />
      <ImageAlbum title="Matched detections" items={matches} />
      <ImageAlbum title="Candidates / no target" items={candidates} />
    </div>
  );
}

function ImageAlbum({
  title,
  items,
}: {
  title: string;
  items: { path: string; label: string; text: string; confidence?: number; matched: boolean }[];
}) {
  if (!items.length) return null;
  return (
    <div className="image-album">
      <div className="album-title">
        <span>{title}</span>
        <span>{items.length}</span>
      </div>
      <div className="album-strip">
        {items.map((item) => (
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
    </div>
  );
}

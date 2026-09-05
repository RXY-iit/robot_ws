import type {
  HealthState,
  Intent,
  MissionEvent,
  MissionState,
  NavigationProgress,
  NavigationRouteState,
  SkillCall,
  SkillResult,
} from "../types";

const AGENT_ORDER = [
  "Mission Agent",
  "Navigation Agent",
  "Health / Recovery Agent",
  "Perception Agent",
] as const;
export type AgentName = (typeof AGENT_ORDER)[number];
export { AGENT_ORDER };

const NAVIGATION_SKILLS = new Set([
  "resolve_semantic_target",
  "goto_pose",
  "move_relative",
  "move_relative_xy",
  "rotate_relative",
  "await_robot_auto_mode",
  "await_robot_stationary",
  "cancel_motion",
]);
const HEALTH_SKILLS = new Set(["check_localization_health", "recover_localization_spin"]);
const PERCEPTION_SKILLS = new Set(["set_camera_pose", "capture_observation", "detect_known_object"]);

export function agentForSkill(name: string): AgentName {
  if (NAVIGATION_SKILLS.has(name)) return "Navigation Agent";
  if (HEALTH_SKILLS.has(name)) return "Health / Recovery Agent";
  if (PERCEPTION_SKILLS.has(name)) return "Perception Agent";
  return "Mission Agent";
}

const STATUS_LABELS: Record<string, string> = {
  RECEIVED: "受信",
  PARSE: "解析中",
  PLAN: "計画中",
  EXECUTE_STEP: "実行中",
  PAUSED_HEALTH: "Health一時停止",
  RECOVERING: "復旧中",
  RESUMING: "再開中",
  NEEDS_HUMAN: "人の確認待ち",
  FAILED_SKILL: "失敗",
  DONE_WITH_STUB: "Stubで完了",
  DONE: "完了",
};

export function statusLabel(value?: string): string {
  const text = String(value || "");
  return STATUS_LABELS[text] || text || "UNKNOWN";
}

export function statusClass(status?: string, success?: boolean): string {
  const text = String(status || "").toUpperCase();
  if (text.includes("FAILED") || text.includes("NEEDS_HUMAN") || success === false) return "failed";
  if (text.includes("DONE") || text === "OK" || success === true) return "done";
  if (text.includes("WAIT") || text.includes("HUMAN")) return "waiting";
  return "";
}

export function shortNumber(value: unknown, digits = 3): string | null {
  const n = Number(value);
  return Number.isFinite(n) ? n.toFixed(digits) : null;
}

export function navigationProgressText(progress: NavigationProgress | null): string {
  if (!progress) return "待機";
  const phase = progress.phase || "navigation";
  const stalled = shortNumber(progress.stalled_sec, 1);
  const limit = shortNumber(progress.progress_watchdog_sec ?? progress.effective_timeout_sec, 1);
  const distance = shortNumber(progress.current_distance_m, 2);
  const best = shortNumber(progress.best_distance_m, 2);
  const parts = [`phase=${phase}`];
  if (stalled && limit) parts.push(`timeout=${stalled}/${limit}s`);
  else if (limit) parts.push(`limit=${limit}s`);
  if (distance) parts.push(`dist=${distance}m`);
  if (best) parts.push(`best=${best}m`);
  return parts.join(" ");
}

export function navigationRouteText(route: NavigationRouteState | null): string {
  if (!route) return "route=none";
  const phase = route.phase || "route";
  const items = Array.isArray(route.items) ? route.items : [];
  const active = Number.isInteger(route.active_index)
    ? items[route.active_index as number]
    : items.find((item) => item.status === "active");
  const parts = [`route=${phase}`];
  if (active) {
    const name = active.name || active.kind || active.id || "goal";
    parts.push(`active=${name}`);
    if (active.status) parts.push(`status=${active.status}`);
    if (active.xy_tolerance_m != null) parts.push(`tol=${active.xy_tolerance_m}m`);
  }
  if (items.length) {
    const summary = items.map((item) => `${item.name || item.kind}:${item.status || "pending"}`).join(" > ");
    parts.push(summary);
  }
  return parts.join(" / ");
}

export function missionEventsOf(events: MissionEvent[], type: string): MissionEvent[] {
  return events.filter((event) => event && event.type === type);
}

export function navigationStatusLines(
  call: SkillCall | undefined,
  result: SkillResult | null | undefined,
  events: MissionEvent[],
  currentStepIndex: number | undefined,
): string[] {
  const lines: string[] = [];
  const payload = call?.payload || {};
  const metrics = result?.metrics || {};
  const profile = metrics.tolerance_profile || (payload as any).tolerance_profile;
  const timeoutPolicy = metrics.timeout_policy || ((payload as any).dynamic_timeout ? "dynamic" : null);
  if (timeoutPolicy) {
    const requested = shortNumber(metrics.requested_timeout_sec ?? (payload as any).timeout_sec, 0);
    const effective = shortNumber(metrics.effective_timeout_sec, 0);
    const distance = shortNumber(metrics.start_xy_distance_m, 2);
    const extensions = metrics.timeout_extensions;
    let line = `Timeout: ${timeoutPolicy}`;
    if (requested) line += ` requested=${requested}s`;
    if (effective) line += ` effective=${effective}s`;
    if (distance) line += ` distance=${distance}m`;
    if (extensions) line += ` extensions=${extensions}`;
    lines.push(line);
  }
  const annotation = metrics.navigation_annotation || {};
  if (annotation.passage_id) {
    lines.push(`Annotation: ${annotation.passage_id}${annotation.direction ? ` ${annotation.direction}` : ""}`);
    if (annotation.waypoint_reachability) {
      const reach = annotation.waypoint_reachability;
      lines.push(`Waypoint reachability: ${reach.available ? "ok" : "failed"}${reach.reason ? ` ${reach.reason}` : ""}`);
    }
    if (annotation.waypoint_failure_action) {
      lines.push(`Waypoint action: ${annotation.waypoint_failure_action}`);
    }
  }
  if (metrics.near_goal_recovery) {
    lines.push(`Near-goal recovery: ${metrics.near_goal_recovery.success ? "ok" : "failed"}`);
  }
  if (metrics.relaxed_tolerance) {
    lines.push("Relaxed tolerance: accepted");
  }
  if (metrics.alternate_goal_sampling) {
    lines.push(`Alternate goal: ${metrics.alternate_goal_sampling.available ? "selected" : "none"}`);
  }
  if (profile && profile !== "default") {
    const xy = metrics.xy_goal_tolerance_m ?? (payload as any).xy_goal_tolerance_m;
    const yaw = metrics.yaw_goal_tolerance_rad ?? (payload as any).yaw_goal_tolerance_rad;
    lines.push(`Tolerance: ${profile}${xy != null ? ` xy=${xy}m` : ""}${yaw != null ? ` yaw=${yaw}rad` : ""}`);
  }
  if (metrics.final_xy_error_m != null || metrics.final_yaw_error_rad != null) {
    const xy = shortNumber(metrics.final_xy_error_m);
    const yaw = shortNumber(metrics.final_yaw_error_rad);
    lines.push(`Final error: xy=${xy ?? "-"}m yaw=${yaw ?? "-"}rad`);
  }
  for (const event of missionEventsOf(events, "navigation_tolerance_adjusted")) {
    const selected = event.payload?.selected_profile || {};
    const step = event.payload?.step_index;
    if (step == null || step === (currentStepIndex ?? step) || event.payload?.skill === call?.name) {
      lines.push(`Adjusted by Health/Recovery: ${selected.name || "profile"}`);
    }
  }
  return lines;
}

export function recoveryStatusLines(events: MissionEvent[]): string[] {
  const lines: string[] = [];
  for (const event of events) {
    if (event.type === "health_cancel_requested") {
      const summary =
        event.payload?.health?.components?.local_planner?.summary ||
        event.payload?.health?.components?.localization?.summary ||
        event.payload?.reason ||
        "health issue";
      lines.push(`Health cancel: ${summary}`);
    }
    if (event.type === "mission_paused_health") {
      const summary =
        event.payload?.health?.components?.local_planner?.summary ||
        event.payload?.health?.components?.localization?.summary ||
        event.payload?.message ||
        "paused";
      lines.push(`Paused: ${summary}`);
    }
    if (event.type === "mission_resuming") {
      lines.push("Recovery OK: mission resumed");
    }
  }
  return lines.slice(-3);
}

export function skillExtraLines(
  call: SkillCall | undefined,
  result: SkillResult | null | undefined,
  events: MissionEvent[],
  currentStepIndex: number | undefined,
): string[] {
  const lines: string[] = [];
  if (call?.name === "goto_pose") lines.push(...navigationStatusLines(call, result, events, currentStepIndex));
  if (call?.name === "recover_localization_spin") {
    const action = result?.metrics?.recovery_action;
    if (action) lines.push(`Recovery action: ${action}`);
  }
  if (call?.name === "detect_known_object") {
    const detections = result?.metrics?.detections || [];
    const matched = detections.filter((item: any) => item.matched_query || (item.matched_tokens || []).length).length;
    if (detections.length) lines.push(`Detections: ${matched}/${detections.length} matched`);
  }
  return lines;
}

export function alignPlanResults(plan: SkillCall[], skillResults: SkillResult[] | undefined): (SkillResult | null)[] {
  const aligned: (SkillResult | null)[] = [];
  const results = Array.isArray(skillResults) ? skillResults : [];
  const byStep = new Map<number, SkillResult>();
  for (const result of results) {
    if (Number.isInteger(result?.step_index)) byStep.set(result.step_index as number, result);
  }
  let cursor = 0;
  for (let stepIndex = 0; stepIndex < (plan || []).length; stepIndex += 1) {
    const call = plan[stepIndex];
    let matched: SkillResult | null = byStep.get(stepIndex) || null;
    if (!matched) {
      for (let i = cursor; i < results.length; i += 1) {
        if (Number.isInteger(results[i]?.step_index)) continue;
        if (results[i]?.skill_name === call.name) {
          matched = results[i];
          cursor = i + 1;
          break;
        }
      }
    }
    aligned.push(matched);
  }
  return aligned;
}

export function levelClass(level?: string): string {
  const normalized = String(level || "UNKNOWN").toLowerCase();
  if (["ok", "available", "observed", "verified", "active", "idle"].includes(normalized)) return "ok";
  if (["warn", "degraded", "estimated", "reported", "recovered", "supervised_only"].includes(normalized)) return "warn";
  // "error" is real_robot_monitor's level string (real hardware only); treat it
  // the same as "bad" so hardware faults render with the same red state.
  if (["bad", "error", "failed", "offline", "blocked", "stale"].includes(normalized)) return "bad";
  return "unknown";
}

export function clarificationCommand(intent: Intent | undefined, targetId: string): string {
  const task = String(intent?.task_type || "");
  if (task === "inspect_place") return `inspect ${targetId}`;
  if (task === "search_object_near_place") {
    const query = intent?.object_query || "requested object";
    return `check ${targetId} for ${query}`;
  }
  return `go to ${targetId}`;
}

export function artifactUrl(path: string): string {
  return `/artifact?path=${encodeURIComponent(path)}`;
}

export interface AgentBlockStatus {
  state: string;
  detail: string;
}

export function computeAgentStatus(
  mission: MissionState,
  navigationProgress: NavigationProgress | null,
  navigationRouteState: NavigationRouteState | null,
  missionEvents: MissionEvent[],
  healthState: HealthState | null,
): Record<AgentName, AgentBlockStatus> {
  const status = {} as Record<AgentName, AgentBlockStatus>;
  for (const name of AGENT_ORDER) status[name] = { state: "", detail: "待機" };

  const statusText = String(mission.status || "");
  if (statusText === "PARSE" || statusText === "PLAN" || statusText === "RECEIVED") {
    status["Mission Agent"] = { state: "active", detail: statusLabel(mission.status) };
  }

  const plan = mission.plan || [];
  const current = Number.isInteger(mission.current_step_index) ? (mission.current_step_index as number) : 0;
  const isDone = statusText.includes("DONE");
  plan.forEach((call, index) => {
    const agent = agentForSkill(call.name);
    if (index < current) status[agent] = { state: "done", detail: "関連ステップ完了" };
    if (index === current && !isDone) {
      status[agent] = { state: "active", detail: call.name };
    }
  });

  if (statusText.includes("NEEDS_HUMAN")) {
    status["Mission Agent"] = { state: "waiting", detail: mission.question_for_human || "人の確認待ち" };
  }
  if (statusText.includes("FAILED")) {
    status["Mission Agent"] = { state: "failed", detail: mission.final_message || "失敗" };
  }
  if (isDone) {
    for (const name of AGENT_ORDER) {
      if (status[name].detail !== "待機") status[name].state = "done";
    }
  }

  if (navigationProgress && !isDone) {
    status["Navigation Agent"] = {
      state: "active",
      detail: `${navigationProgressText(navigationProgress)} / ${navigationRouteText(navigationRouteState)}`,
    };
  } else if (navigationRouteState && !isDone) {
    status["Navigation Agent"] = { state: "active", detail: navigationRouteText(navigationRouteState) };
  }

  const recoveryLines = recoveryStatusLines(missionEvents);
  if (recoveryLines.length) {
    const currentLocalization = healthState?.components?.localization;
    const currentOverall = healthState?.overall || "UNKNOWN";
    const healthIsOk = currentOverall === "OK";
    const recoveryState = healthIsOk ? "done" : isDone ? "done" : "active";
    const liveDetail =
      currentOverall !== "UNKNOWN"
        ? `Current health: ${currentOverall} / Localization: ${currentLocalization?.level || "UNKNOWN"} ${
            currentLocalization?.summary || ""
          }`.trim()
        : recoveryLines.join(" / ");
    status["Health / Recovery Agent"] = { state: recoveryState, detail: liveDetail };
  }

  return status;
}

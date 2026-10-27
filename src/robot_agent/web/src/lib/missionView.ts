import type { Intent, JevCallTrace, MissionEvent, MissionState, SkillCall, SkillResult } from "../types";

export type ExecutionStatus = "pending" | "active" | "done" | "unmet" | "waiting" | "failed" | "cancelled" | "skipped";
export const executionLabels: Record<ExecutionStatus, string> = {
  pending: "未実行", active: "実行中", done: "完了", unmet: "目標未達", waiting: "確認待ち", failed: "実行エラー", cancelled: "中止", skipped: "条件不成立・スキップ",
};
export type TaskRecord = NonNullable<MissionState["tasks"]>[number];
export interface StepView {
  kind: "skill";
  id: string;
  call: SkillCall;
  index: number;
  status: ExecutionStatus;
  attempts: SkillResult[];
  result?: SkillResult;
}
export interface TaskView {
  kind: "task";
  id: string;
  call: SkillCall;
  status: ExecutionStatus;
  record?: TaskRecord;
  entries: ExecutionNode[];
}
export type ExecutionNode = StepView | TaskView;

export function currentEvents(mission: MissionState, events: MissionEvent[]): MissionEvent[] {
  return events.filter(e => e.mission_id === mission.mission_id && e.type !== "agent_heartbeat");
}

export function intentSummary(intent?: Intent): string {
  if (!intent) return "解析待ち";
  const target = intent.target || intent.object_query;
  const slots = intent.constraints || {};
  const values = [target, slots.direction, slots.distance_m !== undefined ? `${slots.distance_m} m` : null,
    slots.yaw_deg !== undefined ? `${slots.yaw_deg}°` : null, slots.profile,
    slots.run_if === "previous_search_found" ? "if previous search found target" : null];
  return [intent.task_type || "unknown", ...values.filter(v => v !== undefined && v !== null && v !== "")].join(" · ");
}

export function parsedTasks(intent?: Intent): Intent[] {
  if (!intent || intent.task_type === "ask_human") return [];
  return intent.task_type === "sequence" ? (intent.constraints?.steps || []).flatMap(parsedTasks) : [intent];
}

export function parsedIntent(mission: MissionState, events: MissionEvent[]): Intent | undefined {
  return mission.parsed_intent || currentEvents(mission, events).find(e => e.type === "intent_parsed")?.payload?.intent || mission.intent;
}

export function resultStatus(result?: SkillResult): ExecutionStatus {
  if (!result) return "pending";
  if (result.code === "CANCELLED") return "cancelled";
  if (result.code === "NEEDS_HUMAN") return "waiting";
  if (!result.success || !["OK", "DRY_RUN_OK"].includes(result.code || "")) return "failed";
  return result.metrics?.goal_satisfied === false ? "unmet" : "done";
}

export function planAccepted(mission: MissionState, events: MissionEvent[]): boolean {
  if (mission.initial_plan !== undefined) return mission.initial_plan.length > 0;
  if (currentEvents(mission, events).some(e => e.type === "plan_created")) return true;
  const review = (mission.jev_trace?.calls || []).filter(c => c.stage === "plan_review").slice(-1)[0];
  if (mission.agent_mode === "jev_primary" && review && /reject|ask_/.test(review.adopted_decision || "")) return false;
  return Boolean(mission.plan?.length) && !["PARSE", "PLAN"].includes(mission.status || "");
}

export function executionGraph(mission: MissionState, events: MissionEvent[] = []): ExecutionNode[] {
  if (mission.initial_plan !== undefined && !planAccepted(mission, events)) return [];
  const plan = mission.plan || [];
  const records = new Map((mission.tasks || []).map(t => [t.task_id, t]));
  const pending = new Map(plan.filter(c => c.kind === "task" && c.task_id).map(c => [c.task_id!, c]));
  const parent = (id: string): string | undefined => records.get(id)?.parent_task_id || pending.get(id)?.catalog?.parent_task_id || undefined;
  const terminal = /^(DONE|FAILED|CANCELLED|NEEDS_HUMAN)/.test(mission.status || "");
  const step = (call: SkillCall, index: number): StepView => {
    const attempts = (mission.skill_results || []).filter(r => call.step_id && r.step_id
      ? r.step_id === call.step_id
      : r.step_index === index && r.skill_name === call.name);
    const result = attempts[attempts.length - 1];
    let status = resultStatus(result);
    if (!result && !terminal && index === mission.current_step_index && mission.status === "EXECUTE_STEP") status = "active";
    if (!result && mission.status === "CANCELLED") status = "cancelled";
    if (index === mission.current_step_index) {
      if (mission.status?.startsWith("FAILED") && !result) status = "failed";
      if (mission.status === "NEEDS_HUMAN" && !result) status = "waiting";
      if (["RECOVERING", "PAUSED_HEALTH"].includes(mission.status || "")) status = "waiting";
      const lastEvent = currentEvents(mission, events).filter(e => e.payload?.step_id === call.step_id && call.step_id
        || e.payload?.step_index === index && ["skill_started", "skill_finished"].includes(e.type || "")).slice(-1)[0];
      if (!terminal && mission.status === "EXECUTE_STEP" && lastEvent?.type === "skill_started") status = "active";
    }
    return { kind: "skill", id: call.step_id || `step-${index}`, call, index, attempts, result, status };
  };
  const taskCall = (id: string): SkillCall => pending.get(id) || {
    kind: "task", task_id: id, name: records.get(id)?.task_type || "task", owner: records.get(id)?.owner,
    payload: records.get(id)?.goal as Record<string, unknown> | undefined,
  };
  const descendant = (id: string | undefined, ancestor: string): boolean => {
    const visited = new Set<string>();
    while (id && !visited.has(id)) {
      if (id === ancestor) return true;
      visited.add(id); id = parent(id);
    }
    return false;
  };
  const task = (call: SkillCall, ancestors: Set<string>): TaskView => {
    const id = call.task_id!;
    const record = records.get(id);
    const entries: { position: number; node: ExecutionNode }[] = [];
    if (!ancestors.has(id)) {
      const next = new Set(ancestors).add(id);
      plan.forEach((c, i) => { if ((!c.kind || c.kind === "skill") && c.task_id === id) entries.push({ position: i, node: step(c, i) }); });
      const childIds = new Set([...records.keys(), ...pending.keys()]);
      for (const child of childIds) if (parent(child) === id && !next.has(child)) {
        const position = plan.findIndex(c => descendant(c.task_id, child));
        entries.push({ position: position < 0 ? plan.length : position, node: task(taskCall(child), next) });
      }
    }
    let status: ExecutionStatus = "pending";
    if (record?.state === "COMPLETED") status = record.goal_satisfied === true ? "done" : "unmet";
    else if (record?.state === "SKIPPED") status = "skipped";
    else if (record?.state === "WAITING_HUMAN") status = "waiting";
    else if (record?.state === "CANCELLED" || mission.status === "CANCELLED") status = "cancelled";
    else if (record?.state?.startsWith("FAILED") || record?.state === "DONE_WITH_STUB") status = "failed";
    else if (record && mission.status?.startsWith("FAILED") && entries.some(e => e.node.status === "failed")) status = "failed";
    else if (record && ["RECOVERING", "PAUSED_HEALTH"].includes(mission.status || "")) status = "waiting";
    else if (record && !terminal) status = "active";
    else if (record && mission.status === "NEEDS_HUMAN") status = "waiting";
    else if (mission.status === "NEEDS_HUMAN" && plan[mission.current_step_index || 0]?.task_id === id) status = "waiting";
    return { kind: "task", id, call, record, status, entries: entries.sort((a, b) => a.position - b.position).map(x => x.node) };
  };

  let roots = mission.initial_plan?.length ? mission.initial_plan
    : currentEvents(mission, events).find(e => e.type === "plan_created")?.payload?.plan as SkillCall[] | undefined;
  if (!roots?.length) {
    roots = [];
    const seen = new Set<string>();
    for (const call of plan) {
      if (call.kind === "task_end") continue;
      if (!call.task_id) { roots.push(call); continue; }
      let id = call.task_id;
      const visited = new Set<string>();
      while (parent(id) && !visited.has(id)) { visited.add(id); id = parent(id)!; }
      if (!seen.has(id)) { seen.add(id); roots.push(taskCall(id)); }
    }
    for (const record of records.values()) if (!record.parent_task_id && !seen.has(record.task_id)) {
      roots.push(taskCall(record.task_id)); seen.add(record.task_id);
    }
  }
  const used = new Set<number>();
  return roots.filter(c => c.kind !== "task_end").map((call, n) => {
    if (call.kind === "task" && call.task_id) return task(call, new Set());
    const index = plan.findIndex((c, i) => !used.has(i) && (call.step_id
      ? c.step_id === call.step_id : c === call || c.name === call.name && !c.task_id));
    if (index >= 0) used.add(index);
    return step(call, index >= 0 ? index : n);
  });
}

export function correlatedCalls(mission: MissionState, events: MissionEvent[]): JevCallTrace[] {
  return (mission.jev_trace?.calls || []).map(call => {
    const event = currentEvents(mission, events).find(e => (e.payload?.trace?.call_id === call.call_id) && call.call_id);
    return { ...call, task_id: call.task_id || event?.payload?.task_id,
      step_id: call.step_id || event?.payload?.step_id,
      step_index: call.step_index ?? event?.payload?.step_index,
      skill_name: call.skill_name || event?.payload?.skill };
  });
}

export function stageDecisionStatus(call: JevCallTrace, mission?: MissionState): ExecutionStatus {
  const decision = call.adopted_decision || "";
  // Match decision tokens, not substrings such as t[ask_]graph_grounded.
  const waiting = /(?:^|[:_])(?:ask_|clarify(?:_|$)|clarification(?:_|$)|recover|retry|unclear|unsupported|unavailable)/.test(decision);
  if (mission && waiting) {
    if (mission.agent_mode === "jev_shadow") return "done";
    if (mission.status === "CANCELLED") return "cancelled";
    if (mission.status?.startsWith("FAILED")) return "failed";
    if (mission.status === "DONE" || mission.status === "DONE_WITH_STUB") return "done";
    const calls = mission.jev_trace?.calls || [];
    const position = call.call_id ? calls.findIndex(c => c.call_id === call.call_id) : -1;
    if (position >= 0 && calls.slice(position + 1).some(c => c.stage === call.stage
      && c.task_id === call.task_id && c.step_id === call.step_id && stageDecisionStatus(c) === "done")) return "done";
    const index = call.step_index ?? mission.plan?.findIndex(s => Boolean(call.step_id) && s.step_id === call.step_id);
    const latest = (mission.skill_results || []).filter(r => call.step_id
      ? r.step_id === call.step_id : index !== undefined && index >= 0 && r.step_index === index).slice(-1)[0];
    if (latest && ["done", "unmet"].includes(resultStatus(latest))) return "done";
    if (index !== undefined && index >= 0 && (mission.current_step_index ?? -1) > index) return "done";
  }
  if (/reject|cancel|stop_|abort/.test(decision)) return "failed";
  if (waiting) return "waiting";
  return decision ? "done" : "pending";
}

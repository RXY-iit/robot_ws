import type { Intent, JevCallTrace, MissionEvent, MissionState } from "../types";
import {
  correlatedCalls, currentEvents, executionGraph, executionLabels, intentSummary, parsedIntent, parsedTasks, planAccepted,
  stageDecisionStatus, type ExecutionNode, type ExecutionStatus, type StepView, type TaskView,
} from "../lib/missionView";

interface Props { mission: MissionState; events: MissionEvent[] }

function Badge({ status }: { status: ExecutionStatus }) {
  return <span className={`execution-badge ${status}`}>{executionLabels[status]}</span>;
}

export function DecisionDetails({ call }: { call: JevCallTrace }) {
  return <details className="decision-details">
    <summary>{call.stage} · {call.adopted_decision || "未確定"} · {Number(call.latency_ms || 0).toFixed(1)} ms</summary>
    <div className="small">{call.provider || "unknown"}{call.fallback_used ? " · fallback" : ""} · {call.call_id}</div>
    <dl className="decision-answers">{(call.questions || []).map((q, i) => <div key={i}>
      <dt>{q.id}</dt><dd>{typeof q.result === "string" ? q.result : JSON.stringify(q.result) ?? "null"}</dd>
    </div>)}</dl>
  </details>;
}

export function IntentRouting({ mission, events }: Props) {
  const calls = correlatedCalls(mission, events);
  const intent = parsedIntent(mission, events);
  const tasks = parsedTasks(intent);
  const hasOriginalParse = Boolean(mission.parsed_intent || currentEvents(mission, events).some(e => e.type === "intent_parsed"));
  const shadow = mission.agent_mode === "jev_shadow";
  const review = calls.filter(c => c.stage === "plan_review" || c.stage === "pre_step_review").slice(-1)[0];
  const approved = planAccepted(mission, events);
  const reviewStatus = approved ? "done" : review && !shadow ? stageDecisionStatus(review) : mission.status === "PLAN" ? "active" : "pending";
  const parseStatus = intent?.task_type === "ask_human" ? "waiting" : tasks.length ? "done" : mission.status === "PARSE" ? "active" : "pending";
  const selected = mission.parser_trace?.plan_review_selected;
  const source = selected || (mission.agent_mode === "classic" || shadow ? "Classic" : approved ? "未記録" : "未選択");
  return <div className="mission-routing" aria-label="Decision Model Routing">
    <div className="section-head"><h2>Decision Model Routing</h2><span className="small">Mission / Domain Agents</span></div>
    <div className={`stage-row ${parseStatus}`}>
      <div className="stage-heading"><strong>command_parse</strong><Badge status={parseStatus} /></div>
      <div className="small">{mission.parser_trace?.accepted_layer || mission.agent_mode || "Parser"} · {tasks.length} Task</div>
      <ol className="parsed-tasks">{tasks.map((task, i) => <li key={i}>{intentSummary(task)}</li>)}</ol>
      {intent?.task_type === "ask_human" && <p>{mission.question_for_human || "確認が必要です"}</p>}
      <details><summary>{hasOriginalParse ? "Original parse result" : "Current intent / original result unavailable"}</summary><pre>{JSON.stringify(intent || mission.parser_trace || {}, null, 2)}</pre></details>
      {calls.filter(c => c.stage?.startsWith("command_parse")).map((c, i) => <DecisionDetails key={c.call_id || i} call={c} />)}
    </div>
    <div className={`stage-row ${reviewStatus}`}>
      <div className="stage-heading"><strong>pre_step_review</strong><Badge status={reviewStatus} /></div>
      <div>採用: {source} · {approved ? "計画確定" : review?.adopted_decision || "未確定"}</div>
      <div className="small">{review ? `plan_review · ${review.adopted_decision || "未確定"}` : mission.agent_mode === "classic" ? "Classic / JEV review なし" : "plan_review 詳細未記録"}{shadow ? " · Shadow / 比較のみ" : ""}</div>
      {review && <DecisionDetails call={review} />}
    </div>
    {calls.filter(c => !c.stage?.startsWith("command_parse") && !["plan_review", "pre_step_review"].includes(c.stage || "")).map((call, i) => {
      const index = call.step_index;
      const skill = call.skill_name || (index !== undefined ? mission.plan?.[index]?.name : undefined);
      const taskId = call.task_id || (index !== undefined ? mission.plan?.[index]?.task_id : undefined);
      const task = mission.tasks?.find(t => t.task_id === taskId);
      const status = stageDecisionStatus(call, mission);
      const historical = stageDecisionStatus(call) === "waiting" && status !== "waiting";
      return <div key={call.call_id || i} className={`stage-row ${status}`}>
        <div className="stage-heading"><strong>{call.stage}</strong><Badge status={status} /></div>
        <div>{task?.owner || "mission"} · {task?.task_type || "Mission"}{skill ? ` · ${skill}` : ""}</div>
        {(taskId || call.step_id) && <div className="small">{taskId} · {call.step_id}</div>}
        <div>判断: {call.adopted_decision} · {Number(call.latency_ms || 0).toFixed(1)} ms{shadow ? " · Shadow / 比較のみ" : call.stage?.endsWith("step_supervision") ? " · 実行方針への提案" : ""}</div>
        {historical && <div className="small">Historical decision; no longer awaiting this response. Original decision retained below.</div>}
        {status === "waiting" && <div className="small">Mission: {mission.status} · {mission.question_for_human || mission.final_message || "確認待ち"}</div>}
        <DecisionDetails call={call} />
      </div>;
    })}
  </div>;
}

function StepTile({ node, ordinal }: { node: StepView; ordinal: number }) {
  return <details className={`skill-tile ${node.status}`} data-step-id={node.id}>
    <summary><span>{ordinal}. {node.call.name}</span><Badge status={node.status} />
      {node.result?.message && <span className="skill-result-summary">{node.result.message}</span>}
      {node.attempts.length > 1 && <span className="small">{node.attempts.length} attempts</span>}
    </summary>
    <div className="small">{node.call.owner || "mission"} · {node.call.step_id || node.id}</div>
    <pre>{JSON.stringify(node.call.payload || {}, null, 2)}</pre>
    {node.attempts.map((r, i) => <div key={i} className="step-attempt">
      <strong>#{i + 1} · {r.code}</strong><div>{r.message}</div>
      {r.metrics?.outcome && <div>{r.metrics.outcome}</div>}
      {r.metrics?.completion_class && <div>{r.metrics.completion_class}</div>}
    </div>)}
    {node.call.catalog?.precondition_review && <pre>{JSON.stringify(node.call.catalog.precondition_review, null, 2)}</pre>}
    {node.attempts.length > 1 && <div>{node.attempts.length} attempts</div>}
  </details>;
}

function TaskSteps({ node }: { node: TaskView }) {
  return <div className="task-expansion">
    <div className="task-agent-heading"><strong>{node.call.owner || node.record?.owner || "domain"} agent</strong><span>{node.entries.length} Step / Task</span></div>
    {node.record?.state === "REFINING" && <div className="small">Task 計画解析中</div>}
    {node.record?.message && <div className="task-message">{node.record.message}</div>}
    {node.entries.length ? <div className="task-step-grid">{node.entries.map((child, i) => child.kind === "skill"
      ? <StepTile key={child.id} node={child} ordinal={i + 1} />
      : <div className="nested-task" key={child.id}>
          <div className="stage-heading"><strong>{i + 1}. {nodeGoal(child)}</strong><Badge status={child.status} /></div>
          <TaskSteps node={child} />
        </div>)}</div> : <div className="small task-placeholder">{node.status === "pending" ? "未展開" : "実行ステップなし"}</div>}
  </div>;
}

function nodeGoal(node: ExecutionNode): string {
  const payload = node.call.payload || {};
  return intentSummary(node.kind === "task" ? (node.record?.goal || { ...payload, task_type: node.call.name }) as Intent : { task_type: node.call.name });
}

export function MissionTaskFlow({ mission, events }: Props) {
  const nodes = executionGraph(mission, events);
  return <div className="mission-flow" aria-label="ミッションイベント">
    <div className="section-head"><h2>ミッションイベント</h2><span className="small">{nodes.length} Task / Step</span></div>
    {!nodes.length && <div className="small">計画待ち</div>}
    {nodes.map((node, i) => <div className="mission-flow-row" key={node.id}>
      <div className={`mission-node ${node.status}`} data-task-id={node.id}>
        <div className="small">{i + 1} · {node.kind === "task" ? "Task" : "Step"}</div>
        <strong>{nodeGoal(node)}</strong><Badge status={node.status} />
        {node.kind === "task" && node.record?.outcome && <span className="small">{node.record.outcome}</span>}
      </div>
      {node.kind === "task" ? <TaskSteps node={node} /> : <div className="direct-step">
        <StepTile node={node} ordinal={i + 1} />
      </div>}
    </div>)}
  </div>;
}

from __future__ import annotations

import json
import mimetypes
import queue
import threading
from html import escape
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robot_skills.semantic_map import SemanticMap


INDEX_HTML = """<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Robot Harness Dashboard</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f6f7f9;
      --panel: #ffffff;
      --line: #d8dde6;
      --text: #17202a;
      --muted: #657386;
      --ok: #147d64;
      --warn: #a15c00;
      --bad: #b42318;
      --accent: #1d5fd1;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font: 14px/1.45 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    header {
      height: 56px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 0 20px;
      background: var(--panel);
      border-bottom: 1px solid var(--line);
    }
    h1 { font-size: 18px; margin: 0; font-weight: 700; }
    main {
      display: grid;
      grid-template-columns: 340px minmax(520px, 1fr) 380px;
      gap: 12px;
      padding: 12px;
      min-height: calc(100vh - 56px);
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      min-width: 0;
    }
    h2 {
      margin: 0 0 10px;
      font-size: 14px;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0;
    }
    .kv {
      display: grid;
      grid-template-columns: 120px 1fr;
      gap: 6px 8px;
      align-items: center;
    }
    .label { color: var(--muted); }
    .value { font-weight: 650; overflow-wrap: anywhere; }
    .pill {
      display: inline-flex;
      align-items: center;
      min-height: 24px;
      padding: 2px 8px;
      border-radius: 999px;
      background: #eef3ff;
      color: var(--accent);
      font-weight: 650;
    }
    .warn { color: var(--warn); }
    .bad { color: var(--bad); }
    .ok { color: var(--ok); }
    textarea {
      width: 100%;
      min-height: 78px;
      resize: vertical;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 10px;
      font: inherit;
    }
    button {
      border: 1px solid var(--accent);
      color: white;
      background: var(--accent);
      border-radius: 6px;
      height: 36px;
      padding: 0 12px;
      font-weight: 700;
      cursor: pointer;
    }
    button.secondary {
      color: var(--accent);
      background: white;
    }
    .row { display: flex; gap: 8px; margin-top: 8px; }
    .quick-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      margin-top: 10px;
    }
    .quick-grid button {
      color: var(--accent);
      background: white;
      border-color: var(--line);
      font-weight: 650;
    }
    .choice-panel {
      margin-top: 12px;
      border: 1px solid #f0c36a;
      background: #fff8e8;
      border-radius: 8px;
      padding: 10px;
    }
    .choice-panel strong { display: block; margin-bottom: 8px; color: var(--warn); }
    .choice-buttons { display: grid; grid-template-columns: 1fr; gap: 8px; }
    .choice-buttons button {
      color: var(--accent);
      background: white;
      border-color: #d8b45b;
      text-align: left;
      height: auto;
      min-height: 36px;
      padding: 8px 10px;
    }
    .summary {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
      margin-bottom: 12px;
    }
    .summary-card, .block {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fbfcfe;
      padding: 10px;
    }
    .summary-card strong {
      display: block;
      color: var(--muted);
      font-size: 12px;
      font-weight: 700;
      margin-bottom: 4px;
    }
    .summary-card span {
      display: block;
      font-size: 15px;
      font-weight: 750;
      overflow-wrap: anywhere;
    }
    .health-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
      margin: 8px 0 14px;
    }
    .health-card {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fbfcfe;
      padding: 10px;
      min-height: 72px;
    }
    .health-card strong {
      display: flex;
      justify-content: space-between;
      gap: 8px;
      font-size: 12px;
      color: var(--muted);
      margin-bottom: 5px;
    }
    .health-card span {
      display: block;
      font-size: 17px;
      font-weight: 800;
    }
    .health-card small {
      display: block;
      margin-top: 4px;
      color: var(--muted);
      overflow-wrap: anywhere;
    }
    .health-card.ok { background: #eefaf6; border-color: #9ad8c5; }
    .health-card.warn { background: #fff8e8; border-color: #e7c16e; }
    .health-card.bad { background: #fff1f0; border-color: #f0a29b; }
    .health-card.unknown { background: #f4f6f8; border-color: var(--line); }
    .block-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 8px;
      margin-bottom: 12px;
    }
    .block {
      min-height: 84px;
      transition: background 120ms ease, border-color 120ms ease;
    }
    .block.active {
      background: #eaf2ff;
      border-color: #78a8ff;
      box-shadow: inset 3px 0 0 var(--accent);
    }
    .block.done {
      background: #eefaf6;
      border-color: #9ad8c5;
    }
    .block.failed {
      background: #fff1f0;
      border-color: #f0a29b;
    }
    .block.waiting {
      background: #fff8e8;
      border-color: #e7c16e;
    }
    .block-title {
      display: flex;
      justify-content: space-between;
      gap: 8px;
      font-weight: 750;
      margin-bottom: 4px;
    }
    .block-meta, .block-detail {
      color: var(--muted);
      font-size: 12px;
      overflow-wrap: anywhere;
    }
    .block-detail {
      margin-top: 6px;
      color: #2c3846;
    }
    .result-images {
      display: grid;
      grid-template-columns: 1fr;
      gap: 12px;
      margin: 8px 0 12px;
    }
    .result-image {
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
      background: #fbfcfe;
    }
    .result-image.matched {
      border-color: #58b88a;
      box-shadow: inset 0 0 0 2px rgba(20, 125, 100, 0.18);
    }
    .result-image img {
      display: block;
      width: 100%;
      aspect-ratio: 16 / 9;
      object-fit: contain;
      background: #eef1f5;
    }
    .result-image div {
      padding: 6px 8px;
      color: var(--muted);
      font-size: 12px;
      overflow-wrap: anywhere;
    }
    .section-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      margin: 14px 0 8px;
    }
    .section-head h2 { margin: 0; }
    .small {
      font-size: 12px;
      color: var(--muted);
    }
    .timeline {
      display: flex;
      flex-direction: column;
      gap: 8px;
      max-height: 520px;
      overflow: auto;
    }
    .event {
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 8px;
      background: #fbfcfe;
    }
    details {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 8px 10px;
      background: #fbfcfe;
    }
    summary {
      cursor: pointer;
      font-weight: 750;
      color: var(--muted);
    }
    .event-title {
      display: flex;
      justify-content: space-between;
      gap: 8px;
      font-weight: 700;
    }
    pre {
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      margin: 8px 0 0;
      color: #26323f;
      font-size: 12px;
    }
    @media (max-width: 1100px) {
      main { grid-template-columns: 1fr; }
      .summary, .block-grid, .result-images { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <header>
    <h1>Robot Harness ダッシュボード</h1>
    <span id="conn" class="pill warn">接続中</span>
  </header>
  <main>
    <section>
      <h2>コマンド</h2>
      <textarea id="cmd">__DEFAULT_COMMAND__</textarea>
      <div class="row">
        <button id="send">送信</button>
        <button id="cancel" class="secondary">キャンセル</button>
      </div>
      <div class="quick-grid">
        __QUICK_COMMANDS__
      </div>
      <h2 style="margin-top:18px">Robot Health</h2>
      <div id="healthGrid" class="health-grid">
        <div class="health-card unknown"><strong>Overall</strong><span>UNKNOWN</span><small>waiting for health state</small></div>
      </div>
      <h2 style="margin-top:18px">ロボット状態</h2>
      <div class="kv">
        <div class="label">robot_mode</div><div id="robot_mode" class="value">UNKNOWN</div>
        <div class="label">safety</div><div id="safety_status" class="value">UNKNOWN</div>
        <div class="label">localization</div><div id="localization_warning" class="value">UNKNOWN</div>
        <div class="label">drift_m</div><div id="stationary_drift_m" class="value">UNKNOWN</div>
        <div class="label">agent</div><div id="agent_status" class="value">UNKNOWN</div>
        <div class="label">prompt</div><div id="human_prompt" class="value warn">なし</div>
      </div>
      <div id="human_choices"></div>
    </section>
    <section>
      <h2>ミッション</h2>
      <div class="summary">
        <div class="summary-card"><strong>状態</strong><span id="mission_status">UNKNOWN</span></div>
        <div class="summary-card"><strong>コマンド</strong><span id="mission_command">なし</span></div>
        <div class="summary-card"><strong>意図</strong><span id="mission_intent">なし</span></div>
        <div class="summary-card"><strong>結果</strong><span id="mission_result">実行中</span></div>
      </div>
      <div class="section-head">
        <h2>Intent Routing</h2>
        <span id="routingSource" class="small">解析待ち</span>
      </div>
      <div id="routingBlocks" class="block-grid"></div>
      <div class="section-head">
        <h2>エージェント</h2>
        <span class="small">実行中の block を強調表示</span>
      </div>
      <div id="agentBlocks" class="block-grid"></div>
      <div class="section-head">
        <h2>スキル実行計画</h2>
        <span id="stepCounter" class="small">0 / 0</span>
      </div>
      <div id="skillBlocks" class="block-grid"></div>
      <div class="section-head">
        <h2>検出結果画像</h2>
        <span class="small">検出に使われた画像</span>
      </div>
      <div id="resultImages" class="result-images"></div>
      <details>
        <summary>Raw mission state</summary>
        <pre id="state">{}</pre>
      </details>
    </section>
    <section>
      <h2>ミッションイベント</h2>
      <div id="events" class="timeline"></div>
    </section>
  </main>
  <script>
    const stateEl = document.getElementById('state');
    const eventsEl = document.getElementById('events');
    const agentBlocksEl = document.getElementById('agentBlocks');
    const skillBlocksEl = document.getElementById('skillBlocks');
    const resultImagesEl = document.getElementById('resultImages');
    const routingBlocksEl = document.getElementById('routingBlocks');
    const healthGridEl = document.getElementById('healthGrid');
    const latestSkillResults = new Map();
    const latestMissionEvents = [];
    let latestMission = null;
    let latestMissionId = null;
    let latestHealth = null;
    let latestNavigationProgress = null;
    let latestNavigationRouteState = null;

    const agentOrder = ['Mission Agent', 'Navigation Agent', 'Health / Recovery Agent', 'Perception Agent'];

    function escapeHtml(value) {
      return String(value ?? '')
        .replaceAll('&', '&amp;')
        .replaceAll('<', '&lt;')
        .replaceAll('>', '&gt;')
        .replaceAll('"', '&quot;')
        .replaceAll("'", '&#039;');
    }

    function agentForSkill(name) {
      if (['resolve_semantic_target', 'goto_pose', 'move_relative', 'move_relative_xy', 'rotate_relative', 'await_robot_auto_mode', 'await_robot_stationary', 'cancel_motion'].includes(name)) return 'Navigation Agent';
      if (['check_localization_health', 'recover_localization_spin'].includes(name)) return 'Health / Recovery Agent';
      if (['set_camera_pose', 'capture_observation', 'detect_known_object'].includes(name)) return 'Perception Agent';
      return 'Mission Agent';
    }

    function agentLabel(name) {
      return {
        'Mission Agent': 'Mission Agent',
        'Navigation Agent': 'Navigation Agent',
        'Health / Recovery Agent': 'Health / Recovery Agent',
        'Perception Agent': 'Perception Agent',
      }[name] || name;
    }

    function statusLabel(value) {
      const text = String(value || '');
      return {
        RECEIVED: '受信',
        PARSE: '解析中',
        PLAN: '計画中',
        EXECUTE_STEP: '実行中',
        PAUSED_HEALTH: 'Health一時停止',
        RECOVERING: '復旧中',
        RESUMING: '再開中',
        NEEDS_HUMAN: '人の確認待ち',
        FAILED_SKILL: '失敗',
        DONE_WITH_STUB: 'Stubで完了',
        DONE: '完了',
      }[text] || text || 'UNKNOWN';
    }

    function statusClass(status, success) {
      const text = String(status || '').toUpperCase();
      if (text.includes('FAILED') || text.includes('NEEDS_HUMAN') || success === false) return 'failed';
      if (text.includes('DONE') || text === 'OK' || success === true) return 'done';
      if (text.includes('WAIT') || text.includes('HUMAN')) return 'waiting';
      return '';
    }

    function shortNumber(value, digits = 3) {
      const n = Number(value);
      return Number.isFinite(n) ? n.toFixed(digits) : null;
    }

    function navigationProgressText(progress) {
      if (!progress) return '待機';
      const phase = progress.phase || 'navigation';
      const stalled = shortNumber(progress.stalled_sec, 1);
      const limit = shortNumber(progress.progress_watchdog_sec ?? progress.effective_timeout_sec, 1);
      const distance = shortNumber(progress.current_distance_m, 2);
      const best = shortNumber(progress.best_distance_m, 2);
      const parts = [`phase=${phase}`];
      if (stalled && limit) parts.push(`timeout=${stalled}/${limit}s`);
      else if (limit) parts.push(`limit=${limit}s`);
      if (distance) parts.push(`dist=${distance}m`);
      if (best) parts.push(`best=${best}m`);
      return parts.join(' ');
    }

    function navigationRouteText(route) {
      if (!route) return 'route=none';
      const phase = route.phase || 'route';
      const items = Array.isArray(route.items) ? route.items : [];
      const active = Number.isInteger(route.active_index) ? items[route.active_index] : items.find(item => item.status === 'active');
      const parts = [`route=${phase}`];
      if (active) {
        const name = active.name || active.kind || active.id || 'goal';
        parts.push(`active=${name}`);
        if (active.status) parts.push(`status=${active.status}`);
        if (active.xy_tolerance_m != null) parts.push(`tol=${active.xy_tolerance_m}m`);
      }
      if (items.length) {
        const summary = items.map(item => `${item.name || item.kind}:${item.status || 'pending'}`).join(' > ');
        parts.push(summary);
      }
      return parts.join(' / ');
    }

    function missionEventsOf(type) {
      return latestMissionEvents.filter(event => event && event.type === type);
    }

    function navigationStatusLines(call, result) {
      const lines = [];
      const payload = call?.payload || {};
      const metrics = result?.metrics || {};
      const profile = metrics.tolerance_profile || payload.tolerance_profile;
      const timeoutPolicy = metrics.timeout_policy || (payload.dynamic_timeout ? 'dynamic' : null);
      if (timeoutPolicy) {
        const requested = shortNumber(metrics.requested_timeout_sec ?? payload.timeout_sec, 0);
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
        lines.push(`Annotation: ${annotation.passage_id}${annotation.direction ? ` ${annotation.direction}` : ''}`);
        if (annotation.waypoint_reachability) {
          const reach = annotation.waypoint_reachability;
          lines.push(`Waypoint reachability: ${reach.available ? 'ok' : 'failed'}${reach.reason ? ` ${reach.reason}` : ''}`);
        }
        if (annotation.waypoint_failure_action) {
          lines.push(`Waypoint action: ${annotation.waypoint_failure_action}`);
        }
      }
      if (metrics.near_goal_recovery) {
        lines.push(`Near-goal recovery: ${metrics.near_goal_recovery.success ? 'ok' : 'failed'}`);
      }
      if (metrics.relaxed_tolerance) {
        lines.push('Relaxed tolerance: accepted');
      }
      if (metrics.alternate_goal_sampling) {
        lines.push(`Alternate goal: ${metrics.alternate_goal_sampling.available ? 'selected' : 'none'}`);
      }
      if (profile && profile !== 'default') {
        const xy = metrics.xy_goal_tolerance_m ?? payload.xy_goal_tolerance_m;
        const yaw = metrics.yaw_goal_tolerance_rad ?? payload.yaw_goal_tolerance_rad;
        lines.push(`Tolerance: ${profile}${xy != null ? ` xy=${xy}m` : ''}${yaw != null ? ` yaw=${yaw}rad` : ''}`);
      }
      if (metrics.final_xy_error_m != null || metrics.final_yaw_error_rad != null) {
        const xy = shortNumber(metrics.final_xy_error_m);
        const yaw = shortNumber(metrics.final_yaw_error_rad);
        lines.push(`Final error: xy=${xy ?? '-'}m yaw=${yaw ?? '-'}rad`);
      }
      for (const event of missionEventsOf('navigation_tolerance_adjusted')) {
        const selected = event.payload?.selected_profile || {};
        const step = event.payload?.step_index;
        if (step == null || step === (latestMission?.current_step_index ?? step) || event.payload?.skill === call?.name) {
          lines.push(`Adjusted by Health/Recovery: ${selected.name || 'profile'}`);
        }
      }
      return lines;
    }

    function recoveryStatusLines() {
      const lines = [];
      for (const event of latestMissionEvents) {
        if (event.type === 'health_cancel_requested') {
          const summary = event.payload?.health?.components?.local_planner?.summary
            || event.payload?.health?.components?.localization?.summary
            || event.payload?.reason
            || 'health issue';
          lines.push(`Health cancel: ${summary}`);
        }
        if (event.type === 'mission_paused_health') {
          const summary = event.payload?.health?.components?.local_planner?.summary
            || event.payload?.health?.components?.localization?.summary
            || event.payload?.message
            || 'paused';
          lines.push(`Paused: ${summary}`);
        }
        if (event.type === 'mission_resuming') {
          lines.push('Recovery OK: mission resumed');
        }
      }
      return lines.slice(-3);
    }

    function skillExtraLines(call, result) {
      const lines = [];
      if (call?.name === 'goto_pose') lines.push(...navigationStatusLines(call, result));
      if (call?.name === 'recover_localization_spin') {
        const action = result?.metrics?.recovery_action;
        if (action) lines.push(`Recovery action: ${action}`);
      }
      if (call?.name === 'detect_known_object') {
        const detections = result?.metrics?.detections || [];
        const matched = detections.filter(item => item.matched_query || (item.matched_tokens || []).length).length;
        if (detections.length) lines.push(`Detections: ${matched}/${detections.length} matched`);
      }
      return lines;
    }

    function alignPlanResults(plan, skillResults) {
      const aligned = [];
      const results = Array.isArray(skillResults) ? skillResults : [];
      const byStep = new Map();
      for (const result of results) {
        if (Number.isInteger(result?.step_index)) byStep.set(result.step_index, result);
      }
      let cursor = 0;
      for (let stepIndex = 0; stepIndex < (plan || []).length; stepIndex += 1) {
        const call = plan[stepIndex];
        let matched = byStep.get(stepIndex) || null;
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

    function updateField(id, value) {
      const el = document.getElementById(id);
      el.textContent = value || 'UNKNOWN';
      el.classList.toggle('ok', value === 'AUTO' || value === 'OK');
      el.classList.toggle('bad', value === 'ESTOP');
    }

    function levelClass(level) {
      const normalized = String(level || 'UNKNOWN').toLowerCase();
      if (['ok'].includes(normalized)) return 'ok';
      if (['warn'].includes(normalized)) return 'warn';
      if (['bad'].includes(normalized)) return 'bad';
      return 'unknown';
    }

    function renderHealth(data) {
      latestHealth = data;
      const components = data.components || {};
      const cards = [
        ['Overall', data.overall || 'UNKNOWN', data.recommended_action || 'check_system'],
        ['Mode', components.mode?.level || 'UNKNOWN', components.mode?.summary || 'UNKNOWN'],
        ['Safety', components.safety?.level || 'UNKNOWN', components.safety?.summary || 'UNKNOWN'],
        ['Localization', components.localization?.level || 'UNKNOWN', components.localization?.summary || 'UNKNOWN'],
        ['Local Planner', components.local_planner?.level || 'UNKNOWN', components.local_planner?.summary || 'UNKNOWN'],
        ['Nav Timeout', latestNavigationProgress ? 'OK' : 'UNKNOWN', navigationProgressText(latestNavigationProgress)],
      ];
      healthGridEl.innerHTML = cards.map(([name, level, summary]) => `
        <div class="health-card ${levelClass(level)}">
          <strong><span>${escapeHtml(name)}</span><span>${escapeHtml(level)}</span></strong>
          <span>${escapeHtml(level)}</span>
          <small>${escapeHtml(summary)}</small>
        </div>
      `).join('');
      const localization = components.localization || {};
      updateField('localization_warning', localization.summary || 'UNKNOWN');
      const drift = localization.details?.stationary_drift_m;
      updateField('stationary_drift_m', Number.isFinite(drift) ? drift.toFixed(3) : 'UNKNOWN');
      if (latestMission) renderMission(latestMission);
    }

    function addEvent(data) {
      if (data.type === 'agent_heartbeat') {
        updateField('robot_mode', data.robot_mode);
        updateField('safety_status', data.safety_status);
        updateField('localization_warning', data.localization_warning || 'UNKNOWN');
        updateField('stationary_drift_m', Number.isFinite(data.stationary_drift_m) ? data.stationary_drift_m.toFixed(3) : 'UNKNOWN');
      }
      const item = document.createElement('div');
      item.className = 'event';
      const type = data.type || 'event';
      const time = data.time || new Date().toISOString();
      item.innerHTML = `<div class="event-title"><span>${type}</span><span>${time}</span></div><pre>${JSON.stringify(data, null, 2)}</pre>`;
      eventsEl.prepend(item);
      while (eventsEl.children.length > 80) eventsEl.removeChild(eventsEl.lastChild);
    }

    function renderMission(state) {
      if (state.mission_id && state.mission_id !== latestMissionId) {
        latestSkillResults.clear();
        latestMissionEvents.length = 0;
        latestNavigationProgress = null;
        latestNavigationRouteState = null;
        latestMissionId = state.mission_id;
      }
      latestMission = state;
      stateEl.textContent = JSON.stringify(state, null, 2);
      const intent = state.intent || {};
      document.getElementById('mission_status').textContent = statusLabel(state.status);
      document.getElementById('mission_command').textContent = state.user_text || 'なし';
      document.getElementById('mission_intent').textContent = intent.task_type
        ? `${intent.task_type}${intent.target ? ' -> ' + intent.target : ''}`
        : 'なし';
      document.getElementById('mission_result').textContent = state.final_message || state.question_for_human || '実行中';
      renderIntentRouting(state);

      const plan = state.plan || [];
      const current = Number.isInteger(state.current_step_index) ? state.current_step_index : 0;
      document.getElementById('stepCounter').textContent = plan.length
        ? `ステップ ${Math.min(current + 1, plan.length)} / ${plan.length}`
        : 'ステップ 0 / 0';

      const agentStatus = {};
      for (const name of agentOrder) agentStatus[name] = {state: '', detail: '待機'};
      if (state.status === 'PARSE' || state.status === 'PLAN' || state.status === 'RECEIVED') {
        agentStatus['Mission Agent'] = {state: 'active', detail: statusLabel(state.status)};
      }
      plan.forEach((call, index) => {
        const agent = agentForSkill(call.name);
        if (index < current) agentStatus[agent] = {state: 'done', detail: '関連ステップ完了'};
        if (index === current && !String(state.status || '').includes('DONE')) {
          agentStatus[agent] = {state: 'active', detail: call.name};
        }
      });
      if (String(state.status || '').includes('NEEDS_HUMAN')) {
        agentStatus['Mission Agent'] = {state: 'waiting', detail: state.question_for_human || '人の確認待ち'};
      }
      if (String(state.status || '').includes('FAILED')) {
        agentStatus['Mission Agent'] = {state: 'failed', detail: state.final_message || '失敗'};
      }
      if (String(state.status || '').includes('DONE')) {
        for (const name of agentOrder) {
          if (agentStatus[name].detail !== '待機') agentStatus[name].state = 'done';
        }
      }
      if (latestNavigationProgress && !String(state.status || '').includes('DONE')) {
        agentStatus['Navigation Agent'] = {
          state: 'active',
          detail: `${navigationProgressText(latestNavigationProgress)} / ${navigationRouteText(latestNavigationRouteState)}`,
        };
      } else if (latestNavigationRouteState && !String(state.status || '').includes('DONE')) {
        agentStatus['Navigation Agent'] = {
          state: 'active',
          detail: navigationRouteText(latestNavigationRouteState),
        };
      }
      const recoveryLines = recoveryStatusLines();
      if (recoveryLines.length) {
        const currentHealth = latestHealth || {};
        const currentLocalization = currentHealth.components?.localization || {};
        const currentOverall = currentHealth.overall || 'UNKNOWN';
        const healthIsOk = currentOverall === 'OK';
        const recoveryState = healthIsOk
          ? 'done'
          : (String(state.status || '').includes('DONE') ? 'done' : 'active');
        const liveDetail = currentOverall !== 'UNKNOWN'
          ? `Current health: ${currentOverall} / Localization: ${currentLocalization.level || 'UNKNOWN'} ${currentLocalization.summary || ''}`.trim()
          : recoveryLines.join(' / ');
        agentStatus['Health / Recovery Agent'] = {
          state: recoveryState,
          detail: liveDetail,
        };
      }

      agentBlocksEl.innerHTML = agentOrder.map(name => `
        <div class="block ${agentStatus[name].state}">
          <div class="block-title"><span>${agentLabel(name)}</span><span>${agentStatus[name].state || 'idle'}</span></div>
          <div class="block-meta">${escapeHtml(agentStatus[name].detail)}</div>
        </div>
      `).join('');

      const alignedResults = alignPlanResults(plan, state.skill_results || []);
      skillBlocksEl.innerHTML = plan.map((call, index) => {
        const result = alignedResults[index];
        let cls = '';
        if (index < current) cls = 'done';
        if (index === current && !String(state.status || '').includes('DONE')) cls = 'active';
        if (result) cls = statusClass(result.code, result.success) || cls;
        const agent = agentForSkill(call.name);
        const detail = result
          ? `${result.code}: ${result.message || ''}`
          : (call.reason || 'pending');
        const payload = call.payload ? JSON.stringify(call.payload) : '{}';
        const extraLines = skillExtraLines(call, result);
        return `
          <div class="block ${cls}">
            <div class="block-title"><span>${index + 1}. ${escapeHtml(call.name)}</span><span>${escapeHtml(agentLabel(agent))}</span></div>
            <div class="block-meta">${escapeHtml(payload)}</div>
            <div class="block-detail">${escapeHtml(detail)}</div>
            ${extraLines.map(line => `<div class="block-detail">${escapeHtml(line)}</div>`).join('')}
          </div>
        `;
      }).join('') || '<div class="block waiting"><div class="block-title"><span>計画なし</span><span>待機</span></div><div class="block-meta">コマンド待ち。</div></div>';
      renderHumanChoices(state);
      renderResultImages(state);
    }

    function renderIntentRouting(state) {
      const trace = state.parser_trace || {};
      const intent = state.intent || {};
      const steps = Array.isArray(trace.route_steps) ? trace.route_steps : [];
      const accepted = trace.accepted_layer || trace.source || '未確定';
      document.getElementById('routingSource').textContent = intent.task_type
        ? `${accepted} -> ${intent.task_type}`
        : '解析待ち';
      if (!steps.length) {
        routingBlocksEl.innerHTML = '<div class="block waiting"><div class="block-title"><span>解析待ち</span><span>waiting</span></div><div class="block-meta">Intent parser の結果がここに表示されます。</div></div>';
        return;
      }
      routingBlocksEl.innerHTML = steps.map(step => {
        const status = String(step.status || '');
        const cls = status === 'accepted' ? 'done' : (status === 'rejected' ? 'waiting' : 'active');
        const score = Number.isFinite(step.score) ? ` score=${step.score.toFixed(3)}` : '';
        const task = step.task_type ? ` task=${step.task_type}` : '';
        const model = step.model ? ` model=${step.model}` : '';
        const engine = step.engine ? ` engine=${step.engine}` : '';
        const template = step.template_id ? ` template=${step.template_id}` : '';
        const reason = step.reason ? ` reason=${step.reason}` : '';
        const error = step.error ? ` error=${step.error}` : '';
        return `
          <div class="block ${cls}">
            <div class="block-title"><span>${escapeHtml(step.layer || 'Parser')}</span><span>${escapeHtml(status || 'checked')}</span></div>
            <div class="block-meta">${escapeHtml([task, score, engine, model, template, reason].join('').trim() || 'checked')}</div>
            ${error ? `<div class="block-detail">${escapeHtml(error)}</div>` : ''}
          </div>
        `;
      }).join('');
    }

    function renderHumanChoices(state) {
      const choicesEl = document.getElementById('human_choices');
      if (!String(state.status || '').includes('NEEDS_HUMAN')) {
        choicesEl.innerHTML = '';
        return;
      }
      const resolveResult = (state.skill_results || []).find(item =>
        item.skill_name === 'resolve_semantic_target' &&
        item.code === 'NEEDS_HUMAN' &&
        item.metrics &&
        item.metrics.alternatives &&
        item.metrics.alternatives.length
      );
      const ambiguityChoices = (((state.intent || {}).constraints || {}).ambiguity || {}).choices || [];
      if (ambiguityChoices.length) {
        const buttons = ambiguityChoices.map((choice, index) => {
          const command = choice.command || '';
          const label = choice.label || `選択 ${index + 1}`;
          return `<button data-choice-command="${escapeHtml(command)}">${escapeHtml(label)}<br><small>${escapeHtml(command)}</small></button>`;
        }).join('');
        choicesEl.innerHTML = `
          <div class="choice-panel">
            <strong>候補を選択してください</strong>
            <div class="block-meta">${escapeHtml(state.question_for_human || state.final_message || '')}</div>
            <div class="choice-buttons" style="margin-top:8px">
              ${buttons}
              <button data-cancel-choice="1">キャンセル</button>
            </div>
          </div>
        `;
        choicesEl.querySelectorAll('[data-choice-command]').forEach(btn => {
          btn.addEventListener('click', () => {
            const command = btn.dataset.choiceCommand;
            if (!command) return;
            document.getElementById('cmd').value = command;
            post('/command', {text: command});
          });
        });
        choicesEl.querySelector('[data-cancel-choice]')?.addEventListener('click', () => post('/cancel', {text: 'cancel'}));
        return;
      }
      if (!resolveResult) {
        choicesEl.innerHTML = `
          <div class="choice-panel">
            <strong>人の確認が必要です</strong>
            <div class="block-meta">${escapeHtml(state.question_for_human || state.final_message || '')}</div>
            <div class="choice-buttons" style="margin-top:8px"><button data-cancel-choice="1">キャンセル</button></div>
          </div>
        `;
        choicesEl.querySelector('[data-cancel-choice]')?.addEventListener('click', () => post('/cancel', {text: 'cancel'}));
        return;
      }
      const candidates = [resolveResult.metrics.object_id, ...(resolveResult.metrics.alternatives || [])].filter(Boolean);
      const buttons = candidates.map((id, index) => {
        const command = clarificationCommand(state.intent || {}, id);
        return `<button data-choice-command="${escapeHtml(command)}">選択 ${index + 1}: ${escapeHtml(id)}<br><small>${escapeHtml(command)}</small></button>`;
      }).join('');
      choicesEl.innerHTML = `
        <div class="choice-panel">
          <strong>候補を選択してください</strong>
          <div class="block-meta">${escapeHtml(state.question_for_human || state.final_message || '')}</div>
          <div class="choice-buttons" style="margin-top:8px">
            ${buttons}
            <button data-cancel-choice="1">キャンセル</button>
          </div>
        </div>
      `;
      choicesEl.querySelectorAll('[data-choice-command]').forEach(btn => {
        btn.addEventListener('click', () => {
          const command = btn.dataset.choiceCommand;
          document.getElementById('cmd').value = command;
          post('/command', {text: command});
        });
      });
      choicesEl.querySelector('[data-cancel-choice]')?.addEventListener('click', () => post('/cancel', {text: 'cancel'}));
    }

    function clarificationCommand(intent, targetId) {
      const task = String(intent.task_type || '');
      if (task === 'inspect_place') return `inspect ${targetId}`;
      if (task === 'search_object_near_place') {
        const query = intent.object_query || 'requested object';
        return `check ${targetId} for ${query}`;
      }
      return `go to ${targetId}`;
    }

    function artifactUrl(path) {
      return `/artifact?path=${encodeURIComponent(path)}`;
    }

    function renderResultImages(state) {
      const seen = new Set();
      const matches = [];
      for (const result of state.skill_results || []) {
        if (result.skill_name !== 'detect_known_object') continue;
        for (const item of (result.metrics && result.metrics.annotated_images) || []) {
          const path = item.annotated_image_path || item.image_path;
          if (!path || seen.has(path)) continue;
          if (!item.matched) continue;
          seen.add(path);
          matches.push({
            path,
            label: item.matched ? `MATCH ${item.top_label || 'detection'}` : (item.top_label || 'candidate'),
            text: `${item.query || ''} detections:${item.detection_count || 0} matched:${item.matched_count || 0}`,
            confidence: item.top_confidence,
            matched: Boolean(item.matched),
          });
        }
      }
      resultImagesEl.innerHTML = matches.map(item => `
        <div class="result-image ${item.matched ? 'matched' : ''}">
          <img src="${artifactUrl(item.path)}" alt="${escapeHtml(item.label)}" loading="lazy" />
          <div>${escapeHtml(item.label)} ${Number.isFinite(item.confidence) ? item.confidence.toFixed(2) : ''}<br>${escapeHtml(item.text || item.path)}</div>
        </div>
      `).join('') || '<div class="block waiting"><div class="block-title"><span>画像なし</span><span>待機</span></div><div class="block-meta">検出に成功すると、該当 observation がここに表示されます。</div></div>';
    }

    function clearMissionForNewEvent(event) {
      latestSkillResults.clear();
      latestMissionEvents.length = 0;
      latestNavigationProgress = null;
      latestNavigationRouteState = null;
      latestMissionId = event.mission_id;
      latestMission = null;
      document.getElementById('mission_status').textContent = '受信';
      document.getElementById('mission_command').textContent = event.payload?.text || 'なし';
      document.getElementById('mission_intent').textContent = '解析待ち';
      document.getElementById('mission_result').textContent = '実行中';
      document.getElementById('routingSource').textContent = '解析待ち';
      document.getElementById('stepCounter').textContent = 'ステップ 0 / 0';
      routingBlocksEl.innerHTML = '<div class="block active"><div class="block-title"><span>Intent Routing</span><span>active</span></div><div class="block-meta">Deterministic -> Embedding -> LLM Fallback の順に解析しています。</div></div>';
      agentBlocksEl.innerHTML = agentOrder.map((name, index) => `
        <div class="block ${index === 0 ? 'active' : ''}">
          <div class="block-title"><span>${agentLabel(name)}</span><span>${index === 0 ? 'active' : 'idle'}</span></div>
          <div class="block-meta">${index === 0 ? 'コマンド受信' : '待機'}</div>
        </div>
      `).join('');
      skillBlocksEl.innerHTML = '<div class="block waiting"><div class="block-title"><span>計画作成中</span><span>Mission Agent</span></div><div class="block-meta">Intent と Skill Plan を生成しています。</div></div>';
      resultImagesEl.innerHTML = '<div class="block waiting"><div class="block-title"><span>画像なし</span><span>待機</span></div><div class="block-meta">検出に成功すると表示されます。</div></div>';
      document.getElementById('human_choices').innerHTML = '';
      stateEl.textContent = '{}';
    }

    const es = new EventSource('/events');
    es.onopen = () => document.getElementById('conn').textContent = 'connected';
    es.onerror = () => document.getElementById('conn').textContent = 'reconnecting';
    es.onmessage = (msg) => {
      const data = JSON.parse(msg.data);
      if (data.topic === '/agent/mission_state') renderMission(JSON.parse(data.data));
      if (data.topic === '/agent/health_state') renderHealth(JSON.parse(data.data));
      if (data.topic === '/agent/navigation_progress') {
        latestNavigationProgress = JSON.parse(data.data);
        if (latestHealth) renderHealth(latestHealth);
        if (latestMission) renderMission(latestMission);
      }
      if (data.topic === '/agent/navigation_route_state') {
        latestNavigationRouteState = JSON.parse(data.data);
        if (latestMission) renderMission(latestMission);
      }
      if (data.topic === '/agent/mission_events') {
        const event = JSON.parse(data.data);
        if (event.type === 'mission_received' && event.mission_id !== latestMissionId) {
          clearMissionForNewEvent(event);
        }
        if (event.mission_id && event.mission_id === latestMissionId) {
          latestMissionEvents.push(event);
          while (latestMissionEvents.length > 120) latestMissionEvents.shift();
        }
        if (event.type === 'skill_finished' && event.payload && event.payload.skill_name) {
          latestSkillResults.set(event.payload.skill_name, event.payload);
          if (latestMission) renderMission(latestMission);
        }
        addEvent(event);
      }
      if (data.topic === '/agent/status') updateField('agent_status', data.data);
      if (data.topic === '/agent/human_prompt') updateField('human_prompt', data.data);
      if (data.topic === '/robot_mode') updateField('robot_mode', data.data);
      if (data.topic === '/safety_status') updateField('safety_status', data.data);
    };

    async function post(path, body) {
      await fetch(path, {method: 'POST', headers: {'content-type': 'application/json'}, body: JSON.stringify(body)});
    }
    document.getElementById('send').onclick = () => post('/command', {text: document.getElementById('cmd').value});
    document.getElementById('cancel').onclick = () => post('/cancel', {text: 'cancel'});
    document.querySelectorAll('[data-command]').forEach(btn => {
      btn.onclick = () => {
        document.getElementById('cmd').value = btn.dataset.command;
        post('/command', {text: btn.dataset.command});
      };
    });
  </script>
</body>
</html>
"""


class DashboardState:
    def __init__(self):
        self.lock = threading.Lock()
        self.clients: list[queue.Queue] = []
        self.latest: dict[str, str] = {}
        self.command_pub = None
        self.cancel_pub = None
        self.quick_commands_html, self.default_command = self._build_quick_commands()

    def publish_update(self, topic: str, data: str):
        payload = json.dumps({"topic": topic, "data": data}, ensure_ascii=False)
        with self.lock:
            self.latest[topic] = data
            clients = list(self.clients)
        for client in clients:
            client.put(payload)

    def render_index(self) -> str:
        return (
            INDEX_HTML.replace("__QUICK_COMMANDS__", self.quick_commands_html)
            .replace("__DEFAULT_COMMAND__", escape(self.default_command))
        )

    @staticmethod
    def _build_quick_commands() -> tuple[str, str]:
        commands: list[tuple[str, str]] = []
        try:
            semantic_map = SemanticMap()
            preferred_ids = [
                "sofa",
                "meeting_table",
                "desk_rxy",
                "trash_bin",
                "tool_box",
            ]
            items = semantic_map.objects + semantic_map.places
            preferred_rank = {item_id: index for index, item_id in enumerate(preferred_ids)}
            sorted_items = sorted(
                enumerate(items),
                key=lambda pair: (preferred_rank.get(str(pair[1].get("id", "")), len(preferred_rank)), pair[0]),
            )
            for _, item in sorted_items:
                if item.get("navigation_ready") is False:
                    continue
                names = [str(name) for name in item.get("names", []) if name]
                target = names[0] if names else str(item.get("id", "")).strip()
                if not target:
                    continue
                label = target.replace("_", " ")
                commands.append((f"Go {label}", f"go to {target}"))
                if len(commands) >= 6:
                    break
        except Exception:
            commands = []

        commands.extend(
            [
                ("左を見る", "左を見て"),
                ("少し前進", "前進少し"),
                ("Turn around", "turn around"),
                ("ロボット状態", "ロボット状態"),
                ("既知の場所", "知っている場所を教えて"),
                ("キャンセル", "キャンセル"),
            ]
        )
        buttons = "\n        ".join(
            f'<button data-command="{escape(command, quote=True)}">{escape(label)}</button>' for label, command in commands
        )
        default_command = commands[0][1] if commands else "ロボット状態"
        return buttons, default_command


class DashboardHandler(BaseHTTPRequestHandler):
    state: DashboardState

    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/":
            encoded = self.state.render_index().encode("utf-8")
            self.send_response(200)
            self.send_header("content-type", "text/html; charset=utf-8")
            self.send_header("content-length", str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)
            return
        if parsed.path == "/artifact":
            params = parse_qs(parsed.query)
            requested = (params.get("path") or [""])[0]
            if self._serve_artifact(requested):
                return
            self.send_error(404)
            return
        if parsed.path == "/events":
            self.send_response(200)
            self.send_header("content-type", "text/event-stream")
            self.send_header("cache-control", "no-cache")
            self.end_headers()
            q: queue.Queue = queue.Queue()
            with self.state.lock:
                self.state.clients.append(q)
                latest = dict(self.state.latest)
            try:
                for topic, data in latest.items():
                    self.wfile.write(f"data: {json.dumps({'topic': topic, 'data': data}, ensure_ascii=False)}\n\n".encode("utf-8"))
                    self.wfile.flush()
                while True:
                    payload = q.get()
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
            except Exception:
                with self.state.lock:
                    if q in self.state.clients:
                        self.state.clients.remove(q)
            return
        self.send_error(404)

    def _serve_artifact(self, requested: str) -> bool:
        if not requested:
            return False
        try:
            path = Path(requested).expanduser()
            if not path.is_absolute():
                path = Path.cwd() / path
            path = path.resolve()
            workspace = Path.cwd().resolve()
            debug_root = (workspace / "debug-output").resolve()
            if not path.is_relative_to(debug_root):
                return False
            if not path.is_file():
                return False
            content_type = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
            data = path.read_bytes()
        except Exception:
            return False
        self.send_response(200)
        self.send_header("content-type", content_type)
        self.send_header("content-length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)
        return True

    def do_POST(self):
        length = int(self.headers.get("content-length", "0"))
        body = self.rfile.read(length).decode("utf-8") if length else "{}"
        try:
            data = json.loads(body)
        except json.JSONDecodeError:
            data = {}
        if self.path == "/command":
            text = str(data.get("text", ""))
            if self.state.command_pub is not None:
                self.state.command_pub.publish(String(data=text))
            self.send_response(204)
            self.end_headers()
            return
        if self.path == "/cancel":
            if self.state.cancel_pub is not None:
                self.state.cancel_pub.publish(String(data="cancel"))
            self.send_response(204)
            self.end_headers()
            return
        self.send_error(404)


class DashboardNode(Node):
    def __init__(self):
        super().__init__("agent_dashboard_node")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.state = DashboardState()
        self.state.command_pub = self.create_publisher(String, "/agent/text_command", 10)
        self.state.cancel_pub = self.create_publisher(String, "/agent/cancel", 10)

        for topic in [
            "/agent/status",
            "/agent/mission_events",
            "/agent/mission_state",
            "/agent/health_state",
            "/agent/navigation_progress",
            "/agent/navigation_route_state",
            "/agent/human_prompt",
            "/robot_mode",
            "/safety_status",
        ]:
            self.create_subscription(String, topic, lambda msg, t=topic: self.state.publish_update(t, msg.data), 10)

        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        DashboardHandler.state = self.state
        self.server = ThreadingHTTPServer((host, port), DashboardHandler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(f"Dashboard ready: http://{host}:{port}")

    def destroy_node(self):
        self.server.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

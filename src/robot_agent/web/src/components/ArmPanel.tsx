import { useEffect, useMemo, useState } from "react";
import type { ArmDashboardState } from "../types";

interface ArmPanelProps {
  arm: ArmDashboardState | null;
  sendArmCommand: (command: Record<string, unknown>) => void;
}

function requestId(): string {
  if (typeof crypto !== "undefined" && "randomUUID" in crypto) return crypto.randomUUID();
  return `dashboard-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

function radians(value: number | null): string {
  return value == null ? "—" : `${value.toFixed(3)} rad / ${(value * 180 / Math.PI).toFixed(1)}°`;
}

function age(value: number | null): string {
  return value == null ? "UNKNOWN" : `${value.toFixed(3)} s`;
}

type MetricTone = "ok" | "warn" | "bad" | "neutral";

function numeric(value: unknown): number | null {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function MotorMetric({ label, value, note, tone }: {
  label: string;
  value: string;
  note?: string;
  tone: MetricTone;
}) {
  return (
    <div className={`arm-motor-metric ${tone}`}>
      <span>{label}</span>
      <strong>{value}</strong>
      {note && <small>{note}</small>}
    </div>
  );
}

export function ArmPanel({ arm, sendArmCommand }: ArmPanelProps) {
  const [targets, setTargets] = useState<Record<string, number>>({});
  const [dirty, setDirty] = useState(true);

  const jointSignature = arm?.joints.map((joint) => joint.name).join("|") ?? "";
  const candidateId = arm?.candidate.id ?? null;
  const controlMode = arm?.control_mode ?? (arm?.mode === "MOCK" ? "mock" : "real2sim");
  const codeControlEnabled = controlMode === "code2real&sim" || controlMode === "mock";
  const actualSignature = codeControlEnabled
    ? ""
    : arm?.joints.map((joint) => `${joint.name}:${joint.actual?.toFixed(4) ?? "null"}`).join("|") ?? "";
  useEffect(() => {
    if (!arm) return;
    const next: Record<string, number> = {};
    for (const joint of arm.joints) {
      next[joint.name] = codeControlEnabled
        ? (arm.candidate.positions[joint.name] ?? joint.target)
        : (joint.actual ?? joint.target);
    }
    setTargets(next);
    setDirty(!arm.candidate.valid);
  }, [candidateId, jointSignature, controlMode, actualSignature, codeControlEnabled]);

  const targetComplete = useMemo(
    () => Boolean(arm && arm.joints.every((joint) => Number.isFinite(targets[joint.name]))),
    [arm, targets],
  );
  if (!arm) {
    return (
      <section className="arm-shell">
        <h2>SO101 Arm / SO101アーム</h2>
        <div className="arm-safety bad">Waiting for `/arm/dashboard_state`; all motion remains BLOCKED. / 状態待機中のため、すべての動作はBLOCKEDです。</div>
        <p className="arm-muted">Start the Arm launch first. This page never opens the follower USB directly. / 先にArm launchを起動してください。このページはfollower USBを直接開きません。</p>
      </section>
    );
  }

  const send = (command: Record<string, unknown>) =>
    sendArmCommand({ request_id: requestId(), requested_by: "dashboard", mode: arm.mode, ...command });
  const torqueValues = Object.values(arm.motor_health?.torque_enable ?? {});
  const torqueOnCount = torqueValues.filter((value) => Number(value) === 1).length;
  const torqueInvariantMissing = Object.keys(
    arm.motor_health?.required_torque_missing ?? {},
  );

  const preview = () => {
    if (!targetComplete) return;
    send({ command: "preview", positions: targets });
  };
  const execute = () => {
    if (!candidateId || dirty) return;
    send({
      command: "execute_candidate",
      candidate_id: candidateId,
      duration_sec: Math.max(2.0, arm.candidate.minimum_duration_sec ?? 2.0),
    });
  };
  const prepareSafeShutdown = () => {
    const confirmed = window.confirm(
      "Support the arm first. This will release all six motor torques. Continue?\n\n" +
      "先にアームを支えてください。6軸すべてのトルクを解除します。続行しますか？",
    );
    if (confirmed) send({ command: "safe_shutdown" });
  };
  const confirmTorqueOff = () => window.confirm(
    "Torque OFF can make the arm fall and may push a joint outside the calibrated range. " +
    "Move to home when practical, physically support the arm, then continue.\n\n" +
    "トルクをOFFにするとアームが落下し、関節がキャリブレーション範囲外に入る可能性があります。" +
    "可能であればhomeへ移動し、アームを支えてから続行してください。",
  );
  const releaseHold = () => {
    if (confirmTorqueOff()) send({ command: "release_hold" });
  };
  const setControlMode = (nextMode: string) => {
    if (arm.hold?.holding && nextMode === "real2sim" && !confirmTorqueOff()) return;
    send({ command: "set_mode", control_mode: nextMode });
  };

  return (
    <section className="arm-shell">
      <div className="arm-title-row">
        <div>
          <h2>SO101 Arm Control / SO101アーム制御</h2>
          <div className="arm-muted">RViz: cyan actual, orange candidate. The Dashboard does not render 3D. / RVizでは実位置をシアン、候補位置をオレンジで表示します。Dashboardは3Dを描画しません。</div>
        </div>
        <span className={`pill ${["MOCK_OK", "REAL_READY", "REAL2SIM_READY", "REAL_HOLDING", "TELEOP_ACTIVE"].includes(arm.health) ? "ok" : "warn"}`}>{arm.health}</span>
      </div>

      <div className="arm-safety">{arm.safety_banner}</div>
      {arm.recovery?.required && (
        <div className="arm-safety bad">
          <strong>RECOVERY REQUIRED / 復帰が必要</strong><br />
          Feedback remains active. Select an inward target such as home; outward goals are blocked.
          / フィードバックは継続中です。homeなど内側の目標を選択してください。外向きの目標は拒否されます。
          <div><code>{JSON.stringify(arm.recovery.joints)}</code></div>
        </div>
      )}

      {arm.control_modes && arm.control_modes.length > 0 && (
        <>
          <h2 className="arm-subtitle">Control mode / 制御モード</h2>
          <div className="arm-button-grid">
            {arm.control_modes.map((mode) => (
              <button
                className={mode.id === controlMode ? "" : "secondary"}
                key={mode.id}
                title={[mode.description, mode.runtime_reason].filter(Boolean).join(" — ")}
                disabled={!mode.available && mode.id !== controlMode}
                onClick={() => setControlMode(mode.id)}
              >
                {mode.label}
                <small>{mode.id === controlMode ? "ACTIVE" : mode.available ? "AVAILABLE" : "LEADER NOT READY"}</small>
              </button>
            ))}
          </div>
        </>
      )}

      <div className="arm-summary-grid">
        <div className="summary-card"><strong>Mode / モード</strong><span>{arm.mode}</span></div>
        <div className="summary-card"><strong>Control mode / 制御</strong><span>{controlMode}</span></div>
        <div className="summary-card"><strong>Controller / コントローラ</strong><span>{arm.controller.state}</span></div>
        <div className="summary-card"><strong>Feedback age / 受信経過</strong><span>{age(arm.feedback_age_sec)}</span></div>
        <div className="summary-card"><strong>Owner / 操作権</strong><span>{arm.command_owner}</span></div>
        <div className="summary-card"><strong>Calibration / キャリブレーション</strong><span>{arm.calibration.id}</span></div>
        <div className="summary-card"><strong>Real hardware / 実機</strong><span>{String(arm.hardware.real_hardware_enabled)}</span></div>
        <div className="summary-card"><strong>HOLD / 保持</strong><span>{arm.hold?.state ?? "UNKNOWN"}</span></div>
        <div className="summary-card">
          <strong>Torque ON / トルクON</strong>
          <span>{torqueOnCount} / {arm.joints.length}</span>
        </div>
        <div className="summary-card">
          <strong>Leader / リーダー</strong>
          <span>{arm.teleoperation?.leader?.state ?? "UNKNOWN"}</span>
        </div>
        <div className="summary-card">
          <strong>Leader feedback / リーダー受信</strong>
          <span>{age(arm.teleoperation?.feedback_age_sec ?? null)}</span>
        </div>
        <div className="summary-card">
          <strong>Teleoperate / 遠隔操作</strong>
          <span>{arm.teleoperation?.active ? "ACTIVE" : arm.teleoperation?.aligning ? "ALIGNING" : arm.teleoperation?.ready ? "READY" : "BLOCKED"}</span>
        </div>
      </div>

      <div className="arm-hash">calibration sha256: {arm.calibration.sha256}</div>

      <div className="arm-joints">
        {arm.joints.map((joint) => {
          const value = targets[joint.name] ?? joint.target;
          const displayedValue = codeControlEnabled ? value : (joint.actual ?? joint.target);
          const motorName = joint.motor_name;
          const voltage = numeric(arm.motor_health?.voltage_deci_v?.[motorName]);
          const temperature = numeric(arm.motor_health?.temperature_c?.[motorName]);
          const current = numeric(arm.motor_health?.current_raw?.[motorName]);
          const torque = numeric(arm.motor_health?.torque_enable?.[motorName]);
          const voltageMin = numeric(arm.protection?.voltage_warning_min_deci_v);
          const voltageMax = numeric(arm.protection?.voltage_warning_max_deci_v);
          const temperatureWarn = numeric(arm.protection?.temperature_warning_c);
          const temperatureMax = numeric(arm.protection?.max_temperature_c);
          const currentWarn = numeric(arm.protection?.resistance_current_raw?.[motorName]);
          const currentMax = numeric(arm.protection?.hard_current_raw);
          const activeMotors = arm.motor_health?.active_motors ?? [];
          const torqueExpected = Boolean(arm.hold?.holding || activeMotors.includes(motorName));
          const voltageTone: MetricTone = voltage == null ? "neutral"
            : ((voltageMin != null && voltage < voltageMin) || (voltageMax != null && voltage > voltageMax)) ? "warn" : "ok";
          const temperatureTone: MetricTone = temperature == null ? "neutral"
            : temperatureMax != null && temperature > temperatureMax ? "bad"
            : temperatureWarn != null && temperature >= temperatureWarn ? "warn" : "ok";
          const currentMagnitude = current == null ? null : Math.abs(current);
          const currentTone: MetricTone = currentMagnitude == null ? "neutral"
            : currentMax != null && currentMagnitude >= currentMax ? "bad"
            : currentWarn != null && currentMagnitude >= currentWarn ? "warn" : "ok";
          const torqueTone: MetricTone = torque == null ? "neutral"
            : (torqueExpected ? torque !== 1 : torque === 1) ? "bad" : "ok";
          return (
            <label className="arm-joint" key={joint.name}>
              <div className="arm-joint-header">
                <strong>{joint.label}</strong>
                <span>actual / 実位置 {radians(joint.actual)}</span>
              </div>
              <input
                type="range"
                min={joint.lower}
                max={joint.upper}
                step={0.01}
                value={displayedValue}
                disabled={!codeControlEnabled}
                onChange={(event) => {
                  const next = Number(event.target.value);
                  setTargets((current) => ({ ...current, [joint.name]: next }));
                  setDirty(true);
                }}
              />
              <div className="arm-joint-footer">
                <span>{joint.lower.toFixed(2)}</span>
                <strong>candidate / 候補 {radians(displayedValue)}</strong>
                <span>{joint.upper.toFixed(2)}</span>
              </div>
              <div className="arm-motor-metrics">
                <MotorMetric
                  label="Voltage / 電圧"
                  value={voltage == null ? "—" : `${(voltage / 10).toFixed(1)} V`}
                  note={voltageMin == null || voltageMax == null ? undefined : `warn < ${(voltageMin / 10).toFixed(1)} / > ${(voltageMax / 10).toFixed(1)} V`}
                  tone={voltageTone}
                />
                <MotorMetric
                  label="Temperature / 温度"
                  value={temperature == null ? "—" : `${temperature.toFixed(0)} °C`}
                  note={temperatureMax == null ? undefined : `stop > ${temperatureMax.toFixed(0)} °C`}
                  tone={temperatureTone}
                />
                <MotorMetric
                  label="Current / 電流"
                  value={current == null ? "—" : `raw ${current.toFixed(0)}`}
                  note={currentWarn == null || currentMax == null ? "A conversion unverified" : `warn ${currentWarn.toFixed(0)} · stop ${currentMax.toFixed(0)}`}
                  tone={currentTone}
                />
                <MotorMetric
                  label="Torque_Enable"
                  value={torque == null ? "—" : torque === 1 ? "1 / ON" : "0 / OFF"}
                  note={torqueExpected ? "expected ON" : "expected OFF"}
                  tone={torqueTone}
                />
              </div>
            </label>
          );
        })}
      </div>

      <div className="row arm-action-row">
        <button onClick={preview} disabled={!targetComplete || !codeControlEnabled}>1. Preview / Validate / 検証</button>
        <button onClick={execute} disabled={!arm.execution.allowed || dirty || !candidateId}>
          {arm.mode === "MOCK" ? "2. Execute MOCK / 模擬実行" : "2. Execute REAL / 実機実行"}
        </button>
        <button className="danger" onClick={() => send({ command: "cancel" })}>Cancel / キャンセル</button>
        <button
          className="danger"
          disabled={!arm.hold?.holding}
          onClick={releaseHold}
        >
          Torque OFF / Release HOLD / トルクOFF
        </button>
        <button
          className="secondary"
          disabled={arm.hold?.state !== "FAULT"}
          onClick={() => send({ command: "reset_hold_fault" })}
        >
          Reset HOLD fault / 故障確認
        </button>
        <button className="danger" onClick={prepareSafeShutdown}>
          Prepare safe shutdown / 安全終了準備
        </button>
      </div>

      <div className={`arm-result ${arm.shutdown?.safe_to_close ? "ok" : ""}`}>
        <strong>Shutdown / 終了:</strong>{" "}
        {arm.shutdown?.message ?? "Waiting for torque evidence / トルク状態待機中"}
        {arm.shutdown && <span> · Torque ON: {arm.shutdown.torque_enabled_count}/6</span>}
      </div>

      <div className={`arm-result ${arm.execution.state === "BLOCKED" ? "bad" : ""}`}>
        <strong>{arm.execution.state}</strong>: {arm.execution.message}
        {dirty && <div>Slider changed; Preview is required before execution. / スライダー変更後は再度Previewが必要です。</div>}
        {arm.candidate.valid && (
          <div>
            joints: {(arm.candidate.changed_joints ?? []).join(", ") || "—"}; guarded minimum / 安全最短時間:
            {(arm.candidate.minimum_duration_sec ?? 0.5).toFixed(2)} s
          </div>
        )}
        {arm.candidate.errors.map((error) => <div key={error}>{error}</div>)}
      </div>

      <h2 className="arm-subtitle">Preset candidates / プリセット候補</h2>
      <div className="arm-button-grid">
        {arm.presets.map((preset) => {
          const enabled = preset.allowed_modes.includes(arm.mode);
          return (
            <button
              className="secondary"
              disabled={!enabled}
              key={preset.id}
              title={preset.description}
              onClick={() => send({ command: "select_preset", preset_id: preset.id })}
            >
              {preset.id}<small>{preset.review_state}</small>
            </button>
          );
        })}
      </div>

      <h2 className="arm-subtitle">Task triggers / タスクトリガー</h2>
      <div className="arm-button-grid">
        {arm.triggers.map((trigger) => (
          <button
            className={trigger.state === "ACTIVE" ? "" : "secondary"}
            key={trigger.id}
            title={trigger.reason}
            onClick={() => send({ command: "trigger", trigger_id: trigger.id })}
          >
            {trigger.label}<small>{trigger.state}</small>
          </button>
        ))}
      </div>
      {arm.touch_anything && (
        <section className={`arm-supplementary ${arm.touch_anything.active ? "ok" : ""}`}>
          <h2>Touch Anything / 任意対象にタッチ</h2>
          <div>
            <strong>{arm.touch_anything.state ?? "UNKNOWN"}</strong> — {arm.touch_anything.message}
          </div>
          <div>
            Base / 台車: {arm.touch_anything.base_lock ? "LOCKED" : "UNLOCKED"};
            execution / 実行: {arm.touch_anything.motion_execution ?? "UNKNOWN"};
            state age / 状態受信: {age(arm.touch_anything.state_age_sec ?? null)}.
          </div>
          <div>
            SAM: {arm.touch_anything.sam?.implementation ?? "UNKNOWN"} · {arm.touch_anything.sam?.device ?? "—"} · {arm.touch_anything.sam?.loaded ? "LOADED" : arm.touch_anything.sam?.available ? "ON DEMAND" : "NOT READY"}
            {arm.touch_anything.sam?.reason && <span> — {arm.touch_anything.sam.reason}</span>}
          </div>
          <div>
            Object selection and preview continue in the RViz Touch Anything panel.
            / 対象の点選択と3D確認はRVizのTouch Anythingパネルで行います。
          </div>
        </section>
      )}

      <section className={`arm-supplementary ${arm.hold?.state === "FAULT" ? "bad" : ""}`}>
        <h2>Supplementary status / 補足情報</h2>
        {arm.hold && (
          <div>
            <strong>POWERED HOLD: {arm.hold.state}</strong> — {arm.hold.reason}
            {arm.hold.holding && (
              <span> Six joints are holding. Support the arm before real2sim or Release HOLD. / 6関節を保持中です。real2simへの切替または保持解除の前にアームを支えてください。 Monitor / 監視頻度: {arm.hold.monitor_rate_hz ?? "—"} Hz.</span>
            )}
          </div>
        )}
        {torqueInvariantMissing.length > 0 && (
          <div>
            <strong>Powered-mode torque invariant / トルク条件:</strong> missing ON confirmation for / ON未確認: {torqueInvariantMissing.join(", ")}.
          </div>
        )}
        {arm.teleoperation && (
          <div>
            <strong>Leader teleoperation / リーダー遠隔操作:</strong>{" "}
            phase={arm.teleoperation.phase ?? "INACTIVE"}; {arm.teleoperation.reason}; mapping={arm.teleoperation.mapping ?? "—"};
            leader torque={arm.teleoperation.leader?.torque_free ? "OFF / FREE" : "NOT VERIFIED"};
            follower command rate={arm.teleoperation.command_rate_hz ?? "—"} Hz;
            initial alignment tolerance={arm.teleoperation.alignment_tolerance_ticks ?? "—"} tick.
            {arm.teleoperation.active_max_joint_velocity_rad_s && (
              <span> Active velocity limits / 現在の速度上限: {JSON.stringify(arm.teleoperation.active_max_joint_velocity_rad_s)} rad/s.</span>
            )}
            {Object.keys(arm.teleoperation.clamped_to_safe_limits ?? {}).length > 0 && (
              <span> Safe-limit clamp / 安全境界制限: {JSON.stringify(arm.teleoperation.clamped_to_safe_limits)}.</span>
            )}
          </div>
        )}
        {arm.recovery?.required && (
          <div>
            <strong>Position recovery / 位置復帰:</strong> bounded feedback is active; command limits were not expanded. / 制限付きフィードバックを継続中。指令範囲は拡張されていません。
          </div>
        )}
        {arm.protection?.armed && (
          <div>
            <strong>Current/resistance protection / 電流・抵抗保護:</strong> {arm.protection.monitor_rate_hz ?? "—"} Hz; hard-current raw {arm.protection.hard_current_raw ?? "—"}.
            <span> Temperature validation / 温度確認: {arm.protection.temperature_consecutive_samples ?? "—"} consecutive sync samples, then {arm.protection.temperature_confirmation_min_hot_reads ?? "—"}/{arm.protection.temperature_confirmation_reads ?? "—"} direct single-motor reads; rejected spikes / 除外スパイク: {arm.protection.temperature_rejected_spike_count ?? 0}.</span>
            {arm.protection.last_temperature_rejection && (
              <span> Last rejected spike / 最終除外: {arm.protection.last_temperature_rejection.motor ?? "—"} sync {arm.protection.last_temperature_rejection.sync_temperature_c ?? "—"} °C → direct {JSON.stringify(arm.protection.last_temperature_rejection.confirmation_reads_c ?? [])} °C ({arm.protection.last_temperature_rejection.context ?? "—"}).</span>
            )}
            {arm.protection.last_trip ? (
              <span> Last stop / 最終停止: {arm.protection.last_trip.reason ?? "UNKNOWN"} / {arm.protection.last_trip.tripped_motor ?? "—"}; {arm.protection.last_trip.boundary_update
                ? `soft limit / ソフト境界 ${arm.protection.last_trip.boundary_update.side} raw ${arm.protection.last_trip.boundary_update.raw}`
                : "no endpoint limit change / 端点境界の更新なし"}; trip temperature {JSON.stringify(arm.protection.last_trip.temperature_c ?? {})} °C; direct confirmation {JSON.stringify(arm.protection.last_trip.temperature_confirmation_c ?? [])} °C.</span>
            ) : <span> No protection stop recorded / 保護停止の記録なし.</span>}
            {arm.protection.last_motion && (
              <span> Last motion / 最終動作: {arm.protection.last_motion.outcome ?? "UNKNOWN"}; peak current raw {JSON.stringify(arm.protection.last_motion.peak_current_raw ?? {})}; final error tick {JSON.stringify(arm.protection.last_motion.final_error_ticks ?? {})}.</span>
            )}
          </div>
        )}
        <div>Calibration SHA-256: <code>{arm.calibration.sha256}</code></div>
      </section>
    </section>
  );
}

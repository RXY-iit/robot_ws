import { effectiveHealthSummaries } from "../lib/effectiveHealthSummary";
import type { RecoveryState } from "../types";
import { useEffect, useMemo, useState } from "react";
import type { ConnectionStatus } from "../lib/useDashboardSocket";
import { ReadinessPanel } from "./ReadinessPanel";
import { levelClass } from "../lib/missionLogic";
import type {
  HealthCapability,
  HealthDashboardDomain,
  HealthDashboardItem,
  HealthDashboardState,
} from "../types";

interface Props {
  recovery: RecoveryState | null;
  connectionStatus: ConnectionStatus;
  state: HealthDashboardState | null;
  receivedAt: number | null;
}

function numberText(value: number | null | undefined, digits = 2): string {
  return typeof value === "number" && Number.isFinite(value) ? value.toFixed(digits) : "—";
}

function valueText(value: unknown): string {
  if (value == null) return "No observed value";
  if (typeof value === "string") return value;
  try {
    return JSON.stringify(value, null, 2);
  } catch {
    return String(value);
  }
}

function statusBadge(status: string) {
  return <span className={`health-status ${levelClass(status)}`}>{status || "UNKNOWN"}</span>;
}

function CapabilityCard({ capability }: { capability: HealthCapability }) {
  return (
    <div className={`capability-card ${levelClass(capability.status)}`}>
      <div>
        <strong>{capability.title}</strong>
        {statusBadge(capability.status)}
      </div>
      <p>{capability.summary}</p>
      {capability.blocked_by && <small>reason: {capability.blocked_by}</small>}
    </div>
  );
}

function DriveElectricalIndicators({ value, status }: { value: unknown; status: string }) {
  const root = value && typeof value === "object" ? value as Record<string, unknown> : null;
  const samples = Array.isArray(root?.electrical_status) ? root.electrical_status : [];
  const positionLabel: Record<string, string> = {
    back: "Rear / 後輪",
    front_left: "Front left / 左前輪",
    front_right: "Front right / 右前輪",
  };
  const finite = (input: unknown): number | null => {
    const parsed = Number(input);
    return Number.isFinite(parsed) ? parsed : null;
  };
  const percent = (input: number | null, ratedMaximum: number) => (
    input == null ? 0 : Math.max(0, Math.min(100, Math.abs(input) / ratedMaximum * 100))
  );
  if (!samples.length) {
    return (
      <div className={`drive-electrical-panel ${levelClass(status)}`}>
        <div className="quick-metric-grid">
          {["Rear / 後輪", "Front left / 左前輪", "Front right / 右前輪"].map((position, index) => (
            <MetricBox key={position} label={`${position} · ID ${index + 1}`} value="NO SAMPLE / 未取得" note="Modbus electrical sample unavailable" />
          ))}
        </div>
        <small>Check the Modbus master, /drive/health_summary and sample age. A retained frame is never shown as a live value.</small>
      </div>
    );
  }
  return (
    <div className={`drive-electrical-panel ${levelClass(status)}`}>
      <div className="drive-electrical-grid">
        {samples.map((sample, index) => {
          const item = sample && typeof sample === "object" ? sample as Record<string, unknown> : {};
          const current = finite(item.main_power_supply_current_a);
          const mainVoltage = finite(item.main_power_supply_voltage_v);
          const inverterVoltage = finite(item.inverter_voltage_v);
          const position = String(item.wheel_position || `axis_${index + 1}`);
          return (
            <section className="drive-axis-indicator" key={String(item.slave_id ?? index)}>
              <div className="drive-axis-title">
                <strong>{positionLabel[position] || position}</strong>
                <span>ID {String(item.slave_id ?? "—")}</span>
              </div>
              <div className="drive-reading-grid">
                <div>
                  <span>Main supply / 主電源</span>
                  <strong>{mainVoltage == null ? "—" : `${mainVoltage.toFixed(1)} V`}</strong>
                  <i><b style={{ width: `${percent(mainVoltage, 48)}%` }} /></i>
                </div>
                <div>
                  <span>Inverter / インバータ</span>
                  <strong>{inverterVoltage == null ? "—" : `${inverterVoltage.toFixed(1)} V`}</strong>
                  <i><b style={{ width: `${percent(inverterVoltage, 48)}%` }} /></i>
                </div>
                <div>
                  <span>Main current / 主電流</span>
                  <strong>{current == null ? "—" : `${current.toFixed(3)} A`}</strong>
                  <i><b style={{ width: `${percent(current, 10.5)}%` }} /></i>
                </div>
              </div>
            </section>
          );
        })}
      </div>
      <small>Nameplate rating / 銘板定格: 24–48 VDC · 10.5 A. Bars use nameplate maxima only; they are not automatic health limits.</small>
    </div>
  );
}

function finiteNumber(value: unknown): number | null {
  if (value == null || value === "") return null;
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function objectValue(value: unknown): Record<string, any> | null {
  return value && typeof value === "object" ? value as Record<string, any> : null;
}

function MetricBox({ label, value, note }: { label: string; value: string; note?: string }) {
  return (
    <div className="quick-metric-box">
      <span>{label}</span>
      <strong>{value}</strong>
      {note && <small>{note}</small>}
    </div>
  );
}

function KeyEvidenceIndicators({ item }: { item: HealthDashboardItem }) {
  const root = objectValue(item.value);
  if (item.id === "camera_pan" || item.id === "camera_tilt") {
    const angle = finiteNumber(root?.angle_deg);
    const pan = item.id === "camera_pan";
    return (
      <div className="key-evidence-panel">
        <MetricBox label={pan ? "Pan feedback" : "Tilt feedback"} value={angle == null ? "—" : `${angle.toFixed(1)}°`} />
        <p>Serial feedback / シリアル実測値. Phase 2 initial target: {pan ? "267.0° (Joy step 2.0°)" : "102.0° (Joy step 5.0°)"}. The initial target is not a mechanical zero; verified physical angle limits are not available.</p>
      </div>
    );
  }
  if (item.id === "lift_position") {
    const motion = objectValue(root?.motion);
    const config = objectValue(root?.configuration);
    const serial = objectValue(root?.serial);
    const position = finiteNumber(motion?.estimated_position_mm);
    return (
      <div className="key-evidence-panel">
        <div className="quick-metric-grid five">
          <MetricBox label="Estimated position" value={position == null ? "—" : `${position.toFixed(1)} mm`} note={String(motion?.position_reference || "UNREFERENCED")} />
          <MetricBox label="Command" value={String(motion?.command || "—")} note={motion?.is_homing ? "homing" : "not homing"} />
          <MetricBox label="Configured range" value={`${numberText(config?.position_min_mm, 0)}–${numberText(config?.position_max_mm, 0)} mm`} />
          <MetricBox label="Home / speed" value={`${numberText(config?.home_position_mm, 0)} mm / ${numberText(config?.jog_speed_mm_s, 0)} mm/s`} />
          <MetricBox label="Serial" value={serial?.open ? "OPEN" : "CLOSED"} note={`${String(serial?.port || "—")} @ ${String(serial?.baud_rate || "—")}`} />
        </div>
        <p>Position is speed × time integration only / 位置は速度×時間の積算値です. UNREFERENCED = not calibrated; TIMED_HOME_ESTIMATE = timed homing estimate. soft limit={String(config?.soft_limits_enabled ?? "—")}, auto home={String(config?.auto_home_on_start ?? "—")}. Neither proves ABZO or limit-switch feedback.</p>
      </div>
    );
  }
  if (item.id === "delivery_robot_mode") {
    const mode = String(item.value || "UNKNOWN").toUpperCase();
    return (
      <div className="key-evidence-panel compact">
        <MetricBox label="Current authority" value={mode} />
        <p>MANUAL: Joy/operator authority / 手動操作. AUTO: Nav2/Agent authority / 自動操作. Other values have no defined motion authority.</p>
      </div>
    );
  }
  if (item.id === "delivery_safety") {
    const safety = String(item.value || "UNKNOWN").toUpperCase();
    return (
      <div className="key-evidence-panel compact">
        <MetricBox label="Software safety" value={safety} />
        <p>OK: the software gate reports no block. ESTOP: software output must be zero. This does not replace the H1 physical red-button DIN3 evidence.</p>
      </div>
    );
  }
  if (item.id === "gicp_score") {
    const score = finiteNumber(root?.inlier_rmse);
    const threshold = 0.25;
    return (
      <div className="key-evidence-panel compact">
        <MetricBox label="Inlier RMSE" value={score == null ? "—" : score.toFixed(4)} note="lower is better" />
        <MetricBox label="Configured acceptance" value={`≤ ${threshold.toFixed(2)}`} note={score == null ? "UNKNOWN" : score <= threshold ? "PASS" : "REJECT"} />
        <p>0.25 is the current gicp_localizer setting. PASS only means that the matcher accepted this attempt; it does not prove the absolute pose.</p>
      </div>
    );
  }
  if (item.id === "drive_alarm_status") {
    const samples = Array.isArray(root?.alarm_status) ? root?.alarm_status : [];
    if (!samples.length) return null;
    return (
      <div className="key-evidence-panel">
        <div className="quick-metric-grid">
          {samples.map((sample: any) => (
            <MetricBox
              key={String(sample.slave_id)}
              label={`ID ${String(sample.slave_id)} ${String(sample.wheel_position || "")}`}
              value={`current 0x${Number(sample.present_alarm || 0).toString(16).toUpperCase().padStart(2, "0")}`}
              note={`history 0x${Number(sample.alarm_history_1 || 0).toString(16).toUpperCase().padStart(2, "0")} · age ${numberText(sample.sample_age_sec)}s`}
            />
          ))}
        </div>
        <p>current != 0 means an active alarm. history changed means a new event occurred in this drive session. Codes remain raw and must be interpreted with the official table.</p>
      </div>
    );
  }
  if (item.id === "dynamixel_hardware_status") {
    const motors = Array.isArray(root?.motors) ? root.motors : [];
    return (
      <div className="key-evidence-panel">
        <div className="quick-metric-grid">
          {motors.map((motor: any) => {
            const current = finiteNumber(motor.present_current_a);
            const voltage = finiteNumber(motor.input_voltage_v);
            const temperature = finiteNumber(motor.temperature_c);
            const sampleAge = finiteNumber(motor.last_hardware_success_age_sec);
            return (
              <section className="dynamixel-indicator" key={String(motor.id)}>
                <div className="drive-axis-title">
                  <strong>ID {String(motor.id)} · {String(motor.model_name || "UNKNOWN")}</strong>
                  <span>{motor.hardware_observed ? "OBSERVED / 取得済み" : "NO SAMPLE / 未取得"}</span>
                </div>
                <div className="quick-metric-grid four">
                  <MetricBox label="Input voltage / 入力電圧" value={voltage == null ? "—" : `${voltage.toFixed(1)} V`} />
                  <MetricBox label="Present current / 現在電流" value={current == null ? "—" : `${current.toFixed(3)} A`} note={`raw ${String(motor.present_current_raw ?? "—")}`} />
                  <MetricBox label="Temperature / 温度" value={temperature == null ? "—" : `${temperature.toFixed(0)} °C`} />
                  <MetricBox label="HW error / HWエラー" value={`0x${Number(motor.hardware_error_status || 0).toString(16).toUpperCase().padStart(2, "0")}`} note={`age ${sampleAge == null ? "—" : sampleAge.toFixed(2)} s · samples ${String(motor.hardware_successes ?? "—")}`} />
                </div>
              </section>
            );
          })}
        </div>
        {!motors.length && <MetricBox label="DYNAMIXEL hardware registers" value="NO SAMPLE / 未取得" />}
        <p>Direct XH540-W270 registers / 直接レジスタ値: Current(126), Voltage(144), Temperature(146), Hardware Error Status(70). Error 0x00 means no hardware-error bit is set; no unverified current or temperature limit is invented here.</p>
      </div>
    );
  }
  if (item.id === "so101_arm_hardware_status") {
    const voltage = objectValue(root?.voltage_deci_v);
    const current = objectValue(root?.current_raw);
    const temperature = objectValue(root?.temperature_c);
    const torque = objectValue(root?.torque_enable);
    const hold = objectValue(root?.hold);
    const motorLabels: Record<string, string> = {
      shoulder_pan: "Shoulder pan / 肩旋回",
      shoulder_lift: "Shoulder lift / 肩上下",
      elbow_flex: "Elbow flex / 肘曲げ",
      wrist_flex: "Wrist flex / 手首曲げ",
      wrist_roll: "Wrist roll / 手首回転",
      gripper: "Gripper / グリッパー",
    };
    const motors = Object.keys(motorLabels);
    const torqueCount = motors.filter((name) => Number(torque?.[name]) === 1).length;
    return (
      <div className="key-evidence-panel">
        <div className="quick-metric-grid four">
          <MetricBox label="Control mode / 制御モード" value={String(root?.control_mode || "UNKNOWN")} />
          <MetricBox label="HOLD / 保持" value={String(hold?.state || "UNKNOWN")} note={String(hold?.reason || "—")} />
          <MetricBox label="Torque enabled / トルクON" value={`${torqueCount} / 6`} />
          <MetricBox label="Guarded server / 保護サーバー" value={root?.healthy === true ? "HEALTHY / 正常" : "UNHEALTHY / 異常"} />
        </div>
        <div className="quick-metric-grid">
          {motors.map((name) => {
            const voltageRaw = finiteNumber(voltage?.[name]);
            const currentRaw = finiteNumber(current?.[name]);
            const temperatureC = finiteNumber(temperature?.[name]);
            const torqueEnabled = Number(torque?.[name]) === 1;
            return (
              <section className="dynamixel-indicator" key={name}>
                <div className="drive-axis-title">
                  <strong>{motorLabels[name]}</strong>
                  <span>{torqueEnabled ? "TORQUE ON / ON" : "TORQUE OFF / OFF"}</span>
                </div>
                <div className="quick-metric-grid four">
                  <MetricBox label="Voltage / 電圧" value={voltageRaw == null ? "—" : `${(voltageRaw / 10).toFixed(1)} V`} note={voltageRaw == null ? undefined : `raw ${voltageRaw} · 0.1 V`} />
                  <MetricBox label="Current / 電流" value={currentRaw == null ? "—" : `raw ${currentRaw.toFixed(0)}`} note="No verified ampere conversion / A換算未検証" />
                  <MetricBox label="Temperature / 温度" value={temperatureC == null ? "—" : `${temperatureC.toFixed(0)} °C`} />
                  <MetricBox label="Torque_Enable" value={torqueEnabled ? "1 / ENABLED" : "0 / DISABLED"} />
                </div>
              </section>
            );
          })}
        </div>
        <p>Direct Feetech follower registers from `/arm/hardware_status` / guarded follower bus ownerから取得したFeetech直接レジスタ値です。Current is intentionally shown in raw units until a verified ampere conversion is configured / 電流は検証済みA換算が設定されるまでraw値で表示します。</p>
      </div>
    );
  }
  if (item.id === "camera_temperatures") {
    const values = objectValue(root?.values);
    if (!values) return null;
    return (
      <div className="key-evidence-panel compact">
        <MetricBox label="ASIC" value={`${String(values["Asic Temperature"] ?? "—")} °C`} />
        <MetricBox label="Projector" value={`${String(values["Projector Temperature"] ?? "—")} °C`} />
        <p>Values come from native RealSense diagnostics / RealSense標準診断値. The Dashboard does not invent temperature limits.</p>
      </div>
    );
  }
  if (item.id === "lidar_device_status") {
    const devices = Array.isArray(root?.devices) ? root.devices : [];
    const workStates: Record<number, string> = {
      1: "SAMPLING",
      2: "IDLE",
      4: "ERROR",
      5: "SELFCHECK",
      6: "MOTOR STARTUP",
      8: "UPGRADE",
      9: "READY",
    };
    const syncTypes: Record<number, string> = { 0: "NONE", 1: "PTP", 2: "GPS" };
    const statusBytes = (value: unknown): number[] => {
      if (typeof value === "string") {
        return value.trim().split(/\s+/).map((field) => Number.parseInt(field, 16)).filter(Number.isFinite);
      }
      return Array.isArray(value) ? value.map(Number).filter(Number.isFinite) : [];
    };
    return (
      <div className="key-evidence-panel">
        {devices.map((device: any, index: number) => {
          const state = objectValue(device?.state);
          const work = finiteNumber(state?.cur_work_state);
          const temperatureRaw = finiteNumber(state?.core_temp);
          const sync = finiteNumber(state?.time_sync_type);
          const diagnostic = finiteNumber(state?.lidar_diag_status);
          const levels = diagnostic == null ? [] : [0, 4, 8, 12].map((shift) => (diagnostic >> shift) & 0xF);
          const hmsCodes = Array.isArray(state?.hms_code)
            ? state.hms_code.map(Number).filter((code: number) => Number.isFinite(code) && code !== 0)
            : [];
          const nativeStatus = statusBytes(state?.status_code);
          const statusClear = nativeStatus.length > 0 && nativeStatus.every((field) => field === 0);
          return (
            <section className="dynamixel-indicator" key={String(device?.handle ?? index)}>
              <div className="drive-axis-title">
                <strong>MID-360 · {String(device?.ip || "UNKNOWN IP")}</strong>
                <span>SDK push age {numberText(finiteNumber(device?.push_age_sec))} s</span>
              </div>
              <div className="quick-metric-grid four">
                <MetricBox
                  label="Work state / 動作状態"
                  value={work == null ? "—" : `${workStates[work] || "UNKNOWN"} (${work})`}
                  note="1=SAMPLING · 9=READY · 4=ERROR"
                />
                <MetricBox
                  label="Core temperature / コア温度"
                  value={temperatureRaw == null ? "—" : `${(temperatureRaw / 100).toFixed(2)} °C`}
                  note={temperatureRaw == null ? undefined : `raw ${temperatureRaw} · unit 0.01 °C`}
                />
                <MetricBox
                  label="Time sync / 時刻同期"
                  value={sync == null ? "—" : `${syncTypes[sync] || "UNKNOWN"} (${sync})`}
                  note="NONE is expected without an external clock / 外部時刻源なしではNONE"
                />
                <MetricBox
                  label="Diagnostic modules / 診断モジュール"
                  value={diagnostic == null ? "—" : `0x${diagnostic.toString(16).toUpperCase().padStart(4, "0")}`}
                  note={levels.length ? `system/scan/ranging/comm = ${levels.join("/")}` : undefined}
                />
                <MetricBox
                  label="HMS codes / HMSコード"
                  value={hmsCodes.length ? hmsCodes.map((code: number) => `0x${code.toString(16).toUpperCase().padStart(8, "0")} (L${code & 0xFF})`).join(", ") : "CLEAR / なし"}
                  note={`L1=info · L2=warning · L3=error · L4=fatal · ${String(device?.callback_count ?? "—")} callbacks`}
                />
                <MetricBox
                  label="256-bit status / 256-bit状態"
                  value={!nativeStatus.length ? "NOT EXPOSED / 未提供" : statusClear ? "CLEAR / クリア" : "NONZERO / 非ゼロ"}
                  note={!nativeStatus.length ? "Optional in this MID-360 SDK state / このSDK状態では任意" : "Raw SDK field is retained below / SDK原値は下欄に保持"}
                />
              </div>
            </section>
          );
        })}
        {!devices.length && <MetricBox label="Livox SDK2 state" value="NO SAMPLE / 未取得" />}
        <p>Direct periodic MID-360 state from Livox SDK2 / Livox SDK2のMID-360周期状態. Temperature warnings and errors are judged from native diagnostic/HMS fields; the Dashboard does not invent a temperature threshold.</p>
      </div>
    );
  }
  return null;
}

function SystemResourceStrip({ domains }: { domains: HealthDashboardDomain[] }) {
  const host = domains.flatMap((domain) => domain.items).find((item) => item.id === "host_resources");
  const value = objectValue(host?.value);
  if (!value) return null;
  const cpu = finiteNumber(value.cpu_used_percent);
  const memory = finiteNumber(value.memory_used_percent);
  const thermal = finiteNumber(value.thermal_zone_max_c);
  const hwmon = finiteNumber(value.hwmon_max_c);
  const maxTemperature = [thermal, hwmon].filter((item): item is number => item != null).sort((a, b) => b - a)[0] ?? null;
  const load = objectValue(value.load_average);
  const processes = Array.isArray(value.top_processes) ? value.top_processes.slice(0, 6) : [];
  return (
    <section className="system-resource-strip">
      <div className="system-resource-head">
        <strong>Host load & temperature</strong>
        <small>Direct sources / 直接取得: /proc and thermal/hwmon; process CPU window: about {numberText(value.process_cpu_window_sec, 1)} s</small>
      </div>
      <div className="system-resource-summary">
        <MetricBox label="Total CPU" value={cpu == null ? "—" : `${cpu.toFixed(1)}%`} note="All logical CPU capacity / 全論理CPU容量" />
        <MetricBox label="Load 1m / cores" value={`${numberText(load?.["1m"])} / ${String(value.logical_cpu_count ?? "—")}`} />
        <MetricBox label="Memory used" value={memory == null ? "—" : `${memory.toFixed(1)}%`} />
        <MetricBox label="Max temperature" value={maxTemperature == null ? "—" : `${maxTemperature.toFixed(0)} °C`} note={host?.status || "UNKNOWN"} />
      </div>
      <div className="process-cpu-list">
        {processes.length ? processes.map((process: any) => (
          <div key={`${String(process.pid)}-${String(process.name)}`}>
            <span>{String(process.name)} <small>PID {String(process.pid)}</small></span>
            <strong>{numberText(process.one_core_percent, 1)}%</strong>
            <small>≈ {numberText(process.host_capacity_percent, 1)}% total capacity</small>
          </div>
        )) : <span className="small">Waiting for the first 5-second per-process CPU window…</span>}
      </div>
      <p>Bold process CPU uses the conventional one-core=100% scale. The total-capacity value can be compared with Total CPU. Temperature is judged only when hardware publishes max/critical limits.</p>
    </section>
  );
}

function SimulationStrip({ domains, stale }: { domains: HealthDashboardDomain[]; stale: boolean }) {
  const items = domains.flatMap((domain) => domain.items);
  const clock = items.find((item) => item.id === "sim_rtf");
  if (!clock) return null;
  const data = objectValue(clock.value) || {};
  const scan = objectValue(items.find((item) => item.id === "gicp_recovery_status")?.value) || {};
  const labels: Record<string, string> = { sim_lidar_pc2: "LiDAR", sim_lidar_imu: "IMU", sim_camera_color: "RGB", sim_camera_depth: "Depth", sim_arm_camera_color: "Arm RGB" };
  const metric = (value: unknown, digits = 2) => stale || finiteNumber(value) == null ? "--" : numberText(Number(value), digits);
  return (
    <section className="system-resource-strip">
      <div className="system-resource-head"><strong>Simulation timing</strong><small>{stale ? "STALE" : clock.summary}</small></div>
      <div className="system-resource-summary">
        <MetricBox label="RTF" value={metric(data.rtf, 3)} note={stale ? "STALE" : clock.status} />
        <MetricBox label="Clock age (wall s)" value={metric(data.clock_receive_age_sec)} />
        <MetricBox label="Clock resets" value={metric(data.reset_count, 0)} />
        <MetricBox label="Scan coverage" value={scan.coverage == null || stale ? "--" : `${numberText(Number(scan.coverage) * 100, 0)}%`} note={stale ? "STALE" : String(scan.failure_reason || scan.phase || "UNKNOWN")} />
      </div>
      <div className="process-cpu-list">
        {items.filter((item) => item.id.startsWith("timing:")).map((item) => {
          const stream = objectValue(item.value) || {};
          return <div key={item.id}><span title={item.id}>{labels[item.id.slice(7)] || item.id.slice(7)}</span><strong>{metric(stream.receive_rate_hz_5s)} wall Hz</strong><small>{metric(stream.receive_rate_hz_sim)} sim Hz</small></div>;
        })}
      </div>
    </section>
  );
}

function EvidenceItem({ item, highlighted }: { item: HealthDashboardItem; highlighted: boolean }) {
  const staticEvidence = item.evidence === "PHYSICAL_INSPECTION_RECORD" || item.evidence === "DOCUMENTED_CONFIGURATION";
  const noTelemetry = item.evidence === "NOT_INSTRUMENTED";
  const unavailableMetric = staticEvidence
    ? "N/A · static"
    : noTelemetry
      ? "N/A · no telemetry"
      : "—";
  const unavailableHeaderAge = item.receive_age_sec != null
    ? "N/A · no header"
    : unavailableMetric;
  return (
    <article className={`evidence-row ${highlighted ? "highlighted" : ""}`} id={`item-${item.id}`}>
      <div className="evidence-main">
        <div className="evidence-title">
          <strong>{item.label}</strong>
          {item.critical && <span className="critical-label">required</span>}
          {statusBadge(item.status)}
        </div>
        <p>{item.summary}</p>
        <div className="evidence-source">
          <span>Source</span>
          <code>{item.source || "No source"}</code>
          <span className="evidence-kind">{item.evidence || "UNKNOWN_EVIDENCE"}</span>
        </div>
      </div>
      <dl className="evidence-metrics">
        <div>
          <dt>Receive age</dt>
          <dd>{item.receive_age_sec == null ? unavailableMetric : `${numberText(item.receive_age_sec)} s`}</dd>
        </div>
        <div>
          <dt>Header age</dt>
          <dd>{item.source_age_sec == null ? unavailableHeaderAge : `${numberText(item.source_age_sec)} s`}</dd>
        </div>
        <div>
          <dt>Rate (5 s)</dt>
          <dd>{item.receive_rate_hz_5s == null ? unavailableMetric : `${numberText(item.receive_rate_hz_5s)} Hz`}</dd>
        </div>
        <div>
          <dt>Messages</dt>
          <dd>{item.message_count ?? unavailableMetric}</dd>
        </div>
      </dl>
      {item.id === "drive_bus_voltage" && (
        <DriveElectricalIndicators value={item.value} status={item.status} />
      )}
      <KeyEvidenceIndicators item={item} />
      <details className="evidence-details">
        <summary>Value and rule</summary>
        <div className="rule-text">
          <strong>Rule</strong>
          <span>{item.standard || "No automated rule"}</span>
        </div>
        <pre>{valueText(item.value)}</pre>
      </details>
    </article>
  );
}

function domainFromHash(domains: HealthDashboardDomain[]): string {
  const requested = window.location.hash.replace(/^#/, "").toUpperCase();
  return domains.some((domain) => domain.id === requested) ? requested : domains[0]?.id || "H1";
}

export function HealthDashboard({ connectionStatus, state, receivedAt, recovery }: Props) {
  const [now, setNow] = useState(Date.now());
  const transportAgeSec = receivedAt == null ? null : Math.max(0, (now - receivedAt) / 1000);
  const generatedAtMs = state?.generated_at ? Date.parse(state.generated_at) : Number.NaN;
  const generatedAgeSec = Number.isFinite(generatedAtMs) ? (now - generatedAtMs) / 1000 : null;
  const stale = connectionStatus !== "connected" || transportAgeSec == null || transportAgeSec > 4.5
    || generatedAgeSec == null || Math.abs(generatedAgeSec) > 4.5;
  const domains = useMemo(() => (state?.domains || []).map(domain => stale ? {
    ...domain, status: "STALE", items: domain.items.map(item => ({...item, status: "STALE",
      summary: `Last recorded (${item.status}): ${item.summary}`})),
  } : domain), [state, stale]);
  const [selectedId, setSelectedId] = useState(() => domainFromHash(domains));

  useEffect(() => {
    const timer = window.setInterval(() => setNow(Date.now()), 500);
    return () => window.clearInterval(timer);
  }, []);

  useEffect(() => {
    if (!domains.length) return;
    const chooseHash = () => setSelectedId(domainFromHash(domains));
    chooseHash();
    window.addEventListener("hashchange", chooseHash);
    return () => window.removeEventListener("hashchange", chooseHash);
  }, [domains]);

  const selected = useMemo(
    () => domains.find((domain) => domain.id === selectedId) || domains[0] || null,
    [domains, selectedId],
  );

  const selectDomain = (domainId: string) => {
    setSelectedId(domainId);
    window.history.replaceState(null, "", `#${domainId}`);
  };

  return (
    <div className="health-page">
      <header className="app-header">
        <div className="brand-block">
          <a href="/">Robot Harness</a>
          <span>/</span>
          <strong>Health Dashboard V1</strong>
        </div>
        <nav className="top-nav" aria-label="Dashboard navigation">
          <a href="/">Agent Dashboard</a>
          <a className="active" href="/health">Health Dashboard</a>
          <a href="/arm">SO101 Arm</a>
        </nav>
        <span className={`pill ${connectionStatus === "connected" ? "ok" : "bad"}`}>
          {connectionStatus}
        </span>
      </header>

      {stale && (
        <div className="data-stale-banner" role="alert">
          DATA STALE — the page has not received a fresh health snapshot. Do not trust retained values.
        </div>
      )}

      <div className="health-page-body">
        <section className="health-overview-section">
          <div className="health-title-row">
            <div>
              <p className="eyebrow">Read-only evidence</p>
              <h1>{domains.some((domain) => domain.items.some((item) => item.id === "sim_rtf")) ? "Simulation health" : "Robot health"}</h1>
              <p className="health-intro">
                UNKNOWN and NOT_INSTRUMENTED are intentionally not shown as healthy.
              </p>
            </div>
            <dl className="snapshot-meta">
              <div><dt>Sequence</dt><dd>{state?.sequence ?? "—"}</dd></div>
              <div><dt>Transport age</dt><dd>{transportAgeSec == null ? "—" : `${numberText(transportAgeSec)} s`}</dd></div>
              <div><dt>Generated age</dt><dd>{generatedAgeSec == null ? "—" : `${numberText(generatedAgeSec)} s`}</dd></div>
            </dl>
          </div>

          <div className="health-summary-grid wide">
            {effectiveHealthSummaries(state, recovery, Date.now()).map((summary) => (
              <button
                type="button"
                key={summary.id}
                className={`health-card health-summary-card ${levelClass(stale ? "STALE" : summary.status)}`}
                onClick={() => selectDomain(summary.reason_domain_id || "H1")}
              >
                <strong><span>{summary.title}</span><span>{stale ? "STALE" : summary.status}</span></strong>
                <span>{stale ? "STALE" : summary.status}</span>
                <small>{summary.summary}</small>
              </button>
            ))}
          </div>

          <div className="section-head health-section-head">
            <h2>Capability availability</h2>
            <span className="small">Conservative V1 dependency gates</span>
          </div>
          <ReadinessPanel state={state} receivedAt={receivedAt} connected={connectionStatus === "connected"} />
          <div className="capability-grid">
            {(state?.capabilities || []).map((capability) => (
              <CapabilityCard key={capability.id} capability={stale ? {...capability, status: "STALE",
                summary: `Last recorded (${capability.status}): ${capability.summary}`} : capability} />
            ))}
          </div>
          {!stale && <SystemResourceStrip domains={domains} />}
          <SimulationStrip domains={domains} stale={stale} />
        </section>

        <div className="health-workspace">
          <aside className="domain-nav">
            <h2>Seven diagnostic domains</h2>
            {!domains.length && <p className="empty-state">Waiting for health_monitor…</p>}
            {domains.map((domain) => (
              <button
                type="button"
                key={domain.id}
                className={domain.id === selected?.id ? "active" : ""}
                onClick={() => selectDomain(domain.id)}
              >
                <span>{domain.id}</span>
                <strong>{domain.title}</strong>
                {statusBadge(domain.status)}
              </button>
            ))}
            <div className="health-legend">
              <strong>Evidence policy</strong>
              <span>Green: observed rule passed</span>
              <span>Amber: degraded or estimated</span>
              <span>Red: blocked, error or stale</span>
              <span>Gray: unknown or not instrumented</span>
            </div>
          </aside>

          <section className="domain-detail">
            {selected ? (
              <>
                <div className="domain-detail-head">
                  <div>
                    <span>{selected.id}</span>
                    <h2>{selected.title}</h2>
                    <p>{selected.summary}</p>
                  </div>
                  {statusBadge(selected.status)}
                </div>
                <div className="evidence-list">
                  {selected.items.map((item) => (
                    <EvidenceItem
                      key={item.id}
                      item={item}
                      highlighted={item.id === selected.reason_item_id}
                    />
                  ))}
                </div>
              </>
            ) : (
              <div className="empty-state">No seven-domain state has been received.</div>
            )}
          </section>
        </div>
      </div>
    </div>
  );
}

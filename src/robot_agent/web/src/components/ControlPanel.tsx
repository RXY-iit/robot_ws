import { useEffect, useRef, type CSSProperties } from "react";
import { JOY_AXIS, JOY_BUTTON, useJoyController } from "../lib/useJoyController";

interface Props {
  robotMode: string;
  cameraPanAngle: number | null;
  cameraTiltAngle: number | null;
  liftPosition: number | null;
  liftState: string;
  joySourceStatus: string;
  sendJoyState: (axes: number[], buttons: number[]) => void;
  sendJoyControl: (action: "ACQUIRE_DASHBOARD" | "RELEASE_DASHBOARD") => void;
  onCancel: () => void;
}

// Keyboard bindings for the movement/camera axes and the L1/R1 hold buttons.
// Axis signs (which direction is "+1") are a best guess -- verify against the
// real robot and flip here if forward/strafe/pan/tilt come out reversed.
// STRAFE and YAW were confirmed reversed against the real robot (2026-07-21) --
// pos/neg swapped below to fix; FORWARD/CAMERA_PAN/CAMERA_TILT are unverified.
const AXIS_KEYS: Record<number, { pos: string; neg: string }> = {
  [JOY_AXIS.FORWARD]: { pos: "KeyW", neg: "KeyS" },
  [JOY_AXIS.STRAFE]: { pos: "KeyA", neg: "KeyD" },
  [JOY_AXIS.YAW]: { pos: "KeyQ", neg: "KeyE" },
  [JOY_AXIS.CAMERA_PAN]: { pos: "ArrowRight", neg: "ArrowLeft" },
  [JOY_AXIS.CAMERA_TILT]: { pos: "ArrowUp", neg: "ArrowDown" },
};
const PRESSABLE_KEYS = new Set(
  Object.values(AXIS_KEYS).flatMap(({ pos, neg }) => [pos, neg]).concat([
    "Space", "ShiftLeft", "ShiftRight", "PageUp", "PageDown",
  ]),
);

function isTypingTarget(el: EventTarget | null): boolean {
  if (!(el instanceof HTMLElement)) return false;
  const tag = el.tagName;
  return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT" || el.isContentEditable;
}

function useHoldHandlers(onDown: () => void, onUp: () => void) {
  return {
    onPointerDown: onDown,
    onPointerUp: onUp,
    onPointerLeave: onUp,
    onPointerCancel: onUp,
  };
}

const CAMERA_ENABLE_BUTTON = JOY_BUTTON.X;
const BASE_ENABLE_BUTTON = JOY_BUTTON.L1;
const BASE_AXIS_INDICES = [JOY_AXIS.FORWARD, JOY_AXIS.STRAFE, JOY_AXIS.YAW];

interface ColorScheme {
  border: string;
  bg: string;
  activeBg: string;
}
const COLOR_ENABLE: ColorScheme = { border: "#2e5fb0", bg: "#eaf1fb", activeBg: "#bcd4f7" };
const COLOR_TURBO: ColorScheme = { border: "#c9a227", bg: "#fdf6e3", activeBg: "#f5e2a0" };
const COLOR_Y: ColorScheme = { border: "#c9a227", bg: "#fdf6e3", activeBg: "#f5e2a0" };
const COLOR_X: ColorScheme = { border: "#2e5fb0", bg: "#eaf1fb", activeBg: "#bcd4f7" };
const COLOR_B: ColorScheme = { border: "#c0392b", bg: "#fdecea", activeBg: "#f5b7ae" };
const COLOR_A: ColorScheme = { border: "#2f9e44", bg: "#eafbf0", activeBg: "#a9e8c2" };

function GamepadButton({
  label,
  className,
  active,
  onDown,
  onUp,
  colors,
  disabled = false,
}: {
  label: string;
  className?: string;
  active: boolean;
  onDown: () => void;
  onUp: () => void;
  colors?: ColorScheme;
  disabled?: boolean;
}) {
  const handlers = useHoldHandlers(onDown, onUp);
  const style: CSSProperties = colors
    ? {
        borderColor: colors.border,
        background: active ? colors.activeBg : colors.bg,
      }
    : active
      ? { borderColor: "var(--accent)", background: "#eaf2ff" }
      : {};
  return (
    <button disabled={disabled} className={`secondary ${className || ""}`} style={style} {...handlers}>
      {label}
    </button>
  );
}

export function ControlPanel({
  robotMode,
  cameraPanAngle,
  cameraTiltAngle,
  liftPosition,
  liftState,
  joySourceStatus,
  sendJoyState,
  sendJoyControl,
  onCancel,
}: Props) {
  let source = "UNKNOWN";
  let sourceReason = "Waiting for Joy source arbiter / Joy入力仲裁待ち";
  let dashboardAcquired = false;
  try {
    const parsed = JSON.parse(joySourceStatus) as Record<string, unknown>;
    source = String(parsed.source || "UNKNOWN");
    sourceReason = String(parsed.reason || "");
    dashboardAcquired = source === "DASHBOARD" && parsed.dashboard_acquired === true;
  } catch {
    sourceReason = joySourceStatus;
  }
  const joy = useJoyController(sendJoyState, dashboardAcquired);
  const pressedKeysRef = useRef<Set<string>>(new Set());

  useEffect(() => {
    const recompute = () => {
      const pressed = pressedKeysRef.current;
      for (const [indexStr, keys] of Object.entries(AXIS_KEYS)) {
        const index = Number(indexStr);
        joy.setAxis(index, (pressed.has(keys.pos) ? 1 : 0) - (pressed.has(keys.neg) ? 1 : 0));
      }
      joy.setButton(
        BASE_ENABLE_BUTTON,
        pressed.has("Space") ||
          pressed.has("KeyW") || pressed.has("KeyS") ||
          pressed.has("KeyA") || pressed.has("KeyD") ||
          pressed.has("KeyQ") || pressed.has("KeyE"),
      );
      joy.setButton(JOY_BUTTON.R1, pressed.has("ShiftLeft") || pressed.has("ShiftRight"));
      joy.setButton(
        CAMERA_ENABLE_BUTTON,
        pressed.has("ArrowLeft") || pressed.has("ArrowRight") ||
          pressed.has("ArrowUp") || pressed.has("ArrowDown"),
      );
      joy.setAxis(JOY_AXIS.LIFT_UP_TRIGGER, pressed.has("PageUp") ? -1 : 0);
      joy.setAxis(JOY_AXIS.LIFT_DOWN_TRIGGER, pressed.has("PageDown") ? -1 : 0);
    };
    const onKeyDown = (event: KeyboardEvent) => {
      if (!dashboardAcquired) return;
      if (isTypingTarget(event.target) || !PRESSABLE_KEYS.has(event.code)) return;
      event.preventDefault();
      if (!pressedKeysRef.current.has(event.code)) {
        pressedKeysRef.current.add(event.code);
        recompute();
      }
    };
    const onKeyUp = (event: KeyboardEvent) => {
      if (!PRESSABLE_KEYS.has(event.code)) return;
      pressedKeysRef.current.delete(event.code);
      recompute();
    };
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [dashboardAcquired, joy.setAxis, joy.setButton]);

  const axisActive = (index: number, dir: 1 | -1) => Math.sign(joy.axes[index]) === dir;
  const buttonActive = (index: number) => joy.buttons[index] === 1;

  const holdAxis = (index: number, value: number) => ({
    onDown: () => joy.setAxis(index, value),
    onUp: () => {
      if (joy.axes[index] === value) joy.setAxis(index, 0);
    },
  });
  const holdBaseAxis = (index: number, value: number) => ({
    onDown: () => {
      joy.setButton(BASE_ENABLE_BUTTON, true);
      joy.setAxis(index, value);
    },
    onUp: () => {
      if (joy.axes[index] === value) joy.setAxis(index, 0);
      const baseStillActive = BASE_AXIS_INDICES.some((axisIndex) => joy.axes[axisIndex] !== 0);
      if (!baseStillActive) joy.setButton(BASE_ENABLE_BUTTON, false);
    },
  });
  const holdButton = (index: number) => ({
    onDown: () => joy.setButton(index, true),
    onUp: () => joy.setButton(index, false),
  });
  const holdCameraAxis = (index: number, value: number) => ({
    onDown: () => {
      joy.setButton(CAMERA_ENABLE_BUTTON, true);
      joy.setAxis(index, value);
    },
    onUp: () => {
      if (joy.axes[index] === value) joy.setAxis(index, 0);
      joy.setButton(CAMERA_ENABLE_BUTTON, false);
    },
  });

  return (
    <>
      <div className="section-head">
        <h2>手動操作 (仮想 Joy-Con)</h2>
        <span className="small">robot_mode: {robotMode}</span>
      </div>
      <div className="block-meta" style={{ marginBottom: 8 }}>
        Dashboard buttons auto-hold their safety enables: W/A/S/D/Q/E drive the base, D-Pad moves the camera, LT/RT moves the lift.
        物理Joyの場合だけ台車はL1、カメラはXを押してください。
        pan={cameraPanAngle != null ? cameraPanAngle.toFixed(1) : "?"}° tilt=
        {cameraTiltAngle != null ? cameraTiltAngle.toFixed(1) : "?"}° lift=
        {liftPosition != null ? liftPosition.toFixed(3) : "?"} ({liftState})
      </div>

      <div className="row" style={{ marginBottom: 10 }}>
        <strong>Control source / 操作入力: {source}</strong>
        <label className={`control-source-switch ${dashboardAcquired ? "active" : ""}`}>
          <input
            type="checkbox"
            role="switch"
            checked={dashboardAcquired}
            onChange={(event) => {
              if (event.target.checked) {
                sendJoyControl("ACQUIRE_DASHBOARD");
              } else {
                joy.resetAll();
                sendJoyControl("RELEASE_DASHBOARD");
              }
            }}
          />
          <span className="control-source-switch-track" aria-hidden="true"><span /></span>
          <span>
            {dashboardAcquired
              ? "Dashboard control ON / Dashboard操作 ON"
              : "Dashboard control OFF / Dashboard操作 OFF"}
          </span>
        </label>
      </div>
      <div className="block-meta" style={{ marginBottom: 8 }}>
        {sourceReason}. Non-neutral physical input immediately pre-empts Dashboard control / 物理入力を検出するとDashboard操作権を直ちに解除します。
      </div>

      <div className="gamepad">
        <GamepadButton
          className="gamepad-shoulder"
          label="LB / L1"
          active={buttonActive(JOY_BUTTON.L1)}
          colors={COLOR_ENABLE}
          disabled={!dashboardAcquired}
          {...holdButton(JOY_BUTTON.L1)}
        />
        <GamepadButton
          className="gamepad-shoulder right"
          label="RB / R1"
          active={buttonActive(JOY_BUTTON.R1)}
          colors={COLOR_TURBO}
          disabled={!dashboardAcquired}
          {...holdButton(JOY_BUTTON.R1)}
        />
        <GamepadButton
          className="gamepad-trigger"
          label="LT / 上昇"
          active={joy.axes[JOY_AXIS.LIFT_UP_TRIGGER] < -0.5}
          colors={COLOR_ENABLE}
          disabled={!dashboardAcquired}
          {...holdAxis(JOY_AXIS.LIFT_UP_TRIGGER, -1)}
        />
        <GamepadButton
          className="gamepad-trigger right"
          label="RT / 下降"
          active={joy.axes[JOY_AXIS.LIFT_DOWN_TRIGGER] < -0.5}
          colors={COLOR_TURBO}
          disabled={!dashboardAcquired}
          {...holdAxis(JOY_AXIS.LIFT_DOWN_TRIGGER, -1)}
        />

        <div className="gamepad-dpad">
          <GamepadButton disabled={!dashboardAcquired} className="round small up" label="▲" active={axisActive(JOY_AXIS.CAMERA_TILT, 1)} {...holdCameraAxis(JOY_AXIS.CAMERA_TILT, 1)} />
          <GamepadButton disabled={!dashboardAcquired} className="round small left" label="◀" active={axisActive(JOY_AXIS.CAMERA_PAN, -1)} {...holdCameraAxis(JOY_AXIS.CAMERA_PAN, -1)} />
          <GamepadButton disabled={!dashboardAcquired} className="round small right" label="▶" active={axisActive(JOY_AXIS.CAMERA_PAN, 1)} {...holdCameraAxis(JOY_AXIS.CAMERA_PAN, 1)} />
          <GamepadButton disabled={!dashboardAcquired} className="round small down" label="▼" active={axisActive(JOY_AXIS.CAMERA_TILT, -1)} {...holdCameraAxis(JOY_AXIS.CAMERA_TILT, -1)} />
        </div>

        <div className="gamepad-center">
          <span>D-Pad: camera</span>
          <span>WASD/QE: base</span>
        </div>

        <div className="gamepad-face">
          <GamepadButton disabled={!dashboardAcquired} className="round btn-y" label="Y" active={buttonActive(JOY_BUTTON.Y)} colors={COLOR_Y} {...holdButton(JOY_BUTTON.Y)} />
          <GamepadButton disabled={!dashboardAcquired} className="round btn-x" label="X" active={buttonActive(JOY_BUTTON.X)} colors={COLOR_X} {...holdButton(JOY_BUTTON.X)} />
          <GamepadButton disabled={!dashboardAcquired} className="round btn-b" label="B" active={buttonActive(JOY_BUTTON.B)} colors={COLOR_B} {...holdButton(JOY_BUTTON.B)} />
          <GamepadButton disabled={!dashboardAcquired} className="round btn-a" label="A" active={buttonActive(JOY_BUTTON.A)} colors={COLOR_A} {...holdButton(JOY_BUTTON.A)} />
        </div>

        <div className="gamepad-stick left">
          <GamepadButton disabled={!dashboardAcquired} className="round up" label="W" active={axisActive(JOY_AXIS.FORWARD, 1)} {...holdBaseAxis(JOY_AXIS.FORWARD, 1)} />
          <GamepadButton disabled={!dashboardAcquired} className="round left" label="A" active={axisActive(JOY_AXIS.STRAFE, 1)} {...holdBaseAxis(JOY_AXIS.STRAFE, 1)} />
          <span className="center-label">L</span>
          <GamepadButton disabled={!dashboardAcquired} className="round right" label="D" active={axisActive(JOY_AXIS.STRAFE, -1)} {...holdBaseAxis(JOY_AXIS.STRAFE, -1)} />
          <GamepadButton disabled={!dashboardAcquired} className="round down" label="S" active={axisActive(JOY_AXIS.FORWARD, -1)} {...holdBaseAxis(JOY_AXIS.FORWARD, -1)} />
        </div>

        <div className="gamepad-stick right">
          <span className="center-label" style={{ gridColumn: 2, gridRow: 1 }}>R</span>
          <GamepadButton disabled={!dashboardAcquired} className="round left" label="Q" active={axisActive(JOY_AXIS.YAW, 1)} {...holdBaseAxis(JOY_AXIS.YAW, 1)} />
          <GamepadButton disabled={!dashboardAcquired} className="round right" label="E" active={axisActive(JOY_AXIS.YAW, -1)} {...holdBaseAxis(JOY_AXIS.YAW, -1)} />
        </div>
      </div>

      <div className="block-meta" style={{ marginTop: 8 }}>
        L スティック: 台車移動 (WASD、自動L1) ・ R スティック: 回転 (Q/E、自動L1) ・ LT: リフト上昇 (Page Up) ・
        RT: リフト下降 (Page Down) ・ Y: safety ESTOP トグル ・ A: AUTO切替 ・
        B: MANUAL切替+ナビキャンセル ・ D-Pad: カメラ操作 (自動X)。リフト位置はフィードバックなしの推定値で、
        現在はソフトウェア可動範囲を強制しません。
      </div>

      <div className="row" style={{ marginTop: 12 }}>
        <button
          className="secondary"
          style={{ borderColor: "var(--bad)", color: "var(--bad)" }}
          onClick={() => {
            onCancel();
            joy.resetAll();
          }}
        >
          全停止 / キャンセル
        </button>
      </div>
    </>
  );
}

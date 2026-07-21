import { useEffect, useRef, type CSSProperties } from "react";
import { JOY_AXIS, JOY_BUTTON, useJoyController } from "../lib/useJoyController";

interface Props {
  robotMode: string;
  cameraPanAngle: number | null;
  cameraTiltAngle: number | null;
  liftPosition: number | null;
  liftState: string;
  sendJoyState: (axes: number[], buttons: number[]) => void;
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
  Object.values(AXIS_KEYS).flatMap(({ pos, neg }) => [pos, neg]).concat(["Space", "ShiftLeft", "ShiftRight"]),
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
}: {
  label: string;
  className?: string;
  active: boolean;
  onDown: () => void;
  onUp: () => void;
  colors?: ColorScheme;
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
    <button className={`secondary ${className || ""}`} style={style} {...handlers}>
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
  sendJoyState,
  onCancel,
}: Props) {
  const joy = useJoyController(sendJoyState);
  const pressedKeysRef = useRef<Set<string>>(new Set());

  useEffect(() => {
    const recompute = () => {
      const pressed = pressedKeysRef.current;
      for (const [indexStr, keys] of Object.entries(AXIS_KEYS)) {
        const index = Number(indexStr);
        joy.setAxis(index, (pressed.has(keys.pos) ? 1 : 0) - (pressed.has(keys.neg) ? 1 : 0));
      }
      joy.setButton(JOY_BUTTON.L1, pressed.has("Space"));
      joy.setButton(JOY_BUTTON.R1, pressed.has("ShiftLeft") || pressed.has("ShiftRight"));
    };
    const onKeyDown = (event: KeyboardEvent) => {
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
  }, [joy.setAxis, joy.setButton]);

  const axisActive = (index: number, dir: 1 | -1) => Math.sign(joy.axes[index]) === dir;
  const buttonActive = (index: number) => joy.buttons[index] === 1;

  const holdAxis = (index: number, value: number) => ({
    onDown: () => joy.setAxis(index, value),
    onUp: () => {
      if (joy.axes[index] === value) joy.setAxis(index, 0);
    },
  });
  const holdButton = (index: number) => ({
    onDown: () => joy.setButton(index, true),
    onUp: () => joy.setButton(index, false),
  });

  return (
    <>
      <div className="section-head">
        <h2>手動操作 (仮想 Joy-Con)</h2>
        <span className="small">robot_mode: {robotMode}</span>
      </div>
      <div className="block-meta" style={{ marginBottom: 8 }}>
        L1 (LB) を押しながらでないと底盘/カメラ/リフトは動きません（実機と同じ dead-man スイッチ）。
        pan={cameraPanAngle != null ? cameraPanAngle.toFixed(1) : "?"}° tilt=
        {cameraTiltAngle != null ? cameraTiltAngle.toFixed(1) : "?"}° lift=
        {liftPosition != null ? liftPosition.toFixed(3) : "?"} ({liftState})
      </div>

      <div className="gamepad">
        <GamepadButton
          className="gamepad-shoulder"
          label="LB / L1"
          active={buttonActive(JOY_BUTTON.L1)}
          colors={COLOR_ENABLE}
          {...holdButton(JOY_BUTTON.L1)}
        />
        <GamepadButton
          className="gamepad-shoulder right"
          label="RB / R1"
          active={buttonActive(JOY_BUTTON.R1)}
          colors={COLOR_TURBO}
          {...holdButton(JOY_BUTTON.R1)}
        />

        <div className="gamepad-dpad">
          <GamepadButton className="round small up" label="▲" active={axisActive(JOY_AXIS.CAMERA_TILT, 1)} {...holdAxis(JOY_AXIS.CAMERA_TILT, 1)} />
          <GamepadButton className="round small left" label="◀" active={axisActive(JOY_AXIS.CAMERA_PAN, -1)} {...holdAxis(JOY_AXIS.CAMERA_PAN, -1)} />
          <GamepadButton className="round small right" label="▶" active={axisActive(JOY_AXIS.CAMERA_PAN, 1)} {...holdAxis(JOY_AXIS.CAMERA_PAN, 1)} />
          <GamepadButton className="round small down" label="▼" active={axisActive(JOY_AXIS.CAMERA_TILT, -1)} {...holdAxis(JOY_AXIS.CAMERA_TILT, -1)} />
        </div>

        <div className="gamepad-center">
          <span>D-Pad: camera</span>
          <span>M / Back / Start (未使用)</span>
        </div>

        <div className="gamepad-face">
          <GamepadButton className="round btn-y" label="Y" active={buttonActive(JOY_BUTTON.Y)} colors={COLOR_Y} {...holdButton(JOY_BUTTON.Y)} />
          <GamepadButton className="round btn-x" label="X" active={buttonActive(JOY_BUTTON.X)} colors={COLOR_X} {...holdButton(JOY_BUTTON.X)} />
          <GamepadButton className="round btn-b" label="B" active={buttonActive(JOY_BUTTON.B)} colors={COLOR_B} {...holdButton(JOY_BUTTON.B)} />
          <GamepadButton className="round btn-a" label="A" active={buttonActive(JOY_BUTTON.A)} colors={COLOR_A} {...holdButton(JOY_BUTTON.A)} />
        </div>

        <div className="gamepad-stick left">
          <GamepadButton className="round up" label="W" active={axisActive(JOY_AXIS.FORWARD, 1)} {...holdAxis(JOY_AXIS.FORWARD, 1)} />
          <GamepadButton className="round left" label="A" active={axisActive(JOY_AXIS.STRAFE, 1)} {...holdAxis(JOY_AXIS.STRAFE, 1)} />
          <span className="center-label">L</span>
          <GamepadButton className="round right" label="D" active={axisActive(JOY_AXIS.STRAFE, -1)} {...holdAxis(JOY_AXIS.STRAFE, -1)} />
          <GamepadButton className="round down" label="S" active={axisActive(JOY_AXIS.FORWARD, -1)} {...holdAxis(JOY_AXIS.FORWARD, -1)} />
        </div>

        <div className="gamepad-stick right">
          <span className="center-label" style={{ gridColumn: 2, gridRow: 1 }}>R</span>
          <GamepadButton className="round left" label="Q" active={axisActive(JOY_AXIS.YAW, 1)} {...holdAxis(JOY_AXIS.YAW, 1)} />
          <GamepadButton className="round right" label="E" active={axisActive(JOY_AXIS.YAW, -1)} {...holdAxis(JOY_AXIS.YAW, -1)} />
        </div>
      </div>

      <div className="block-meta" style={{ marginTop: 8 }}>
        L スティック: 底盘移動 (WASD) ・ R スティック: 回転 (Q/E) ・ Y: L1+Y でリフト上昇
        (⚠ cmd_vel_safety_node の ESTOP トグルも兼ねる、実機と同じ) ・ A: AUTO切替 / L1+A でリフト下降 ・
        B: MANUAL切替+ナビキャンセル ・ X: 未使用
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

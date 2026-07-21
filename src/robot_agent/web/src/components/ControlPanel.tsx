import { useCallback, useEffect, useRef } from "react";

type Target = "chassis" | "camera" | "lift";

interface Props {
  robotMode: string;
  cameraPanAngle: number | null;
  cameraTiltAngle: number | null;
  liftPosition: number | null;
  liftState: string;
  onManualControl: (target: Target, value: Record<string, unknown>) => void;
  onManualControlStop: (target: Target) => void;
  onCancel: () => void;
}

const HOLD_INTERVAL_MS = 100;

function useHoldButton(onTick: () => void, onStop: () => void) {
  const intervalRef = useRef<number | null>(null);

  const start = useCallback(() => {
    if (intervalRef.current != null) return;
    onTick();
    intervalRef.current = window.setInterval(onTick, HOLD_INTERVAL_MS);
  }, [onTick]);

  const stop = useCallback(() => {
    if (intervalRef.current != null) {
      window.clearInterval(intervalRef.current);
      intervalRef.current = null;
      onStop();
    }
  }, [onStop]);

  useEffect(() => {
    window.addEventListener("blur", stop);
    document.addEventListener("visibilitychange", stop);
    return () => {
      window.removeEventListener("blur", stop);
      document.removeEventListener("visibilitychange", stop);
      stop();
    };
  }, [stop]);

  return {
    onPointerDown: start,
    onPointerUp: stop,
    onPointerLeave: stop,
    onPointerCancel: stop,
  };
}

function HoldButton({
  label,
  onTick,
  onStop,
}: {
  label: string;
  onTick: () => void;
  onStop: () => void;
}) {
  const handlers = useHoldButton(onTick, onStop);
  return (
    <button className="secondary" {...handlers}>
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
  onManualControl,
  onManualControlStop,
  onCancel,
}: Props) {
  const chassis = (vx: number, vy: number, yaw: number) => () => onManualControl("chassis", { vx, vy, yaw });
  const chassisStop = () => onManualControlStop("chassis");

  const camera = (pan: number, tilt: number) => () => onManualControl("camera", { pan, tilt });
  const cameraStop = () => onManualControlStop("camera");

  const lift = (direction: "UP" | "DOWN") => () => onManualControl("lift", { direction });
  const liftStop = () => onManualControlStop("lift");

  return (
    <>
      <div className="section-head">
        <h2>手動操作</h2>
        <span className="small">robot_mode: {robotMode}</span>
      </div>

      <div className="block-meta">底盘 (hold)</div>
      <div className="quick-grid">
        <HoldButton label="前進" onTick={chassis(0.3, 0, 0)} onStop={chassisStop} />
        <HoldButton label="後退" onTick={chassis(-0.3, 0, 0)} onStop={chassisStop} />
        <HoldButton label="左移動" onTick={chassis(0, 0.3, 0)} onStop={chassisStop} />
        <HoldButton label="右移動" onTick={chassis(0, -0.3, 0)} onStop={chassisStop} />
        <HoldButton label="左回転" onTick={chassis(0, 0, 0.5)} onStop={chassisStop} />
        <HoldButton label="右回転" onTick={chassis(0, 0, -0.5)} onStop={chassisStop} />
      </div>

      <div className="block-meta" style={{ marginTop: 10 }}>
        カメラ pan={cameraPanAngle != null ? cameraPanAngle.toFixed(1) : "?"}° tilt=
        {cameraTiltAngle != null ? cameraTiltAngle.toFixed(1) : "?"}°
      </div>
      <div className="quick-grid">
        <HoldButton label="Pan 左" onTick={camera(-1, 0)} onStop={cameraStop} />
        <HoldButton label="Pan 右" onTick={camera(1, 0)} onStop={cameraStop} />
        <HoldButton label="Tilt 上" onTick={camera(0, -1)} onStop={cameraStop} />
        <HoldButton label="Tilt 下" onTick={camera(0, 1)} onStop={cameraStop} />
      </div>

      <div className="block-meta" style={{ marginTop: 10 }}>
        リフト position={liftPosition != null ? liftPosition.toFixed(3) : "?"} state={liftState}
      </div>
      <div className="quick-grid">
        <HoldButton label="上昇" onTick={lift("UP")} onStop={liftStop} />
        <HoldButton label="下降" onTick={lift("DOWN")} onStop={liftStop} />
      </div>

      <div className="row" style={{ marginTop: 12 }}>
        <button
          className="secondary"
          style={{ borderColor: "var(--bad)", color: "var(--bad)" }}
          onClick={() => {
            onCancel();
            onManualControlStop("chassis");
            onManualControlStop("camera");
            onManualControlStop("lift");
          }}
        >
          停止 / キャンセル
        </button>
      </div>
    </>
  );
}

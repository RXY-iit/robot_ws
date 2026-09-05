import { useCallback, useEffect, useRef, useState } from "react";

// Mirrors the real Joy-Con / gamepad indices the existing joystick-consumer
// nodes already use -- see the comment block above JoyManager in
// dashboard_node.py for the exact source files. Keep these in sync with that
// file; they are not independently configurable from the browser.
export const JOY_AXIS = {
  STRAFE: 0,
  FORWARD: 1,
  LIFT_UP_TRIGGER: 2,
  YAW: 3,
  LIFT_DOWN_TRIGGER: 5,
  CAMERA_PAN: 6,
  CAMERA_TILT: 7,
} as const;

export const JOY_BUTTON = {
  A: 0,
  B: 1,
  X: 2,
  Y: 3,
  L1: 4,
  R1: 5,
} as const;

const AXIS_COUNT = 8;
const BUTTON_COUNT = 8;
const HEARTBEAT_MS = 100;

export function useJoyController(
  sendJoyState: (axes: number[], buttons: number[]) => void,
  enabled = true,
) {
  const axesRef = useRef<number[]>(new Array(AXIS_COUNT).fill(0));
  const buttonsRef = useRef<number[]>(new Array(BUTTON_COUNT).fill(0));
  const heartbeatRef = useRef<number | null>(null);
  const [, bump] = useState(0);

  const isNeutral = () => axesRef.current.every((v) => v === 0) && buttonsRef.current.every((v) => v === 0);

  const publish = useCallback(() => {
    sendJoyState([...axesRef.current], [...buttonsRef.current]);
  }, [sendJoyState]);

  const ensureHeartbeat = useCallback(() => {
    if (heartbeatRef.current != null) return;
    heartbeatRef.current = window.setInterval(() => {
      if (!enabled || isNeutral()) {
        if (heartbeatRef.current != null) {
          window.clearInterval(heartbeatRef.current);
          heartbeatRef.current = null;
        }
        return;
      }
      publish();
    }, HEARTBEAT_MS);
  }, [enabled, publish]);

  const setAxis = useCallback(
    (index: number, value: number) => {
      if (axesRef.current[index] === value) return;
      if (!enabled) return;
      axesRef.current[index] = value;
      publish();
      if (value !== 0) ensureHeartbeat();
      bump((n) => n + 1);
    },
    [enabled, publish, ensureHeartbeat],
  );

  const setButton = useCallback(
    (index: number, pressed: boolean) => {
      const value = pressed ? 1 : 0;
      if (buttonsRef.current[index] === value) return;
      if (!enabled) return;
      buttonsRef.current[index] = value;
      publish();
      if (value !== 0) ensureHeartbeat();
      bump((n) => n + 1);
    },
    [enabled, publish, ensureHeartbeat],
  );

  const resetAll = useCallback(() => {
    axesRef.current = new Array(AXIS_COUNT).fill(0);
    buttonsRef.current = new Array(BUTTON_COUNT).fill(0);
    publish();
    bump((n) => n + 1);
  }, [publish]);

  useEffect(() => {
    window.addEventListener("blur", resetAll);
    document.addEventListener("visibilitychange", resetAll);
    return () => {
      window.removeEventListener("blur", resetAll);
      document.removeEventListener("visibilitychange", resetAll);
      if (heartbeatRef.current != null) window.clearInterval(heartbeatRef.current);
      resetAll();
    };
  }, [resetAll]);

  useEffect(() => {
    if (!enabled) resetAll();
  }, [enabled, resetAll]);

  return {
    axes: axesRef.current,
    buttons: buttonsRef.current,
    setAxis,
    setButton,
    resetAll,
  };
}

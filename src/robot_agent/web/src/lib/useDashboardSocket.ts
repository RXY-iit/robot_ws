import { useCallback, useEffect, useRef, useState } from "react";
import type {
  ArmDashboardState,
  HealthState,
  HealthDashboardState,
  MissionEvent,
  MissionState,
  NavigationProgress,
  NavigationRouteState,
  RealRobotState,
} from "../types";

export type ConnectionStatus = "connecting" | "connected" | "reconnecting";

export interface DashboardData {
  connectionStatus: ConnectionStatus;
  missionState: MissionState | null;
  healthState: HealthState | null;
  healthDashboardState: HealthDashboardState | null;
  healthDashboardReceivedAt: number | null;
  // Real hardware only; stays null in sim (real_robot_monitor never publishes there).
  realRobotState: RealRobotState | null;
  navigationProgress: NavigationProgress | null;
  navigationRouteState: NavigationRouteState | null;
  missionEvents: MissionEvent[];
  eventsTimeline: MissionEvent[];
  robotMode: string;
  safetyStatus: string;
  agentStatus: string;
  humanPrompt: string;
  localizationWarning: string;
  stationaryDriftM: string;
  receivedPending: boolean;
  pendingCommandText: string;
  cameraPanAngle: number | null;
  cameraTiltAngle: number | null;
  liftPosition: number | null;
  liftState: string;
  joySourceStatus: string;
  armState: ArmDashboardState | null;
  sendCommand: (text: string) => void;
  sendCancel: () => void;
  sendRealRobotCommand: (text: string) => void;
  sendJoyState: (axes: number[], buttons: number[]) => void;
  sendJoyControl: (action: "ACQUIRE_DASHBOARD" | "RELEASE_DASHBOARD") => void;
  sendArmCommand: (command: Record<string, unknown>) => void;
  sendJson: (message: Record<string, unknown>) => void;
}

const MISSION_EVENTS_CAP = 120;
const TIMELINE_CAP = 80;

function driftText(value: unknown): string {
  const n = Number(value);
  return Number.isFinite(n) ? n.toFixed(3) : "UNKNOWN";
}

export function useDashboardSocket(): DashboardData {
  const [connectionStatus, setConnectionStatus] = useState<ConnectionStatus>("connecting");
  const [missionState, setMissionState] = useState<MissionState | null>(null);
  const [healthState, setHealthState] = useState<HealthState | null>(null);
  const [healthDashboardState, setHealthDashboardState] = useState<HealthDashboardState | null>(null);
  const [healthDashboardReceivedAt, setHealthDashboardReceivedAt] = useState<number | null>(null);
  const [realRobotState, setRealRobotState] = useState<RealRobotState | null>(null);
  const [navigationProgress, setNavigationProgress] = useState<NavigationProgress | null>(null);
  const [navigationRouteState, setNavigationRouteState] = useState<NavigationRouteState | null>(null);
  const [missionEvents, setMissionEvents] = useState<MissionEvent[]>([]);
  const [eventsTimeline, setEventsTimeline] = useState<MissionEvent[]>([]);
  const [robotMode, setRobotMode] = useState("UNKNOWN");
  const [safetyStatus, setSafetyStatus] = useState("UNKNOWN");
  const [agentStatus, setAgentStatus] = useState("UNKNOWN");
  const [humanPrompt, setHumanPrompt] = useState("なし");
  const [localizationWarning, setLocalizationWarning] = useState("UNKNOWN");
  const [stationaryDriftM, setStationaryDriftM] = useState("UNKNOWN");
  const [receivedPending, setReceivedPending] = useState(false);
  const [pendingCommandText, setPendingCommandText] = useState("");
  const [cameraPanAngle, setCameraPanAngle] = useState<number | null>(null);
  const [cameraTiltAngle, setCameraTiltAngle] = useState<number | null>(null);
  const [liftPosition, setLiftPosition] = useState<number | null>(null);
  const [liftState, setLiftState] = useState("UNKNOWN");
  const [joySourceStatus, setJoySourceStatus] = useState(
    JSON.stringify({ source: "UNKNOWN", dashboard_acquired: false, reason: "waiting for arbiter" }),
  );
  const [armState, setArmState] = useState<ArmDashboardState | null>(null);

  const wsRef = useRef<WebSocket | null>(null);
  const missionIdRef = useRef<string | null>(null);

  const sendJson = useCallback((message: Record<string, unknown>) => {
    const ws = wsRef.current;
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(message));
    }
  }, []);

  const sendCommand = useCallback((text: string) => sendJson({ type: "command", text }), [sendJson]);
  const sendCancel = useCallback(() => sendJson({ type: "cancel" }), [sendJson]);
  const sendRealRobotCommand = useCallback(
    (text: string) => sendJson({ type: "real_robot_command", text }),
    [sendJson],
  );
  const sendJoyState = useCallback(
    (axes: number[], buttons: number[]) => sendJson({ type: "joy_state", axes, buttons }),
    [sendJson],
  );
  const sendJoyControl = useCallback(
    (action: "ACQUIRE_DASHBOARD" | "RELEASE_DASHBOARD") => sendJson({ type: "joy_control", action }),
    [sendJson],
  );
  const sendArmCommand = useCallback(
    (command: Record<string, unknown>) => sendJson({ type: "arm_command", command }),
    [sendJson],
  );

  useEffect(() => {
    let cancelled = false;

    function connect() {
      if (cancelled) return;
      const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
      const ws = new WebSocket(`${protocol}//${window.location.host}/ws`);
      wsRef.current = ws;

      ws.onopen = () => setConnectionStatus("connected");
      ws.onerror = () => setConnectionStatus("reconnecting");
      ws.onclose = () => {
        setConnectionStatus("reconnecting");
        if (!cancelled) setTimeout(connect, 1000);
      };
      ws.onmessage = (event) => {
        let data: { topic: string; data: string };
        try {
          data = JSON.parse(event.data);
        } catch {
          return;
        }

        if (data.topic === "/joy/source_status") {
          setJoySourceStatus(data.data);
        } else if (data.topic === "/arm/dashboard_state") {
          try {
            setArmState(JSON.parse(data.data) as ArmDashboardState);
          } catch {
            setArmState(null);
          }
        } else if (data.topic === "/agent/mission_state") {
          const state: MissionState = JSON.parse(data.data);
          if (state.mission_id && state.mission_id !== missionIdRef.current) {
            missionIdRef.current = state.mission_id ?? null;
            setMissionEvents([]);
            setNavigationProgress(null);
            setNavigationRouteState(null);
          }
          setReceivedPending(false);
          setMissionState(state);
        }

        if (data.topic === "/agent/health_state") {
          const health: HealthState = JSON.parse(data.data);
          setHealthState(health);
          const localization = health.components?.localization;
          if (localization) {
            setLocalizationWarning(localization.summary || "UNKNOWN");
            setStationaryDriftM(driftText(localization.details?.stationary_drift_m));
          }
        }

        if (data.topic === "/agent/real_robot_state") {
          setRealRobotState(JSON.parse(data.data));
        }

        if (data.topic === "/agent/health_dashboard_state") {
          setHealthDashboardState(JSON.parse(data.data));
          setHealthDashboardReceivedAt(Date.now());
        }

        if (data.topic === "/agent/navigation_progress") {
          setNavigationProgress(JSON.parse(data.data));
        }

        if (data.topic === "/agent/navigation_route_state") {
          setNavigationRouteState(JSON.parse(data.data));
        }

        if (data.topic === "/agent/mission_events") {
          const evt: MissionEvent = JSON.parse(data.data);
          if (evt.type === "mission_received" && evt.mission_id !== missionIdRef.current) {
            missionIdRef.current = evt.mission_id ?? null;
            setMissionState(null);
            setMissionEvents([]);
            setNavigationProgress(null);
            setNavigationRouteState(null);
            setReceivedPending(true);
            setPendingCommandText(String(evt.payload?.text || "なし"));
          }
          if (evt.mission_id && evt.mission_id === missionIdRef.current) {
            setMissionEvents((prev) => {
              const next = [...prev, evt];
              return next.length > MISSION_EVENTS_CAP ? next.slice(next.length - MISSION_EVENTS_CAP) : next;
            });
          }
          if (evt.type === "agent_heartbeat") {
            if (evt.payload?.robot_mode) setRobotMode(String(evt.payload.robot_mode));
            if (evt.payload?.safety_status) setSafetyStatus(String(evt.payload.safety_status));
            if (evt.payload?.localization_warning) setLocalizationWarning(String(evt.payload.localization_warning));
            if (Number.isFinite(evt.payload?.stationary_drift_m)) {
              setStationaryDriftM(driftText(evt.payload?.stationary_drift_m));
            }
          }
          setEventsTimeline((prev) => {
            const next = [evt, ...prev];
            return next.length > TIMELINE_CAP ? next.slice(0, TIMELINE_CAP) : next;
          });
        }

        if (data.topic === "/agent/status") setAgentStatus(data.data);
        if (data.topic === "/agent/human_prompt") setHumanPrompt(data.data);
        if (data.topic === "/robot_mode") setRobotMode(data.data);
        if (data.topic === "/safety_status") setSafetyStatus(data.data);
        if (data.topic === "/lift/state") setLiftState(data.data);
        if (data.topic === "/chokudomotor/angle") setCameraPanAngle(Number(data.data));
        if (data.topic === "/cameraswingmotor/angle") setCameraTiltAngle(Number(data.data));
        if (data.topic === "/lift/position") setLiftPosition(Number(data.data));
      };
    }

    connect();
    return () => {
      cancelled = true;
      wsRef.current?.close();
    };
  }, []);

  return {
    connectionStatus,
    missionState,
    healthState,
    healthDashboardState,
    healthDashboardReceivedAt,
    realRobotState,
    navigationProgress,
    navigationRouteState,
    missionEvents,
    eventsTimeline,
    robotMode,
    safetyStatus,
    agentStatus,
    humanPrompt,
    localizationWarning,
    stationaryDriftM,
    receivedPending,
    pendingCommandText,
    cameraPanAngle,
    cameraTiltAngle,
    liftPosition,
    liftState,
    joySourceStatus,
    armState,
    sendCommand,
    sendCancel,
    sendRealRobotCommand,
    sendJoyState,
    sendJoyControl,
    sendArmCommand,
    sendJson,
  };
}

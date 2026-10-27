import { useEffect, useState } from "react";
import { useDashboardSocket } from "./lib/useDashboardSocket";
import { HealthPanel } from "./components/HealthPanel";
import { RealRobotPanel } from "./components/RealRobotPanel";
import { RobotStatusPanel } from "./components/RobotStatusPanel";
import { HumanChoices } from "./components/HumanChoices";
import { CommandBox } from "./components/CommandBox";
import { MissionPanel } from "./components/MissionPanel";
import { EventTimeline } from "./components/EventTimeline";
import { ControlPanel } from "./components/ControlPanel";
import { FoxglovePanel } from "./components/FoxglovePanel";
import { HealthDashboard } from "./components/HealthDashboard";
import { ArmPanel } from "./components/ArmPanel";
import { AgentModePanel } from "./components/AgentModePanel";
import { RecoveryPanel } from "./components/RecoveryPanel";

interface QuickCommand {
  label: string;
  command: string;
}

interface DashboardConfig {
  quick_commands: [string, string][];
  mission_test_commands?: [string, string][];
  default_command: string;
  foxglove_bridge_port: number;
  foxglove_ui_port: number;
  foxglove_ui_enabled: boolean;
}

export function App() {
  const data = useDashboardSocket();
  const [commandText, setCommandText] = useState("");
  const [quickCommands, setQuickCommands] = useState<QuickCommand[]>([]);
  const [testCommands, setTestCommands] = useState<QuickCommand[]>([]);
  const [foxgloveConfig, setFoxgloveConfig] = useState<DashboardConfig | null>(null);

  useEffect(() => {
    fetch("/api/config")
      .then((response) => response.json())
      .then((config: DashboardConfig) => {
        setQuickCommands(config.quick_commands.map(([label, command]) => ({ label, command })));
        setTestCommands((config.mission_test_commands ?? []).map(([label, command]) => ({ label, command })));
        setCommandText(config.default_command);
        setFoxgloveConfig(config);
      })
      .catch(() => {});
  }, []);

  const sendCurrentCommand = () => data.sendCommand(commandText);
  const sendQuickCommand = (command: string) => {
    setCommandText(command);
    data.sendCommand(command);
  };
  const chooseHumanCommand = (command: string) => {
    setCommandText(command);
    data.sendCommand(command);
  };
  const foxgloveEnabled = Boolean(foxgloveConfig?.foxglove_ui_enabled);

  if (window.location.pathname === "/health" || window.location.pathname === "/health/") {
    return (
      <HealthDashboard
        connectionStatus={data.connectionStatus}
        state={data.healthDashboardState}
        recovery={data.recoveryState}
        receivedAt={data.healthDashboardReceivedAt}
      />
    );
  }

  if (window.location.pathname === "/arm" || window.location.pathname === "/arm/") {
    return (
      <>
        <header className="app-header">
          <h1>Robot Harness Agent Dashboard</h1>
          <nav className="top-nav" aria-label="Dashboard navigation">
            <a href="/">Agent Dashboard</a>
            <a href="/health">Health Dashboard</a>
            <a className="active" href="/arm">SO101 Arm</a>
          </nav>
          <span className={`pill ${data.connectionStatus === "connected" ? "ok" : "bad"}`}>
            {data.connectionStatus}
          </span>
        </header>
        <main className="arm-page">
          <ArmPanel arm={data.armState} sendArmCommand={data.sendArmCommand} />
        </main>
      </>
    );
  }

  return (
    <>
      <header className="app-header">
        <h1>Robot Harness Agent Dashboard</h1>
        <nav className="top-nav" aria-label="Dashboard navigation">
          <a className="active" href="/">Agent Dashboard</a>
          <a href="/health">Health Dashboard</a>
          <a href="/arm">SO101 Arm</a>
        </nav>
        <span className={`pill ${data.connectionStatus === "connected" ? "ok" : "bad"}`}>
          {data.connectionStatus}
        </span>
      </header>
      <main className="agent-workspace">
        <section className="dashboard-column health-column" aria-label="Health and Recovery">
          <h2>Robot Health Summary</h2>
          <HealthPanel recovery={data.recoveryState} health={data.healthState} dashboardHealth={data.healthDashboardState}
            receivedAt={data.healthDashboardReceivedAt} connected={data.connectionStatus === "connected"} />
          {data.realRobotState && (
            <RealRobotPanel
              realRobot={data.realRobotState}
              robotMode={data.robotMode}
              sendRealRobotCommand={data.sendRealRobotCommand}
            />
          )}
          <h2 style={{ marginTop: 12 }}>Recovery Agent</h2>
          <RecoveryPanel recovery={data.recoveryState} sendCommand={data.sendRecoveryCommand} />
        </section>
        <section className="dashboard-column mission-column" aria-label="Mission execution">
          <MissionPanel
            mission={data.missionState}
            health={data.healthState}
            navigationProgress={data.navigationProgress}
            navigationRouteState={data.navigationRouteState}
            missionEvents={data.missionEvents}
            humanReport={data.humanReportState}
            receivedPending={data.receivedPending}
            pendingCommandText={data.pendingCommandText}
          />
          <EventTimeline events={data.missionState ? data.missionEvents.filter(e => e.mission_id === data.missionState?.mission_id && e.type !== "agent_heartbeat") : []} />
          {foxgloveEnabled && <details><summary>Foxglove</summary>
            <FoxglovePanel
              foxgloveUiPort={foxgloveConfig!.foxglove_ui_port}
              foxgloveBridgePort={foxgloveConfig!.foxglove_bridge_port}
            />
          </details>}
        </section>
        <section className="dashboard-column operator-column" aria-label="Robot controls">
          <h2>ロボット状態</h2>
          <RobotStatusPanel
            robotMode={data.robotMode}
            safetyStatus={data.safetyStatus}
            localizationWarning={data.localizationWarning}
            stationaryDriftM={data.stationaryDriftM}
            agentStatus={data.agentStatus}
            humanPrompt={data.humanPrompt}
          />
          <ControlPanel
            robotMode={data.robotMode}
            cameraPanAngle={data.cameraPanAngle}
            cameraTiltAngle={data.cameraTiltAngle}
            liftPosition={data.liftPosition}
            liftState={data.liftState}
            joySourceStatus={data.joySourceStatus}
            sendJoyState={data.sendJoyState}
            sendJoyControl={data.sendJoyControl}
            onCancel={data.sendCancel}
          />
          <div style={{ marginTop: 18 }}>
            <AgentModePanel
              modeState={data.agentModeState}
              mission={data.missionState}
              onModeChange={data.sendAgentMode}
            />
          </div>
          <div style={{ marginTop: 18 }}>
            <CommandBox
              value={commandText}
              onChange={setCommandText}
              onSend={sendCurrentCommand}
              onCancel={data.sendCancel}
              onQuickCommand={sendQuickCommand}
              quickCommands={quickCommands}
              testCommands={testCommands}
            />
          </div>
          <HumanChoices mission={data.missionState} onChoose={chooseHumanCommand} onCancel={data.sendCancel} onControl={data.sendMissionControl} />
        </section>
      </main>
    </>
  );
}

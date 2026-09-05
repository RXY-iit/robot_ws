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

interface QuickCommand {
  label: string;
  command: string;
}

interface DashboardConfig {
  quick_commands: [string, string][];
  default_command: string;
  foxglove_bridge_port: number;
  foxglove_ui_port: number;
  foxglove_ui_enabled: boolean;
}

export function App() {
  const data = useDashboardSocket();
  const [commandText, setCommandText] = useState("");
  const [quickCommands, setQuickCommands] = useState<QuickCommand[]>([]);
  const [foxgloveConfig, setFoxgloveConfig] = useState<DashboardConfig | null>(null);

  useEffect(() => {
    fetch("/api/config")
      .then((response) => response.json())
      .then((config: DashboardConfig) => {
        setQuickCommands(config.quick_commands.map(([label, command]) => ({ label, command })));
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
      <main className={foxgloveEnabled ? "" : "no-foxglove"}>
        <section>
          <h2>Robot Health Summary</h2>
          <HealthPanel health={data.healthState} dashboardHealth={data.healthDashboardState} />
          {data.realRobotState && (
            <RealRobotPanel
              realRobot={data.realRobotState}
              robotMode={data.robotMode}
              sendRealRobotCommand={data.sendRealRobotCommand}
            />
          )}
          <h2 style={{ marginTop: 12 }}>ロボット状態</h2>
          <RobotStatusPanel
            robotMode={data.robotMode}
            safetyStatus={data.safetyStatus}
            localizationWarning={data.localizationWarning}
            stationaryDriftM={data.stationaryDriftM}
            agentStatus={data.agentStatus}
            humanPrompt={data.humanPrompt}
          />
          <MissionPanel
            mission={data.missionState}
            health={data.healthState}
            navigationProgress={data.navigationProgress}
            navigationRouteState={data.navigationRouteState}
            missionEvents={data.missionEvents}
            receivedPending={data.receivedPending}
            pendingCommandText={data.pendingCommandText}
          />
          <EventTimeline events={data.eventsTimeline} />
        </section>
        {foxgloveEnabled && (
          <section>
            <FoxglovePanel
              foxgloveUiPort={foxgloveConfig!.foxglove_ui_port}
              foxgloveBridgePort={foxgloveConfig!.foxglove_bridge_port}
            />
          </section>
        )}
        <section>
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
            <CommandBox
              value={commandText}
              onChange={setCommandText}
              onSend={sendCurrentCommand}
              onCancel={data.sendCancel}
              onQuickCommand={sendQuickCommand}
              quickCommands={quickCommands}
            />
          </div>
          <HumanChoices mission={data.missionState} onChoose={chooseHumanCommand} onCancel={data.sendCancel} />
        </section>
      </main>
    </>
  );
}

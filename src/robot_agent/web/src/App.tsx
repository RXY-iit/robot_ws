import { useEffect, useState } from "react";
import { useDashboardSocket } from "./lib/useDashboardSocket";
import { HealthPanel } from "./components/HealthPanel";
import { RobotStatusPanel } from "./components/RobotStatusPanel";
import { HumanChoices } from "./components/HumanChoices";
import { CommandBox } from "./components/CommandBox";
import { MissionPanel } from "./components/MissionPanel";
import { EventTimeline } from "./components/EventTimeline";
import { ControlPanel } from "./components/ControlPanel";
import { FoxglovePanel } from "./components/FoxglovePanel";

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

  return (
    <>
      <header>
        <h1>Robot Harness ダッシュボード</h1>
        <span className="pill warn">{data.connectionStatus}</span>
      </header>
      <main>
        <section>
          <h2>Robot Health</h2>
          <HealthPanel health={data.healthState} navigationProgress={data.navigationProgress} />
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
        <section>
          {foxgloveConfig?.foxglove_ui_enabled && (
            <FoxglovePanel
              foxgloveUiPort={foxgloveConfig.foxglove_ui_port}
              foxgloveBridgePort={foxgloveConfig.foxglove_bridge_port}
            />
          )}
        </section>
        <section>
          <ControlPanel
            robotMode={data.robotMode}
            cameraPanAngle={data.cameraPanAngle}
            cameraTiltAngle={data.cameraTiltAngle}
            liftPosition={data.liftPosition}
            liftState={data.liftState}
            sendJoyState={data.sendJoyState}
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

import type { RealRobotState } from "../types";

// Real hardware only -- real_robot_monitor never runs in sim, so realRobotState
// stays null there and this panel renders nothing (see App.tsx).
interface Props {
  realRobot: RealRobotState;
  robotMode: string;
  sendRealRobotCommand: (text: string) => void;
}

export function RealRobotPanel({ realRobot, robotMode, sendRealRobotCommand }: Props) {
  const restart = realRobot.drive_restart || {};
  const virtualDrive = realRobot.drive_virtual_mode === true;

  const restartDrive = () => {
    if (virtualDrive) {
      alert("Oriental Motor is disabled in virtual-drive test mode. Real drive restart is unavailable.");
      return;
    }
    const mode = realRobot.robot_mode || robotMode;
    if (mode !== "MANUAL") {
      alert("Drive motor restart requires MANUAL mode. Press B first.");
      return;
    }
    if (confirm("Confirm all three wheel speeds are zero, Dashboard control is released, physical Joy is neutral, and E-stop is reachable.\n\n3輪の速度ゼロ、Dashboard操作権の解放、物理Joyの中立、および非常停止へ手が届くことを確認してください。\n\nRestart drive motor?")) {
      sendRealRobotCommand("restart_drive_motor");
    }
  };

  return (
    <details className="recovery-panel">
      <summary>Real robot recovery control</summary>
      <p className="small">
        Component health moved to the seven-domain Health Dashboard. This control remains here because that
        dashboard is read-only.
      </p>
      <div className="row">
        <button className="danger" onClick={restartDrive} disabled={virtualDrive}>
          {virtualDrive ? "Drive motor disabled (virtual test mode)" : "Restart drive motor"}
        </button>
      </div>
      <div className="small" style={{ marginTop: 8 }}>
        mode={realRobot.robot_mode || robotMode} / {restart.state || "IDLE"}: {restart.message || ""}
      </div>
      <details style={{ marginTop: 8 }}>
        <summary>Drive restart log</summary>
        <pre>{restart.log_tail || restart.message || "No restart requested."}</pre>
      </details>
    </details>
  );
}

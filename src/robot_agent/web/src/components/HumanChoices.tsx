import type { MissionState } from "../types";
import { clarificationCommand } from "../lib/missionLogic";

interface Props {
  mission: MissionState | null;
  onChoose: (command: string) => void;
  onCancel: () => void;
}

export function HumanChoices({ mission, onChoose, onCancel }: Props) {
  if (!mission || !String(mission.status || "").includes("NEEDS_HUMAN")) return null;

  const message = mission.question_for_human || mission.final_message || "";
  const ambiguityChoices = mission.intent?.constraints?.ambiguity?.choices || [];

  if (ambiguityChoices.length) {
    return (
      <div className="choice-panel">
        <strong>候補を選択してください</strong>
        <div className="block-meta">{message}</div>
        <div className="choice-buttons" style={{ marginTop: 8 }}>
          {ambiguityChoices.map((choice, index) => (
            <button key={index} onClick={() => onChoose(choice.command || "")}>
              {choice.label || `選択 ${index + 1}`}
              <br />
              <small>{choice.command}</small>
            </button>
          ))}
          <button onClick={onCancel}>キャンセル</button>
        </div>
      </div>
    );
  }

  const resolveResult = (mission.skill_results || []).find(
    (item) =>
      item.skill_name === "resolve_semantic_target" &&
      item.code === "NEEDS_HUMAN" &&
      item.metrics?.alternatives &&
      item.metrics.alternatives.length,
  );

  if (!resolveResult) {
    return (
      <div className="choice-panel">
        <strong>人の確認が必要です</strong>
        <div className="block-meta">{message}</div>
        <div className="choice-buttons" style={{ marginTop: 8 }}>
          <button onClick={onCancel}>キャンセル</button>
        </div>
      </div>
    );
  }

  const candidates: string[] = [resolveResult.metrics!.object_id, ...(resolveResult.metrics!.alternatives || [])].filter(
    Boolean,
  );

  return (
    <div className="choice-panel">
      <strong>候補を選択してください</strong>
      <div className="block-meta">{message}</div>
      <div className="choice-buttons" style={{ marginTop: 8 }}>
        {candidates.map((id, index) => {
          const command = clarificationCommand(mission.intent, id);
          return (
            <button key={id} onClick={() => onChoose(command)}>
              選択 {index + 1}: {id}
              <br />
              <small>{command}</small>
            </button>
          );
        })}
        <button onClick={onCancel}>キャンセル</button>
      </div>
    </div>
  );
}

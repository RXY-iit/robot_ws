import type { MissionState } from "../types";
import { clarificationCommand } from "../lib/missionLogic";
import { MissionCancelButton } from "./MissionCancelButton";

interface Props {
  mission: MissionState | null;
  onChoose: (command: string) => void;
  onCancel: () => void;
  onControl: (command: Record<string, unknown>) => void;
}

export function HumanChoices({ mission, onChoose, onCancel, onControl }: Props) {
  if (!mission || !String(mission.status || "").includes("NEEDS_HUMAN")) return null;

  const message = mission.question_for_human || mission.final_message || "";
  const ambiguityChoices = (mission.intent?.constraints?.ambiguity?.choices || []).filter(choice =>
    !/^(cancel(?: all| task| mission)?|stop(?: all| task)?|abort|キャンセル|全体キャンセル|全部取消|停止|中止)[.!。！]*$/i.test((choice.command || "").trim()));
  const wait = mission.human_wait;
  if (wait) return <div className="choice-panel">
    <strong>現在のステップで待機中</strong><div className="block-meta">{message}</div>
    <div className="small">{wait.skill_name} · {wait.auto_resume ? "AUTO 確認後に条件を再チェック" : "再チェックして同じステップから続行"}</div>
    <div className="choice-buttons">
      {wait.actions.filter(action => ["set_auto", "resume"].includes(action) || wait.action_labels?.[action]).map(action => <button key={action}
        onClick={() => onControl({action, mission_id: mission.mission_id, wait_id: wait.wait_id})}>
        {wait.action_labels?.[action] || (action === "set_auto" ? "AUTO に変更して続行" : "条件を再確認して続行")}
      </button>)}
      <MissionCancelButton onCancel={onCancel} />
    </div>
  </div>;

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
          <MissionCancelButton onCancel={onCancel} />
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
          <MissionCancelButton onCancel={onCancel} />
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
        <MissionCancelButton onCancel={onCancel} />
      </div>
    </div>
  );
}

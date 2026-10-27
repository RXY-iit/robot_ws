interface Props {
  onCancel: () => void;
}

export function MissionCancelButton({ onCancel }: Props) {
  return <button className="secondary" onClick={onCancel}
    title="現在の Mission と、その実行中・未実行の全 Step をキャンセル">
    キャンセル（全部） / Cancel all
  </button>;
}

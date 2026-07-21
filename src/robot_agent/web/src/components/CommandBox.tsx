import { useSpeechRecognition } from "../lib/useSpeechRecognition";

interface QuickCommand {
  label: string;
  command: string;
}

interface Props {
  value: string;
  onChange: (value: string) => void;
  onSend: () => void;
  onCancel: () => void;
  onQuickCommand: (command: string) => void;
  quickCommands: QuickCommand[];
}

const SPEECH_LANGS: [string, string][] = [
  ["ja-JP", "日本語"],
  ["en-US", "English"],
  ["zh-CN", "中文"],
];

export function CommandBox({ value, onChange, onSend, onCancel, onQuickCommand, quickCommands }: Props) {
  const speech = useSpeechRecognition(onChange);

  return (
    <>
      <h2>コマンド</h2>
      <textarea id="cmd" value={value} onChange={(event) => onChange(event.target.value)} />
      <div className="row">
        <button onClick={onSend}>送信</button>
        <button className="secondary" onClick={onCancel}>
          キャンセル
        </button>
        {speech.supported && (
          <>
            <button
              className="secondary"
              onClick={speech.toggle}
              style={speech.listening ? { borderColor: "var(--bad)", color: "var(--bad)" } : undefined}
              title="音声入力"
            >
              {speech.listening ? "● 録音中" : "🎤"}
            </button>
            <select
              value={speech.lang}
              onChange={(event) => speech.setLang(event.target.value)}
              disabled={speech.listening}
              style={{ height: 36, borderRadius: 6, border: "1px solid var(--line)" }}
            >
              {SPEECH_LANGS.map(([code, label]) => (
                <option key={code} value={code}>
                  {label}
                </option>
              ))}
            </select>
          </>
        )}
      </div>
      <div className="quick-grid">
        {quickCommands.map((quick) => (
          <button key={quick.label} onClick={() => onQuickCommand(quick.command)} title={quick.command}>
            {quick.label}
          </button>
        ))}
      </div>
    </>
  );
}

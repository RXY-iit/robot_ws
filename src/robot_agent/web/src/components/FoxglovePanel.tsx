interface Props {
  foxgloveUiPort: number;
  foxgloveBridgePort: number;
}

export function FoxglovePanel({ foxgloveUiPort, foxgloveBridgePort }: Props) {
  const host = window.location.hostname;
  const uiUrl = `http://${host}:${foxgloveUiPort}/`;
  const wsUrl = `ws://${host}:${foxgloveBridgePort}`;
  const src = `${uiUrl}?ds=foxglove-websocket&ds.url=${encodeURIComponent(wsUrl)}`;

  return (
    <>
      <div className="section-head">
        <h2>Foxglove</h2>
        <span className="small">{wsUrl}</span>
      </div>
      <iframe
        title="Foxglove"
        src={src}
        style={{ width: "100%", flex: 1, minHeight: 600, border: "1px solid var(--line)", borderRadius: 8 }}
      />
    </>
  );
}

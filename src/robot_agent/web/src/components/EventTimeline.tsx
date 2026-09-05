import type { MissionEvent } from "../types";

interface Props {
  events: MissionEvent[];
}

export function EventTimeline({ events }: Props) {
  return (
    <>
      <h2>ミッションイベント</h2>
      <div className="timeline">
        {events.map((event, index) => (
          <div key={index} className="event">
            <div className="event-title">
              <span>{event.type || "event"}</span>
              <span>{event.time || ""}</span>
            </div>
            <pre>{JSON.stringify(event, null, 2)}</pre>
          </div>
        ))}
      </div>
    </>
  );
}

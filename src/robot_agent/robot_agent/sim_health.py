"""Receiver-side simulation evidence. No actuator or simulator control."""

from collections import deque
import math


class SimClockEvidence:
    def __init__(self):
        self.samples = deque(maxlen=4096)
        self.resets = 0
        self.last_reset = None
        self.last_advance = None

    def update(self, sim, wall):
        if not math.isfinite(sim):
            return
        if self.samples and sim < self.samples[-1][1]:
            self.samples.clear()
            self.resets += 1
            self.last_reset = wall
            self.last_advance = wall
        if not self.samples or sim > self.samples[-1][1]:
            self.last_advance = wall
        self.samples.append((wall, sim))
        while len(self.samples) > 2 and wall - self.samples[0][0] > 10:
            self.samples.popleft()

    def snapshot(self, now):
        if not self.samples:
            return "UNKNOWN", "No /clock received", {"rtf": None}
        first, last = self.samples[0], self.samples[-1]
        duration = last[0] - first[0]
        rtf = (last[1] - first[1]) / duration if duration >= 1 else None
        value = {"rtf": rtf, "window_wall_sec": duration, "sim_time_sec": last[1],
                 "clock_receive_age_sec": now - last[0], "reset_count": self.resets,
                 "not_advancing_sec": now - self.last_advance}
        if now - last[0] > 2:
            value["rtf"] = None
            return "WARN", "Clock delivery stopped: paused simulation or bridge unavailable", value
        if self.last_reset is not None and now - self.last_reset < 10:
            return "WARN", "Simulation time moved backwards; new time epoch", value
        if now - self.last_advance > 2:
            value["rtf"] = 0.0
            return "WARN", "Clock messages received but simulation time is not advancing", value
        if rtf is None:
            return "UNKNOWN", "Collecting clock interval", value
        return ("WARN" if rtf < 0.8 else "OK"), f"RTF {rtf:.3f} over {duration:.1f}s wall time", value


def sensor_timing(snapshot, sim_now):
    age = snapshot.get("receive_age_sec")
    if age is None or age > 10:
        return "UNKNOWN", "No fresh stream evidence"
    stamp = snapshot.get("source_stamp_sec")
    if stamp is None or sim_now is None:
        return "UNKNOWN", "Source timestamp or simulation clock unavailable"
    if (snapshot.get("source_not_advancing_sec") or 0) > 2:
        return "WARN", "Messages arrive but source timestamps are not advancing"
    lag = sim_now - stamp
    if lag < -0.1:
        return "WARN", "Source timestamp is ahead of /clock (clock domain/reset mismatch)"
    if lag > 2:
        return "WARN", f"Source data age {lag:.2f}s in simulation time"
    return "OK", "Timestamp advancing in the simulation clock domain"

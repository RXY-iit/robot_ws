"""Time-aligned wheel/LiDAR-inertial motion comparison for investigation.

This is an observation, not a slip detector. Each source's path length and
turn magnitude are frame invariant, so they can be compared without asserting
that their global position origins are identical.
"""

from collections import deque
import math


class MotionPairWindow:
    def __init__(self, *, window_sec=5.0, max_skew_sec=0.25):
        self.window_sec = float(window_sec)
        self.max_skew_sec = float(max_skew_sec)
        self.samples = {"wheel": deque(maxlen=300), "lio": deque(maxlen=300)}

    def observe(self, source, stamp, x, y, yaw):
        if source not in self.samples:
            raise ValueError("Unknown motion source")
        values = (float(stamp), float(x), float(y), float(yaw))
        if not all(math.isfinite(v) for v in values):
            return False
        buf = self.samples[source]
        if buf and values[0] < buf[-1][0]:
            buf.clear()  # New sim epoch or upstream reset.
        if buf and values[0] == buf[-1][0]:
            return False
        buf.append(values)
        while buf and values[0] - buf[0][0] > self.window_sec + 2.0:
            buf.popleft()
        return True

    @staticmethod
    def _nearest(buf, stamp):
        return min(range(len(buf)), key=lambda i: abs(buf[i][0] - stamp))

    @staticmethod
    def _travel(buf, begin, end):
        distance = 0.0
        turn = 0.0
        for a, b in zip(list(buf)[begin:end], list(buf)[begin + 1:end + 1]):
            distance += math.hypot(b[1] - a[1], b[2] - a[2])
            turn += abs(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])))
        return distance, turn

    def assess(self):
        wheel, lio = self.samples["wheel"], self.samples["lio"]
        if len(wheel) < 2 or len(lio) < 2:
            return None
        end = min(wheel[-1][0], lio[-1][0])
        begin = max(wheel[0][0], lio[0][0], end - self.window_sec)
        if end - begin < 2.0:
            return None
        indices = {key: (self._nearest(buf, begin), self._nearest(buf, end))
                   for key, buf in self.samples.items()}
        if any(start >= finish or abs(buf[start][0] - begin) > self.max_skew_sec
               or abs(buf[finish][0] - end) > self.max_skew_sec
               for key, buf in self.samples.items()
               for start, finish in [indices[key]]):
            return None
        wheel_path, wheel_turn = self._travel(wheel, *indices["wheel"])
        lio_path, lio_turn = self._travel(lio, *indices["lio"])
        duration = min(wheel[indices["wheel"][1]][0], lio[indices["lio"][1]][0]) - begin
        # LIO may jump or drift after a restart while the wheels are still.
        # That is useful evidence, but must never be presented as wheel slip.
        wheel_moving = wheel_path >= 0.10 or wheel_turn >= 0.10
        lio_moving = lio_path >= 0.10 or lio_turn >= 0.10
        return {
            "end_source_sec": round(end, 3),
            "window_source_sec": round(duration, 3),
            "wheel_path_m": round(wheel_path, 4),
            "lio_path_m": round(lio_path, 4),
            "path_difference_m": round(wheel_path - lio_path, 4),
            "path_difference_fraction": (round((wheel_path - lio_path) / max(lio_path, 0.1), 4)
                                         if wheel_moving else None),
            "wheel_abs_turn_rad": round(wheel_turn, 4),
            "lio_abs_turn_rad": round(lio_turn, 4),
            "turn_difference_rad": round(wheel_turn - lio_turn, 4),
            "motion_observed": wheel_moving,
            "lio_motion_while_wheel_stationary": lio_moving and not wheel_moving,
            "method": "same source-time interval; separate path-length/absolute-turn integrals",
            "limitation": "not a slip classification; LIO and wheel can share faults",
        }

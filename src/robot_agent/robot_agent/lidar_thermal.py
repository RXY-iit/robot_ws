"""LiDAR temperature evidence. Sim thresholds are experimental, not device limits."""

from collections import deque
import math


class LidarThermalEvidence:
    def __init__(self, *, sim_warn_c=65.0, sim_bad_c=78.0, forecast_sec=60.0, real_warn_c=None):
        if not 0 < sim_warn_c < sim_bad_c or forecast_sec <= 0:
            raise ValueError("Invalid simulated thermal thresholds")
        self.sim_warn_c = sim_warn_c
        self.sim_bad_c = sim_bad_c
        self.forecast_sec = forecast_sec
        self.real_warn_c = real_warn_c if real_warn_c is not None and real_warn_c > 0 else None
        self.samples = {}

    @staticmethod
    def temperatures(payload, *, sim):
        if not isinstance(payload, dict):
            return {}
        if sim:
            values = {"sim_lidar": payload.get("core_temp_c")}
        else:
            values = {}
            for device in payload.get("devices", []) or []:
                if not isinstance(device, dict) or not isinstance(device.get("state"), dict):
                    continue
                identity = str(device.get("ip") or device.get("handle") or "unknown")
                raw = device["state"].get("core_temp")
                try:
                    values[identity] = float(raw) / 100.0  # MID-360 SDK unit: 0.01 C
                except (ValueError, TypeError):
                    continue
        result = {}
        for identity, value in values.items():
            try:
                temp = float(value)
            except (ValueError, TypeError):
                continue
            if math.isfinite(temp) and -40.0 <= temp <= 125.0:
                result[identity] = temp
        return result

    def observe(self, payload, *, source_sec, sim):
        try:
            t = float(source_sec)
        except (TypeError, ValueError):
            return
        if not math.isfinite(t):
            return
        for identity, temp in self.temperatures(payload, sim=sim).items():
            samples = self.samples.setdefault(identity, deque(maxlen=120))
            if samples and t < samples[-1][0]:
                samples.clear()  # Isaac Play/reset: never bridge two time epochs.
            if samples and t == samples[-1][0]:
                continue
            samples.append((t, temp))
            while samples and t - samples[0][0] > 30.0:
                samples.popleft()

    def assess(self, *, sim):
        if not self.samples:
            return "UNKNOWN", "No valid LiDAR core temperature", None
        readings = []
        for identity, samples in self.samples.items():
            if not samples:
                continue
            t, temp = samples[-1]
            span = t - samples[0][0]
            slope = (temp - samples[0][1]) / span if span >= 5.0 and len(samples) >= 3 else None
            readings.append((identity, temp, slope))
        if not readings:
            return "UNKNOWN", "No valid LiDAR core temperature", None
        identity, temp, slope = max(readings, key=lambda row: row[1])
        details = {"device": identity, "core_temp_c": round(temp, 3),
                   "rise_c_per_sec_30s": None if slope is None else round(slope, 5),
                   "threshold_basis": "sim_experiment" if sim else "native_diagnostic_only"}
        if not sim:
            if self.real_warn_c is not None:
                projected = temp + max(0.0, slope or 0.0) * self.forecast_sec
                details.update(configured_warn_c=self.real_warn_c, projected_c=round(projected, 3),
                               threshold_basis="operator_calibrated_real_warning")
                if temp >= self.real_warn_c or (slope is not None and slope >= .03 and projected >= self.real_warn_c):
                    return "WARN", f"MID-360 temperature {temp:.1f} C is approaching calibrated warning threshold", details
            return "OBSERVED", f"MID-360 core temperature {temp:.1f} C; native diagnostic determines severity", details
        projected = temp + max(0.0, slope or 0.0) * self.forecast_sec
        details.update(sim_warn_c=self.sim_warn_c, sim_bad_c=self.sim_bad_c,
                       projected_c=round(projected, 3))
        if temp >= self.sim_bad_c:
            return "ERROR", f"Sim LiDAR thermal model {temp:.1f} C reached experimental BAD threshold", details
        if temp >= self.sim_warn_c or (slope is not None and slope >= 0.03 and projected >= self.sim_warn_c):
            return "WARN", f"Sim LiDAR temperature {temp:.1f} C rising; projected {projected:.1f} C in {self.forecast_sec:.0f} sim s", details
        return "OK", f"Sim LiDAR temperature {temp:.1f} C; no modeled thermal warning", details

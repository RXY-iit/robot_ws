#!/usr/bin/env python3
"""Sim-only LiDAR thermal model; optional upstream cloud-loss coupling.

Command /b1/lidar_thermal_command (std_msgs/String): start, cool, stop.
The cool command represents an operator-verified cooling intervention, not a
physical cooling actuator. No command is sent to the robot or Nav2.
"""

import argparse
import json
import math
import os
from pathlib import Path
import time


class ThermalScenario:
    def __init__(self, initial_c=40.0, peak_c=85.0, warm_c_per_sec=0.5, cool_c_per_sec=1.0):
        if not (0 < initial_c < peak_c <= 125 and warm_c_per_sec > 0 and cool_c_per_sec > 0):
            raise ValueError("Finite positive thermal model parameters required")
        self.initial_c = initial_c
        self.peak_c = peak_c
        self.warm_c_per_sec = warm_c_per_sec
        self.cool_c_per_sec = cool_c_per_sec
        self.temp_c = initial_c
        self.phase = "idle"
        self.sim_sec = None
        self.run = 0

    def command(self, value):
        if value == "start" and self.phase in {"idle", "cooling"}:
            self.phase = "warming"
            self.run += 1
            return True
        if value == "cool" and self.phase in {"warming", "holding"}:
            self.phase = "cooling"
            return True
        if value == "stop":
            self.phase = "idle"
            return True
        return False

    def advance(self, sim_sec):
        t = float(sim_sec)
        if not math.isfinite(t):
            return "invalid_clock"
        if self.sim_sec is not None and t < self.sim_sec:
            self.sim_sec = t
            self.phase = "idle"
            self.temp_c = self.initial_c
            return "clock_reset"
        dt = 0.0 if self.sim_sec is None else max(0.0, t - self.sim_sec)
        self.sim_sec = t
        if self.phase == "warming":
            self.temp_c = min(self.peak_c, self.temp_c + dt * self.warm_c_per_sec)
            if self.temp_c >= self.peak_c:
                self.phase = "holding"
        elif self.phase == "cooling":
            self.temp_c = max(self.initial_c, self.temp_c - dt * self.cool_c_per_sec)
            if self.temp_c <= self.initial_c:
                self.phase = "idle"
        return "advanced"

    def state(self, *, cloud_effect):
        return {"model_version": 1, "sim_sec": self.sim_sec, "core_temp_c": round(self.temp_c, 4),
                "phase": self.phase, "run": self.run, "cloud_effect_requested": cloud_effect,
                "basis": "simulated core temperature, not MID-360 device telemetry"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--initial-c", type=float, default=40.0)
    parser.add_argument("--peak-c", type=float, default=85.0)
    parser.add_argument("--warm-c-per-sim-sec", type=float, default=0.5)
    parser.add_argument("--cool-c-per-sim-sec", type=float, default=1.0)
    parser.add_argument("--cloud-effect", action="store_true",
                        help="At 78 C request the already running upstream LiDAR relay thermal profile; requires fresh B1 capture")
    parser.add_argument("--log", type=Path, default=None)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "20" or os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp":
        parser.error("Requires ROS_DOMAIN_ID=20 and RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
    model = ThermalScenario(args.initial_c, args.peak_c, args.warm_c_per_sim_sec, args.cool_c_per_sim_sec)
    output = args.log or Path("debug-output") / f"b1-lidar-thermal-{time.strftime('%Y%m%d-%H%M%S')}.jsonl"
    output.parent.mkdir(parents=True, exist_ok=True)
    log = output.open("x", buffering=1)

    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    class ThermalNode(Node):
        def __init__(self):
            super().__init__("b1_lidar_thermal_injector")
            self.clock_received_wall = 0.0
            self.clock_advanced_wall = 0.0
            self.cloud_status = None
            self.cloud_status_wall = 0.0
            self.capture = None
            self.capture_wall = 0.0
            self.cloud_owned = False
            self.last_cloud_command = 0.0
            self.last_reported_sim = None
            self.publisher = self.create_publisher(String, "/b1/sim_lidar_thermal_state", 10)
            self.cloud_command = self.create_publisher(String, "/b1/lidar_injection_command", 10)
            self.create_subscription(Clock, "/clock", self.clock_cb, 10)
            self.create_subscription(String, "/b1/lidar_thermal_command", self.command_cb, 10)
            self.create_subscription(String, "/b1/lidar_injection_status", self.cloud_status_cb, 10)
            self.create_subscription(String, "/b1/capture_status", self.capture_cb, 10)
            self.create_timer(0.5, self.heartbeat)
            self.record("model_started", parameters=vars(args))

        def record(self, event, **fields):
            values = {"event": event, "wall_unix_sec": time.time(), "sim_sec": model.sim_sec,
                      "temperature_c": model.temp_c, "phase": model.phase, **fields}
            log.write(json.dumps(values, default=str) + "\n")

        def clock_cb(self, msg):
            before = model.sim_sec
            result = model.advance(msg.clock.sec + msg.clock.nanosec / 1e9)
            self.clock_received_wall = time.monotonic()
            if model.sim_sec != before:
                self.clock_advanced_wall = self.clock_received_wall
            if result == "clock_reset":
                self.stop_owned_cloud("clock_reset")
                self.record("clock_reset")

        def command_cb(self, msg):
            command = msg.data.strip().lower()
            if command == "start" and (model.sim_sec is None or time.monotonic() - self.clock_advanced_wall > 2):
                self.record("command_rejected", command=command, reason="No advancing Isaac clock")
                return
            if command == "start" and args.cloud_effect and (not self.cloud_ready() or (self.cloud_status or {}).get("active")):
                self.record("command_rejected", command=command, reason="Upstream relay and fresh B1 capture required")
                return
            accepted = model.command(command)
            self.record("command_accepted" if accepted else "command_rejected", command=command)
            if command == "stop" and accepted:
                self.stop_owned_cloud("model_stopped")

        def cloud_status_cb(self, msg):
            try:
                self.cloud_status = json.loads(msg.data)
                self.cloud_status_wall = time.monotonic()
            except ValueError:
                self.cloud_status = None

        def capture_cb(self, msg):
            try:
                self.capture = json.loads(msg.data)
                self.capture_wall = time.monotonic()
            except ValueError:
                self.capture = None

        def cloud_ready(self):
            now = time.monotonic()
            state = self.cloud_status or {}
            capture = self.capture or {}
            ages = capture.get("ages") or {}
            counts = capture.get("counts") or {}
            return (now - self.cloud_status_wall < 3 and self.count_publishers("/b1/lidar_injection_status") == 1
                    and state.get("scope") == "upstream" and state.get("input_age_wall_sec", 99) < 2
                    and now - self.capture_wall < 3 and capture.get("recording") is True
                    and all(counts.get(topic, 0) > 0 and ages.get(topic, 99) < 3
                            for topic in ("/clock", "/b1/ground_truth_odom")))

        def stop_owned_cloud(self, reason):
            if self.cloud_owned and (self.cloud_status or {}).get("active"):
                self.cloud_command.publish(String(data="stop"))
                self.record("cloud_stop_requested", reason=reason)
            self.cloud_owned = False

        def heartbeat(self):
            if model.sim_sec is None or time.monotonic() - self.clock_advanced_wall > 2:
                self.stop_owned_cloud("clock_stale")
                return
            self.publisher.publish(String(data=json.dumps(model.state(cloud_effect=args.cloud_effect))))
            if self.last_reported_sim is None or model.sim_sec - self.last_reported_sim >= 1:
                self.record("thermal_sample", **model.state(cloud_effect=args.cloud_effect))
                self.last_reported_sim = model.sim_sec
            if not args.cloud_effect:
                return
            if model.temp_c < 72:
                self.stop_owned_cloud("cooled_below_threshold")
                return
            if model.temp_c < 78 or model.phase == "idle" or time.monotonic() - self.last_cloud_command < 5:
                return
            if not self.cloud_ready():
                self.record("cloud_effect_blocked", reason="Upstream relay/capture unavailable")
                return
            if not (self.cloud_status or {}).get("active"):
                self.cloud_command.publish(String(data=json.dumps({"command": "start", "profile": "thermal"})))
                self.cloud_owned = True
                self.last_cloud_command = time.monotonic()
                self.record("cloud_start_requested", profile="thermal", scope="upstream")

    rclpy.init()
    node = ThermalNode()
    print("Thermal model ready; log:", output, flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_owned_cloud("shutdown")
        node.record("model_stopped")
        node.destroy_node()
        log.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

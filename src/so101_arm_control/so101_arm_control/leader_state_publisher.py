#!/usr/bin/env python3

"""Read-only SO101 leader adapter for the guarded follower control server.

This process owns only the 5 V leader serial port.  It never calls
``SOLeader.connect()`` because that high-level path configures motors.  Runtime
operation is restricted to handshake and register reads; follower commands stay
inside ``real_trajectory_server.py``.
"""

from __future__ import annotations

import fcntl
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from so101_arm_control.read_only_probe import MOTOR_NAMES, SafetyViolation


CONFIG_PATH = Path(
    "/home/matsunaga-h/robot_ws/src/so101_arm_control/config/arm_real_dashboard_test.yaml"
)
LEADER_SERIAL_OWNER_LOCK = Path("/tmp/so101_leader_5AE6084777.lock")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_leader_calibration(
    path: Path, expected_sha256: str
) -> dict[str, dict[str, int]]:
    if not path.is_file():
        raise SafetyViolation(f"Leader calibration is missing: {path}")
    actual_sha256 = sha256_file(path)
    if actual_sha256 != expected_sha256:
        raise SafetyViolation(
            "Leader calibration SHA-256 mismatch: "
            f"expected {expected_sha256}, got {actual_sha256}"
        )
    data = json.loads(path.read_text(encoding="utf-8"))
    if tuple(data) != MOTOR_NAMES:
        raise SafetyViolation(
            f"Unexpected leader motor order/names: {tuple(data)}"
        )
    for expected_id, name in enumerate(MOTOR_NAMES, start=1):
        entry = data[name]
        if int(entry.get("id", -1)) != expected_id:
            raise SafetyViolation(f"Unexpected leader motor id for {name}: {entry}")
        if int(entry.get("drive_mode", -1)) != 0:
            raise SafetyViolation(f"Unexpected leader drive mode for {name}: {entry}")
        minimum = int(entry.get("range_min", -1))
        maximum = int(entry.get("range_max", -1))
        if not 0 <= minimum < maximum <= 4095:
            raise SafetyViolation(f"Invalid leader calibration range for {name}: {entry}")
    return data


def validate_leader_port(port: Path, expected_serial: str) -> Path:
    if not port.is_symlink():
        raise SafetyViolation(f"Stable leader by-id symlink is missing: {port}")
    if expected_serial not in port.name:
        raise SafetyViolation(
            f"Leader stable port does not contain serial {expected_serial}: {port}"
        )
    resolved = port.resolve(strict=True)
    if not resolved.name.startswith("ttyACM"):
        raise SafetyViolation(f"Unexpected leader device target: {resolved}")
    return resolved


def make_read_only_leader_bus(
    port: Path, calibration_data: dict[str, dict[str, int]]
) -> Any:
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    class ReadOnlyLeaderBus(FeetechMotorsBus):
        def _blocked(self, *_args: Any, **_kwargs: Any) -> None:
            raise SafetyViolation("A motor write path was reached in the leader reader")

        write = _blocked
        sync_write = _blocked
        _write = _blocked
        configure_motors = _blocked
        write_calibration = _blocked
        disable_torque = _blocked
        enable_torque = _blocked

    calibration = {
        name: MotorCalibration(**calibration_data[name]) for name in MOTOR_NAMES
    }
    motors = {
        name: Motor(
            calibration_data[name]["id"],
            "sts3215",
            MotorNormMode.RANGE_0_100
            if name == "gripper"
            else MotorNormMode.RANGE_M100_100,
        )
        for name in MOTOR_NAMES
    }
    return ReadOnlyLeaderBus(
        port=str(port), motors=motors, calibration=calibration
    )


def valid_normalized_positions(values: dict[str, Any]) -> dict[str, float]:
    if set(values) != set(MOTOR_NAMES):
        raise SafetyViolation("Leader feedback does not contain exactly six motors")
    normalized: dict[str, float] = {}
    for name in MOTOR_NAMES:
        value = float(values[name])
        lower, upper = (0.0, 100.0) if name == "gripper" else (-100.0, 100.0)
        if not math.isfinite(value) or value < lower - 1.0 or value > upper + 1.0:
            raise SafetyViolation(
                f"Leader normalized position is invalid for {name}: {value}"
            )
        normalized[name] = min(upper, max(lower, value))
    return normalized


def unwrap_cyclic_position(
    current_wrapped: float,
    previous_wrapped: float | None,
    previous_unwrapped: float | None,
    *,
    period: float = 200.0,
) -> float:
    """Continue a cyclic normalized position through the -100/100 boundary."""
    if previous_wrapped is None or previous_unwrapped is None:
        return current_wrapped
    delta = current_wrapped - previous_wrapped
    half_period = period / 2.0
    if delta > half_period:
        delta -= period
    elif delta < -half_period:
        delta += period
    return previous_unwrapped + delta


class LeaderStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("so101_leader_state_publisher")
        config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
        leader = dict(config.get("leader", {}))
        self._enabled = bool(leader.get("enabled", False))
        self._port = Path(str(leader.get("stable_port", "")))
        self._serial = str(leader.get("serial", ""))
        calibration = dict(leader.get("calibration", {}))
        self._calibration_path = Path(str(calibration.get("path", "")))
        self._calibration_sha256 = str(calibration.get("sha256", ""))
        self._calibration_id = str(calibration.get("id", ""))
        self._publish_rate_hz = float(leader.get("publish_rate_hz", 20.0))
        self._health_every_samples = max(
            1, int(round(self._publish_rate_hz / float(leader.get("health_rate_hz", 5.0))))
        )
        self._reconnect_period_sec = float(leader.get("reconnect_period_sec", 1.0))
        self._publisher = self.create_publisher(String, "/arm/leader_status", 10)
        self._bus = None
        self._connected = False
        self._lock_file = None
        self._last_connect_attempt = 0.0
        self._sample_count = 0
        self._last_error = "startup"
        self._last_health: dict[str, Any] = {}
        self._last_wrist_wrapped: float | None = None
        self._wrist_unwrapped: float | None = None
        self.create_timer(1.0 / self._publish_rate_hz, self._tick)
        self.get_logger().warning(
            "SO101 leader adapter is READ-ONLY; leader torque must remain OFF"
        )

    def _publish(self, payload: dict[str, Any]) -> None:
        payload.update(
            {
                "schema_version": 1,
                "enabled": self._enabled,
                "connected": self._connected,
                "serial": self._serial,
                "stable_port": str(self._port),
                "calibration_id": self._calibration_id,
                "calibration_sha256": self._calibration_sha256,
                "writes_blocked": True,
            }
        )
        self._publisher.publish(
            String(data=json.dumps(payload, sort_keys=True, allow_nan=False))
        )

    def _acquire_lock(self) -> None:
        if self._lock_file is not None:
            return
        lock_file = LEADER_SERIAL_OWNER_LOCK.open("a+")
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            lock_file.close()
            raise SafetyViolation("Leader serial port is already owned") from exc
        self._lock_file = lock_file

    def _connect(self) -> None:
        if not self._enabled:
            raise SafetyViolation("Leader integration is disabled in config")
        validate_leader_port(self._port, self._serial)
        calibration = load_leader_calibration(
            self._calibration_path, self._calibration_sha256
        )
        self._acquire_lock()
        bus = make_read_only_leader_bus(self._port, calibration)
        try:
            bus.connect(handshake=True)
            torque = bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
            enabled = {
                name: int(value) for name, value in torque.items() if int(value) != 0
            }
            if enabled:
                raise SafetyViolation(
                    f"Leader torque must be OFF before teleoperation: {enabled}"
                )
        except Exception:
            if bus.is_connected:
                bus.disconnect(disable_torque=False)
            raise
        self._bus = bus
        self._connected = True
        self._last_error = ""
        self.get_logger().info(
            f"Leader connected read-only: {self._port} ({self._serial})"
        )

    def _disconnect(self) -> None:
        bus = self._bus
        self._bus = None
        self._connected = False
        self._last_wrist_wrapped = None
        self._wrist_unwrapped = None
        if bus is not None and bus.is_connected:
            try:
                bus.disconnect(disable_torque=False)
            except Exception as exc:
                self.get_logger().error(f"Leader disconnect failed: {exc}")

    def _tick(self) -> None:
        now = time.monotonic()
        if not self._connected:
            if now - self._last_connect_attempt >= self._reconnect_period_sec:
                self._last_connect_attempt = now
                try:
                    self._connect()
                except Exception as exc:
                    self._last_error = str(exc)
            if not self._connected:
                self._publish(
                    {
                        "healthy": False,
                        "torque_free": False,
                        "state": "UNAVAILABLE",
                        "reason": self._last_error,
                    }
                )
                return

        try:
            positions = valid_normalized_positions(
                self._bus.sync_read("Present_Position", num_retry=2)
            )
            wrapped_wrist = positions["wrist_roll"]
            self._wrist_unwrapped = unwrap_cyclic_position(
                wrapped_wrist,
                self._last_wrist_wrapped,
                self._wrist_unwrapped,
            )
            self._last_wrist_wrapped = wrapped_wrist
            positions["wrist_roll"] = self._wrist_unwrapped
            raw_positions = self._bus.sync_read(
                "Present_Position", normalize=False, num_retry=2
            )
            self._sample_count += 1
            if not self._last_health or self._sample_count % self._health_every_samples == 0:
                torque = self._bus.sync_read(
                    "Torque_Enable", normalize=False, num_retry=2
                )
                self._last_health = {
                    "torque_enable": {name: int(torque[name]) for name in MOTOR_NAMES},
                    "voltage_deci_v": self._bus.sync_read(
                        "Present_Voltage", normalize=False, num_retry=2
                    ),
                    "temperature_c": self._bus.sync_read(
                        "Present_Temperature", normalize=False, num_retry=2
                    ),
                    "current_raw": self._bus.sync_read(
                        "Present_Current", normalize=False, num_retry=2
                    ),
                }
            torque_free = all(
                int(value) == 0
                for value in self._last_health.get("torque_enable", {}).values()
            ) and len(self._last_health.get("torque_enable", {})) == len(MOTOR_NAMES)
            self._publish(
                {
                    "healthy": torque_free,
                    "torque_free": torque_free,
                    "state": "READY" if torque_free else "UNSAFE_TORQUE_ON",
                    "reason": "fresh calibrated leader feedback"
                    if torque_free
                    else "one or more leader motors have Torque_Enable != 0",
                    "positions_norm": positions,
                    "raw_position": {
                        name: int(raw_positions[name]) for name in MOTOR_NAMES
                    },
                    "sample_count": self._sample_count,
                    **self._last_health,
                }
            )
        except Exception as exc:
            self._last_error = f"Leader read failed: {exc}"
            self.get_logger().error(self._last_error)
            self._disconnect()
            self._publish(
                {
                    "healthy": False,
                    "torque_free": False,
                    "state": "UNAVAILABLE",
                    "reason": self._last_error,
                }
            )

    def close(self) -> None:
        self._disconnect()
        if self._lock_file is not None:
            fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
            self._lock_file.close()
            self._lock_file = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeaderStatePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

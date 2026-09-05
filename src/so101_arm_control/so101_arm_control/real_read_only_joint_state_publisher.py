#!/usr/bin/env python3

"""Publish real SO101 feedback to ROS 2 without exposing a command path."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from so101_arm_control.read_only_probe import (
    EXPECTED_CALIBRATION,
    EXPECTED_CALIBRATION_SHA256,
    EXPECTED_PORT,
    MOTOR_NAMES,
    SafetyViolation,
    _make_write_blocked_bus,
    load_and_validate_calibration,
    validate_port,
)


JOINT_SPECS = {
    "shoulder_pan": ("shoulder_pan_joint", -1.91986, 1.91986),
    "shoulder_lift": ("shoulder_lift_joint", -1.74533, 1.74533),
    "elbow_flex": ("elbow_flex_joint", -1.69, 1.54),
    "wrist_flex": ("wrist_flex_joint", -1.6, 1.6),
    "wrist_roll": ("wrist_roll_joint", -2.3, 2.3),
    "gripper": ("gripper_joint", 0.0, 1.7),
}


def raw_positions_to_urdf(
    raw_positions: dict[str, int | float],
    calibration: dict[str, dict[str, int]],
    max_outside_ticks: int,
) -> tuple[list[str], list[float], dict[str, int]]:
    names: list[str] = []
    positions: list[float] = []
    clipped_ticks: dict[str, int] = {}

    for motor_name in MOTOR_NAMES:
        raw = int(raw_positions[motor_name])
        minimum = int(calibration[motor_name]["range_min"])
        maximum = int(calibration[motor_name]["range_max"])
        outside = max(minimum - raw, raw - maximum, 0)
        if outside > max_outside_ticks:
            raise SafetyViolation(
                f"{motor_name}={raw} is {outside} ticks outside calibrated range "
                f"[{minimum}, {maximum}] (tolerance={max_outside_ticks})"
            )

        bounded = min(maximum, max(minimum, raw))
        if bounded != raw:
            clipped_ticks[motor_name] = raw - bounded
        joint_name, lower, upper = JOINT_SPECS[motor_name]
        fraction = (bounded - minimum) / (maximum - minimum)
        radians = lower + fraction * (upper - lower)
        if not math.isfinite(radians):
            raise SafetyViolation(f"Non-finite URDF position produced for {motor_name}")
        names.append(joint_name)
        positions.append(radians)

    return names, positions, clipped_ticks


class RealReadOnlyJointStatePublisher(Node):
    def __init__(
        self,
        *,
        port: Path,
        calibration_path: Path,
        rate_hz: float,
        max_outside_ticks: int,
    ) -> None:
        super().__init__("so101_real_read_only_joint_state_publisher")
        validate_port(port)
        self._calibration = load_and_validate_calibration(
            calibration_path, EXPECTED_CALIBRATION_SHA256
        )
        self._max_outside_ticks = max_outside_ticks
        self._bus = _make_write_blocked_bus(port, self._calibration)
        self._connected = False
        self._faulted = False
        self._sample_count = 0
        self._last_raw: dict[str, int | float] = {}
        self._last_clipped: dict[str, int] = {}

        self._joint_pub = self.create_publisher(JointState, "/joint_states", 20)
        self._status_pub = self.create_publisher(String, "/arm/hardware_status", 10)

        self._bus.connect(handshake=True)
        self._connected = True
        torque = self._bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
        enabled = {name: int(value) for name, value in torque.items() if int(value) != 0}
        if enabled:
            raise SafetyViolation(f"Torque is unexpectedly enabled: {enabled}")

        self._timer = self.create_timer(1.0 / rate_hz, self._read_and_publish)
        self._health_timer = self.create_timer(1.0, self._publish_health)
        self.get_logger().warning(
            "REAL READ-ONLY mode: motor writes are blocked; no command/action interface exists"
        )

    def _read_and_publish(self) -> None:
        if self._faulted:
            return
        try:
            raw = self._bus.sync_read("Present_Position", normalize=False, num_retry=2)
            names, positions, clipped = raw_positions_to_urdf(
                raw, self._calibration, self._max_outside_ticks
            )
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = names
            message.position = positions
            self._joint_pub.publish(message)
            self._sample_count += 1
            self._last_raw = raw
            self._last_clipped = clipped
        except Exception as exc:
            self._enter_fault(str(exc))

    def _publish_health(self) -> None:
        if self._faulted:
            return
        try:
            torque = self._bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
            enabled = {name: int(value) for name, value in torque.items() if int(value) != 0}
            if enabled:
                raise SafetyViolation(f"Torque became enabled: {enabled}")
            voltage = self._bus.sync_read("Present_Voltage", normalize=False, num_retry=2)
            temperature = self._bus.sync_read("Present_Temperature", normalize=False, num_retry=2)
            current = self._bus.sync_read("Present_Current", normalize=False, num_retry=2)
            payload: dict[str, Any] = {
                "mode": "REAL_READ_ONLY",
                "healthy": True,
                "writes_blocked": True,
                "command_interface_available": False,
                "calibration_sha256": EXPECTED_CALIBRATION_SHA256,
                "samples": self._sample_count,
                "raw_position": self._last_raw,
                "display_clipped_ticks": self._last_clipped,
                "torque_enable": torque,
                "voltage_deci_v": voltage,
                "temperature_c": temperature,
                "current_raw": current,
            }
            msg = String()
            msg.data = json.dumps(payload, sort_keys=True)
            self._status_pub.publish(msg)
        except Exception as exc:
            self._enter_fault(str(exc))

    def _enter_fault(self, reason: str) -> None:
        self._faulted = True
        self.get_logger().fatal(f"READ-ONLY publisher stopped: {reason}")
        message = String()
        message.data = json.dumps(
            {
                "mode": "REAL_READ_ONLY",
                "healthy": False,
                "writes_blocked": True,
                "command_interface_available": False,
                "fault": reason,
            },
            sort_keys=True,
        )
        self._status_pub.publish(message)

    def close_bus(self) -> None:
        if self._connected:
            # Never call the normal torque-disabling disconnect: that is a write.
            self._bus.disconnect(disable_torque=False)
            self._connected = False


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=Path, default=EXPECTED_PORT)
    parser.add_argument("--calibration", type=Path, default=EXPECTED_CALIBRATION)
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--max-outside-ticks", type=int, default=5)
    parser.add_argument("--ack-read-only", action="store_true")
    return parser.parse_known_args()


def main() -> int:
    args, ros_args = parse_args()
    if not args.ack_read_only:
        raise SystemExit("REFUSED: --ack-read-only is required")
    if not 0.5 <= args.rate_hz <= 20.0:
        raise SystemExit("REFUSED: --rate-hz must be between 0.5 and 20")
    if not 0 <= args.max_outside_ticks <= 5:
        raise SystemExit("REFUSED: --max-outside-ticks must be between 0 and 5")

    rclpy.init(args=ros_args)
    node: RealReadOnlyJointStatePublisher | None = None
    try:
        node = RealReadOnlyJointStatePublisher(
            port=args.port,
            calibration_path=args.calibration,
            rate_hz=args.rate_hz,
            max_outside_ticks=args.max_outside_ticks,
        )
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    finally:
        if node is not None:
            node.close_bus()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

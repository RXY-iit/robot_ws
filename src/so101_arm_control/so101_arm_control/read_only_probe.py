#!/usr/bin/env python3

"""One-shot, write-blocked health probe for the local SO-ARM101 follower.

This utility deliberately uses ``FeetechMotorsBus`` directly.  It must not use
``SOFollower.connect()``, because that higher-level method configures motors and
writes registers during connection.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any


EXPECTED_PORT = Path("/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6054086-if00")
EXPECTED_CALIBRATION = Path(
    "/home/matsunaga-h/.cache/huggingface/lerobot/calibration/robots/so_follower/"
    "follower_12v_recal_20260831_v2.json"
)
EXPECTED_CALIBRATION_SHA256 = "276e40239a27baa4b910d9f1f384e934bf70265e8f558fbc51e4266ffcf5f68c"
MOTOR_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
READ_ONLY_REGISTERS = (
    # Torque_Enable is writable in the control table, but this probe only reads
    # it.  Its value is needed before asking a person to reposition the arm.
    "Torque_Enable",
    "Present_Position",
    "Present_Voltage",
    "Present_Temperature",
    "Present_Current",
    "Moving",
)


class SafetyViolation(RuntimeError):
    """Raised when a write path is reached or a safety invariant fails."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_and_validate_calibration(path: Path, expected_sha256: str) -> dict[str, dict[str, int]]:
    actual_sha256 = sha256_file(path)
    if actual_sha256 != expected_sha256:
        raise SafetyViolation(
            f"Calibration SHA-256 mismatch: expected {expected_sha256}, got {actual_sha256}"
        )

    data = json.loads(path.read_text(encoding="utf-8"))
    if tuple(data) != MOTOR_NAMES:
        raise SafetyViolation(f"Unexpected motor order/names in calibration: {tuple(data)}")

    for expected_id, name in enumerate(MOTOR_NAMES, start=1):
        entry = data[name]
        if entry.get("id") != expected_id:
            raise SafetyViolation(f"Unexpected motor id for {name}: {entry.get('id')}")
        if entry.get("drive_mode") != 0:
            raise SafetyViolation(f"Unexpected drive_mode for {name}: {entry.get('drive_mode')}")
        if not 0 <= entry.get("range_min", -1) < entry.get("range_max", -1) <= 4095:
            raise SafetyViolation(f"Invalid calibrated range for {name}: {entry}")
    return data


def validate_port(port: Path) -> Path:
    if port != EXPECTED_PORT:
        raise SafetyViolation(f"Refusing non-audited port: {port}")
    if not port.is_symlink():
        raise SafetyViolation(f"Stable by-id symlink is missing: {port}")
    resolved = port.resolve(strict=True)
    if not resolved.name.startswith("ttyACM"):
        raise SafetyViolation(f"Unexpected follower device target: {resolved}")
    return resolved


def _make_write_blocked_bus(port: Path, calibration_data: dict[str, dict[str, int]]) -> Any:
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    class WriteBlockedFeetechBus(FeetechMotorsBus):
        def _blocked(self, *_args: Any, **_kwargs: Any) -> None:
            raise SafetyViolation("A motor write path was reached during a read-only probe")

        write = _blocked
        sync_write = _blocked
        _write = _blocked
        configure_motors = _blocked
        write_calibration = _blocked
        disable_torque = _blocked
        enable_torque = _blocked

    calibration = {
        name: MotorCalibration(**calibration_data[name])
        for name in MOTOR_NAMES
    }
    body_mode = MotorNormMode.RANGE_M100_100
    motors = {
        name: Motor(
            calibration_data[name]["id"],
            "sts3215",
            MotorNormMode.RANGE_0_100 if name == "gripper" else body_mode,
        )
        for name in MOTOR_NAMES
    }
    return WriteBlockedFeetechBus(port=str(port), motors=motors, calibration=calibration)


def run_probe(port: Path, calibration_path: Path) -> dict[str, Any]:
    resolved_port = validate_port(port)
    calibration = load_and_validate_calibration(calibration_path, EXPECTED_CALIBRATION_SHA256)
    bus = _make_write_blocked_bus(port, calibration)

    registers: dict[str, dict[str, int | float]] = {}
    connected = False
    try:
        # Handshake only pings expected IDs and reads their firmware versions.
        bus.connect(handshake=True)
        connected = True
        for register in READ_ONLY_REGISTERS:
            registers[register] = bus.sync_read(register, normalize=False, num_retry=2)
    finally:
        if connected:
            # Passing False is essential: the normal disconnect path writes Torque_Enable.
            bus.disconnect(disable_torque=False)

    positions = registers["Present_Position"]
    range_checks = {
        name: calibration[name]["range_min"]
        <= positions[name]
        <= calibration[name]["range_max"]
        for name in MOTOR_NAMES
    }
    return {
        "mode": "REAL_READ_ONLY",
        "writes_blocked": True,
        "port": str(port),
        "resolved_port": str(resolved_port),
        "calibration": str(calibration_path),
        "calibration_sha256": EXPECTED_CALIBRATION_SHA256,
        "registers": registers,
        "position_within_calibrated_range": range_checks,
        "all_positions_within_calibrated_range": all(range_checks.values()),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=Path, default=EXPECTED_PORT)
    parser.add_argument("--calibration", type=Path, default=EXPECTED_CALIBRATION)
    parser.add_argument(
        "--ack-read-only",
        action="store_true",
        help="Acknowledge that the arm is supported, the area is clear, and only reads are allowed.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.ack_read_only:
        raise SystemExit(
            "REFUSED: support the arm, clear the workspace, turn on 12 V, then pass --ack-read-only"
        )
    result = run_probe(args.port, args.calibration)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

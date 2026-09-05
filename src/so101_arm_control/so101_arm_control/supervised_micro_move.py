#!/usr/bin/env python3

"""One-shot, single-motor supervised write test for the local SO101 follower."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

from so101_arm_control.read_only_probe import (
    EXPECTED_CALIBRATION,
    EXPECTED_CALIBRATION_SHA256,
    EXPECTED_PORT,
    MOTOR_NAMES,
    SafetyViolation,
    load_and_validate_calibration,
    validate_port,
)


TEST_MOTOR = "wrist_roll"
MAX_ABS_DELTA_TICKS = 20
CALIBRATION_TOLERANCE_TICKS = 5
TARGET_LIMIT_MARGIN_TICKS = 100
MAX_CURRENT_RAW = 300
MAX_TEMPERATURE_C = 55
TARGET_REACHED_TOLERANCE_TICKS = 5


def validate_delta(delta_ticks: int) -> None:
    if delta_ticks == 0 or abs(delta_ticks) > MAX_ABS_DELTA_TICKS:
        raise SafetyViolation(
            f"delta_ticks must be nonzero and within +/-{MAX_ABS_DELTA_TICKS}, got {delta_ticks}"
        )


def validate_positions(
    positions: dict[str, int | float], calibration: dict[str, dict[str, int]]
) -> None:
    for name in MOTOR_NAMES:
        raw = int(positions[name])
        minimum = int(calibration[name]["range_min"])
        maximum = int(calibration[name]["range_max"])
        outside = max(minimum - raw, raw - maximum, 0)
        if outside > CALIBRATION_TOLERANCE_TICKS:
            raise SafetyViolation(
                f"{name}={raw} is {outside} ticks outside [{minimum}, {maximum}]"
            )


def compute_target(start: int, delta_ticks: int, calibration: dict[str, dict[str, int]]) -> int:
    validate_delta(delta_ticks)
    minimum = int(calibration[TEST_MOTOR]["range_min"]) + TARGET_LIMIT_MARGIN_TICKS
    maximum = int(calibration[TEST_MOTOR]["range_max"]) - TARGET_LIMIT_MARGIN_TICKS
    target = start + delta_ticks
    if not minimum <= target <= maximum:
        raise SafetyViolation(
            f"wrist_roll target {target} lacks {TARGET_LIMIT_MARGIN_TICKS}-tick limit margin "
            f"[{minimum}, {maximum}]"
        )
    return target


def _make_micro_move_bus(port: Path, calibration_data: dict[str, dict[str, int]]) -> Any:
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    class MicroMoveFeetechBus(FeetechMotorsBus):
        def write(
            self,
            data_name: str,
            motor: str,
            value: Any,
            *,
            normalize: bool = True,
            num_retry: int = 0,
        ) -> None:
            if motor != TEST_MOTOR or data_name not in {"Torque_Enable", "Lock"}:
                raise SafetyViolation(f"Blocked write: {data_name} on {motor}")
            return super().write(
                data_name, motor, value, normalize=normalize, num_retry=num_retry
            )

        def sync_write(
            self,
            data_name: str,
            motors_values: dict[str, Any],
            *,
            normalize: bool = True,
            num_retry: int = 0,
        ) -> None:
            if data_name != "Goal_Position" or set(motors_values) != {TEST_MOTOR}:
                raise SafetyViolation(f"Blocked sync write: {data_name} on {tuple(motors_values)}")
            return super().sync_write(
                data_name, motors_values, normalize=normalize, num_retry=num_retry
            )

        configure_motors = lambda self, *_args, **_kwargs: (_ for _ in ()).throw(
            SafetyViolation("Motor configuration is forbidden in the micro-move test")
        )
        write_calibration = lambda self, *_args, **_kwargs: (_ for _ in ()).throw(
            SafetyViolation("Calibration writes are forbidden in the micro-move test")
        )

    calibration = {
        name: MotorCalibration(**calibration_data[name])
        for name in MOTOR_NAMES
    }
    motors = {
        name: Motor(
            calibration_data[name]["id"],
            "sts3215",
            MotorNormMode.RANGE_0_100 if name == "gripper" else MotorNormMode.RANGE_M100_100,
        )
        for name in MOTOR_NAMES
    }
    return MicroMoveFeetechBus(port=str(port), motors=motors, calibration=calibration)


def run_micro_move(port: Path, calibration_path: Path, delta_ticks: int) -> dict[str, Any]:
    validate_port(port)
    calibration = load_and_validate_calibration(
        calibration_path, EXPECTED_CALIBRATION_SHA256
    )
    validate_delta(delta_ticks)
    bus = _make_micro_move_bus(port, calibration)
    connected = False
    test_motor_torque_enabled = False
    result: dict[str, Any] = {}
    try:
        bus.connect(handshake=True)
        connected = True
        torque_before = bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
        enabled_before = {name: int(value) for name, value in torque_before.items() if int(value)}
        if enabled_before:
            if set(enabled_before) == {TEST_MOTOR}:
                bus.disable_torque(TEST_MOTOR, num_retry=2)
                torque_after_cleanup = int(
                    bus.read("Torque_Enable", TEST_MOTOR, normalize=False, num_retry=2)
                )
                if torque_after_cleanup != 0:
                    raise SafetyViolation(
                        f"Failed to clear pre-existing wrist_roll torque: {torque_after_cleanup}"
                    )
                raise SafetyViolation(
                    "Found pre-existing wrist_roll torque; disabled it and refused this move"
                )
            raise SafetyViolation(f"Expected all torque disabled, found {enabled_before}")

        positions_before = bus.sync_read("Present_Position", normalize=False, num_retry=2)
        validate_positions(positions_before, calibration)
        start = int(positions_before[TEST_MOTOR])
        target = compute_target(start, delta_ticks, calibration)

        voltage = int(bus.read("Present_Voltage", TEST_MOTOR, normalize=False, num_retry=2))
        temperature_before = int(
            bus.read("Present_Temperature", TEST_MOTOR, normalize=False, num_retry=2)
        )
        if not 110 <= voltage <= 135:
            raise SafetyViolation(f"Unexpected wrist_roll voltage: {voltage / 10:.1f} V")
        if temperature_before > MAX_TEMPERATURE_C:
            raise SafetyViolation(f"wrist_roll temperature too high: {temperature_before} C")

        # Feetech firmware may automatically enable torque when Goal_Position is
        # written. Mark cleanup active before the seed write. The seeded value is
        # the measured current position, so automatic enable cannot target a stale
        # position.
        test_motor_torque_enabled = True
        bus.sync_write("Goal_Position", {TEST_MOTOR: start}, normalize=False, num_retry=2)
        seeded_goal = int(bus.read("Goal_Position", TEST_MOTOR, normalize=False, num_retry=2))
        if abs(seeded_goal - start) > 1:
            raise SafetyViolation(f"Goal seed verification failed: start={start}, readback={seeded_goal}")

        torque_enabled = int(bus.read("Torque_Enable", TEST_MOTOR, normalize=False, num_retry=2))
        if torque_enabled == 0:
            bus.enable_torque(TEST_MOTOR, num_retry=2)
            torque_enabled = int(
                bus.read("Torque_Enable", TEST_MOTOR, normalize=False, num_retry=2)
            )
        if torque_enabled != 1:
            raise SafetyViolation(f"wrist_roll torque enable verification failed: {torque_enabled}")

        bus.sync_write("Goal_Position", {TEST_MOTOR: target}, normalize=False, num_retry=2)
        samples = []
        deadline = time.monotonic() + 2.0
        reached = False
        while time.monotonic() < deadline:
            position = int(bus.read("Present_Position", TEST_MOTOR, normalize=False, num_retry=2))
            current = int(bus.read("Present_Current", TEST_MOTOR, normalize=False, num_retry=2))
            temperature = int(
                bus.read("Present_Temperature", TEST_MOTOR, normalize=False, num_retry=2)
            )
            samples.append({"position": position, "current": current, "temperature_c": temperature})
            if abs(current) > MAX_CURRENT_RAW:
                raise SafetyViolation(f"wrist_roll current exceeded limit: {current}")
            if temperature > MAX_TEMPERATURE_C:
                raise SafetyViolation(f"wrist_roll temperature exceeded limit: {temperature} C")
            if abs(position - target) <= TARGET_REACHED_TOLERANCE_TICKS:
                reached = True
                break
            time.sleep(0.05)
        if not reached:
            raise SafetyViolation(f"wrist_roll did not reach target {target}: last={samples[-1]}")

        result = {
            "mode": "REAL_SUPERVISED_MICRO_MOVE",
            "motor": TEST_MOTOR,
            "start_ticks": start,
            "target_ticks": target,
            "delta_ticks": delta_ticks,
            "reached": True,
            "samples": samples,
            "voltage_deci_v": voltage,
            "temperature_before_c": temperature_before,
            "other_motors_torque_enabled": False,
        }
    finally:
        if bus.is_connected and test_motor_torque_enabled:
            bus.disable_torque(TEST_MOTOR, num_retry=2)
            test_motor_torque_enabled = False
        if bus.is_connected:
            bus.disconnect(disable_torque=False)

    result["wrist_roll_torque_disabled_after_test"] = True
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=Path, default=EXPECTED_PORT)
    parser.add_argument("--calibration", type=Path, default=EXPECTED_CALIBRATION)
    parser.add_argument("--delta-ticks", type=int, default=20)
    parser.add_argument("--ack-supervised-move", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.ack_supervised_move:
        raise SystemExit("REFUSED: --ack-supervised-move is required")
    result = run_micro_move(args.port, args.calibration, args.delta_ticks)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

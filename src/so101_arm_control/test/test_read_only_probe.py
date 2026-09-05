import hashlib
import json
from pathlib import Path

import pytest

from so101_arm_control.read_only_probe import (
    MOTOR_NAMES,
    SafetyViolation,
    load_and_validate_calibration,
    validate_port,
)


def _valid_calibration():
    return {
        name: {
            "id": index,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": 100,
            "range_max": 4000,
        }
        for index, name in enumerate(MOTOR_NAMES, start=1)
    }


def _write_calibration(tmp_path: Path, data: dict):
    path = tmp_path / "calibration.json"
    payload = json.dumps(data).encode()
    path.write_bytes(payload)
    return path, hashlib.sha256(payload).hexdigest()


def test_calibration_validation_accepts_expected_schema(tmp_path):
    data = _valid_calibration()
    path, digest = _write_calibration(tmp_path, data)
    assert load_and_validate_calibration(path, digest) == data


def test_calibration_validation_rejects_hash_mismatch(tmp_path):
    path, _ = _write_calibration(tmp_path, _valid_calibration())
    with pytest.raises(SafetyViolation, match="SHA-256 mismatch"):
        load_and_validate_calibration(path, "0" * 64)


def test_calibration_validation_rejects_wrong_motor_id(tmp_path):
    data = _valid_calibration()
    data["elbow_flex"]["id"] = 6
    path, digest = _write_calibration(tmp_path, data)
    with pytest.raises(SafetyViolation, match="Unexpected motor id"):
        load_and_validate_calibration(path, digest)


def test_port_validation_refuses_unrelated_device():
    with pytest.raises(SafetyViolation, match="non-audited port"):
        validate_port(Path("/dev/ttyACM0"))

from importlib.util import module_from_spec, spec_from_file_location
from functools import lru_cache
from pathlib import Path

import pytest


DRIVE_MOTOR_PATH = (
    Path(__file__).resolve().parents[2]
    / "om_modbus_master_V201"
    / "om_modbus_master"
    / "sample"
    / "BLV_R"
    / "drive_motor.py"
)


@lru_cache(maxsize=1)
def _load_drive_motor_module():
    spec = spec_from_file_location("drive_motor_under_test", DRIVE_MOTOR_PATH)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_combined_drive_diagnostics_preserve_axis_grouping_and_scales():
    module = _load_drive_motor_module()
    direct_io, electrical = module.decode_drive_diagnostics(
        [
            196616, 101, 242, 245,
            196608, 202, 243, 246,
            196616, 303, 244, 247,
        ]
    )

    assert [(item["slave_id"], item["wheel_position"]) for item in direct_io] == [
        (1, "back"),
        (2, "front_left"),
        (3, "front_right"),
    ]
    assert [item["din3"] for item in direct_io] == [True, False, True]
    assert electrical[0]["main_power_supply_current_a"] == 0.101
    assert electrical[1]["inverter_voltage_v"] == 24.3
    assert electrical[2]["main_power_supply_voltage_v"] == pytest.approx(24.7)


def test_combined_drive_diagnostics_reject_short_response():
    module = _load_drive_motor_module()

    try:
        module.decode_drive_diagnostics([0] * 11)
    except ValueError as exc:
        assert "Expected 12" in str(exc)
    else:
        raise AssertionError("short diagnostic response must be rejected")


def test_drive_alarm_status_preserves_physical_axis_and_codes():
    module = _load_drive_motor_module()

    value = module.decode_drive_alarm_status([0x30, 0x41], 1)

    assert value == {
        "slave_id": 1,
        "wheel_position": "back",
        "present_alarm": 0x30,
        "alarm_history_1": 0x41,
    }


def test_drive_alarm_status_rejects_unknown_axis_and_short_response():
    module = _load_drive_motor_module()

    with pytest.raises(ValueError, match="Unexpected BLVD slave ID"):
        module.decode_drive_alarm_status([0, 0], 4)
    with pytest.raises(ValueError, match="Expected 2 alarm values"):
        module.decode_drive_alarm_status([0], 1)


def test_completion_callback_deadline_matches_serial_timeout_budget():
    module = _load_drive_motor_module()

    assert module.COMPLETION_CALLBACK_DEADLINE_SEC == pytest.approx(0.5)

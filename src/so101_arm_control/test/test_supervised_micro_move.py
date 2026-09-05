import pytest

from so101_arm_control.read_only_probe import MOTOR_NAMES, SafetyViolation
from so101_arm_control.supervised_micro_move import compute_target, validate_delta, validate_positions


def _calibration():
    return {
        name: {
            "id": index,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": 0,
            "range_max": 4095,
        }
        for index, name in enumerate(MOTOR_NAMES, start=1)
    }


@pytest.mark.parametrize("delta", [-20, -1, 1, 20])
def test_delta_accepts_only_small_nonzero_values(delta):
    validate_delta(delta)


@pytest.mark.parametrize("delta", [-21, 0, 21])
def test_delta_rejects_unsafe_values(delta):
    with pytest.raises(SafetyViolation):
        validate_delta(delta)


def test_target_requires_limit_margin():
    calibration = _calibration()
    assert compute_target(2000, 20, calibration) == 2020
    with pytest.raises(SafetyViolation, match="lacks 100-tick limit margin"):
        compute_target(4080, 20, calibration)


def test_position_validation_allows_only_five_ticks_outside():
    calibration = _calibration()
    positions = {name: 2000 for name in MOTOR_NAMES}
    positions["shoulder_lift"] = -5
    validate_positions(positions, calibration)
    positions["shoulder_lift"] = -6
    with pytest.raises(SafetyViolation, match="6 ticks outside"):
        validate_positions(positions, calibration)

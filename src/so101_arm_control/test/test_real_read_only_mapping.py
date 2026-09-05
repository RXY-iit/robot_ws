import pytest

from so101_arm_control.read_only_probe import MOTOR_NAMES, SafetyViolation
from so101_arm_control.real_read_only_joint_state_publisher import JOINT_SPECS, raw_positions_to_urdf


def _calibration():
    return {
        name: {
            "id": index,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": 100,
            "range_max": 1100,
        }
        for index, name in enumerate(MOTOR_NAMES, start=1)
    }


def test_maps_calibrated_endpoints_to_urdf_limits():
    calibration = _calibration()
    raw = {name: 100 for name in MOTOR_NAMES}
    names, positions, clipped = raw_positions_to_urdf(raw, calibration, 5)
    assert names == [JOINT_SPECS[name][0] for name in MOTOR_NAMES]
    assert positions == pytest.approx([JOINT_SPECS[name][1] for name in MOTOR_NAMES])
    assert clipped == {}

    raw = {name: 1100 for name in MOTOR_NAMES}
    _, positions, clipped = raw_positions_to_urdf(raw, calibration, 5)
    assert positions == pytest.approx([JOINT_SPECS[name][2] for name in MOTOR_NAMES])
    assert clipped == {}


def test_small_outside_error_is_clipped_for_display_only():
    calibration = _calibration()
    raw = {name: 600 for name in MOTOR_NAMES}
    raw["shoulder_lift"] = 97
    _, positions, clipped = raw_positions_to_urdf(raw, calibration, 5)
    assert clipped == {"shoulder_lift": -3}
    lift_index = list(MOTOR_NAMES).index("shoulder_lift")
    assert positions[lift_index] == JOINT_SPECS["shoulder_lift"][1]


def test_error_beyond_five_ticks_is_rejected():
    calibration = _calibration()
    raw = {name: 600 for name in MOTOR_NAMES}
    raw["elbow_flex"] = 1106
    with pytest.raises(SafetyViolation, match="6 ticks outside"):
        raw_positions_to_urdf(raw, calibration, 5)


def test_recovery_window_keeps_bounded_feedback_alive_after_torque_off_fall():
    calibration = _calibration()
    raw = {name: 600 for name in MOTOR_NAMES}
    raw["elbow_flex"] = 1124
    _, positions, clipped = raw_positions_to_urdf(raw, calibration, 100)
    assert clipped == {"elbow_flex": 24}
    elbow_index = list(MOTOR_NAMES).index("elbow_flex")
    assert positions[elbow_index] == JOINT_SPECS["elbow_flex"][2]

import pytest

from so101_arm_control.arm_visual_joint_state_calibrator import (
    REFERENCE,
    SCALE,
    VISUAL_OFFSET,
    corrected_position,
)


def test_reference_positions_are_preserved():
    for name, reference in REFERENCE.items():
        assert corrected_position(name, reference) == pytest.approx(
            reference + VISUAL_OFFSET.get(name, 0.0)
        )


def test_wrist_roll_uses_measured_visual_scale():
    name = "wrist_roll_joint"
    reference = REFERENCE[name]
    assert corrected_position(name, reference + 0.5) == pytest.approx(
        reference + 0.5 * SCALE[name] + VISUAL_OFFSET[name]
    )


def test_operator_wrist_zero_alignment():
    # A real -0.031 rad must render as the pose that the old visualization
    # showed at -0.277 rad.
    name = "wrist_roll_joint"
    old_visual_at_minus_0277 = (
        REFERENCE[name] + SCALE[name] * (-0.277 - REFERENCE[name])
    )
    assert corrected_position(name, -0.031) == pytest.approx(
        old_visual_at_minus_0277, abs=1.0e-7
    )


def test_gripper_is_not_scaled():
    assert corrected_position("gripper_joint", 0.75) == 0.75

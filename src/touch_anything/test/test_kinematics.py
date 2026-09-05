import importlib.util
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "touch_kinematics.py"
SPEC = importlib.util.spec_from_file_location("touch_kinematics", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_fk_is_finite_homogeneous_transform():
    pose = MODULE.tip_transform(np.zeros(5))
    assert pose.shape == (4, 4)
    assert np.isfinite(pose).all()
    np.testing.assert_allclose(pose[3], [0.0, 0.0, 0.0, 1.0])


def test_ik_recovers_reachable_position_and_axis():
    expected = np.array([0.25, -0.65, 0.85, -0.45, -0.35])
    pose = MODULE.tip_transform(expected)
    result = MODULE.solve_pre_touch(
        expected + np.array([0.03, -0.02, 0.02, -0.02, 0.03]),
        pose[:3, 3], pose[:3, 2],
    )
    assert result["success"]
    assert result["position_error_m"] <= 0.012
    assert result["axis_error_deg"] <= 30.0
    assert result["collision_checked"] is False


def test_horizontal_axis_is_soft_when_position_is_reachable():
    expected = np.array([0.1, -0.7, 0.9, -0.3, 0.2])
    pose = MODULE.tip_transform(expected)
    result = MODULE.solve_pre_touch(expected, pose[:3, 3], -pose[:3, 2])
    assert result["success"]
    assert result["axis_error_deg"] > 90.0
    assert result["orientation_constrained"] is False


def test_ik_respects_gateway_effective_limits():
    expected = np.array([0.1, -0.7, 0.9, -0.3, 0.2])
    pose = MODULE.tip_transform(expected)
    lower = MODULE.LOWER + 0.03
    upper = MODULE.UPPER - 0.03
    result = MODULE.solve_pre_touch(
        expected, pose[:3, 3], pose[:3, 2], lower, upper
    )
    positions = np.array([result["positions"][name] for name in MODULE.JOINT_NAMES])
    assert np.all(positions >= lower)
    assert np.all(positions <= upper)
    assert result["joint_limits_source"] == "ARM_GATEWAY_EFFECTIVE_LIMITS"


def test_near_alignment_locks_both_wrist_joints():
    current = np.array([0.10, -0.70, 0.90, -0.30, 0.20])
    target_joint_pose = current.copy()
    target_joint_pose[:3] += np.array([0.03, -0.02, 0.02])
    target = MODULE.tip_transform(target_joint_pose)[:3, 3]
    result = MODULE.solve_near_tip_alignment(current, target)
    positions = np.array([result["positions"][name] for name in MODULE.JOINT_NAMES])
    assert result["success"]
    assert result["position_error_m"] <= 0.008
    np.testing.assert_allclose(positions[3:5], current[3:5], atol=1.0e-12)
    assert result["prepare_strategy"] == "NEAR_LOCAL_SHOULDER_PAN_LIFT_ELBOW_ONLY"


def test_near_forward_press_locks_pan_and_roll():
    current = np.array([0.10, -0.70, 0.90, -0.30, 0.20])
    target = MODULE.tip_transform(current)[:3, 3].copy()
    target[0] += 0.010
    result = MODULE.solve_near_forward_press(current, target)
    positions = np.array([result["positions"][name] for name in MODULE.JOINT_NAMES])
    assert result["success"]
    assert result["position_error_m"] <= 0.008
    np.testing.assert_allclose(positions[[0, 4]], current[[0, 4]], atol=1.0e-12)
    assert result["primary_joints"] == [
        "shoulder_lift_joint", "elbow_flex_joint"
    ]
    assert result["compensation_joint"] == "wrist_flex_joint"


def test_prepare_strategy_uses_operator_reference_bank():
    expected = np.array(MODULE.PREPARE_REFERENCE_SEEDS[2][1])
    pose = MODULE.tip_transform(expected)
    result = MODULE.solve_pre_touch(expected, pose[:3, 3], pose[:3, 2])
    assert result["success"]
    assert result["prepare_reference_count"] == 1
    assert result["prepare_strategy"] == "GEOMETRY_CLASSIFIED_OPERATOR_REFERENCE_BOUNDED_IK"
    assert result["prepare_camera_pan_deg"] == 271.0
    assert result["prepare_camera_tilt_deg"] == 32.8


def test_front_low_far_target_cannot_select_left_camera_reference():
    reference = next(
        item for item in MODULE.PREPARE_REFERENCE_SEEDS
        if item[0] == "front_low_far"
    )
    expected = np.array(reference[1])
    pose = MODULE.tip_transform(expected)
    # A current pose near LEFT must not override the target's FRONT sector.
    current = np.array(next(
        item[1] for item in MODULE.PREPARE_REFERENCE_SEEDS
        if item[0] == "left_low"
    ))
    result = MODULE.solve_pre_touch(
        current, pose[:3, 3], pose[:3, 2],
        reference_sector="front", reference_stage="far",
    )
    assert result["success"]
    assert result["prepare_reference_id"] == "front_low_far"
    assert result["prepare_camera_pan_deg"] == 270.97
    assert result["prepare_camera_tilt_deg"] == 45.18


def test_locked_reference_is_retained_for_approach_steps():
    reference = next(
        item for item in MODULE.PREPARE_REFERENCE_SEEDS
        if item[0] == "front_low_far"
    )
    expected = np.array(reference[1])
    pose = MODULE.tip_transform(expected)
    result = MODULE.solve_pre_touch(
        expected, pose[:3, 3], pose[:3, 2],
        locked_reference_id="front_low_far",
    )
    assert result["success"]
    assert result["prepare_reference_id"] == "front_low_far"
    assert result["prepare_reference_locked"] is True


def test_touch_tip_is_gripper_frame_origin():
    assert MODULE.TOUCH_TIP_OFFSET == (0.0, 0.0, 0.0)


def test_touch_ik_keeps_wrist_roll_near_neutral():
    expected = np.array(MODULE.PREPARE_REFERENCE_SEEDS[2][1])
    pose = MODULE.tip_transform(expected)
    result = MODULE.solve_pre_touch(expected, pose[:3, 3], pose[:3, 2])
    assert abs(result["positions"]["wrist_roll_joint"]) <= 0.03 + 1.0e-9
    assert result["wrist_roll_target_rad"] == 0.0

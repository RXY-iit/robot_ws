import json
from pathlib import Path

import pytest
import yaml

from so101_arm_control.leader_state_publisher import (
    load_leader_calibration,
    unwrap_cyclic_position,
    valid_normalized_positions,
)
from so101_arm_control.read_only_probe import MOTOR_NAMES, SafetyViolation
from so101_arm_control.real_trajectory_server import (
    leader_absolute_targets_raw,
    leader_relative_targets_raw,
)


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


def test_relative_mapping_has_no_mode_entry_jump_and_scales_displacement():
    entry_leader = {name: 0.0 for name in MOTOR_NAMES}
    entry_leader["gripper"] = 50.0
    entry_follower = {name: 600 for name in MOTOR_NAMES}
    safe = {name: (120, 1080) for name in MOTOR_NAMES}

    targets, clamped = leader_relative_targets_raw(
        entry_leader_norm=entry_leader,
        current_leader_norm=dict(entry_leader),
        entry_follower_raw=entry_follower,
        calibration=_calibration(),
        safe_limits=safe,
        deadband_norm=0.2,
    )
    assert targets == entry_follower
    assert clamped == {}

    moved = dict(entry_leader)
    moved["shoulder_pan"] = 20.0
    moved["gripper"] = 60.0
    targets, _ = leader_relative_targets_raw(
        entry_leader_norm=entry_leader,
        current_leader_norm=moved,
        entry_follower_raw=entry_follower,
        calibration=_calibration(),
        safe_limits=safe,
        deadband_norm=0.2,
    )
    assert targets["shoulder_pan"] == 700
    assert targets["gripper"] == 700


def test_relative_mapping_clamps_to_follower_safe_limits():
    entry = {name: 0.0 for name in MOTOR_NAMES}
    entry["gripper"] = 0.0
    moved = dict(entry)
    moved["shoulder_pan"] = 100.0
    targets, clamped = leader_relative_targets_raw(
        entry_leader_norm=entry,
        current_leader_norm=moved,
        entry_follower_raw={name: 900 for name in MOTOR_NAMES},
        calibration=_calibration(),
        safe_limits={name: (120, 1080) for name in MOTOR_NAMES},
        deadband_norm=0.2,
    )
    assert targets["shoulder_pan"] == 1080
    assert clamped["shoulder_pan"] == {"requested": 1400, "bounded": 1080}


def test_absolute_mapping_converts_leader_pose_and_clamps_safe_limits():
    leader = {name: 0.0 for name in MOTOR_NAMES}
    leader["gripper"] = 50.0
    targets, clamped = leader_absolute_targets_raw(
        leader_norm=leader,
        calibration=_calibration(),
        safe_limits={name: (120, 1080) for name in MOTOR_NAMES},
    )
    assert targets == {name: 600 for name in MOTOR_NAMES}
    assert clamped == {}

    leader["shoulder_pan"] = 100.0
    targets, clamped = leader_absolute_targets_raw(
        leader_norm=leader,
        calibration=_calibration(),
        safe_limits={name: (120, 1080) for name in MOTOR_NAMES},
    )
    assert targets["shoulder_pan"] == 1080
    assert clamped["shoulder_pan"] == {"requested": 1100, "bounded": 1080}


def test_absolute_mapping_wraps_unwrapped_wrist_roll():
    leader = {name: 0.0 for name in MOTOR_NAMES}
    leader["gripper"] = 50.0
    leader["wrist_roll"] = 101.0
    targets, _ = leader_absolute_targets_raw(
        leader_norm=leader,
        calibration=_calibration(),
        safe_limits={name: (100, 1100) for name in MOTOR_NAMES},
    )
    assert targets["wrist_roll"] == 105


def test_teleoperate_has_separate_faster_alignment_and_follow_profiles():
    config_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "arm_real_dashboard_test.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    code_limits = config["safety"]["max_joint_velocity_rad_s"]
    alignment_limits = config["teleoperation"][
        "alignment_max_joint_velocity_rad_s"
    ]
    follow_limits = config["teleoperation"]["follow_max_joint_velocity_rad_s"]
    expected = set(MOTOR_NAMES)
    assert set(code_limits) == expected
    assert set(alignment_limits) == expected
    assert set(follow_limits) == expected
    assert all(alignment_limits[name] > code_limits[name] for name in expected)
    assert all(follow_limits[name] >= alignment_limits[name] for name in expected)


def test_leader_calibration_hash_and_motor_identity_are_strict(tmp_path):
    path = tmp_path / "leader.json"
    path.write_text(json.dumps(_calibration()), encoding="utf-8")
    import hashlib

    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    loaded = load_leader_calibration(path, digest)
    assert tuple(loaded) == MOTOR_NAMES
    with pytest.raises(SafetyViolation, match="SHA-256 mismatch"):
        load_leader_calibration(path, "0" * 64)


def test_leader_normalized_feedback_requires_all_six_finite_values():
    values = {name: 0.0 for name in MOTOR_NAMES}
    values["gripper"] = 50.0
    assert valid_normalized_positions(values)["gripper"] == 50.0
    values.pop("wrist_roll")
    with pytest.raises(SafetyViolation, match="exactly six"):
        valid_normalized_positions(values)


def test_wrist_roll_unwraps_across_both_normalized_boundaries():
    assert unwrap_cyclic_position(-99.0, 99.0, 99.0) == 101.0
    assert unwrap_cyclic_position(99.0, -99.0, -99.0) == -101.0
    assert unwrap_cyclic_position(20.0, 10.0, 210.0) == 220.0

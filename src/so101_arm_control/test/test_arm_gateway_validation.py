import math
from pathlib import Path

import yaml

from so101_arm_control.arm_gateway import (
    changed_command_limit_errors,
    minimum_target_duration,
    validate_target,
)
from so101_arm_control.real_dashboard_sequence_test import TEST_POSES


JOINTS = [
    {"name": "a", "lower": -1.0, "upper": 1.0},
    {"name": "b", "lower": 0.0, "upper": 2.0},
]

REMOUNT_PRESETS = {
    "home": [-0.036, -1.583, 1.367, 0.783, -0.124, 0.222],
    "see_ground": [-0.181, -0.202, 0.893, 0.241, -1.109, 0.223],
    "see_front": [-0.014, -0.704, 1.285, -0.665, -1.109, 0.222],
    "see_right": [0.933, -0.706, 1.285, -0.665, -1.109, 0.222],
    "see_left": [-1.050, -0.706, 1.287, -0.665, -1.109, 0.222],
    "see_upside": [0.046, -0.845, 0.770, -0.432, -1.108, 0.225],
}


def test_accepts_complete_bounded_target():
    normalized, errors = validate_target(JOINTS, {"a": 0.25, "b": 1})
    assert errors == []
    assert normalized == {"a": 0.25, "b": 1.0}


def test_rejects_missing_and_unknown_joints():
    _, errors = validate_target(JOINTS, {"a": 0.0, "c": 0.0})
    assert any("missing joints: b" in error for error in errors)
    assert any("unknown joints: c" in error for error in errors)


def test_rejects_limit_and_non_finite_values():
    _, errors = validate_target(JOINTS, {"a": 1.1, "b": math.nan})
    assert any("outside" in error for error in errors)
    assert any("finite" in error for error in errors)


def test_gateway_computes_multi_joint_guarded_duration():
    specs = [
        {"name": "a", "motor_name": "ma"},
        {"name": "b", "motor_name": "mb"},
    ]
    duration = minimum_target_duration(
        specs,
        {"a": 0.0, "b": 0.0},
        {"a": 0.8, "b": 1.0},
        {"ma": 0.4, "mb": 0.25},
    )
    assert duration == 4.0


def test_unchanged_joint_may_rest_outside_command_inset_but_cannot_drive_outward():
    specs = [
        {"name": "a", "lower": -0.9, "upper": 0.9},
        {"name": "b", "lower": -0.9, "upper": 0.9},
    ]
    actual = {"a": 1.0, "b": 0.0}
    assert changed_command_limit_errors(specs, actual, {"a": 1.0, "b": 0.5}) == []
    errors = changed_command_limit_errors(specs, actual, {"a": 0.95, "b": 0.5})
    assert any("outside safe" in error for error in errors)
    assert changed_command_limit_errors(specs, actual, {"a": 0.8, "b": 0.5}) == []


def test_real_config_contains_only_operator_captured_remount_presets():
    config_path = Path(__file__).parents[1] / "config" / "arm_real_dashboard_test.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    joint_specs = config["joints"]
    joint_names = [item["name"] for item in joint_specs]
    presets = {item["id"]: item for item in config["presets"]}

    assert set(presets) == set(REMOUNT_PRESETS)
    for preset_id, expected_values in REMOUNT_PRESETS.items():
        preset = presets[preset_id]
        assert preset["review_state"] == "OPERATOR_CAPTURED"
        assert preset["allowed_modes"] == ["REAL_DASHBOARD_TEST"]
        assert list(preset["positions"]) == joint_names
        assert list(preset["positions"].values()) == expected_values
        _, errors = validate_target(joint_specs, preset["positions"])
        assert errors == []


def test_sequence_test_has_no_legacy_pose_targets():
    assert {
        preset_id: list(positions.values())
        for preset_id, positions in TEST_POSES.items()
    } == REMOUNT_PRESETS

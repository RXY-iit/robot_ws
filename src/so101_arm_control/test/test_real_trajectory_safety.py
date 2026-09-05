from types import SimpleNamespace
import inspect

import pytest

from so101_arm_control.read_only_probe import SafetyViolation
from so101_arm_control.real_trajectory_server import (
    CurrentProtectionMonitor,
    RealTrajectoryServer,
    classify_hold_inset_recovery,
    changed_joint_indices,
    goal_tolerance_violations,
    near_goal_settle_extension_allowed,
    hold_sample_update,
    minimum_motion_duration,
    position_recovery_details,
    trajectory_errors,
    urdf_radians_to_raw,
)


JOINTS = [
    {"name": "a", "motor_name": "ma", "lower": -1.0, "upper": 1.0},
    {"name": "b", "motor_name": "mb", "lower": -2.0, "upper": 2.0},
]


def _trajectory(positions=(0.0, 0.0), duration=2.0):
    sec = int(duration)
    nanosec = int((duration - sec) * 1_000_000_000)
    point = SimpleNamespace(
        positions=list(positions),
        time_from_start=SimpleNamespace(sec=sec, nanosec=nanosec),
    )
    return SimpleNamespace(joint_names=["a", "b"], points=[point])


def test_real_trajectory_accepts_one_bounded_point():
    assert trajectory_errors(JOINTS, _trajectory(), 5.0) == []


def test_real_trajectory_rejects_multiple_points_and_duration():
    trajectory = _trajectory(duration=6.0)
    assert "duration" in trajectory_errors(JOINTS, trajectory, 5.0)[0]
    trajectory.points.append(trajectory.points[0])
    assert trajectory_errors(JOINTS, trajectory, 5.0) == [
        "real baseline accepts exactly one trajectory point"
    ]


def test_one_or_more_joints_may_change_without_delta_cap():
    assert changed_joint_indices([0.0, 0.0], [0.8, 0.0]) == [0]
    assert changed_joint_indices([0.0, 0.0], [0.8, -1.5]) == [0, 1]
    with pytest.raises(SafetyViolation, match="At least one"):
        changed_joint_indices([0.0, 0.0], [0.001, 0.0])


def test_hold_inset_allows_only_inward_recovery_targets():
    recovering, required = classify_hold_inset_recovery(
        {"shoulder": 891, "elbow": 2996, "wrist": 1000},
        {"shoulder": 950, "elbow": 2996, "wrist": 1100},
        {"shoulder": (906, 3229), "elbow": (870, 2979), "wrist": (800, 1200)},
    )
    assert set(recovering) == {"shoulder"}
    assert set(required) == {"elbow"}
    assert required["elbow"]["safe"] == [870, 2979]


def test_position_recovery_reports_calibration_overrun_without_expanding_commands():
    details = position_recovery_details(
        {"elbow_flex": 3023, "wrist": 1000},
        {
            "elbow_flex": {"range_min": 850, "range_max": 2999},
            "wrist": {"range_min": 800, "range_max": 1200},
        },
        {"elbow_flex": (870, 2979), "wrist": (820, 1180)},
    )
    assert details == {
        "elbow_flex": {
            "raw": 3023,
            "safe": [870, 2979],
            "calibration": [850, 2999],
            "calibration_outside_ticks": 24,
            "required_direction": "decrease",
        }
    }


def test_position_recovery_tolerance_does_not_expand_command_limits():
    calibration = {"shoulder_lift": {"range_min": 796, "range_max": 3133}}
    safe = {"shoulder_lift": (816, 3113)}
    assert position_recovery_details(
        {"shoulder_lift": 3118}, calibration, safe, tolerance_ticks=15
    ) == {}
    details = position_recovery_details(
        {"shoulder_lift": 3129}, calibration, safe, tolerance_ticks=15
    )
    assert details["shoulder_lift"]["safe"] == [816, 3113]
    assert details["shoulder_lift"]["required_direction"] == "decrease"


def test_trip_recorders_accept_the_data_their_callers_supply():
    protection = inspect.signature(RealTrajectoryServer._record_protection_trip).parameters
    hold = inspect.signature(RealTrajectoryServer._record_hold_trip).parameters
    assert "targets" not in protection
    assert "targets" in hold


def test_minimum_duration_uses_slowest_required_joint():
    assert minimum_motion_duration(
        [0.0, 0.0], [0.8, 1.0], ["ma", "mb"], {"ma": 0.4, "mb": 0.25}
    ) == 4.0


def test_goal_acceptance_uses_per_motor_tolerance():
    tolerances = {"elbow_flex": 45, "wrist_flex": 20, "shoulder_pan": 12}
    assert goal_tolerance_violations(
        {"elbow_flex": -35, "wrist_flex": -10, "shoulder_pan": 1}, tolerances
    ) == {}
    assert goal_tolerance_violations(
        {"elbow_flex": -46, "wrist_flex": 21, "shoulder_pan": -12}, tolerances
    ) == {
        "elbow_flex": {"error": -46, "tolerance": 45},
        "wrist_flex": {"error": 21, "tolerance": 20},
    }


def test_near_goal_extension_requires_below_hold_boundary_and_progress():
    violations = {"elbow_flex": {"error": -47, "tolerance": 45}}
    assert near_goal_settle_extension_allowed(
        violations, {"elbow_flex": -61}, 50
    )
    assert not near_goal_settle_extension_allowed(
        violations, {"elbow_flex": -47}, 50
    )
    assert not near_goal_settle_extension_allowed(
        {"elbow_flex": {"error": -50, "tolerance": 45}},
        {"elbow_flex": -61},
        50,
    )


def test_urdf_inverse_mapping_and_margin():
    calibration = {"ma": {"range_min": 0, "range_max": 1000}}
    assert urdf_radians_to_raw("ma", 0.0, JOINTS[0], calibration, 20) == 500
    with pytest.raises(SafetyViolation, match="lacks 20-tick"):
        urdf_radians_to_raw("ma", -1.0, JOINTS[0], calibration, 20)
    with pytest.raises(SafetyViolation, match="lacks 20-tick"):
        urdf_radians_to_raw("ma", 0.8, JOINTS[0], calibration, 20, None, 850)


def test_current_monitor_trips_hard_current_immediately():
    monitor = CurrentProtectionMonitor(
        hard_current_raw=300,
        resistance_current_raw={"ma": 80},
        resistance_samples=3,
        following_error_ticks=12,
        max_progress_ticks=1,
        grace_sec=0.25,
    )
    assert monitor.update(
        "ma",
        current_raw=300,
        present_raw=100,
        previous_raw=100,
        commanded_raw=120,
        elapsed_sec=0.01,
    ) == "HARD_OVERCURRENT"


def test_current_monitor_debounces_resistance_and_requires_stall():
    monitor = CurrentProtectionMonitor(
        hard_current_raw=300,
        resistance_current_raw={"ma": 80},
        resistance_samples=3,
        following_error_ticks=12,
        max_progress_ticks=1,
        grace_sec=0.25,
    )
    for _ in range(2):
        assert monitor.update(
            "ma",
            current_raw=90,
            present_raw=100,
            previous_raw=100,
            commanded_raw=130,
            elapsed_sec=0.5,
        ) is None
    assert monitor.update(
        "ma",
        current_raw=90,
        present_raw=100,
        previous_raw=100,
        commanded_raw=130,
        elapsed_sec=0.5,
    ) == "RESISTANCE_STALL"
    assert monitor.update(
        "ma",
        current_raw=90,
        present_raw=110,
        previous_raw=100,
        commanded_raw=140,
        elapsed_sec=0.5,
    ) is None


def test_hold_sample_protects_torque_temperature_current_and_position():
    base = {
        "torque_enabled": 1,
        "temperature_c": 35,
        "max_temperature_c": 55,
        "current_raw": 10,
        "present_raw": 1000,
        "target_raw": 1000,
        "hard_current_raw": 300,
        "sustained_current_raw": 80,
        "current_count": 0,
        "current_samples_required": 3,
        "position_error_ticks": 50,
        "position_count": 0,
        "position_samples_required": 2,
    }
    assert hold_sample_update(**base) == (None, 0, 0)
    assert hold_sample_update(**{**base, "torque_enabled": 0})[0] == "HOLD_TORQUE_LOST"
    assert hold_sample_update(**{**base, "temperature_c": 56})[0] == "HOLD_OVERTEMPERATURE"
    assert hold_sample_update(**{**base, "current_raw": 300})[0] == "HOLD_HARD_OVERCURRENT"
    assert hold_sample_update(
        **{**base, "current_raw": 90, "current_count": 2}
    )[0] == "HOLD_SUSTAINED_CURRENT"
    assert hold_sample_update(
        **{**base, "present_raw": 1060, "position_count": 1}
    )[0] == "HOLD_POSITION_DISTURBANCE"

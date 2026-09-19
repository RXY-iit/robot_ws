import pytest

from so101_arm_control.visual_servo import (
    PROTOCOL,
    select_action_horizon_target,
    validate_command,
    validate_sequence,
    velocity_limited_positions,
    watchdog_expired,
)


SPECS = [
    {"name": "joint_a", "motor_name": "motor_a", "lower": -1.0, "upper": 1.0},
    {"name": "joint_b", "motor_name": "motor_b", "lower": 0.0, "upper": 2.0},
]


def command(op="START", sequence=0, positions=None):
    return {
        "protocol": PROTOCOL,
        "op": op,
        "session_id": "session-1234",
        "sequence": sequence,
        "positions": {"joint_a": 0.2, "joint_b": 0.5} if positions is None else positions,
        "issued_ros_sec": 123.0,
        "tracker_stamp": 122.9,
    }


def test_command_requires_exact_finite_bounded_joint_target():
    normalized, errors = validate_command(command(), SPECS)
    assert errors == []
    assert normalized["positions"] == {"joint_a": 0.2, "joint_b": 0.5}

    _, missing = validate_command(command(positions={"joint_a": 0.2}), SPECS)
    _, outside = validate_command(
        command(positions={"joint_a": 1.1, "joint_b": 0.5}), SPECS
    )
    assert "exactly configured joints" in missing[0]
    assert "outside" in outside[0]


def test_command_accepts_guarded_ten_step_action_horizon():
    payload = command()
    payload.update({
        "action_horizon": [
            {"joint_a": 0.02 * (index + 1), "joint_b": 1.0 - 0.05 * (index + 1)}
            for index in range(10)
        ],
        "positions": {"joint_a": 0.2, "joint_b": 0.5},
        "horizon_step_period_sec": 0.05,
        "horizon_plan_id": "plan-12345678",
        "observation_age_sec": 0.08,
        "prediction_lead_sec": 0.13,
    })
    normalized, errors = validate_command(payload, SPECS)
    assert errors == []
    assert len(normalized["action_horizon"]) == 10
    assert normalized["action_horizon"][-1] == normalized["positions"]


def test_action_horizon_selects_by_elapsed_command_time_and_holds_terminal():
    horizon = [{"joint_a": float(index)} for index in range(10)]
    target, index = select_action_horizon_target(
        horizon, elapsed_since_issue_sec=0.126, step_period_sec=0.05
    )
    assert index == 2
    assert target == {"joint_a": 2.0}
    terminal, index = select_action_horizon_target(
        horizon, elapsed_since_issue_sec=2.0, step_period_sec=0.05
    )
    assert index == 9
    assert terminal == {"joint_a": 9.0}


def test_action_horizon_rejects_incomplete_point_and_terminal_mismatch():
    incomplete = command()
    incomplete.update({
        "action_horizon": [
            {"joint_a": 0.1, "joint_b": 0.8},
            {"joint_a": 0.2},
        ],
        "horizon_step_period_sec": 0.05,
        "horizon_plan_id": "plan-incomplete",
    })
    _, errors = validate_command(incomplete, SPECS)
    assert any("action_horizon[1]" in error for error in errors)

    mismatch = command()
    mismatch.update({
        "action_horizon": [
            {"joint_a": 0.1, "joint_b": 0.8},
            {"joint_a": 0.3, "joint_b": 0.5},
        ],
        "horizon_step_period_sec": 0.05,
        "horizon_plan_id": "plan-mismatch-1",
    })
    _, errors = validate_command(mismatch, SPECS)
    assert "positions must equal the terminal action_horizon target" in errors


def test_session_sequence_rejects_overlap_mismatch_and_replay():
    assert validate_sequence(
        op="START", session_id="new", sequence=0,
        active_session_id=None, last_sequence=-1,
    ) is None
    assert "already active" in validate_sequence(
        op="START", session_id="new", sequence=0,
        active_session_id="old", last_sequence=4,
    )
    assert "session mismatch" in validate_sequence(
        op="UPDATE", session_id="wrong", sequence=5,
        active_session_id="old", last_sequence=4,
    )
    assert "not newer" in validate_sequence(
        op="UPDATE", session_id="old", sequence=4,
        active_session_id="old", last_sequence=4,
    )


def test_velocity_limiter_bounds_every_joint_per_tick():
    result = velocity_limited_positions(
        {"joint_a": 0.0, "joint_b": 1.0},
        {"joint_a": 1.0, "joint_b": 0.0},
        SPECS,
        {"motor_a": 0.4, "motor_b": 0.2},
        0.1,
    )
    assert result == pytest.approx({"joint_a": 0.04, "joint_b": 0.98})


def test_watchdog_has_strict_freshness_boundary():
    assert not watchdog_expired(10.0, 0.35, now_monotonic=10.35)
    assert watchdog_expired(10.0, 0.35, now_monotonic=10.351)

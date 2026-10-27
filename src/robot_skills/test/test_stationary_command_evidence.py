from types import SimpleNamespace
import time

from robot_skills.executors.system import SystemSkills


def test_no_command_evidence_does_not_prove_stationary():
    result = SystemSkills(None).await_robot_stationary(settle_sec=0, timeout_sec=.01)
    assert not result.success
    assert result.metrics["cmd_vel_receipt_age_sec"] is None


def test_fresh_zero_command_can_settle():
    skill = SystemSkills(None)
    skill.update_cmd_vel(0, 0, 0)
    result = skill.await_robot_stationary(settle_sec=0, timeout_sec=.01)
    assert result.success
    assert result.metrics["cmd_vel_receipt_age_sec"] < .5


def test_old_zero_command_cannot_settle():
    skill = SystemSkills(None)
    skill.update_cmd_vel(0, 0, 0)
    skill._last_cmd_receipt_time = time.monotonic() - 2
    assert not skill.await_robot_stationary(settle_sec=0, timeout_sec=.01).success


def test_slow_sim_motion_is_not_stationary():
    skill = SystemSkills(None)
    skill.update_cmd_vel(.05, 0, 0)
    assert not skill.await_robot_stationary(settle_sec=0, timeout_sec=.01).success


def test_cancel_interrupts_stationary_wait():
    result = SystemSkills(None).await_robot_stationary(
        context=SimpleNamespace(cancel_check=lambda: True), timeout_sec=180)
    assert result.code == "CANCELLED"

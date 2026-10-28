import pytest
from robot_skills.executors.system import SystemSkills


def qualified_system():
    system=SystemSkills(None,'sim')
    # Scan-time odom lags receipt-time odom by 0.1m; moving robot.
    for i in range(5):
        system.update_cmd_vel(.1,0,0)
        system.update_wheel_odom_pose(4+.1*i,0,0,stamp=10+i)
        system.update_gicp_score(.1)
        system.update_odom_pose(20+.1*i,0,0,stamp=9+i,odom_anchor=(3.9+.1*i,0,0))
    return system


def test_anchor_uses_scan_time_odom_not_latest_receipt_odom():
    system=qualified_system()
    assert system._wheel_odom_pose_at_trust == pytest.approx((4.3,0,0))
    assert system.odom_propagated_trusted_pose() == pytest.approx((20.5,0,0))
    assert system._trusted_pose_stamp==13


def test_odom_reset_invalidates_trusted_anchor():
    system=qualified_system();assert system.last_trusted_pose() is not None
    system.update_wheel_odom_pose(0,0,0,stamp=14.1)
    assert system.last_trusted_pose() is None
    assert system.odom_propagated_trusted_pose() is None


def test_warning_does_not_poison_anchor_or_force_task_failure():
    s=qualified_system();anchor=s.last_trusted_pose()
    s.update_gicp_score(.17)
    s.update_odom_pose(20.4,0,0,stamp=13.1,odom_anchor=(4.3,0,0))
    assert s.last_trusted_pose()==anchor
    assert s.localization_is_healthy()  # HealthMonitor owns WARN persistence, not a task abort.
    s.update_gicp_score(.7)
    assert not s.localization_is_healthy()


def test_receipt_age_does_not_follow_calendar_clock_changes(monkeypatch):
    import robot_skills.executors.system as module
    monotonic=[100.0];monkeypatch.setattr(module.time,'monotonic',lambda:monotonic[0])
    system=SystemSkills(None,'sim');system.update_wheel_odom_pose(0,0,0,stamp=10)
    monkeypatch.setattr(module.time,'time',lambda:-1000000);monotonic[0]+=.25
    assert system.localization_metrics()['wheel_odom_age_sec']==.25

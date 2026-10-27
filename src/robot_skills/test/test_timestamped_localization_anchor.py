from robot_skills.executors.system import SystemSkills


def test_anchor_uses_scan_time_odom_not_latest_receipt_odom():
    system = SystemSkills(None, 'sim')
    system.update_wheel_odom_pose(5, 0, 0, stamp=10)
    for _ in range(3):
        system.update_gicp_score(.1)
    system.update_odom_pose(20, 0, 0, stamp=9, odom_anchor=(4, 0, 0))
    assert system._wheel_odom_pose_at_trust == (4, 0, 0)
    assert system.odom_propagated_trusted_pose() == (21, 0, 0)
    assert system._trusted_pose_stamp == 9


def test_odom_reset_invalidates_trusted_anchor():
    system = SystemSkills(None, 'sim')
    system.update_wheel_odom_pose(5, 0, 0, stamp=10)
    for _ in range(3):
        system.update_gicp_score(.1)
    system.update_odom_pose(20, 0, 0, stamp=10, odom_anchor=(5, 0, 0))
    system.update_wheel_odom_pose(0, 0, 0, stamp=10.1)
    assert system.last_trusted_pose() is None


def test_receipt_age_does_not_follow_calendar_clock_changes(monkeypatch):
    import robot_skills.executors.system as module
    monotonic = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: monotonic[0])
    system = SystemSkills(None, 'sim')
    system.update_wheel_odom_pose(0, 0, 0, stamp=10)
    monkeypatch.setattr(module.time, 'time', lambda: -1000000)
    monotonic[0] += .25
    assert system.localization_metrics()['wheel_odom_age_sec'] == .25

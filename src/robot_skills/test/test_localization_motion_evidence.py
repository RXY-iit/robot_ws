from robot_skills.executors.system import SystemSkills


def test_normal_travel_with_fresh_localization_is_not_divergence():
    system = SystemSkills(None, "sim")
    system.update_wheel_odom_pose(0, 0, 0)
    for _ in range(3):
        system.update_gicp_score(.1)
    system.update_odom_pose(0, 0, 0)
    system.update_cmd_vel(.1, 0, 0)
    system.update_wheel_odom_pose(1, 0, 0)
    system.update_odom_pose(1, 0, 0)
    result = system.localization_metrics()
    assert result["trusted_pose_delta_m"] == 0
    assert result["localization_warning"] == "MOVING"
    assert result["localization_healthy"]


def test_travel_with_stale_localization_still_requires_attention():
    system = SystemSkills(None, "sim")
    system.update_wheel_odom_pose(0, 0, 0)
    for _ in range(3):
        system.update_gicp_score(.1)
    system.update_odom_pose(0, 0, 0)
    system.update_cmd_vel(.1, 0, 0)
    system.update_wheel_odom_pose(1, 0, 0)
    system._last_gicp_score_time -= 10
    assert system.localization_metrics()["localization_warning"] == "GICP_TRUSTED_POSE_DIVERGENCE"


def test_initialpose_keeps_previous_anchor_but_not_healthy_evidence():
    system = SystemSkills(None, "sim")
    system.update_wheel_odom_pose(0, 0, 0, stamp=10.)
    for _ in range(3):
        system.update_gicp_score(.1)
    system.update_odom_pose(1, 2, 0)
    before = system.localization_metrics()
    assert before["trusted_pose_has_odom_anchor"]
    system.update_initialpose()
    system.update_wheel_odom_pose(.02, 0, 0, stamp=10.1)
    after = system.localization_metrics()
    assert after["trusted_pose_has_odom_anchor"]
    assert not after["localization_healthy"]
    assert after["odom_propagated_trusted_pose"][0] == 1.02
    system.update_wheel_odom_pose(0, 0, 0, stamp=1.)
    assert not system.localization_metrics()["trusted_pose_has_odom_anchor"]

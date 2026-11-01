from robot_agent.motion_pair import MotionPairWindow


def test_aligned_motion_reports_distance_disagreement_without_declaring_slip():
    pair = MotionPairWindow()
    for i in range(31):
        t = i * 0.2
        pair.observe("wheel", t, 0.2 * t, 0.0, 0.0)
        pair.observe("lio", t + 0.04, 0.1 * t, 0.0, 0.0)
    result = pair.assess()
    assert result is not None
    assert result["motion_observed"]
    assert result["wheel_path_m"] > result["lio_path_m"]
    assert result["path_difference_fraction"] > 0.8
    assert "not a slip classification" in result["limitation"]


def test_stationary_pair_does_not_invent_relative_disagreement():
    pair = MotionPairWindow()
    for i in range(31):
        for source in ("wheel", "lio"):
            pair.observe(source, i * 0.2, 0.0, 0.0, 0.0)
    result = pair.assess()
    assert result is not None
    assert result["motion_observed"] is False
    assert result["path_difference_fraction"] is None


def test_lio_restart_drift_while_wheel_still_is_not_classified_as_motion_slip():
    pair = MotionPairWindow()
    for i in range(31):
        t = i * 0.2
        pair.observe("wheel", t, 0.0, 0.0, 0.0)
        pair.observe("lio", t, 0.05 * t, 0.0, 0.0)
    result = pair.assess()
    assert result["motion_observed"] is False
    assert result["lio_motion_while_wheel_stationary"] is True
    assert result["path_difference_fraction"] is None


def test_missing_alignment_and_epoch_reset_do_not_bridge_windows():
    pair = MotionPairWindow()
    for i in range(31):
        pair.observe("wheel", i * 0.2, i * 0.01, 0.0, 0.0)
        pair.observe("lio", 100 + i * 0.2, i * 0.01, 0.0, 0.0)
    assert pair.assess() is None
    pair.observe("wheel", 1.0, 0.0, 0.0, 0.0)
    assert pair.assess() is None

import math

import pytest

from robot_skills.executors.navigation import update_recovered_pose_metrics
from robot_skills.schemas import Pose2D


def test_recovery_updates_original_goal_error_and_preserves_history():
    old_pose = {"x": 0, "y": 0, "yaw": 0}
    metrics = {"final_pose": old_pose, "final_xy_error_m": 5, "final_yaw_error_rad": 1}
    new_pose = {"x": 2.8, "y": 4, "yaw": -.1}
    update_recovered_pose_metrics(metrics, Pose2D(3, 4, .1), new_pose, "alternate_goal_result")
    assert metrics["final_xy_error_m"] == pytest.approx(.2)
    assert metrics["final_yaw_error_rad"] == pytest.approx(.2)
    assert metrics["final_pose"] == new_pose
    assert metrics["pre_recovery_pose_evidence"]["final_pose"] == old_pose
    assert metrics["pre_recovery_pose_evidence"]["final_xy_error_m"] == 5


def test_missing_recovery_pose_does_not_reuse_stale_final_evidence():
    metrics = {"final_pose": {"x": 1, "y": 1, "yaw": 0}, "final_xy_error_m": 2}
    update_recovered_pose_metrics(metrics, Pose2D(0, 0, 0), None, "retry")
    assert metrics["final_pose"] is None
    assert metrics["final_xy_error_m"] is None
    assert metrics["final_yaw_error_rad"] is None


def test_recovery_error_wraps_yaw_and_retains_original_history():
    metrics = {"final_xy_error_m": 3}
    for source in ("retry1", "retry2"):
        update_recovered_pose_metrics(
            metrics, Pose2D(0, 0, math.pi), {"x": 0, "y": 0, "yaw": -math.pi + .1}, source)
    assert metrics["final_yaw_error_rad"] == pytest.approx(.1)
    assert metrics["pre_recovery_pose_evidence"]["final_xy_error_m"] == 3

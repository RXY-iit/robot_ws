import math

import pytest

from robot_skills.executors.navigation import near_goal_yaw_error, yaw_progress_reference
from robot_skills.schemas import Pose2D


def test_near_goal_rotation_counts_as_progress_without_translation():
    goal = Pose2D(x=0, y=0, yaw=0)
    error = near_goal_yaw_error({"x": .03, "y": 0, "yaw": 1.5}, goal, .25)
    assert yaw_progress_reference(1.8, error) == (1.5, True)


def test_far_from_goal_rotation_is_not_final_alignment_progress():
    assert near_goal_yaw_error({"x": 1, "y": 0, "yaw": 1}, Pose2D(0, 0, 0), .25) is None
    assert yaw_progress_reference(1.8, None) == (1.8, False)


def test_server_default_tolerance_does_not_raise_after_goal_acceptance():
    assert near_goal_yaw_error({"x": 0, "y": 0, "yaw": 1}, Pose2D(0, 0, 0), None) is None


def test_angle_wrap_uses_shortest_goal_error():
    error = near_goal_yaw_error({"x": 0, "y": 0, "yaw": -math.pi + .1}, Pose2D(0, 0, math.pi), .25)
    assert error == pytest.approx(.1)


def test_noise_or_oscillation_cannot_keep_watchdog_alive():
    best = 1.0
    for error in [1.01, .98, 1.2, .97, 1.0]:
        best, progressed = yaw_progress_reference(best, error)
        assert not progressed
        assert best == 1.0
    assert yaw_progress_reference(best, .94) == (.94, True)
    assert yaw_progress_reference(None, 1.0) == (1.0, False)

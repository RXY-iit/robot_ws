import math

import pytest

from robot_skills.motion_evidence import MotionProgress, swept_grid_clear, translation_error
from robot_skills.executors.base import make_result


def test_wrong_direction_and_sideways_are_not_completion():
    start = {"x": 0, "y": 0, "yaw": math.pi / 2}
    along, cross, error = translation_error(start, {"x": 0, "y": -.3}, .3, 0)
    assert along == pytest.approx(-.3)
    assert error == pytest.approx(.6)
    along, cross, error = translation_error(start, {"x": .3, "y": 0}, .3, 0)
    assert cross == pytest.approx(.3)
    assert error > .3
    assert translation_error(start, {"x": 0, "y": .3}, .3, 0)[2] < 1e-10


def test_stall_uses_source_time_with_independent_wall_cap():
    p = MotionProgress(1, 0, 0, .01)
    assert not p.stalled(1, 1, 20)
    assert p.stalled(1, 5, 21)
    assert not p.stalled(.95, 5, 21)
    for wall, error in [(22, .96), (23, 1), (24, .95)]:
        assert not p.stalled(error, 5, wall)
    assert p.stalled(.95, 5, 52)


def test_swept_grid_rejects_blocked_unknown_outside_and_invalid():
    grid = [0] * 100
    footprint = [(2, 2), (3, 2), (3, 3), (2, 3)]
    args = (10, 10, 1, (0, 0, 0), footprint, (2, 0))
    assert swept_grid_clear(grid, *args)
    grid[25] = 100
    assert not swept_grid_clear(grid, *args)
    grid[25] = -1
    assert not swept_grid_clear(grid, *args)
    assert not swept_grid_clear([0] * 100, 10, 10, 1, (0, 0, 0), footprint, (20, 0))
    assert not swept_grid_clear([], *args)


@pytest.mark.parametrize("status,success,expected", [(4, True, "nav2_strict"),
    ("cancelled_after_task_tolerance_reached", True, "task_tolerance"),
    ("staged_relaxed_tolerance_success", True, "degraded"), (6, False, "not_successful")])
def test_navigation_completion_grades(status, success, expected):
    assert make_result("goto_pose", success, "OK", "", metrics={"nav2_status": status}).metrics["completion_class"] == expected


def test_strict_result_after_fallback_is_not_strict_baseline():
    result = make_result("goto_pose", True, "OK", "", metrics={"nav2_status": 4,
        "staged_recovery": {"micro_adjust": {"attempted": True}}})
    assert result.metrics["completion_class"] == "degraded"

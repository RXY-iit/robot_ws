import threading
import time
from types import SimpleNamespace

import pytest

from robot_skills.executors.navigation import NavigationSkills
from robot_skills.schemas import Pose2D, SkillContext
from robot_skills import result_codes as codes


def test_recovery_constraint_prevents_dispatch_without_touching_nav2():
    nav = NavigationSkills.__new__(NavigationSkills)
    context = SkillContext("m1")
    context.recovery_check = lambda: True
    result = nav.goto_pose(context, Pose2D(1, 0, 0))
    assert result.code == codes.CANCELLED
    assert "Recovery Agent" in result.message


@pytest.mark.parametrize("pending,done,velocity,age,expected", [
    (0, True, (0, 0, 0), .01, True),
    (1, True, (0, 0, 0), .01, False),
    (0, False, (0, 0, 0), .01, False),
    (0, True, (.1, 0, 0), .01, False),
    (0, True, (0, 0, float('nan')), .01, False),
    (0, True, (0, 0, 0), 2, False),
])
def test_pause_ack_requires_terminal_goal_and_fresh_measured_stop(pending, done, velocity, age, expected):
    nav = NavigationSkills.__new__(NavigationSkills)
    nav._pending_nav_goals = pending
    nav._nav_result_futures = [SimpleNamespace(done=lambda: done, result=lambda: SimpleNamespace(status=5))]
    nav._wheel_odom_lock = threading.Lock()
    nav._recovery_wheel_twist = velocity
    nav._latest_wheel_odom_time = time.monotonic() - age
    nav._latest_wheel_odom_stamp = 10 - age
    nav.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10e9)))
    assert nav.recovery_motion_stopped() is expected


@pytest.mark.parametrize("invalid", [None, SimpleNamespace(status=0), RuntimeError("result transport failed")])
def test_done_future_without_terminal_action_result_cannot_acknowledge_pause(invalid):
    nav = NavigationSkills.__new__(NavigationSkills)
    nav._pending_nav_goals = 0
    def result():
        if isinstance(invalid, Exception):
            raise invalid
        return invalid
    nav._nav_result_futures = [SimpleNamespace(done=lambda: True, result=result)]
    assert not nav.recovery_motion_stopped()

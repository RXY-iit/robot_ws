import math
from types import SimpleNamespace

import pytest

from robot_skills.executors.navigation import NavigationSkills
from robot_skills.schemas import SkillContext


def driver(monkeypatch, wrong_direction=False):
    nav = object.__new__(NavigationSkills)
    clock = [0.0]
    pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    nav.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock[0] * 1e9))))
    nav._current_map_pose = lambda: dict(pose)
    nav._current_wheel_odom_pose = lambda: dict(pose)
    nav._publish_stop = lambda: None
    def publish(twist):
        sign = -1 if wrong_direction else 1
        pose["x"] += sign * twist.linear.x * .05
        pose["y"] += sign * twist.linear.y * .05
        wz = twist.angular.z if abs(twist.angular.z) >= .14 else 0.0
        pose["yaw"] = math.atan2(math.sin(pose["yaw"] + wz * .05), math.cos(pose["yaw"] + wz * .05))
    nav._cmd_vel_pub = SimpleNamespace(publish=publish)
    monkeypatch.setattr("robot_skills.executors.navigation.time.monotonic", lambda: clock[0])
    monkeypatch.setattr("robot_skills.executors.navigation.time.sleep", lambda dt: clock.__setitem__(0, clock[0] + dt))
    return nav


def test_wrong_direction_stops_without_false_success(monkeypatch):
    nav = driver(monkeypatch, True)
    result = nav._direct_translate("move_relative", SkillContext("test"), .3, 0, 30, {})
    assert not result.success
    assert result.metrics["timeout_reason"] == "relative_motion_off_path"


@pytest.mark.parametrize("angle", [math.radians(20), math.pi, -math.pi, 2 * math.pi])
def test_rotation_requires_signed_accumulated_motion(monkeypatch, angle):
    nav = driver(monkeypatch)
    result = nav._direct_rotate("rotate_relative", SkillContext("test"), angle, 40, {})
    assert result.success
    assert result.metrics["signed_accumulated_yaw_rad"] == pytest.approx(angle, abs=.05)


def test_micro_adjust_guard_checked_before_publishing(monkeypatch):
    nav = driver(monkeypatch)
    nav._cmd_vel_pub = SimpleNamespace(publish=lambda _: pytest.fail("Published through failed guard"))
    result = nav._direct_translate("goto_pose_near_goal_micro_adjust", SkillContext("test"), .3, 0, 5, {},
                                   motion_guard=lambda: "micro_adjust_swept_footprint_blocked")
    assert not result.success
    assert result.metrics["timeout_reason"] == "micro_adjust_swept_footprint_blocked"


@pytest.mark.parametrize("reason", ["navigation_localization_unhealthy", "micro_adjust_tf_stale", "micro_adjust_swept_footprint_blocked"])
def test_navigation_alignment_checks_guard_before_publishing(monkeypatch, reason):
    nav = driver(monkeypatch)
    def guard(dx, dy, rotating=False):
        assert rotating and dx == dy == 0
        return reason
    nav._micro_adjust_guard = guard
    nav._cmd_vel_pub = SimpleNamespace(publish=lambda _: pytest.fail("Published through failed rotation guard"))
    result = nav._direct_rotate("goto_pose_pre_align", SkillContext("test"), .5, 5, {})
    assert not result.success
    assert result.metrics["timeout_reason"] == reason


@pytest.mark.parametrize("wall_age,source_age,reason", [(1, 0, "receive_stale"),
    (.1, 1, "source_stale"), (.1, -.1, "stamp_future"), (.1, .1, None)])
def test_wheel_odom_rejection_retains_exact_evidence(monkeypatch, wall_age, source_age, reason):
    import threading
    nav = driver(monkeypatch)
    nav._wheel_odom_lock = threading.Lock()
    nav._latest_wheel_odom_time = -wall_age
    nav._latest_wheel_odom_stamp = -source_age
    nav._latest_wheel_odom_pose = {"x": 0, "y": 0, "yaw": 0}
    result = NavigationSkills._current_wheel_odom_pose(nav)
    assert (result is None) == (reason is not None)
    assert nav._last_wheel_odom_evidence["reason"] == reason
    assert nav._last_wheel_odom_evidence["receive_age_sec"] == wall_age
    assert nav._last_wheel_odom_evidence["source_age_sec"] == source_age

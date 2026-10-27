import math
from types import SimpleNamespace

import pytest

from robot_agent.recovery.trusted_pose import trusted_seed
from robot_agent.recovery.backends import RosActionBackend
from robot_agent.recovery.executor import DONE, ERROR


def metrics():
    return {
        "last_trusted_pose_age_sec": 8,
        "wheel_odom_age_sec": 0.1,
        "trusted_pose_has_odom_anchor": True,
        "odom_propagated_trusted_pose": [1, 2, 0],
        "trusted_pose_delta_m": 0.2,
        "trusted_pose_delta_yaw_rad": 0.1,
    }


def test_trusted_seed_uses_propagated_pose():
    assert trusted_seed(metrics(), {}, 1) == (1., 2., 0.)


def test_deferred_recovery_uses_motion_since_trusted_pose():
    from robot_skills.executors.system import SystemSkills
    system = object.__new__(SystemSkills)
    system._last_trusted_pose = (10., 20., math.pi / 2)
    system._wheel_odom_pose_at_trust = (0., 0., 0.)
    system._latest_wheel_odom_pose = (-.2, 0., 0.)
    pose = system.odom_propagated_trusted_pose()
    assert pose == pytest.approx((10., 19.8, math.pi / 2))
    assert trusted_seed({**metrics(), 'odom_propagated_trusted_pose': pose}, {}, .1) == pose


@pytest.mark.parametrize("key,value", [
    ("wheel_odom_age_sec", 10), ("last_trusted_pose_age_sec", 121),
    ("trusted_pose_has_odom_anchor", False), ("trusted_pose_delta_m", 3),
    ("trusted_pose_delta_yaw_rad", 1), ("odom_propagated_trusted_pose", [float("nan"), 0, 0]),
])
def test_rejects_unsafe_or_unknown_seed(key, value):
    with pytest.raises(ValueError):
        trusted_seed({**metrics(), key: value}, {}, 0.1)


def test_stale_health_rejected():
    with pytest.raises(ValueError):
        trusted_seed(metrics(), {}, 5)


def test_source_age_is_not_added_to_wall_age():
    state = {**metrics(), "trusted_pose_frame": "base_footprint", "trusted_pose_stamp_sec": 9,
             "wheel_odom_stamp_sec": 10, "wheel_odom_source_age_sec": 2}
    assert trusted_seed(state, {"require_timestamped_base": True}, 2) == (1., 2., 0.)


def test_publish_propagates_timestamped_base_and_is_immediate(monkeypatch):
    from builtin_interfaces.msg import Time
    import robot_agent.recovery.backends as module
    monkeypatch.setattr(module.time, "monotonic", lambda: 10)
    backend = object.__new__(RosActionBackend)
    backend.config = {"last_trusted_pose": {"require_timestamped_base": True}}
    backend._localization_metrics = {**metrics(), "odom_propagated_trusted_pose": [1, 2, math.pi / 2],
                                    "last_trusted_pose": [1, 2, math.pi/2], "trusted_odom_anchor": [0, 0, 0],
                                    "trusted_pose_frame": "base_footprint", "trusted_pose_stamp_sec": 9,
                                    "wheel_odom_stamp_sec": 10, "wheel_odom_source_age_sec": 0}
    backend._health_received = 10
    backend.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10000000000, to_msg=lambda: Time(sec=10))))
    backend._tf_buffer = SimpleNamespace(lookup_transform=lambda *_: SimpleNamespace(header=SimpleNamespace(stamp=Time(sec=10)), transform=SimpleNamespace(
        translation=SimpleNamespace(x=-0.2, y=0., z=0.), rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    published = []
    backend._pose_pub = SimpleNamespace(publish=published.append, get_subscription_count=lambda: 1)
    handle = backend.start("publish_initialpose", {"source": "last_trusted_pose"})
    assert handle.poll()[0] == DONE
    assert len(published) == 1
    assert published[0].pose.pose.position.x == pytest.approx(1)
    assert published[0].pose.pose.position.y == pytest.approx(1.8)
    backend._health_received = 0
    assert backend.start("publish_initialpose", {"source": "last_trusted_pose"}).poll()[0] == ERROR
    assert len(published) == 1

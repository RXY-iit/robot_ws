import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from robot_agent.nodes.agent_node import AgentNode


def evidence():
    return dict(stamp_sec=9.5, map_frame="map", odom_frame="odom", base_frame="base_footprint",
                map_base_pose=[1, 2, .1], odom_base_pose=[.2, .3, .4], rmse_m=.1)


def deliver(state):
    system = SimpleNamespace(_evidence_stamp=None, update_gicp_score=Mock(), update_odom_pose=Mock())
    node = SimpleNamespace(skill_client=SimpleNamespace(system=system),
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)))
    AgentNode._accepted_localization_cb(node, SimpleNamespace(data=json.dumps(state)))
    return system


def test_atomic_evidence_contains_same_timestamp_odom_anchor():
    system = deliver(evidence())
    system.update_gicp_score.assert_called_once_with(.1)
    system.update_odom_pose.assert_called_once_with(1, 2, .1, stamp=9.5, odom_anchor=(.2, .3, .4))


@pytest.mark.parametrize("change", [dict(stamp_sec=1), dict(stamp_sec=11),
    dict(base_frame="livox_frame"), dict(rmse_m=float("nan")),
    dict(map_base_pose=[1, 2]), dict(odom_base_pose=[1, 2, float("inf")])])
def test_invalid_evidence_does_not_update_either_pose_or_score(change):
    system = deliver({**evidence(), **change})
    system.update_gicp_score.assert_not_called()
    system.update_odom_pose.assert_not_called()

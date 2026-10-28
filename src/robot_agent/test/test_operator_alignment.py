"""A human re-alignment can replace a reset reference; PB seeds cannot."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_agent.nodes.agent_node import AgentNode
from robot_skills.executors.system import SystemSkills


def deliver(operator, stopped=True, frame='map', invalid=False):
    system = SystemSkills(None, 'sim')
    system.update_robot_mode('AUTO')
    system.invalidate_localization_anchor()
    client = SimpleNamespace(system=system, update_initialpose=system.update_initialpose,
                             navigation=SimpleNamespace(recovery_motion_stopped=lambda: stopped))
    node = SimpleNamespace(skill_client=client, _publish_event=Mock(),
                           _yaw_from_quaternion=AgentNode._yaw_from_quaternion, get_logger=lambda: Mock())
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame
    msg.pose.pose.orientation.w = 1.
    if invalid:
        msg.pose.pose.position.x = float('nan')
    AgentNode._handle_initialpose(node, msg, operator_requested=operator)
    return system, node


def samples(s, clock, times, score=.12):
    for t in times:
        clock[0] = 120. + t
        s.update_cmd_vel(0., 0., 0.)
        s.update_wheel_odom_pose(0., 0., 0., stamp=t)
        s.update_gicp_score(score)
        s.update_odom_pose(5., 0., 0., stamp=t, odom_anchor=(0., 0., 0.))


def test_auto_rviz_alignment_after_reset_requires_stable_new_evidence(monkeypatch):
    clock = [100.]
    monkeypatch.setattr('robot_skills.executors.system.time.monotonic', lambda: clock[0])
    s, node = deliver(True)
    assert s._manual_alignment_pending and not s._reference.verified
    assert not s.localization_is_healthy()
    samples(s, clock, [1., 2., 3.])
    assert not s._reference.verified
    assert not s.localization_is_healthy()
    samples(s, clock, [4.])
    assert s._reference.verified and s.localization_is_healthy()
    assert s.last_trusted_pose() == (5., 0., 0.)
    assert node._publish_event.call_args.args[0]['payload']['operator_alignment_authorized']


@pytest.mark.parametrize('mode', ['MANUAL', 'AUTO'])
def test_pb_or_unknown_seed_cannot_reset_reference_even_in_manual(mode, monkeypatch):
    clock = [100.]
    monkeypatch.setattr('robot_skills.executors.system.time.monotonic', lambda: clock[0])
    s, node = deliver(False)
    s.update_robot_mode(mode)
    s.update_initialpose(source='/initialpose')
    samples(s, clock, [1., 2., 3., 4., 5.])
    assert s._reference.reset_required and not s._reference.verified
    assert not s.localization_is_healthy()
    assert node._publish_event.call_args.args[0]['type'] == 'initialpose_received'


@pytest.mark.parametrize('kwargs', [dict(stopped=False), dict(frame='odom'), dict(invalid=True)])
def test_operator_request_needs_stopped_valid_alignment(kwargs):
    s, node = deliver(True, **kwargs)
    assert s._reference.reset_required and not s._manual_alignment_pending
    assert not node._publish_event.call_args.args[0]['payload']['operator_alignment_authorized']


@pytest.mark.parametrize('disturbance', ['odom_reset', 'another_seed'])
def test_alignment_authorization_does_not_survive_new_reset_or_pb_seed(disturbance, monkeypatch):
    clock = [100.]
    monkeypatch.setattr('robot_skills.executors.system.time.monotonic', lambda: clock[0])
    s, _ = deliver(True)
    samples(s, clock, [1., 2.])
    if disturbance == 'odom_reset':
        s.update_wheel_odom_pose(20., 0., 0., stamp=2.1)
    else:
        s.update_initialpose(source='/initialpose')
    samples(s, clock, [3., 4., 5., 6.])
    assert s._reference.reset_required and not s._reference.verified
    assert not s.localization_is_healthy()


def test_bad_score_after_operator_alignment_does_not_qualify(monkeypatch):
    clock = [100.]
    monkeypatch.setattr('robot_skills.executors.system.time.monotonic', lambda: clock[0])
    s, _ = deliver(True)
    samples(s, clock, [1., 2., 3., 4., 5.], score=.21)
    assert not s._reference.verified and not s.localization_is_healthy()

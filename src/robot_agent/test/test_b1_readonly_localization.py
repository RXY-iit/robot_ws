"""The TOR observer must accept only the same fresh, ordered evidence as COR."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from robot_skills.executors.system import SystemSkills

path=Path(__file__).resolve().parents[3]/'tools/sim/b1_readonly_localization.py'
spec=importlib.util.spec_from_file_location('b1_readonly',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

def observer():
    clock=SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100_000_000_000))
    node=SimpleNamespace(get_clock=lambda:clock)
    return SimpleNamespace(system=SystemSkills(node,'sim'),get_clock=lambda:clock)

def message(**changes):
    data=dict(stamp_sec=99.,map_base_pose=[1.,2.,.1],odom_base_pose=[.5,1.,.1],rmse_m=.12,base_frame='base_footprint',map_frame='map',odom_frame='odom')
    data.update(changes);return SimpleNamespace(data=json.dumps(data))

def test_fresh_accepted_and_duplicate_does_not_refresh():
    o=observer();module.Observer.accepted(o,message())
    assert o.system._gicp_score==.12
    assert o.system._evidence_stamp==99.
    received=o.system._last_gicp_score_time
    module.Observer.accepted(o,message(rmse_m=.6))
    assert o.system._last_gicp_score_time==received
    assert o.system._gicp_score==.12

@pytest.mark.parametrize('change',[dict(stamp_sec=97.),dict(stamp_sec=101.),dict(map_frame='odom'),dict(rmse_m=float('nan')),dict(map_base_pose=[1,2])])
def test_invalid_evidence_does_not_seed_health(change):
    o=observer();module.Observer.accepted(o,message(**change))
    assert o.system._evidence_stamp is None
    assert o.system._gicp_score is None

@pytest.mark.parametrize('statuses,velocity,stamp,allowed', [
    ([4], (0., 0., 0.), 100., True),
    ([2], (0., 0., 0.), 100., False),
    (None, (0., 0., 0.), 100., False),
    ([4], (.2, 0., 0.), 100., False),
    ([4], (0., 0., 0.), 98., False),
])
def test_tor_operator_alignment_requires_stopped_terminal_navigation(statuses, velocity, stamp, allowed):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    o=observer();o.system.invalidate_localization_anchor()
    o.system.update_wheel_odom_pose(0.,0.,0.,stamp=stamp)
    o.goal_status=statuses;o.wheel_velocity=velocity
    msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.pose.pose.orientation.w=1.
    module.Observer.operator_initialpose(o,msg)
    assert o.system._manual_alignment_pending is allowed
    assert o.system._reference.reset_required is not allowed

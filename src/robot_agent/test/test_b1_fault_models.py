import sys
from pathlib import Path
import math
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'/'sim'))
from b1_fault_models import OdomError,StableSegment
from make_b1_fault_map import deform


def test_turn_error_remains_after_stopping_growth():
    m=OdomError(turn_scale=.2);m.update((0,0,0),1);m.active=True
    pose,twist=m.update((0,0,math.pi/2),2)
    assert pose[2]==pytest.approx(math.pi*.6)
    m.active=False;pose,twist=m.update((0,1,math.pi/2),3)
    assert pose[0]<-.3 and pose[1]>.9
    assert pose[2]==pytest.approx(math.pi*.6)
    assert twist[2]==0


def test_scale_accumulates_without_pose_jump_and_detects_reset():
    m=OdomError(distance_scale=.1);m.update((0,0,0),1);m.active=True
    assert m.update((1,0,0),2)[0][0]==pytest.approx(1.1)
    with pytest.raises(ValueError):m.update((0,0,0),2.01)


def test_straight_gate_needs_ordered_events_and_turn_settling():
    g=StableSegment((1,0),(5,0))
    g.goal((5,0),'accepted');assert not g.update((2,0),0,.1,0,1)
    g.goal((1,0),'succeeded');g.goal((5,0),'accepted')
    assert not g.update((1,0),1,.1,.3,2)
    assert not g.update((1.3,0),0,.1,0,3)
    assert g.update((1.6,0),0,.1,0,5)


def test_double_wall_changes_only_selected_points_and_records_actual_fraction():
    points=np.array([[0,y,z] for y in range(4) for z in range(4)]+[[10,10,10]],float)
    result,d=deform(points,[-.1,-.1,-.1,.1,3.1,3.1],[1,0,0],.2,.5,0,7)
    assert len(result)==25 and d['duplicated_points']==8
    assert np.array_equal(result[:len(points)],points)
    assert np.allclose(result[len(points):,0],.2)
    coarse,meta=deform(points,[-.1,-.1,-.1,.1,3.1,3.1],[1,0,0],0,0,2,7)
    assert len(coarse)<len(points) and meta['duplicated_points']==0
    with pytest.raises(ValueError):deform(points,[20,20,20,21,21,21],[1,0,0],.2,.5,0,7)


def test_negative_turn_bias_reduces_accumulated_turn():
    m=OdomError(turn_bias_rad_per_m=-.1);m.update((0,0,0),1);m.active=True
    assert m.update((1,0,.2),2)[0][2]==pytest.approx(.1)


def test_physics_calibration_handles_rotation_and_translation():
    from calibrate_b1_physics_frame import calibration
    from importlib.util import spec_from_file_location,module_from_spec
    spec=spec_from_file_location('b1plot',Path(__file__).resolve().parents[3]/'tools/sim/analysis/plot_b1_factors.py')
    plot=module_from_spec(spec);spec.loader.exec_module(plot)
    transform=calibration([1,2,math.pi/2],[5,6,math.pi])
    assert plot.aligned([1,2,math.pi/2],transform)==pytest.approx([5,6,math.pi])
    assert plot.se2([1,5,math.pi/2],[2,0,0])==pytest.approx([1,3,math.pi/2])


def test_committed_odom_window_does_not_follow_goal_cancellation_or_wall_delay():
    from b1_fault_models import FaultSchedule
    cor,tor=FaultSchedule(),FaultSchedule()
    for schedule in (cor,tor):schedule.start('O',100,20)
    # COR may be cancelled/paused at 105, TOR may still move. Neither completes early.
    assert not cor.elapsed('O',105) and not tor.elapsed('O',105)
    assert not cor.elapsed('O',119.9)
    assert cor.elapsed('O',120) and tor.elapsed('O',120)
    with pytest.raises(ValueError):cor.start('O',130,20)
    with pytest.raises(ValueError):cor.elapsed('O',1)

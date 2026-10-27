import pytest
from test_recovery_executor import make, feed


def active_mission(skill='goto_pose'):
    return {'mission_id': 'm1', 'status': 'EXECUTE_STEP',
            'plan': [{'name': skill}], 'current_step_index': 0}


def fault(engine):
    feed(engine, 0, {'gicp_score': 'DEGRADED'})
    feed(engine, 6, {'gicp_score': 'DEGRADED'})


def test_idle_localization_fault_starts_background_recovery():
    engine, backend = make(profile='nav2')
    fault(engine)
    assert [c[0] for c in backend.calls] == ['cancel_motion', 'publish_initialpose']


@pytest.mark.parametrize('skill', ['move_relative', 'rotate_relative', 'capture_observation', 'set_pose'])
def test_stationary_independent_task_does_not_defer_recovery(skill):
    engine, backend = make(profile='nav2')
    engine.mission_state = active_mission(skill)
    fault(engine)
    assert [c[0] for c in backend.calls] == ['cancel_motion', 'publish_initialpose']
    assert engine.mission_state['status'] == 'EXECUTE_STEP'
    assert not engine._pending_requests
    constraints = engine.state()['task_supervision']['constraints']
    assert constraints and constraints[0]['capabilities'] == ['auto_navigation']
    feed(engine, 7)
    assert engine.executor.finished[-1].state == 'SUCCEEDED'


def test_unrelated_motion_waits_for_stationary_window_without_cancelling_task():
    engine, backend = make(profile='nav2')
    engine.mission_state = active_mission('move_relative')
    moving = {'cmd_vel': {'linear_x': .2, 'linear_y': 0, 'angular_z': 0}}
    feed(engine, 0, {'gicp_score': 'DEGRADED'}, moving)
    feed(engine, 6, {'gicp_score': 'DEGRADED'}, moving)
    assert not backend.calls
    assert engine.executor.active['gicp'].state == 'WAITING_PRECONDITIONS'
    feed(engine, 7, {'gicp_score': 'DEGRADED'})
    assert backend.calls[-1][0] == 'publish_initialpose'
    assert engine.mission_state['status'] == 'EXECUTE_STEP'


def test_localization_precheck_wait_does_not_require_navigation_ack():
    engine, backend = make(profile='nav2')
    engine.mission_state = active_mission('check_localization_health')
    engine.mission_state.update(status='PAUSED_HEALTH', recovery_pause={'motion_stopped': False})
    fault(engine)
    assert backend.calls[-1][0] == 'publish_initialpose'


def test_navigation_is_paused_on_health_without_skill_failure_or_request():
    engine, backend = make(profile='nav2')
    engine.mission_state = active_mission()
    fault(engine)
    assert not backend.calls
    assert 'acknowledge' in engine.incidents['gicp'].execution_blocked
    assert engine.state()['task_supervision']['constraints'][0]['action'] == 'pause_dependent'
    engine.mission_state.update(status='PAUSED_HEALTH', recovery_pause={'motion_stopped': False})
    feed(engine, 7, {'gicp_score': 'DEGRADED'})
    assert not backend.calls
    engine.mission_state['recovery_pause']['motion_stopped'] = True
    feed(engine, 8, {'gicp_score': 'DEGRADED'})
    assert backend.calls[-1][0] == 'publish_initialpose'
    assert not engine._pending_requests


def test_healthy_evidence_must_settle_before_constraint_release():
    engine, backend = make(profile='nav2')
    engine.mission_state = active_mission()
    fault(engine)
    feed(engine, 7)
    assert engine.state()['task_supervision']['constraints']
    feed(engine, 30)
    assert not engine.state()['task_supervision']['constraints']
    assert not backend.calls


def test_mission_cancel_does_not_cancel_global_health_recovery():
    engine, backend = make(profile='nav2')
    engine.mission_state = active_mission()
    fault(engine)
    engine.mission_state['status'] = 'CANCELLED'
    moving = {'cmd_vel': {'linear_x': .2, 'linear_y': 0, 'angular_z': 0}}
    feed(engine, 7, {'gicp_score': 'DEGRADED'}, moving)
    assert not backend.calls
    feed(engine, 8, {'gicp_score': 'DEGRADED'})
    assert backend.calls[-1][0] == 'publish_initialpose'
    feed(engine, 9)
    assert engine.executor.finished[-1].state == 'SUCCEEDED'
    assert engine.mission_state['status'] == 'CANCELLED'


def test_new_unrelated_task_does_not_abort_running_global_recovery():
    engine, backend = make(profile='nav2')
    fault(engine)
    execution_id = engine.executor.active['gicp'].id
    engine.mission_state = active_mission('capture_observation')
    feed(engine, 7, {'gicp_score': 'DEGRADED'})
    assert engine.executor.active['gicp'].id == execution_id
    assert len(backend.calls) == 2


def test_operator_abort_blocks_automatic_execution_but_keeps_capability_hold():
    engine, backend = make(profile='nav2')
    engine.mission_state = active_mission()
    fault(engine)
    engine.handle_command({'type': 'abort', 'incident_key': 'gicp'}, 6)
    engine.mission_state['status'] = 'CANCELLED'
    feed(engine, 7, {'gicp_score': 'DEGRADED'})
    assert not backend.calls
    assert engine.state()['task_supervision']['constraints']
    engine.handle_command({'type': 'retry', 'incident_key': 'gicp'}, 8)
    assert backend.calls[-1][0] == 'publish_initialpose'


def test_stale_health_cannot_authorize_task_release():
    engine, _ = make(profile='nav2')
    feed(engine, 0)
    assert engine.state()['task_supervision']['ready']
    engine.tick(10)
    assert not engine.state()['task_supervision']['ready']

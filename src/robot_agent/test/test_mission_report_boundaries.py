import json
import pytest
from robot_agent.human_report import HumanReportManager


def send(manager, topic, data):
    return manager.on_topic('/agent/' + topic, json.dumps(data))


def test_heartbeat_never_starts_or_evicts_mission(tmp_path):
    m = HumanReportManager(tmp_path)
    assert send(m, 'mission_events', {'type': 'agent_heartbeat'}) is None
    assert m.current is None
    send(m, 'mission_events', {'type': 'mission_received', 'mission_id': 'm1', 'payload': {'text': 'go back'}})
    for _ in range(200):
        send(m, 'mission_events', {'type': 'agent_heartbeat'})
    assert len(m.current['mission_events']) == 1
    assert m.current['mission']['user_text'] == 'go back'


def test_terminal_report_is_frozen_and_new_mission_is_separate(tmp_path):
    m = HumanReportManager(tmp_path)
    send(m, 'mission_events', {'type': 'mission_received', 'mission_id': 'm1'})
    send(m, 'mission_state', {'mission_id': 'm1', 'status': 'DONE', 'user_text': 'go back'})
    send(m, 'mission_events', {'type': 'mission_done', 'mission_id': 'm1'})
    before = json.dumps(m.current, sort_keys=True)
    send(m, 'recovery_state', {'incidents': [{'id': 'old', 'severity': 'blocking'}]})
    send(m, 'human_prompt', 'old prompt')
    send(m, 'mission_events', {'type': 'agent_heartbeat'})
    assert json.dumps(m.current, sort_keys=True) == before
    send(m, 'mission_events', {'type': 'mission_received', 'mission_id': 'm2'})
    send(m, 'mission_state', {'mission_id': 'm1', 'status': 'NEEDS_HUMAN'})
    assert m.current['mission_id'] == 'm2'
    assert len(list(m.root.glob('*/*.json'))) == 2


def test_cancel_request_keeps_collecting_until_cancelled(tmp_path):
    m = HumanReportManager(tmp_path)
    send(m, 'mission_events', {'type': 'mission_received', 'mission_id': 'm1'})
    m.on_dashboard_command('cancel', {})
    assert m.current['closed_at'] is None
    send(m, 'mission_state', {'mission_id': 'm1', 'status': 'CANCELLED'})
    assert m.current['closed_at']


def test_healthy_ticks_do_not_append_transitions(tmp_path):
    m = HumanReportManager(tmp_path)
    send(m, 'mission_events', {'type': 'mission_received', 'mission_id': 'm1'})
    for i in range(100):
        send(m, 'health_dashboard_state', {'sequence': i, 'domains': []})
    assert len(m.current['health_transitions']) == 1


def test_background_incident_does_not_take_over_mission(tmp_path):
    m = HumanReportManager(tmp_path)
    send(m, 'mission_events', {'type': 'mission_received', 'mission_id': 'm1'})
    send(m, 'mission_state', {'mission_id': 'm1', 'status': 'EXECUTE_STEP'})
    send(m, 'recovery_state', {'incidents': [{'id': 'i1', 'key': 'host'}],
                              'execution': {'operator_requests': [{'id': 'op1', 'incident_key': 'host'}]}})
    assert m.current['status'] == 'EXECUTE_STEP'
    assert not m.current['why_call_human']
    assert m.current['current_situation']['background_incidents']


@pytest.mark.parametrize('event,status', [('mission_done', 'DONE'), ('mission_failed', 'FAILED_SKILL'),
                                         ('mission_cancelled', 'CANCELLED')])
def test_terminal_event_before_state_reconciles_once_without_reopening(tmp_path, event, status):
    m = HumanReportManager(tmp_path)
    send(m, 'mission_state', {'mission_id': 'm1', 'status': 'EXECUTE_STEP'})
    send(m, 'mission_events', {'mission_id': 'm1', 'type': event})
    closed = m.current['closed_at']
    send(m, 'mission_state', {'mission_id': 'm1', 'status': status, 'final_message': 'final evidence'})
    assert m.current['mission']['status'] == status
    assert m.current['mission']['final_message'] == 'final evidence'
    assert m.current['closed_at'] == closed
    before = json.dumps(m.current, sort_keys=True)
    send(m, 'mission_state', {'mission_id': 'm1', 'status': status})
    send(m, 'mission_state', {'mission_id': 'm1', 'status': 'EXECUTE_STEP'})
    assert json.dumps(m.current, sort_keys=True) == before

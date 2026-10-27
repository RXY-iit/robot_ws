from robot_agent.health_response import ScoreBand, annotate_domains, robot_summary
from robot_agent.recovery.snapshot import HealthSnapshot
from test_recovery_engine import make_engine, make_state


def test_score_persistence_hysteresis_and_gap():
    b = ScoreBand()
    for t in range(4):
        state = b.update(.17, t)
    assert state == 'WARN'
    assert b.update(.155, 4) == 'WARN'
    assert b.update(.21, 5) == 'WARN'
    assert b.update(.21, 6) == 'BAD'
    for t in range(7, 18):
        state = b.update(.12, t)
    assert state == 'OK'
    b.update(.17, 18)
    assert b.update(.17, 25) == 'OK'  # missing samples cannot prove persistence
    assert b.update(float('nan'), 26) == 'BAD'


def test_ignore_never_changes_evidence_or_hides_bad():
    domains = [{'id': 'H5', 'items': [{'id':'a', 'status':'WARN'}, {'id':'b', 'status':'ERROR'}]}]
    annotate_domains(domains)
    items = {i['id']:i for i in domains[0]['items']}
    s = robot_summary(items, ['a','b'])
    assert s['level'] == 'BAD'
    assert s['items'][0]['level'] == 'OK'
    assert items['a']['status'] == 'WARN'
    assert not s['items'][1]['ignored']


def test_warn_routes_to_checks_bad_routes_to_pb():
    engine = make_engine()
    for t in [0., 6.]:
        state = make_state({'gicp_score':'WARN'})
        annotate_domains(state['domains'])
        engine.on_snapshot(HealthSnapshot.from_state(state, t))
    assert engine.incidents['gicp'].status == 'WARNING'
    assert engine.incidents['gicp'].diagnosis.playbook_id is None
    engine.handle_command({'type':'ignore', 'incident_key':'gicp'}, 6.)
    assert engine.state()['robot_health_summary']['level'] == 'OK'
    for t in [7., 14.]:
        state = make_state({'gicp_score':'ERROR'})
        annotate_domains(state['domains'])
        engine.on_snapshot(HealthSnapshot.from_state(state, t))
    assert engine.incidents['gicp'].diagnosis.playbook_id == 'PB-LOC-LOST'
    assert engine.state()['robot_health_summary']['level'] == 'BAD'


def test_empty_summary_does_not_claim_health():
    assert robot_summary({})['level'] == 'WARN'


def test_current_clean_start_range_does_not_latch_warning():
    b = ScoreBand()
    for t in range(30):
        assert b.update([.1257,.1362,.1479][t % 3],t) == 'OK'
    for t in range(30,34):
        level = b.update(.17,t)
    assert level == 'WARN'
    for t in range(34,45):
        level = b.update(.149,t)
    assert level == 'OK'

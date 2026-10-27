from robot_agent.sim_health import SimClockEvidence, sensor_timing


def test_rtf_slow_pause_reset_and_recovery():
    c = SimClockEvidence()
    assert c.snapshot(0)[0] == 'UNKNOWN'
    for i in range(11):
        c.update(i * .5, i)
    assert c.snapshot(10)[2]['rtf'] == .5
    assert c.snapshot(10)[0] == 'WARN'
    for i in range(11, 15):
        c.update(5, i)
    assert c.snapshot(14)[2]['rtf'] == 0
    c.update(0, 15)
    assert c.snapshot(15)[2]['reset_count'] == 1
    assert c.snapshot(15)[2]['rtf'] is None
    for i in range(16, 28):
        c.update(i - 15, i)
    assert c.snapshot(27)[0] == 'OK'
    assert c.snapshot(27)[2]['rtf'] == 1
    assert c.snapshot(31)[0] == 'WARN'


def test_sensor_timing_missing_stale_repeated_future():
    assert sensor_timing({}, None)[0] == 'UNKNOWN'
    base = {'receive_age_sec': .1, 'source_stamp_sec': 10, 'source_not_advancing_sec': .1}
    assert sensor_timing(base, 10.1)[0] == 'OK'
    assert sensor_timing({**base, 'source_not_advancing_sec': 3}, 10.1)[0] == 'WARN'
    assert sensor_timing(base, 2)[0] == 'WARN'
    assert sensor_timing({**base, 'receive_age_sec': 11}, 10.1)[0] == 'UNKNOWN'

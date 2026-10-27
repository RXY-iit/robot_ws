from robot_skills.motion_clock import MotionClock


def test_slow_sim_has_longer_wall_budget_without_scaling_velocities():
    clock = MotionClock(30, True, 100, 0)
    for wall in range(1, 50):
        assert not clock.expired(100 + wall * .45, wall)
    assert clock.expired(130, 67)


def test_paused_and_reversed_sim_stop_without_waiting_forever():
    assert MotionClock(30, True, 100, 0).expired(100, 3.1)
    assert MotionClock(30, True, 100, 0).expired(99, .1)


def test_wall_limit_is_independent_of_slow_clock():
    clock = MotionClock(30, True, 100, 0)
    for wall in range(1, 300):
        assert not clock.expired(100 + wall * .01, wall)
    assert clock.expired(103, 300)
    assert clock.reason == 'wall_time_budget_exhausted'


def test_real_budget_stays_wall_time():
    clock = MotionClock(30, False, 100, 0)
    assert not clock.expired(1000, 29)
    assert clock.expired(1000, 30)


def test_low_rtf_does_not_expire_three_times_wall_budget():
    clock = MotionClock(30, True, 100, 0)
    for wall in range(1, 200):
        assert not clock.expired(100 + wall * .137, wall)
    assert 200 < clock.wall_limit <= 300
    assert clock.expired(130, 220)
    assert clock.reason == "simulation_time_budget_exhausted"


def test_budget_extension_does_not_remove_pause_guard():
    clock = MotionClock(30, True, 100, 0)
    for wall in range(1, 100):
        assert not clock.expired(100 + wall * .137, wall)
    assert clock.expired(clock.last_source, 103)
    assert clock.reason == "simulation_clock_stopped"

from robot_skills.executors import camera


def skill(monkeypatch, stamp):
    monkeypatch.setattr(camera.time, "monotonic", lambda: 100.0)
    obj = object.__new__(camera.CameraSkills)
    obj._pan_feedback = 12.0
    obj._tilt_feedback = 23.0
    obj._pan_feedback_time = obj._tilt_feedback_time = stamp
    return obj


def test_equal_but_stale_feedback_is_not_success(monkeypatch):
    assert not skill(monkeypatch, 90)._feedback_at_target(12, 23, 2)


def test_feedback_before_command_cannot_acknowledge_it(monkeypatch):
    assert not skill(monkeypatch, 99.5)._feedback_at_target(12, 23, 2, since=99.7)


def test_fresh_post_command_target_feedback_is_success(monkeypatch):
    assert skill(monkeypatch, 99.9)._feedback_at_target(12, 23, 2, since=99.7)


def test_both_axes_must_have_fresh_feedback(monkeypatch):
    obj = skill(monkeypatch, 99.9)
    obj._tilt_feedback_time = 95
    assert not obj._feedback_at_target(12, 23, 2, since=99.7)

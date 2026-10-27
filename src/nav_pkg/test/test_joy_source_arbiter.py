import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'joy_source_arbiter_node.py'
SPEC = importlib.util.spec_from_file_location('joy_source_arbiter_node', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)
JoySourceArbiterCore = MODULE.JoySourceArbiterCore


def test_physical_is_the_only_default_source():
    core = JoySourceArbiterCore()

    physical_forward, preempted = core.physical_message([0.0], [0], 1.0)

    assert physical_forward is True
    assert preempted is False
    assert core.dashboard_message(1.0) is False


def test_dashboard_requires_explicit_acquire_and_ignores_physical_neutral():
    core = JoySourceArbiterCore()

    acquired, changed = core.acquire_dashboard(1.0)
    physical_forward, preempted = core.physical_message([0.0], [0], 1.1)

    assert acquired is True
    assert changed is True
    assert core.dashboard_message(1.1) is True
    assert physical_forward is False
    assert preempted is False


def test_released_trigger_axes_do_not_block_dashboard_acquire():
    core = JoySourceArbiterCore()

    # Current physical Joy reports axes 2 and 5 as +1.0 while untouched.
    core.physical_message([0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0], [0] * 11, 1.0)
    acquired, _ = core.acquire_dashboard(1.1)

    assert acquired is True
    assert core.source == core.DASHBOARD


def test_pressed_lift_trigger_does_not_preempt_dashboard():
    core = JoySourceArbiterCore()
    core.acquire_dashboard(1.0)

    physical_forward, preempted = core.physical_message(
        [0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0], [0] * 11, 1.1
    )

    assert physical_forward is False
    assert preempted is False
    assert core.source == core.DASHBOARD


def test_used_physical_axis_does_not_preempt_dashboard():
    core = JoySourceArbiterCore()
    core.acquire_dashboard(1.0)

    physical_forward, preempted = core.physical_message(
        [0.0, 0.0, 1.0, 0.2, 0.0, 1.0, 0.0, 0.0], [0] * 11, 1.1
    )

    assert physical_forward is False
    assert preempted is False
    assert core.source == core.DASHBOARD


def test_non_neutral_physical_input_is_ignored_until_dashboard_release():
    core = JoySourceArbiterCore()
    core.acquire_dashboard(1.0)

    physical_forward, preempted = core.physical_message([0.4], [0], 1.1)

    assert physical_forward is False
    assert preempted is False
    assert core.source == core.DASHBOARD
    assert core.dashboard_acquired is True
    assert core.dashboard_message(1.2) is True


def test_dashboard_release_returns_to_physical_source():
    core = JoySourceArbiterCore()
    core.acquire_dashboard(1.0)

    assert core.release_dashboard() is True
    physical_forward, preempted = core.physical_message([0.4], [0], 1.1)

    assert physical_forward is True
    assert preempted is False
    assert core.source == core.PHYSICAL
    assert core.dashboard_message(1.2) is False


def test_dashboard_acquire_wins_even_if_physical_input_is_recent():
    core = JoySourceArbiterCore(physical_active_hold_sec=0.5)
    core.physical_message([0.0], [1], 1.0)

    acquired, _ = core.acquire_dashboard(1.2)

    assert acquired is True
    assert core.source == core.DASHBOARD
    assert 'acquired' in core.reason


def test_dashboard_timeout_returns_to_physical_source():
    core = JoySourceArbiterCore(dashboard_timeout_sec=0.3)
    core.acquire_dashboard(1.0)
    core.dashboard_message(1.1)

    assert core.watchdog(1.39) is False
    assert core.watchdog(1.41) is True
    assert core.source == core.PHYSICAL
    assert core.dashboard_acquired is False


def test_zero_dashboard_timeout_keeps_dashboard_until_release():
    core = JoySourceArbiterCore(dashboard_timeout_sec=0.0)
    core.acquire_dashboard(1.0)
    core.dashboard_message(1.1)

    assert core.watchdog(100.0) is False
    assert core.source == core.DASHBOARD
    assert core.dashboard_acquired is True

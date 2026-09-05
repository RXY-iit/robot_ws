import importlib.util
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / 'serial_transciever'
    / 'lift_control'
    / 'lift_joy_node.py'
)
SPEC = importlib.util.spec_from_file_location('lift_joy_node', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)
lift_command_from_axes = MODULE.lift_command_from_axes


def axes(lt: float, rt: float):
    return [0.0, 0.0, lt, 0.0, 0.0, rt, 0.0, 0.0]


def test_released_triggers_stop():
    assert lift_command_from_axes(axes(1.0, 1.0)) == 'STOP'
    assert lift_command_from_axes(axes(0.0, 0.0)) == 'STOP'


def test_lt_is_up_and_rt_is_down():
    assert lift_command_from_axes(axes(-1.0, 1.0)) == 'UP'
    assert lift_command_from_axes(axes(1.0, -1.0)) == 'DOWN'


def test_both_pressed_is_stop():
    assert lift_command_from_axes(axes(-1.0, -1.0)) == 'STOP'


def test_missing_axes_fail_safe_to_stop():
    assert lift_command_from_axes([]) == 'STOP'

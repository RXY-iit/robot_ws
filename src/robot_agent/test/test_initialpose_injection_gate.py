import importlib.util
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location('initialpose_inject', Path(__file__).resolve().parents[3] / 'tools/sim/inject_localization_initialpose.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def test_explicit_auto_allows_motion():
    m.check_motion_gate('AUTO', .1, .1, [.3, .4], during_navigation=True)
    with pytest.raises(ValueError):
        m.check_motion_gate('AUTO', .1, .1, [.3, .4])


def test_default_requires_stationary_manual():
    m.check_motion_gate('MANUAL', .1, .1, [0, 0])
    with pytest.raises(ValueError):
        m.check_motion_gate('MANUAL', .1, .1, [.2])


@pytest.mark.parametrize('mode,mode_age,odom_age,speeds', [
    ('MANUAL', .1, .1, [.2]), ('AUTO', 3, .1, [.2]),
    ('AUTO', .1, 2, [.2]), ('AUTO', .1, .1, []),
    ('AUTO', .1, .1, [float('nan')])])
def test_auto_still_requires_fresh_finite_evidence(mode,mode_age,odom_age,speeds):
    with pytest.raises(ValueError):
        m.check_motion_gate(mode,mode_age,odom_age,speeds,during_navigation=True)


def test_no_moving_restore():
    with pytest.raises(ValueError):
        m.check_motion_gate('AUTO', .1, .1, [0], during_navigation=True,restore=True)

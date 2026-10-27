import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location("localization_evidence", Path(__file__).resolve().parents[1] / "scripts/localization_evidence.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize("stamp,received,expected", [(99.9, 9.9, True), (83, 9.9, False),
                                                        (101, 9.9, False), (100, 1, False),
                                                        (0, 9.9, False), (float("nan"), 9.9, False)])
def test_hint_requires_source_and_receive_freshness(stamp, received, expected):
    assert module.hint_is_current(100, stamp, received, 10) == expected


def test_fallback_preserves_map_height_in_registration_only():
    prior, odom = np.eye(4), np.eye(4)
    prior[2, 3], odom[2, 3] = -1.5, 1.5
    assert module.matching_seed(prior, odom)[2, 3] == pytest.approx(0)


def test_low_rmse_without_correspondences_is_not_good_localization():
    assert not module.registration_valid(0, 0, 0, np.eye(4))
    assert not module.registration_valid(.1, .01, 100, np.eye(4))
    assert module.registration_valid(.1, .8, 100, np.eye(4))


def test_invalid_geometry_cannot_be_accepted():
    assert not module.registration_valid(float("nan"), 1, 100, np.eye(4))
    transform = np.eye(4)
    transform[0, 0] = float("inf")
    assert not module.registration_valid(.1, 1, 100, transform)


def test_fresh_but_divergent_fast_lio_is_not_a_valid_hint():
    wheel, hint = np.eye(4), np.eye(4)
    hint[:3, 3] = [3790, 6263, 2115]
    assert not module.hint_agrees_with_odometry(hint, wheel)
    hint[:3, 3] = [.1, 0, 0]
    assert module.hint_agrees_with_odometry(hint, wheel)
    hint[0, 0] = hint[1, 1] = -1
    assert not module.hint_agrees_with_odometry(hint, wheel)

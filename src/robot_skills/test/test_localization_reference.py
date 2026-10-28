import pytest
from robot_skills.localization_reference import LocalizationReference


def healthy():
    r=LocalizationReference()
    for t in range(5):
        r.odom((.1*t,0,0),t+1)
        r.observe((10+.1*t,0,0),(.1*t,0,0),t+1,100+t,.12)
    assert r.verified
    return r


def test_no_single_good_sample_and_bad_score_never_refreshes_anchor():
    r=LocalizationReference();r.odom((0,0,0),1)
    assert not r.observe((0,0,0),(0,0,0),1,100,.1)
    r=healthy();a=r.anchor.copy();r.odom((.5,0,0),6)
    assert not r.observe((11,0,0),(.5,0,0),6,105,.21)
    assert r.anchor==a
    assert not r.observe((11,0,0),(.5,0,0),6,106,.12)
    assert r.reason=='reference_disagreement'


def test_clean_consistent_evidence_rebuilds_after_fault():
    r=healthy();r.disturb('noise')
    for t in range(6,11):
        r.odom((.4,0,0),t)
        r.observe((10.4,0,0),(.4,0,0),t,100+t,.12)
    assert r.verified and not r.frozen


@pytest.mark.parametrize('pose,stamp',[((8,0,0),5.1),((.4,0,0),2),((.4,0,0),9),((float('nan'),0,0),6)])
def test_discontinuous_odom_latches_reference_invalid(pose,stamp):
    r=healthy();r.odom(pose,stamp)
    assert r.anchor is None and r.reset_required
    assert r.prediction((.4,0,0),110) is None


def test_total_turn_budget_and_age_not_just_net_distance():
    r=healthy();r.disturb('fault')
    r.odom((.4,0,.6),6);r.odom((.4,0,0),7)
    assert r.prediction((.4,0,0),107) is None
    r=healthy();assert r.prediction((.4,0,0),300) is None


def test_source_lag_does_not_erase_propagation_distance():
    r=healthy()
    r.odom((.5,0,0),6)
    r.observe((10.4,0,0),(.4,0,0),5.1,106,.12)
    # Source time 5.1 precedes odom at 6: most travel already occurred AFTER anchor.
    assert r.metrics(106)['reference_path_m']==pytest.approx(.09)

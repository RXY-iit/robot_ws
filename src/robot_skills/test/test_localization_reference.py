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


def test_new_epoch_requalifies_without_reusing_old_prediction():
    r=healthy();old_epoch=r.epoch
    r.odom((0,0,0),1)  # clock rollback
    assert r.reset_required and r.prediction((0,0,0),110) is None
    for t in range(1,6):
        r.odom((.1*t,0,0),t+1)
        assert not r.observe((20+.1*t,0,0),(.1*t,0,0),t+1,110+t,.14)
    r.odom((.6,0,0),7)
    assert r.observe((20.6,0,0),(.6,0,0),7,117,.14)
    assert r.verified and not r.reset_required and r.epoch>old_epoch
    assert r.metrics(117)['reference_qualification_kind']=='automatic_current_epoch'


@pytest.mark.parametrize('fault',['bad_score','map_jump','odom_jump','settling'])
def test_new_epoch_window_restarts_on_unqualified_evidence(fault):
    r=LocalizationReference();r.invalidate('reset')
    for t in range(1,5):
        r.odom((0,0,0),t);r.observe((5,0,0),(0,0,0),t,100+t,.12)
    r.odom((10 if fault=='odom_jump' else 0,0,0),5)
    r.observe((7 if fault=='map_jump' else 5,0,0),(0,0,0),5,105,
              .21 if fault=='bad_score' else .12,settling=fault=='settling')
    assert not r.verified and r.reset_required
    assert r.prediction((0,0,0),105) is None


def test_frozen_reference_rebuild_requires_stationary_window_then_new_epoch():
    r=healthy();r.disturb('fault');epoch=r.epoch
    for t in [6+i*.5 for i in range(10)]:
        r.odom((.4,0,0),t)
        assert not r.observe((11.4,0,0),(.4,0,0),t,100+t,.12,stationary=True)
    assert r.anchor is not None and r.epoch==epoch
    r.odom((.4,0,0),11)
    assert not r.observe((11.4,0,0),(.4,0,0),11,111,.12,stationary=True)
    assert r.anchor is None and r.reset_required and r.epoch==epoch+1
    assert r.prediction((.4,0,0),111) is None
    for t in range(12,17):
        r.odom((.4,0,0),t)
        assert not r.observe((11.4,0,0),(.4,0,0),t,100+t,.12,stationary=True)
    r.odom((.4,0,0),17)
    assert r.observe((11.4,0,0),(.4,0,0),17,117,.12,stationary=True)
    assert r.verified and not r.reset_required


@pytest.mark.parametrize('fault',['moving','bad_score','map_motion','odom_motion','settling','gap'])
def test_stationary_requalification_cannot_accumulate_through_bad_evidence(fault):
    r=healthy();r.disturb('fault');epoch=r.epoch
    for t in [6+i*.5 for i in range(10)]:
        r.odom((.4,0,0),t)
        r.observe((11.4,0,0),(.4,0,0),t,100+t,.12,stationary=True)
    t=14 if fault=='gap' else 11
    odom=(.5 if fault=='odom_motion' else .4,0,0)
    r.odom(odom,t)
    r.observe((11.5 if fault=='map_motion' else 11.4,0,0),odom,t,100+t,
              .21 if fault=='bad_score' else .12,settling=fault=='settling',stationary=fault!='moving')
    assert not r.verified
    if fault!='gap':assert r.epoch==epoch
    assert r.requalification is None or r.requalification['count']<=1

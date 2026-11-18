import math
import numpy as np
import pytest

from adapters.core.pose_prediction import BoundedPosePredictor


def pose(x=0.0, angle=0.0):
    return np.array([x, 0., 0.]), np.array([0., 0., math.sin(angle/2), math.cos(angle/2)])


def test_fading_prediction_is_bounded_and_holds_after_horizon():
    predictor = BoundedPosePredictor()
    predictor.observe(pose(), 1., 10.)
    predictor.observe(pose(.004, .008), 1.01, 10.01)
    assert predictor.predict(1.03)[0][0] == pytest.approx(.004)
    result = predictor.predict(1.11)
    assert .004 < result[0][0] <= .024
    assert np.linalg.norm(result[1]) == pytest.approx(1.)
    assert np.allclose(result[0], predictor.predict(5.)[0])
    assert np.allclose(result[1], predictor.predict(5.)[1])
    assert predictor.stamp == 1.01  # Prediction never makes input fresh.


def test_burst_delivery_uses_headset_sample_interval_not_receive_interval():
    predictor = BoundedPosePredictor()
    predictor.observe(pose(), 1., 10.)
    predictor.observe(pose(.001), 1.001, 10.01)
    assert predictor.velocity[0] == pytest.approx(.1)


@pytest.mark.parametrize('sample', [10., 9.9])
def test_duplicate_or_backwards_sample_cannot_seed_motion(sample):
    predictor = BoundedPosePredictor()
    predictor.observe(pose(), 1., 10.)
    predictor.observe(pose(.01), 1.01, sample)
    assert np.allclose(predictor.velocity, 0.)


def test_stationary_and_long_outage_cannot_extrapolate():
    predictor = BoundedPosePredictor()
    predictor.observe(pose(), 1., 10.)
    predictor.observe(pose(), 1.01, 10.01)
    assert np.allclose(predictor.predict(1.10)[0], 0.)
    predictor.observe(pose(.02), 1.3, 10.3)
    assert np.allclose(predictor.velocity, 0.)
    predictor.reset()
    assert predictor.predict(2.) is None


def test_late_packet_correction_starts_continuously_and_decays():
    predictor = BoundedPosePredictor()
    predictor.observe(pose(), 1., 10.)
    predictor.observe(pose(.004, .008), 1.01, 10.01)
    before = predictor.predict(1.08)
    predictor.observe(pose(.012, .024), 1.08, 10.08)
    assert np.allclose(predictor.predict(1.08)[0], before[0])
    assert abs(np.dot(predictor.predict(1.08)[1], before[1])) == pytest.approx(1.)
    assert np.linalg.norm(predictor.correction) <= .02
    assert np.linalg.norm(predictor.angular_correction) <= .04


def test_reversal_reconciles_without_unbounded_chasing():
    predictor = BoundedPosePredictor()
    predictor.observe(pose(), 1., 10.)
    predictor.observe(pose(.004), 1.01, 10.01)
    predictor.predict(1.09)
    predictor.observe(pose(-.03), 1.09, 10.09)
    for now in np.arange(1.09, 1.30, .001):
        assert np.linalg.norm(predictor.predict(now)[0] - pose(-.03)[0]) <= .02 + 1e-12


@pytest.mark.parametrize('horizon', [0., .15, float('nan')])
def test_horizon_must_stay_below_150_ms(horizon):
    predictor = BoundedPosePredictor()
    predictor.observe(pose(), 1., 10.)
    with pytest.raises(ValueError):
        predictor.predict(1.1, horizon=horizon)

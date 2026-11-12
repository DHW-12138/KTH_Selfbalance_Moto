import pytest
from iim42652_gyro.calibrate import summarize


def samples(rate, count=6000):
    return [(i/100, rate) for i in range(count)]


def test_static_bias_and_variance():
    result = summarize(samples((0.001, -0.002, 0.003)), 60, 100, 0.01, 0.1)
    assert result['accepted']
    assert result['bias_rad_s'] == pytest.approx((0.001, -0.002, 0.003))
    assert result['std_rad_s'] == pytest.approx((0, 0, 0))


def test_movement_rejected():
    data = [(i/100, (0.2 if i%2 else -0.2, 0, 0)) for i in range(6000)]
    assert not summarize(data, 60, 100, 0.01, 0.1)['accepted']


def test_missing_data_and_warmup_drift_rejected():
    assert not summarize(samples((0, 0, 0), 20), 60, 100, 0.01, 0.1)['accepted']
    data = [(i/100, (0 if i<3000 else 0.03, 0, 0)) for i in range(6000)]
    assert not summarize(data, 60, 100, 0.01, 0.1)['accepted']
    with pytest.raises(ValueError):
        summarize([], 60, 100, 0.01, 0.1)

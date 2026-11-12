import math
import pytest
from iim42652_gyro.integration import OrientationIntegrator


def test_constant_z_rotation():
    integrator = OrientationIntegrator()
    for i in range(101):
        q, skipped = integrator.update((0, 0, math.pi/2), i/100)
        assert not skipped
    assert q == pytest.approx((0, 0, math.sin(math.pi/4), math.cos(math.pi/4)))


def test_body_rotation_composition():
    integrator = OrientationIntegrator(max_dt=2)
    integrator.update((math.pi/2, 0, 0), 0)
    integrator.update((math.pi/2, 0, 0), 1)
    # Begin a separate interval with no interpolation across the change of axis.
    integrator.previous = (1, (0, math.pi/2, 0))
    q, _ = integrator.update((0, math.pi/2, 0), 2)
    assert q == pytest.approx((0.5, 0.5, 0.5, 0.5))


def test_gap_and_reset():
    integrator = OrientationIntegrator()
    integrator.update((0, 0, 1), 0)
    q, skipped = integrator.update((0, 0, 1), 1)
    assert skipped and q == (0, 0, 0, 1)
    q, skipped = integrator.update((0, 0, 1), 1.01)
    assert not skipped
    assert q[2] == pytest.approx(math.sin(0.005))
    integrator.reset()
    assert integrator.update((1, 2, 3), 20)[0] == (0, 0, 0, 1)


def test_negative_rotation_normalization():
    integrator = OrientationIntegrator()
    for i in range(10001):
        q, _ = integrator.update((0, 0, -1), i/100)
    assert sum(v*v for v in q) == pytest.approx(1)
    assert q[2] == pytest.approx(math.sin(-50))


def test_bad_dt_limit():
    for value in (0, -1, float('nan')):
        with pytest.raises(ValueError):
            OrientationIntegrator(value)


def test_rpy_single_axes():
    from iim42652_gyro.integration import quaternion_to_rpy
    assert quaternion_to_rpy((0, 0, 0, 1)) == pytest.approx((0, 0, 0))
    for axis in range(3):
        angle = -0.7
        q = [0.0, 0.0, 0.0, math.cos(angle/2)]
        q[axis] = math.sin(angle/2)
        expected = [0, 0, 0]
        expected[axis] = angle
        assert quaternion_to_rpy(q) == pytest.approx(expected)


def test_rpy_combined_normalization_and_wrap():
    from iim42652_gyro.integration import quaternion_to_rpy
    # Rz(90) Ry(0) Rx(90), scaled to test normalization.
    assert quaternion_to_rpy((1, 1, 1, 1)) == pytest.approx((math.pi/2, 0, math.pi/2))
    yaw = math.radians(190)
    q = (0, 0, math.sin(yaw/2), math.cos(yaw/2))
    assert quaternion_to_rpy(q)[2] == pytest.approx(math.radians(-170))
    pitch90 = (0, math.sin(math.pi/4), 0, math.cos(math.pi/4))
    assert quaternion_to_rpy(pitch90)[1] == pytest.approx(math.pi/2)
    with pytest.raises(ValueError):
        quaternion_to_rpy((0, 0, 0, 0))

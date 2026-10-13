import numpy as np
import pytest

from pbs.eval.metrics import ade, fde
from pbs.prediction import ConstantVelocityPredictor, StaticPredictor
from pbs.types import Trajectory


def test_static_predictor_shape():
    t = np.arange(5) * 0.1
    h = Trajectory(t, t, np.zeros(5), np.ones(5), np.zeros(5))
    out = StaticPredictor().predict(h, horizon=20, dt=0.1)
    assert out.shape == (20, 2)


def test_cv_predictor_straight_line():
    t = np.arange(5) * 0.1
    h = Trajectory(
        t, 3 + 8 * t, -2 + 3 * t, np.full(5, np.hypot(8, 3)), np.full(5, np.arctan2(3, 8))
    )
    out = ConstantVelocityPredictor().predict(h, horizon=20, dt=0.1)
    future = t[-1] + np.arange(1, 21) * 0.1
    truth = np.column_stack((3 + 8 * future, -2 + 3 * future))
    np.testing.assert_allclose(out, truth, atol=1e-12)
    assert ade(out, truth) < 1e-12
    assert fde(out, truth) < 1e-12


def test_cv_uses_observed_lateral_velocity_with_irregular_sampling():
    # Deliberately stale speed/heading metadata must not erase observed lateral movement.
    t = np.array([1.0, 1.15, 1.7])
    h = Trajectory(t, 10 * t, 2 - 2 * t, np.zeros(3), np.zeros(3))
    out = ConstantVelocityPredictor().predict(h, 3, 0.2)
    future = np.array([1.9, 2.1, 2.3])
    np.testing.assert_allclose(out, np.column_stack((10 * future, 2 - 2 * future)))


@pytest.mark.parametrize("heading", [0.0, np.pi / 2, -np.pi / 2, np.pi])
def test_cv_single_observation_uses_speed_and_heading(heading):
    h = Trajectory(
        np.array([0.0]), np.array([3.0]), np.array([-1.0]), np.array([4.0]), np.array([heading])
    )
    expected = np.array([3.0, -1.0]) + np.array([0.5, 1.0])[:, None] * 4 * np.array(
        [np.cos(heading), np.sin(heading)]
    )
    np.testing.assert_allclose(ConstantVelocityPredictor().predict(h, 2, 0.5), expected)


def test_cv_stationary_agent_and_zero_horizon():
    h = Trajectory(np.array([0.0, 0.1]), np.ones(2), np.full(2, 2.0), np.zeros(2), np.zeros(2))
    predictor = ConstantVelocityPredictor()
    np.testing.assert_array_equal(predictor.predict(h, 3, 0.1), [[1, 2], [1, 2], [1, 2]])
    assert predictor.predict(h, 0, 0.1).shape == (0, 2)


def test_cv_does_not_mutate_history():
    t = np.array([0.0, 0.1, 0.2])
    h = Trajectory(t, 8 * t, -t, np.full(3, np.hypot(8, 1)), np.full(3, np.arctan2(-1, 8)))
    before = [a.copy() for a in (h.t, h.x, h.y, h.v, h.heading)]
    ConstantVelocityPredictor().predict(h, 5, 0.1)
    for current, original in zip((h.t, h.x, h.y, h.v, h.heading), before, strict=True):
        np.testing.assert_array_equal(current, original)


@pytest.mark.skip(reason="TODO(P): CTRV with yaw_rate≈0 equals CV")
def test_ctrv_degenerates_to_cv(): ...


@pytest.mark.skip(reason="TODO(P): Oracle returns true future")
def test_oracle_exact(): ...

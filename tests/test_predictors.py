import numpy as np
import pytest

from pbs.eval.metrics import ade, fde
from pbs.prediction import (
    ConstantAccelPredictor,
    ConstantVelocityPredictor,
    CTRVPredictor,
    OraclePredictor,
    StaticPredictor,
)
from pbs.types import Scenario, Trajectory


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


def test_ctrv_degenerates_to_cv():
    t = np.arange(5) * 0.1
    history = Trajectory(
        t, 3 + 8 * t, -2 + 3 * t, np.full(5, np.hypot(8, 3)), np.full(5, np.arctan2(3, 8))
    )
    np.testing.assert_allclose(
        CTRVPredictor().predict(history, horizon=20, dt=0.1),
        ConstantVelocityPredictor().predict(history, horizon=20, dt=0.1),
        atol=1e-12,
    )


def test_ctrv_turn_is_not_cv():
    dt = 0.1
    yaw_rate = 0.5
    speed = 6.0
    t = np.arange(8) * dt
    heading = yaw_rate * t
    history = Trajectory(
        t,
        speed / yaw_rate * np.sin(heading),
        speed / yaw_rate * (1 - np.cos(heading)),
        np.full_like(t, speed),
        heading,
    )
    turned = CTRVPredictor().predict(history, horizon=6, dt=dt)
    straight = ConstantVelocityPredictor().predict(history, horizon=6, dt=dt)
    assert turned.shape == (6, 2)
    assert np.isfinite(turned).all()
    assert not np.allclose(turned, straight, atol=1e-6)


def test_ca_matches_constant_acceleration():
    dt = 0.1
    t = np.arange(8) * dt
    x0, y0, vx, vy, ax, ay = 1.0, -2.0, 4.0, -1.0, 0.5, 0.25
    history = Trajectory(
        t,
        x0 + vx * t + 0.5 * ax * t**2,
        y0 + vy * t + 0.5 * ay * t**2,
        np.hypot(vx + ax * t, vy + ay * t),
        np.arctan2(vy + ay * t, vx + ax * t),
    )
    out = ConstantAccelPredictor().predict(history, horizon=5, dt=dt)
    future = t[-1] + np.arange(1, 6) * dt
    truth = np.column_stack(
        (x0 + vx * future + 0.5 * ax * future**2, y0 + vy * future + 0.5 * ay * future**2)
    )
    np.testing.assert_allclose(out, truth, atol=1e-9)


def test_ca_two_samples_matches_cv():
    t = np.arange(2) * 0.1
    history = Trajectory(
        t, 3 + 8 * t, -2 + 3 * t, np.full(2, np.hypot(8, 3)), np.full(2, np.arctan2(3, 8))
    )
    np.testing.assert_allclose(
        ConstantAccelPredictor().predict(history, 5, 0.1),
        ConstantVelocityPredictor().predict(history, 5, 0.1),
        atol=1e-12,
    )


def test_oracle_exact():
    t = np.arange(0, 4.01, 0.1)
    agent = Trajectory(
        t,
        2 + 3 * t,
        -1 + 0.5 * t,
        np.full_like(t, np.hypot(3, 0.5)),
        np.full_like(t, np.arctan2(0.5, 3)),
        agent_id="a",
    )
    scenario = Scenario(
        ego_start=(0.0, 0.0, 0.0, 1.0),
        ego_path=np.array([[0.0, 0.0], [100.0, 0.0]]),
        agents=[agent],
    )
    predictor = OraclePredictor()
    predictor.bind(scenario)
    prefix = Trajectory(
        agent.t[:11], agent.x[:11], agent.y[:11], agent.v[:11], agent.heading[:11], agent_id="a"
    )
    out = predictor.predict(prefix, horizon=10, dt=0.1)
    times = prefix.t[-1] + np.arange(1, 11) * 0.1
    truth = np.column_stack((2 + 3 * times, -1 + 0.5 * times))
    np.testing.assert_allclose(out, truth, atol=1e-12)

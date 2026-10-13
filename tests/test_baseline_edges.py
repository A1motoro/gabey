"""Round 3: contract boundaries and deterministic randomized L1 properties."""

import numpy as np
import pytest

from pbs.prediction import ConstantVelocityPredictor
from pbs.risk import ReactiveAssessor
from pbs.risk.ttc import assess_risk
from tests.test_risk import context, history


@pytest.mark.parametrize("horizon", [-1, 1.5, True])
def test_invalid_prediction_horizon(horizon):
    with pytest.raises(ValueError, match="horizon"):
        ConstantVelocityPredictor().predict(history(), horizon, 0.1)


@pytest.mark.parametrize("dt", [0, -0.1, np.nan, np.inf])
def test_invalid_prediction_step(dt):
    with pytest.raises(ValueError, match="dt"):
        ConstantVelocityPredictor().predict(history(), 3, dt)


@pytest.mark.parametrize("problem", ["empty", "duplicate", "descending", "length", "nan"])
def test_invalid_history(problem):
    h = history()
    if problem == "empty":
        h.t = np.array([])
    elif problem == "duplicate":
        h.t[:] = 0
    elif problem == "descending":
        h.t = h.t[::-1]
    elif problem == "length":
        h.y = h.y[:1]
    else:
        h.x[0] = np.nan
    with pytest.raises(ValueError):
        ConstantVelocityPredictor().predict(h, 3, 0.1)


@pytest.mark.parametrize(
    "ego,agent",
    [
        (np.zeros(3), np.zeros(3)),
        (np.zeros((3, 2)), np.zeros((2, 2))),
        (np.zeros((2, 3)), np.zeros((2, 3))),
        (np.full((2, 2), np.inf), np.zeros((2, 2))),
    ],
)
def test_invalid_future_positions(ego, agent):
    with pytest.raises(ValueError):
        assess_risk(ego, agent, 0.1, 2.5)


@pytest.mark.parametrize("dt,radius", [(0, 2.5), (np.nan, 2.5), (0.1, -1), (0.1, np.inf)])
def test_invalid_ttc_parameters(dt, radius):
    with pytest.raises(ValueError):
        assess_risk(np.zeros((2, 2)), np.zeros((2, 2)), dt, radius)


def test_strict_radius_and_zero_radius():
    ego = np.zeros((1, 2))
    assert np.isinf(assess_risk(ego, np.array([[2.5, 0]]), 0.1, 2.5).ttc)
    assert np.isinf(assess_risk(ego, ego, 0.1, 0).ttc)
    assert assess_risk(ego, np.array([[2.499, 0]]), 0.1, 2.5).ttc == 0.1


@pytest.mark.parametrize("seed", range(20))
def test_random_cv_exact_for_linear_motion(seed):
    rng = np.random.default_rng(seed)
    velocity = rng.uniform(-20, 20, 2)
    origin = rng.uniform(-100, 100, 2)
    t = np.cumsum(rng.uniform(0.01, 0.5, 6))
    xy = origin + t[:, None] * velocity
    h = history()
    h.t, h.x, h.y = t, xy[:, 0], xy[:, 1]
    h.v = np.full(len(t), np.linalg.norm(velocity))
    h.heading = np.full(len(t), np.arctan2(velocity[1], velocity[0]))
    dt = float(rng.uniform(0.02, 0.2))
    out = ConstantVelocityPredictor().predict(h, 40, dt)
    expected = origin + (t[-1] + np.arange(1, 41) * dt)[:, None] * velocity
    np.testing.assert_allclose(out, expected, atol=1e-10)


@pytest.mark.parametrize("seed", range(20))
def test_ttc_invariant_under_rigid_transformation_and_time_scaling(seed):
    rng = np.random.default_rng(seed)
    ego = rng.normal(size=(30, 2))
    agent = ego + rng.normal(size=(30, 2)) * 5
    angle = rng.uniform(-np.pi, np.pi)
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    translation = rng.uniform(-100, 100, 2)
    original = assess_risk(ego, agent, 0.1, 2.5)
    moved = assess_risk(ego @ rotation + translation, agent @ rotation + translation, 0.2, 2.5)
    assert moved.conflict_step == original.conflict_step
    assert moved.ttc == pytest.approx(original.ttc * 2)


@pytest.mark.parametrize("angle", [0.0, 0.4, np.pi / 2, np.pi])
def test_reactive_ttc_invariant_under_lane_rotation(angle):
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    ctx = context()
    ctx.ego_path = ctx.ego_path @ rotation
    ctx.ego_xy = ctx.ego_xy @ rotation
    h = history()
    positions = np.column_stack((h.x, h.y)) @ rotation
    h.x, h.y = positions[:, 0], positions[:, 1]
    h.heading[:] = -angle
    assert ReactiveAssessor().assess(ctx, h).ttc == pytest.approx(3.5)

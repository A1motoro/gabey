"""L1 TTC and Reactive/CV distinction, including observation-time alignment."""

import numpy as np
import pytest

from pbs.prediction import ConstantVelocityPredictor
from pbs.risk import PredictiveAssessor, ReactiveAssessor
from pbs.risk.ttc import assess_risk
from pbs.types import RiskContext, Trajectory


def context(ego_v=10.0, horizon=30, dt=0.1, t_now=0.0, ego_xy=None, path=None):
    ego_xy = np.array([0.0, 0.0]) if ego_xy is None else np.asarray(ego_xy)
    path = np.array([[0.0, 0.0], [500.0, 0.0]]) if path is None else np.asarray(path)
    future = ego_xy + np.column_stack((np.arange(1, horizon + 1) * dt * ego_v, np.zeros(horizon)))
    return RiskContext(t_now, ego_xy, 0.0, ego_v, future, path, 3.5, horizon, dt, 2.5)


def history(x=20.0, y=0.0, vx=5.0, vy=0.0, t_last=0.0):
    t = np.array([t_last - 0.1, t_last])
    xy = np.array([x, y]) + (t - t_last)[:, None] * np.array([vx, vy])
    return Trajectory(
        t,
        xy[:, 0],
        xy[:, 1],
        np.full(2, np.hypot(vx, vy)),
        np.full(2, np.arctan2(vy, vx)),
        agent_id="agent",
    )


def test_assess_risk_head_on():
    times = np.arange(1, 41) * 0.1
    ego = np.column_stack((10 * times, np.zeros(40)))
    agent = np.column_stack((17 + 5 * times, np.zeros(40)))
    risk = assess_risk(ego, agent, dt=0.1, r_safe=2.0)
    # 3.0 s is exactly on the boundary; strict centre distance < r_safe means 3.1 s.
    assert risk.ttc == pytest.approx(3.1)
    assert risk.conflict_step == 30
    assert risk.considered


def test_assess_risk_first_conflict_not_closest_approach():
    risk = assess_risk(np.zeros((3, 2)), np.array([[3.0, 0], [2.0, 0], [0, 0]]), 0.2, 2.5)
    assert risk.conflict_step == 1
    assert risk.ttc == pytest.approx(0.4)


def test_assess_risk_no_conflict_and_empty_horizon():
    for ego, agent in [
        (np.zeros((3, 2)), np.full((3, 2), 20.0)),
        (np.empty((0, 2)), np.empty((0, 2))),
    ]:
        risk = assess_risk(ego, agent, 0.1, 2.5)
        assert np.isinf(risk.ttc)
        assert risk.conflict_step is None


def test_reactive_ignores_adjacent_lane():
    risk = ReactiveAssessor().assess(context(), history(y=3.5))
    assert not risk.considered
    assert np.isinf(risk.ttc)


def test_cv_sees_cut_in_before_reactive():
    ctx = context(ego_v=15.0)
    cutter = history(x=15, y=3.5, vx=10, vy=-1.5)
    reactive = ReactiveAssessor().assess(ctx, cutter)
    predictive = PredictiveAssessor(ConstantVelocityPredictor()).assess(ctx, cutter)
    assert not reactive.considered
    assert np.isfinite(predictive.ttc)
    assert predictive.conflict_step is not None


def test_reactive_head_on_continuous_ttc():
    risk = ReactiveAssessor().assess(context(), history(x=30))
    assert risk.ttc == pytest.approx((30 - 2.5) / 5)
    assert risk.conflict_step is None
    assert risk.considered


@pytest.mark.parametrize("x,vx", [(-10, 5), (20, 10), (20, 15)])
def test_reactive_ignores_behind_or_nonclosing_agents(x, vx):
    assert np.isinf(ReactiveAssessor().assess(context(), history(x=x, vx=vx)).ttc)


def test_reactive_current_overlap_is_immediate_even_when_opening():
    assert ReactiveAssessor().assess(context(), history(x=1, vx=20)).ttc == 0


def test_reactive_includes_lane_boundary_but_ignores_lateral_velocity():
    risk = ReactiveAssessor().assess(context(), history(y=1.75, vy=-4))
    assert risk.considered
    assert risk.ttc == pytest.approx(3.5)


def test_reactive_uses_path_frame_on_rotated_lane():
    ctx = context(path=[[0, 0], [0, 100]])
    ctx.ego_heading = np.pi / 2
    risk = ReactiveAssessor().assess(ctx, history(x=0, y=20, vx=0, vy=5))
    assert risk.ttc == pytest.approx(3.5)
    assert not ReactiveAssessor().assess(ctx, history(x=3.5, y=20, vx=0, vy=5)).considered


def test_reactive_projects_curved_lane_and_repeated_vertices():
    ctx = context(ego_xy=[5, 0], path=[[0, 0], [10, 0], [10, 0], [10, 20]])
    # Remaining arc length is 5+10, rather than world-x distance of 5.
    risk = ReactiveAssessor().assess(ctx, history(x=10, y=10, vx=0, vy=5))
    assert risk.ttc == pytest.approx((15 - 2.5) / 5)


def test_reactive_extends_path_endpoints():
    ctx = context(path=[[0, 0], [10, 0]])
    assert ReactiveAssessor().assess(ctx, history(x=20)).ttc == pytest.approx(3.5)


@pytest.mark.parametrize("age", [0.0, 0.3, 0.35])
def test_predictive_aligns_delayed_observations(age):
    ctx = context(t_now=1.0)
    current = history(x=12, t_last=1.0)
    delayed = history(x=12 - 5 * age, t_last=1.0 - age)
    assessor = PredictiveAssessor(ConstantVelocityPredictor())
    expected = assessor.assess(ctx, current)
    actual = assessor.assess(ctx, delayed)
    assert actual.ttc == pytest.approx(expected.ttc)
    assert actual.conflict_step == expected.conflict_step


def test_predictive_rejects_future_observations():
    with pytest.raises(ValueError, match="newer"):
        PredictiveAssessor(ConstantVelocityPredictor()).assess(context(), history(t_last=1.0))

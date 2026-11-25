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


def test_assess_risk_swept_catches_substep_crossing():
    # Ego holds at (0, 0) heading +x; fast perpendicular agent jumps from y=-6 to y=+6 in dt=0.1s.
    # At both discrete endpoints |y|=6 > 2.5+1.25, so discrete check misses it,
    # whereas swept check catches contact around y=-3.75 (fraction = (6 - 3.75) / 12).
    ego_origin = np.array([0.0, 0.0])
    agent_origin = np.array([0.0, -6.0])
    ego_future = np.array([[0.0, 0.0], [0.0, 0.0]])
    agent_future = np.array([[0.0, 6.0], [0.0, 18.0]])
    discrete = assess_risk(
        ego_future,
        agent_future,
        dt=0.1,
        ego_heading=0.0,
        agent_heading=np.pi / 2,
        swept=False,
    )
    swept = assess_risk(
        ego_future,
        agent_future,
        dt=0.1,
        ego_heading=0.0,
        agent_heading=np.pi / 2,
        ego_origin=ego_origin,
        agent_origin=agent_origin,
        ego_origin_heading=0.0,
        agent_origin_heading=np.pi / 2,
        swept=True,
    )
    assert np.isinf(discrete.ttc)
    assert swept.conflict_step == 0
    assert swept.ttc == pytest.approx(0.1 * (6.0 - 3.75) / 12.0, abs=1e-6)


def test_predictive_assessor_l2_uses_swept_ttc():
    # In L2 OBB mode (r_safe=None), PredictiveAssessor sweeps each forecast interval.
    ctx = context(ego_v=0.0, horizon=5, dt=0.1)
    ctx.r_safe = None
    ctx.ego_future_heading = np.zeros(5)
    # Moving at vy=120 m/s crosses ego at y=0 in the middle of step 0 without overlapping endpoints.
    cutter = history(x=0.0, y=-6.0, vx=0.0, vy=120.0, t_last=0.0)
    est = PredictiveAssessor(ConstantVelocityPredictor()).assess(ctx, cutter)
    assert est.conflict_step == 0
    assert est.ttc == pytest.approx(0.1 * (6.0 - 3.75) / 12.0, abs=1e-6)


def test_predictive_assessor_l1_circle_stays_discrete():
    # Explicit r_safe selects the legacy L1 discrete circle check (no substep sweep).
    ctx = context(ego_v=0.0, horizon=5, dt=0.1)
    ctx.r_safe = 2.5
    cutter = history(x=0.0, y=-6.0, vx=0.0, vy=120.0, t_last=0.0)
    est = PredictiveAssessor(ConstantVelocityPredictor()).assess(ctx, cutter)
    assert np.isinf(est.ttc)
    assert est.conflict_step is None


def test_assess_risk_swept_requires_origins():
    ego = np.array([[0.0, 0.0], [1.0, 0.0]])
    agent = np.array([[0.0, 5.0], [0.0, 4.0]])
    with pytest.raises(ValueError, match="ego_origin and agent_origin"):
        assess_risk(ego, agent, dt=0.1, swept=True)


def test_assess_risk_vehicle_width_passthrough_and_narrow_clearance():
    # Two parallel cars separated laterally by 2.2 m:
    # - L2 width 2.5 m -> combined half-width 2.5 m > 2.2 m -> overlap (collision).
    # - L3 width 2.0 m -> combined half-width 2.0 m < 2.2 m -> no overlap.
    ego = np.array([[10.0, 0.0], [11.0, 0.0]])
    agent = np.array([[10.0, 2.2], [11.0, 2.2]])
    l2 = assess_risk(ego, agent, dt=0.1, ego_heading=0.0, agent_heading=0.0, vehicle_width=2.5)
    l3 = assess_risk(ego, agent, dt=0.1, ego_heading=0.0, agent_heading=0.0, vehicle_width=2.0)
    assert l2.conflict_step == 0 and np.isfinite(l2.ttc)
    assert l3.conflict_step is None and np.isinf(l3.ttc)

    # Swept mode respects ctx.vehicle_width=2.0 on PredictiveAssessor
    ctx_l2 = context(ego_v=10.0, horizon=5, dt=0.1)
    ctx_l2.r_safe = None
    ctx_l2.ego_future_heading = np.zeros(5)
    ctx_l2.vehicle_width = 2.5

    ctx_l3 = context(ego_v=10.0, horizon=5, dt=0.1)
    ctx_l3.r_safe = None
    ctx_l3.ego_future_heading = np.zeros(5)
    ctx_l3.vehicle_width = 2.0

    parallel = history(x=5.0, y=2.2, vx=5.0, vy=0.0)
    assessor = PredictiveAssessor(ConstantVelocityPredictor())
    assert np.isfinite(assessor.assess(ctx_l2, parallel).ttc)
    assert np.isinf(assessor.assess(ctx_l3, parallel).ttc)


def test_reactive_assessor_reads_ctx_vehicle_dimensions():
    ctx_narrow = context(ego_v=10.0)
    ctx_narrow.r_safe = None
    ctx_narrow.vehicle_length = 4.0
    ctx_narrow.vehicle_width = 2.0

    ctx_wide = context(ego_v=10.0)
    ctx_wide.r_safe = None
    ctx_wide.vehicle_length = 5.0
    ctx_wide.vehicle_width = 2.5

    # Agent rotated 90 deg ahead in ego lane so its lateral extent uses vehicle_width/2.
    h = Trajectory(
        np.array([-0.1, 0.0]),
        np.array([20.0, 20.0]),
        np.array([0.0, 0.0]),
        np.array([0.0, 0.0]),
        np.array([np.pi / 2, np.pi / 2]),
    )
    r_narrow = ReactiveAssessor().assess(ctx_narrow, h)
    r_wide = ReactiveAssessor().assess(ctx_wide, h)
    assert r_narrow.ttc == pytest.approx((20.0 - (2.0 + 1.0)) / 10.0)
    assert r_wide.ttc == pytest.approx((20.0 - (2.5 + 1.25)) / 10.0)


def test_uncertainty_aware_swept_ttc_inflation_cap_and_delay():
    class _FixedStdPredictor(ConstantVelocityPredictor):
        def __init__(self, std_lon: float, std_lat: float):
            self._std = (std_lon, std_lat)
            self.latest_std = None

        def predict(self, hist: Trajectory, horizon: int, dt: float) -> np.ndarray:
            out = super().predict(hist, horizon, dt)
            self.latest_std = np.tile(self._std, (horizon, 1))
            return out

    # Parallel car at y=2.4 m with L3 width 2.0 m: clearance is 0.4 m per side.
    # - kappa=0.0 -> no overlap (TTC=inf)
    # - kappa=1.0 with sigma_lat=0.5 m -> single-side inflation 0.5 m > 0.4 m -> finite TTC!
    ctx = context(ego_v=10.0, horizon=10, dt=0.1)
    ctx.r_safe = None
    ctx.ego_future_heading = np.zeros(10)
    ctx.vehicle_width = 2.0
    h = history(x=5.0, y=2.4, vx=5.0, vy=0.0)

    r0 = PredictiveAssessor(_FixedStdPredictor(0.2, 0.5), uncertainty_scale=0.0).assess(ctx, h)
    r1 = PredictiveAssessor(_FixedStdPredictor(0.2, 0.5), uncertainty_scale=1.0).assess(ctx, h)
    assert np.isinf(r0.ttc)
    assert np.isfinite(r1.ttc)
    assert r1.ttc <= r0.ttc

    # Single-side inflation must be capped at 1.5 m (total agent_width <= 2.0 + 3.0 = 5.0 m ->
    # combined half-width = 1.0 + 2.5 = 3.5 m, so y=3.6 m stays collision-free even for huge sigma).
    h_far = history(x=5.0, y=3.6, vx=5.0, vy=0.0)
    r_capped = PredictiveAssessor(_FixedStdPredictor(10.0, 10.0), uncertainty_scale=2.0).assess(
        ctx, h_far
    )
    assert np.isinf(r_capped.ttc)

    # Delayed observation aligns latest_std without shape error
    ctx_delayed = context(ego_v=10.0, horizon=10, dt=0.1, t_now=0.25)
    ctx_delayed.r_safe = None
    ctx_delayed.ego_future_heading = np.zeros(10)
    ctx_delayed.vehicle_width = 2.0
    h_old = history(x=3.75, y=2.4, vx=5.0, vy=0.0, t_last=0.0)
    r_del = PredictiveAssessor(_FixedStdPredictor(0.2, 0.5), uncertainty_scale=1.0).assess(
        ctx_delayed, h_old
    )
    assert np.isfinite(r_del.ttc)


def test_assess_risk_padding_pedestrian_circle_and_validation():
    # Pedestrian padding inflates effective disk radius
    ego = np.array([[0.0, 0.0], [1.0, 0.0]])
    # Ego half-width=1.0, pedestrian radius=1.25 -> edge at y=2.25;
    # y=2.5 misses without pad, hits with pad=1.0.
    ped = np.array([[0.0, 2.5], [1.0, 2.5]])
    r_no_pad = assess_risk(
        ego, ped, dt=0.1, ego_heading=0.0, agent_heading=0.0, pedestrian=True, vehicle_width=2.0
    )
    r_pad = assess_risk(
        ego,
        ped,
        dt=0.1,
        ego_heading=0.0,
        agent_heading=0.0,
        pedestrian=True,
        vehicle_width=2.0,
        agent_width_pad=1.0,
    )
    assert np.isinf(r_no_pad.ttc) and r_pad.conflict_step == 0

    # Validation rejects negative / NaN / wrong-shape pads and invalid uncertainty_scale
    with pytest.raises(ValueError, match="agent_length_pad"):
        assess_risk(ego, ped, dt=0.1, agent_length_pad=-0.1)
    with pytest.raises(ValueError, match="agent_width_pad"):
        assess_risk(ego, ped, dt=0.1, agent_width_pad=np.array([0.1, 0.2, 0.3]))
    with pytest.raises(ValueError, match="vehicle_width"):
        assess_risk(ego, ped, dt=0.1, vehicle_width=0.0)
    with pytest.raises(ValueError, match="vehicle_width"):
        assess_risk(ego, ped, dt=0.1, vehicle_width=np.bool_(True))
    with pytest.raises(ValueError, match="uncertainty_scale"):
        PredictiveAssessor(ConstantVelocityPredictor(), uncertainty_scale=-0.5)
    with pytest.raises(ValueError, match="uncertainty_scale"):
        PredictiveAssessor(ConstantVelocityPredictor(), uncertainty_scale=np.bool_(True))
    ctx_none_w = context(ego_v=10.0)
    ctx_none_w.vehicle_width = None
    with pytest.raises(ValueError, match="Vehicle dimensions"):
        ReactiveAssessor().assess(ctx_none_w, history())

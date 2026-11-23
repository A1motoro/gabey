"""C-only verification: these assessors are test doubles, not P's algorithms."""

import numpy as np
import pytest

from pbs.eval.metrics import evaluate
from pbs.risk.assessors import NullAssessor, RiskAssessor
from pbs.sim.policy import (
    BRAKE,
    CRUISE,
    HARD_BRAKE,
    RECOVER,
    HysteresisPolicy,
    NeverBrakePolicy,
    StagedBrakingPolicy,
    TTCThresholdPolicy,
)
from pbs.sim.simulator import MissingObservationError, ego_rollout, run_sim
from pbs.systems import make_system
from pbs.types import RiskEstimate, Scenario, SimLog, Trajectory
from pbs.utils import history_until, path_heading, path_point


class FixedRisk(RiskAssessor):
    def __init__(self, ttc=0.5):
        self.ttc = ttc
        self.contexts = []

    def bind(self, scenario):
        self.contexts = []

    def assess(self, ctx, history):
        self.contexts.append(ctx)
        return RiskEstimate(ttc=self.ttc)


def scene(speed=15.0, dt=0.1, history_start=-1.0, end=10.0):
    times = np.array([history_start, end])
    lead = Trajectory(
        t=times,
        x=np.full(2, 100.0),
        y=np.full(2, 20.0),
        v=np.zeros(2),
        heading=np.zeros(2),
        agent_id="lead",
    )
    return Scenario(
        ego_start=(0, 0, 0, speed),
        ego_path=np.array([[0.0, 0.0], [1000.0, 0.0]]),
        agents=[lead],
        dt=dt,
    )


def test_empty_scene_cruises_and_log_lengths():
    scenario = scene()
    scenario.agents = []
    log = run_sim(scenario, NullAssessor(), HysteresisPolicy(), duration=1)
    assert np.allclose(log.ego_x, 15 * log.t)
    assert np.all(log.ego_v == 15)
    assert log.action == [CRUISE] * 11
    assert log.executed_steps == 10
    assert log.min_dist == np.inf and not log.collision
    assert all(
        len(getattr(log, key)) == 11
        for key in ("t", "ego_x", "ego_y", "ego_v", "ego_a", "ttc", "action")
    )


def test_braking_matches_analytic_stopping_distance_and_recomputes_rollout():
    assessor = FixedRisk()
    log = run_sim(scene(), assessor, HysteresisPolicy(), duration=3)
    assert log.ego_x[1] == pytest.approx(1.47)
    assert log.ego_v[1] == pytest.approx(14.4)
    assert log.ego_x[-1] == pytest.approx(15**2 / (2 * 6))
    assert log.ego_v[-1] == pytest.approx(0)
    assert np.all(log.ego_v >= 0)
    assert np.all(np.diff(log.ego_x) >= 0)
    assert assessor.contexts[1].ego_future[0, 0] == pytest.approx(1.47 + 14.4 * 0.1)
    assert len(assessor.contexts) == len(log.t)


def test_stops_partway_through_first_step():
    log = run_sim(scene(speed=0.3), FixedRisk(), HysteresisPolicy(), duration=0.3)
    assert log.ego_v[1] == 0
    assert np.allclose(log.ego_x[1:], 0.3**2 / 12)
    assert log.ego_a[1] == 0


def test_minimum_ttc_ignores_excluded_agents():
    scenario = scene()
    base = scenario.agents[0]
    scenario.agents = [
        Trajectory(base.t, base.x, base.y, base.v, base.heading, agent_id=key)
        for key in ("ignored", "far", "near")
    ]

    class PerAgentRisk(RiskAssessor):
        def assess(self, ctx, history):
            return RiskEstimate(
                ttc={"ignored": 0.1, "far": 4.0, "near": 1.5}[history.agent_id],
                considered=history.agent_id != "ignored",
            )

    log = run_sim(scenario, PerAgentRisk(), TTCThresholdPolicy(), duration=0.2)
    assert np.all(log.ttc == 1.5)
    assert log.action[0] == BRAKE


def test_policy_reset_between_runs():
    policy = HysteresisPolicy()
    run_sim(scene(), FixedRisk(1), policy, duration=0.2)
    assert policy.state == BRAKE
    log = run_sim(scene(), FixedRisk(2.5), policy, duration=0.2)
    assert log.action == [CRUISE] * 3


def test_history_cutoff_and_missing_observations():
    scenario = scene(history_start=0)
    history = history_until(scenario.agents[0], -0.1)
    assert len(history.t) == 0
    with pytest.raises(MissingObservationError, match="provide history"):
        run_sim(scenario, FixedRisk(), HysteresisPolicy(), duration=1, latency=0.2)

    class HistorySpy(RiskAssessor):
        def assess(self, ctx, history):
            assert np.all(history.t <= ctx.t_now - 0.2 + 1e-9)
            assert np.allclose(
                ctx.ego_future[:, 0],
                ctx.ego_xy[0] + ctx.ego_v * ctx.dt * np.arange(1, ctx.horizon + 1),
            )
            return RiskEstimate()

    run_sim(scene(), HistorySpy(), NeverBrakePolicy(), duration=1, latency=0.2)


def test_endpoint_safe_but_midstep_collision():
    scenario = scene(speed=0, dt=1, end=1)
    agent = scenario.agents[0]
    agent.t = np.array([0.0, 1.0])
    agent.x = np.array([-10.0, 10.0])
    agent.y = np.zeros(2)
    agent.v = np.full(2, 20.0)
    for collision_dt in (1, 0.02):
        log = run_sim(
            scenario,
            NullAssessor(),
            NeverBrakePolicy(),
            duration=1,
            r_safe=0.1,
            collision_dt=collision_dt,
        )
        assert log.collision
        assert log.min_dist == pytest.approx(0)


def test_replay_knot_inside_step_is_not_skipped():
    scenario = scene(speed=0, dt=1, end=1)
    agent = scenario.agents[0]
    agent.t = np.array([0.0, 0.4, 1.0])
    agent.x = np.array([10.0, 0.0, 10.0])
    agent.y = np.zeros(3)
    agent.v = np.zeros(3)
    agent.heading = np.zeros(3)
    log = run_sim(scenario, NullAssessor(), NeverBrakePolicy(), duration=1, collision_dt=1)
    assert log.collision and log.min_dist == pytest.approx(0)


def test_ego_path_corner_is_not_replaced_by_a_diagonal():
    scenario = scene(speed=2, dt=1, end=1)
    scenario.ego_path = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 100.0]])
    scenario.agents[0].x[:] = 0.5
    scenario.agents[0].y[:] = 0.5
    log = run_sim(
        scenario,
        NullAssessor(),
        NeverBrakePolicy(),
        duration=1,
        horizon=1,
        r_safe=0.1,
        collision_dt=1,
    )
    assert not log.collision  # a diagonal from (0, 0) to (1, 1) would hit
    assert log.min_dist == pytest.approx(0.5)
    assert log.ego_x[-1] == 1 and log.ego_y[-1] == 1


def test_accelerated_sweep_converges_to_analytic_minimum():
    scenario = scene(speed=2, dt=1, end=1)
    agent = scenario.agents[0]
    agent.t = np.array([0.0, 1.0])
    agent.x = np.ones(2)
    agent.y = np.array([-1.0, 1.0])
    agent.v = np.full(2, 2.0)
    agent.heading = np.full(2, np.pi / 2)
    # Ego x(t)=2t-t²; other vehicle (1, 2t-1).
    # d²=(2t-t²-1)²+(1-2t)²; its derivative is a cubic.
    roots = np.roots([4.0, -12.0, 20.0, -8.0])
    candidates = [0.0, 1.0] + [
        float(z.real) for z in roots if abs(z.imag) < 1e-10 and 0 <= z.real <= 1
    ]
    expected = min(np.hypot(2 * t - t * t - 1, 1 - 2 * t) for t in candidates)
    errors = []
    for substep in (0.1, 0.01):
        log = run_sim(
            scenario,
            FixedRisk(),
            TTCThresholdPolicy(a_brake=2),
            duration=1,
            r_safe=0.1,
            collision_dt=substep,
        )
        errors.append(abs(log.min_dist - expected))
        assert not log.collision
    assert errors[1] < errors[0]
    assert errors[1] < 1e-4


def test_initial_collision_is_retained():
    scenario = scene()
    scenario.agents[0].x[:] = 0
    scenario.agents[0].y[:] = 0
    log = run_sim(scenario, NullAssessor(), NeverBrakePolicy(), duration=1)
    assert log.collision and log.min_dist == 0


def test_terminal_risk_does_not_trigger_unexecuted_brake():
    class TerminalRisk(RiskAssessor):
        def assess(self, ctx, history):
            return RiskEstimate(ttc=0.1 if ctx.t_now >= 0.2 else np.inf)

    log = run_sim(scene(), TerminalRisk(), HysteresisPolicy(), duration=0.2)
    assert log.ttc[-1] == 0.1
    assert log.action == [CRUISE] * 3
    assert log.ego_a[-1] == 0
    assert evaluate(log, benign=True)["brake_onset_t"] is None
    log.action[-1] = BRAKE  # also verifies D respects the execution metadata
    assert not evaluate(log, benign=True)["false_brake"]


def test_zero_duration_has_no_executed_actions():
    log = run_sim(scene(), FixedRisk(), HysteresisPolicy(), duration=0)
    assert len(log.t) == 1 and log.executed_steps == 0
    assert not evaluate(log, benign=True)["false_brake"]


def test_legacy_simlog_still_supported():
    log = SimLog(
        np.array([0.0]),
        *(np.array([0.0]) for _ in range(4)),
        [BRAKE],
        np.array([1.0]),
        5.0,
        False,
    )
    assert log.executed_steps is None
    assert evaluate(log, benign=True)["false_brake"]


@pytest.mark.parametrize(
    "kwargs",
    [
        {"duration": 0.15},
        {"latency": -1},
        {"horizon": 0},
        {"r_safe": 0},
        {"collision_dt": 0},
        {"duration": np.nan},
    ],
)
def test_reject_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        run_sim(scene(), NullAssessor(), NeverBrakePolicy(), **kwargs)


def test_reject_short_path_and_truth():
    scenario = scene()
    scenario.ego_path[-1, 0] = 1
    with pytest.raises(ValueError, match="path is too short"):
        run_sim(scenario, NullAssessor(), NeverBrakePolicy(), duration=1)
    with pytest.raises(ValueError, match="truth must cover"):
        run_sim(scene(end=0.5), NullAssessor(), NeverBrakePolicy(), duration=1)


def test_reject_nan_risk():
    with pytest.raises(ValueError, match="Invalid TTC"):
        run_sim(scene(), FixedRisk(np.nan), HysteresisPolicy(), duration=0.2)


def test_vehicle_size_flows_through_context_truth_and_both_assessors():
    scenario = scene(speed=10)
    agent = scenario.agents[0]
    agent.x[:] = 4.95
    agent.y[:] = 2.25
    for width, hit in [(2.5, True), (2.0, False)]:
        spy = FixedRisk()
        log = run_sim(scenario, spy, NeverBrakePolicy(), duration=0, vehicle_width=width)
        assert log.collision == hit
        assert spy.contexts[0].vehicle_width == width
        log = run_sim(
            scenario, make_system("cv"), NeverBrakePolicy(), duration=0, vehicle_width=width
        )
        assert np.isfinite(log.ttc[0]) == hit
    agent.x[:] = 20
    agent.y[:] = 0
    agent.heading[:] = np.pi / 2
    log = run_sim(
        scenario,
        make_system("reactive"),
        NeverBrakePolicy(),
        duration=0,
        vehicle_width=2,
        vehicle_length=8,
    )
    assert log.ttc[0] == pytest.approx((20 - 4 - 1) / 10)


@pytest.mark.parametrize(
    "arg,value",
    [
        ("vehicle_width", 0),
        ("vehicle_length", -1),
        ("vehicle_width", np.nan),
        ("vehicle_length", np.inf),
    ],
)
def test_invalid_vehicle_dimensions(arg, value):
    with pytest.raises(ValueError, match="dimensions"):
        run_sim(scene(), NullAssessor(), NeverBrakePolicy(), duration=0, **{arg: value})


def test_staged_braking_stops_and_recovers_with_bounded_jerk_on_executed_steps():
    class TemporaryRisk(RiskAssessor):
        def assess(self, ctx, history):
            return RiskEstimate(ttc=1.0 if ctx.t_now < 3 else np.inf)

    scenario = scene(speed=7.23)
    policy = StagedBrakingPolicy()
    log = run_sim(scenario, TemporaryRisk(), policy, duration=10)
    assert log.ego_v.min() == 0
    assert np.all(np.diff(log.ego_x) >= 0)
    assert log.ego_v.max() == log.ego_v[-1] == scenario.ego_start[3]
    assert log.action[-1] == CRUISE and RECOVER in log.action
    accel = log.ego_a[: log.executed_steps]
    assert np.min(accel) == -6 and np.max(accel) == 1.5
    limits = np.where(np.array(log.action[: log.executed_steps]) == HARD_BRAKE, 20.0, 5.0)
    assert np.all(abs(np.diff(accel, prepend=0)) / scenario.dt <= limits + 1e-9)
    reset_run = run_sim(scene(speed=3), NullAssessor(), policy, duration=1)
    assert np.all(reset_run.ego_v == 3) and np.all(reset_run.ego_a == 0)


def test_recovery_from_braking_hits_corner_between_frames():
    class OneBrake(RiskAssessor):
        def assess(self, ctx, history):
            return RiskEstimate(ttc=1 if ctx.t_now == 0 else np.inf)

    scenario = scene(speed=2, dt=1, end=2)
    # First second: x=1.5, v=1. Next half-second: s=1.5+0.5+0.125 at corner.
    scenario.ego_path = np.array([[0.0, 0.0], [2.125, 0.0], [2.125, 20.0]])
    scenario.agents[0].x[:] = 2.125
    scenario.agents[0].y[:] = 0
    policy = StagedBrakingPolicy(a_hard=1, a_soft=1, a_recover=1)
    log = run_sim(scenario, OneBrake(), policy, duration=2, horizon=1, collision_dt=1, r_safe=0.001)
    assert log.action[:2] == [HARD_BRAKE, RECOVER]
    assert log.first_collision_t == pytest.approx(1.5, abs=0.001)
    assert log.min_dist == pytest.approx(0)
    assert log.ego_y[-1] == pytest.approx(0.875)


def test_arc_cache_matches_scalar_queries_and_vector_rollout():
    path = np.array([[0.0, 0.0], [3.0, 0.0], [3.0, 4.0], [7.0, 7.0]])
    cum = np.array([0.0, 3.0, 7.0, 12.0])
    arcs = [-1.0, 0.0, 1.0, 3.0, 5.0, 7.0, 8.0, 12.0, 15.0]
    expected = [path_point(path, arc) for arc in arcs]
    np.testing.assert_allclose(path_point(path, arcs, cum=cum), expected)
    for arc in arcs:
        assert path_heading(path, arc, cum=cum) == path_heading(path, arc)
    np.testing.assert_allclose(
        ego_rollout(path, 1, 2, 5, 1, cum=cum), [path_point(path, s) for s in (3, 5, 7, 9, 11)]
    )


@pytest.mark.parametrize("width,collision", [(2.0, False), (2.5, True), (8.0, True)])
def test_width_controls_truth_sweep_when_endpoints_are_clear(width, collision):
    scenario = scene(speed=0, dt=1, history_start=0, end=1)
    agent = scenario.agents[0]
    agent.x[:] = [-20, 20]
    # The widest case also exceeds the default broad-phase cutoff.
    agent.y[:] = 7 if width == 8 else 2.25
    log = run_sim(
        scenario,
        NullAssessor(),
        NeverBrakePolicy(),
        duration=1,
        vehicle_width=width,
        collision_dt=1,
    )
    assert log.collision == collision
    if collision:
        assert log.first_collision_t == pytest.approx(0.375)
    else:
        assert log.first_collision_t is None

"""C-only verification: these assessors are test doubles, not P's algorithms."""

import numpy as np
import pytest

from pbs.eval.metrics import evaluate
from pbs.risk.assessors import NullAssessor, RiskAssessor
from pbs.sim.policy import BRAKE, CRUISE, HysteresisPolicy, NeverBrakePolicy, TTCThresholdPolicy
from pbs.sim.simulator import MissingObservationError, run_sim
from pbs.types import RiskEstimate, Scenario, SimLog, Trajectory
from pbs.utils import history_until


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

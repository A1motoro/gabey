"""C-T6 frame scheduling, observation loss and closed-loop actuator regression tests."""

import math
import time
from dataclasses import replace

import numpy as np
import pytest

from pbs.data.scenarios import make_scenario
from pbs.risk.assessors import NullAssessor
from pbs.sim import FailsafeConfig, FrameFailsafeCoordinator
from pbs.sim.policy import COAST_ZERO, CRUISE, HARD_BRAKE, RECOVER, SOFT_BRAKE, StagedBrakingPolicy
from pbs.sim.simulator import MissingObservationError, run_sim
from pbs.types import RiskContext, RiskEstimate
from pbs.utils import history_until


class RiskByAgent(NullAssessor):
    def __init__(self, **values):
        self.values = values

    def assess(self, ctx, history):
        return RiskEstimate(self.values.get(history.agent_id, math.inf))


class Blocked(NullAssessor):
    def assess(self, ctx, history):
        while True:
            time.sleep(20)


class BrokenBind(NullAssessor):
    def bind(self, scenario):
        raise RuntimeError("failed to load weights")


class DropPackets:
    def __init__(self, start, end):
        self.start, self.end = start, end

    def observe(self, history, *, t_now):
        return None if self.start <= round(t_now, 6) < self.end else history


class Frames:
    def __init__(self, *agents, primary=None, reactive=None, config=None, **kwargs):
        self.scene = make_scenario("cut_in", duration=4)
        lead = self.scene.agents[0]
        self.scene.agents = [replace(lead, agent_id=agent) for agent in agents]
        self.policy = StagedBrakingPolicy()
        self.control = FrameFailsafeCoordinator(
            primary or NullAssessor(), reactive=reactive or NullAssessor(), config=config, **kwargs
        )
        self.control.bind(self.scene, policy=self.policy)
        self.index = 0

    def step(self, *, absent=(), sample_time=None, timings=None):
        now = self.index * self.scene.dt
        ctx = RiskContext(
            now, np.zeros(2), 0, 15, np.zeros((2, 2)), self.scene.ego_path, 3.5, 2, 0.1
        )
        history = {
            a.agent_id: history_until(a, now if sample_time is None else sample_time)
            for a in self.scene.agents
            if a.agent_id not in absent
        }
        decision = self.control.step_frame(
            self.index, ctx, history, current_policy_mode=self.policy.state, channel_timings=timings
        )
        self.policy.decide_accel(
            decision.effective_ttc, v=15, dt=0.1, mode_override=decision.mode_override
        )
        self.index += 1
        return decision


def test_shared_deadline_partial_results_and_late_frame_discard():
    now = [0.0]
    f = Frames("a", "b", "c", primary=RiskByAgent(a=1, b=1, c=1), clock=lambda: now[0])
    d = f.step(timings={"reactive_s": 0.008, "primary_s": [0.04, 0.04, 0.04]})
    assert d.elapsed_s == pytest.approx(0.05)
    assert d.agent_sources == {"a": "fused", "b": "reactive_fallback", "c": "reactive_fallback"}
    now[0] = 0.1
    assert set(f.step().agent_sources.values()) == {"reactive_fallback"}
    now[0] = 0.2
    d = f.step()
    assert d.dropped_stale_results == 1 and d.source == "fused"


def test_80ms_primary_discard_and_hung_channels():
    primary = RiskByAgent(lead=0.2)
    f = Frames("lead", primary=primary)
    d = f.step(timings={"reactive_s": 0.008, "primary_s": 0.080})
    assert d.source == "reactive_fallback" and d.effective_ttc == math.inf
    assert d.elapsed_s == pytest.approx(0.05)
    primary.values.clear()
    d = f.step(timings={"reactive_s": math.inf, "primary_s": math.inf})
    assert d.dropped_stale_results == 1 and math.isnan(d.effective_ttc)
    assert f.step().mode_override == SOFT_BRAKE
    assert f.control.fallback_reasons["primary", "busy"] == 1


@pytest.mark.parametrize(
    "primary,reason", [(BrokenBind(), "bind_error"), (RiskByAgent(lead="bad"), "invalid_output")]
)
def test_invalid_primary_or_failed_bind_falls_back(primary, reason):
    f = Frames("lead", primary=primary, reactive=RiskByAgent(lead=1))
    assert f.step().effective_ttc == 1
    assert f.control.fallback_reasons["primary", reason] == 1


def test_reactive_timeout_primary_only_and_unknown_distinct_from_clear():
    f = Frames("lead", primary=RiskByAgent(lead=2))
    assert f.step(timings={"reactive_s": 0.09}).source == "primary_only"
    f.control.primary.values["lead"] = math.inf
    assert f.step().effective_ttc == math.inf
    d = f.step(absent=("lead",))
    assert math.isnan(d.effective_ttc) and d.missing_agents == ("lead",)
    assert Frames().step().effective_ttc == math.inf


@pytest.mark.parametrize("grace,soft_frame", [(0.1, 1), (0.2, 2), (0.3, 3)])
def test_repeated_sample_does_not_reset_age(grace, soft_frame):
    f = Frames("lead", config=FailsafeConfig(missing_grace_s=grace, max_obs_age_s=1))
    assert f.step().effective_ttc == math.inf
    for i in range(1, soft_frame + 1):
        d = f.step(sample_time=0)
        assert d.mode_override == (SOFT_BRAKE if i == soft_frame else COAST_ZERO)
    assert f.step().mode_override == SOFT_BRAKE
    assert f.step().mode_override == "normal"


def test_missing_target_cannot_be_released_by_neighbor_or_mask_new_threat():
    primary = RiskByAgent(lead=2.4)
    f = Frames("lead", "neighbor", primary=primary)
    f.step()
    assert f.step(absent=("lead",)).mode_override == SOFT_BRAKE
    primary.values["neighbor"] = 0.8
    f.step(absent=("lead",))
    assert f.policy.state == HARD_BRAKE
    primary.values = {"lead": 1.0}
    f.step()
    assert f.policy.state == HARD_BRAKE
    # Losing this now-dangerous lead requires confirmation before release.
    f.step(absent=("lead",))
    primary.values.clear()
    assert f.step().mode_override == HARD_BRAKE
    assert f.step().mode_override == "normal"
    for _ in range(3):
        assert f.step(absent=("neighbor",)).mode_override == "normal"


@pytest.mark.parametrize("margin", [0, 0.2])
def test_margin_hysteresis_boundaries_and_healthy_baseline(margin):
    primary = RiskByAgent()
    f = Frames("lead", primary=primary, config=FailsafeConfig(latency_margin=margin))
    sequence = [3.0, 2.9, 1.5, 1.4, 2.0, 2.1, 3.5, 3.6]
    expected = [
        CRUISE,
        SOFT_BRAKE,
        SOFT_BRAKE,
        HARD_BRAKE,
        HARD_BRAKE,
        SOFT_BRAKE,
        SOFT_BRAKE,
        CRUISE,
    ]
    for raw, mode in zip(sequence, expected, strict=True):
        primary.values["lead"] = raw + margin
        assert f.step().effective_ttc == pytest.approx(raw)
        assert f.policy.state == mode


def test_shared_actuator_through_loss_recovery_and_stop():
    scene = make_scenario("cut_in", duration=5, v_ego=8)
    primary = RiskByAgent(cutter=1.0)
    c = FrameFailsafeCoordinator(primary, reactive=NullAssessor())
    log = run_sim(
        scene,
        primary,
        StagedBrakingPolicy(),
        duration=5,
        failsafe_coordinator=c,
        observation_model=DropPackets(0.1, 5),
    )
    assert log.ego_a[:3] == pytest.approx([-2, -4, -6])
    assert log.ego_v[-1] == pytest.approx(0, abs=1e-8)
    assert np.all(log.ego_v >= 0)
    executed = log.ego_a[: log.executed_steps]
    assert max(abs(np.diff(executed))) <= 2 + 1e-8
    stopped = np.flatnonzero(log.ego_v[:-1] < 1e-8)
    assert np.allclose(executed[stopped], 0, atol=1e-8)


def test_loss_during_recovery_uses_last_executed_acceleration():
    p = StagedBrakingPolicy(v_target=25)
    for _ in range(3):
        mode, a = p.decide_accel(math.inf, v=15, dt=0.1)
    assert mode == RECOVER and a == 1.5
    actual = [
        p.decide_accel(math.nan, v=15, dt=0.1, mode_override=mode)[1]
        for mode in [COAST_ZERO, COAST_ZERO, SOFT_BRAKE, SOFT_BRAKE]
    ]
    assert actual == pytest.approx([1, 0.5, 0, -0.5])


def test_pre_history_loss_is_opt_in_and_nominal_latency_is_compensated():
    scene = make_scenario("cut_in", duration=1)
    assessor = NullAssessor()
    with pytest.raises(MissingObservationError):
        run_sim(scene, assessor, StagedBrakingPolicy(), latency=0.2, duration=0.6)
    c = FrameFailsafeCoordinator(assessor, reactive=NullAssessor())
    log = run_sim(
        scene, assessor, StagedBrakingPolicy(), latency=0.2, duration=0.6, failsafe_coordinator=c
    )
    assert np.isnan(log.ttc[:2]).all() and np.isinf(log.ttc[2:]).all()
    assert log.action[:4] == [COAST_ZERO, COAST_ZERO, COAST_ZERO, CRUISE]
    again = run_sim(scene, assessor, StagedBrakingPolicy(), duration=0.2, failsafe_coordinator=c)
    assert again.action == [CRUISE] * 3


def test_async_blocked_predictor_never_blocks_reactive_or_shutdown():
    scene = make_scenario("cut_in", duration=1)
    p = Blocked()
    c = FrameFailsafeCoordinator(
        p,
        reactive=RiskByAgent(cutter=1),
        config=FailsafeConfig(frame_budget_s=0.15),
        executor_mode="async",
    )
    log = run_sim(scene, p, StagedBrakingPolicy(), failsafe_coordinator=c, duration=0.3)
    assert log.action == [HARD_BRAKE] * 4
    assert all(d.source == "reactive_fallback" for d in log.failsafe_decisions)
    assert c.fallback_reasons["primary", "timeout"] == 1
    assert c.fallback_reasons["primary", "busy"] == 3
    assert c.channels is None


def test_recovery_after_lost_packets_preserves_jerk_in_closed_loop():
    class InitialBrake(NullAssessor):
        def assess(self, ctx, history):
            return RiskEstimate(0.8 if ctx.t_now < 0.6 else math.inf)

    scene = make_scenario("cut_in", duration=5)
    primary = InitialBrake()
    log = run_sim(
        scene,
        primary,
        StagedBrakingPolicy(),
        duration=5,
        failsafe_coordinator=FrameFailsafeCoordinator(primary, reactive=NullAssessor()),
        observation_model=DropPackets(2.2, 2.8),
    )
    assert log.ego_a[21:25] == pytest.approx([1.5, 1.0, 0.5, 0.0])
    assert log.action[22:25] == [COAST_ZERO, SOFT_BRAKE, SOFT_BRAKE]
    assert log.action[28:30] == [SOFT_BRAKE, RECOVER]
    executed = log.executed_steps
    jerk = np.abs(np.diff(log.ego_a[:executed], prepend=0)) / scene.dt
    limits = np.where(np.array(log.action[:executed]) == HARD_BRAKE, 20.0, 5.0)
    assert np.all(jerk <= limits + 1e-8)


def test_grace_sweep_on_paired_cut_ins():
    """Run with pytest -s for the acceptance comparison (minimum centre distance in metres)."""
    from pbs.eval.counterfactual import is_benign
    from pbs.sim.policy import is_braking_action
    from pbs.systems import make_system

    for gap, closing, expected_benign in [(16.0, 4.0, False), (45.0, 0.0, True)]:
        scene = make_scenario("cut_in", gap_d=gap, rel_v=closing, duration=6)
        benign = is_benign(scene, duration=6)
        assert benign == expected_benign
        for grace in (0.1, 0.2, 0.3):
            primary = make_system("cv")
            coordinator = FrameFailsafeCoordinator(
                primary, config=FailsafeConfig(missing_grace_s=grace)
            )
            log = run_sim(
                scene,
                primary,
                StagedBrakingPolicy(),
                duration=6,
                failsafe_coordinator=coordinator,
                observation_model=DropPackets(0.3, 1.0),
            )
            n = log.executed_steps
            false_brake = benign and any(is_braking_action(action) for action in log.action[:n])
            speed_loss = scene.ego_start[3] - np.min(log.ego_v)
            print(
                f"benign={benign} grace={grace:.1f} collision={log.collision} "
                f"min_dist={log.min_dist_until_contact:.3f} false_brake={false_brake} "
                f"peak_speed_loss={speed_loss:.3f}"
            )
            assert log.action[2 + round(grace / scene.dt)] == SOFT_BRAKE
            assert false_brake == benign and speed_loss > 0


def test_async_healthy_workers_match_offline_decisions_and_release_resources():
    scene = make_scenario("cut_in", duration=0.5)
    primary = RiskByAgent(cutter=2.0)
    policy = StagedBrakingPolicy(t_on_soft=2.0, t_off_soft=2.5)
    config = FailsafeConfig(frame_budget_s=0.25)
    for backend in ("async", "deterministic"):
        c = FrameFailsafeCoordinator(primary, config=config, executor_mode=backend)
        log = run_sim(scene, primary, policy, duration=0.5, failsafe_coordinator=c)
        assert log.action == [CRUISE] * 6
        assert all(d.source == "fused" for d in log.failsafe_decisions)
        assert c.channels is None


def test_corrupt_shared_input_becomes_unknown_even_if_assessor_would_say_clear():
    class Corrupted:
        def observe(self, history, *, t_now):
            return replace(history, x=np.full(len(history.x), np.nan))

    scene = make_scenario("cut_in", duration=0.5)
    primary = NullAssessor()
    log = run_sim(
        scene,
        primary,
        StagedBrakingPolicy(),
        duration=0.5,
        observation_model=Corrupted(),
        failsafe_coordinator=FrameFailsafeCoordinator(primary, reactive=NullAssessor()),
    )
    assert np.isnan(log.ttc).all()
    assert log.action[:3] == [COAST_ZERO, COAST_ZERO, SOFT_BRAKE]

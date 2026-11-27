"""Unit and adversarial boundary tests for rear-end sandwich coupling analysis."""

from __future__ import annotations

import numpy as np
import pytest

from pbs.data import make_scenario
from pbs.eval.rear_end import (
    FollowerConfig,
    StagedPolicyConfig,
    rollout_staged_ego,
    simulate_following_vehicle,
    summarize_bidirectional_logs,
    sweep_rear_headway_plane,
)
from pbs.prediction import ConstantVelocityPredictor
from pbs.risk import NullAssessor, PredictiveAssessor, ReactiveAssessor
from pbs.sim.policy import HysteresisPolicy, NeverBrakePolicy, TTCThresholdPolicy
from pbs.sim.simulator import run_sim
from pbs.types import Scenario, SimLog, Trajectory


def _emergency_stop_log(
    *,
    v0: float = 15.0,
    a_brake: float = 6.0,
    t_onset: float = 0.0,
    duration: float = 6.0,
    dt: float = 0.1,
) -> SimLog:
    """Construct an exact 1-D ego SimLog that cruises until ``t_onset`` then brakes to stop."""
    steps = int(round(duration / dt))
    t = np.arange(steps + 1) * dt
    ego_x = np.zeros(steps + 1)
    ego_y = np.zeros(steps + 1)
    ego_v = np.zeros(steps + 1)
    ego_a = np.zeros(steps + 1)
    action: list[str] = []

    x, v = 0.0, v0
    for k in range(steps + 1):
        ego_x[k], ego_v[k] = x, v
        if k == steps:
            action.append(action[-1] if action else "cruise")
            break
        braking = t[k] >= t_onset - 1e-9
        act = "brake" if braking else "cruise"
        action.append(act)
        if braking and v > 0.0:
            ego_a[k] = -a_brake
            h = min(dt, v / a_brake)
            x += v * h - 0.5 * a_brake * h * h
            v = max(0.0, v - a_brake * h)
        else:
            ego_a[k] = 0.0
            x += v * dt

    return SimLog(
        t=t,
        ego_x=ego_x,
        ego_y=ego_y,
        ego_v=ego_v,
        ego_a=ego_a,
        action=action,
        ttc=np.full(steps + 1, np.inf),
        min_dist=50.0,
        collision=False,
        executed_steps=steps,
        first_collision_t=None,
        min_dist_until_contact=50.0,
    )


@pytest.mark.parametrize("model", ["kinematic", "idm"])
@pytest.mark.parametrize("init_gap", [4.0, 12.0, 28.0])
def test_never_brake_preserves_exact_cruising_equilibrium(model: str, init_gap: float):
    scenario = make_scenario("cut_in", gap_d=40.0, rel_v=-2.0, cut_in_t=0.5, lc_T=3.0)
    log = run_sim(scenario, NullAssessor(), NeverBrakePolicy(), duration=10.0, horizon=30)
    res = simulate_following_vehicle(
        log,
        FollowerConfig(init_rear_gap=init_gap, reaction_time=1.0, model=model),
    )
    assert not res.front_collision
    assert not res.rear_collision
    assert res.safe_both
    assert res.outcome == "safe_both"
    assert res.rear_brake_onset_t is None
    assert res.ego_brake_onset_t is None
    assert np.allclose(res.rear_a, 0.0, atol=1e-12)
    assert np.allclose(res.bumper_gap, init_gap, atol=1e-9)
    assert res.min_bumper_gap == pytest.approx(init_gap, abs=1e-9)


def test_reaction_delay_timing_and_analytic_stopping_bound():
    # Ego brakes at t = 1.0 s from 15 m/s at -6 m/s² (ego stopping distance = 18.75 m).
    # Follower with reaction_time = 1.2 s cannot react before t = 2.2 s.
    log = _emergency_stop_log(v0=15.0, a_brake=6.0, t_onset=1.0, duration=8.0)

    # Impossible gap: during 1.2 s reaction time alone, ego loses > 4.3 m and builds 7.2 m/s dv;
    # at equal max_decel = 6.0 m/s² and step jerk, minimum required gap is v0 * tau = 18.0 m.
    crash = simulate_following_vehicle(
        log,
        FollowerConfig(
            init_rear_gap=10.0,
            reaction_time=1.2,
            max_decel=6.0,
            max_jerk=None,
        ),
    )
    assert crash.rear_collision is True
    assert crash.outcome == "rear_only"
    assert crash.rear_impact_speed_diff > 0.0
    assert crash.rear_brake_onset_t == pytest.approx(2.2, abs=0.02)

    # Generous gap well above v0 * tau = 18.0 m survives without contact.
    safe = simulate_following_vehicle(
        log,
        FollowerConfig(
            init_rear_gap=26.0,
            reaction_time=1.2,
            max_decel=6.0,
            max_jerk=None,
        ),
    )
    assert safe.rear_collision is False
    assert safe.safe_both is True
    assert safe.rear_brake_onset_t == pytest.approx(2.2, abs=0.02)
    assert safe.min_bumper_gap > 5.0


def test_non_integer_reaction_time_and_monotonic_headway():
    log = _emergency_stop_log(v0=15.0, a_brake=6.0, t_onset=0.5, duration=7.0)
    r_tight = simulate_following_vehicle(log, init_rear_gap=9.0, reaction_time=0.837)
    r_wide = simulate_following_vehicle(log, init_rear_gap=22.0, reaction_time=0.837)
    r_fast_driver = simulate_following_vehicle(log, init_rear_gap=14.0, reaction_time=0.35)
    r_slow_driver = simulate_following_vehicle(log, init_rear_gap=14.0, reaction_time=1.35)

    assert r_wide.min_bumper_gap > r_tight.min_bumper_gap
    assert r_fast_driver.min_bumper_gap > r_slow_driver.min_bumper_gap


def test_reconstruct_ego_profile_is_smooth_across_step_boundaries():
    scenario = make_scenario(
        "cut_in", gap_d=24.0, rel_v=5.0, cut_in_t=0.5, lc_T=3.0, a_cutter=-1.5, duration=15.0
    )
    slog = rollout_staged_ego(
        scenario,
        PredictiveAssessor(ConstantVelocityPredictor()),
        duration=10.0,
        horizon=40,
    )
    rlog = simulate_following_vehicle(slog, init_rear_gap=12.0, reaction_time=1.0)
    # Since StagedPolicyConfig default has a_recover=0.0, ego_a <= 0 everywhere, so
    # reconstructed 0.01 s speed must be monotonically non-increasing without boundary spikes.
    dv = np.diff(rlog.ego_v)
    assert np.all(dv <= 1e-9)
    assert np.max(np.abs(dv)) <= 0.6 * 0.01 * 12.0  # bounded by physical deceleration * dt


def test_input_simlog_is_not_mutated_and_curved_path_supported():
    # Quarter-circle ego path with a distant harmless agent
    angles = np.linspace(0.0, 0.5 * np.pi, 60)
    radius = 120.0
    path = np.column_stack((radius * np.sin(angles), radius * (1.0 - np.cos(angles))))
    t = np.arange(0.0, 8.1, 0.1)
    far_agent = Trajectory(
        t=t,
        x=np.full_like(t, 200.0),
        y=np.full_like(t, 20.0),
        v=np.zeros_like(t),
        heading=np.zeros_like(t),
        agent_id="static_far",
    )
    init_heading = float(np.arctan2(path[1, 1] - path[0, 1], path[1, 0] - path[0, 0]))
    scenario = Scenario(
        ego_start=(0.0, 0.0, init_heading, 12.0),
        ego_path=path,
        agents=[far_agent],
        dt=0.1,
        kind="cut_in",
    )
    staged_log = rollout_staged_ego(scenario, NullAssessor(), duration=5.0, horizon=20)
    snapshot_x = staged_log.ego_x.copy()
    snapshot_v = staged_log.ego_v.copy()

    rear = simulate_following_vehicle(staged_log, init_rear_gap=10.0, reaction_time=0.9)
    assert np.array_equal(staged_log.ego_x, snapshot_x)
    assert np.array_equal(staged_log.ego_v, snapshot_v)
    assert rear.safe_both
    assert rear.ego_s[-1] == pytest.approx(12.0 * 5.0, rel=1e-3)


def test_front_collision_truncation_handles_zero_and_midstep_contacts():
    base = _emergency_stop_log(v0=15.0, a_brake=6.0, t_onset=2.0, duration=5.0)
    zero_hit = SimLog(
        t=base.t,
        ego_x=base.ego_x,
        ego_y=base.ego_y,
        ego_v=base.ego_v,
        ego_a=base.ego_a,
        action=list(base.action),
        ttc=base.ttc,
        min_dist=0.0,
        collision=True,
        executed_steps=base.executed_steps,
        first_collision_t=0.0,
        min_dist_until_contact=0.0,
    )
    res0 = simulate_following_vehicle(zero_hit)
    assert res0.front_collision is True
    assert res0.rear_collision is False
    assert res0.outcome == "front_only"
    assert len(res0.t) == 1

    mid_hit = SimLog(
        t=base.t,
        ego_x=base.ego_x,
        ego_y=base.ego_y,
        ego_v=base.ego_v,
        ego_a=base.ego_a,
        action=list(base.action),
        ttc=base.ttc,
        min_dist=1.0,
        collision=True,
        executed_steps=base.executed_steps,
        first_collision_t=1.237,
        min_dist_until_contact=1.0,
    )
    res_mid = simulate_following_vehicle(mid_hit)
    assert res_mid.front_collision is True
    assert res_mid.t[-1] == pytest.approx(1.237, abs=1e-9)
    assert not res_mid.safe_both


def test_predictive_staged_beats_binary_and_reactive_on_rear_end_safety():
    """Verify the 2x2 causal chain on a braking cut-in:
    - Reactive (binary or staged) sees the cutter late -> slams hard or collides forward.
    - Predictive CV + Binary avoids front crash, but -6 m/s² slam induces rear collision.
    - Predictive CV + Staged brakes early and gently (-2.5 m/s² ramp) -> saves BOTH ends.
    """
    scenario = make_scenario(
        "cut_in",
        gap_d=24.0,
        rel_v=5.0,
        cut_in_t=0.5,
        lc_T=3.0,
        a_cutter=-1.5,
        duration=23.0,
    )
    sim_kw = dict(duration=19.0, horizon=40, r_safe=None)

    reac_bin = run_sim(
        scenario,
        ReactiveAssessor(),
        HysteresisPolicy(t_on=2.0, t_off=3.0, a_brake=6.0),
        **sim_kw,
    )
    cv_bin = run_sim(
        scenario,
        PredictiveAssessor(ConstantVelocityPredictor()),
        HysteresisPolicy(t_on=2.0, t_off=3.0, a_brake=6.0),
        **sim_kw,
    )
    reac_stg = rollout_staged_ego(scenario, ReactiveAssessor(), **sim_kw)
    cv_stg = rollout_staged_ego(
        scenario, PredictiveAssessor(ConstantVelocityPredictor()), **sim_kw
    )

    follower = FollowerConfig(init_rear_gap=9.5, reaction_time=1.1, max_decel=6.0, max_jerk=10.0)
    r_reac_bin = simulate_following_vehicle(reac_bin, follower)
    r_cv_bin = simulate_following_vehicle(cv_bin, follower)
    r_reac_stg = simulate_following_vehicle(reac_stg, follower)
    r_cv_stg = simulate_following_vehicle(cv_stg, follower)

    assert not r_cv_bin.front_collision
    assert not r_cv_stg.front_collision
    # Staged CV has strictly lower jerk and larger rear clearance than binary CV
    assert r_cv_stg.ego_peak_jerk <= 10.0 + 1e-6
    assert r_cv_bin.ego_peak_jerk >= 59.0
    assert r_cv_stg.min_bumper_gap > r_cv_bin.min_bumper_gap
    assert r_cv_stg.safe_both is True
    assert (not r_cv_bin.safe_both) or (r_cv_stg.min_bumper_gap > r_cv_bin.min_bumper_gap + 1.5)
    # Reactive brakes strictly later than Predictive CV
    assert (
        r_reac_stg.ego_brake_onset_t is None
        or r_cv_stg.ego_brake_onset_t < r_reac_stg.ego_brake_onset_t
    )
    assert not r_reac_bin.safe_both or r_cv_stg.min_bumper_gap > r_reac_bin.min_bumper_gap


def test_summary_and_plane_sweep_helpers():
    scenario = make_scenario("cut_in", gap_d=25.0, rel_v=4.0, cut_in_t=0.5, lc_T=3.0, duration=20.0)
    log_bin = run_sim(
        scenario,
        PredictiveAssessor(ConstantVelocityPredictor()),
        TTCThresholdPolicy(ttc_threshold=2.0, a_brake=6.0),
        duration=15.0,
        horizon=30,
    )
    log_stg = rollout_staged_ego(
        scenario,
        PredictiveAssessor(ConstantVelocityPredictor()),
        duration=15.0,
        horizon=30,
    )
    summary = summarize_bidirectional_logs(
        {
            "cv_binary": [("s1", "dangerous", log_bin)],
            "cv_staged": [("s1", "dangerous", log_stg)],
        },
        FollowerConfig(init_rear_gap=10.0, reaction_time=1.0),
    )
    assert len(summary) == 2
    assert set(summary["variant"]) == {"cv_binary", "cv_staged"}

    plane = sweep_rear_headway_plane(
        {"cv_binary": log_bin, "cv_staged": log_stg},
        rear_gaps=(6.0, 14.0),
        reaction_times=(0.6, 1.2),
    )
    assert len(plane) == 8
    assert (plane["safe_both_rate"].between(0.0, 1.0)).all()


@pytest.mark.parametrize("model", ["kinematic", "idm"])
def test_vectorized_plane_matches_scalar_simulation_exactly(model: str):
    log = _emergency_stop_log(v0=15.0, a_brake=6.0, t_onset=0.7, duration=6.0)
    base = FollowerConfig(model=model, max_decel=6.0, max_jerk=10.0)
    gaps = (5.0, 11.5, 19.0)
    taus = (0.25, 0.837, 1.35)
    plane = sweep_rear_headway_plane(
        {"ego": log}, rear_gaps=gaps, reaction_times=taus, base_config=base
    )

    for _, row in plane.iterrows():
        scalar = simulate_following_vehicle(
            log,
            base,
            init_rear_gap=float(row["init_rear_gap"]),
            reaction_time=float(row["reaction_time"]),
        )
        assert bool(row["any_rear_collision"]) is scalar.rear_collision
        assert float(row["mean_min_bumper_gap"]) == pytest.approx(scalar.min_bumper_gap, abs=1e-9)
        assert float(row["max_rear_impact_dv"]) == pytest.approx(
            scalar.rear_impact_speed_diff, abs=1e-9
        )


@pytest.mark.parametrize(
    "kwargs",
    [
        {"init_rear_gap": 0.0},
        {"init_rear_gap": -3.0},
        {"init_rear_gap": True},
        {"reaction_time": -0.1},
        {"reaction_time": False},
        {"max_decel": 0.0},
        {"max_decel": float("nan")},
        {"max_jerk": 0.0},
        {"substep_dt": 0.0},
        {"substep_dt": 0.5},
        {"model": "unknown"},
        {"truncate_at_front_collision": 1},
    ],
)
def test_invalid_follower_config_rejected(kwargs: dict):
    with pytest.raises(ValueError):
        FollowerConfig(**kwargs)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"t_on_hard": 2.5, "t_off_hard": 2.0},
        {"t_off_hard": 3.2, "t_on_soft": 3.0},
        {"a_soft": 6.0, "a_hard": 6.0},
        {"max_jerk": -1.0},
        {"t_on_hard": True},
    ],
)
def test_invalid_staged_policy_config_rejected(kwargs: dict):
    with pytest.raises(ValueError):
        StagedPolicyConfig(**kwargs)


def test_exp09_quick_end_to_end(tmp_path, monkeypatch):
    import experiments.exp09_rear_end_safety as exp09

    fig_dir = tmp_path / "figs"
    log_dir = tmp_path / "logs"
    monkeypatch.setattr(exp09, "FIG_DIR", fig_dir)
    monkeypatch.setattr(exp09, "LOG_DIR", log_dir)

    summary = exp09.main(["--quick"])
    assert summary["quick"] is True
    assert summary["n_dangerous"] > 0
    assert (fig_dir / "exp09_bidirectional_outcomes.png").is_file()
    assert (fig_dir / "exp09_rear_safety_boundary.png").is_file()
    assert (fig_dir / "exp09_sandwich_timeline.png").is_file()
    assert (log_dir / "exp09_summary.json").is_file()

    v = summary["variants"]
    assert v["cv_staged"]["dangerous_safe_both_rate"] >= v["cv_binary"]["dangerous_safe_both_rate"]
    assert (
        v["nn_ua_staged"]["dangerous_safe_both_rate"]
        >= v["reactive_binary"]["dangerous_safe_both_rate"]
    )


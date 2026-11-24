"""P L3: Option-B PhysicsAnchoredResMLP features, adaptive widths, SE(2), and UA-TTC."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from pbs.data import make_scenario
from pbs.data.womd import load_clip
from pbs.prediction import (
    ConstantVelocityPredictor,
    NeuralPredictor,
    extract_lane_features,
    extract_traj_features,
    forward_resmlp_numpy,
    gaussian_nll_2d,
)
from pbs.prediction.features import (
    HISTORY_STEPS,
    LANE_FEATURE_DIM,
    TRAJ_FEATURE_DIM,
    agent_to_world,
    world_to_agent,
)
from pbs.prediction.neural import (
    DEFAULT_LANE_WEIGHTS,
    DEFAULT_TRAJ_WEIGHTS,
    SIGMA_MAX,
    SIGMA_MIN,
)
from pbs.risk import PredictiveAssessor
from pbs.sim.policy import HysteresisPolicy
from pbs.sim.simulator import run_sim
from pbs.types import RiskContext, Scenario, Trajectory
from pbs.utils import history_until


def make_synthetic_weights(
    input_dim: int,
    hidden_dim: int,
    *,
    horizon: int = 40,
    dt: float = 0.1,
    use_lane: bool = True,
    seed: int = 0,
) -> dict[str, np.ndarray]:
    """Build a valid Option-B PhysicsAnchoredResMLP weight dictionary for any width H."""
    rng = np.random.default_rng(seed)
    scale = 0.05

    def rand(shape):
        return (rng.standard_normal(shape) * scale).astype(np.float32)

    return {
        "schema_version": np.int32(1),
        "use_lane": np.int32(1 if use_lane else 0),
        "input_dim": np.int32(input_dim),
        "hidden_dim": np.int32(hidden_dim),
        "n_res_blocks": np.int32(2),
        "horizon": np.int32(horizon),
        "dt": np.float64(dt),
        "history_steps": np.int32(HISTORY_STEPS),
        "sigma_min": np.float64(SIGMA_MIN),
        "sigma_max": np.float64(SIGMA_MAX),
        "ln_eps": np.float64(1e-5),
        "in_w": rand((hidden_dim, input_dim)),
        "in_b": rand((hidden_dim,)),
        "in_ln_w": np.ones(hidden_dim, dtype=np.float32),
        "in_ln_b": np.zeros(hidden_dim, dtype=np.float32),
        "b1_w1": rand((hidden_dim, hidden_dim)),
        "b1_b1": rand((hidden_dim,)),
        "b1_ln_w": np.ones(hidden_dim, dtype=np.float32),
        "b1_ln_b": np.zeros(hidden_dim, dtype=np.float32),
        "b1_w2": rand((hidden_dim, hidden_dim)),
        "b1_b2": rand((hidden_dim,)),
        "b2_w1": rand((hidden_dim, hidden_dim)),
        "b2_b1": rand((hidden_dim,)),
        "b2_ln_w": np.ones(hidden_dim, dtype=np.float32),
        "b2_ln_b": np.zeros(hidden_dim, dtype=np.float32),
        "b2_w2": rand((hidden_dim, hidden_dim)),
        "b2_b2": rand((hidden_dim,)),
        "mean_w": rand((2 * horizon, hidden_dim)),
        "mean_b": rand((2 * horizon,)),
        "std_w": rand((2 * horizon, hidden_dim)),
        "std_b": rand((2 * horizon,)),
    }


def transform_se2_trajectory(
    traj: Trajectory, shift: np.ndarray, angle: float, *, wrap_heading: bool = False
) -> Trajectory:
    c, s = np.cos(angle), np.sin(angle)
    rot = np.array([[c, -s], [s, c]])
    xy = np.column_stack((traj.x, traj.y)) @ rot.T + shift
    heading = traj.heading + angle
    if wrap_heading:
        heading = np.arctan2(np.sin(heading), np.cos(heading))
    return Trajectory(
        traj.t.copy(),
        xy[:, 0],
        xy[:, 1],
        traj.v.copy(),
        heading,
        agent_type=traj.agent_type,
        agent_id=traj.agent_id,
    )


def transform_se2_path(path: np.ndarray, shift: np.ndarray, angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    rot = np.array([[c, -s], [s, c]])
    return np.asarray(path, dtype=float) @ rot.T + shift


def sample_curved_scenario() -> tuple[Scenario, Trajectory]:
    s = np.linspace(0.0, 180.0, 91)
    radius = 80.0
    theta = s / radius
    ego_path = np.column_stack((radius * np.sin(theta), radius * (1.0 - np.cos(theta))))
    t = np.arange(0.0, 2.5, 0.1)
    ag_s = 25.0 + 12.0 * t
    ag_theta = ag_s / radius
    d_lat = 3.2 - 0.8 * t
    x = radius * np.sin(ag_theta) - d_lat * np.sin(ag_theta)
    y = radius * (1.0 - np.cos(ag_theta)) + d_lat * np.cos(ag_theta)
    vx = np.gradient(x, 0.1)
    vy = np.gradient(y, 0.1)
    traj = Trajectory(t, x, y, np.hypot(vx, vy), np.unwrap(np.arctan2(vy, vx)), agent_id="curved")
    sc = Scenario((0.0, 0.0, 0.0, 12.0), ego_path, [traj], dt=0.1, lane_width=3.6)
    return sc, traj


@pytest.mark.parametrize("hidden_dim", [128, 256, 512, 1024])
@pytest.mark.parametrize("use_lane", [True, False])
def test_adaptive_hidden_widths_work_without_code_changes(hidden_dim, use_lane):
    in_dim = TRAJ_FEATURE_DIM + (LANE_FEATURE_DIM if use_lane else 0)
    w = make_synthetic_weights(in_dim, hidden_dim, use_lane=use_lane, seed=hidden_dim)

    # Verify exact Option-B §4.2 parameter count formula for D=80.
    if use_lane:
        param_keys = [
            k
            for k in w
            if k
            not in (
                "schema_version",
                "use_lane",
                "input_dim",
                "hidden_dim",
                "n_res_blocks",
                "horizon",
                "dt",
                "history_steps",
                "sigma_min",
                "sigma_max",
                "ln_eps",
            )
        ]
        n_params = sum(w[k].size for k in param_keys)
        assert n_params == 4 * hidden_dim**2 + 251 * hidden_dim + 160

    sc = make_scenario("cut_in", gap_d=25.0, rel_v=2.0)
    hist = history_until(sc.agents[0], 1.5)
    pred = NeuralPredictor(w, use_lane=use_lane)
    pred.bind(sc)
    out = pred.predict(hist, 40, 0.1)
    assert pred.hidden_dim == hidden_dim
    assert out.shape == (40, 2)
    assert pred.latest is not None and pred.latest.shape == (40, 2)
    assert pred.latest_std is not None and pred.latest_std.shape == (40, 2)
    assert np.isfinite(out).all()
    assert np.isfinite(pred.latest_std).all()
    assert (pred.latest_std >= SIGMA_MIN - 1e-12).all()
    assert (pred.latest_std <= SIGMA_MAX + 1e-12).all()

    # Direct forward_resmlp_numpy matches NeuralPredictor's pre-transposed path.
    traj_f = extract_traj_features(hist)
    feat = (
        np.concatenate((traj_f, extract_lane_features(hist, sc.ego_path, sc.lane_width)))
        if use_lane
        else traj_f
    )
    tau = np.arange(1, 41, dtype=float) * 0.1
    cv_anchor = np.column_stack((float(hist.v[-1]) * tau, np.zeros(40)))
    mu_ref, std_ref = forward_resmlp_numpy(feat, w, cv_anchor)
    expected_world = agent_to_world(
        mu_ref, np.array([hist.x[-1], hist.y[-1]]), float(hist.heading[-1])
    )
    np.testing.assert_allclose(out, expected_world, atol=1e-5)
    np.testing.assert_allclose(pred.latest_std, std_ref, atol=1e-5)


@pytest.mark.parametrize(
    "shift,angle",
    [
        (np.array([150.0, -42.5]), 0.63),
        (np.array([-5e5, 7e5]), -2.41),
        (np.array([12.0, 99.0]), np.pi),
    ],
)
def test_features_se2_invariance_on_curved_road_and_large_coords(shift, angle):
    sc, hist = sample_curved_scenario()
    hist_rot = transform_se2_trajectory(hist, shift, angle, wrap_heading=True)
    path_rot = transform_se2_path(sc.ego_path, shift, angle)

    np.testing.assert_allclose(
        extract_traj_features(hist_rot), extract_traj_features(hist), rtol=0.0, atol=1e-5
    )
    np.testing.assert_allclose(
        extract_lane_features(hist_rot, path_rot, sc.lane_width),
        extract_lane_features(hist, sc.ego_path, sc.lane_width),
        rtol=0.0,
        atol=1e-5,
    )


@pytest.mark.parametrize("use_lane", [True, False])
@pytest.mark.parametrize(
    "shift,angle",
    [
        (np.array([32.5, -18.0]), 0.85),
        (np.array([1.2e5, -2.5e5]), -2.15),
    ],
)
def test_neural_predictor_se2_equivariance_and_std_invariance(use_lane, shift, angle):
    sc, hist = sample_curved_scenario()
    pred = NeuralPredictor(use_lane=use_lane)
    pred.bind(sc)
    out0 = pred.predict(hist, 30, 0.1)
    std0 = pred.latest_std.copy()

    hist_rot = transform_se2_trajectory(hist, shift, angle, wrap_heading=True)
    path_rot = transform_se2_path(sc.ego_path, shift, angle)
    sc_rot = Scenario(sc.ego_start, path_rot, [hist_rot], dt=sc.dt, lane_width=sc.lane_width)
    pred.bind(sc_rot)
    out_rot = pred.predict(hist_rot, 30, 0.1)
    std_rot = pred.latest_std.copy()

    expected_rot = transform_se2_path(out0, shift, angle)
    np.testing.assert_allclose(out_rot, expected_rot, rtol=0.0, atol=1e-4)
    np.testing.assert_allclose(std_rot, std0, rtol=0.0, atol=1e-5)


def test_interface_contracts_horizon_dt_isolation_and_no_mutation():
    sc = make_scenario("cut_in", gap_d=25.0, rel_v=2.0)
    hist = history_until(sc.agents[0], 2.0)
    before = [arr.copy() for arr in (hist.t, hist.x, hist.y, hist.v, hist.heading)]
    path_before = sc.ego_path.copy()

    pred = NeuralPredictor(use_lane=True)
    pred.bind(sc)

    # horizon=0 returns (0, 2) and updates latest / latest_std to (0, 2)
    z = pred.predict(hist, 0, 0.1)
    assert z.shape == (0, 2)
    assert pred.latest.shape == (0, 2)
    assert pred.latest_std.shape == (0, 2)

    # Arbitrary horizons (shorter, exact, longer than 4.0 s) and non-default dt
    for h, dt in [(1, 0.1), (15, 0.1), (30, 0.1), (40, 0.1), (45, 0.1), (12, 0.2), (20, 0.05)]:
        out = pred.predict(hist, h, dt)
        assert out.shape == (h, 2)
        assert pred.latest.shape == (h, 2)
        assert pred.latest_std.shape == (h, 2)
        assert np.isfinite(out).all() and np.isfinite(pred.latest_std).all()
        assert (pred.latest_std >= SIGMA_MIN - 1e-12).all()
        assert (pred.latest_std <= SIGMA_MAX + 1e-12).all()

    # Returned array must not alias pred.latest
    baseline = pred.predict(hist, 20, 0.1)
    baseline_copy = baseline.copy()
    baseline[0, 0] = 9999.0
    np.testing.assert_array_equal(pred.latest, baseline_copy)

    # Poisoning GT accel & future trajectory in Scenario cannot change prediction
    hist.accel = np.full((len(hist.t), 2), np.nan)
    sc.agents[0].x[sc.agents[0].t > 2.0] += 5000.0
    sc.agents[0].y[sc.agents[0].t > 2.0] -= 5000.0
    sc.ego_path[:] += 1000.0  # bind() made a defensive copy
    after = pred.predict(hist, 20, 0.1)
    np.testing.assert_array_equal(after, baseline_copy)

    # Original history arrays untouched
    for arr, orig in zip((hist.t, hist.x, hist.y, hist.v, hist.heading), before, strict=True):
        np.testing.assert_array_equal(arr, orig)
    assert not np.array_equal(sc.ego_path, path_before)


def test_short_history_and_single_frame_stationary_degradation():
    pred = NeuralPredictor(use_lane=True)
    # Single stationary frame degrades identically to ConstantVelocityPredictor (Option-B §3.1)
    h_static = Trajectory(
        np.array([1.0]), np.array([10.0]), np.array([2.5]), np.array([0.0]), np.array([0.4])
    )
    out = pred.predict(h_static, 15, 0.1)
    expected = ConstantVelocityPredictor().predict(h_static, 15, 0.1)
    np.testing.assert_allclose(out, expected, atol=1e-12)
    np.testing.assert_allclose(pred.latest_std, np.full((15, 2), SIGMA_MIN))

    # Single moving frame & 3-frame short history work without NaN and stay SE(2) equivariant
    h_one = Trajectory(
        np.array([0.0]), np.array([15.0]), np.array([3.5]), np.array([12.0]), np.array([0.1])
    )
    out1 = pred.predict(h_one, 20, 0.1)
    assert out1.shape == (20, 2) and np.isfinite(out1).all()

    shift, angle = np.array([-40.0, 25.0]), 1.2
    out1_rot = pred.predict(transform_se2_trajectory(h_one, shift, angle), 20, 0.1)
    np.testing.assert_allclose(out1_rot, transform_se2_path(out1, shift, angle), atol=1e-4)


def test_irregular_large_timestamps_and_duplicate_path_vertices():
    t = 1e6 + np.array([0.0, 0.05, 0.22, 0.31, 0.55])
    x = 5e5 + 10.0 * (t - t[0])
    y = -7e5 + np.zeros_like(t)
    h = Trajectory(t, x, y, np.full_like(t, 10.0), np.zeros_like(t))
    path = np.array([[5e5, -7e5], [5e5 + 10, -7e5], [5e5 + 10, -7e5], [5e5 + 200, -7e5]])
    feat_t = extract_traj_features(h)
    feat_l = extract_lane_features(h, path, lane_width=3.5)
    assert feat_t.shape == (TRAJ_FEATURE_DIM,) and np.isfinite(feat_t).all()
    assert feat_l.shape == (LANE_FEATURE_DIM,) and np.isfinite(feat_l).all()
    np.testing.assert_allclose(feat_l[0], 0.0, atol=1e-5)


def test_neural_predictor_input_and_checkpoint_validation():
    pred = NeuralPredictor(use_lane=True)
    h = Trajectory(
        np.array([0.0, 0.1]),
        np.array([0.0, 1.0]),
        np.array([0.0, 0.0]),
        np.array([10.0, 10.0]),
        np.array([0.0, 0.0]),
    )
    for bad_h, bad_dt in [(-1, 0.1), (True, 0.1), (1.5, 0.1), (5, 0.0), (5, np.nan)]:
        with pytest.raises(ValueError):
            pred.predict(h, bad_h, bad_dt)

    # Mismatched use_lane vs checkpoint raises ValueError
    with pytest.raises(ValueError, match="use_lane"):
        NeuralPredictor(DEFAULT_TRAJ_WEIGHTS, use_lane=True)
    with pytest.raises(ValueError, match="use_lane"):
        NeuralPredictor(DEFAULT_LANE_WEIGHTS, use_lane=False)

    # Non-finite weights rejected
    bad_w = make_synthetic_weights(80, 128, use_lane=True)
    bad_w["mean_w"][0, 0] = np.nan
    with pytest.raises(ValueError, match="non-finite"):
        NeuralPredictor(bad_w, use_lane=True)

    # Missing file rejected
    with pytest.raises(FileNotFoundError):
        NeuralPredictor(Path("nonexistent_weights_file.npz"), use_lane=True)

    # Zero hidden_dim, invalid history_steps, non-contiguous blocks, degenerate bind path
    w_zero = make_synthetic_weights(80, 128, use_lane=True)
    w_zero["in_w"] = np.empty((0, 80), dtype=np.float32)
    with pytest.raises(ValueError, match="hidden_dim"):
        NeuralPredictor(w_zero, use_lane=True)

    w_bad_steps = make_synthetic_weights(80, 128, use_lane=True)
    w_bad_steps["history_steps"] = np.int32(0)
    with pytest.raises(ValueError, match="history_steps"):
        NeuralPredictor(w_bad_steps, use_lane=True)

    w_gap = make_synthetic_weights(80, 128, use_lane=True)
    for suffix in ("w1", "b1", "ln_w", "ln_b", "w2", "b2"):
        w_gap[f"b3_{suffix}"] = w_gap.pop(f"b2_{suffix}")
    with pytest.raises(ValueError, match="contiguous"):
        NeuralPredictor(w_gap, use_lane=True)

    # forward_resmlp_numpy rejects missing keys, NaN weights, and bool bounds with ValueError
    w_missing = make_synthetic_weights(80, 128, use_lane=True)
    del w_missing["std_w"]
    with pytest.raises(ValueError, match="Missing"):
        forward_resmlp_numpy(np.zeros(80, dtype=np.float32), w_missing, np.zeros((40, 2)))
    w_nan = make_synthetic_weights(80, 128, use_lane=True)
    w_nan["in_w"][0, 0] = np.nan
    with pytest.raises(ValueError, match="non-finite"):
        forward_resmlp_numpy(np.zeros(80, dtype=np.float32), w_nan, np.zeros((40, 2)))
    with pytest.raises(ValueError, match="sigma_min"):
        forward_resmlp_numpy(
            np.zeros(80, dtype=np.float32),
            make_synthetic_weights(80, 128, use_lane=True),
            np.zeros((40, 2)),
            sigma_min=True,
        )

    # bind() rejects degenerate zero-length ego_path and clears stale latest/latest_std
    pred.predict(h, 5, 0.1)
    assert pred.latest is not None and pred.latest_std is not None
    bad_sc = Scenario((0, 0, 0, 10), np.array([[5.0, 5.0], [5.0, 5.0]]), [h], dt=0.1)
    with pytest.raises(ValueError, match="non-zero length"):
        pred.bind(bad_sc)
    assert pred.latest is None and pred.latest_std is None


def test_adversarial_edge_cases_empty_batch_sharp_bend_jitter_and_float32_meta():
    # 1. Empty batch N=0 in forward_batch works cleanly
    pred = NeuralPredictor(use_lane=True)
    m0, s0 = pred.forward_batch(np.empty((0, 10, 6), np.float32), np.empty((0, 20), np.float32))
    assert m0.shape == (0, 40, 2) and s0.shape == (0, 40, 2)

    # 2. Float32-quantized scalar metadata (0.1 -> 0.10000000149) snaps cleanly to exact 0.1
    w32 = make_synthetic_weights(80, 128, use_lane=True)
    w32["dt"] = np.float32(0.1)
    w32["sigma_min"] = np.float32(0.05)
    p32 = NeuralPredictor(w32, use_lane=True)
    assert p32.model_dt == 0.1
    assert p32.sigma_min == 0.05

    # 3. Sharp 90-degree bend interior vertex preserves continuous Euclidean signed d_lat
    bend = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]])
    d_lats = []
    for y_val in (-0.01, 0.0, 0.01):
        h_bend = Trajectory(
            np.array([0.0, 0.1]),
            np.array([15.0, 15.0]),
            np.array([y_val, y_val]),
            np.array([5.0, 5.0]),
            np.array([0.0, 0.0]),
        )
        feat = extract_lane_features(h_bend, bend, lane_width=3.5)
        d_lats.append(float(feat[0]) * 3.5)
    np.testing.assert_allclose(d_lats, [-5.0, -5.0, -5.0], atol=1e-3)

    # Diagonal point outside corner (13, -3) -> exact distance -sqrt(18) = -4.24264 m
    h_diag = Trajectory(
        np.array([0.0, 0.1]),
        np.array([13.0, 13.0]),
        np.array([-3.0, -3.0]),
        np.array([5.0, 5.0]),
        np.array([0.0, 0.0]),
    )
    feat_diag = extract_lane_features(h_diag, bend, lane_width=3.5)
    np.testing.assert_allclose(float(feat_diag[0]) * 3.5, -np.hypot(3.0, 3.0), atol=1e-4)

    # 4. Near-duplicate 1e-12 m endpoint jitter vertex is filtered cleanly
    clean_path = np.array([[0.0, 0.0], [100.0, 0.0]])
    jitter_path = np.array([[0.0, 0.0], [0.0, 1e-12], [100.0, 0.0]])
    h_behind = Trajectory(
        np.array([0.0, 0.1]),
        np.array([-1.5, -0.5]),
        np.array([-3.0, -3.0]),
        np.array([10.0, 10.0]),
        np.array([0.0, 0.0]),
    )
    np.testing.assert_allclose(
        extract_lane_features(h_behind, jitter_path, 3.5),
        extract_lane_features(h_behind, clean_path, 3.5),
        atol=1e-6,
    )

    # 5. np.bool_ rejected for lane_width
    with pytest.raises(ValueError, match="lane_width"):
        extract_lane_features(h_behind, clean_path, lane_width=np.bool_(True))


def test_uncertainty_aware_ttc_monotonicity_and_closed_loop_sim():
    sc = make_scenario("cut_in", gap_d=22.0, rel_v=3.0, cut_in_t=0.5, lc_T=3.0)
    nn_plain = PredictiveAssessor(NeuralPredictor(use_lane=True), uncertainty_scale=0.0)
    nn_ua = PredictiveAssessor(NeuralPredictor(use_lane=True), uncertainty_scale=1.0, name="nn_ua")
    nn_plain.bind(sc)
    nn_ua.bind(sc)

    assert nn_plain.name == "nn"
    assert nn_ua.name == "nn_ua"

    for t_now in (1.0, 1.5, 2.0, 2.5):
        hist = history_until(sc.agents[0], t_now)
        ego_xy = np.array([15.0 * t_now, 0.0])
        tau = np.arange(1, 41) * 0.1
        ego_fut = ego_xy + np.column_stack((15.0 * tau, np.zeros(40)))
        ctx = RiskContext(
            t_now,
            ego_xy,
            0.0,
            15.0,
            ego_fut,
            sc.ego_path,
            sc.lane_width,
            40,
            0.1,
            r_safe=None,
            ego_future_heading=np.zeros(40),
        )
        r_plain = nn_plain.assess(ctx, hist)
        r_ua = nn_ua.assess(ctx, hist)
        assert r_ua.ttc <= r_plain.ttc + 1e-12

    # Full closed-loop simulation on synthetic + frozen real Waymo clip
    for kind in ("cut_in", "lead_brake", "crossing"):
        s = make_scenario(kind, duration=6.0)
        for assessor in (
            PredictiveAssessor(NeuralPredictor(use_lane=False)),
            PredictiveAssessor(NeuralPredictor(use_lane=True)),
            PredictiveAssessor(NeuralPredictor(use_lane=True), uncertainty_scale=1.0, name="nn_ua"),
        ):
            log = run_sim(s, assessor, HysteresisPolicy(), duration=6.0, horizon=40)
            assert np.isfinite(log.ego_x).all()
            assert (log.ego_v >= 0.0).all()
            assert not np.isnan(log.ttc).any()

    clip = load_clip("tests/fixtures/cases/womd_dangerous_6e86a639df5d4fa1_track1782.npz")
    log_real = run_sim(
        clip,
        PredictiveAssessor(NeuralPredictor(use_lane=True), uncertainty_scale=1.0, name="nn_ua"),
        HysteresisPolicy(),
        duration=clip.params["duration"],
        horizon=40,
    )
    assert np.isfinite(log_real.ego_x).all() and not np.isnan(log_real.ttc).any()


def test_gaussian_nll_2d_exact_and_validation():
    mu = np.array([[1.0, 2.0], [3.0, 4.0]])
    std = np.array([[0.5, 2.0], [1.0, 1.0]])
    y = np.array([[1.5, 2.0], [3.0, 4.0]])
    nll = gaussian_nll_2d(mu, std, y)
    # Row 0: dx=0.5/0.5=1 -> 0.5*1 + log(0.5) + log(2) + log(2*pi) = 0.5 + log(2*pi)
    # Row 1: zero error, unit std -> log(2*pi)
    np.testing.assert_allclose(nll, [0.5 + np.log(2 * np.pi), np.log(2 * np.pi)], atol=1e-12)
    with pytest.raises(ValueError, match="strictly positive"):
        gaussian_nll_2d(mu, np.zeros_like(std), y)


def test_world_agent_frame_round_trip():
    pts = np.array([[12.3, -4.5], [100.0, 250.0]])
    origin = np.array([10.0, 20.0])
    heading = -1.37
    loc = world_to_agent(pts, origin, heading)
    recovered = agent_to_world(loc, origin, heading)
    np.testing.assert_allclose(recovered, pts, atol=1e-12)


def test_exp01_cli_exports_neural_and_nll(tmp_path):
    root = Path(__file__).resolve().parents[1]
    subprocess.run(
        [
            sys.executable,
            str(root / "experiments/exp01_predictor_ade.py"),
            "--include-lr",
            "--include-nn",
            "--include-waymo-clips",
            "--output-dir",
            str(tmp_path),
        ],
        cwd=tmp_path,
        check=True,
        text=True,
        capture_output=True,
    )
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert summary["models"] == ["cv", "ca", "ctrv", "lr", "nn_traj", "nn", "oracle"]
    assert "nn_synthetic_2s" in summary
    assert summary["waymo_clips_windows_by_cohort"] == {"illustrative": 10, "primary": 50}
    waymo_acc = pd.read_csv(tmp_path / "waymo_clips_ade_fde.csv")
    assert set(waymo_acc["cohort"].unique()) == {"primary", "illustrative"}
    errors = pd.read_csv(tmp_path / "prediction_errors.csv")
    assert "nll_nats" in errors.columns
    assert errors[errors.model == "nn"]["nll_nats"].notna().all()
    assert errors[errors.model == "nn_traj"]["nll_nats"].notna().all()
    assert errors[errors.model == "cv"]["nll_nats"].isna().all()
    assert (tmp_path / "exp01_predictor_ade.png").stat().st_size > 100

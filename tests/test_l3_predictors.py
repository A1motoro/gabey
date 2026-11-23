"""P L3: linear-fit accuracy, history isolation, and existing simulator integration."""

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from pbs.data import make_scenario
from pbs.eval.metrics import ade
from pbs.eval.sweep import run_matrix
from pbs.prediction import ConstantVelocityPredictor, LinearRegressionPredictor
from pbs.risk import PredictiveAssessor
from pbs.sim.policy import TTCThresholdPolicy
from pbs.sim.simulator import run_sim
from pbs.systems import make_system
from pbs.types import RiskContext, Trajectory
from pbs.utils import history_until


def history(t, xy):
    n = len(t)
    return Trajectory(np.asarray(t), xy[:, 0], xy[:, 1], np.zeros(n), np.zeros(n))


@pytest.mark.parametrize("seed", range(5))
def test_lr_exact_linear_motion_with_irregular_large_timestamps(seed):
    rng = np.random.default_rng(seed)
    t = 1e6 + np.cumsum(rng.uniform(0.01, 0.3, 8))
    velocity = rng.uniform(-20, 20, 2)
    origin = np.array([5e5, -7e5])
    xy = origin + (t - t[0])[:, None] * velocity
    future = t[-1] - t[0] + np.arange(1, 21) * 0.1
    expected = origin + future[:, None] * velocity
    np.testing.assert_allclose(
        LinearRegressionPredictor().predict(history(t, xy), 20, 0.1),
        expected,
        rtol=0,
        atol=1e-8,
    )


def test_lr_is_the_windowed_position_fit_and_ignores_metadata_and_old_samples():
    t = np.arange(12) * 0.1
    xy = np.column_stack((t**2, 2 * t + 0.01 * np.sin(10 * t)))
    h = history(t, xy.copy())
    model = LinearRegressionPredictor(m=5)
    design = np.column_stack((np.ones(5), t[-5:] - t[-1]))
    coefficients = np.linalg.lstsq(design, xy[-5:], rcond=None)[0]
    expected = np.column_stack((np.ones(10), np.arange(1, 11) * 0.1)) @ coefficients
    np.testing.assert_allclose(model.predict(h, 10, 0.1), expected, atol=1e-12)
    h.x[:-5] += 1000
    h.y[:-5] -= 1000
    h.v[:] = 999
    h.heading[:] = 2
    h.accel = np.full((len(t), 2), np.nan)
    np.testing.assert_allclose(model.predict(h, 10, 0.1), expected, atol=1e-12)


@pytest.mark.parametrize("angle", [0, np.pi / 2, np.pi])
def test_lr_one_observation_falls_back_to_observed_speed_and_heading(angle):
    h = history(np.array([2.0]), np.array([[3.0, -1.0]]))
    h.v[:] = 4
    h.heading[:] = angle
    np.testing.assert_allclose(
        LinearRegressionPredictor().predict(h, 10, 0.1),
        ConstantVelocityPredictor().predict(h, 10, 0.1),
    )


def test_lr_stationary_zero_horizon_and_no_mutation():
    t = np.arange(5) * 0.1
    h = history(t, np.tile([3, 5], (5, 1)))
    before = [value.copy() for value in (h.t, h.x, h.y, h.v, h.heading)]
    model = LinearRegressionPredictor()
    assert model.predict(h, 0, 0.1).shape == (0, 2)
    np.testing.assert_allclose(model.predict(h, 10, 0.1), np.tile([3, 5], (10, 1)))
    for value, original in zip((h.t, h.x, h.y, h.v, h.heading), before, strict=True):
        np.testing.assert_array_equal(value, original)


@pytest.mark.parametrize("m", [0, 1, True, 2.5])
def test_lr_rejects_invalid_window(m):
    with pytest.raises(ValueError, match="m must"):
        LinearRegressionPredictor(m)


@pytest.mark.parametrize("horizon,dt", [(-1, 0.1), (True, 0.1), (1.5, 0.1), (2, 0), (2, np.inf)])
def test_lr_reuses_prediction_input_validation(horizon, dt):
    h = history(np.array([0.0, 0.1]), np.array([[0, 0], [1, 0]]))
    with pytest.raises(ValueError):
        LinearRegressionPredictor().predict(h, horizon, dt)


@pytest.mark.parametrize("problem", ["empty", "duplicate", "nan", "length"])
def test_lr_rejects_invalid_observation_arrays(problem):
    h = history(np.array([0.0, 0.1]), np.array([[0.0, 0.0], [1.0, 0.0]]))
    if problem == "empty":
        h.t = np.array([])
    elif problem == "duplicate":
        h.t[:] = 0
    elif problem == "nan":
        h.x[0] = np.nan
    else:
        h.y = h.y[:1]
    with pytest.raises(ValueError):
        LinearRegressionPredictor().predict(h, 2, 0.1)


def test_lr_reduces_noise_error_for_constant_velocity_over_fixed_seeds():
    t = np.arange(20) * 0.1
    future = t[-1] + np.arange(1, 21) * 0.1
    truth = np.column_stack((8 * future, -future))
    errors = []
    for seed in range(20):
        noise = np.random.default_rng(seed).normal(0, 0.05, (len(t), 2))
        h = history(t, np.column_stack((8 * t, -t)) + noise)
        errors.append(
            [
                ade(LinearRegressionPredictor().predict(h, 20, 0.1), truth),
                ade(ConstantVelocityPredictor().predict(h, 20, 0.1), truth),
            ]
        )
    mean_lr, mean_cv = np.mean(errors, axis=0)
    assert mean_lr < 0.3 * mean_cv


def test_lr_future_truth_changes_do_not_change_forecast():
    sc = make_scenario("cut_in")
    model = LinearRegressionPredictor()
    model.bind(sc)
    h = history_until(sc.agents[0], 2)
    expected = model.predict(h, 20, sc.dt)
    sc.agents[0].x[sc.agents[0].t > 2] += 1000
    sc.agents[0].y[sc.agents[0].t > 2] += 1000
    np.testing.assert_array_equal(model.predict(h, 20, sc.dt), expected)


@pytest.mark.parametrize("age", [0.2, 0.25])
def test_lr_assessor_aligns_delayed_linear_observations(age):
    t = np.arange(11) * 0.1
    h = history(t, np.column_stack((10 * t, np.zeros(len(t)))))
    now = 1 + age
    future = np.column_stack((10 * (now + np.arange(1, 11) * 0.1), np.zeros(10)))
    ctx = RiskContext(
        now, future[0], 0, 10, future, np.array([[0, 0], [500, 0]]), 3.5, 10, 0.1, 0.01
    )
    assert PredictiveAssessor(LinearRegressionPredictor()).assess(ctx, h).ttc == pytest.approx(0.1)


@pytest.mark.parametrize("kind", ["cut_in", "lead_brake", "crossing"])
def test_lr_existing_system_registry_runs_all_scenario_families(kind):
    sc = make_scenario(kind)
    log = run_sim(sc, make_system("lr"), TTCThresholdPolicy(), duration=sc.params["duration"])
    assert np.isfinite(log.ego_x).all()
    assert (log.ego_v >= 0).all()
    assert not np.isnan(log.ttc).any()


def test_lr_runs_in_existing_d_matrix():
    sc = make_scenario("crossing", duration=2)
    frame = run_matrix(["cv", "lr"], [sc], [2], sim_kwargs={"duration": 1})
    assert frame["system"].tolist() == ["cv", "lr"]
    assert frame["benign"].nunique() == 1


def test_exp01_l3_cli_exports_lr_on_the_same_forecast_origins(tmp_path):
    root = Path(__file__).resolve().parents[1]
    subprocess.run(
        [
            sys.executable,
            str(root / "experiments/exp01_predictor_ade.py"),
            "--include-lr",
            "--output-dir",
            str(tmp_path),
        ],
        cwd=tmp_path,
        check=True,
        text=True,
        capture_output=True,
    )
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert summary["models"] == ["cv", "ca", "ctrv", "lr", "oracle"]
    assert summary["lr_estimation_samples"] == 10
    errors = pd.read_csv(tmp_path / "prediction_errors.csv")
    pairs = errors.groupby(["scenario", "t0_s", "horizon_s"])["model"].nunique()
    assert (pairs == 5).all()
    assert len(errors) == 14 * 23 * 4 * 5
    assert summary["oracle_max_error_m"] < 1e-10
    assert (tmp_path / "exp01_predictor_ade.png").stat().st_size > 100

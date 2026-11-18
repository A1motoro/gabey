"""Analytic motion, observation isolation and unchanged P/C/D integration for L2."""

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from pbs.data import make_scenario, prediction_slices
from pbs.prediction import (
    ConstantAccelPredictor,
    ConstantVelocityPredictor,
    CTRVPredictor,
    OraclePredictor,
)
from pbs.risk import PredictiveAssessor
from pbs.sim.policy import TTCThresholdPolicy
from pbs.sim.simulator import run_sim
from pbs.systems import make_system
from pbs.types import RiskContext, Scenario, Trajectory
from pbs.utils import history_until, state_at


def track(t, xy, speed=None, heading=None, agent_id="target"):
    n = len(t)
    return Trajectory(
        np.asarray(t),
        xy[:, 0],
        xy[:, 1],
        np.zeros(n) if speed is None else np.asarray(speed),
        np.zeros(n) if heading is None else np.asarray(heading),
        agent_id=agent_id,
    )


def scenario_with(*agents):
    return Scenario((0, 0, 0, 10), np.array([[0, 0], [500, 0]]), list(agents))


@pytest.mark.parametrize("angle", [0, 0.7, np.pi / 2, np.pi])
@pytest.mark.parametrize("a", [-2.0, 0.0, 2.0])
def test_ca_exact_quadratic_irregular_times_and_rotation(angle, a):
    t = 1000 + np.array([0, 0.07, 0.2, 0.31, 0.6, 0.9])
    relative = t - 1000
    direction = np.array([np.cos(angle), np.sin(angle)])
    xy = [123, -40] + (12 * relative + 0.5 * a * relative**2)[:, None] * direction
    h = track(t, xy)  # Speed metadata is deliberately stale.
    h.accel = np.full((len(t), 2), 999.0)  # GT must never influence CA.
    future = relative[-1] + np.arange(1, 21) * 0.1
    expected = [123, -40] + (12 * future + 0.5 * a * future**2)[:, None] * direction
    np.testing.assert_allclose(ConstantAccelPredictor().predict(h, 20, 0.1), expected, atol=1e-9)


def test_ca_estimates_both_axes_without_truth_acceleration():
    t = np.arange(10) * 0.1
    v, a = np.array([10, 2]), np.array([2, 1])
    h = track(t, t[:, None] * v + 0.5 * t[:, None] ** 2 * a)
    future = t[-1] + np.arange(1, 11) * 0.1
    expected = future[:, None] * v + 0.5 * future[:, None] ** 2 * a
    np.testing.assert_allclose(ConstantAccelPredictor().predict(h, 10, 0.1), expected, atol=1e-12)


@pytest.mark.parametrize("angle", [0, 0.8, np.pi / 2, np.pi])
def test_ca_braking_stops_at_physical_stop_position(angle):
    t = np.arange(10) * 0.1
    direction = np.array([np.cos(angle), np.sin(angle)])
    h = track(t, (4 * t - t**2)[:, None] * direction)
    future = np.minimum(t[-1] + np.arange(1, 41) * 0.1, 2.0)
    expected = (4 * future - future**2)[:, None] * direction
    np.testing.assert_allclose(ConstantAccelPredictor().predict(h, 40, 0.1), expected, atol=1e-11)


@pytest.mark.parametrize("after_stop", [0.0, 0.2])
def test_ca_already_stopped_does_not_reverse_after_mixed_history(after_stop):
    scenario = make_scenario("lead_brake")
    agent = scenario.agents[0]
    stop = agent.t[np.flatnonzero(agent.v == 0)[0]]
    h = history_until(agent, stop + after_stop)
    np.testing.assert_allclose(
        ConstantAccelPredictor().predict(h, 30, 0.1), np.tile([h.x[-1], h.y[-1]], (30, 1))
    )


@pytest.mark.parametrize("predictor", [ConstantAccelPredictor, CTRVPredictor])
@pytest.mark.parametrize("n", [1, 2])
def test_l2_short_history_cv_fallback(predictor, n):
    t = np.arange(n) * 0.1
    h = track(t, np.column_stack((4 * t, -t)), np.full(n, 4), np.zeros(n))
    np.testing.assert_allclose(
        predictor().predict(h, 4, 0.1), ConstantVelocityPredictor().predict(h, 4, 0.1)
    )


@pytest.mark.parametrize("rate", [-0.8, 0.8, 1e-7])
def test_ctrv_exact_circle_wrapped_heading_and_irregular_sampling(rate):
    t = np.array([0, 0.07, 0.2, 0.31, 0.6, 0.9])
    heading0, speed = 3.05, 7.0

    def positions(times):
        delta = rate * times
        length = speed * times * np.sinc(delta / (2 * np.pi))
        return length[:, None] * np.column_stack(
            (np.cos(heading0 + delta / 2), np.sin(heading0 + delta / 2))
        )

    heading = (heading0 + rate * t + np.pi) % (2 * np.pi) - np.pi
    h = track(t, positions(t), np.full(len(t), speed), heading)
    out = CTRVPredictor().predict(h, 40, 0.1)
    np.testing.assert_allclose(out, positions(t[-1] + np.arange(1, 41) * 0.1), atol=1e-10)


@pytest.mark.parametrize("predictor", [ConstantAccelPredictor, CTRVPredictor, OraclePredictor])
def test_l2_inputs_zero_horizon_and_no_mutation(predictor):
    sc = make_scenario("cut_in", a_cutter=2)
    h = history_until(sc.agents[0], 2)
    model = predictor()
    model.bind(sc)
    before = [a.copy() for a in (h.t, h.x, h.y, h.v, h.heading, h.accel)]
    assert model.predict(h, 0, 0.1).shape == (0, 2)
    assert np.isfinite(model.predict(h, 20, 0.1)).all()
    for current, original in zip((h.t, h.x, h.y, h.v, h.heading, h.accel), before, strict=True):
        np.testing.assert_array_equal(current, original)
    for horizon, dt in [(-1, 0.1), (True, 0.1), (1.5, 0.1), (2, 0), (2, np.nan)]:
        with pytest.raises(ValueError):
            model.predict(h, horizon, dt)
    h.t[-1] = h.t[-2]
    with pytest.raises(ValueError, match="strictly increasing"):
        model.predict(h, 2, 0.1)


@pytest.mark.parametrize("predictor", [ConstantAccelPredictor, CTRVPredictor])
@pytest.mark.parametrize("m", [0, 1, True, 2.5])
def test_invalid_window(predictor, m):
    with pytest.raises(ValueError, match="m must"):
        predictor(m=m)


def test_ca_uses_only_recent_position_window():
    t = np.arange(20) * 0.1
    xy = np.column_stack((3 * t + t**2, t))
    h = track(t, xy)
    expected = ConstantAccelPredictor(m=5).predict(h, 20, 0.1)
    h.x[:-5] += 500
    h.accel = np.full((20, 2), np.nan)
    np.testing.assert_array_equal(ConstantAccelPredictor(m=5).predict(h, 20, 0.1), expected)


def test_oracle_identity_interpolation_endpoint_hold_and_rebind():
    t = np.array([0.0, 0.17, 0.6, 1.0])
    first = track(t, np.column_stack((t**2, -t)), agent_id="one")
    second = track(t, np.column_stack((10 + t, 2 * t)), agent_id="two")
    model = OraclePredictor()
    with pytest.raises(ValueError, match="bind"):
        model.predict(history_until(first, 0), 3, 0.1)
    sc = scenario_with(first, second)
    model.bind(sc)
    h = history_until(second, 0.17)
    expected = np.array([state_at(second, h.t[-1] + k * 0.2) for k in range(1, 8)])
    h.x[:] = -999  # Query uses ID and timestamp, not observed position as truth.
    np.testing.assert_allclose(model.predict(h, 7, 0.2), expected)
    model.bind(scenario_with(first))
    with pytest.raises(ValueError, match="no truth"):
        model.predict(h, 2, 0.1)


@pytest.mark.parametrize("duplicate", [True, False])
def test_oracle_rejects_ambiguous_identity(duplicate):
    sc = make_scenario("crossing")
    if duplicate:
        sc.agents += sc.agents
    else:
        sc.agents[0].agent_id = ""
    with pytest.raises(ValueError, match="unique non-empty"):
        OraclePredictor().bind(sc)


@pytest.mark.parametrize("age", [0.2, 0.25])
@pytest.mark.parametrize("model", [ConstantAccelPredictor, CTRVPredictor, OraclePredictor])
def test_l2_assessor_aligns_delayed_observations(model, age):
    t = np.arange(61) * 0.1
    agent = track(t, np.column_stack((10 * t, np.zeros(len(t)))), np.full(len(t), 10))
    h = history_until(agent, 1.0)
    now = 1.0 + age
    future = np.column_stack((10 * (now + np.arange(1, 11) * 0.1), np.zeros(10)))
    ctx = RiskContext(
        now, future[0], 0, 10, future, np.array([[0, 0], [500, 0]]), 3.5, 10, 0.1, 0.01
    )
    assessor = PredictiveAssessor(model())
    assessor.bind(scenario_with(agent))
    assert assessor.assess(ctx, h).ttc == pytest.approx(0.1)


@pytest.mark.parametrize("system", ["ca", "ctrv", "oracle"])
@pytest.mark.parametrize("kind", ["cut_in", "lead_brake", "crossing"])
def test_l2_systems_run_through_existing_simulator(system, kind):
    sc = make_scenario(kind)
    log = run_sim(sc, make_system(system), TTCThresholdPolicy(), duration=sc.params["duration"])
    assert np.isfinite(log.ego_x).all()
    assert (log.ego_v >= 0).all()
    assert not np.isnan(log.ttc).any()
    assert log.t[-1] == pytest.approx(sc.params["duration"])


def test_ca_beats_cv_during_observed_constant_lead_braking():
    sc = make_scenario("lead_brake")
    sl = next(s for s in prediction_slices(sc, horizon=1) if s.t0 == 3.0)
    cv = ConstantVelocityPredictor().predict(sl.history, 10, sc.dt)
    ca = ConstantAccelPredictor().predict(sl.history, 10, sc.dt)
    assert np.linalg.norm(ca - sl.truth, axis=1).mean() < 0.01
    assert np.linalg.norm(cv - sl.truth, axis=1).mean() > 0.5


def test_exp01_cli_exports_all_l2_models(tmp_path):
    root = Path(__file__).resolve().parents[1]
    subprocess.run(
        [
            sys.executable,
            str(root / "experiments/exp01_predictor_ade.py"),
            "--output-dir",
            str(tmp_path),
        ],
        cwd=tmp_path,
        check=True,
        capture_output=True,
        text=True,
    )
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert summary["models"] == ["cv", "ca", "ctrv", "oracle"]
    assert summary["oracle_max_error_m"] < 1e-10
    for name in ["prediction_errors.csv", "predictor_ade_fde.csv", "exp01_predictor_ade.png"]:
        assert (tmp_path / name).stat().st_size > 100

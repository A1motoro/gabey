"""Round 4: P's completed L1 systems through the unchanged C/D simulation APIs."""

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from experiments.l1_fixtures import cut_in_fixture
from pbs.eval.counterfactual import is_benign
from pbs.eval.metrics import evaluate
from pbs.risk import NullAssessor
from pbs.sim.policy import NeverBrakePolicy, TTCThresholdPolicy
from pbs.sim.simulator import run_sim
from pbs.systems import make_system
from tests.test_scenarios import straight_scenario


@pytest.mark.parametrize("system", ["reactive", "cv"])
@pytest.mark.parametrize("lead_v,benign", [(10.0, False), (20.0, True)])
def test_l1_straight_scenario_braking_and_counterfactual(system, lead_v, benign):
    scenario = straight_scenario(lead_v)
    assert is_benign(scenario) == benign
    log = run_sim(scenario, make_system(system), TTCThresholdPolicy())
    metrics = evaluate(log, benign)
    assert not metrics["collision"]
    assert not metrics["false_brake"]
    assert (metrics["brake_onset_t"] is None) == benign
    assert np.isfinite(log.ego_v).all()
    assert (log.ego_v >= 0).all()


@pytest.mark.parametrize("system", ["reactive", "cv"])
def test_l1_without_agents_cruises(system):
    scenario = straight_scenario()
    scenario.agents = []
    log = run_sim(scenario, make_system(system), TTCThresholdPolicy(), duration=1.0)
    assert set(log.action) == {"cruise"}
    assert np.isinf(log.ttc).all()
    assert not log.collision


@pytest.mark.parametrize("system", ["reactive", "cv"])
def test_l1_repeated_run_is_deterministic_and_does_not_change_scenario(system):
    scenario = cut_in_fixture()
    original = [(a.x.copy(), a.y.copy()) for a in scenario.agents]
    assessor = make_system(system)
    first = run_sim(scenario, assessor, TTCThresholdPolicy(), duration=6.0)
    second = run_sim(scenario, assessor, TTCThresholdPolicy(), duration=6.0)
    np.testing.assert_array_equal(first.ego_x, second.ego_x)
    np.testing.assert_array_equal(first.ttc, second.ttc)
    assert first.action == second.action
    for agent, (x, y) in zip(scenario.agents, original, strict=True):
        np.testing.assert_array_equal(agent.x, x)
        np.testing.assert_array_equal(agent.y, y)


@pytest.mark.parametrize("system", ["reactive", "cv"])
def test_l1_delayed_straight_history_remains_safe(system):
    log = run_sim(straight_scenario(), make_system(system), TTCThresholdPolicy(), latency=0.3)
    assert not log.collision
    assert "brake" in log.action
    assert not np.isnan(log.ttc).any()


@pytest.mark.parametrize("system", ["reactive", "cv"])
def test_l1_multiagent_takes_nearest_risk(system):
    near = straight_scenario()
    far = straight_scenario()
    far.agents[0].x += 40
    far.agents[0].agent_id = "far"
    single = run_sim(near, make_system(system), TTCThresholdPolicy(), duration=1.0, horizon=100)
    near.agents += far.agents
    combined = run_sim(near, make_system(system), TTCThresholdPolicy(), duration=1.0, horizon=100)
    np.testing.assert_allclose(combined.ttc, single.ttc)
    assert combined.action == single.action


def test_l1_cut_in_cv_brakes_before_reactive():
    scenario = cut_in_fixture()
    no_brake = run_sim(scenario, NullAssessor(), NeverBrakePolicy(), duration=6.0)
    assert no_brake.collision
    onsets = {}
    for system in ("reactive", "cv"):
        log = run_sim(scenario, make_system(system), TTCThresholdPolicy(), duration=6.0)
        assert not log.collision
        onsets[system] = evaluate(log, benign=False)["brake_onset_t"]
    assert onsets["cv"] == pytest.approx(1.7)
    assert onsets["reactive"] == pytest.approx(2.0)
    assert onsets["reactive"] - onsets["cv"] == pytest.approx(0.3)


@pytest.mark.parametrize("system", ["reactive", "cv"])
def test_l1_benign_cut_in_is_not_a_false_brake(system):
    scenario = cut_in_fixture(rel_v=-2.0)
    assert is_benign(scenario, duration=6.0)
    log = run_sim(scenario, make_system(system), TTCThresholdPolicy(), duration=6.0)
    assert not evaluate(log, benign=True)["false_brake"]
    assert not log.collision


def test_l1_evidence_cli_generates_accuracy_and_timeline(tmp_path):
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [
            sys.executable,
            str(root / "experiments/exp00_l1_baseline.py"),
            "--output-dir",
            str(tmp_path),
        ],
        cwd=tmp_path,
        text=True,
        capture_output=True,
        check=True,
    )
    assert "Artifacts:" in result.stdout
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert summary["cv_lead_time_s"] == pytest.approx(0.3)
    assert not summary["systems"]["cv"]["collision"]
    for name in [
        "cv_ade_fde.csv",
        "cv_ade_fde.png",
        "cut_in_timeline.png",
        "reactive_timeline.csv",
        "cv_timeline.csv",
    ]:
        assert (tmp_path / name).stat().st_size > 100

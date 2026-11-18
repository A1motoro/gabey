"""D's experiment matrix: schema, cached benign labels, and the L2 reductions."""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pytest

from pbs.eval.counterfactual import is_benign
from pbs.eval.plots import plot_boundaries
from pbs.eval.sweep import (
    failure_boundary,
    lead_times,
    rescue_rate,
    run_matrix,
    safe_area,
    select_operating_point,
    tradeoff,
)
from tests.test_scenarios import straight_scenario

REQUIRED = {
    "scenario_id",
    "kind",
    "system",
    "threshold",
    "latency",
    "benign",
    "collision",
    "false_brake",
    "min_dist",
    "brake_onset_t",
    "seed",
    "git_rev",
}


def test_run_matrix_schema_and_cached_benign(monkeypatch):
    calls = {"n": 0}
    real = is_benign

    def wrapped(scenario, **kwargs):
        calls["n"] += 1
        return real(scenario, **kwargs)

    monkeypatch.setattr("pbs.eval.sweep.is_benign", wrapped)
    slow = straight_scenario(lead_v=10.0)
    fast = straight_scenario(lead_v=20.0)
    slow.params = {"gap_d": 30.0, "rel_v": 5.0}
    fast.params = {"gap_d": 30.0, "rel_v": -5.0}
    frame = run_matrix(
        ["reactive", "cv"],
        [slow, fast],
        [2.0],
        sim_kwargs={"duration": 8.0},
        seed=3,
    )
    assert calls["n"] == 2
    assert REQUIRED <= set(frame.columns)
    assert {"gap_d", "rel_v"} <= set(frame.columns)
    assert len(frame) == 4
    assert set(frame["seed"]) == {3}
    assert frame["git_rev"].astype(str).str.len().min() > 0
    labels = frame.groupby("scenario_id")["benign"].first()
    assert set(labels) == {False, True}
    assert frame.loc[frame["rel_v"] < 0, "benign"].all()
    assert not frame.loc[frame["rel_v"] > 0, "benign"].any()


def test_tradeoff_rates_ignore_the_other_class():
    frame = pd.DataFrame(
        [
            _row("a", "cv", 1.0, benign=False, collision=True, false_brake=False),
            _row("b", "cv", 1.0, benign=False, collision=False, false_brake=False),
            _row("c", "cv", 1.0, benign=True, collision=False, false_brake=True),
            _row("d", "cv", 1.0, benign=True, collision=False, false_brake=False),
            _row("a", "cv", 2.0, benign=False, collision=False, false_brake=False),
            _row("c", "cv", 2.0, benign=True, collision=False, false_brake=True),
            _row("d", "cv", 2.0, benign=True, collision=False, false_brake=True),
        ]
    )
    table = tradeoff(frame).sort_values("threshold")
    low, high = table.iloc[0], table.iloc[1]
    assert low["collision_rate"] == 0.5
    assert low["false_brake_rate"] == 0.5
    assert high["collision_rate"] == 0.0
    assert high["false_brake_rate"] == 1.0
    assert low["n_dangerous"] == 2 and low["n_benign"] == 2


def test_operating_point_respects_the_false_brake_budget():
    table = pd.DataFrame(
        [
            {
                "system": "cv",
                "latency": 0.0,
                "threshold": 1.0,
                "collision_rate": 0.4,
                "false_brake_rate": 0.0,
            },
            {
                "system": "cv",
                "latency": 0.0,
                "threshold": 2.0,
                "collision_rate": 0.2,
                "false_brake_rate": 0.04,
            },
            {
                "system": "cv",
                "latency": 0.0,
                "threshold": 3.0,
                "collision_rate": 0.1,
                "false_brake_rate": 0.2,
            },
        ]
    )
    chosen = select_operating_point(table, max_false_brake=0.05)
    assert float(chosen.iloc[0]["threshold"]) == 2.0
    assert bool(chosen.iloc[0]["met_budget"])


def test_boundary_rescue_and_safe_area():
    rows = []
    for gap, rel, benign, collision in (
        (10.0, 4.0, False, True),
        (20.0, 4.0, False, False),
        (10.0, -2.0, True, False),
        (20.0, -2.0, True, False),
    ):
        rows.append(_row(f"{gap}-{rel}", "cv", 2.0, benign, collision, False, gap, rel))
    frame = pd.DataFrame(rows)
    cells = failure_boundary(frame, "cv")
    assert len(cells) == 4
    assert rescue_rate(frame, "cv") == 0.5
    assert safe_area(frame, "cv") == 0.75


def test_lead_time_keeps_one_sided_brakes():
    frame = pd.DataFrame(
        [
            _onset("a", "reactive", 3.0),
            _onset("a", "cv", 2.5),
            _onset("b", "reactive", np.nan),
            _onset("b", "cv", 1.0),
        ]
    )
    out = lead_times(frame)
    assert len(out) == 2
    defined = out.loc[out["scenario_id"] == "a"].iloc[0]
    missing = out.loc[out["scenario_id"] == "b"].iloc[0]
    assert defined["lead_time"] == pytest.approx(0.5)
    assert bool(defined["defined"])
    assert missing["brake_status"] == "predictive_only"
    assert np.isnan(missing["lead_time"])


def test_plot_boundaries_returns_axes():
    rows = []
    for gap in (10.0, 20.0, 30.0):
        for rel in (0.0, 5.0, 10.0):
            rows.append(
                _row(
                    f"{gap}-{rel}",
                    "reactive",
                    2.0,
                    benign=rel == 0.0,
                    collision=gap + rel < 25.0,
                    false_brake=False,
                    gap_d=gap,
                    rel_v=rel,
                )
            )
    fig, ax = plt.subplots()
    plot_boundaries(pd.DataFrame(rows), ["reactive"], ax=ax, threshold=2.0)
    assert ax.get_xlabel() == "gap_d [m]"
    plt.close(fig)


def _row(sid, system, threshold, benign, collision, false_brake, gap_d=10.0, rel_v=1.0):
    return {
        "scenario_id": sid,
        "kind": "cut_in",
        "gap_d": gap_d,
        "rel_v": rel_v,
        "system": system,
        "threshold": threshold,
        "latency": 0.0,
        "benign": benign,
        "collision": collision,
        "false_brake": false_brake,
        "min_dist": 1.0,
        "brake_onset_t": 1.0,
        "seed": 0,
        "git_rev": "abc",
    }


def _onset(sid, system, onset):
    row = _row(sid, system, 2.0, False, False, False)
    row["brake_onset_t"] = onset
    return row

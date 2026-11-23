"""D's experiment matrix: schema, cached benign labels, and the L2 reductions."""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pytest

from pbs.eval.counterfactual import is_benign, never_brake_label
from pbs.eval.plots import plot_boundaries
from pbs.eval.sweep import (
    failure_boundary,
    late_contact_outcomes,
    lead_times,
    rescue_rate,
    run_matrix,
    safe_area,
    scenario_id,
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
    real = never_brake_label

    def wrapped(scenario, decision_duration=None, **kwargs):
        calls["n"] += 1
        return real(scenario, decision_duration, **kwargs)

    monkeypatch.setattr("pbs.eval.sweep.never_brake_label", wrapped)
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
    assert set(frame["label"]) == {"dangerous", "benign"}
    assert (frame["decision_duration"] == 8.0).all()
    assert (frame["outcome_duration"] == 8.0).all()
    dangerous = frame.loc[frame["label"] == "dangerous"]
    assert np.allclose(dangerous["never_brake_contact_t"], 5.0)
    assert frame.loc[frame["benign"], "never_brake_contact_t"].isna().all()


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


def test_operating_point_without_eligible_threshold_minimizes_false_brakes():
    table = pd.DataFrame(
        [
            {
                "system": "cv",
                "latency": 0.0,
                "threshold": 1.0,
                "collision_rate": 0.4,
                "false_brake_rate": 0.1,
            },
            {
                "system": "cv",
                "latency": 0.0,
                "threshold": 2.0,
                "collision_rate": 0.0,
                "false_brake_rate": 0.2,
            },
        ]
    )
    row = select_operating_point(table, max_false_brake=0.05).iloc[0]
    assert row["threshold"] == 1.0
    assert not row["met_budget"]


def test_scenario_identity_includes_interior_motion_and_geometry():
    import copy

    first = straight_scenario()
    same = copy.deepcopy(first)
    assert scenario_id(first) == scenario_id(same)
    changed = copy.deepcopy(first)
    changed.agents[0].y[10] += 5.0
    assert scenario_id(first) != scenario_id(changed)
    changed = copy.deepcopy(first)
    changed.lane_width += 1.0
    assert scenario_id(first) != scenario_id(changed)
    changed = copy.deepcopy(first)
    changed.ego_path[1, 1] += 1.0
    assert scenario_id(first) != scenario_id(changed)


def test_benign_cache_separates_same_endpoints_with_different_interior(monkeypatch):
    import copy

    first = straight_scenario()
    second = copy.deepcopy(first)
    second.agents[0].y[10] += 5.0
    calls = []

    def label(sc, decision_duration=None, **kwargs):
        calls.append(sc)
        return ("benign", None) if sc is second else ("dangerous", 0.5)

    monkeypatch.setattr("pbs.eval.sweep.never_brake_label", label)
    frame = run_matrix(["cv"], [first, second], [2], sim_kwargs={"duration": 1})
    assert len(calls) == 2
    assert frame["scenario_id"].nunique() == 2
    assert frame["benign"].tolist() == [False, True]


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


@pytest.mark.parametrize("system", ["reactive", "ca"])
def test_collision_cross_markers_remain_visible(system):
    frame = pd.DataFrame(
        [
            _row("a", system, 2.0, False, True, False, gap_d=10.0, rel_v=2.0),
            _row("b", system, 2.0, True, False, False, gap_d=20.0, rel_v=-2.0),
        ]
    )
    fig, ax = plt.subplots()
    plot_boundaries(frame, [system], ax=ax)
    colors = ax.collections[0].get_edgecolors()
    assert len(colors) and (colors[:, 3] > 0).all()
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


def test_run_matrix_l2_hysteresis_and_obb_schema():
    slow = straight_scenario(lead_v=10.0)
    slow.params = {"gap_d": 30.0, "rel_v": 5.0}
    frame = run_matrix(
        ["reactive", "cv"],
        [slow],
        [2.0],
        sim_kwargs={"duration": 8.0, "horizon": 40, "r_safe": None},
        policy_kind="hysteresis",
        hysteresis_gap=1.0,
    )
    expected_cols = {
        "policy",
        "t_on",
        "t_off",
        "geometry",
        "horizon",
        "first_collision_t",
        "min_dist_until_contact",
    }
    assert expected_cols <= set(frame.columns)
    assert set(frame["policy"]) == {"hysteresis"}
    assert set(frame["geometry"]) == {"obb"}
    assert set(frame["horizon"]) == {40}
    assert frame["t_on"].tolist() == [2.0, 2.0]
    assert frame["t_off"].tolist() == [3.0, 3.0]


def test_evaluate_truncates_brake_onset_at_first_collision():
    from pbs.eval.metrics import evaluate
    from pbs.types import SimLog

    log = SimLog(
        t=np.array([0.0, 0.1, 0.2, 0.3]),
        ego_x=np.array([0.0, 1.0, 2.0, 3.0]),
        ego_y=np.zeros(4),
        ego_v=np.full(4, 10.0),
        ego_a=np.zeros(4),
        action=["cruise", "cruise", "brake", "brake"],
        ttc=np.array([np.inf, np.inf, 0.5, 0.5]),
        min_dist=0.0,
        collision=True,
        executed_steps=3,
        first_collision_t=0.15,
        min_dist_until_contact=5.0,
    )
    metrics = evaluate(log, benign=False)
    assert metrics["brake_onset_t"] is None
    assert metrics["first_collision_t"] == pytest.approx(0.15)
    assert metrics["min_dist_until_contact"] == pytest.approx(5.0)


def test_plot_timeline_ignores_unexecuted_terminal_action():
    from pbs.eval.plots import plot_timeline
    from pbs.types import SimLog

    log = SimLog(
        t=np.array([0.0, 0.1, 0.2]),
        ego_x=np.array([0.0, 1.0, 2.0]),
        ego_y=np.zeros(3),
        ego_v=np.array([10.0, 10.0, 9.4]),
        ego_a=np.array([0.0, -6.0, 0.0]),
        action=["cruise", "brake", "brake"],
        ttc=np.array([np.inf, 1.0, 1.0]),
        min_dist=5.0,
        collision=False,
        executed_steps=2,
    )
    fig, ax = plt.subplots()
    plot_timeline(log, ax=ax)
    assert len(ax.patches) == 1
    plt.close(fig)


def test_evaluate_counts_only_brakes_inside_the_decision_window():
    from pbs.eval.metrics import evaluate
    from pbs.types import SimLog

    log = SimLog(
        t=np.array([0.0, 0.1, 0.2, 0.3]),
        ego_x=np.array([0.0, 1.0, 2.0, 3.0]),
        ego_y=np.zeros(4),
        ego_v=np.full(4, 10.0),
        ego_a=np.zeros(4),
        action=["cruise", "cruise", "brake", "brake"],
        ttc=np.array([np.inf, np.inf, 0.5, 0.5]),
        min_dist=5.0,
        collision=False,
        executed_steps=3,
    )
    assert evaluate(log, benign=True)["false_brake"]
    assert evaluate(log, benign=True, decision_end=0.3)["false_brake"]
    late = evaluate(log, benign=True, decision_end=0.2)
    assert not late["false_brake"]
    assert late["brake_onset_t"] == pytest.approx(0.2)
    assert not evaluate(log, benign=False, decision_end=0.3)["false_brake"]


def test_never_brake_label_has_three_classes_with_two_windows():
    # 25 m of clearance closes at 5, 3 and -5 m/s: contact at 5.0 s, 8.33 s, never.
    early, late, never = (straight_scenario(lead_v=v) for v in (10.0, 12.0, 20.0))
    assert never_brake_label(early, 7.0, duration=10.0) == ("dangerous", pytest.approx(5.0))
    label, contact = never_brake_label(late, 7.0, duration=10.0)
    assert label == "late_contact" and contact == pytest.approx(25.0 / 3.0, abs=0.01)
    assert never_brake_label(never, 7.0, duration=10.0) == ("benign", None)
    assert never_brake_label(late, duration=10.0)[0] == "dangerous"
    assert never_brake_label(late, duration=7.0) == ("benign", None)
    assert is_benign(late, duration=7.0)
    assert never_brake_label(late, 10.0, duration=10.0)[0] == "dangerous"


@pytest.mark.parametrize("decision", [0.0, -1.0, 10.5])
def test_never_brake_label_rejects_a_decision_window_outside_the_run(decision):
    with pytest.raises(ValueError):
        never_brake_label(straight_scenario(), decision, duration=10.0)


def test_never_brake_label_needs_the_run_duration_for_two_windows():
    with pytest.raises(ValueError):
        never_brake_label(straight_scenario(), 5.0)


def test_run_matrix_keeps_late_contact_out_of_both_rates():
    early, late, never = (straight_scenario(lead_v=v) for v in (10.0, 12.0, 20.0))
    for scenario, rel in ((early, 5.0), (late, 3.0), (never, -5.0)):
        scenario.params = {"gap_d": 30.0, "rel_v": rel}
    frame = run_matrix(
        ["cv"], [early, late, never], [2.0], sim_kwargs={"duration": 10.0}, decision_duration=7.0
    )
    rows = frame.set_index("rel_v")
    assert rows["label"].to_dict() == {5.0: "dangerous", 3.0: "late_contact", -5.0: "benign"}
    assert rows["benign"].to_dict() == {5.0: False, 3.0: False, -5.0: True}
    assert (frame["decision_duration"] == 7.0).all()
    assert (frame["outcome_duration"] == 10.0).all()
    # CV brakes for the late contact inside the decision window. It is not a false brake.
    assert rows.loc[3.0, "brake_onset_t"] == pytest.approx(6.4)
    assert not rows.loc[3.0, "false_brake"]
    assert not frame["collision"].any()
    table = tradeoff(frame).iloc[0]
    assert (table["n_dangerous"], table["n_benign"], table["n_late_contact"]) == (1, 1, 1)
    assert table["false_brake_rate"] == 0.0
    cells = failure_boundary(frame, "cv")
    assert cells["dangerous"].sum() == 1
    assert rescue_rate(frame, "cv") == 1.0
    outcome = late_contact_outcomes(frame).iloc[0]
    assert (outcome["n_late_contact"], outcome["braked_in_window"]) == (1, 1)
    assert (outcome["braked_later"], outcome["collided"]) == (0, 0)
    # With a 6 s decision window the same brake at 6.4 s comes after it.
    shorter = run_matrix(
        ["cv"], [late], [2.0], sim_kwargs={"duration": 10.0}, decision_duration=6.0
    )
    outcome = late_contact_outcomes(shorter).iloc[0]
    assert (outcome["braked_in_window"], outcome["braked_later"]) == (0, 1)


def test_equal_windows_are_the_single_window():
    scenarios = [straight_scenario(lead_v=v) for v in (10.0, 12.0, 20.0)]
    kwargs = {"sim_kwargs": {"duration": 10.0}}
    one = run_matrix(["cv", "reactive"], scenarios, [2.0], **kwargs)
    two = run_matrix(["cv", "reactive"], scenarios, [2.0], decision_duration=10.0, **kwargs)
    pd.testing.assert_frame_equal(one, two)
    assert set(one["label"]) == {"dangerous", "benign"}


def test_tradeoff_reads_two_class_tables_without_a_label_column():
    frame = pd.DataFrame(
        [
            _row("a", "cv", 2.0, benign=False, collision=True, false_brake=False),
            _row("b", "cv", 2.0, benign=True, collision=False, false_brake=True),
        ]
    )
    table = tradeoff(frame).iloc[0]
    assert (table["n_dangerous"], table["n_benign"], table["n_late_contact"]) == (1, 1, 0)
    assert late_contact_outcomes(frame).empty

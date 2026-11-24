import numpy as np
import pandas as pd
import pytest

from experiments.check_outcome_window import compare, paired_ids
from experiments.matrix_setup import (
    L1_SIM_KWARGS,
    MATRIX_KWARGS,
    QUICK_THRESHOLDS,
    SIM_KWARGS,
    experiment_config,
)
from experiments.scenario_sets import (
    DECISION_DURATION,
    LOG_DURATION,
    LOG_PAD,
    OUTCOME_DURATION,
    abort_cut_ins,
    boundary_grid,
    kind_label,
    tradeoff_set,
)
from pbs.data import make_scenario
from pbs.eval.counterfactual import is_benign, never_brake_label
from pbs.eval.sweep import run_matrix, scenario_id


def test_boundary_grid_is_make_scenario_with_padded_logs():
    grid = boundary_grid(quick=True)
    assert len(grid) == 25
    first = grid[0]
    expected = make_scenario(
        "cut_in",
        gap_d=first.params["gap_d"],
        rel_v=first.params["rel_v"],
        lc_T=3.0,
        duration=LOG_DURATION,
    )
    np.testing.assert_allclose(first.agents[0].y, expected.agents[0].y)
    assert LOG_PAD == 4.0
    assert first.agents[0].t[-1] >= OUTCOME_DURATION + 4.0 - 1e-9


def test_windows_are_fifteen_seconds_of_decisions_and_one_horizon_more_of_outcome():
    sim, matrix, meta = experiment_config()
    assert (DECISION_DURATION, OUTCOME_DURATION, LOG_DURATION) == (15.0, 19.0, 23.0)
    assert sim["duration"] == OUTCOME_DURATION
    assert matrix["decision_duration"] == DECISION_DURATION
    assert OUTCOME_DURATION - DECISION_DURATION == sim["horizon"] * 0.1
    assert (meta["decision_duration_s"], meta["outcome_duration_s"]) == (15.0, 19.0)


def test_legacy_l1_keeps_one_fifteen_second_window():
    sim, matrix, meta = experiment_config(legacy_l1=True)
    assert sim["duration"] == 15.0
    assert "decision_duration" not in matrix
    assert (meta["decision_duration_s"], meta["outcome_duration_s"]) == (15.0, 15.0)


def test_tradeoff_set_keeps_a_grid_and_counts_aborts():
    scenarios, counts = tradeoff_set()
    assert counts["default_grid"] == 96
    assert counts["abort_cut_ins"] == 6
    families = {kind_label(s) for s in scenarios}
    assert {"cut_in", "lead_brake", "crossing", "cut_in_ca", "cut_in_abort"} <= families


def test_aborts_are_benign():
    for scenario in abort_cut_ins():
        assert min(scenario.agents[0].y) > 2.5
        assert is_benign(scenario, **SIM_KWARGS)
        assert is_benign(scenario, **L1_SIM_KWARGS)


def _cut_in(gap_d, rel_v):
    return make_scenario("cut_in", gap_d=gap_d, rel_v=rel_v, lc_T=3.0, duration=LOG_DURATION)


def _lead_brake(init_gap, decel):
    return make_scenario("lead_brake", init_gap=init_gap, decel=decel, duration=LOG_DURATION)


def _label(scenario):
    return never_brake_label(scenario, DECISION_DURATION, **SIM_KWARGS)


def test_dangerous_is_the_fifteen_second_label():
    for scenario in (_cut_in(20.0, 4.0), _cut_in(20.0, 1.0), _lead_brake(35.0, 0.3)):
        dangerous = _label(scenario)[0] == "dangerous"
        assert dangerous == (not is_benign(scenario, **{**SIM_KWARGS, "duration": 15.0}))


def test_contact_between_the_two_windows_is_its_own_class():
    assert _label(_cut_in(20.0, 4.0)) == ("dangerous", pytest.approx(3.75, abs=0.01))
    assert _label(_cut_in(20.0, -4.0)) == ("benign", None)
    assert _label(_lead_brake(35.0, 0.3)) == ("late_contact", pytest.approx(15.142, abs=0.001))
    assert _label(_lead_brake(50.0, 0.3)) == ("late_contact", pytest.approx(18.320, abs=0.001))
    # Contact after the outcome window (23.4 s) is still benign: no 15 s decision can see it.
    assert _label(_lead_brake(80.0, 0.3)) == ("benign", None)


def test_touching_at_fifteen_seconds_is_late_contact_not_dangerous():
    # 15 m of clearance closes at 1 m/s. At t = 15.0 the two boxes touch with zero overlap,
    # which is not a collision, so the 15 s run ends without one. The first overlap is in
    # the step that starts at 15.0, and its timestamp is 15.0.
    scenario = _cut_in(20.0, 1.0)
    assert is_benign(scenario, **{**SIM_KWARGS, "duration": 15.0})
    assert _label(scenario) == ("late_contact", pytest.approx(15.0))


def test_oracle_has_no_false_brake_and_brakes_for_late_contact():
    late = [_cut_in(20.0, 1.0), _lead_brake(35.0, 0.3), _lead_brake(50.0, 0.3)]
    benign = [_cut_in(20.0, -4.0), _lead_brake(80.0, 0.3), *abort_cut_ins()[:2]]
    frame = run_matrix(
        ["oracle"], late + benign, QUICK_THRESHOLDS, sim_kwargs=SIM_KWARGS, **MATRIX_KWARGS
    )
    assert not frame["false_brake"].any()
    assert frame.loc[frame["benign"], "brake_onset_t"].isna().all()
    assert set(frame.loc[~frame["benign"], "label"]) == {"late_contact"}
    by_scenario = frame.loc[frame["threshold"] == 2.0].set_index("scenario_id")
    onsets = [by_scenario.loc[scenario_id(s), "brake_onset_t"] for s in late]
    # Two brakes fall inside the decision window; under one 15 s window both were false brakes.
    assert onsets[0] == pytest.approx(13.1) and onsets[1] == pytest.approx(13.2)
    assert onsets[2] > DECISION_DURATION


def test_collision_postponed_past_fifteen_seconds_is_a_collision():
    scenario = _lead_brake(100.0, 1.0)
    assert _label(scenario) == ("dangerous", pytest.approx(14.78, abs=0.01))
    row = run_matrix(["cv"], [scenario], [1.0], sim_kwargs=SIM_KWARGS, **MATRIX_KWARGS).iloc[0]
    assert row["brake_onset_t"] < DECISION_DURATION < row["first_collision_t"]
    assert row["collision"]
    one_window = run_matrix(
        ["cv"],
        [scenario],
        [1.0],
        sim_kwargs={**SIM_KWARGS, "duration": DECISION_DURATION},
        policy_kind="hysteresis",
    ).iloc[0]
    assert not one_window["collision"]


def test_paired_ids_matches_scenarios_across_log_lengths():
    short = [_cut_in(20.0, 1.0), _lead_brake(35.0, 0.3)]
    long = [
        make_scenario("cut_in", gap_d=20.0, rel_v=1.0, lc_T=3.0, duration=29.0),
        make_scenario("lead_brake", init_gap=35.0, decel=0.3, duration=29.0),
    ]
    pairs = paired_ids(short, long)
    assert pairs == {scenario_id(b): scenario_id(a) for a, b in zip(short, long, strict=True)}
    assert set(pairs).isdisjoint(pairs.values())
    with pytest.raises(ValueError):
        paired_ids(short, long[::-1])
    with pytest.raises(ValueError):
        paired_ids(short, long[:1])


def test_compare_reports_label_collision_and_decision_changes():
    def row(sid, label, collision, onset, contact=np.nan, false_brake=False, never=np.nan):
        return {
            "scenario_id": sid,
            "kind": "cut_in",
            "system": "cv",
            "threshold": 2.0,
            "latency": 0.0,
            "label": label,
            "never_brake_contact_t": never,
            "decision_duration": 15.0,
            "collision": collision,
            "first_collision_t": contact,
            "false_brake": false_brake,
            "brake_onset_t": onset,
        }

    saved = pd.DataFrame(
        [
            row("same", "dangerous", False, 3.0, never=5.0),
            row("relabelled", "benign", False, 14.0, false_brake=True),
            row("postponed", "dangerous", False, 14.5, never=14.8),
            row("late_brake", "benign", False, 18.0),
        ]
    )
    extended = pd.DataFrame(
        [
            row("same+", "dangerous", False, 3.0, never=5.0),
            row("relabelled+", "late_contact", False, 14.0, never=21.0),
            row("postponed+", "dangerous", True, 14.5, contact=20.5, never=14.8),
            row("late_brake+", "benign", False, 18.0),
        ]
    )
    pairs = {f"{sid}+": sid for sid in saved["scenario_id"]}
    out = compare(saved, extended, pairs)
    assert (out["n_runs"], out["n_scenarios"]) == (4, 4)
    assert [r["scenario_id"] for r in out["label_changes"]] == ["relabelled"]
    assert out["label_changes"][0]["never_brake_contact_t_ext"] == 21.0
    assert [r["scenario_id"] for r in out["collision_changes"]] == ["postponed"]
    assert out["collision_changes"][0]["first_collision_t_ext"] == 20.5
    assert out["decision_changes"] == 0
    assert out["false_brakes_on_changed_labels"] == 1
    extended.loc[0, "brake_onset_t"] = 3.1
    assert compare(saved, extended, pairs)["decision_changes"] == 1
    with pytest.raises(ValueError):
        compare(saved, extended.iloc[:3], pairs)

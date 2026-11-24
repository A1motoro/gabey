import numpy as np
import pandas as pd

from experiments.check_label_window import label_window, split_false_brakes
from experiments.matrix_setup import L1_SIM_KWARGS, SIM_KWARGS
from experiments.scenario_sets import (
    LOG_DURATION,
    LOG_PAD,
    SIM_DURATION,
    abort_cut_ins,
    boundary_grid,
    kind_label,
    tradeoff_set,
)
from pbs.data import make_scenario
from pbs.eval.counterfactual import is_benign
from pbs.eval.sweep import scenario_id


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
    assert first.agents[0].t[-1] >= SIM_DURATION + 4.0 - 1e-9


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


def test_label_window_flags_contact_between_scored_end_and_log_end():
    closing_late = _cut_in(20.0, 1.0)
    closing_early = _cut_in(20.0, 4.0)
    opening = _cut_in(20.0, -4.0)
    table = label_window([closing_late, closing_early, opening]).set_index("scenario_id")
    late, early, never = (table.loc[scenario_id(s)] for s in (closing_late, closing_early, opening))
    assert late["benign"] and late["late_contact"]
    assert SIM_DURATION <= late["contact_t_in_log"] <= LOG_DURATION
    assert not early["benign"] and not early["late_contact"]
    assert early["contact_t_in_log"] < SIM_DURATION
    assert never["benign"] and not never["late_contact"]
    assert pd.isna(never["contact_t_in_log"])


def test_split_false_brakes_separates_late_contact_aborts_and_other():
    def row(sid, family, false_brake, benign=True):
        return {
            "scenario_id": sid,
            "family": family,
            "system": "cv",
            "threshold": 2.0,
            "benign": benign,
            "false_brake": false_brake,
        }

    frame = pd.DataFrame(
        [
            row("late", "cut_in", True),
            row("abort", "cut_in_abort", True),
            row("other", "crossing", True),
            row("quiet", "cut_in", False),
            row("dangerous", "cut_in", False, benign=False),
        ]
    )
    split = split_false_brakes(frame, {"late"}).iloc[0]
    assert split["n_benign"] == 4
    assert split["false_brakes"] == 3
    assert (split["on_late_contact"], split["on_aborts"], split["other"]) == (1, 1, 1)
    assert split["n_benign_without_late_contact"] == 3

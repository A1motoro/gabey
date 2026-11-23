import numpy as np

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

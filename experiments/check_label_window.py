"""How many of exp02's false brakes sit at the end of the scoring window? (D)

``benign`` means "no contact within SIM_DURATION when the ego never brakes"
(docs/scenarios.md §10). The logs run LOG_PAD longer so that Oracle can look past the
last scored frame. This script repeats the never-brake run over the whole log and
splits the false brakes of a saved exp02 matrix by whether the benign label survives.
It changes no label and no metric.
Output: outputs/logs/exp02_label_window.json
"""

import argparse
import json
from pathlib import Path

import pandas as pd

try:
    from matrix_setup import SIM_KWARGS, write_json
    from scenario_sets import LOG_DURATION, SIM_DURATION, boundary_grid, kind_label, tradeoff_set
except ModuleNotFoundError:
    from experiments.matrix_setup import SIM_KWARGS, write_json
    from experiments.scenario_sets import (
        LOG_DURATION,
        SIM_DURATION,
        boundary_grid,
        kind_label,
        tradeoff_set,
    )

from pbs.eval.sweep import scenario_id
from pbs.risk.assessors import NullAssessor
from pbs.sim.policy import NeverBrakePolicy
from pbs.sim.simulator import run_sim
from pbs.types import Scenario

ROOT = Path(__file__).resolve().parents[1]
LABEL_COLUMNS = ["scenario_id", "family", "benign", "contact_t_in_log", "late_contact"]


def label_window(scenarios: list[Scenario], sim_kwargs=None) -> pd.DataFrame:
    """Never-brake label inside the scoring window and first contact over the whole log."""
    scored = dict(sim_kwargs or SIM_KWARGS)
    whole_log = {**scored, "duration": LOG_DURATION}
    rows = []
    for scenario in scenarios:
        short = run_sim(scenario, NullAssessor(), NeverBrakePolicy(), **scored)
        long = run_sim(scenario, NullAssessor(), NeverBrakePolicy(), **whole_log)
        rows.append(
            {
                "scenario_id": scenario_id(scenario),
                "family": kind_label(scenario),
                "benign": not short.collision,
                "contact_t_in_log": long.first_collision_t,
                "late_contact": bool(long.collision and not short.collision),
            }
        )
    return pd.DataFrame(rows, columns=LABEL_COLUMNS)


def split_false_brakes(frame: pd.DataFrame, late_ids: set[str]) -> pd.DataFrame:
    """Per (system, threshold): false brakes on late-contact scenarios vs the rest."""
    benign = frame.loc[frame["benign"].astype(bool)].copy()
    benign["false_brake"] = benign["false_brake"].astype(bool)
    benign["late"] = benign["scenario_id"].isin(late_ids)
    benign["abort"] = benign["family"] == "cut_in_abort"
    rows = []
    for (system, threshold), group in benign.groupby(["system", "threshold"], sort=False):
        braked = group.loc[group["false_brake"]]
        rows.append(
            {
                "system": system,
                "threshold": float(threshold),
                "n_benign": int(len(group)),
                "false_brakes": int(len(braked)),
                "on_late_contact": int(braked["late"].sum()),
                "on_aborts": int((braked["abort"] & ~braked["late"]).sum()),
                "other": int((~braked["abort"] & ~braked["late"]).sum()),
                "n_benign_without_late_contact": int((~group["late"]).sum()),
            }
        )
    return pd.DataFrame(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--csv", type=Path, default=ROOT / "outputs/logs/exp02.csv", help="saved exp02 matrix"
    )
    args = parser.parse_args()
    scenarios, _ = tradeoff_set()
    labels = label_window(scenarios)
    late = labels.loc[labels["late_contact"]]
    frame = pd.read_csv(args.csv)
    split = split_false_brakes(frame, set(late["scenario_id"]))
    grids = {}
    for name, quick in (("quick", True), ("full", False)):
        grid = label_window(boundary_grid(quick=quick))
        grids[name] = {
            "n": int(len(grid)),
            "n_benign": int(grid["benign"].sum()),
            "n_late_contact": int(grid["late_contact"].sum()),
        }
    summary = {
        "scored_duration_s": SIM_DURATION,
        "log_duration_s": LOG_DURATION,
        "exp02": {
            "n": int(len(labels)),
            "n_benign": int(labels["benign"].sum()),
            "n_late_contact": int(len(late)),
            "late_contact": late.to_dict(orient="records"),
            "false_brake_split": split.to_dict(orient="records"),
        },
        "boundary_grid": grids,
        "csv": str(args.csv),
    }
    path = ROOT / "outputs/logs/exp02_label_window.json"
    write_json(path, summary)
    print(json.dumps({key: value for key, value in summary.items() if key != "exp02"}, indent=2))
    print(late.to_string(index=False))
    print(split.to_string(index=False))
    print(f"Artifacts: {path.resolve()}")


if __name__ == "__main__":
    main()

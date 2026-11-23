"""Does a longer outcome window change any score of a saved matrix? (D)

exp02, exp03 and exp05 score brakes for DECISION_DURATION and look for contact until
OUTCOME_DURATION. This script reruns a saved matrix with ``--extra`` more seconds of
simulation and compares the two run by run:

- ``label_changes``: scenarios whose class differs. A benign scenario whose never-brake
  contact comes after the outcome window is listed here.
- ``collision_changes``: runs whose collision flag differs. A collision that braking only
  postponed past the outcome window is listed here.
- ``decision_changes``: runs whose first brake inside the decision window differs. The
  first OUTCOME_DURATION seconds are the same simulation, so this should be zero.
- ``false_brakes_on_changed_labels``: false brakes of the saved matrix in scenarios
  whose label changed.

It changes no label and no metric.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

try:
    from matrix_setup import MATRIX_KWARGS, SIM_KWARGS, write_json
    from scenario_sets import (
        DECISION_DURATION,
        LOG_PAD,
        OUTCOME_DURATION,
        boundary_grid,
        tradeoff_set,
    )
except ModuleNotFoundError:
    from experiments.matrix_setup import MATRIX_KWARGS, SIM_KWARGS, write_json
    from experiments.scenario_sets import (
        DECISION_DURATION,
        LOG_PAD,
        OUTCOME_DURATION,
        boundary_grid,
        tradeoff_set,
    )

from pbs.eval.sweep import run_matrix, scenario_id
from pbs.types import Scenario

ROOT = Path(__file__).resolve().parents[1]
RUN_KEYS = ["scenario_id", "system", "threshold", "latency"]
DESCRIBE = [
    "family",
    "kind",
    "gap_d",
    "rel_v",
    "lc_T",
    "init_gap",
    "decel",
    "arrival_t",
    "a_cutter",
    "maneuver",
]
SETS = {
    "tradeoff": lambda log_duration: tradeoff_set(log_duration)[0],
    "boundary-quick": lambda log_duration: boundary_grid(True, log_duration),
    "boundary-full": lambda log_duration: boundary_grid(False, log_duration),
}


def paired_ids(standard: list[Scenario], extended: list[Scenario]) -> dict[str, str]:
    """Id of each longer-log scenario -> id of the same scenario in the saved matrix.

    Ids hash the whole trajectory, so the same scenario has a new id on a longer log.
    """
    if len(standard) != len(extended):
        raise ValueError(f"{len(standard)} scenarios against {len(extended)} on longer logs")
    pairs = {}
    for short, long in zip(standard, extended, strict=True):
        if _signature(short) != _signature(long):
            raise ValueError(f"scenario order differs: {_signature(short)} vs {_signature(long)}")
        pairs[scenario_id(long)] = scenario_id(short)
    return pairs


def compare(saved: pd.DataFrame, extended: pd.DataFrame, pairs: dict[str, str]) -> dict:
    """Differences between a saved matrix and its rerun with a longer outcome window."""
    left = _keyed(saved)
    right = _keyed(extended.assign(scenario_id=extended["scenario_id"].map(pairs)))
    merged = left.merge(right, on=RUN_KEYS, suffixes=("", "_ext"), validate="one_to_one")
    if len(merged) != len(left):
        raise ValueError(f"the rerun matches {len(merged)} of {len(left)} saved runs")
    decision = merged["decision_duration"].astype(float)
    onset = _in_window(merged["brake_onset_t"], decision)
    onset_ext = _in_window(merged["brake_onset_t_ext"], decision)
    same_onset = (onset.isna() & onset_ext.isna()) | np.isclose(onset, onset_ext, equal_nan=True)
    label_changed = merged["label"] != merged["label_ext"]
    collision_changed = merged["collision"].astype(bool) != merged["collision_ext"].astype(bool)
    described = [column for column in DESCRIBE if column in merged.columns]
    label_rows = merged.loc[label_changed].drop_duplicates("scenario_id")
    return {
        "n_runs": int(len(merged)),
        "n_scenarios": int(merged["scenario_id"].nunique()),
        "label_changes": _records(
            label_rows, ["scenario_id", *described, "label", "label_ext"], "never_brake_contact_t"
        ),
        "collision_changes": _records(
            merged.loc[collision_changed],
            ["scenario_id", *described, "system", "threshold", "label", "collision"],
            "first_collision_t",
        ),
        "decision_changes": int((~same_onset).sum()),
        "false_brakes_on_changed_labels": int(
            (label_changed & merged["false_brake"].astype(bool)).sum()
        ),
    }


def _signature(scenario: Scenario):
    params = {key: repr(value) for key, value in scenario.params.items() if key != "duration"}
    return scenario.kind, sorted(params.items())


def _keyed(frame: pd.DataFrame) -> pd.DataFrame:
    out = frame.copy()
    for column in ("threshold", "latency"):
        out[column] = out[column].astype(float).round(9)
    return out


def _in_window(onset: pd.Series, decision: pd.Series) -> pd.Series:
    onset = onset.astype(float)
    return onset.where(onset < decision - 1e-9)


def _records(rows: pd.DataFrame, columns: list[str], extended_column: str) -> list[dict]:
    out = []
    for _, row in rows.iterrows():
        record = {key: row[key] for key in columns if not pd.isna(row[key])}
        value = row[f"{extended_column}_ext"]
        record[f"{extended_column}_ext"] = None if pd.isna(value) else float(value)
        out.append(record)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=ROOT / "outputs/logs/exp02.csv")
    parser.add_argument(
        "--scenarios",
        choices=sorted(SETS),
        default="tradeoff",
        help="scenario set the CSV was run on: exp02 = tradeoff, exp03 / exp05 = boundary-*",
    )
    parser.add_argument("--extra", type=float, default=6.0, help="seconds added to the run")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    saved = pd.read_csv(args.csv)
    if "label" not in saved.columns or not np.allclose(saved["outcome_duration"], OUTCOME_DURATION):
        raise SystemExit(f"{args.csv} was not written by an L2 run with two windows")
    extended_duration = OUTCOME_DURATION + args.extra
    build = SETS[args.scenarios]
    scenarios = build(extended_duration + LOG_PAD)
    pairs = paired_ids(build(OUTCOME_DURATION + LOG_PAD), scenarios)
    extended = run_matrix(
        list(dict.fromkeys(saved["system"])),
        scenarios,
        sorted(set(saved["threshold"].astype(float))),
        sorted(set(saved["latency"].astype(float))),
        sim_kwargs={**SIM_KWARGS, "duration": extended_duration},
        **MATRIX_KWARGS,
    )
    summary = {
        "csv": str(args.csv),
        "decision_duration_s": DECISION_DURATION,
        "outcome_duration_s": OUTCOME_DURATION,
        "extended_outcome_duration_s": extended_duration,
        **compare(saved, extended, pairs),
    }
    out = args.out or args.csv.with_name(f"{args.csv.stem}_outcome_window.json")
    write_json(out, summary)
    print(json.dumps(summary, indent=2, default=str))
    print(f"Artifacts: {out.resolve()}")


if __name__ == "__main__":
    main()

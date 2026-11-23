"""exp03 — HEADLINE FIGURE (D). Failure boundaries of Reactive / CV / CA / Oracle
on the (gap_d, rel_v) plane for A's ``make_scenario("cut_in")`` at lc_T = 3 s.

  rescue(CV) − rescue(Reactive) = value of prediction
  rescue(Oracle) − rescue(CV)   = cost of prediction error

The left panel picks, per system, the threshold whose false-brake rate is at most 5%
and whose collision rate is then smallest. The right panel holds every system at t_on = 2.0 s.
Both panels overlay the 1D kinematic stopping limit d = clearance + rel_v²/(2a) at a = 6 m/s²
(clearance = 5.0 m for L2 OBB geometry; 2.5 m for legacy L1 circle mode).
Output: outputs/figures/exp03_boundaries.png
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd
from matrix_setup import (
    BOUNDARY_SYSTEMS,
    FULL_THRESHOLDS,
    QUICK_THRESHOLDS,
    experiment_config,
    write_json,
)
from scenario_sets import boundary_grid

from pbs.eval.plots import plot_boundaries, plot_stopping_distance
from pbs.eval.sweep import rescue_rate, run_matrix, select_operating_point, tradeoff
from pbs.sim.collision import R_SAFE, VEHICLE_LENGTH

ROOT = Path(__file__).resolve().parents[1]
FIXED_THRESHOLD = 2.0
A_BRAKE = 6.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true", help="5×5 grid and two thresholds")
    parser.add_argument(
        "--legacy-l1",
        action="store_true",
        help="use legacy L1 circle geometry (r_safe=2.5) and single-threshold policy",
    )
    parser.add_argument(
        "--from-csv", type=Path, default=None, help="redraw the figure from a saved matrix"
    )
    args = parser.parse_args()
    sim_kwargs, matrix_kwargs, meta = experiment_config(legacy_l1=args.legacy_l1)
    csv_path = ROOT / "outputs/logs/exp03.csv"
    figure_path = ROOT / "outputs/figures/exp03_boundaries.png"
    summary_path = ROOT / "outputs/logs/exp03_summary.json"
    if args.from_csv:
        frame = pd.read_csv(args.from_csv)
        clearance = _infer_clearance(frame, meta["clearance_m"])
    else:
        thresholds = QUICK_THRESHOLDS if args.quick else FULL_THRESHOLDS
        if FIXED_THRESHOLD not in thresholds:
            thresholds = tuple(sorted(set(thresholds) | {FIXED_THRESHOLD}))
        scenarios = boundary_grid(quick=args.quick)
        frame = run_matrix(
            BOUNDARY_SYSTEMS,
            scenarios,
            thresholds,
            sim_kwargs=sim_kwargs,
            **matrix_kwargs,
        )
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        frame.to_csv(csv_path, index=False)
        frame = pd.read_csv(csv_path)
        clearance = meta["clearance_m"]
    table = tradeoff(frame)
    chosen = select_operating_point(table, max_false_brake=0.05)
    budget_rows = _rows_at(frame, chosen)
    rescue = {
        "budget": _rescue_block(budget_rows),
        "fixed_2s": _rescue_block(frame, threshold=FIXED_THRESHOLD),
    }
    _plot(budget_rows, frame, chosen, figure_path, clearance=clearance)
    summary = {
        "quick": bool(args.quick) if not args.from_csv else None,
        "config": None if args.from_csv else meta,
        "stopping_clearance_m": float(clearance),
        "cells": _cell_counts(frame),
        "operating_point": chosen.to_dict(orient="records"),
        "rescue": rescue,
        "value_of_prediction": _delta(rescue["budget"], "cv", "reactive"),
        "cost_of_prediction_error": _delta(rescue["budget"], "oracle", "cv"),
        "value_of_prediction_fixed_2s": _delta(rescue["fixed_2s"], "cv", "reactive"),
        "cost_of_prediction_error_fixed_2s": _delta(rescue["fixed_2s"], "oracle", "cv"),
        "csv": str(csv_path),
        "figure": str(figure_path),
    }
    write_json(summary_path, summary)
    print(json.dumps(summary, indent=2, default=str))
    print(f"Artifacts: {figure_path.resolve()}")


def _infer_clearance(frame: pd.DataFrame, default: float) -> float:
    if "geometry" in frame.columns and len(frame):
        geom = str(frame["geometry"].iloc[0]).strip().lower()
        if geom == "circle":
            return float(R_SAFE)
        if geom == "obb":
            return float(VEHICLE_LENGTH)
    return float(default)


def _cell_counts(frame: pd.DataFrame) -> dict:
    """Cells per class. Tables written before the ``label`` column have two classes."""
    cells = frame.drop_duplicates("scenario_id")
    if "label" in cells.columns:
        labels = cells["label"]
    else:
        benign = cells["benign"].astype(str).str.lower() == "true"
        labels = benign.map({True: "benign", False: "dangerous"})
    return {name: int((labels == name).sum()) for name in ("dangerous", "late_contact", "benign")}


def _rows_at(frame: pd.DataFrame, chosen: pd.DataFrame) -> pd.DataFrame:
    parts = []
    for _, row in chosen.iterrows():
        mask = frame["system"] == row["system"]
        mask &= frame["threshold"].astype(float).sub(float(row["threshold"])).abs() < 1e-9
        mask &= frame["latency"].astype(float).sub(float(row["latency"])).abs() < 1e-9
        parts.append(frame.loc[mask])
    return pd.concat(parts, ignore_index=True)


def _rescue_block(frame: pd.DataFrame, threshold=None) -> dict:
    return {
        system: rescue_rate(frame, system, threshold=threshold)
        for system in BOUNDARY_SYSTEMS
        if system in set(frame["system"])
    }


def _delta(block: dict, left: str, right: str):
    if left not in block or right not in block:
        return None
    return block[left] - block[right]


def _plot(budget_rows, frame, chosen, path: Path, *, clearance: float = VEHICLE_LENGTH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.2), sharey=True)
    labels = []
    for system in BOUNDARY_SYSTEMS:
        match = chosen.loc[chosen["system"] == system]
        if match.empty:
            continue
        labels.append(f"{system} @ {float(match['threshold'].iloc[0]):.1f}s")
    present = [system for system in BOUNDARY_SYSTEMS if system in set(budget_rows["system"])]
    plot_boundaries(budget_rows, present, ax=axes[0], labels=labels[: len(present)])
    plot_stopping_distance(
        axes[0], t_react=0.0, a_brake=A_BRAKE, clearance=clearance, rel_max=_rel_max(frame)
    )
    axes[0].legend(fontsize=8)
    axes[0].set_title("false-brake ≤ 5%")
    fixed_labels = [f"{system} @ {FIXED_THRESHOLD:.1f}s" for system in present]
    plot_boundaries(frame, present, ax=axes[1], threshold=FIXED_THRESHOLD, labels=fixed_labels)
    plot_stopping_distance(
        axes[1], t_react=0.0, a_brake=A_BRAKE, clearance=clearance, rel_max=_rel_max(frame)
    )
    axes[1].legend(fontsize=8)
    axes[1].set_title(f"fixed t_on = {FIXED_THRESHOLD:.1f} s")
    fig.suptitle("exp03  cut-in failure boundaries")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _rel_max(frame: pd.DataFrame) -> float:
    return float(frame["rel_v"].max())


if __name__ == "__main__":
    main()

"""exp03 — HEADLINE FIGURE (D). Failure boundaries of Reactive / CV / CA / Oracle
on the (gap_d, rel_v) plane for completed cut-ins.

  rescue(CV) − rescue(Reactive) = value of prediction
  rescue(Oracle) − rescue(CV)   = cost of prediction error

The left panel picks, per system, the threshold whose false-brake rate is at most 5%
and whose collision rate is then smallest. The right panel holds every system at 2.0 s.
Both panels overlay d = rel_v·t_react + rel_v²/(2a) at t_react = 2.0 s, a = 6 m/s².
Output: outputs/figures/exp03_boundaries.png
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd
from cut_in_grid import cut_in_grid
from matrix_setup import (
    BOUNDARY_SYSTEMS,
    FULL_THRESHOLDS,
    QUICK_THRESHOLDS,
    SIM_KWARGS,
    write_json,
)

from pbs.eval.plots import plot_boundaries, plot_stopping_distance
from pbs.eval.sweep import rescue_rate, run_matrix, select_operating_point, tradeoff

ROOT = Path(__file__).resolve().parents[1]
FIXED_THRESHOLD = 2.0
A_BRAKE = 6.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true", help="5×5 grid and two thresholds")
    parser.add_argument(
        "--from-csv", type=Path, default=None, help="redraw the figure from a saved matrix"
    )
    args = parser.parse_args()
    csv_path = ROOT / "outputs/logs/exp03.csv"
    figure_path = ROOT / "outputs/figures/exp03_boundaries.png"
    summary_path = ROOT / "outputs/logs/exp03_summary.json"
    if args.from_csv:
        frame = pd.read_csv(args.from_csv)
    else:
        thresholds = QUICK_THRESHOLDS if args.quick else FULL_THRESHOLDS
        if FIXED_THRESHOLD not in thresholds:
            thresholds = tuple(sorted(set(thresholds) | {FIXED_THRESHOLD}))
        scenarios = cut_in_grid(quick=args.quick, maneuvers=("complete",))
        frame = run_matrix(BOUNDARY_SYSTEMS, scenarios, thresholds, sim_kwargs=SIM_KWARGS)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        frame.to_csv(csv_path, index=False)
        frame = pd.read_csv(csv_path)
    table = tradeoff(frame)
    chosen = select_operating_point(table, max_false_brake=0.05)
    budget_rows = _rows_at(frame, chosen)
    rescue = {
        "budget": _rescue_block(budget_rows),
        "fixed_2s": _rescue_block(frame, threshold=FIXED_THRESHOLD),
    }
    _plot(budget_rows, frame, chosen, figure_path)
    summary = {
        "quick": bool(args.quick) if not args.from_csv else None,
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


def _plot(budget_rows, frame, chosen, path: Path) -> None:
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
    plot_stopping_distance(axes[0], FIXED_THRESHOLD, A_BRAKE, rel_max=_rel_max(frame))
    axes[0].legend(fontsize=8)
    axes[0].set_title("false-brake ≤ 5%")
    fixed_labels = [f"{system} @ {FIXED_THRESHOLD:.1f}s" for system in present]
    plot_boundaries(frame, present, ax=axes[1], threshold=FIXED_THRESHOLD, labels=fixed_labels)
    plot_stopping_distance(axes[1], FIXED_THRESHOLD, A_BRAKE, rel_max=_rel_max(frame))
    axes[1].legend(fontsize=8)
    axes[1].set_title(f"fixed threshold {FIXED_THRESHOLD:.1f} s")
    fig.suptitle("exp03  cut-in failure boundaries")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _rel_max(frame: pd.DataFrame) -> float:
    return float(frame["rel_v"].max())


if __name__ == "__main__":
    main()

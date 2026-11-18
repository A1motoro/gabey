"""exp02 — collision rate vs false-brake rate, one curve per system in CORE (D).

The scenario set mixes completed cut-ins with aborted near-misses. A completed cut-in
with rel_v <= 0 never closes the gap, so nobody brakes and it cannot show a false brake.
The abort set dips toward the lane and returns outside r_safe; constant-velocity
extrapolation still predicts a collision. Output: outputs/figures/exp02_tradeoff.png
and outputs/logs/exp02.csv. The figure is drawn from the CSV.
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from cut_in_grid import cut_in_grid
from matrix_setup import (
    BOUNDARY_SYSTEMS,
    FULL_THRESHOLDS,
    QUICK_THRESHOLDS,
    SIM_KWARGS,
    write_json,
)

from pbs.eval.plots import SYSTEM_COLORS
from pbs.eval.sweep import run_matrix, tradeoff

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true", help="5×5 grid and two thresholds")
    parser.add_argument(
        "--from-csv", type=Path, default=None, help="redraw the figure from a saved matrix"
    )
    args = parser.parse_args()
    csv_path = ROOT / "outputs/logs/exp02.csv"
    figure_path = ROOT / "outputs/figures/exp02_tradeoff.png"
    summary_path = ROOT / "outputs/logs/exp02_summary.json"
    if args.from_csv:
        frame = pd.read_csv(args.from_csv)
    else:
        thresholds = QUICK_THRESHOLDS if args.quick else FULL_THRESHOLDS
        scenarios = cut_in_grid(quick=args.quick, maneuvers=("complete", "abort"))
        frame = run_matrix(BOUNDARY_SYSTEMS, scenarios, thresholds, sim_kwargs=SIM_KWARGS)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        frame.to_csv(csv_path, index=False)
        frame = pd.read_csv(csv_path)
    table = tradeoff(frame)
    _plot(table, figure_path)
    one = frame[frame["system"] == frame["system"].iloc[0]]
    one = one[np.isclose(one["threshold"].astype(float), float(one["threshold"].iloc[0]))]
    one = one[np.isclose(one["latency"].astype(float), float(one["latency"].iloc[0]))]
    benign = one["benign"].map(
        lambda value: bool(value) if isinstance(value, (bool, np.bool_)) else str(value) == "True"
    )
    summary = {
        "quick": bool(args.quick) if not args.from_csv else None,
        "n_rows": int(len(frame)),
        "n_scenarios": int(len(one)),
        "n_benign_scenarios": int(benign.sum()),
        "n_dangerous_scenarios": int((~benign).sum()),
        "tradeoff": table.to_dict(orient="records"),
        "false_brake_identically_zero": bool((table["false_brake_rate"].fillna(0) == 0).all()),
        "csv": str(csv_path),
        "figure": str(figure_path),
    }
    write_json(summary_path, summary)
    printable = {key: value for key, value in summary.items() if key != "tradeoff"}
    print(json.dumps(printable, indent=2))
    print(table.to_string(index=False))
    print(f"Artifacts: {figure_path.resolve()}")


def _plot(table: pd.DataFrame, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.2, 5.4))
    for system, group in table.groupby("system", sort=False):
        group = group.sort_values("threshold")
        color = SYSTEM_COLORS.get(system)
        ax.plot(
            group["false_brake_rate"],
            group["collision_rate"],
            marker="o",
            color=color,
            label=system,
        )
        for _, row in group.iterrows():
            ax.annotate(
                f"{row['threshold']:.1f}",
                (row["false_brake_rate"], row["collision_rate"]),
                textcoords="offset points",
                xytext=(4, 4),
                fontsize=8,
                color=color,
            )
    ax.set_xlabel("false-brake rate (benign scenarios)")
    ax.set_ylabel("collision rate (dangerous scenarios)")
    ax.set_xlim(-0.05, 1.05)
    ax.set_ylim(-0.05, 1.05)
    ax.set_title("exp02  safety–false-brake trade-off")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()

"""exp02 — collision rate vs false-brake rate, one curve per system in CORE (D).

Scenarios: A's default grid (cut_in, lead_brake, crossing), A's CA slice, and six
aborted cut-ins. On A's cut-ins alone no system ever false-brakes, so the cost of a
wrong cut-in prediction would not show; the aborts lean toward the lane and return
outside r_safe. The summary reports the curve with and without them, and per family.
Brakes are scored for 15 s and collisions for 19 s; scenarios whose never-brake contact
falls in between are in neither rate and are listed under ``late_contact`` in the summary.
Output: outputs/figures/exp02_tradeoff.png and outputs/logs/exp02.csv. The figure is
drawn from the CSV.
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matrix_setup import (
    BOUNDARY_SYSTEMS,
    FULL_THRESHOLDS,
    QUICK_THRESHOLDS,
    experiment_config,
    write_json,
)
from scenario_sets import kind_label, tradeoff_set

from pbs.eval.plots import SYSTEM_COLORS
from pbs.eval.sweep import late_contact_outcomes, run_matrix, scenario_id, tradeoff

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true", help="two thresholds instead of six")
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
    csv_path = ROOT / "outputs/logs/exp02.csv"
    figure_path = ROOT / "outputs/figures/exp02_tradeoff.png"
    summary_path = ROOT / "outputs/logs/exp02_summary.json"
    counts = None
    if args.from_csv:
        frame = pd.read_csv(args.from_csv)
    else:
        thresholds = QUICK_THRESHOLDS if args.quick else FULL_THRESHOLDS
        scenarios, counts = tradeoff_set()
        frame = run_matrix(
            BOUNDARY_SYSTEMS,
            scenarios,
            thresholds,
            sim_kwargs=sim_kwargs,
            **matrix_kwargs,
        )
        families = {scenario_id(s): kind_label(s) for s in scenarios}
        frame.insert(2, "family", frame["scenario_id"].map(families))
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        frame.to_csv(csv_path, index=False)
        frame = pd.read_csv(csv_path)
    table = tradeoff(frame)
    team_only = tradeoff(frame[frame["family"] != "cut_in_abort"])
    by_family = {
        family: tradeoff(group).to_dict(orient="records")
        for family, group in frame.groupby("family", sort=False)
    }
    _plot(table, figure_path)
    one = frame[frame["system"] == frame["system"].iloc[0]]
    one = one[np.isclose(one["threshold"].astype(float), float(one["threshold"].iloc[0]))]
    one = one[np.isclose(one["latency"].astype(float), float(one["latency"].iloc[0]))]
    benign = one["benign"].map(
        lambda value: bool(value) if isinstance(value, bool | np.bool_) else str(value) == "True"
    )
    if "label" in one.columns:
        late = one["label"] == "late_contact"
    else:
        late = pd.Series(False, index=one.index)
    oracle = table.loc[table["system"] == "oracle"]
    summary = {
        "quick": bool(args.quick) if not args.from_csv else None,
        "config": None if args.from_csv else meta,
        "n_rows": int(len(frame)),
        "n_scenarios": int(len(one)),
        "n_benign_scenarios": int(benign.sum()),
        "n_dangerous_scenarios": int((~benign & ~late).sum()),
        "n_late_contact_scenarios": int(late.sum()),
        "scenario_counts": counts,
        "tradeoff": table.to_dict(orient="records"),
        "tradeoff_without_aborts": team_only.to_dict(orient="records"),
        "tradeoff_by_family": by_family,
        "false_brake_identically_zero": bool((table["false_brake_rate"].fillna(0) == 0).all()),
        "oracle_false_brake_free": (
            bool((oracle["false_brake_rate"].fillna(0) == 0).all()) if len(oracle) else None
        ),
        "late_contact": late_contact_outcomes(frame).to_dict(orient="records"),
        "late_contact_scenarios": _late_scenarios(one.loc[late]),
        "csv": str(csv_path),
        "figure": str(figure_path),
    }
    write_json(summary_path, summary)
    printable = {
        key: value
        for key, value in summary.items()
        if not key.startswith("tradeoff") and not key.startswith("late_contact")
    }
    print(json.dumps(printable, indent=2))
    print(table.to_string(index=False))
    print("without aborts:")
    print(team_only.to_string(index=False))
    print(f"Artifacts: {figure_path.resolve()}")


def _late_scenarios(rows: pd.DataFrame) -> list[dict]:
    keep = ["scenario_id", "family", "never_brake_contact_t"]
    return rows[[column for column in keep if column in rows.columns]].to_dict(orient="records")


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
    x_max = max(0.2, 1.3 * float(table["false_brake_rate"].max()))
    ax.set_xlim(-0.02 * x_max, x_max)
    ax.set_ylim(-0.05, 1.05)
    ax.set_title("exp02  safety–false-brake trade-off")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()

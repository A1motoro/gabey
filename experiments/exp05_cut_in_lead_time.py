"""exp05 — cut-in detection lead time (P + D).

For each ``make_scenario("cut_in")`` on the boundary grid:
brake_onset(Reactive) − brake_onset(CV / CTRV), at one shared threshold.
Defined only when both systems brake. Scenarios where only one side brakes
are counted, not dropped. Onsets are taken over the whole 19 s run; the summary counts
those after the 15 s decision window. Output: outputs/figures/exp05_lead_time.png
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matrix_setup import LEAD_SYSTEMS, experiment_config, write_json
from scenario_sets import boundary_grid

from pbs.eval.plots import SYSTEM_COLORS
from pbs.eval.sweep import lead_times, run_matrix

ROOT = Path(__file__).resolve().parents[1]
THRESHOLD = 2.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true", help="5×5 completed cut-ins")
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
    csv_path = ROOT / "outputs/logs/exp05.csv"
    figure_path = ROOT / "outputs/figures/exp05_lead_time.png"
    summary_path = ROOT / "outputs/logs/exp05_summary.json"
    if args.from_csv:
        frame = pd.read_csv(args.from_csv)
    else:
        scenarios = boundary_grid(quick=args.quick)
        frame = run_matrix(
            LEAD_SYSTEMS,
            scenarios,
            [THRESHOLD],
            sim_kwargs=sim_kwargs,
            **matrix_kwargs,
        )
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        frame.to_csv(csv_path, index=False)
        frame = pd.read_csv(csv_path)
    leads = lead_times(frame, reference="reactive")
    _plot(leads, figure_path)
    counts = (
        leads.groupby(["system", "brake_status"])
        .size()
        .rename("n")
        .reset_index()
        .to_dict(orient="records")
    )
    defined = leads.loc[leads["defined"]]
    medians = {
        system: float(group["lead_time"].median())
        for system, group in defined.groupby("system")
        if len(group)
    }
    earlier = defined.loc[defined["lead_time"] > 0.3]
    earlier_medians = {
        system: float(group["lead_time"].median())
        for system, group in earlier.groupby("system")
        if len(group)
    }
    summary = {
        "quick": bool(args.quick) if not args.from_csv else None,
        "config": None if args.from_csv else meta,
        "threshold_s": THRESHOLD,
        "n_rows": int(len(leads)),
        "median_lead_time_s": medians,
        "median_lead_time_when_predictive_is_earlier_s": earlier_medians,
        "counts": counts,
        "brake_onsets_after_decision_window": _late_onsets(frame),
        "csv": str(csv_path),
        "figure": str(figure_path),
    }
    write_json(summary_path, summary)
    print(json.dumps(summary, indent=2, default=str))
    print(f"Artifacts: {figure_path.resolve()}")


def _late_onsets(frame: pd.DataFrame) -> int | None:
    """Lead times use the first brake of the whole run; this counts those after 15 s."""
    if "decision_duration" not in frame.columns:
        return None
    onset = frame["brake_onset_t"].astype(float)
    return int((onset >= frame["decision_duration"].astype(float) - 1e-9).sum())


def _plot(leads: pd.DataFrame, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    systems = [
        system for system in ("cv", "ctrv", "ca", "oracle") if system in set(leads["system"])
    ]
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.6))
    bins = np.linspace(-1.0, 4.0, 11)
    for system in systems:
        values = leads.loc[(leads["system"] == system) & leads["defined"], "lead_time"]
        axes[0].hist(
            values,
            bins=bins,
            histtype="step",
            linewidth=2.0,
            color=SYSTEM_COLORS.get(system),
            label=f"{system} (n={len(values)})",
        )
    axes[0].axvline(0.0, color="grey", linestyle=":", linewidth=1.0)
    axes[0].set_xlabel("lead time [s]  (reactive onset − predictive onset)")
    axes[0].set_ylabel("scenarios")
    axes[0].set_title("both systems braked")
    axes[0].grid(alpha=0.3)
    axes[0].legend()
    statuses = ["both", "predictive_only", "reference_only", "neither"]
    x = np.arange(len(systems))
    width = 0.18
    for index, status in enumerate(statuses):
        heights = []
        for system in systems:
            heights.append(
                int(((leads["system"] == system) & (leads["brake_status"] == status)).sum())
            )
        axes[1].bar(x + (index - 1.5) * width, heights, width=width, label=status)
    axes[1].set_xticks(x, systems)
    axes[1].set_ylabel("scenarios")
    axes[1].set_title("undefined lead times are counted")
    axes[1].grid(alpha=0.3, axis="y")
    axes[1].legend(fontsize=8)
    fig.suptitle(f"exp05  cut-in lead time at TTC {THRESHOLD:.1f} s")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()

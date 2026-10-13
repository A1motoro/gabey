"""P's L1 evidence: CV ADE/FDE and Reactive/CV cut-in timeline.

Run after pip install -e .: python experiments/exp00_l1_baseline.py
Analytic validation fixtures only; M1 data loading and M4 experiment matrix are separate.
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from l1_fixtures import cut_in_fixture, straight_track

from pbs.eval.counterfactual import is_benign
from pbs.eval.metrics import ade, evaluate, fde
from pbs.prediction import ConstantVelocityPredictor
from pbs.sim.policy import TTCThresholdPolicy
from pbs.sim.simulator import run_sim
from pbs.systems import make_system
from pbs.utils import history_until

ROOT = Path(__file__).resolve().parents[1]


def main(output_dir: Path) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    scenario = cut_in_fixture()
    predictor = ConstantVelocityPredictor()
    rows = []
    for label, track in [("straight_2d", straight_track()), ("quintic_cut_in", scenario.agents[0])]:
        for horizon in (5, 10, 15, 20, 25, 30):
            errors = []
            for t_now in (1.0, 1.5, 2.0, 2.5):
                observed = history_until(track, t_now)
                predicted = predictor.predict(observed, horizon, scenario.dt)
                times = observed.t[-1] + np.arange(1, horizon + 1) * scenario.dt
                truth = np.column_stack(
                    (np.interp(times, track.t, track.x), np.interp(times, track.t, track.y))
                )
                errors.append((ade(predicted, truth), fde(predicted, truth)))
            mean_error = np.mean(errors, axis=0)
            rows.append(
                {
                    "fixture": label,
                    "horizon_s": horizon * scenario.dt,
                    "ade_m": mean_error[0],
                    "fde_m": mean_error[1],
                    "windows": len(errors),
                }
            )
    accuracy = pd.DataFrame(rows)
    accuracy.to_csv(output_dir / "cv_ade_fde.csv", index=False)
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    for label, values in accuracy.groupby("fixture"):
        axes[0].plot(values.horizon_s, values.ade_m, "o-", label=label)
        axes[1].plot(values.horizon_s, values.fde_m, "o-", label=label)
    for ax, metric in zip(axes, ("ADE [m]", "FDE [m]"), strict=True):
        ax.set(xlabel="Prediction horizon [s]", ylabel=metric)
        ax.grid(alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "cv_ade_fde.png", dpi=150)
    plt.close(fig)

    benign = is_benign(scenario, duration=6.0)
    summaries, logs = {}, {}
    for name in ("reactive", "cv"):
        log = run_sim(scenario, make_system(name), TTCThresholdPolicy(), duration=6.0)
        logs[name] = log
        summaries[name] = evaluate(log, benign)
        pd.DataFrame(
            {
                "t_s": log.t,
                "ego_x_m": log.ego_x,
                "ego_v_mps": log.ego_v,
                "ttc_s": log.ttc,
                "action": log.action,
            }
        ).to_csv(output_dir / f"{name}_timeline.csv", index=False)
    reactive_onset = summaries["reactive"]["brake_onset_t"]
    cv_onset = summaries["cv"]["brake_onset_t"]
    lead_time = None if reactive_onset is None or cv_onset is None else reactive_onset - cv_onset
    summary = {
        "fixture": "analytic minimum-jerk cut-in, not M1 production scenario",
        "parameters": scenario.params,
        "duration_s": 6.0,
        "dt_s": scenario.dt,
        "horizon_s": 3.0,
        "ttc_threshold_s": 2.0,
        "r_safe_m": 2.5,
        "systems": summaries,
        "cv_lead_time_s": lead_time,
        "claim": "Fixture-specific demonstration; no traffic-wide safety conclusion.",
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    for name, log in logs.items():
        axes[0].plot(log.t, np.where(np.isfinite(log.ttc), log.ttc, np.nan), label=name)
        axes[1].plot(log.t, log.ego_v, label=name)
        onset = summaries[name]["brake_onset_t"]
        if onset is not None:
            axes[1].axvline(onset, linestyle="--", alpha=0.6)
    axes[0].axhline(2.0, color="grey", linestyle=":", label="brake threshold")
    axes[0].set(ylabel="TTC [s]")
    axes[1].set(ylabel="Ego speed [m/s]")
    agent = scenario.agents[0]
    axes[2].plot(agent.t, agent.y, color="black", label="agent lateral position")
    axes[2].axhline(scenario.lane_width / 2, color="grey", linestyle=":", label="lane boundary")
    axes[2].set(xlabel="Time [s]", ylabel="Agent y [m]", xlim=(0, 6))
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend()
    fig.suptitle(
        f"L1 cut-in: CV brakes {lead_time:.1f} s earlier" if lead_time is not None else "L1 cut-in"
    )
    fig.tight_layout()
    fig.savefig(output_dir / "cut_in_timeline.png", dpi=150)
    plt.close(fig)
    print(json.dumps(summary, indent=2))
    print(f"Artifacts: {output_dir.resolve()}")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/figures/l1-baseline")
    main(parser.parse_args().output_dir)

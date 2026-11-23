"""A's reproducible real-trajectory replay study. Runs entirely from frozen NPZ clips.

python experiments/exp07_real_clip.py
No new predictors, risk logic or evaluation definitions are implemented here.
"""

import argparse
import csv
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from pbs.data import DEFAULT_GRID, make_scenario  # noqa: E402
from pbs.data.womd import load_clip, load_evidence  # noqa: E402
from pbs.eval.counterfactual import is_benign  # noqa: E402
from pbs.eval.metrics import ade, evaluate, fde  # noqa: E402
from pbs.risk.assessors import NullAssessor  # noqa: E402
from pbs.sim.policy import HysteresisPolicy, NeverBrakePolicy  # noqa: E402
from pbs.sim.simulator import run_sim  # noqa: E402
from pbs.systems import SYSTEMS, make_system  # noqa: E402
from pbs.types import Trajectory  # noqa: E402


def forecast_windows(scenario, horizon, *, history_frames=20, stride_steps=5):
    """Same forecast origins for 1/2/3 s, including prehistory available at t=0.

    Twenty input frames INCLUDE current (1.9 s span). Future starts at t+dt;
    the 21st earlier frame retained in the clip is available to other models.
    """
    agent = scenario.agents[0]
    start = int(np.flatnonzero(np.isclose(agent.t, 0, atol=1e-9))[0])
    last = int(np.flatnonzero(agent.t <= scenario.params["duration"] - 3 + 1e-9)[-1])
    for i in range(start, last + 1, stride_steps):
        slc = slice(i - history_frames + 1, i + 1)
        if slc.start < 0 or i + horizon >= len(agent.t):
            raise ValueError("Forecast history/future is incomplete")
        history = Trajectory(
            *(getattr(agent, k)[slc].copy() for k in ("t", "x", "y", "v", "heading")),
            agent_type=agent.agent_type,
            agent_id=agent.agent_id,
        )
        truth = np.column_stack(
            (agent.x[i + 1 : i + horizon + 1], agent.y[i + 1 : i + horizon + 1])
        )
        yield history, truth


def write_csv(path, rows, fields=None):
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields or list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def timeline_figure(scenario, logs, metrics, path):
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    agent = scenario.agents[0]
    xy = np.column_stack(
        (
            np.interp(next(iter(logs.values())).t, agent.t, agent.x),
            np.interp(next(iter(logs.values())).t, agent.t, agent.y),
        )
    )
    for name, log in logs.items():
        line = axes[0].plot(log.t, np.where(np.isfinite(log.ttc), log.ttc, np.nan), label=name)
        axes[1].plot(log.t, log.ego_v, label=name)
        dist = np.linalg.norm(xy - np.column_stack((log.ego_x, log.ego_y)), axis=1)
        axes[2].plot(log.t, dist, label=name)
        onset = metrics[name]["brake_onset_t"]
        if onset is not None:
            for ax in axes:
                ax.axvline(onset, color=line[0].get_color(), linestyle="--", alpha=0.45)
    no_brake = run_sim(scenario, NullAssessor(), NeverBrakePolicy(), duration=6, r_safe=2.5)
    no_brake_dist = np.linalg.norm(xy - np.column_stack((no_brake.ego_x, no_brake.ego_y)), axis=1)
    axes[2].plot(
        no_brake.t, no_brake_dist, color="grey", ls="--", label="never brake (counterfactual)"
    )
    axes[0].axhline(2, color="grey", ls=":", label="brake threshold")
    axes[0].set(ylabel="Estimated TTC [s]", ylim=(0, 5))
    axes[1].set(ylabel="Simulated ego v [m/s]")
    axes[2].axhline(2.5, color="grey", ls=":", label="contact radius")
    axes[2].set(ylabel="Center distance [m]")
    axes[3].plot(agent.t, agent.y, color="black", label="target y in fixed anchor frame")
    for boundary in (-1.75, 1.75):
        axes[3].axhline(boundary, color="grey", ls=":")
    cross = scenario.params.get("t_cross_s")
    if cross is not None:
        axes[3].axvline(
            cross, color="#d77a23", ls="-.", label="observed center entry (moving ego frame)"
        )
    axes[3].set(xlabel="Time after anchor [s]", ylabel="Target y [m]", xlim=(0, 6))
    reactive, cv = (metrics.get(s, {}).get("brake_onset_t") for s in ("reactive", "cv"))
    lead = None if reactive is None or cv is None else reactive - cv
    subtitle = (
        "one system did not brake" if lead is None else f"Reactive onset - CV onset = {lead:+.1f} s"
    )
    fig.suptitle(
        f"WOMD {scenario.params['scenario_id']} | {subtitle}\n"
        "Illustrative replay case selected after full audit; TTC gaps denote infinity"
    )
    for ax in axes:
        ax.grid(alpha=0.25)
        ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def ade_figure(rows, path):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for group, ax in zip(("cut_in", "control"), axes, strict=True):
        systems = sorted({r["system"] for r in rows})
        for system in systems:
            values = [
                [
                    r["ade_m"]
                    for r in rows
                    if r["group"] == group and r["system"] == system and r["horizon_s"] == h
                ]
                for h in (1, 2, 3)
            ]
            means = [np.mean(v) for v in values]
            spread = [np.std(v, ddof=1) if len(v) > 1 else 0 for v in values]
            ax.errorbar((1, 2, 3), means, yerr=spread, marker="o", capsize=4, label=system)
        ax.set(title=group, xlabel="Forecast horizon [s]", ylabel="ADE [m]", xticks=(1, 2, 3))
        ax.set_ylim(bottom=0)
        ax.grid(alpha=0.25)
        ax.legend()
    fig.suptitle("Real replay prediction error | bars: across-clip SD, not confidence intervals")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def synth_figure(scenarios, path):
    fig, ax = plt.subplots(figsize=(9, 6))
    grid = DEFAULT_GRID["cut_in"]
    seen = set()
    for gap in grid["gap_d"]:
        for rel in grid["rel_v"]:
            # Canonical lc_T=3 s, cut_in_t=.5 s: one synthetic slice, not exp03 boundaries.
            scenario = make_scenario("cut_in", gap_d=gap, rel_v=rel, lc_T=3, cut_in_t=0.5)
            benign = is_benign(scenario, duration=scenario.params["duration"], r_safe=2.5)
            label = "synthetic benign" if benign else "synthetic dangerous"
            ax.scatter(
                rel,
                gap,
                marker="s",
                color="#80bb96" if benign else "#ef9595",
                s=130,
                alpha=0.6,
                label=label if label not in seen else None,
            )
            seen.add(label)
    for group, marker in (("cut_in", "*"), ("control", "^"), ("illustrative", "D")):
        clips = [
            s
            for s in scenarios
            if (
                s.params.get("cohort", "primary") == "illustrative"
                if group == "illustrative"
                else s.params["group"] == group and s.params.get("cohort", "primary") == "primary"
            )
        ]
        ax.scatter(
            [s.params["rel_v"] for s in clips],
            [s.params["gap_d"] for s in clips],
            marker=marker,
            s=110,
            edgecolors="black",
            label=f"real {group}",
        )
    ax.set(
        xlabel="Initial closing speed: ego - target longitudinal v [m/s]",
        ylabel="Initial target longitudinal distance [m]",
        title="Real clips vs one synthetic cut-in grid slice\n"
        "Real points may lie outside the grid; geometry is not calibrated",
    )
    ax.grid(alpha=0.25)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main(args):
    args.figure_dir.mkdir(parents=True, exist_ok=True)
    args.log_dir.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((args.fixtures / "womd_manifest.json").read_text(encoding="utf-8"))
    clips = []
    for item in manifest["selected"]:
        path = args.fixtures / item["filename"]
        if hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
            raise ValueError(f"Frozen clip checksum mismatch: {path}")
        clips.append((path, load_clip(path)))
    primary_clips = list(clips)
    case_manifest_path = args.fixtures / "womd_cases_manifest.json"
    if case_manifest_path.exists():
        case_manifest = json.loads(case_manifest_path.read_text(encoding="utf-8"))
        for item in case_manifest["selected"]:
            path = args.fixtures / item["filename"]
            if hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
                raise ValueError(f"Illustrative clip checksum mismatch: {path}")
            clips.append((path, load_clip(path)))
    if not clips:
        raise ValueError("No frozen clips in manifest")
    available, unavailable = [], {}
    for name in args.systems:
        if name not in SYSTEMS:
            unavailable[name] = "not registered in this checkout"
            continue
        if name == "oracle" and any(
            agent.t[-1] < scenario.params["duration"] + 3 - 1e-9
            for _, scenario in clips
            for agent in scenario.agents
        ):
            unavailable[name] = "frozen truth does not cover the full simulation plus 3 s horizon"
            continue
        try:
            run_sim(clips[0][1], make_system(name), HysteresisPolicy(), duration=6, r_safe=2.5)
        except NotImplementedError as exc:
            unavailable[name] = str(exc)
        else:
            available.append(name)
    if not {"reactive", "cv"}.issubset(available):
        raise RuntimeError("This study requires implemented reactive and cv systems")
    rows, accuracy, timeline_rows = [], [], []
    all_logs, all_metrics = {}, {}
    for _clip_path, scenario in clips:
        p = scenario.params
        cohort = p.get("cohort", "primary")
        benign = is_benign(scenario, duration=p["duration"], r_safe=2.5)
        metrics, logs, error_3s = {}, {}, {}
        for name in available:
            log = run_sim(
                scenario, make_system(name), HysteresisPolicy(), duration=p["duration"], r_safe=2.5
            )
            logs[name] = log
            metrics[name] = evaluate(log, benign)
            predictor = getattr(make_system(name), "predictor", None)
            if predictor is not None:
                predictor.bind(scenario)
                for horizon in (10, 20, 30):
                    errors = []
                    for history, truth in forecast_windows(scenario, horizon):
                        pred = predictor.predict(history, horizon, scenario.dt)
                        if pred.shape != truth.shape or not np.isfinite(pred).all():
                            raise ValueError("Invalid forecast output")
                        errors.append((ade(pred, truth), fde(pred, truth)))
                    values = np.mean(errors, axis=0)
                    accuracy.append(
                        {
                            "clip": p["scenario_id"],
                            "group": p["group"],
                            "cohort": cohort,
                            "system": name,
                            "horizon_s": horizon * scenario.dt,
                            "ade_m": float(values[0]),
                            "fde_m": float(values[1]),
                            "windows": len(errors),
                            "history_frames": 20,
                        }
                    )
                    if horizon == 30:
                        error_3s[name] = values
            for k, t in enumerate(log.t):
                timeline_rows.append(
                    {
                        "clip": p["scenario_id"],
                        "group": p["group"],
                        "system": name,
                        "t_s": float(t),
                        "ego_x_m": float(log.ego_x[k]),
                        "ego_v_mps": float(log.ego_v[k]),
                        "ttc_s": float(log.ttc[k]),
                        "action": log.action[k],
                        "executed": k < log.executed_steps,
                    }
                )
        onset_ref = metrics["reactive"]["brake_onset_t"]
        for name in available:
            onset = metrics[name]["brake_onset_t"]
            row = {
                "clip": p["scenario_id"],
                "group": p["group"],
                "cohort": cohort,
                "system": name,
                **metrics[name],
                "lead_time_s": None
                if onset_ref is None or onset is None
                else float(onset_ref - onset),
                "ade_3s_m": None if name not in error_3s else float(error_3s[name][0]),
                "fde_3s_m": None if name not in error_3s else float(error_3s[name][1]),
            }
            # NumPy scalar -> JSON/CSV-friendly native type.
            row["brake_onset_t"] = None if onset is None else float(onset)
            rows.append(row)
        all_logs[p["scenario_id"]], all_metrics[p["scenario_id"]] = logs, metrics
        print(
            f"{p['group']} {p['scenario_id']}: benign={benign}, "
            f"reactive={metrics['reactive']['brake_onset_t']}, cv={metrics['cv']['brake_onset_t']}"
        )
    for group, filename in (
        ("cut_in", "exp07_real_clip.csv"),
        ("control", "exp07_real_control.csv"),
    ):
        write_csv(
            args.log_dir / filename,
            [r for r in rows if r["group"] == group and r["cohort"] == "primary"],
        )
    cases = [r for r in rows if r["cohort"] == "illustrative"]
    if cases:
        write_csv(args.log_dir / "exp07_real_cases.csv", cases)
    write_csv(args.log_dir / "exp07_real_accuracy.csv", accuracy)
    write_csv(args.log_dir / "exp07_real_timelines.csv", timeline_rows)
    summary = {
        "dataset": manifest["dataset"],
        "records_scanned": manifest["records_scanned"],
        "cohorts": dict(Counter(s.params["group"] for _, s in primary_clips)),
        "illustrative_cases": [
            s.params for _, s in clips if s.params.get("cohort") == "illustrative"
        ],
        "available_systems": available,
        "unavailable_systems": unavailable,
        "simulation": {
            "duration_s": 6,
            "collision_model": "legacy_circle",
            "horizon_s": 3,
            "dt_s": 0.1,
            "latency_s": 0,
            "r_safe_m": 2.5,
            "policy": "HysteresisPolicy",
            "t_on_s": 2,
            "t_off_s": 3,
            "a_brake_mps2": 6,
        },
        "history": "21 frames <=0 retained; forecast benchmark uses final 20 including current",
        "selection": manifest["selection"],
        "results": rows,
        "accuracy": accuracy,
        "claim": (
            "small selected-target real-trajectory replay; no significance or traffic safety claim"
        ),
    }
    (args.log_dir / "exp07_real_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    selected_path, selected_sc = next(
        ((path, s) for path, s in clips if s.params.get("case_role") == "dangerous"),
        next((path, s) for path, s in clips if s.params["group"] == "cut_in"),
    )
    sid = selected_sc.params["scenario_id"]
    timeline_figure(
        selected_sc, all_logs[sid], all_metrics[sid], args.figure_dir / "exp07_real_timeline.png"
    )
    ade_figure(
        [r for r in accuracy if r["cohort"] == "primary"], args.figure_dir / "exp07_real_ade.png"
    )
    synth_figure([s for _, s in clips], args.figure_dir / "exp07_real_vs_synth.png")
    evidence = load_evidence(selected_path)
    fig, ax = plt.subplots(figsize=(10, 4))
    agent = selected_sc.agents[0]
    ax.plot(agent.x, agent.y, label="recorded target")
    ego = evidence["observed_ego_xy"]
    ax.plot(ego[:, 0], ego[:, 1], label="recorded ego")
    ax.axhline(0, color="grey", ls=":", label="simulated ego path")
    ax.set(
        aspect="equal",
        xlabel="Anchor-frame x [m]",
        ylabel="Anchor-frame y [m]",
        title=f"Frozen real cut-in {sid} | fixed anchor frame",
    )
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.figure_dir / "exp07_real_clip_bev.png", dpi=160)
    plt.close(fig)
    print("Systems unavailable:", unavailable)
    print("Artifacts:", args.figure_dir, args.log_dir)
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixtures", type=Path, default=ROOT / "tests/fixtures")
    parser.add_argument("--figure-dir", type=Path, default=ROOT / "outputs/figures")
    parser.add_argument("--log-dir", type=Path, default=ROOT / "outputs/logs")
    parser.add_argument("--systems", nargs="+", default=["reactive", "cv"])
    main(parser.parse_args())

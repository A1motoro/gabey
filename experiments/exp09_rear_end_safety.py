"""Rear-End Sandwich Extension Experiment (Direction 2): Bidirectional Safety Analysis.

Compares {Reactive, Predictive (CV / CTRV / NN-UA / Oracle)} x {L2 Binary Braking,
L3 Staged Gentle Braking} when followed by a human-driven vehicle with reaction delay
tau_rear and bounded braking deceleration.

Outputs:
  outputs/figures/rear-end/exp09_bidirectional_outcomes.png
  outputs/figures/rear-end/exp09_rear_safety_boundary.png
  outputs/figures/rear-end/exp09_sandwich_timeline.png
  outputs/logs/rear-end/exp09_cut_in_outcomes.csv
  outputs/logs/rear-end/exp09_rear_plane.csv
  outputs/logs/rear-end/exp09_waymo_plane.csv
  outputs/logs/rear-end/exp09_summary.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from pbs.data import ca_scenario_set, make_scenario, scenario_set  # noqa: E402
from pbs.data.womd import load_clip  # noqa: E402
from pbs.eval.counterfactual import BENIGN, DANGEROUS, never_brake_label  # noqa: E402
from pbs.eval.rear_end import (  # noqa: E402
    FollowerConfig,
    StagedPolicyConfig,
    rollout_staged_ego,
    simulate_following_vehicle,
    summarize_bidirectional_logs,
    sweep_rear_headway_plane,
)
from pbs.eval.sweep import scenario_id  # noqa: E402
from pbs.prediction import (  # noqa: E402
    ConstantVelocityPredictor,
    CTRVPredictor,
    NeuralPredictor,
    OraclePredictor,
)
from pbs.risk import PredictiveAssessor, ReactiveAssessor  # noqa: E402
from pbs.sim.collision import R_SAFE  # noqa: E402
from pbs.sim.policy import HysteresisPolicy  # noqa: E402
from pbs.sim.simulator import run_sim  # noqa: E402

try:
    from scenario_sets import (  # noqa: E402
        DECISION_DURATION,
        LOG_DURATION,
        OUTCOME_DURATION,
        abort_cut_ins,
    )
except ModuleNotFoundError:
    from experiments.scenario_sets import (  # noqa: E402
        DECISION_DURATION,
        LOG_DURATION,
        OUTCOME_DURATION,
        abort_cut_ins,
    )

FIG_DIR = ROOT / "outputs" / "figures" / "rear-end"
LOG_DIR = ROOT / "outputs" / "logs" / "rear-end"
WAYMO_DANGER_CLIP = (
    ROOT / "tests" / "fixtures" / "cases" / "womd_dangerous_6e86a639df5d4fa1_track1782.npz"
)

VARIANTS = (
    ("reactive_binary", "Reactive\n+ Binary", "reactive", "binary", "#4C78A8"),
    ("reactive_staged", "Reactive\n+ Staged", "reactive", "staged", "#7FA6C9"),
    ("cv_binary", "CV\n+ Binary", "cv", "binary", "#E45756"),
    ("cv_staged", "CV\n+ Staged", "cv", "staged", "#F58518"),
    ("ctrv_staged", "CTRV\n+ Staged", "ctrv", "staged", "#72B7B2"),
    ("nn_ua_staged", "NN-UA\n+ Staged", "nn_ua", "staged", "#54A24B"),
    ("oracle_staged", "Oracle\n+ Staged", "oracle", "staged", "#B279A2"),
)


def _make_assessor(system_key: str):
    if system_key == "reactive":
        return ReactiveAssessor()
    if system_key == "cv":
        return PredictiveAssessor(ConstantVelocityPredictor(), name="cv")
    if system_key == "ctrv":
        return PredictiveAssessor(CTRVPredictor(), name="ctrv")
    if system_key == "nn_ua":
        return PredictiveAssessor(
            NeuralPredictor(use_lane=True),
            uncertainty_scale=1.0,
            name="nn_ua",
        )
    if system_key == "oracle":
        return PredictiveAssessor(OraclePredictor(), name="oracle")
    raise ValueError(f"Unknown system key: {system_key!r}")


def _cut_in_suite(quick: bool = False):
    """Build a balanced cut-in suite covering constant-speed, braking, and aborted merges."""
    if quick:
        base = scenario_set(
            ("cut_in",),
            {"cut_in": {"gap_d": [16.0, 24.0, 34.0], "rel_v": [-2.0, 3.0, 6.0], "lc_T": [3.0]}},
            duration=LOG_DURATION,
        ).scenarios
        ca_decel = scenario_set(
            ("cut_in",),
            {"cut_in": {"gap_d": [22.0], "rel_v": [4.0, 6.0], "a_cutter": [-1.5], "lc_T": [3.0]}},
            duration=LOG_DURATION,
        ).scenarios
        aborts = abort_cut_ins(LOG_DURATION)[:2]
        ca_std = []
    else:
        gaps = [12.0, 16.0, 20.0, 24.0, 28.0, 34.0, 42.0]
        rels = [-2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0]
        base = scenario_set(
            ("cut_in",),
            {"cut_in": {"gap_d": gaps, "rel_v": rels, "lc_T": [3.0]}},
            duration=LOG_DURATION,
        ).scenarios
        ca_decel = scenario_set(
            ("cut_in",),
            {
                "cut_in": {
                    "gap_d": [18.0, 22.0, 26.0, 30.0],
                    "rel_v": [2.0, 4.0, 6.0],
                    "a_cutter": [-1.5, -2.5],
                    "lc_T": [3.0],
                }
            },
            duration=LOG_DURATION,
        ).scenarios
        ca_std = ca_scenario_set(duration=LOG_DURATION).scenarios
        aborts = abort_cut_ins(LOG_DURATION)

    out, seen = [], set()
    for sc in [*base, *ca_decel, *ca_std, *aborts]:
        sid = scenario_id(sc)
        if sid not in seen:
            seen.add(sid)
            out.append(sc)
    return out


def _run_ego_variant(
    scenario,
    assessor,
    mode: str,
    *,
    duration: float,
    horizon: int,
    r_safe: float | None,
    staged_cfg: StagedPolicyConfig,
):
    if mode == "binary":
        policy = HysteresisPolicy(t_on=2.0, t_off=3.0, a_brake=6.0)
        return run_sim(
            scenario,
            assessor,
            policy,
            duration=duration,
            horizon=horizon,
            r_safe=r_safe,
        )
    if mode == "staged":
        return rollout_staged_ego(
            scenario,
            assessor,
            staged_cfg,
            duration=duration,
            horizon=horizon,
            r_safe=r_safe,
        )
    raise ValueError(f"Unknown mode: {mode!r}")


def _plot_bidirectional_outcomes(df: pd.DataFrame, follower: FollowerConfig, out_path: Path):
    dangerous = df.loc[df["label"] == DANGEROUS]
    benign = df.loc[df["label"] == BENIGN]

    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(13.2, 5.2), gridspec_kw={"width_ratios": [1.3, 1.0]}
    )

    labels = [item[1] for item in VARIANTS]
    keys = [item[0] for item in VARIANTS]
    x = np.arange(len(keys))

    safe_rates, rear_only_rates, front_rates = [], [], []
    min_gaps, peak_jerks, benign_rear_rates = [], [], []

    for key in keys:
        sub_d = dangerous.loc[dangerous["variant"] == key]
        safe_rates.append(100.0 * float(sub_d["safe_both"].mean()))
        rear_only_rates.append(
            100.0 * float((~sub_d["front_collision"] & sub_d["rear_collision"]).mean())
        )
        front_rates.append(100.0 * float(sub_d["front_collision"].mean()))
        min_gaps.append(float(sub_d["min_bumper_gap"].mean()))
        peak_jerks.append(float(sub_d["ego_peak_jerk"].mean()))

        sub_b = benign.loc[benign["variant"] == key]
        benign_rear_rates.append(
            100.0 * float(sub_b["rear_collision"].mean()) if len(sub_b) else 0.0
        )

    safe_arr = np.asarray(safe_rates)
    rear_arr = np.asarray(rear_only_rates)
    front_arr = np.asarray(front_rates)

    w = 0.62
    ax1.bar(x, safe_arr, width=w, color="#2E8B57", label="Both Ends Safe (No Front or Rear Hit)")
    ax1.bar(
        x,
        rear_arr,
        width=w,
        bottom=safe_arr,
        color="#F28E2B",
        label="Front Saved, Rear-Ended by Follower",
    )
    ax1.bar(
        x,
        front_arr,
        width=w,
        bottom=safe_arr + rear_arr,
        color="#D64541",
        label="Front Cut-in Collision",
    )

    for idx, val in enumerate(safe_arr):
        if val >= 8.0:
            ax1.text(
                x[idx],
                val * 0.5,
                f"{val:.1f}%",
                ha="center",
                va="center",
                color="white",
                fontsize=9,
                fontweight="bold",
            )
        if rear_arr[idx] >= 8.0:
            ax1.text(
                x[idx],
                safe_arr[idx] + rear_arr[idx] * 0.5,
                f"{rear_arr[idx]:.1f}%",
                ha="center",
                va="center",
                color="white",
                fontsize=8.5,
                fontweight="bold",
            )

    ax1.set_xticks(x, labels, fontsize=9)
    ax1.set_ylim(0, 108)
    ax1.set_ylabel("Share of Dangerous Cut-in Scenarios [%]")
    ax1.set_title(
        f"A. Bidirectional Safety Breakdown (d_rear={follower.init_rear_gap:g} m, "
        f"tau={follower.reaction_time:g} s)"
    )
    ax1.grid(axis="y", alpha=0.25)
    ax1.legend(loc="upper left", fontsize=8.5, framealpha=0.92)

    # Panel B: Bidirectional Safety Rate vs Peak Ego Jerk (Safety-Comfort Frontier)
    label_offsets = {
        "reactive_binary": (-6, -15),
        "reactive_staged": (8, -14),
        "cv_binary": (-10, 10),
        "cv_staged": (10, -4),
        "ctrv_staged": (10, -13),
        "nn_ua_staged": (10, -3),
        "oracle_staged": (10, 5),
    }
    for idx, (key, lbl, _, mode, col) in enumerate(VARIANTS):
        marker = "s" if mode == "binary" else "o"
        ax2.scatter(
            peak_jerks[idx],
            safe_arr[idx],
            s=115,
            color=col,
            marker=marker,
            edgecolors="#1c302f",
            linewidths=1.0,
            zorder=4,
            label=lbl.replace("\n", " "),
        )
        dx, dy = label_offsets.get(key, (8, 6))
        align = "right" if dx < 0 else "left"
        ax2.annotate(
            lbl.replace("\n", " "),
            (peak_jerks[idx], safe_arr[idx]),
            xytext=(dx, dy),
            textcoords="offset points",
            ha=align,
            fontsize=8.2,
        )

    ax2.axvline(10.0, color="#526360", linestyle="--", linewidth=1.0, alpha=0.7)
    ax2.set_xlim(2, 68)
    ax2.set_ylim(min(safe_arr) - 8, min(102.0, max(safe_arr) + 9))
    ax2.text(
        11.2,
        ax2.get_ylim()[0] + 2.0,
        "Comfort jerk cap (10 m/s³)",
        fontsize=8,
        color="#526360",
    )
    ax2.set_xlabel("Mean Peak Ego Jerk [m/s³] (lower = smoother)")
    ax2.set_ylabel("Bidirectional Safe Rate [%] (higher = safer)")
    ax2.set_title("B. Safety-Smoothness Pareto Frontier")
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def _plot_rear_safety_boundary(
    synth_plane: pd.DataFrame,
    waymo_plane: pd.DataFrame | None,
    out_path: Path,
):
    ncols = 2 if waymo_plane is not None and not waymo_plane.empty else 1
    fig, axes = plt.subplots(1, ncols, figsize=(12.8 if ncols == 2 else 6.8, 5.2), sharey=True)
    if ncols == 1:
        axes = [axes]

    compare_specs = [
        ("reactive_binary", "Reactive + Binary (-6 m/s²)", "#4C78A8", "--"),
        ("reactive_staged", "Reactive + Staged", "#7FA6C9", ":"),
        ("cv_binary", "CV + Binary (-6 m/s²)", "#E45756", "-."),
        ("cv_staged", "CV + Staged (-2.5/-6 m/s²)", "#F58518", "-"),
        ("nn_ua_staged", "NN-UA + Staged (-2.5/-6 m/s²)", "#2E8B57", "-"),
    ]

    def _draw_plane(ax, plane_df: pd.DataFrame, title: str):
        for key, label, color, ls in compare_specs:
            sub = plane_df.loc[plane_df["variant"] == key]
            if sub.empty:
                continue
            piv = (
                sub.pivot(index="reaction_time", columns="init_rear_gap", values="safe_both_rate")
                .sort_index()
                .sort_index(axis=1)
            )
            xs = piv.columns.to_numpy(dtype=float)
            ys = piv.index.to_numpy(dtype=float)
            z = piv.to_numpy(dtype=float)
            safe_frac = 100.0 * float(np.mean(z >= 0.5))
            lbl = f"{label} (safe area {safe_frac:.0f}%)"
            if np.nanmin(z) < 0.5 <= np.nanmax(z):
                ax.contour(xs, ys, z, levels=[0.5], colors=[color], linestyles=[ls], linewidths=2.2)
                ax.plot([], [], color=color, linestyle=ls, linewidth=2.2, label=lbl)
            else:
                state_txt = "100% safe" if np.nanmin(z) >= 0.5 else "0% safe"
                ax.plot(
                    [],
                    [],
                    color=color,
                    linestyle=ls,
                    linewidth=2.2,
                    label=f"{label} ({state_txt})",
                )

            if key == "nn_ua_staged":
                ax.contourf(xs, ys, z, levels=[0.5, 1.01], colors=["#2E8B57"], alpha=0.10)

        ax.set_xlabel("Initial Rear Bumper Gap d_rear,0 [m]")
        ax.set_title(title)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8.0, framealpha=0.92)

    _draw_plane(
        axes[0],
        synth_plane,
        "A. Synthetic Cut-in Suite: 50% Bidirectional Safe Frontier",
    )
    axes[0].set_ylabel("Follower Reaction Delay tau_rear [s]")
    axes[0].text(
        0.97,
        0.06,
        "Bottom-right of curve\n= Safe Zone",
        transform=axes[0].transAxes,
        ha="right",
        va="bottom",
        fontsize=8.5,
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "#f1f0eb", "edgecolor": "#cbd2c9"},
    )

    if ncols == 2 and waymo_plane is not None:
        _draw_plane(
            axes[1],
            waymo_plane,
            "B. Real Waymo Dangerous Cut-in (6e86a639df5d4fa1)",
        )

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def _plot_sandwich_timeline(staged_cfg: StagedPolicyConfig, out_path: Path):
    scenario = make_scenario(
        "cut_in",
        gap_d=24.0,
        rel_v=5.0,
        cut_in_t=0.5,
        lc_T=3.0,
        a_cutter=-1.5,
        duration=23.0,
    )
    follower = FollowerConfig(init_rear_gap=9.5, reaction_time=1.1, max_decel=6.0, max_jerk=10.0)
    sim_kw = dict(duration=10.0, horizon=40, r_safe=None)

    runs = [
        (
            "Reactive + Binary (-6 m/s²)",
            run_sim(
                scenario,
                ReactiveAssessor(),
                HysteresisPolicy(t_on=2.0, t_off=3.0, a_brake=6.0),
                **sim_kw,
            ),
            "#4C78A8",
        ),
        (
            "Predictive CV + Binary (-6 m/s²)",
            run_sim(
                scenario,
                PredictiveAssessor(ConstantVelocityPredictor()),
                HysteresisPolicy(t_on=2.0, t_off=3.0, a_brake=6.0),
                **sim_kw,
            ),
            "#E45756",
        ),
        (
            "Predictive NN-UA + Staged (-2.5/-6 m/s²)",
            rollout_staged_ego(
                scenario,
                _make_assessor("nn_ua"),
                staged_cfg,
                **sim_kw,
            ),
            "#2E8B57",
        ),
    ]

    fig, axes = plt.subplots(4, 1, figsize=(10.2, 9.6), sharex=True)

    for label, slog, color in runs:
        rlog = simulate_following_vehicle(slog, follower)
        ttc_clean = np.where(np.isfinite(slog.ttc), np.minimum(slog.ttc, 5.0), np.nan)
        axes[0].plot(slog.t, ttc_clean, color=color, linewidth=2.0, label=label)
        axes[1].plot(rlog.t, rlog.ego_v, color=color, linewidth=2.0, label=f"{label} (ego)")
        axes[1].plot(
            rlog.t, rlog.rear_v, color=color, linestyle="--", linewidth=1.3, alpha=0.8,
            label=f"{label} (follower)",
        )
        axes[2].plot(rlog.t, rlog.ego_a, color=color, linewidth=2.0, label=f"{label} (ego)")
        axes[2].plot(
            rlog.t, rlog.rear_a, color=color, linestyle="--", linewidth=1.3, alpha=0.8
        )
        status = (
            "BOTH SAFE"
            if rlog.safe_both
            else ("FRONT+REAR CRASH" if rlog.front_collision and rlog.rear_collision
                  else ("FRONT CRASH" if rlog.front_collision else "REAR-ENDED!"))
        )
        axes[3].plot(
            rlog.t,
            np.maximum(0.0, rlog.bumper_gap),
            color=color,
            linewidth=2.1,
            label=f"{label} -> {status} (min {rlog.min_bumper_gap:.2f} m)",
        )
        if rlog.rear_collision and rlog.first_rear_collision_t is not None:
            axes[3].scatter(
                [rlog.first_rear_collision_t],
                [0.0],
                color=color,
                marker="X",
                s=90,
                zorder=5,
            )

    axes[0].axhline(
        3.0, color="#8c8c8c", linestyle=":", linewidth=1.0, label="Staged soft t_on (3.0 s)"
    )
    axes[0].axhline(
        2.0, color="#595959", linestyle="--", linewidth=1.0, label="Binary t_on (2.0 s)"
    )
    axes[0].axhline(
        1.5, color="#a83c30", linestyle=":", linewidth=1.0, label="Staged hard t_on (1.5 s)"
    )
    axes[0].set_ylabel("Estimated TTC [s]")
    axes[0].set_ylim(0, 5.2)
    axes[0].set_title(
        "Rear-End Sandwich Anatomy: Decelerating Cut-in\n"
        "(gap=24 m, rel_v=5 m/s, a_cut=-1.5 m/s², d_rear=9.5 m, tau_rear=1.1 s)",
        fontsize=11,
    )
    axes[0].legend(loc="upper right", fontsize=8, ncol=2)
    axes[0].grid(alpha=0.28)

    axes[1].set_ylabel("Speed [m/s]\n(solid=ego, dash=rear)")
    axes[1].grid(alpha=0.28)

    axes[2].axhline(-2.5, color="#2E8B57", linestyle=":", alpha=0.5, linewidth=1.0)
    axes[2].axhline(-6.0, color="#E45756", linestyle=":", alpha=0.5, linewidth=1.0)
    axes[2].set_ylabel("Accel [m/s²]\n(solid=ego, dash=rear)")
    axes[2].set_ylim(-6.6, 2.0)
    axes[2].grid(alpha=0.28)

    axes[3].axhline(
        0.0, color="#a83c30", linestyle="--", linewidth=1.2, label="Bumper contact (0 m)"
    )
    axes[3].set_ylabel("Rear Bumper Gap [m]")
    axes[3].set_xlabel("Simulation Time t [s]")
    axes[3].legend(loc="upper right", fontsize=8.2)
    axes[3].grid(alpha=0.28)

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def main(argv: list[str] | None = None) -> dict:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--quick", action="store_true", help="Run compact scenario & headway grids."
    )
    args = parser.parse_args(argv)

    FIG_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    staged_cfg = StagedPolicyConfig()
    follower_nominal = FollowerConfig(
        init_rear_gap=9.5,
        reaction_time=1.1,
        max_decel=6.0,
        max_jerk=10.0,
        model="kinematic",
    )

    scenarios = _cut_in_suite(quick=args.quick)
    sim_kwargs = {"duration": OUTCOME_DURATION, "horizon": 40, "r_safe": None}

    labels_by_id: dict[str, str] = {}
    for sc in scenarios:
        sid = scenario_id(sc)
        lbl, _ = never_brake_label(sc, DECISION_DURATION, **sim_kwargs)
        labels_by_id[sid] = lbl

    assessors = {sys_key: _make_assessor(sys_key) for _, _, sys_key, _, _ in VARIANTS}
    logs_by_variant: dict[str, list[tuple[str, str, object]]] = {v[0]: [] for v in VARIANTS}
    dangerous_logs_for_plane: dict[str, list] = {v[0]: [] for v in VARIANTS}

    for sc in scenarios:
        sid = scenario_id(sc)
        lbl = labels_by_id[sid]
        for key, _, sys_key, mode, _ in VARIANTS:
            slog = _run_ego_variant(
                sc,
                assessors[sys_key],
                mode,
                duration=OUTCOME_DURATION,
                horizon=40,
                r_safe=None,
                staged_cfg=staged_cfg,
            )
            logs_by_variant[key].append((sid, lbl, slog))
            if lbl == DANGEROUS:
                dangerous_logs_for_plane[key].append(slog)

    outcomes_df = summarize_bidirectional_logs(logs_by_variant, follower_nominal)
    outcomes_csv = LOG_DIR / "exp09_cut_in_outcomes.csv"
    outcomes_df.to_csv(outcomes_csv, index=False)

    # Also cross-check under Delayed-IDM follower model
    idm_df = summarize_bidirectional_logs(
        logs_by_variant,
        FollowerConfig(
            init_rear_gap=follower_nominal.init_rear_gap,
            reaction_time=follower_nominal.reaction_time,
            max_decel=follower_nominal.max_decel,
            max_jerk=follower_nominal.max_jerk,
            model="idm",
        ),
    )

    rear_gaps = (
        tuple(np.linspace(4.0, 22.0, 7))
        if args.quick
        else tuple(np.linspace(4.0, 24.0, 21))
    )
    reaction_times = (
        tuple(np.linspace(0.3, 1.7, 6))
        if args.quick
        else tuple(np.linspace(0.2, 1.8, 17))
    )

    synth_plane = sweep_rear_headway_plane(
        dangerous_logs_for_plane,
        rear_gaps=rear_gaps,
        reaction_times=reaction_times,
        base_config=follower_nominal,
    )
    synth_plane.to_csv(LOG_DIR / "exp09_rear_plane.csv", index=False)

    waymo_plane = None
    waymo_summary = {}
    if WAYMO_DANGER_CLIP.is_file():
        w_scene = load_clip(WAYMO_DANGER_CLIP)
        # Match A's frozen Waymo clip parameters (duration=6.0, horizon=30, r_safe=2.5)
        waymo_logs = {}
        for key, _, sys_key, mode, _ in VARIANTS:
            if sys_key == "oracle":
                continue  # Waymo clip truth ends at 6.0 s (insufficient for 6+3 s oracle)
            waymo_logs[key] = _run_ego_variant(
                w_scene,
                assessors[sys_key],
                mode,
                duration=6.0,
                horizon=30,
                r_safe=float(R_SAFE),
                staged_cfg=staged_cfg,
            )
        waymo_plane = sweep_rear_headway_plane(
            waymo_logs,
            rear_gaps=rear_gaps,
            reaction_times=reaction_times,
            base_config=follower_nominal,
        )
        waymo_plane.to_csv(LOG_DIR / "exp09_waymo_plane.csv", index=False)
        for key, wlog in waymo_logs.items():
            sub_w = waymo_plane.loc[waymo_plane["variant"] == key]
            r_nom = simulate_following_vehicle(wlog, follower_nominal)
            waymo_summary[key] = {
                "front_collision": bool(r_nom.front_collision),
                "ego_brake_onset_s": r_nom.ego_brake_onset_t,
                "ego_peak_decel_mps2": float(r_nom.ego_peak_decel),
                "ego_peak_jerk_mps3": float(r_nom.ego_peak_jerk),
                "nominal_rear_collision": bool(r_nom.rear_collision),
                "nominal_min_rear_gap_m": float(r_nom.min_bumper_gap),
                "plane_safe_both_area": float(sub_w["safe_both_rate"].mean()),
            }

    _plot_bidirectional_outcomes(
        outcomes_df,
        follower_nominal,
        FIG_DIR / "exp09_bidirectional_outcomes.png",
    )
    _plot_rear_safety_boundary(
        synth_plane,
        waymo_plane,
        FIG_DIR / "exp09_rear_safety_boundary.png",
    )
    _plot_sandwich_timeline(
        staged_cfg,
        FIG_DIR / "exp09_sandwich_timeline.png",
    )

    variant_stats = {}
    for key, label, sys_key, mode, _ in VARIANTS:
        sub_d = outcomes_df.loc[
            (outcomes_df["variant"] == key) & (outcomes_df["label"] == DANGEROUS)
        ]
        sub_b = outcomes_df.loc[
            (outcomes_df["variant"] == key) & (outcomes_df["label"] == BENIGN)
        ]
        sub_idm_d = idm_df.loc[
            (idm_df["variant"] == key) & (idm_df["label"] == DANGEROUS)
        ]
        sub_plane = synth_plane.loc[synth_plane["variant"] == key]
        variant_stats[key] = {
            "label": label.replace("\n", " "),
            "system": sys_key,
            "controller": mode,
            "n_dangerous": int(len(sub_d)),
            "n_benign": int(len(sub_b)),
            "dangerous_front_collision_rate": float(sub_d["front_collision"].mean()),
            "dangerous_rear_collision_rate": float(sub_d["rear_collision"].mean()),
            "dangerous_front_saved_rear_hit_rate": float(
                (~sub_d["front_collision"] & sub_d["rear_collision"]).mean()
            ),
            "dangerous_safe_both_rate": float(sub_d["safe_both"].mean()),
            "dangerous_safe_both_rate_idm": float(sub_idm_d["safe_both"].mean()),
            "benign_rear_collision_rate": (
                float(sub_b["rear_collision"].mean()) if len(sub_b) else 0.0
            ),
            "mean_min_rear_gap_m": float(sub_d["min_bumper_gap"].mean()),
            "mean_ego_peak_decel_mps2": float(sub_d["ego_peak_decel"].mean()),
            "mean_ego_peak_jerk_mps3": float(sub_d["ego_peak_jerk"].mean()),
            "rear_plane_50pct_safe_area": float((sub_plane["safe_both_rate"] >= 0.5).mean()),
            "rear_plane_mean_safe_both_rate": float(sub_plane["safe_both_rate"].mean()),
        }

    summary = {
        "experiment": "exp09_rear_end_safety",
        "quick": bool(args.quick),
        "n_total_scenarios": len(scenarios),
        "n_dangerous": int(sum(v == DANGEROUS for v in labels_by_id.values())),
        "n_benign": int(sum(v == BENIGN for v in labels_by_id.values())),
        "nominal_follower": {
            "init_rear_gap_m": follower_nominal.init_rear_gap,
            "reaction_time_s": follower_nominal.reaction_time,
            "max_decel_mps2": follower_nominal.max_decel,
            "max_jerk_mps3": follower_nominal.max_jerk,
            "model": follower_nominal.model,
        },
        "staged_policy": {
            "t_on_soft_s": staged_cfg.t_on_soft,
            "t_off_soft_s": staged_cfg.t_off_soft,
            "t_on_hard_s": staged_cfg.t_on_hard,
            "t_off_hard_s": staged_cfg.t_off_hard,
            "a_soft_mps2": staged_cfg.a_soft,
            "a_hard_mps2": staged_cfg.a_hard,
            "max_jerk_mps3": staged_cfg.max_jerk,
        },
        "variants": variant_stats,
        "waymo_dangerous_case": waymo_summary,
    }
    summary_path = LOG_DIR / "exp09_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


if __name__ == "__main__":
    result = main()
    print(json.dumps(result["variants"], indent=2))

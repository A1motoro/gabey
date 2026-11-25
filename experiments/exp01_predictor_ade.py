"""exp01 — P L2/L3: motion & neural predictors on identical windows.

Uses A's CA grid plus lead braking and crossing.
- Default: L2 four-model comparison (CV / CA / CTRV / Oracle).
- ``--include-lr``: adds P's L3 LinearRegressionPredictor (5 models).
- ``--include-nn``: adds P's L3 NeuralPredictor variants (NN-Traj / NN-Lane) and Gaussian NLL.
- ``--include-waymo-clips`` / ``--waymo-val``: optional open-loop evaluation on Waymo clips/NPZ.
  Illustrative outcome-selected clips (``cohort="illustrative"``) are always kept separate
  from primary Waymo cohort statistics (per ``docs/interfaces.md``).
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from pbs.data import ca_scenario_set, make_scenario, prediction_slices
from pbs.data.womd import load_clip
from pbs.eval.metrics import ade, fde
from pbs.prediction import (
    ConstantAccelPredictor,
    ConstantVelocityPredictor,
    CTRVPredictor,
    LinearRegressionPredictor,
    NeuralPredictor,
    OraclePredictor,
    gaussian_nll_2d,
)
from pbs.prediction.features import (
    HISTORY_STEPS,
    TRAJ_POS_SCALE,
    TRAJ_VEL_SCALE,
    world_to_agent,
)

ROOT = Path(__file__).resolve().parents[1]
FIXTURES_DIR = ROOT / "tests/fixtures"


def _infer_is_unnormalized(hist: np.ndarray, future: np.ndarray) -> bool:
    """Speed-invariant physical ratio check: |future[:,0,0]| / |hist[:,-1,2]| is ~0.1 vs ~2.0."""
    vx = np.abs(hist[:, -1, 2].astype(np.float64))
    dx1 = np.abs(future[:, 0, 0].astype(np.float64))
    moving = (vx > 1e-4) & (dx1 > 0.05)
    if np.any(moving):
        ratio = float(np.median(dx1[moving] / vx[moving]))
        return ratio < 0.5
    return float(np.max(np.abs(hist[:, :, :4]))) > 3.0


def _eval_waymo_clips(predictors: list, *, include_nn: bool) -> pd.DataFrame:
    """Open-loop ADE/FDE/NLL over frozen Waymo NPZ clips, strictly separated by cohort."""
    clip_paths = sorted(FIXTURES_DIR.glob("womd_*.npz")) + sorted(
        (FIXTURES_DIR / "cases").glob("womd_*.npz")
    )
    rows = []
    for index, clip_path in enumerate(clip_paths):
        sc = load_clip(clip_path)
        cohort = str(sc.params.get("cohort", "primary"))
        for predictor in predictors:
            predictor.bind(sc)
        for sl in prediction_slices(sc, history=1.0, horizon=3.0, stride=0.5):
            origin = np.array([sl.history.x[-1], sl.history.y[-1]], dtype=float)
            theta0 = float(sl.history.heading[-1])
            gt_local = world_to_agent(sl.truth, origin, theta0)
            for predictor in predictors:
                out = predictor.predict(sl.history, len(sl.truth), sc.dt)
                std = getattr(predictor, "latest_std", None) if include_nn else None
                pred_local = world_to_agent(out, origin, theta0) if std is not None else None
                for seconds in (0.5, 1.0, 2.0, 3.0):
                    n = int(round(seconds / sc.dt))
                    row = {
                        "scenario": index,
                        "cohort": cohort,
                        "family": sc.kind,
                        "clip": clip_path.stem,
                        "t0_s": sl.t0,
                        "model": predictor.name,
                        "horizon_s": seconds,
                        "ade_m": ade(out[:n], sl.truth[:n]),
                        "fde_m": fde(out[:n], sl.truth[:n]),
                    }
                    if include_nn:
                        if std is not None and pred_local is not None:
                            row["nll_nats"] = float(
                                gaussian_nll_2d(pred_local[:n], std[:n], gt_local[:n]).mean()
                            )
                            row["cov_1sigma_lat"] = float(
                                np.mean(np.abs(gt_local[:n, 1] - pred_local[:n, 1]) <= std[:n, 1])
                            )
                        else:
                            row["nll_nats"] = np.nan
                            row["cov_1sigma_lat"] = np.nan
                    rows.append(row)
    return pd.DataFrame(rows)


def _eval_waymo_val_npz(val_npz: Path, nn_predictors: list[NeuralPredictor]) -> pd.DataFrame:
    """Evaluate CV and NeuralPredictor checkpoints on a Contract-1 Waymo validation .npz."""
    with np.load(val_npz, allow_pickle=False) as z:
        hist = np.asarray(z["hist_traj"], dtype=np.float32).copy()
        lane = np.asarray(z["lane_ctx"], dtype=np.float32).copy()
        future = np.asarray(z["future_xy"], dtype=np.float64)
        normalized = bool(int(z["traj_normalized"])) if "traj_normalized" in z else None

    if len(hist) == 0 or hist.ndim != 3 or hist.shape[1:] != (HISTORY_STEPS, 6):
        raise ValueError(f"Invalid hist_traj shape in {val_npz}: {hist.shape}")
    if future.ndim != 3 or future.shape[0] != len(hist) or future.shape[2] != 2:
        raise ValueError(f"Invalid future_xy shape in {val_npz}: {future.shape}")
    if not (np.isfinite(hist).all() and np.isfinite(lane).all() and np.isfinite(future).all()):
        raise ValueError(f"Dataset {val_npz} contains non-finite values")

    if normalized is False or (normalized is None and _infer_is_unnormalized(hist, future)):
        hist[:, :, 0:2] /= TRAJ_POS_SCALE
        hist[:, :, 2:4] /= TRAJ_VEL_SCALE

    dt = 0.1
    horizon_steps = future.shape[1]
    tau = np.arange(1, horizon_steps + 1, dtype=np.float64) * dt
    v_local = hist[:, HISTORY_STEPS - 1, 2:4].astype(np.float64) * TRAJ_VEL_SCALE
    cv_pred = tau[None, :, None] * v_local[:, None, :]

    rows = []
    horizons = [s for s in (0.5, 1.0, 2.0, 3.0, 4.0) if int(round(s / dt)) <= horizon_steps]
    cv_disp = np.linalg.norm(cv_pred - future, axis=-1)
    for seconds in horizons:
        n = int(round(seconds / dt))
        rows.append(
            {
                "model": "cv",
                "horizon_s": seconds,
                "ade_m": float(cv_disp[:, :n].mean()),
                "fde_m": float(cv_disp[:, n - 1].mean()),
                "nll_nats": np.nan,
                "cov_1sigma_lat": np.nan,
                "windows": int(len(future)),
            }
        )

    for pred in nn_predictors:
        mu_raw, std_raw = pred.forward_batch(
            hist, lane if pred.use_lane else None, traj_normalized=True
        )
        if horizon_steps <= pred.model_horizon:
            mu, std = mu_raw[:, :horizon_steps], std_raw[:, :horizon_steps]
        else:
            resampled = [
                pred._resample_horizon(mu_raw[i], std_raw[i], horizon_steps, dt)
                for i in range(len(hist))
            ]
            mu = np.stack([pair[0] for pair in resampled], axis=0)
            std = np.stack([pair[1] for pair in resampled], axis=0)
        disp = np.linalg.norm(mu - future, axis=-1)
        nll = gaussian_nll_2d(mu, std, future)
        lat_hit = np.abs(future[:, :, 1] - mu[:, :, 1]) <= std[:, :, 1]
        for seconds in horizons:
            n = int(round(seconds / dt))
            rows.append(
                {
                    "model": pred.name,
                    "horizon_s": seconds,
                    "ade_m": float(disp[:, :n].mean()),
                    "fde_m": float(disp[:, n - 1].mean()),
                    "nll_nats": float(nll[:, :n].mean()),
                    "cov_1sigma_lat": float(lat_hit[:, :n].mean()),
                    "windows": int(len(future)),
                }
            )
    return pd.DataFrame(rows)


def main(
    output_dir: Path = ROOT / "outputs/figures/l2-predictors",
    *,
    include_lr: bool = False,
    include_nn: bool = False,
    include_waymo_clips: bool = False,
    waymo_val_npz: Path | None = None,
) -> dict:
    if waymo_val_npz is not None:
        if not include_nn:
            raise ValueError("--waymo-val requires --include-nn")
        if not waymo_val_npz.is_file():
            raise FileNotFoundError(f"Waymo validation dataset not found: {waymo_val_npz}")

    output_dir.mkdir(parents=True, exist_ok=True)
    scenarios = ca_scenario_set().scenarios + [
        make_scenario("lead_brake"),
        make_scenario("crossing"),
    ]
    predictors = [
        ConstantVelocityPredictor(),
        ConstantAccelPredictor(),
        CTRVPredictor(),
        OraclePredictor(),
    ]
    if include_lr:
        predictors.insert(-1, LinearRegressionPredictor())
    nn_predictors: list[NeuralPredictor] = []
    if include_nn:
        nn_traj = NeuralPredictor(use_lane=False)
        nn_lane = NeuralPredictor(use_lane=True)
        nn_predictors = [nn_traj, nn_lane]
        predictors.insert(-1, nn_traj)
        predictors.insert(-1, nn_lane)

    rows = []
    max_oracle_error = 0.0
    for index, sc in enumerate(scenarios):
        label = f"cut_in a={sc.params['a_cutter']:+g}" if sc.kind == "cut_in" else sc.kind
        for predictor in predictors:
            predictor.bind(sc)
        # All horizons share the same forecast origins; no change of denominator.
        for sl in prediction_slices(sc, history=1.0, horizon=3.0, stride=0.5):
            origin = np.array([sl.history.x[-1], sl.history.y[-1]], dtype=float)
            theta0 = float(sl.history.heading[-1])
            gt_local = world_to_agent(sl.truth, origin, theta0) if include_nn else None
            for predictor in predictors:
                out = predictor.predict(sl.history, len(sl.truth), sc.dt)
                if predictor.name == "oracle":
                    max_oracle_error = max(max_oracle_error, float(np.abs(out - sl.truth).max()))
                std = getattr(predictor, "latest_std", None) if include_nn else None
                pred_local = world_to_agent(out, origin, theta0) if std is not None else None
                for seconds in (0.5, 1.0, 2.0, 3.0):
                    n = int(round(seconds / sc.dt))
                    row = {
                        "scenario": index,
                        "family": label,
                        "t0_s": sl.t0,
                        "model": predictor.name,
                        "horizon_s": seconds,
                        "ade_m": ade(out[:n], sl.truth[:n]),
                        "fde_m": fde(out[:n], sl.truth[:n]),
                    }
                    if include_nn:
                        if std is not None and pred_local is not None and gt_local is not None:
                            row["nll_nats"] = float(
                                gaussian_nll_2d(pred_local[:n], std[:n], gt_local[:n]).mean()
                            )
                            row["cov_1sigma_lat"] = float(
                                np.mean(np.abs(gt_local[:n, 1] - pred_local[:n, 1]) <= std[:n, 1])
                            )
                        else:
                            row["nll_nats"] = np.nan
                            row["cov_1sigma_lat"] = np.nan
                    rows.append(row)

    errors = pd.DataFrame(rows)
    errors.to_csv(output_dir / "prediction_errors.csv", index=False)
    grouped = errors.groupby(["family", "model", "horizon_s"], sort=True)
    agg_spec: dict[str, tuple[str, str]] = {
        "ade_m": ("ade_m", "mean"),
        "fde_m": ("fde_m", "mean"),
        "windows": ("ade_m", "size"),
    }
    if include_nn:
        agg_spec["nll_nats"] = ("nll_nats", "mean")
        agg_spec["cov_1sigma_lat"] = ("cov_1sigma_lat", "mean")
    accuracy = grouped.agg(**agg_spec).reset_index()
    accuracy.to_csv(output_dir / "predictor_ade_fde.csv", index=False)

    families = sorted(accuracy.family.unique())
    metrics_to_plot = (
        (("ade_m", "ADE [m]"), ("fde_m", "FDE [m]"), ("nll_nats", "Gaussian NLL [nats]"))
        if include_nn
        else (("ade_m", "ADE [m]"), ("fde_m", "FDE [m]"))
    )
    fig, axes = plt.subplots(
        len(metrics_to_plot), len(families), figsize=(20, 3.5 * len(metrics_to_plot)), squeeze=False
    )
    for column, family in enumerate(families):
        values = accuracy[accuracy.family == family]
        for row_idx, (metric, ylabel) in enumerate(metrics_to_plot):
            ax = axes[row_idx, column]
            for model, data in values.groupby("model"):
                if data[metric].isna().all():
                    continue
                ax.plot(data.horizon_s, data[metric], "o-", label=model)
            ax.set(title=family, xlabel="Horizon [s]", ylabel=ylabel)
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output_dir / "exp01_predictor_ade.png", dpi=150)
    plt.close(fig)

    summary = {
        "models": [p.name for p in predictors],
        "scenarios": len(scenarios),
        "history_window_s": 1.0,
        "ca_ctrv_estimation_samples": 10,
        "lr_estimation_samples": 10 if include_lr else None,
        "nn_hidden_dims": {p.name: p.hidden_dim for p in nn_predictors} if include_nn else None,
        "stride_s": 0.5,
        "horizons_s": [0.5, 1, 2, 3],
        "forecast_origins_per_scenario": len(errors) // (len(scenarios) * len(predictors) * 4),
        "oracle_max_error_m": max_oracle_error,
        "claim": "Synthetic prediction accuracy; safety benefits require separate experiments.",
        "oracle_endpoint_rule": "Endpoint hold outside truth coverage; benchmark stays inside.",
    }

    if include_nn:
        nn_summary = {}
        for p in nn_predictors:
            sub = errors[(errors.model == p.name) & (errors.horizon_s == 2.0)]
            nn_summary[p.name] = {
                "ade_2s_m": round(float(sub["ade_m"].mean()), 4),
                "fde_2s_m": round(float(sub["fde_m"].mean()), 4),
                "nll_2s_nats": round(float(sub["nll_nats"].mean()), 4),
                "cov_1sigma_lat_2s": round(float(sub["cov_1sigma_lat"].mean()), 4),
            }
        summary["nn_synthetic_2s"] = nn_summary

    if include_waymo_clips:
        waymo_df = _eval_waymo_clips(predictors, include_nn=include_nn)
        waymo_df.to_csv(output_dir / "waymo_clips_errors.csv", index=False)
        # Keep primary (5+5) and outcome-selected illustrative cohorts strictly separate.
        w_grouped = waymo_df.groupby(["cohort", "model", "horizon_s"], sort=True)
        w_agg: dict[str, tuple[str, str]] = {
            "ade_m": ("ade_m", "mean"),
            "fde_m": ("fde_m", "mean"),
            "windows": ("ade_m", "size"),
        }
        if include_nn:
            w_agg["nll_nats"] = ("nll_nats", "mean")
            w_agg["cov_1sigma_lat"] = ("cov_1sigma_lat", "mean")
        waymo_acc = w_grouped.agg(**w_agg).reset_index()
        waymo_acc.to_csv(output_dir / "waymo_clips_ade_fde.csv", index=False)
        cohort_counts = {}
        for cohort, grp in waymo_df.groupby("cohort"):
            cohort_counts[str(cohort)] = int(len(grp) // (len(predictors) * 4))
        summary["waymo_clips_windows_by_cohort"] = cohort_counts

    if waymo_val_npz is not None:
        val_df = _eval_waymo_val_npz(waymo_val_npz, nn_predictors)
        val_df.to_csv(output_dir / "waymo_val_ade_fde_nll.csv", index=False)
        summary["waymo_val_npz"] = str(waymo_val_npz)

    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Artifacts: {output_dir.resolve()}")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--include-lr", action="store_true", help="include the L3 linear fit")
    parser.add_argument(
        "--include-nn",
        action="store_true",
        help="include L3 NeuralPredictor (NN-Traj / NN-Lane) and Gaussian NLL",
    )
    parser.add_argument(
        "--include-waymo-clips",
        action="store_true",
        help="also evaluate open-loop accuracy on frozen Waymo NPZ clips (separated by cohort)",
    )
    parser.add_argument(
        "--waymo-val",
        type=Path,
        default=None,
        help="optional Contract-1 Waymo validation .npz (e.g. data/waymo_val.npz)",
    )
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    default_subdir = "l3-predictors" if (args.include_lr or args.include_nn) else "l2-predictors"
    output = args.output_dir or ROOT / "outputs/figures" / default_subdir
    main(
        output,
        include_lr=args.include_lr,
        include_nn=args.include_nn,
        include_waymo_clips=args.include_waymo_clips,
        waymo_val_npz=args.waymo_val,
    )

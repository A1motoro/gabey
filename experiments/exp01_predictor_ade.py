"""exp01 — P L2/L3: motion predictors on identical synthetic windows.

Uses A's CA grid plus lead braking and crossing. --include-lr adds P's L3 model.
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
from pbs.eval.metrics import ade, fde
from pbs.prediction import (
    ConstantAccelPredictor,
    ConstantVelocityPredictor,
    CTRVPredictor,
    LinearRegressionPredictor,
    OraclePredictor,
)

ROOT = Path(__file__).resolve().parents[1]


def main(
    output_dir: Path = ROOT / "outputs/figures/l2-predictors", *, include_lr: bool = False
) -> dict:
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
    rows = []
    max_oracle_error = 0.0
    for index, sc in enumerate(scenarios):
        label = f"cut_in a={sc.params['a_cutter']:+g}" if sc.kind == "cut_in" else sc.kind
        for predictor in predictors:
            predictor.bind(sc)
        # All horizons share the same forecast origins; no change of denominator.
        for sl in prediction_slices(sc, history=1.0, horizon=3.0, stride=0.5):
            for predictor in predictors:
                out = predictor.predict(sl.history, len(sl.truth), sc.dt)
                if predictor.name == "oracle":
                    max_oracle_error = max(max_oracle_error, float(np.abs(out - sl.truth).max()))
                for seconds in (0.5, 1.0, 2.0, 3.0):
                    n = int(round(seconds / sc.dt))
                    rows.append(
                        {
                            "scenario": index,
                            "family": label,
                            "t0_s": sl.t0,
                            "model": predictor.name,
                            "horizon_s": seconds,
                            "ade_m": ade(out[:n], sl.truth[:n]),
                            "fde_m": fde(out[:n], sl.truth[:n]),
                        }
                    )
    errors = pd.DataFrame(rows)
    errors.to_csv(output_dir / "prediction_errors.csv", index=False)
    grouped = errors.groupby(["family", "model", "horizon_s"], sort=True)
    accuracy = grouped.agg(
        ade_m=("ade_m", "mean"), fde_m=("fde_m", "mean"), windows=("ade_m", "size")
    ).reset_index()
    accuracy.to_csv(output_dir / "predictor_ade_fde.csv", index=False)
    families = sorted(accuracy.family.unique())
    fig, axes = plt.subplots(2, len(families), figsize=(20, 7), squeeze=False)
    for column, family in enumerate(families):
        values = accuracy[accuracy.family == family]
        for row, metric in enumerate(("ade_m", "fde_m")):
            ax = axes[row, column]
            for model, data in values.groupby("model"):
                ax.plot(data.horizon_s, data[metric], "o-", label=model)
            ax.set(title=family, xlabel="Horizon [s]", ylabel=f"{metric[:3].upper()} [m]")
            ax.grid(alpha=0.3)
            ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "exp01_predictor_ade.png", dpi=150)
    plt.close(fig)
    summary = {
        "models": [p.name for p in predictors],
        "scenarios": len(scenarios),
        "history_window_s": 1.0,
        "ca_ctrv_estimation_samples": 10,
        "lr_estimation_samples": 10 if include_lr else None,
        "stride_s": 0.5,
        "horizons_s": [0.5, 1, 2, 3],
        "forecast_origins_per_scenario": len(errors) // (len(scenarios) * len(predictors) * 4),
        "oracle_max_error_m": max_oracle_error,
        "claim": "Synthetic prediction accuracy; safety benefits require separate experiments.",
        "oracle_endpoint_rule": "Endpoint hold outside truth coverage; benchmark stays inside.",
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Artifacts: {output_dir.resolve()}")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--include-lr", action="store_true", help="include the L3 linear fit")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    output = args.output_dir or ROOT / "outputs/figures" / (
        "l3-predictors" if args.include_lr else "l2-predictors"
    )
    main(output, include_lr=args.include_lr)

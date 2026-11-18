"""M4 — experiment matrix. Owner: D.

One row per ``run_sim``. Trade-off curves, failure boundaries, and lead times are
groupbys of that table. ``is_benign`` is cached per scenario and uses the same
``sim_kwargs`` as the measured runs.
"""

import hashlib
import json
import subprocess

import numpy as np
import pandas as pd

from pbs.eval.counterfactual import is_benign
from pbs.eval.metrics import evaluate
from pbs.sim.policy import TTCThresholdPolicy
from pbs.sim.simulator import run_sim
from pbs.systems import make_system
from pbs.types import Scenario

_TRUE = {"1", "true", "t", "yes"}


def run_matrix(
    systems,
    scenarios: list[Scenario],
    thresholds,
    latencies=(0.0,),
    *,
    sim_kwargs=None,
    seed: int = 0,
) -> pd.DataFrame:
    """Run ``systems × scenarios × thresholds × latencies``.

    Columns follow ``docs/d-baseline.md``: scenario id, kind, scalar scenario params,
    system, threshold, latency, benign, collision, false_brake, min_dist,
    brake_onset_t, seed, git_rev. The simulation itself is deterministic; ``seed``
    is recorded so a later stochastic scenario can reuse the same table.
    """
    if not scenarios:
        raise ValueError("scenarios is empty")
    sim_kwargs = dict(sim_kwargs or {})
    names = [str(name) for name in systems]
    threshold_list = [float(value) for value in thresholds]
    latency_list = [float(value) for value in latencies]
    if not names or not threshold_list or not latency_list:
        raise ValueError("systems, thresholds, and latencies must be non-empty")
    param_keys = _param_keys(scenarios)
    revision = _git_rev()
    benign_cache: dict[str, bool] = {}
    rows = []
    for scenario in scenarios:
        sid = scenario_id(scenario)
        if sid not in benign_cache:
            benign_cache[sid] = bool(is_benign(scenario, **sim_kwargs))
        benign = benign_cache[sid]
        params = {key: _scalar(scenario.params.get(key)) for key in param_keys}
        for name in names:
            assessor = make_system(name)
            for threshold in threshold_list:
                policy = TTCThresholdPolicy(ttc_threshold=threshold)
                for latency in latency_list:
                    log = run_sim(scenario, assessor, policy, latency=latency, **sim_kwargs)
                    metrics = evaluate(log, benign)
                    onset = metrics["brake_onset_t"]
                    rows.append(
                        {
                            "scenario_id": sid,
                            "kind": scenario.kind,
                            **params,
                            "system": name,
                            "threshold": threshold,
                            "latency": latency,
                            "benign": benign,
                            "collision": bool(metrics["collision"]),
                            "false_brake": bool(metrics["false_brake"]),
                            "min_dist": float(metrics["min_dist"]),
                            "brake_onset_t": np.nan if onset is None else float(onset),
                            "seed": int(seed),
                            "git_rev": revision,
                        }
                    )
    return pd.DataFrame(rows)


def tradeoff(df: pd.DataFrame) -> pd.DataFrame:
    """Collision rate on dangerous rows and false-brake rate on benign rows."""
    frame = _normalize(df)
    rows = []
    grouped = frame.groupby(["system", "threshold", "latency"], sort=False)
    for (system, threshold, latency), group in grouped:
        dangerous = group.loc[~group["benign"]]
        benign = group.loc[group["benign"]]
        rows.append(
            {
                "system": system,
                "threshold": float(threshold),
                "latency": float(latency),
                "n_dangerous": int(len(dangerous)),
                "n_benign": int(len(benign)),
                "collision_rate": (
                    float(dangerous["collision"].mean()) if len(dangerous) else np.nan
                ),
                "false_brake_rate": float(benign["false_brake"].mean()) if len(benign) else np.nan,
            }
        )
    return pd.DataFrame(rows)


def select_operating_point(
    tradeoff_df: pd.DataFrame, max_false_brake: float = 0.05
) -> pd.DataFrame:
    """Lowest collision rate among thresholds inside the false-brake budget.

    If no threshold meets the budget, the lowest false-brake row is used and
    ``met_budget`` is false.
    Equal collision rates keep the lower false-brake rate, then the lower threshold.
    """
    rows = []
    keys = ["system", "latency"] if "latency" in tradeoff_df.columns else ["system"]
    for _, group in tradeoff_df.groupby(keys, sort=False):
        eligible = group.loc[group["false_brake_rate"] <= max_false_brake]
        pool = eligible if not eligible.empty else group
        pool = pool.sort_values(
            ["collision_rate", "false_brake_rate", "threshold"],
            ascending=[True, True, True],
            na_position="last",
        )
        chosen = pool.iloc[0].to_dict()
        chosen["met_budget"] = bool(len(eligible))
        chosen["false_brake_budget"] = float(max_false_brake)
        rows.append(chosen)
    return pd.DataFrame(rows)


def failure_boundary(df: pd.DataFrame, system: str, threshold=None, latency=None) -> pd.DataFrame:
    """Collision flag on the (gap_d, rel_v) cells of one system at one working point."""
    sub = _subset(df, system, threshold, latency)
    for column in ("gap_d", "rel_v"):
        if column not in sub.columns:
            raise ValueError(f"failure_boundary requires a {column} column")
    cells = sub.groupby(["gap_d", "rel_v"], as_index=False).agg(
        collision=("collision", "max"), benign=("benign", "min")
    )
    cells["collision"] = cells["collision"].astype(bool)
    cells["benign"] = cells["benign"].astype(bool)
    return cells.sort_values(["rel_v", "gap_d"]).reset_index(drop=True)


def safe_area(df: pd.DataFrame, system: str, threshold=None, latency=None) -> float:
    """Fraction of (gap_d, rel_v) cells that are collision-free, benign cells included."""
    cells = failure_boundary(df, system, threshold, latency)
    if cells.empty:
        return float("nan")
    return float((~cells["collision"]).mean())


def rescue_rate(df: pd.DataFrame, system: str, threshold=None, latency=None) -> float:
    """Fraction of dangerous cells that this system leaves collision-free.

    Primary L2 comparison: rescue(CV) - rescue(Reactive) is the value of prediction;
    rescue(Oracle) - rescue(CV) is the cost of prediction error.
    """
    cells = failure_boundary(df, system, threshold, latency)
    dangerous = cells.loc[~cells["benign"]]
    if dangerous.empty:
        return float("nan")
    return float(1.0 - dangerous["collision"].mean())


def lead_times(df: pd.DataFrame, reference: str = "reactive") -> pd.DataFrame:
    """``onset(reference) - onset(system)`` on matching scenario, threshold, and latency.

    Rows where either system never brakes stay in the table with a null lead time.
    ``brake_status`` is ``both``, ``reference_only``, ``predictive_only``, or ``neither``.
    """
    frame = _normalize(df)
    ref_cols = ["scenario_id", "threshold", "latency", "brake_onset_t"]
    missing = [column for column in ref_cols if column not in frame.columns]
    if missing:
        raise ValueError(f"lead_times requires columns {missing}")
    reference_rows = frame.loc[frame["system"] == reference, ref_cols].rename(
        columns={"brake_onset_t": "onset_ref"}
    )
    others = frame.loc[frame["system"] != reference]
    merged = others.merge(reference_rows, on=["scenario_id", "threshold", "latency"], how="inner")
    ref_braked = merged["onset_ref"].notna()
    pred_braked = merged["brake_onset_t"].notna()
    status = np.full(len(merged), "neither", dtype=object)
    status[ref_braked & pred_braked] = "both"
    status[ref_braked & ~pred_braked] = "reference_only"
    status[~ref_braked & pred_braked] = "predictive_only"
    merged["lead_time"] = np.where(
        status == "both", merged["onset_ref"] - merged["brake_onset_t"], np.nan
    )
    merged["defined"] = status == "both"
    merged["brake_status"] = status
    return merged.reset_index(drop=True)


def scenario_id(scenario: Scenario) -> str:
    """Stable id from kind, params, ego start, and the agent endpoints."""
    agents = []
    for agent in scenario.agents:
        agents.append(
            {
                "id": agent.agent_id,
                "n": int(np.asarray(agent.t).size),
                "x0": round(float(agent.x[0]), 4),
                "y0": round(float(agent.y[0]), 4),
                "x1": round(float(agent.x[-1]), 4),
                "y1": round(float(agent.y[-1]), 4),
            }
        )
    payload = {
        "kind": scenario.kind,
        "ego": [round(float(value), 5) for value in scenario.ego_start],
        "params": {key: _scalar(value) for key, value in scenario.params.items()},
        "agents": agents,
    }
    raw = json.dumps(payload, sort_keys=True, default=str)
    digest = hashlib.sha1(raw.encode()).hexdigest()[:10]
    return f"{scenario.kind}-{digest}"


def _subset(df: pd.DataFrame, system: str, threshold, latency) -> pd.DataFrame:
    sub = _normalize(df)
    sub = sub.loc[sub["system"] == system]
    if threshold is None:
        if sub["threshold"].nunique(dropna=False) != 1:
            raise ValueError("threshold is required when the matrix has more than one")
    else:
        sub = sub.loc[np.isclose(sub["threshold"].astype(float), float(threshold))]
    if latency is None:
        if sub["latency"].nunique(dropna=False) != 1:
            raise ValueError("latency is required when the matrix has more than one")
    else:
        sub = sub.loc[np.isclose(sub["latency"].astype(float), float(latency))]
    if sub.empty:
        raise ValueError(f"no rows for system={system!r}")
    return sub


def _normalize(df: pd.DataFrame) -> pd.DataFrame:
    out = df.copy()
    for column in ("benign", "collision", "false_brake"):
        if column in out.columns:
            out[column] = _as_bool(out[column])
    return out


def _as_bool(series: pd.Series) -> pd.Series:
    if series.dtype == bool:
        return series
    if pd.api.types.is_numeric_dtype(series):
        return series.fillna(0).astype(bool)

    def one(value):
        if isinstance(value, (bool, np.bool_)):
            return bool(value)
        if pd.isna(value):
            return False
        return str(value).strip().lower() in _TRUE

    return series.map(one).astype(bool)


def _param_keys(scenarios: list[Scenario]) -> list[str]:
    keys: list[str] = []
    for scenario in scenarios:
        for key, value in scenario.params.items():
            if _scalar(value) is not None and key not in keys:
                keys.append(key)
    preferred = [key for key in ("gap_d", "rel_v") if key in keys]
    return preferred + [key for key in keys if key not in preferred]


def _scalar(value):
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value)
    if isinstance(value, str):
        return value
    return None


def _git_rev() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"

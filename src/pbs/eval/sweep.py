"""M4 — experiment matrix. Owner: D.

One row per ``run_sim``. Trade-off curves, failure boundaries, and lead times are
groupbys of that table. The never-brake label is cached per scenario and uses the same
``sim_kwargs`` as the measured runs.
"""

import hashlib
import json
import subprocess

import numpy as np
import pandas as pd

from pbs.eval.counterfactual import BENIGN, DANGEROUS, LATE_CONTACT, never_brake_label
from pbs.eval.metrics import evaluate
from pbs.sim.policy import HysteresisPolicy, TTCThresholdPolicy
from pbs.sim.simulator import DEFAULT_A_BRAKE, run_sim
from pbs.systems import make_system
from pbs.types import Scenario

_TRUE = {"1", "true", "t", "yes"}


def _parse_threshold(entry, policy_kind: str, hysteresis_gap: float) -> tuple[float, float]:
    """Resolve one threshold specification into ``(t_on, t_off)``."""
    if isinstance(entry, tuple | list):
        if len(entry) != 2:
            raise ValueError(f"threshold pair must have length 2, got {entry!r}")
        t_on, t_off = float(entry[0]), float(entry[1])
        if policy_kind == "threshold" and not np.isclose(t_on, t_off):
            raise ValueError(
                "threshold policy requires t_on == t_off; use policy_kind='hysteresis'"
            )
        return t_on, t_off
    t_on = float(entry)
    if policy_kind == "hysteresis":
        return t_on, t_on + float(hysteresis_gap)
    if policy_kind == "threshold":
        return t_on, t_on
    raise ValueError(f"unknown policy_kind: {policy_kind!r}")


def _make_policy(policy_kind: str, t_on: float, t_off: float, a_brake: float):
    if policy_kind == "hysteresis":
        return HysteresisPolicy(t_on=t_on, t_off=t_off, a_brake=a_brake)
    if policy_kind == "threshold":
        return TTCThresholdPolicy(ttc_threshold=t_on, a_brake=a_brake)
    raise ValueError(f"unknown policy_kind: {policy_kind!r}")


def run_matrix(
    systems,
    scenarios: list[Scenario],
    thresholds,
    latencies=(0.0,),
    *,
    sim_kwargs=None,
    seed: int = 0,
    policy_kind: str = "threshold",
    hysteresis_gap: float = 1.0,
    a_brake: float = DEFAULT_A_BRAKE,
    decision_duration: float | None = None,
) -> pd.DataFrame:
    """Run ``systems × scenarios × thresholds × latencies``.

    Columns follow ``docs/d-baseline.md`` and ``docs/l2-baseline.md``: scenario id,
    kind, scalar scenario params, system, policy, threshold (= t_on), t_on, t_off,
    latency, geometry, horizon, label, benign, never_brake_contact_t, decision_duration,
    outcome_duration, collision, first_collision_t, false_brake, min_dist,
    min_dist_until_contact, brake_onset_t, seed, git_rev.

    ``decision_duration`` splits the run into two windows. Every simulation still runs
    for ``sim_kwargs["duration"]``, and ``collision`` covers all of it. ``false_brake``
    counts only a brake commanded before ``decision_duration``, and only in a scenario
    with no never-brake contact in the whole run. A scenario whose never-brake contact
    comes after ``decision_duration`` is ``late_contact``: ``benign`` is false, and
    ``tradeoff`` and ``rescue_rate`` leave it out of both denominators. Without
    ``decision_duration`` there is one window and no ``late_contact``.
    """
    if not scenarios:
        raise ValueError("scenarios is empty")
    if policy_kind not in ("threshold", "hysteresis"):
        raise ValueError(f"unknown policy_kind: {policy_kind!r}")
    sim_kwargs = dict(sim_kwargs or {})
    names = [str(name) for name in systems]
    threshold_pairs = [_parse_threshold(value, policy_kind, hysteresis_gap) for value in thresholds]
    latency_list = [float(value) for value in latencies]
    if not names or not threshold_pairs or not latency_list:
        raise ValueError("systems, thresholds, and latencies must be non-empty")
    geometry = "circle" if sim_kwargs.get("r_safe") is not None else "obb"
    horizon = int(sim_kwargs.get("horizon", 30))
    param_keys = _param_keys(scenarios)
    revision = _git_rev()
    decision_end = None if decision_duration is None else float(decision_duration)
    label_cache: dict[str, tuple[str, float | None]] = {}
    rows = []
    for scenario in scenarios:
        sid = scenario_id(scenario)
        if sid not in label_cache:
            label_cache[sid] = never_brake_label(scenario, decision_end, **sim_kwargs)
        label, never_brake_contact = label_cache[sid]
        benign = label == BENIGN
        params = {key: _scalar(scenario.params.get(key)) for key in param_keys}
        for name in names:
            assessor = make_system(name)
            for t_on, t_off in threshold_pairs:
                policy = _make_policy(policy_kind, t_on, t_off, float(a_brake))
                for latency in latency_list:
                    log = run_sim(scenario, assessor, policy, latency=latency, **sim_kwargs)
                    metrics = evaluate(log, benign, decision_end)
                    outcome_end = float(log.t[-1])
                    onset = metrics["brake_onset_t"]
                    contact = metrics["first_collision_t"]
                    rows.append(
                        {
                            "scenario_id": sid,
                            "kind": scenario.kind,
                            **params,
                            "system": name,
                            "policy": policy_kind,
                            "threshold": t_on,
                            "t_on": t_on,
                            "t_off": t_off,
                            "latency": latency,
                            "geometry": geometry,
                            "horizon": horizon,
                            "label": label,
                            "benign": benign,
                            "never_brake_contact_t": (
                                np.nan if never_brake_contact is None else never_brake_contact
                            ),
                            "decision_duration": (
                                outcome_end if decision_end is None else decision_end
                            ),
                            "outcome_duration": outcome_end,
                            "collision": bool(metrics["collision"]),
                            "first_collision_t": np.nan if contact is None else float(contact),
                            "false_brake": bool(metrics["false_brake"]),
                            "min_dist": float(metrics["min_dist"]),
                            "min_dist_until_contact": float(metrics["min_dist_until_contact"]),
                            "brake_onset_t": np.nan if onset is None else float(onset),
                            "seed": int(seed),
                            "git_rev": revision,
                        }
                    )
    return pd.DataFrame(rows)


def tradeoff(df: pd.DataFrame) -> pd.DataFrame:
    """Collision rate on dangerous rows and false-brake rate on benign rows.

    ``late_contact`` rows are in neither rate; ``n_late_contact`` counts them.
    """
    frame = _normalize(df)
    rows = []
    grouped = frame.groupby(["system", "threshold", "latency"], sort=False)
    for (system, threshold, latency), group in grouped:
        dangerous = group.loc[_is_dangerous(group)]
        benign = group.loc[group["benign"]]
        rows.append(
            {
                "system": system,
                "threshold": float(threshold),
                "latency": float(latency),
                "n_dangerous": int(len(dangerous)),
                "n_benign": int(len(benign)),
                "n_late_contact": int(len(group) - len(dangerous) - len(benign)),
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
        order = (
            ["collision_rate", "false_brake_rate", "threshold"]
            if not eligible.empty
            else ["false_brake_rate", "collision_rate", "threshold"]
        )
        pool = pool.sort_values(
            order,
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
    sub = sub.assign(dangerous=_is_dangerous(sub))
    cells = sub.groupby(["gap_d", "rel_v"], as_index=False).agg(
        collision=("collision", "max"), benign=("benign", "min"), dangerous=("dangerous", "max")
    )
    for column in ("collision", "benign", "dangerous"):
        cells[column] = cells[column].astype(bool)
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
    dangerous = cells.loc[cells["dangerous"]]
    if dangerous.empty:
        return float("nan")
    return float(1.0 - dangerous["collision"].mean())


def late_contact_outcomes(df: pd.DataFrame) -> pd.DataFrame:
    """What each system did in the ``late_contact`` scenarios, which no rate includes.

    ``braked_in_window`` counts brakes commanded before ``decision_duration``.
    """
    frame = _normalize(df)
    columns = [
        "system",
        "threshold",
        "latency",
        "n_late_contact",
        "braked_in_window",
        "braked_later",
        "collided",
    ]
    if "label" not in frame.columns:
        return pd.DataFrame(columns=columns)
    late = frame.loc[frame["label"] == LATE_CONTACT]
    rows = []
    for (system, threshold, latency), group in late.groupby(
        ["system", "threshold", "latency"], sort=False
    ):
        onset = group["brake_onset_t"].astype(float)
        in_window = onset < group["decision_duration"].astype(float) - 1e-9
        rows.append(
            {
                "system": system,
                "threshold": float(threshold),
                "latency": float(latency),
                "n_late_contact": int(len(group)),
                "braked_in_window": int(in_window.sum()),
                "braked_later": int((onset.notna() & ~in_window).sum()),
                "collided": int(group["collision"].sum()),
            }
        )
    return pd.DataFrame(rows, columns=columns)


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
    """Stable id of the full simulation input, including interior trajectory samples."""
    payload = {
        "kind": scenario.kind,
        "ego": [float(value) for value in scenario.ego_start],
        "dt": float(scenario.dt),
        "lane_width": float(scenario.lane_width),
        "params": {key: _scalar(value) for key, value in scenario.params.items()},
        "agents": [(a.agent_id, a.agent_type) for a in scenario.agents],
    }
    digest = hashlib.sha256(json.dumps(payload, sort_keys=True, default=str).encode())
    arrays = [scenario.ego_path]
    for agent in scenario.agents:
        arrays.extend(getattr(agent, key) for key in ("t", "x", "y", "v", "heading"))
    for values in arrays:
        array = np.ascontiguousarray(values, dtype="<f8")
        digest.update(str(array.shape).encode())
        digest.update(array.tobytes())
    return f"{scenario.kind}-{digest.hexdigest()[:20]}"


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


def _is_dangerous(frame: pd.DataFrame) -> pd.Series:
    """Tables written before the ``label`` column have two classes: dangerous = not benign."""
    if "label" in frame.columns:
        return frame["label"] == DANGEROUS
    return ~_as_bool(frame["benign"])


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
        if isinstance(value, bool | np.bool_):
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
    if isinstance(value, bool | np.bool_):
        return bool(value)
    if isinstance(value, int | np.integer):
        return int(value)
    if isinstance(value, float | np.floating):
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

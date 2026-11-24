"""Shared settings for D's experiment scripts."""

import json
from pathlib import Path

import numpy as np

from pbs.sim.collision import R_SAFE, VEHICLE_LENGTH
from pbs.systems import CORE

try:
    from scenario_sets import DECISION_DURATION, LOG_DURATION, OUTCOME_DURATION
except ModuleNotFoundError:
    from experiments.scenario_sets import DECISION_DURATION, LOG_DURATION, OUTCOME_DURATION

SIM_KWARGS = {"duration": OUTCOME_DURATION, "horizon": 40, "r_safe": None}
POLICY_KWARGS = {"policy_kind": "hysteresis", "hysteresis_gap": 1.0, "a_brake": 6.0}
MATRIX_KWARGS = {**POLICY_KWARGS, "decision_duration": DECISION_DURATION}
# The legacy mode reproduces PR #8: one 15 s window for decisions, labels and outcomes.
L1_SIM_KWARGS = {"duration": DECISION_DURATION, "horizon": 30, "r_safe": R_SAFE}
L1_POLICY_KWARGS = {"policy_kind": "threshold", "a_brake": 6.0}
QUICK_THRESHOLDS = (1.0, 2.0)
FULL_THRESHOLDS = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)
LEAD_SYSTEMS = ("reactive", "cv", "ctrv")
BOUNDARY_SYSTEMS = CORE


def experiment_config(legacy_l1: bool = False) -> tuple[dict, dict, dict]:
    """Return ``(sim_kwargs, run_matrix kwargs, metadata)`` for L2 baseline or L1 legacy mode."""
    if legacy_l1:
        sim = dict(L1_SIM_KWARGS)
        policy = dict(L1_POLICY_KWARGS)
        meta = {
            "baseline": "l1_legacy",
            "geometry": "circle",
            "r_safe": float(R_SAFE),
            "clearance_m": float(R_SAFE),
            "horizon": int(sim["horizon"]),
            "policy": "threshold",
            "hysteresis_gap": 0.0,
            "a_brake": float(policy["a_brake"]),
            "decision_duration_s": float(sim["duration"]),
            "outcome_duration_s": float(sim["duration"]),
            "log_duration_s": float(LOG_DURATION),
        }
        return sim, policy, meta
    sim = dict(SIM_KWARGS)
    policy = dict(MATRIX_KWARGS)
    meta = {
        "baseline": "l2",
        "geometry": "obb",
        "r_safe": None,
        "clearance_m": float(VEHICLE_LENGTH),
        "horizon": int(sim["horizon"]),
        "policy": "hysteresis",
        "hysteresis_gap": float(policy["hysteresis_gap"]),
        "a_brake": float(policy["a_brake"]),
        "decision_duration_s": float(policy["decision_duration"]),
        "outcome_duration_s": float(sim["duration"]),
        "log_duration_s": float(LOG_DURATION),
    }
    return sim, policy, meta


def write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")


def _jsonable(value):
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, list | tuple):
        return [_jsonable(item) for item in value]
    if isinstance(value, np.floating | float):
        number = float(value)
        return None if np.isnan(number) else number
    if isinstance(value, np.integer | int) and not isinstance(value, bool):
        return int(value)
    if isinstance(value, np.bool_ | bool):
        return bool(value)
    return value

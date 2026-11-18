"""Shared settings for D's experiment scripts."""

import json
from pathlib import Path

import numpy as np
from scenario_sets import SIM_DURATION

from pbs.systems import CORE

SIM_KWARGS = {"duration": SIM_DURATION, "horizon": 30, "r_safe": 2.5}
QUICK_THRESHOLDS = (1.0, 2.0)
FULL_THRESHOLDS = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)
LEAD_SYSTEMS = ("reactive", "cv", "ctrv")
BOUNDARY_SYSTEMS = CORE


def write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")


def _jsonable(value):
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return None if np.isnan(number) else number
    if isinstance(value, (np.integer, int)) and not isinstance(value, bool):
        return int(value)
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    return value

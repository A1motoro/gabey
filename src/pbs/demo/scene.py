"""Data adapters keep dataset conventions out of the simulator and renderer."""

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from pbs.data.scenarios import make_scenario
from pbs.types import Scenario, Trajectory

DEFAULTS = dict(gap_d=20.0, rel_v=5.0, cut_in_t=0.5, lc_T=3.0)
RANGES = dict(gap_d=(5.0, 60.0), rel_v=(-2.0, 8.0), cut_in_t=(0.5, 3.0), lc_T=(3.0, 5.0))
PRESETS = {
    "standard": {"label": "Standard cut-in", "params": DEFAULTS},
    "faster": {"label": "Faster cutter · benign", "params": {**DEFAULTS, "rel_v": -2.0}},
    "tight_slow": {
        "label": "5 m gap · slow merge",
        "params": {**DEFAULTS, "gap_d": 5, "rel_v": 1, "lc_T": 5},
    },
    "tight_fast": {
        "label": "5 m gap · faster merge",
        "params": {**DEFAULTS, "gap_d": 5, "rel_v": 1, "lc_T": 3},
    },
}


@dataclass
class DemoScene:
    """Normalized world coordinates [m], seconds, radians; ego remains simulated.

    Dataset adapters must normalize time/units/coordinates, validate IDs and
    missing samples, and record those transformations in source. Agent recordings
    must cover the full simulation. A measured track is reference, not truth.
    Road lines are optional world-coordinate polylines; absent lines are schematic.
    """

    scenario: Scenario
    source: dict = field(default_factory=dict)
    road_lines: list[np.ndarray] | None = None
    ego_reference: Trajectory | None = None


class SceneAdapter(Protocol):
    def load(self, config: dict) -> DemoScene: ...


def resolve_config(config: dict) -> dict:
    if not isinstance(config, dict):
        raise ValueError("Parameters must be a JSON object.")
    unknown = config.keys() - DEFAULTS.keys()
    if unknown:
        raise ValueError(f"Unknown parameter: {sorted(unknown)[0]}")
    result = DEFAULTS.copy()
    for key, value in config.items():
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise ValueError(f"{key} must be a number.")
        lo, hi = RANGES[key]
        if not np.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f"{key} must be between {lo:g} and {hi:g}.")
        result[key] = float(value)
    return result


class SyntheticCutInAdapter:
    def load(self, config: dict) -> DemoScene:
        params = resolve_config(config)
        scenario = make_scenario("cut_in", **params, duration=10.0, v_ego=15.0)
        length = scenario.ego_path[-1, 0]
        lines = [np.array([[-1000.0, y], [length + 1000.0, y]]) for y in (-1.75, 1.75, 5.25)]
        return DemoScene(
            scenario=scenario,
            source={
                "kind": "synthetic",
                "label": "Synthetic cut-in",
                "reference_kind": "synthetic truth",
                "road_kind": "generated straight road",
                "transforms": {"time_origin_s": 0, "units": "m, s, rad", "frame": "world +x"},
            },
            road_lines=lines,
        )

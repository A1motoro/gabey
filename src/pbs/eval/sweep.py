"""M4 — experiment matrix. Owner: D.

Matrix: systems (pbs.systems.CORE [+ ctrv, lr]) × scenario kind × param grid × ttc_threshold
[× latency]. One row per run; everything else is a groupby on this DataFrame.
"""

import pandas as pd

from pbs.types import Scenario


def run_matrix(
    systems, scenarios: list[Scenario], thresholds, latencies=(0.0,)
) -> pd.DataFrame:
    """Columns: system, kind, **params, threshold, latency, benign, collision, false_brake,
    min_dist, brake_onset_t. Cache is_benign per scenario (it is system-independent)."""
    raise NotImplementedError("TODO(D): W5–W6")


def tradeoff(df: pd.DataFrame) -> pd.DataFrame:
    """Per (system, threshold): collision rate (dangerous set) vs false-brake rate (benign set)."""
    raise NotImplementedError("TODO(D): W7")


def failure_boundary(df: pd.DataFrame, system: str) -> pd.DataFrame:
    """On the (gap_d, rel_v) grid: collision True/False per cell for one system."""
    raise NotImplementedError("TODO(D): W8–W10")


def safe_area(df: pd.DataFrame, system: str) -> float:
    """Fraction of grid cells that are collision-free → 'how much prediction expands the ODD'."""
    raise NotImplementedError("TODO(D): W8–W10")

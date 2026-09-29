"""M1 — parametric synthetic scenarios. Owner: A.

Physical plausibility first: lane changes follow a quintic (minimum-jerk) lateral profile
    y(τ) = y0 + Δy·(10τ³ − 15τ⁴ + 6τ⁵),  τ = (t − t0)/T ∈ [0, 1]
whose peak lateral acceleration is (10/√3)·|Δy|/T² ≈ 5.77·|Δy|/T².
Reject parameter combos with peak a_lat > MAX_LAT_ACC (report how many were rejected).

kind        params
cut_in      gap_d [m], rel_v [m/s] (ego − cutter; >0 = ego faster), cut_in_t [s], lc_T [s]
lead_brake  decel [m/s²], init_gap [m]
crossing    lateral_v [m/s], arrival_t [s]

Every scenario set MUST contain benign cases (dangerous-looking but collision-free under
NeverBrakePolicy, e.g. cut-in with rel_v ≤ 0 or large gap_d). Otherwise the false-brake
rate is identically 0 and the trade-off curve cannot be drawn.
"""

import numpy as np

from pbs.types import Scenario

KINDS = ("cut_in", "lead_brake", "crossing")
MAX_LAT_ACC = 3.0  # m/s²; TODO(A): cite a source for a comfortable/aggressive bound


def quintic_lateral(y0: float, y1: float, T: float, t: np.ndarray, t0: float = 0.0) -> np.ndarray:
    """Lateral position of a minimum-jerk lane change starting at t0 with duration T."""
    raise NotImplementedError("TODO(A): W3–W4")


def peak_lateral_acc(dy: float, T: float) -> float:
    return 10 / np.sqrt(3) * abs(dy) / T**2


def make_scenario(kind: str, **params) -> Scenario:
    """Agents must get unique agent_id (OraclePredictor relies on it)."""
    if kind not in KINDS:
        raise ValueError(f"unknown scenario kind {kind!r}; expected one of {KINDS}")
    raise NotImplementedError(f"TODO(A): cut_in W3–W4, others W5–W6 — {kind}")

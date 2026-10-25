"""M3 — footprint & ground-truth collision. Owner: C.

Vehicles are circles; two vehicles conflict when centre distance < r_safe.
r_safe is owned here and passed read-only to P via RiskContext.
"""

import numpy as np

R_SAFE = 2.5  # m; TODO(C): justify (vehicle half-length + half-width + margin?) in report


def in_collision(p: np.ndarray, q: np.ndarray, r_safe: float = R_SAFE) -> bool:
    """Ground-truth check on actual positions (not predictions)."""
    return bool(np.linalg.norm(np.asarray(p) - np.asarray(q)) < r_safe)


def swept_min_distance(p0, p1, q0, q1) -> float:
    """Minimum simultaneous distance for linear motion over one common interval.

    Exact for linear interpolation, approximate for accelerated/curved motion.
    The caller subdivides such motion before using this function.
    """
    relative = np.asarray(p0, dtype=float) - np.asarray(q0, dtype=float)
    delta = (np.asarray(p1) - p0) - (np.asarray(q1) - q0)
    squared = float(delta @ delta)
    fraction = 0.0 if squared == 0 else float(np.clip(-(relative @ delta) / squared, 0, 1))
    return float(np.linalg.norm(relative + fraction * delta))

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


def swept_contact_fraction(p0, p1, q0, q1, r_safe: float = R_SAFE) -> float | None:
    """Entry fraction for an interval that enters distance < r_safe; tangent is not collision.

    Returns the boundary contact time (infimum of the strictly overlapping times).
    Uses exactly the linear interpolation used by swept_min_distance.
    """
    if swept_min_distance(p0, p1, q0, q1) >= r_safe:
        return None
    relative = np.asarray(p0, dtype=float) - np.asarray(q0, dtype=float)
    delta = (np.asarray(p1) - p0) - (np.asarray(q1) - q0)
    c = float(relative @ relative) - r_safe**2
    if c <= 0:
        return 0.0
    a = float(delta @ delta)
    b = 2 * float(relative @ delta)
    return float(np.clip((-b - np.sqrt(max(0.0, b * b - 4 * a * c))) / (2 * a), 0, 1))

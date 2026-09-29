"""M3 — footprint & ground-truth collision. Owner: C.

Vehicles are circles; two vehicles conflict when centre distance < r_safe.
r_safe is owned here and passed read-only to P via RiskContext.
"""

import numpy as np

R_SAFE = 2.5  # m; TODO(C): justify (vehicle half-length + half-width + margin?) in report


def in_collision(p: np.ndarray, q: np.ndarray, r_safe: float = R_SAFE) -> bool:
    """Ground-truth check on actual positions (not predictions)."""
    return bool(np.linalg.norm(np.asarray(p) - np.asarray(q)) < r_safe)

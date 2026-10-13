"""Shared input checks and velocity estimation for the motion baselines."""

from numbers import Integral

import numpy as np

from pbs.types import Trajectory


def prediction_inputs(history: Trajectory, horizon: int, dt: float):
    if isinstance(horizon, bool) or not isinstance(horizon, Integral) or horizon < 0:
        raise ValueError("horizon must be a non-negative integer")
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and positive")
    arrays = [
        np.asarray(getattr(history, key), dtype=float) for key in ("t", "x", "y", "v", "heading")
    ]
    if any(a.ndim != 1 for a in arrays) or not arrays[0].size:
        raise ValueError("history must contain non-empty one-dimensional arrays")
    if any(a.shape != arrays[0].shape or not np.isfinite(a).all() for a in arrays):
        raise ValueError("history arrays must have equal lengths and finite values")
    t, x, y, _, _ = arrays
    if np.any(np.diff(t) <= 0):
        raise ValueError("history timestamps must be strictly increasing")
    return t, np.column_stack((x, y)), np.arange(1, horizon + 1, dtype=float) * dt


def last_velocity(history: Trajectory, t: np.ndarray, xy: np.ndarray) -> np.ndarray:
    """Use observed 2D displacement; one sample falls back to speed and heading."""
    if len(t) >= 2:
        return (xy[-1] - xy[-2]) / (t[-1] - t[-2])
    heading = history.heading[-1]
    return history.v[-1] * np.array([np.cos(heading), np.sin(heading)])

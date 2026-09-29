"""Shared helpers (geometry, interpolation)."""

import numpy as np

from pbs.types import Trajectory


def state_at(traj: Trajectory, t: float) -> np.ndarray:
    """Agent (x, y) at time t by linear interpolation; holds endpoints outside range."""
    return np.array([np.interp(t, traj.t, traj.x), np.interp(t, traj.t, traj.y)])


def history_until(traj: Trajectory, t: float) -> Trajectory:
    """Slice of traj with timestamps <= t (what is observable at time t)."""
    m = traj.t <= t + 1e-9
    if not m.any():  # agent not yet observed: expose first sample only
        m = np.zeros_like(traj.t, dtype=bool)
        m[0] = True
    return Trajectory(
        t=traj.t[m],
        x=traj.x[m],
        y=traj.y[m],
        v=traj.v[m],
        heading=traj.heading[m],
        agent_type=traj.agent_type,
        agent_id=traj.agent_id,
    )


def _cum_arclength(path: np.ndarray) -> np.ndarray:
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])


def path_point(path: np.ndarray, s: float) -> np.ndarray:
    """Point at arc length s along polyline path (M, 2); clamps to the ends."""
    cum = _cum_arclength(path)
    return np.array([np.interp(s, cum, path[:, 0]), np.interp(s, cum, path[:, 1])])


def path_heading(path: np.ndarray, s: float) -> float:
    """Heading [rad] of the path segment containing arc length s."""
    cum = _cum_arclength(path)
    i = int(np.clip(np.searchsorted(cum, s, side="right") - 1, 0, len(path) - 2))
    d = path[i + 1] - path[i]
    return float(np.arctan2(d[1], d[0]))

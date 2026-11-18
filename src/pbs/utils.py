"""Shared helpers (geometry, interpolation)."""

import numpy as np

from pbs.types import Trajectory


def state_at(traj: Trajectory, t: float) -> np.ndarray:
    """Agent (x, y) at time t by linear interpolation; holds endpoints outside range."""
    return np.array([np.interp(t, traj.t, traj.x), np.interp(t, traj.t, traj.y)])


def history_until(traj: Trajectory, t: float) -> Trajectory:
    """Slice of traj with timestamps <= t (what is observable at time t)."""
    m = traj.t <= t + 1e-9
    return Trajectory(
        t=traj.t[m],
        x=traj.x[m],
        y=traj.y[m],
        v=traj.v[m],
        heading=traj.heading[m],
        agent_type=traj.agent_type,
        agent_id=traj.agent_id,
        accel=None if traj.accel is None else traj.accel[m],
    )


def history_window(traj: Trajectory, t: float, window: float) -> Trajectory:
    """Slice of traj with timestamps in ``[t - window, t]`` — the samples a windowed
    predictor may see at time t.

    Different from :func:`history_until`, which keeps the whole prefix ``[0, t]``: the
    convention matters for model fitting, because a least-squares fit over the full prefix
    mixes in dynamics from long before the forecast (e.g. a cut-in that started at t = 0.5 s
    keeps biasing an acceleration estimate at t = 13 s).
    """
    m = (traj.t >= t - window - 1e-9) & (traj.t <= t + 1e-9)
    return Trajectory(
        t=traj.t[m],
        x=traj.x[m],
        y=traj.y[m],
        v=traj.v[m],
        heading=traj.heading[m],
        agent_type=traj.agent_type,
        agent_id=traj.agent_id,
        accel=None if traj.accel is None else traj.accel[m],
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

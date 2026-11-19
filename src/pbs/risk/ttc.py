"""Owner: P."""

import numpy as np

from pbs.sim.collision import (
    VEHICLE_LENGTH,
    VEHICLE_WIDTH,
    overlaps,
    swept_contact_fraction,
    swept_min_distance,
)
from pbs.types import RiskEstimate


def forecast_headings(points, origin, initial_heading):
    """Infer directions from predicted displacements, retaining heading at stops."""
    displacements = np.diff(points, axis=0, prepend=np.asarray(origin)[None, :])
    moving = np.linalg.norm(displacements, axis=1) > 1e-9
    directions = np.arctan2(displacements[:, 1], displacements[:, 0])
    if initial_heading is None:
        initial_heading = directions[moving][0] if np.any(moving) else 0.0
    angles = np.r_[initial_heading, directions]
    indices = np.maximum.accumulate(np.where(moving, np.arange(1, len(points) + 1), 0))
    return angles[indices]


def _segment_headings(
    future_heading, origin_heading, n_steps: int
) -> tuple[np.ndarray, np.ndarray]:
    """Return per-segment midpoint headings and step-end headings for swept checks."""
    arr = np.asarray(future_heading, dtype=float)
    if arr.ndim == 0:
        end = np.full(n_steps, float(arr))
        start = float(arr) if origin_heading is None else float(origin_heading)
    else:
        end = arr
        start = float(end[0]) if origin_heading is None else float(origin_heading)
    unwrapped = np.unwrap(np.r_[start, end])
    mid = 0.5 * (unwrapped[:-1] + unwrapped[1:])
    return mid, end


def assess_risk(
    ego_future: np.ndarray,
    agent_future: np.ndarray,
    dt: float,
    r_safe: float | None = None,
    *,
    ego_heading=None,
    agent_heading=None,
    pedestrian=False,
    ego_origin=None,
    agent_origin=None,
    ego_origin_heading: float | None = None,
    agent_origin_heading: float | None = None,
    swept: bool = False,
) -> RiskEstimate:
    """First forecast conflict with strict footprint overlap.

    An explicit ``r_safe`` uses legacy circles. When ``swept=True``, each step
    interval ``[k·dt, (k+1)·dt]`` is swept linearly using ``swept_contact_fraction``
    so fast crossings cannot tunnel between discrete 0.1 s samples; both
    ``ego_origin`` and ``agent_origin`` are required to sweep the first interval.
    """
    ego = np.asarray(ego_future, dtype=float)
    agent = np.asarray(agent_future, dtype=float)
    if ego.ndim != 2 or ego.shape[1] != 2 or agent.shape != ego.shape:
        raise ValueError("future arrays must have matching (H, 2) shapes")
    if not np.isfinite(ego).all() or not np.isfinite(agent).all():
        raise ValueError("future positions must be finite")
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and positive")
    if swept and (ego_origin is None or agent_origin is None):
        raise ValueError("swept=True requires both ego_origin and agent_origin")
    if not len(ego):
        if r_safe is not None and (not np.isfinite(r_safe) or r_safe < 0):
            raise ValueError("r_safe must be finite and non-negative")
        return RiskEstimate()

    if r_safe is not None:
        if not np.isfinite(r_safe) or r_safe < 0:
            raise ValueError("r_safe must be finite and non-negative")
        if not swept:
            mask = np.linalg.norm(ego - agent, axis=1) < r_safe
            conflicts = np.flatnonzero(mask)
            if not len(conflicts):
                return RiskEstimate()
            k = int(conflicts[0])
            return RiskEstimate(ttc=(k + 1) * dt, conflict_step=k)
        p_start = ego[0] if ego_origin is None else np.asarray(ego_origin, dtype=float)
        q_start = agent[0] if agent_origin is None else np.asarray(agent_origin, dtype=float)
        if p_start.shape != (2,) or q_start.shape != (2,) or not (
            np.isfinite(p_start).all() and np.isfinite(q_start).all()
        ):
            raise ValueError("origin positions must be finite (2,) arrays")
        ego_pts = np.vstack((p_start, ego))
        agent_pts = np.vstack((q_start, agent))
        for k in range(len(ego)):
            if (
                swept_min_distance(ego_pts[k], ego_pts[k + 1], agent_pts[k], agent_pts[k + 1])
                <= r_safe
            ):
                fraction = swept_contact_fraction(
                    ego_pts[k], ego_pts[k + 1], agent_pts[k], agent_pts[k + 1], r_safe
                )
                if fraction is not None:
                    return RiskEstimate(ttc=float((k + fraction) * dt), conflict_step=k)
        return RiskEstimate()

    p_start = ego[0] if ego_origin is None else np.asarray(ego_origin, dtype=float)
    q_start = agent[0] if agent_origin is None else np.asarray(agent_origin, dtype=float)
    if ego_heading is None:
        ego_heading = forecast_headings(ego, p_start, ego_origin_heading)
    if agent_heading is None:
        agent_heading = forecast_headings(agent, q_start, agent_origin_heading)
    for heading in (ego_heading, agent_heading):
        if np.shape(heading) not in ((), (len(ego),)) or not np.isfinite(heading).all():
            raise ValueError("headings must be finite scalars or one angle per forecast step")

    if not swept:
        mask = overlaps(agent - ego, ego_heading, agent_heading, pedestrian)
        conflicts = np.flatnonzero(mask)
        if not len(conflicts):
            return RiskEstimate()
        k = int(conflicts[0])
        return RiskEstimate(ttc=(k + 1) * dt, conflict_step=k)

    if p_start.shape != (2,) or q_start.shape != (2,) or not (
        np.isfinite(p_start).all() and np.isfinite(q_start).all()
    ):
        raise ValueError("origin positions must be finite (2,) arrays")
    for heading in (ego_origin_heading, agent_origin_heading):
        if heading is not None and not np.isfinite(heading):
            raise ValueError("origin headings must be finite scalars")

    ego_pts = np.vstack((p_start, ego))
    agent_pts = np.vstack((q_start, agent))
    ego_mid, ego_end = _segment_headings(ego_heading, ego_origin_heading, len(ego))
    agent_mid, agent_end = _segment_headings(agent_heading, agent_origin_heading, len(ego))
    broad_limit = float(np.hypot(VEHICLE_LENGTH, VEHICLE_WIDTH))
    for k in range(len(ego)):
        dist = swept_min_distance(ego_pts[k], ego_pts[k + 1], agent_pts[k], agent_pts[k + 1])
        if dist <= broad_limit:
            fraction = swept_contact_fraction(
                ego_pts[k],
                ego_pts[k + 1],
                agent_pts[k],
                agent_pts[k + 1],
                None,
                ego_heading=float(ego_mid[k]),
                agent_heading=float(agent_mid[k]),
                pedestrian=pedestrian,
            )
            if fraction is not None:
                return RiskEstimate(ttc=float((k + fraction) * dt), conflict_step=k)
            if bool(
                overlaps(
                    agent[k] - ego[k],
                    float(ego_end[k]),
                    float(agent_end[k]),
                    pedestrian,
                )
            ):
                return RiskEstimate(ttc=float((k + 1) * dt), conflict_step=k)
    return RiskEstimate()

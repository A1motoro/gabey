"""Owner: P."""

import numpy as np

from pbs.sim.collision import overlaps
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


def assess_risk(
    ego_future: np.ndarray,
    agent_future: np.ndarray,
    dt: float,
    r_safe: float | None = None,
    *,
    ego_heading=None,
    agent_heading=None,
    pedestrian=False,
) -> RiskEstimate:
    """First forecast step with strict footprint overlap. An explicit radius uses legacy circles."""
    ego = np.asarray(ego_future, dtype=float)
    agent = np.asarray(agent_future, dtype=float)
    if ego.ndim != 2 or ego.shape[1] != 2 or agent.shape != ego.shape:
        raise ValueError("future arrays must have matching (H, 2) shapes")
    if not np.isfinite(ego).all() or not np.isfinite(agent).all():
        raise ValueError("future positions must be finite")
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and positive")
    if r_safe is not None:
        if not np.isfinite(r_safe) or r_safe < 0:
            raise ValueError("r_safe must be finite and non-negative")
        mask = np.linalg.norm(ego - agent, axis=1) < r_safe
    elif len(ego):
        if ego_heading is None:
            ego_heading = forecast_headings(ego, ego[0], None)
        if agent_heading is None:
            agent_heading = forecast_headings(agent, agent[0], None)
        for heading in (ego_heading, agent_heading):
            if np.shape(heading) not in ((), (len(ego),)) or not np.isfinite(heading).all():
                raise ValueError("headings must be finite scalars or one angle per forecast step")
        mask = overlaps(agent - ego, ego_heading, agent_heading, pedestrian)
    else:
        return RiskEstimate()
    conflicts = np.flatnonzero(mask)
    if not len(conflicts):
        return RiskEstimate()
    k = int(conflicts[0])
    return RiskEstimate(ttc=(k + 1) * dt, conflict_step=k)

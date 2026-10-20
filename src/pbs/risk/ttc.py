"""Owner: P."""

import numpy as np

from pbs.types import RiskEstimate


def assess_risk(
    ego_future: np.ndarray, agent_future: np.ndarray, dt: float, r_safe: float
) -> RiskEstimate:
    """Pure function. First step k with ||ego_future[k] - agent_future[k]|| < r_safe
    → RiskEstimate(ttc=(k+1)*dt, conflict_step=k); none → RiskEstimate() (ttc=inf).
    """
    ego = np.asarray(ego_future, dtype=float)
    agent = np.asarray(agent_future, dtype=float)
    if ego.ndim != 2 or ego.shape[1] != 2 or agent.shape != ego.shape:
        raise ValueError("future arrays must have matching (H, 2) shapes")
    if not np.isfinite(ego).all() or not np.isfinite(agent).all():
        raise ValueError("future positions must be finite")
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and positive")
    if not np.isfinite(r_safe) or r_safe < 0:
        raise ValueError("r_safe must be finite and non-negative")
    conflicts = np.flatnonzero(np.linalg.norm(ego - agent, axis=1) < r_safe)
    if not len(conflicts):
        return RiskEstimate()
    k = int(conflicts[0])
    return RiskEstimate(ttc=(k + 1) * dt, conflict_step=k)

"""Owner: P."""

import numpy as np

from pbs.types import RiskEstimate


def assess_risk(
    ego_future: np.ndarray, agent_future: np.ndarray, dt: float, r_safe: float
) -> RiskEstimate:
    """Pure function. First step k with ||ego_future[k] - agent_future[k]|| < r_safe
    → RiskEstimate(ttc=(k+1)*dt, conflict_step=k); none → RiskEstimate() (ttc=inf).
    """
    raise NotImplementedError("TODO(P): W3–W4")

"""M4 — metrics. Owner: D (ADE/FDE shared with P)."""

import numpy as np

from pbs.types import SimLog


def ade(pred: np.ndarray, gt: np.ndarray) -> float:
    """Average displacement error over (H, 2) arrays."""
    return float(np.linalg.norm(pred - gt, axis=1).mean())


def fde(pred: np.ndarray, gt: np.ndarray) -> float:
    """Final displacement error."""
    return float(np.linalg.norm(pred[-1] - gt[-1]))


def evaluate(log: SimLog, benign: bool) -> dict:
    """benign: from pbs.eval.counterfactual.is_benign(scenario), computed once per scenario."""
    onset = next((ti for ti, a in zip(log.t, log.action, strict=True) if a == "brake"), None)
    braked = onset is not None
    return {
        "collision": log.collision,
        "min_dist": log.min_dist,
        "brake_onset_t": onset,
        "false_brake": benign and braked,
        "benign": benign,
    }

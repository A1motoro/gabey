"""M4 — metrics. Owner: D (ADE/FDE shared with P)."""

import numpy as np

from pbs.types import SimLog


def ade(pred: np.ndarray, gt: np.ndarray) -> float:
    """Average displacement error over (H, 2) arrays."""
    return float(np.linalg.norm(pred - gt, axis=1).mean())


def fde(pred: np.ndarray, gt: np.ndarray) -> float:
    """Final displacement error."""
    return float(np.linalg.norm(pred[-1] - gt[-1]))


def brake_activity(log: SimLog) -> dict:
    """Count BRAKE-command episodes, not BRAKE frames or physical pedal movements.

    Series at t[k] include the decision starting at t[k], but duration includes
    elapsed intervals only. The terminal display action is never executed.
    A final active episode has release_t=None: simulation end is not a release.
    Legacy logs without executed_steps treat every action as executed, matching evaluate.
    """
    end = log.executed_steps if log.executed_steps is not None else len(log.t)
    count, elapsed, start = 0, 0.0, None
    counts, durations, episodes = [], [], []
    dt = float(log.t[-1] - log.t[-2]) if len(log.t) > 1 else 0.0
    for k, t in enumerate(log.t):
        if k and k - 1 < end and log.action[k - 1] == "brake":
            elapsed += float(t - log.t[k - 1])
        if k < end:
            if log.action[k] == "brake" and start is None:
                start = float(t)
                count += 1
            elif log.action[k] != "brake" and start is not None:
                episodes.append(
                    {
                        "start_t": start,
                        "release_t": float(t),
                        "end_t": float(t),
                        "duration_s": float(t) - start,
                    }
                )
                start = None
        counts.append(count)
        durations.append(elapsed)
    if start is not None:
        finish = float(log.t[end]) if end < len(log.t) else float(log.t[-1]) + dt
        episodes.append(
            {"start_t": start, "release_t": None, "end_t": finish, "duration_s": finish - start}
        )
    return {
        "count": counts,
        "duration_s": durations,
        "episodes": episodes,
        "total_count": count,
        "total_duration_s": float(sum(e["duration_s"] for e in episodes)),
    }


def evaluate(log: SimLog, benign: bool) -> dict:
    """benign: from pbs.eval.counterfactual.is_benign(scenario), computed once per scenario."""
    end = log.executed_steps if log.executed_steps is not None else len(log.t)
    onset = next(
        (ti for ti, a in zip(log.t[:end], log.action[:end], strict=True) if a == "brake"),
        None,
    )
    braked = onset is not None
    return {
        "collision": log.collision,
        "min_dist": log.min_dist,
        "brake_onset_t": onset,
        "false_brake": benign and braked,
        "benign": benign,
    }

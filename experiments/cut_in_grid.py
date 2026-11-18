"""Analytic cut-in grid for D's experiments.

A's ``make_scenario`` is still unimplemented, so the matrix cannot call it.
These logs use the same minimum-jerk lane change as ``l1_fixtures.cut_in_fixture``
and cover the full simulation duration, so an agent is not frozen at the last sample.
``maneuver='abort'`` dips toward the ego lane and returns. It stays outside ``r_safe``
when ``rel_v <= 0``, but a constant-velocity extrapolation still predicts a conflict.
That is the benign-looking-dangerous set the false-brake rate needs.
"""

import numpy as np

from pbs.types import Scenario, Trajectory

QUICK_GAPS = (8.0, 14.0, 22.0, 32.0, 48.0)
QUICK_RELS = (-4.0, 0.0, 4.0, 8.0, 12.0)
FULL_GAPS = tuple(float(value) for value in np.linspace(6.0, 60.0, 15))
FULL_RELS = tuple(float(value) for value in np.linspace(-4.0, 12.0, 17))
DURATION = 15.0
DT = 0.1
EGO_V = 15.0
# The risk horizon is 3 s. The log must cover it, or Oracle holds the last sample
# and treats a still-moving cutter as a stopped obstacle.
LOG_PAD = 3.0


def cut_in_grid(quick: bool = False, maneuvers=("complete",)) -> list[Scenario]:
    gaps = QUICK_GAPS if quick else FULL_GAPS
    rels = QUICK_RELS if quick else FULL_RELS
    scenarios = []
    for maneuver in maneuvers:
        if maneuver == "abort":
            gap_values = (0.0, 1.5, 3.0) if quick else (0.0, 1.0, 2.0, 3.0)
            rel_values = (0.0, -2.0) if quick else (-3.0, -1.0, 0.0)
        elif maneuver == "complete":
            gap_values, rel_values = gaps, rels
        else:
            raise ValueError(f"unknown maneuver {maneuver!r}")
        for gap in gap_values:
            for rel in rel_values:
                scenarios.append(make_cut_in(gap, rel, maneuver=maneuver))
    return scenarios


def make_cut_in(
    gap_d: float,
    rel_v: float,
    *,
    duration: float = DURATION,
    dt: float = DT,
    ego_v: float = EGO_V,
    maneuver: str = "complete",
) -> Scenario:
    t = np.arange(int(round((duration + LOG_PAD) / dt)) + 1) * dt
    vx = ego_v - rel_v
    y, vy = _lateral(t, maneuver)
    agent = Trajectory(
        t,
        gap_d + vx * t,
        y,
        np.hypot(vx, vy),
        np.arctan2(vy, vx),
        agent_id="cutter",
    )
    return Scenario(
        ego_start=(0.0, 0.0, 0.0, ego_v),
        ego_path=np.array([[0.0, 0.0], [5000.0, 0.0]]),
        agents=[agent],
        dt=dt,
        params={"gap_d": float(gap_d), "rel_v": float(rel_v), "maneuver": maneuver},
        kind="cut_in",
        lane_width=3.5,
    )


def _lateral(t: np.ndarray, maneuver: str) -> tuple[np.ndarray, np.ndarray]:
    if maneuver == "complete":
        return _quintic_segment(t, 0.5, 3.5, 3.5, 0.0)
    if maneuver == "abort":
        # Stay at y >= 2.8 m, outside the 2.5 m footprint, then return to the adjacent lane.
        y_in, vy_in = _quintic_segment(t, 0.5, 2.0, 3.5, 2.8)
        y_out, vy_out = _quintic_segment(t, 2.0, 3.5, 2.8, 3.5)
        y = np.full_like(t, 3.5)
        vy = np.zeros_like(t)
        inward = (t >= 0.5) & (t <= 2.0)
        outward = (t > 2.0) & (t <= 3.5)
        y[inward], vy[inward] = y_in[inward], vy_in[inward]
        y[outward], vy[outward] = y_out[outward], vy_out[outward]
        return y, vy
    raise ValueError(f"unknown maneuver {maneuver!r}")


def _quintic_segment(t, t0, t1, y0, y1):
    """Minimum-jerk lateral move. Outside [t0, t1] the position is held and speed is zero."""
    u = np.clip((t - t0) / (t1 - t0), 0.0, 1.0)
    poly = 10 * u**3 - 15 * u**4 + 6 * u**5
    dpoly = 30 * u**2 - 60 * u**3 + 30 * u**4
    y = y0 + (y1 - y0) * poly
    vy = np.where((t >= t0) & (t <= t1), (y1 - y0) / (t1 - t0) * dpoly, 0.0)
    return y, vy

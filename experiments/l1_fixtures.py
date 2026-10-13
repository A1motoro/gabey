"""Deterministic P-side validation fixtures, independent of M1's scenario API.

These small analytic logs supply L1 evidence while A's make_scenario remains unfinished.
They do not implement load_toy, make_scenario, or the production scenario collection.
"""

import numpy as np

from pbs.types import Scenario, Trajectory


def straight_track(vx: float = 10.0, vy: float = 2.0) -> Trajectory:
    t = np.arange(81) * 0.1
    return Trajectory(
        t,
        20 + vx * t,
        3.5 + vy * t,
        np.full_like(t, np.hypot(vx, vy)),
        np.full_like(t, np.arctan2(vy, vx)),
        agent_id="straight",
    )


def cut_in_fixture(gap: float = 20.0, rel_v: float = 5.0) -> Scenario:
    """Straight ego (15 m/s); agent uses a 3 s minimum-jerk lateral log.

    Lane change starts at 0.5 s and ends at 3.5 s. Analytic peak lateral acceleration
    is 10/sqrt(3)*3.5/3**2 = 2.245 m/s². Parameters only vary longitudinal motion.
    """
    t = np.arange(81) * 0.1
    u = np.clip((t - 0.5) / 3.0, 0, 1)
    y = 3.5 * (1 - (10 * u**3 - 15 * u**4 + 6 * u**5))
    vy = -3.5 / 3.0 * (30 * u**2 - 60 * u**3 + 30 * u**4)
    vx = 15.0 - rel_v
    agent = Trajectory(t, gap + vx * t, y, np.hypot(vx, vy), np.arctan2(vy, vx), agent_id="cutter")
    return Scenario(
        ego_start=(0.0, 0.0, 0.0, 15.0),
        ego_path=np.array([[0.0, 0.0], [500.0, 0.0]]),
        agents=[agent],
        params={"gap_d": gap, "rel_v": rel_v, "fixture": "P L1 validation"},
    )

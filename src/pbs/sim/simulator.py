"""M3 — closed-loop simulation. Owner: C.

Per step (dt = 0.1 s): observe (with latency) → ego rollout → assess each agent → min TTC
→ decide → update. Receding horizon: every step re-assesses; nothing is cached.
Ego is simulated (reacts to braking); agents are log-replayed (do not react).
"""

import numpy as np

from pbs.risk.assessors import RiskAssessor
from pbs.sim.collision import R_SAFE, in_collision
from pbs.sim.policy import BRAKE
from pbs.types import RiskContext, Scenario, SimLog
from pbs.utils import history_until, path_heading, path_point, state_at

DEFAULT_A_BRAKE = 6.0  # m/s²


def ego_rollout(path: np.ndarray, s: float, v: float, horizon: int, dt: float) -> np.ndarray:
    """Ego future under 'keep current speed' along path. (horizon, 2)."""
    return np.array([path_point(path, s + v * dt * (k + 1)) for k in range(horizon)])


def run_sim(
    scenario: Scenario,
    assessor: RiskAssessor,
    policy,
    duration: float = 10.0,
    horizon: int = 30,
    r_safe: float = R_SAFE,
    latency: float = 0.0,
) -> SimLog:
    """latency τ [s]: at time t the assessor only sees agent history up to t - τ."""
    dt = scenario.dt
    n = int(round(duration / dt)) + 1
    t = np.arange(n) * dt
    a_brake = getattr(policy, "a_brake", DEFAULT_A_BRAKE)
    assessor.bind(scenario)

    ego_x, ego_y, ego_v, ego_a = (np.zeros(n) for _ in range(4))
    ttc = np.full(n, np.inf)
    action: list[str] = []

    s, v = 0.0, float(scenario.ego_start[3])  # arc length along ego_path, speed
    min_dist, collision = np.inf, False

    for k in range(n):
        ego_xy = path_point(scenario.ego_path, s)
        ego_x[k], ego_y[k], ego_v[k] = ego_xy[0], ego_xy[1], v

        ctx = RiskContext(
            t_now=t[k],
            ego_xy=ego_xy,
            ego_heading=path_heading(scenario.ego_path, s),
            ego_v=v,
            ego_future=ego_rollout(scenario.ego_path, s, v, horizon, dt),
            ego_path=scenario.ego_path,
            lane_width=scenario.lane_width,
            horizon=horizon,
            dt=dt,
            r_safe=r_safe,
        )
        for agent in scenario.agents:
            true_xy = state_at(agent, t[k])
            min_dist = min(min_dist, float(np.linalg.norm(ego_xy - true_xy)))
            collision |= in_collision(ego_xy, true_xy, r_safe)
            est = assessor.assess(ctx, history_until(agent, t[k] - latency))
            if est.considered:
                ttc[k] = min(ttc[k], est.ttc)

        act = policy.decide(ttc[k])
        action.append(act)
        a = -a_brake if act == BRAKE and v > 0 else 0.0
        ego_a[k] = a
        v = max(0.0, v + a * dt)
        s += v * dt

    return SimLog(t, ego_x, ego_y, ego_v, ego_a, action, ttc, float(min_dist), bool(collision))

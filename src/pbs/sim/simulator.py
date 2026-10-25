"""M3 — closed-loop simulation. Owner: C.

Per step (dt = 0.1 s): observe (with latency) → ego rollout → assess each agent → min TTC
→ decide → update. Receding horizon: every step re-assesses; nothing is cached.
Ego is simulated (reacts to braking); agents are log-replayed (do not react).
"""

import numpy as np

from pbs.risk.assessors import RiskAssessor
from pbs.sim.collision import R_SAFE, in_collision, swept_min_distance
from pbs.sim.policy import BRAKE, CRUISE
from pbs.types import RiskContext, Scenario, SimLog
from pbs.utils import history_until, path_heading, path_point, state_at

DEFAULT_A_BRAKE = 6.0  # m/s²


class MissingObservationError(ValueError):
    """Invalid experiment input, not evidence that the road is empty."""


def advance_motion(v: float, a_brake: float, dt: float, action: str) -> tuple[float, float]:
    """Return travelled arc length and next speed, including stopping within dt."""
    if action == CRUISE:
        return v * dt, v
    h = min(dt, v / a_brake)
    return v * h - 0.5 * a_brake * h * h, max(0.0, v - a_brake * h)


def _validate(scenario, duration, horizon, r_safe, latency, collision_dt, a_brake):
    """Reject inputs that would otherwise silently clamp paths or replay endpoints."""
    dt = scenario.dt
    values = (dt, duration, r_safe, latency, collision_dt, a_brake, scenario.lane_width)
    if not np.all(np.isfinite(values)):
        raise ValueError("Simulation parameters must be finite.")
    if min(dt, r_safe, collision_dt, a_brake, scenario.lane_width) <= 0:
        raise ValueError("dt, r_safe, collision_dt, a_brake and lane_width must be positive.")
    if duration < 0 or latency < 0:
        raise ValueError("duration and latency must be nonnegative.")
    if not isinstance(horizon, (int, np.integer)) or isinstance(horizon, bool) or horizon < 1:
        raise ValueError("horizon must be a positive integer number of steps.")
    steps = int(round(duration / dt))
    if not np.isclose(steps * dt, duration, rtol=0, atol=1e-9):
        raise ValueError("duration must be an integer multiple of scenario.dt.")
    path = np.asarray(scenario.ego_path, dtype=float)
    start = np.asarray(scenario.ego_start, dtype=float)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2:
        raise ValueError("ego_path must have shape (M, 2), M >= 2.")
    if start.shape != (4,) or not np.all(np.isfinite(start)) or start[3] < 0:
        raise ValueError("ego_start must be finite (x, y, heading, nonnegative speed).")
    if not np.all(np.isfinite(path)) or np.any(np.linalg.norm(np.diff(path, axis=0), axis=1) == 0):
        raise ValueError("ego_path must be finite and have no zero-length segments.")
    if not np.allclose(start[:2], path[0], rtol=0, atol=1e-8):
        raise ValueError("ego_start position must match ego_path[0].")
    heading_error = np.arctan2(
        np.sin(start[2] - path_heading(path, 0)), np.cos(start[2] - path_heading(path, 0))
    )
    if abs(heading_error) > 1e-6:
        raise ValueError("ego_start heading must match the first path segment.")
    ids = set()
    for agent in scenario.agents:
        arrays = [np.asarray(getattr(agent, key)) for key in ("t", "x", "y", "v", "heading")]
        if any(a.ndim != 1 or not len(a) or not np.all(np.isfinite(a)) for a in arrays):
            raise ValueError(f"Agent {agent.agent_id!r}: require nonempty finite 1-D arrays.")
        if len({len(a) for a in arrays}) != 1 or np.any(np.diff(agent.t) <= 0):
            raise ValueError(
                f"Agent {agent.agent_id!r}: lengths must match and time must increase."
            )
        if not agent.agent_id or agent.agent_id in ids:
            raise ValueError("Each agent must have a unique nonempty agent_id.")
        ids.add(agent.agent_id)
        if agent.t[0] > -latency + 1e-9:
            raise MissingObservationError(
                f"Agent {agent.agent_id!r}: provide history at or before {-latency:g} s."
            )
        if agent.t[-1] < duration - 1e-9:
            raise ValueError(f"Agent {agent.agent_id!r}: truth must cover duration={duration}.")
    return steps, path


def ego_rollout(path: np.ndarray, s: float, v: float, horizon: int, dt: float) -> np.ndarray:
    """Ego future under 'keep current speed' along path. (horizon, 2)."""
    length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    if s + v * horizon * dt > length + 1e-8:
        raise ValueError("ego_path is too short for the requested ego prediction horizon.")
    return np.array([path_point(path, s + v * dt * (k + 1)) for k in range(horizon)])


def run_sim(
    scenario: Scenario,
    assessor: RiskAssessor,
    policy,
    duration: float = 10.0,
    horizon: int = 30,
    r_safe: float = R_SAFE,
    latency: float = 0.0,
    collision_dt: float = 0.01,
) -> SimLog:
    """Run one independent experiment using A's Scenario and P's RiskAssessor.

    Missing prehistory raises MissingObservationError; it is never called safe.
    The assessor must align delayed observations to ctx.t_now + (k+1)*dt.
    State rows include the terminal time. Only action[:executed_steps] is applied.
    Collision sweeps use <= collision_dt linear subintervals (an approximation
    for acceleration), also split at stopping times, path corners and truth knots.
    A collision is recorded permanently; this model does not simulate impacts.
    """
    dt = scenario.dt
    a_brake = getattr(policy, "a_brake", DEFAULT_A_BRAKE)
    steps, path = _validate(scenario, duration, horizon, r_safe, latency, collision_dt, a_brake)
    n = steps + 1
    t = np.arange(n) * dt
    reset = getattr(policy, "reset", None)
    if callable(reset):
        reset()
    assessor.bind(scenario)
    path_distances = np.concatenate(
        [[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    )

    ego_x, ego_y, ego_v, ego_a = (np.zeros(n) for _ in range(4))
    ttc = np.full(n, np.inf)
    action: list[str] = []

    s, v = 0.0, float(scenario.ego_start[3])  # arc length along ego_path, speed
    min_dist, collision = np.inf, False

    for k in range(n):
        ego_xy = path_point(path, s)
        ego_x[k], ego_y[k], ego_v[k] = ego_xy[0], ego_xy[1], v

        ctx = RiskContext(
            t_now=t[k],
            ego_xy=ego_xy,
            ego_heading=path_heading(path, s),
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
            history = history_until(agent, t[k] - latency)
            if not len(history.t):
                raise MissingObservationError(f"No observable history for {agent.agent_id!r}.")
            est = assessor.assess(ctx, history)
            if est.considered:
                if np.isnan(est.ttc) or est.ttc < 0:
                    raise ValueError(f"Invalid TTC from assessor for {agent.agent_id!r}.")
                ttc[k] = min(ttc[k], est.ttc)

        if k == steps:
            # Keep equal-length columns without asking policy to execute an extra action.
            action.append(action[-1] if action else CRUISE)
            break
        act = policy.decide(ttc[k])
        if act not in (BRAKE, CRUISE):
            raise ValueError(f"Unknown policy action: {act!r}.")
        action.append(act)
        a = -a_brake if act == BRAKE and v > 0 else 0.0
        ego_a[k] = a
        ds, next_v = advance_motion(v, a_brake, dt, act)
        if s + ds > path_distances[-1] + 1e-8:
            raise ValueError("ego_path ends before the next simulated position.")

        # Sweep true trajectories, not the predictor's output.
        offsets = list(np.linspace(0, dt, int(np.ceil(dt / collision_dt)) + 1))
        if a < 0 and 0 < v / a_brake < dt:
            offsets.append(v / a_brake)
        for corner in path_distances[1:-1]:
            distance = corner - s
            if 0 < distance < ds:
                if a < 0:
                    offsets.append(
                        2 * distance / (v + np.sqrt(max(0, v * v - 2 * a_brake * distance)))
                    )
                else:
                    offsets.append(distance / v)
        for agent in scenario.agents:
            knots = agent.t[(agent.t > t[k]) & (agent.t < t[k + 1])] - t[k]
            times = np.unique(np.concatenate([offsets, knots]))
            positions = [path_point(path, s + advance_motion(v, a_brake, h, act)[0]) for h in times]
            truth = [state_at(agent, t[k] + h) for h in times]
            for j in range(len(times) - 1):
                distance = swept_min_distance(
                    positions[j], positions[j + 1], truth[j], truth[j + 1]
                )
                min_dist = min(min_dist, distance)
                collision |= distance < r_safe
        s += ds
        v = next_v

    return SimLog(
        t,
        ego_x,
        ego_y,
        ego_v,
        ego_a,
        action,
        ttc,
        float(min_dist),
        bool(collision),
        executed_steps=steps,
    )

"""Three tiers of AEB risk assessment. Owner: P.

  Reactive   — no prediction. Only agents ALREADY inside the ego lane; longitudinal
               TTC = range / range_rate (line-of-sight constant velocity). Lateral motion
               is ignored ON PURPOSE.
  Predictive — predictor extrapolates every agent in 2D (incl. lateral) → assess_risk.
  Oracle     — Predictive with OraclePredictor (true future).

Why Reactive ≠ CV (must hold, see docs/spec.md §1.4): Reactive only sees a cutter once it
is in-lane; CV sees it while it is still changing lanes. The difference = lead time.
"""

from abc import ABC, abstractmethod

import numpy as np

from pbs.prediction._motion import last_velocity, prediction_inputs
from pbs.prediction.base import Predictor
from pbs.types import RiskContext, RiskEstimate, Scenario, Trajectory


class RiskAssessor(ABC):
    name = "base"

    def bind(self, scenario: Scenario) -> None:
        """Called once by run_sim before the loop."""
        return None

    @abstractmethod
    def assess(self, ctx: RiskContext, history: Trajectory) -> RiskEstimate:
        """history = what is observable at ctx.t_now (after latency)."""


class NullAssessor(RiskAssessor):
    """Walking-skeleton placeholder: never sees a conflict."""

    name = "null"

    def assess(self, ctx: RiskContext, history: Trajectory) -> RiskEstimate:
        return RiskEstimate()


class ReactiveAssessor(RiskAssessor):
    name = "reactive"

    def assess(self, ctx: RiskContext, history: Trajectory) -> RiskEstimate:
        t, xy, _ = prediction_inputs(history, 0, ctx.dt)
        if not np.isfinite(ctx.lane_width) or ctx.lane_width <= 0:
            raise ValueError("lane_width must be finite and positive")
        if not np.isfinite(ctx.r_safe) or ctx.r_safe < 0:
            raise ValueError("r_safe must be finite and non-negative")
        agent_s, offset, tangent = _path_projection(ctx.ego_path, xy[-1])
        if offset > ctx.lane_width / 2:
            return RiskEstimate(considered=False)
        ego_s, _, _ = _path_projection(ctx.ego_path, ctx.ego_xy)
        gap = agent_s - ego_s
        if gap < 0:
            return RiskEstimate()
        remaining = gap - ctx.r_safe
        if remaining < 0:
            return RiskEstimate(ttc=0.0)
        closing = ctx.ego_v - float(last_velocity(history, t, xy) @ tangent)
        if closing <= 0:
            return RiskEstimate()
        # Continuous longitudinal TTC; conflict_step applies only to sampled predictions.
        return RiskEstimate(ttc=float(remaining / closing))


def _path_projection(path: np.ndarray, point: np.ndarray):
    """Arc length, lane offset and tangent at the nearest polyline projection.

    Extend end segments so vehicles beyond path endpoints are not mistaken for lateral traffic.
    """
    path = np.asarray(path, dtype=float)
    point = np.asarray(point, dtype=float)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2:
        raise ValueError("ego_path must contain at least two 2D points")
    if point.shape != (2,) or not np.isfinite(path).all() or not np.isfinite(point).all():
        raise ValueError("path and point must be finite")
    vectors = np.diff(path, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    valid = lengths > 0
    if not valid.any():
        raise ValueError("ego_path must contain a non-zero length segment")
    starts = path[:-1][valid]
    arc_starts = np.concatenate(([0.0], np.cumsum(lengths)))[:-1][valid]
    vectors, lengths = vectors[valid], lengths[valid]
    raw = np.sum((point - starts) * vectors, axis=1) / lengths**2
    fractions = np.clip(raw, 0, 1)
    if len(fractions) == 1:
        fractions = raw
    else:
        fractions[0] = min(raw[0], 1.0)
        fractions[-1] = max(raw[-1], 0.0)
    projected = starts + fractions[:, None] * vectors
    distances = np.linalg.norm(projected - point, axis=1)
    i = int(np.argmin(distances))
    return (
        float(arc_starts[i] + fractions[i] * lengths[i]),
        float(distances[i]),
        vectors[i] / lengths[i],
    )


class PredictiveAssessor(RiskAssessor):
    def __init__(self, predictor: Predictor):
        self.predictor = predictor
        self.name = predictor.name

    def bind(self, scenario: Scenario) -> None:
        self.predictor.bind(scenario)

    def assess(self, ctx: RiskContext, history: Trajectory) -> RiskEstimate:
        from pbs.risk.ttc import assess_risk

        # Predictors are anchored to observation time; align stale observations to t_now.
        prediction_inputs(history, ctx.horizon, ctx.dt)
        age = ctx.t_now - history.t[-1]
        if not np.isfinite(age) or age < -1e-9:
            raise ValueError("history cannot be newer than the risk context")
        age = max(0.0, float(age))
        extra = int(np.ceil(age / ctx.dt))
        forecast = self.predictor.predict(history, ctx.horizon + extra, ctx.dt)
        if forecast.shape != (ctx.horizon + extra, 2):
            raise ValueError("predictor must return (horizon, 2) positions")
        if extra:
            times = np.arange(len(forecast) + 1) * ctx.dt
            xy = np.vstack(([history.x[-1], history.y[-1]], forecast))
            query = age + np.arange(1, ctx.horizon + 1) * ctx.dt
            agent_future = np.column_stack(
                (np.interp(query, times, xy[:, 0]), np.interp(query, times, xy[:, 1]))
            )
        else:
            agent_future = forecast
        return assess_risk(ctx.ego_future, agent_future, ctx.dt, ctx.r_safe)

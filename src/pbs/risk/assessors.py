"""Three tiers of AEB risk assessment. Owner: P.

  Reactive   — no prediction. Only agents ALREADY inside the ego lane; longitudinal
               TTC = range / range_rate (line-of-sight constant velocity). Lateral motion
               is ignored ON PURPOSE.
  Predictive — predictor extrapolates every agent in 2D (incl. lateral) → assess_risk.
               Optional ``uncertainty_scale`` (κ > 0) inflates the predicted target
               footprint by ``2·κ·σ`` (capped at 1.5 m per side) using ``latest_std``.
  Oracle     — Predictive with OraclePredictor (true future).

Why Reactive ≠ CV (must hold, see docs/spec.md §1.4): Reactive only sees a cutter once it
is in-lane; CV sees it while it is still changing lanes. The difference = lead time.
"""

from abc import ABC, abstractmethod

import numpy as np

from pbs.prediction._motion import last_velocity, prediction_inputs
from pbs.prediction.base import Predictor
from pbs.sim.collision import PEDESTRIAN_RADIUS, VEHICLE_LENGTH, VEHICLE_WIDTH
from pbs.types import RiskContext, RiskEstimate, Scenario, Trajectory

MAX_UNCERTAINTY_SIDE_PAD = 1.5  # m per side (L3 spec P-T3)


def _is_bool(value) -> bool:
    return isinstance(value, (bool, np.bool_))


def _ctx_dimensions(ctx: RiskContext) -> tuple[float, float]:
    raw_l = getattr(ctx, "vehicle_length", VEHICLE_LENGTH)
    raw_w = getattr(ctx, "vehicle_width", VEHICLE_WIDTH)
    if _is_bool(raw_l) or _is_bool(raw_w):
        raise ValueError("Vehicle dimensions must be finite and positive")
    try:
        length = float(raw_l)
        width = float(raw_w)
    except (TypeError, ValueError) as exc:
        raise ValueError("Vehicle dimensions must be finite and positive") from exc
    if not (np.isfinite(length) and np.isfinite(width)) or length <= 0.0 or width <= 0.0:
        raise ValueError("Vehicle dimensions must be finite and positive")
    return length, width


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
        if ctx.r_safe is not None and (not np.isfinite(ctx.r_safe) or ctx.r_safe < 0):
            raise ValueError("r_safe must be finite and non-negative")
        vehicle_length, vehicle_width = _ctx_dimensions(ctx)
        agent_s, offset, tangent = _path_projection(ctx.ego_path, xy[-1])
        if offset > ctx.lane_width / 2:
            return RiskEstimate(considered=False)
        ego_s, _, _ = _path_projection(ctx.ego_path, ctx.ego_xy)
        gap = agent_s - ego_s
        if gap < 0:
            return RiskEstimate()
        extent = ctx.r_safe
        if extent is None:
            # Reactive still ignores lateral motion once the agent is in lane.
            relative_heading = history.heading[-1] - np.arctan2(tangent[1], tangent[0])
            agent_extent = (
                PEDESTRIAN_RADIUS
                if history.agent_type == "pedestrian"
                else (vehicle_length / 2.0) * abs(np.cos(relative_heading))
                + (vehicle_width / 2.0) * abs(np.sin(relative_heading))
            )
            extent = (vehicle_length / 2.0) + agent_extent
        remaining = gap - extent
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
    def __init__(
        self,
        predictor: Predictor,
        *,
        uncertainty_scale: float = 0.0,
        name: str | None = None,
    ):
        if _is_bool(uncertainty_scale):
            raise ValueError("uncertainty_scale must be finite and non-negative")
        try:
            scale = float(uncertainty_scale)
        except (TypeError, ValueError) as exc:
            raise ValueError("uncertainty_scale must be finite and non-negative") from exc
        if not np.isfinite(scale) or scale < 0.0:
            raise ValueError("uncertainty_scale must be finite and non-negative")
        self.predictor = predictor
        self.uncertainty_scale = scale
        self.name = predictor.name if name is None else str(name)

    def bind(self, scenario: Scenario) -> None:
        self.predictor.bind(scenario)

    def _uncertainty_pads(
        self,
        ctx: RiskContext,
        extra: int,
        times: np.ndarray | None,
        query: np.ndarray | None,
    ) -> tuple[np.ndarray | float, np.ndarray | float]:
        if self.uncertainty_scale <= 0.0 or ctx.horizon == 0:
            return 0.0, 0.0
        raw_std = getattr(self.predictor, "latest_std", None)
        if raw_std is None:
            return 0.0, 0.0
        std = np.asarray(raw_std, dtype=float)
        if std.shape != (ctx.horizon + extra, 2):
            raise ValueError("predictor.latest_std must have shape (horizon, 2)")
        if not np.isfinite(std).all() or np.any(std < 0.0):
            raise ValueError("predictor.latest_std must be finite and non-negative")
        if extra:
            assert times is not None and query is not None
            std_padded = np.vstack((std[0:1], std))
            aligned_std = np.column_stack(
                (
                    np.interp(query, times, std_padded[:, 0]),
                    np.interp(query, times, std_padded[:, 1]),
                )
            )
        else:
            aligned_std = std
        side_lon = np.minimum(self.uncertainty_scale * aligned_std[:, 0], MAX_UNCERTAINTY_SIDE_PAD)
        side_lat = np.minimum(self.uncertainty_scale * aligned_std[:, 1], MAX_UNCERTAINTY_SIDE_PAD)
        return 2.0 * side_lon, 2.0 * side_lat

    def assess(self, ctx: RiskContext, history: Trajectory) -> RiskEstimate:
        from pbs.risk.ttc import assess_risk, forecast_headings

        # Predictors are anchored to observation time; align stale observations to t_now.
        prediction_inputs(history, ctx.horizon, ctx.dt)
        vehicle_length, vehicle_width = _ctx_dimensions(ctx)
        age = ctx.t_now - history.t[-1]
        if not np.isfinite(age) or age < -1e-9:
            raise ValueError("history cannot be newer than the risk context")
        age = max(0.0, float(age))
        extra = int(np.ceil(age / ctx.dt))
        forecast = self.predictor.predict(history, ctx.horizon + extra, ctx.dt)
        if forecast.shape != (ctx.horizon + extra, 2):
            raise ValueError("predictor must return (horizon, 2) positions")

        times = None
        query = None
        if extra:
            times = np.arange(len(forecast) + 1) * ctx.dt
            xy = np.vstack(([history.x[-1], history.y[-1]], forecast))
            query = age + np.arange(1, ctx.horizon + 1) * ctx.dt
            agent_future = np.column_stack(
                (np.interp(query, times, xy[:, 0]), np.interp(query, times, xy[:, 1]))
            )
        else:
            agent_future = forecast

        pad_l, pad_w = self._uncertainty_pads(ctx, extra, times, query)

        if ctx.r_safe is not None:
            return assess_risk(
                ctx.ego_future,
                agent_future,
                ctx.dt,
                ctx.r_safe,
                vehicle_length=vehicle_length,
                vehicle_width=vehicle_width,
                agent_length_pad=pad_l,
                agent_width_pad=pad_w,
            )
        angles = forecast_headings(forecast, [history.x[-1], history.y[-1]], history.heading[-1])
        if extra:
            unwrapped = np.unwrap(np.r_[history.heading[-1], angles])
            agent_now = np.array([np.interp(age, times, xy[:, 0]), np.interp(age, times, xy[:, 1])])
            agent_now_heading = float(np.interp(age, times, unwrapped))
            angles = np.interp(query, times, unwrapped)
        else:
            agent_now = np.array([history.x[-1], history.y[-1]], dtype=float)
            agent_now_heading = float(history.heading[-1])
        return assess_risk(
            ctx.ego_future,
            agent_future,
            ctx.dt,
            ego_heading=ctx.ego_future_heading,
            agent_heading=angles,
            pedestrian=history.agent_type == "pedestrian",
            ego_origin=ctx.ego_xy,
            agent_origin=agent_now,
            ego_origin_heading=ctx.ego_heading,
            agent_origin_heading=agent_now_heading,
            swept=True,
            vehicle_length=vehicle_length,
            vehicle_width=vehicle_width,
            agent_length_pad=pad_l,
            agent_width_pad=pad_w,
        )

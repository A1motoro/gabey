"""Observation boundary: transforms may change measurements, never reference tracks."""

from typing import Protocol

import numpy as np

from pbs.types import Trajectory


class ObservationModel(Protocol):
    """Return a nonempty, causal trajectory with the same agent identity.

    Input arrays are independent of scenario truth. Implementations must preserve
    timestamps and cache stochastic errors per (agent_id, timestamp), so rereading
    a sample does not redraw its noise. Construct a fresh model per simulation.
    """

    def observe(self, history: Trajectory, *, t_now: float) -> Trajectory: ...


class IdentityObservation:
    """Explicit no-noise adapter; run_sim already supplies isolated history arrays."""

    def observe(self, history: Trajectory, *, t_now: float) -> Trajectory:
        return history


class GaussianObservationModel:
    """Seeded measurement errors shared by repeated reads of the same sample."""

    def __init__(self, *, pos_std=0.15, heading_std=0.03, speed_std=0.25, seed=0):
        self._scale = np.array([pos_std, pos_std, speed_std, heading_std], dtype=float)
        if not np.all(np.isfinite(self._scale)) or np.any(self._scale < 0):
            raise ValueError("Noise standard deviations must be finite and nonnegative.")
        self._rng = np.random.default_rng(seed)
        self._errors: dict[tuple[str, float], np.ndarray] = {}

    def observe(self, history: Trajectory, *, t_now: float) -> Trajectory:
        if not len(history.t) or np.any(history.t > t_now + 1e-9):
            raise ValueError("Expected a nonempty history at or before t_now.")
        errors = []
        for timestamp in history.t:
            key = history.agent_id, round(float(timestamp), 6)
            if key not in self._errors:
                self._errors[key] = self._rng.normal(scale=self._scale)
            errors.append(self._errors[key])
        dx, dy, dv, dheading = np.asarray(errors).T
        return Trajectory(
            t=history.t.copy(),
            x=history.x + dx,
            y=history.y + dy,
            v=np.maximum(0, history.v + dv),
            heading=history.heading + dheading,
            agent_type=history.agent_type,
            agent_id=history.agent_id,
            accel=None,
        )

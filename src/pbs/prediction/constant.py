"""M2 — CV / CA motion models. Owner: P.

NB: CV extrapolates the full 2D velocity (including lateral). That lateral term is what
separates it from the Reactive baseline — see docs/spec.md §1.4.
"""

import numpy as np

from pbs.prediction._motion import last_velocity, prediction_inputs
from pbs.prediction.base import Predictor
from pbs.types import Trajectory


class ConstantVelocityPredictor(Predictor):
    name = "cv"

    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        t, xy, tau = prediction_inputs(history, horizon, dt)
        return xy[-1] + tau[:, None] * last_velocity(history, t, xy)


class ConstantAccelPredictor(Predictor):
    """Extrapolate the last observed acceleration. Two or fewer samples fall back to CV."""

    name = "ca"

    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        t, xy, tau = prediction_inputs(history, horizon, dt)
        if len(tau) == 0:
            return np.zeros((0, 2))
        if len(t) < 3:
            return xy[-1] + tau[:, None] * last_velocity(history, t, xy)
        dt1 = float(t[-1] - t[-2])
        dt0 = float(t[-2] - t[-3])
        v1 = (xy[-1] - xy[-2]) / dt1
        v0 = (xy[-2] - xy[-3]) / dt0
        accel = (v1 - v0) / (0.5 * (dt0 + dt1))
        # Interval velocities sit at the sample midpoints; shift to the last timestamp.
        v_now = v1 + 0.5 * accel * dt1
        return xy[-1] + tau[:, None] * v_now + 0.5 * (tau**2)[:, None] * accel

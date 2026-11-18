"""M2 — CV / CA motion models. Owner: P.

NB: CV extrapolates the full 2D velocity (including lateral). That lateral term is what
separates it from the Reactive baseline — see docs/spec.md §1.4.
"""

from numbers import Integral

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
    """Fit recent positions, anchored at the last observation; never read GT accel.

    One/two observations use CV. Braking clamps travel along the fitted velocity
    at zero longitudinal speed; perpendicular acceleration is still extrapolated.
    """

    name = "ca"

    def __init__(self, m: int = 10):
        if isinstance(m, bool) or not isinstance(m, Integral) or m < 3:
            raise ValueError("m must be an integer >= 3")
        self.m = m

    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        t, xy, tau = prediction_inputs(history, horizon, dt)
        if len(t) < 3:
            return xy[-1] + tau[:, None] * last_velocity(history, t, xy)
        if np.linalg.norm(xy[-1] - xy[-2]) < 1e-10:
            return np.tile(xy[-1], (len(tau), 1))
        # Relative times/positions avoid conditioning on large world coordinates.
        times = t[-self.m :] - t[-1]
        scale = -times[0]
        u = times / scale
        design = np.column_stack((np.ones(len(u)), u, 0.5 * u**2))
        coefficients = np.linalg.lstsq(design, xy[-self.m :] - xy[-1], rcond=None)[0]
        velocity = coefficients[1] / scale
        acceleration = coefficients[2] / scale**2
        observed_velocity = last_velocity(history, t, xy)
        # A window straddling the stop can fit a small negative end velocity.
        # Do not turn that estimator overshoot into reverse motion.
        if velocity @ observed_velocity <= 0 and acceleration @ observed_velocity < 0:
            return np.tile(xy[-1], (len(tau), 1))
        speed = np.linalg.norm(velocity)
        travel_time = tau.copy()
        parallel = np.zeros(2)
        if speed > 1e-10:
            direction = velocity / speed
            decel = float(acceleration @ direction)
            parallel = decel * direction
            if decel < 0:
                travel_time = np.minimum(tau, speed / -decel)
        return (
            xy[-1]
            + travel_time[:, None] * velocity
            + 0.5 * travel_time[:, None] ** 2 * parallel
            + 0.5 * tau[:, None] ** 2 * (acceleration - parallel)
        )

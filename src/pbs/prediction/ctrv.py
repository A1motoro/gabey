"""M2 — CTRV (constant turn rate and velocity). Owner: P.

State (x, y, v, heading, yaw_rate); yaw_rate estimated from heading history.
Models curved motion when heading differs from lane direction. Changing turn rate
(e.g. a quintic lane change) can cause overshoot; CTRV is not always better than CV.
"""

from numbers import Integral

import numpy as np

from pbs.prediction._motion import last_velocity, prediction_inputs
from pbs.prediction.base import Predictor
from pbs.types import Trajectory


class CTRVPredictor(Predictor):
    name = "ctrv"

    def __init__(self, m: int = 10):
        if isinstance(m, bool) or not isinstance(m, Integral) or m < 2:
            raise ValueError("m must be an integer >= 2")
        self.m = m

    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        t, xy, tau = prediction_inputs(history, horizon, dt)
        if len(t) < 2:
            return xy[-1] + tau[:, None] * last_velocity(history, t, xy)
        times = t[-self.m :] - t[-1]
        headings = np.unwrap(history.heading[-self.m :])
        centered = times - times.mean()
        yaw_rate = float(centered @ (headings - headings.mean()) / (centered @ centered))
        if abs(yaw_rate) < 1e-8:
            return xy[-1] + tau[:, None] * last_velocity(history, t, xy)
        # Mid-angle form is stable even when yaw_rate * tau is tiny.
        angle = yaw_rate * tau
        distance = history.v[-1] * tau * np.sinc(angle / (2 * np.pi))
        midpoint_heading = history.heading[-1] + angle / 2
        return xy[-1] + distance[:, None] * np.column_stack(
            (np.cos(midpoint_heading), np.sin(midpoint_heading))
        )

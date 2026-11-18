"""M2 — LinearRegressionPredictor: fit x(t), y(t) on the last m steps. Owner: P.

Windowed ordinary least squares on positions only. The fitted intercept smooths
observation noise; this is a linear trend model, not an acceleration/turn model.
"""

from numbers import Integral

import numpy as np

from pbs.prediction._motion import last_velocity, prediction_inputs
from pbs.prediction.base import Predictor
from pbs.types import Trajectory


class LinearRegressionPredictor(Predictor):
    name = "lr"

    def __init__(self, m: int = 10):
        if isinstance(m, bool) or not isinstance(m, Integral) or m < 2:
            raise ValueError("m must be an integer >= 2")
        self.m = m

    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        t, xy, tau = prediction_inputs(history, horizon, dt)
        if len(t) == 1:
            return xy[-1] + tau[:, None] * last_velocity(history, t, xy)
        times = t[-self.m :] - t[-1]
        scale = -times[0]
        design = np.column_stack((np.ones(len(times)), times / scale))
        # Center time and position before fitting to avoid large-coordinate conditioning.
        intercept, scaled_velocity = np.linalg.lstsq(design, xy[-self.m :] - xy[-1], rcond=None)[0]
        return xy[-1] + intercept + tau[:, None] * (scaled_velocity / scale)

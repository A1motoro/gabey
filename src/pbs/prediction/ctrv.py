"""M2 — CTRV (constant turn rate and velocity). Owner: P.

State (x, y, v, heading, yaw_rate); yaw_rate estimated from heading history.
Motivation: during a lane change heading ≠ lane direction, so CTRV extrapolates the
lateral motion better than CV → earlier cut-in detection.
"""

import numpy as np

from pbs.prediction._motion import last_velocity, prediction_inputs
from pbs.prediction.base import Predictor
from pbs.types import Trajectory

_YAW_EPS = 1e-6


def _wrap(angle: float) -> float:
    return float((angle + np.pi) % (2 * np.pi) - np.pi)


class CTRVPredictor(Predictor):
    """Constant turn rate and speed. A negligible yaw rate uses the CV update exactly."""

    name = "ctrv"

    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        t, xy, tau = prediction_inputs(history, horizon, dt)
        vel = last_velocity(history, t, xy)
        if len(tau) == 0:
            return np.zeros((0, 2))
        speed = float(np.linalg.norm(vel))
        if len(t) >= 2 and speed > _YAW_EPS:
            heading = float(np.arctan2(vel[1], vel[0]))
        else:
            heading = float(history.heading[-1])
        yaw_rate = 0.0
        if len(t) >= 3 and speed > _YAW_EPS:
            prev = xy[-2] - xy[-3]
            if float(np.linalg.norm(prev)) > _YAW_EPS:
                prev_heading = float(np.arctan2(prev[1], prev[0]))
                yaw_rate = _wrap(heading - prev_heading) / float(t[-1] - t[-2])
        if abs(yaw_rate) < _YAW_EPS or speed <= _YAW_EPS:
            return xy[-1] + tau[:, None] * vel
        turn = yaw_rate * tau
        origin = xy[-1]
        x = origin[0] + speed / yaw_rate * (np.sin(heading + turn) - np.sin(heading))
        y = origin[1] + speed / yaw_rate * (-np.cos(heading + turn) + np.cos(heading))
        return np.column_stack((x, y))

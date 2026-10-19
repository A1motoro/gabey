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
    name = "ca"

    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        raise NotImplementedError("TODO(P): W5–W6")

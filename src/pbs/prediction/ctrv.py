"""M2 — CTRV (constant turn rate and velocity). Owner: P.

State (x, y, v, heading, yaw_rate); yaw_rate estimated from heading history.
Motivation: during a lane change heading ≠ lane direction, so CTRV extrapolates the
lateral motion better than CV → earlier cut-in detection.
"""

import numpy as np

from pbs.prediction.base import Predictor
from pbs.types import Trajectory


class CTRVPredictor(Predictor):
    name = "ctrv"

    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        raise NotImplementedError("TODO(P): W5–W6 (handle yaw_rate ≈ 0 → fall back to CV)")

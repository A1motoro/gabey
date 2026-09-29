"""M2 — LinearRegressionPredictor: fit x(t), y(t) on the last m steps. Owner: P.

The only data-driven predictor. Lowest priority — cut first if time runs out (L3).
"""

import numpy as np

from pbs.prediction.base import Predictor
from pbs.types import Trajectory


class LinearRegressionPredictor(Predictor):
    name = "lr"

    def __init__(self, m: int = 10):
        self.m = m

    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        raise NotImplementedError("TODO(P): W8–W10, optional (L3)")

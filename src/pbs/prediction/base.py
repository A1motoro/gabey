"""M2 — predictor interface. Owner: P."""

from abc import ABC, abstractmethod

import numpy as np

from pbs.prediction._motion import prediction_inputs
from pbs.types import Scenario, Trajectory


class Predictor(ABC):
    name = "base"

    def bind(self, scenario: Scenario) -> None:
        """Called once by run_sim before the loop. No-op except for OraclePredictor."""
        return None

    @abstractmethod
    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        """Future positions at t_last + k*dt, k = 1..horizon. Returns (horizon, 2)."""


class StaticPredictor(Predictor):
    """Walking-skeleton placeholder: agent stays at its last observed position."""

    name = "static"

    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        _, xy, tau = prediction_inputs(history, horizon, dt)
        return np.tile(xy[-1], (len(tau), 1))

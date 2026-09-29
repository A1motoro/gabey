"""M2 — OraclePredictor: returns the agent's TRUE future (upper bound). Owner: P.

Gap(Oracle, CV) in the boundary plot = cost of prediction error.
"""

import numpy as np

from pbs.prediction.base import Predictor
from pbs.types import Scenario, Trajectory


class OraclePredictor(Predictor):
    name = "oracle"

    def bind(self, scenario: Scenario) -> None:
        self._agents = {a.agent_id: a for a in scenario.agents}

    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        # look up self._agents[history.agent_id];
        # sample at history.t[-1] + k*dt via pbs.utils.state_at
        raise NotImplementedError("TODO(P): W8")

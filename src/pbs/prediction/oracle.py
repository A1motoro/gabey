"""M2 — OraclePredictor: returns the agent's TRUE future (upper bound). Owner: P.

Gap(Oracle, CV) in the boundary plot = cost of prediction error.
"""

import numpy as np

from pbs.prediction._motion import prediction_inputs
from pbs.prediction.base import Predictor
from pbs.types import Scenario, Trajectory


class OraclePredictor(Predictor):
    """True future of the bound scenario. Prediction error against CV is the oracle gap."""

    name = "oracle"

    def bind(self, scenario: Scenario) -> None:
        self._agents = {a.agent_id: a for a in scenario.agents}

    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        t, _, tau = prediction_inputs(history, horizon, dt)
        if len(tau) == 0:
            return np.zeros((0, 2))
        if not hasattr(self, "_agents"):
            raise RuntimeError("OraclePredictor.bind(scenario) must be called before predict")
        try:
            agent = self._agents[history.agent_id]
        except KeyError as exc:
            raise KeyError(f"no agent_id {history.agent_id!r} in the bound scenario") from exc
        times = float(t[-1]) + tau
        # Same endpoint-hold interpolation as pbs.utils.state_at, vectorized.
        return np.column_stack(
            (np.interp(times, agent.t, agent.x), np.interp(times, agent.t, agent.y))
        )

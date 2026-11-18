"""M2 — OraclePredictor: returns the agent's TRUE future (upper bound). Owner: P.

Gap(Oracle, CV) in the boundary plot = cost of prediction error.
"""

import numpy as np

from pbs.prediction._motion import prediction_inputs
from pbs.prediction.base import Predictor
from pbs.types import Scenario, Trajectory


class OraclePredictor(Predictor):
    name = "oracle"

    def bind(self, scenario: Scenario) -> None:
        agents = {}
        for agent in scenario.agents:
            if not agent.agent_id or agent.agent_id in agents:
                raise ValueError("Oracle requires unique non-empty agent_id values")
            t, xy, _ = prediction_inputs(agent, 0, scenario.dt)
            # Snapshot truth so repeated predictions neither expose nor mutate scenario arrays.
            agents[agent.agent_id] = (t.copy(), xy.copy())
        self._agents = agents

    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        t, _, tau = prediction_inputs(history, horizon, dt)
        if not hasattr(self, "_agents"):
            raise ValueError("OraclePredictor must bind a scenario before predict")
        if history.agent_id not in self._agents:
            raise ValueError(f"Oracle has no truth for agent_id {history.agent_id!r}")
        truth_t, truth_xy = self._agents[history.agent_id]
        query = t[-1] + tau
        # Same linear interpolation / endpoint hold semantics as pbs.utils.state_at.
        return np.column_stack(
            (np.interp(query, truth_t, truth_xy[:, 0]), np.interp(query, truth_t, truth_xy[:, 1]))
        )

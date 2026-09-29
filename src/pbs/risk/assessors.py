"""Three tiers of AEB risk assessment. Owner: P.

  Reactive   — no prediction. Only agents ALREADY inside the ego lane; longitudinal
               TTC = range / range_rate (line-of-sight constant velocity). Lateral motion
               is ignored ON PURPOSE.
  Predictive — predictor extrapolates every agent in 2D (incl. lateral) → assess_risk.
  Oracle     — Predictive with OraclePredictor (true future).

Why Reactive ≠ CV (must hold, see docs/spec.md §1.4): Reactive only sees a cutter once it
is in-lane; CV sees it while it is still changing lanes. The difference = lead time.
"""

from abc import ABC, abstractmethod

from pbs.prediction.base import Predictor
from pbs.types import RiskContext, RiskEstimate, Scenario, Trajectory


class RiskAssessor(ABC):
    name = "base"

    def bind(self, scenario: Scenario) -> None:
        """Called once by run_sim before the loop."""
        return None

    @abstractmethod
    def assess(self, ctx: RiskContext, history: Trajectory) -> RiskEstimate:
        """history = what is observable at ctx.t_now (after latency)."""


class NullAssessor(RiskAssessor):
    """Walking-skeleton placeholder: never sees a conflict."""

    name = "null"

    def assess(self, ctx: RiskContext, history: Trajectory) -> RiskEstimate:
        return RiskEstimate()


class ReactiveAssessor(RiskAssessor):
    name = "reactive"

    def assess(self, ctx: RiskContext, history: Trajectory) -> RiskEstimate:
        # 1. lateral offset of agent from ctx.ego_path; |offset| > lane_width/2 → considered=False
        # 2. agent must be ahead (longitudinal gap > 0)
        # 3. range = gap - r_safe; range_rate = ego_v - agent longitudinal speed
        # 4. ttc = range / range_rate if range_rate > 0 else inf
        raise NotImplementedError("TODO(P): W5–W6")


class PredictiveAssessor(RiskAssessor):
    def __init__(self, predictor: Predictor):
        self.predictor = predictor
        self.name = predictor.name

    def bind(self, scenario: Scenario) -> None:
        self.predictor.bind(scenario)

    def assess(self, ctx: RiskContext, history: Trajectory) -> RiskEstimate:
        from pbs.risk.ttc import assess_risk

        agent_future = self.predictor.predict(history, ctx.horizon, ctx.dt)
        return assess_risk(ctx.ego_future, agent_future, ctx.dt, ctx.r_safe)

"""Replace demo risk assessors here; each factory must create a fresh instance.

Custom assessors implement RiskAssessor.bind/assess. Use AssessorBundle.label
for their displayed name. Optional predictor captures the actual forecast; omit
it if the new assessor does not expose a trajectory (the overlay stays empty).
"""

from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from pbs.prediction.constant import ConstantVelocityPredictor
from pbs.risk.assessors import PredictiveAssessor, ReactiveAssessor, RiskAssessor


class RecordedCV(ConstantVelocityPredictor):
    def predict(self, history, horizon, dt):
        forecast = super().predict(history, horizon, dt)
        self.latest = forecast.copy()
        return forecast


class RecordingAssessor(RiskAssessor):
    """Record exactly what the wrapped assessor saw and returned, without extra inference."""

    def __init__(self, inner, predictor=None):
        self.inner, self.predictor = inner, predictor
        self.frames = []

    def bind(self, scenario):
        self.frames.clear()
        self.inner.bind(scenario)

    def assess(self, ctx, history):
        estimate = self.inner.assess(ctx, history)
        self.frames.append(
            {
                "t": float(ctx.t_now),
                "agent_id": history.agent_id,
                "observed": {
                    "t": history.t.tolist(),
                    "x": history.x.tolist(),
                    "y": history.y.tolist(),
                },
                "predicted": self.predictor.latest.tolist() if self.predictor else [],
                "considered": bool(estimate.considered),
                "ttc": float(estimate.ttc) if np.isfinite(estimate.ttc) else None,
                "ego_heading": float(ctx.ego_heading),
            }
        )
        return estimate


class ForecastTrace(Protocol):
    latest: np.ndarray


@dataclass
class AssessorBundle:
    assessor: RiskAssessor
    label: str
    predictor: ForecastTrace | None = None


def make_demo_assessor(name: str) -> AssessorBundle:
    if name == "reactive":
        return AssessorBundle(ReactiveAssessor(), "Reactive")
    if name == "cv":
        predictor = RecordedCV()
        return AssessorBundle(PredictiveAssessor(predictor), "Predictive · CV", predictor)
    raise ValueError(f"Unknown demo system: {name}")


AssessorFactory = Callable[[str], AssessorBundle]

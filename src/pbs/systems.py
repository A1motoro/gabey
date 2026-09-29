"""Registry of AEB systems compared in every experiment. Owner: D (with P).

Tier        name       what it isolates
reactive    reactive   no prediction; in-lane agents only, longitudinal TTC
predictive  cv/ca/ctrv/lr  prediction quality
upper bound oracle     prediction error = 0
"""

from pbs.prediction import (
    ConstantAccelPredictor,
    ConstantVelocityPredictor,
    CTRVPredictor,
    LinearRegressionPredictor,
    OraclePredictor,
)
from pbs.risk import PredictiveAssessor, ReactiveAssessor, RiskAssessor

SYSTEMS = {
    "reactive": lambda: ReactiveAssessor(),
    "cv": lambda: PredictiveAssessor(ConstantVelocityPredictor()),
    "ca": lambda: PredictiveAssessor(ConstantAccelPredictor()),
    "ctrv": lambda: PredictiveAssessor(CTRVPredictor()),
    "lr": lambda: PredictiveAssessor(LinearRegressionPredictor()),  # L3, optional
    "oracle": lambda: PredictiveAssessor(OraclePredictor()),
}
CORE = ("reactive", "cv", "ca", "oracle")  # the 4 boundaries on the headline figure


def make_system(name: str) -> RiskAssessor:
    return SYSTEMS[name]()

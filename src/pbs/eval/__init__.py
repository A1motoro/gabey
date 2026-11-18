"""Evaluation metrics, the experiment matrix, and figures. Owner: D."""

from pbs.eval.counterfactual import is_benign
from pbs.eval.metrics import ade, evaluate, fde
from pbs.eval.plots import plot_boundaries, plot_stopping_distance, plot_timeline
from pbs.eval.sweep import (
    failure_boundary,
    lead_times,
    rescue_rate,
    run_matrix,
    safe_area,
    select_operating_point,
    tradeoff,
)

__all__ = [
    "ade",
    "evaluate",
    "fde",
    "failure_boundary",
    "is_benign",
    "lead_times",
    "plot_boundaries",
    "plot_stopping_distance",
    "plot_timeline",
    "rescue_rate",
    "run_matrix",
    "safe_area",
    "select_operating_point",
    "tradeoff",
]

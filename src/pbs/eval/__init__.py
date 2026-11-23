"""Evaluation metrics, the experiment matrix, and figures. Owner: D."""

from pbs.eval.counterfactual import is_benign, never_brake_label
from pbs.eval.metrics import ade, evaluate, fde
from pbs.eval.plots import plot_boundaries, plot_stopping_distance, plot_timeline
from pbs.eval.sweep import (
    failure_boundary,
    late_contact_outcomes,
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
    "late_contact_outcomes",
    "lead_times",
    "never_brake_label",
    "plot_boundaries",
    "plot_stopping_distance",
    "plot_timeline",
    "rescue_rate",
    "run_matrix",
    "safe_area",
    "select_operating_point",
    "tradeoff",
]

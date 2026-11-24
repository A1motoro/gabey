"""Counterfactual scenario labels. Owner: D.

A scenario is BENIGN iff the ego does not collide when it never brakes.
In a benign scenario, any brake is a false brake.

``never_brake_label`` separates two windows. Decisions are scored for
``decision_duration`` seconds; contact is looked for until ``sim_kwargs["duration"]``.
A scenario whose never-brake contact falls between the two is neither class: a brake
before it is correct, and one run of ``decision_duration`` cannot see why.
"""

from pbs.risk.assessors import NullAssessor
from pbs.sim.policy import NeverBrakePolicy
from pbs.sim.simulator import run_sim
from pbs.types import Scenario

DANGEROUS = "dangerous"
LATE_CONTACT = "late_contact"
BENIGN = "benign"
LABELS = (DANGEROUS, LATE_CONTACT, BENIGN)


def is_benign(scenario: Scenario, **sim_kwargs) -> bool:
    return not run_sim(scenario, NullAssessor(), NeverBrakePolicy(), **sim_kwargs).collision


def never_brake_label(
    scenario: Scenario, decision_duration: float | None = None, **sim_kwargs
) -> tuple[str, float | None]:
    """Class of ``scenario`` and the time of its first never-brake contact.

    ``dangerous``: contact in a never-brake run of ``decision_duration``. This is
    ``not is_benign(scenario, duration=decision_duration)``, the one-window label.
    ``late_contact``: no contact in that run, contact in the run of ``duration``.
    ``benign``: no contact in the run of ``duration``.

    With ``decision_duration=None`` there is one window and no ``late_contact``.
    """
    if decision_duration is None:
        log = _never_brake(scenario, sim_kwargs)
        return (DANGEROUS, log.first_collision_t) if log.collision else (BENIGN, None)
    if "duration" not in sim_kwargs:
        raise ValueError("decision_duration needs an explicit sim duration")
    outcome = float(sim_kwargs["duration"])
    decision = float(decision_duration)
    if not 0.0 < decision <= outcome:
        raise ValueError(f"decision_duration must be in (0, {outcome}], got {decision}")
    scored = _never_brake(scenario, {**sim_kwargs, "duration": decision})
    if scored.collision:
        return DANGEROUS, scored.first_collision_t
    if decision == outcome:
        return BENIGN, None
    whole = _never_brake(scenario, sim_kwargs)
    return (LATE_CONTACT, whole.first_collision_t) if whole.collision else (BENIGN, None)


def _never_brake(scenario: Scenario, sim_kwargs: dict):
    return run_sim(scenario, NullAssessor(), NeverBrakePolicy(), **sim_kwargs)

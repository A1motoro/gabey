"""Counterfactual definition of benign scenarios. Owner: D.

A scenario is BENIGN iff the ego does not collide when it never brakes.
In a benign scenario, any brake is a false brake.
"""

from pbs.risk.assessors import NullAssessor
from pbs.sim.policy import NeverBrakePolicy
from pbs.sim.simulator import run_sim
from pbs.types import Scenario


def is_benign(scenario: Scenario, **sim_kwargs) -> bool:
    return not run_sim(scenario, NullAssessor(), NeverBrakePolicy(), **sim_kwargs).collision

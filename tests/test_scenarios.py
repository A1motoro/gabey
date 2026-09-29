"""Smoke test: the walking skeleton runs end-to-end and returns a valid SimLog."""

import numpy as np

from pbs.risk import NullAssessor
from pbs.sim.policy import NeverBrakePolicy
from pbs.sim.simulator import run_sim
from pbs.types import Scenario, SimLog, Trajectory


def straight_scenario(lead_v: float = 10.0) -> Scenario:
    """Lead vehicle 30 m ahead in the ego lane; ego at 15 m/s."""
    t = np.arange(0, 10.01, 0.1)
    lead = Trajectory(
        t,
        30 + lead_v * t,
        np.zeros_like(t),
        np.full_like(t, lead_v),
        np.zeros_like(t),
        agent_id="lead",
    )
    path = np.array([[0.0, 0.0], [500.0, 0.0]])
    return Scenario(ego_start=(0.0, 0.0, 0.0, 15.0), ego_path=path, agents=[lead])


def test_skeleton_runs():
    log = run_sim(straight_scenario(), NullAssessor(), NeverBrakePolicy(), duration=10.0)
    assert isinstance(log, SimLog)
    n = len(log.t)
    assert n == 101
    arrays = (log.ego_x, log.ego_y, log.ego_v, log.ego_a, log.ttc, log.action)
    assert all(len(a) == n for a in arrays)
    assert set(log.action) <= {"cruise", "brake"}
    assert np.isclose(log.ego_x[10], 15.0)
    # ego closes a 30 m gap at 5 m/s → contact at t ≈ 6 s
    assert log.collision


def test_latency_arg_accepted():
    run_sim(straight_scenario(), NullAssessor(), NeverBrakePolicy(), latency=0.3)

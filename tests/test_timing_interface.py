"""Reserved timing models never pretend that nonzero delays have been simulated."""

import numpy as np
import pytest

from pbs.demo.service import simulate
from pbs.risk.assessors import NullAssessor
from pbs.sim.observation import IdentityObservation
from pbs.sim.policy import NeverBrakePolicy
from pbs.sim.simulator import run_sim
from pbs.sim.timing import NoComputationDelay
from tests.test_scenarios import straight_scenario


def test_explicit_identity_noise_and_zero_delay_preserve_demo_results():
    assert simulate({}) == simulate(
        {}, observation_factory=IdentityObservation, computation_delay_factory=NoComputationDelay
    )


def test_timing_model_called_once_per_executed_decision_not_per_agent_or_terminal():
    class Timing:
        def __init__(self):
            self.times = []

        def delay_seconds(self, *, t_now):
            self.times.append(t_now)
            return 0.0

    scenario = straight_scenario()
    timing = Timing()
    run_sim(
        scenario, NullAssessor(), NeverBrakePolicy(), duration=0.2, computation_delay_model=timing
    )
    assert timing.times == pytest.approx([0, 0.1])


@pytest.mark.parametrize("delay", [0.001, 0.1, 0.3])
def test_nonzero_computation_delay_fails_explicitly(delay):
    class Timing:
        def delay_seconds(self, *, t_now):
            return delay

    with pytest.raises(NotImplementedError, match="execution queue"):
        run_sim(
            straight_scenario(),
            NullAssessor(),
            NeverBrakePolicy(),
            computation_delay_model=Timing(),
        )


@pytest.mark.parametrize("delay", [-0.1, np.nan, np.inf, True, "0"])
def test_invalid_delay_cannot_be_silently_accepted(delay):
    class Timing:
        def delay_seconds(self, *, t_now):
            return delay

    with pytest.raises(ValueError, match="finite and nonnegative"):
        run_sim(
            straight_scenario(),
            NullAssessor(),
            NeverBrakePolicy(),
            computation_delay_model=Timing(),
        )


def test_demo_constructs_independent_timing_instances_per_side():
    models = []

    class Timing:
        def __init__(self):
            self.calls = 0

        def delay_seconds(self, *, t_now):
            self.calls += 1
            return 0.0

    def factory():
        model = Timing()
        models.append(model)
        return model

    simulate({}, computation_delay_factory=factory)
    assert len(models) == 2 and models[0] is not models[1]
    assert [m.calls for m in models] == [100, 100]

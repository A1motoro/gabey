import numpy as np
import pytest

from pbs.sim.policy import BRAKE, CRUISE, HysteresisPolicy


def test_hysteresis_equalities_and_reset():
    policy = HysteresisPolicy(2, 3)
    values = [2.0, 1.8, 2.0, 3.0, 3.1, 2.5, 1.9, np.inf]
    expected = [CRUISE, BRAKE, BRAKE, BRAKE, CRUISE, CRUISE, BRAKE, CRUISE]
    assert [policy.decide(x) for x in values] == expected
    policy.decide(1)
    policy.reset()
    assert policy.decide(2.5) == CRUISE


@pytest.mark.parametrize("args", [(3, 2, 6), (0, 3, 6), (2, 2, 6), (2, 3, 0), (2, np.inf, 6)])
def test_invalid_policy_parameters(args):
    with pytest.raises(ValueError):
        HysteresisPolicy(*args)


@pytest.mark.parametrize("ttc", [np.nan, -1, -np.inf])
def test_invalid_ttc(ttc):
    with pytest.raises(ValueError):
        HysteresisPolicy().decide(ttc)

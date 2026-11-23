import numpy as np
import pytest

from pbs.sim.policy import (
    BRAKE,
    CRUISE,
    HARD_BRAKE,
    RECOVER,
    SOFT_BRAKE,
    HysteresisPolicy,
    StagedBrakingPolicy,
    is_braking_action,
)


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


def test_staged_hysteresis_and_action_labels():
    policy = StagedBrakingPolicy()
    policy.reset(v_target=12)
    cases = [
        (3.0, 12, CRUISE),
        (2.9, 12, SOFT_BRAKE),
        (1.5, 11, SOFT_BRAKE),
        (1.49, 11, HARD_BRAKE),
        (2.0, 10, HARD_BRAKE),
        (2.01, 10, SOFT_BRAKE),
        (3.5, 9, SOFT_BRAKE),
        (3.51, 9, RECOVER),
        (np.inf, 12, CRUISE),
    ]
    for ttc, speed, label in cases:
        assert policy.decide_accel(ttc, v=speed, dt=0.1)[0] == label
    assert all(is_braking_action(label) for label in [BRAKE, SOFT_BRAKE, HARD_BRAKE])
    assert not any(is_braking_action(label) for label in [CRUISE, RECOVER])


@pytest.mark.parametrize(
    "settings",
    [
        {"t_on_soft": 1.0},
        {"t_off_hard": 1.5},
        {"a_soft": 8},
        {"max_jerk": 0},
        {"a_recover": np.nan},
    ],
)
def test_staged_rejects_invalid_configuration(settings):
    with pytest.raises(ValueError):
        StagedBrakingPolicy(**settings)

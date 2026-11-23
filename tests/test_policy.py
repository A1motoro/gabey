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


def test_staged_pressure_ramps_and_comfort_release():
    policy = StagedBrakingPolicy(v_target=20)

    def ramp(ttc, steps):
        return [policy.decide_accel(ttc, v=15, dt=0.1)[1] for _ in range(steps)]

    assert ramp(2.5, 5) == [-0.5, -1, -1.5, -2, -2.5]
    assert ramp(1.0, 2) == [-4.5, -6]
    assert ramp(3.5, 1) == [-5.5] and policy.state == SOFT_BRAKE
    assert ramp(3.2, 6) == [-5, -4.5, -4, -3.5, -3, -2.5]
    assert ramp(np.inf, 8) == [-2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5]
    assert policy.state == RECOVER
    policy.reset()
    assert policy.v_target == 20 and ramp(1.0, 3) == [-2, -4, -6]


@pytest.mark.parametrize("ttc", [2.01, 3.0, 3.2, 3.5])
def test_hard_braking_downgrades_across_entire_release_band(ttc):
    policy = StagedBrakingPolicy()
    policy.decide_accel(1.0, v=10, dt=0.1)
    assert policy.decide_accel(ttc, v=10, dt=0.1)[0] == SOFT_BRAKE


def test_custom_jerk_rates_and_legacy_uniform_rate():
    policy = StagedBrakingPolicy(jerk_soft=3, jerk_hard=12, a_recover=0, v_target=20)
    assert policy.decide_accel(np.inf, v=15, dt=0.2) == (CRUISE, 0.0)
    assert policy.decide_accel(2.5, v=15, dt=0.2)[1] == pytest.approx(-0.6)
    assert policy.decide_accel(1.0, v=15, dt=0.2)[1] == pytest.approx(-3.0)
    old = StagedBrakingPolicy(max_jerk=10)
    assert old.decide_accel(2.5, v=15, dt=0.1)[1] == -1
    assert old.decide_accel(1.0, v=15, dt=0.1)[1] == -2


@pytest.mark.parametrize(
    "kwargs", [{"jerk_soft": 0}, {"jerk_hard": np.inf}, {"jerk_soft": 21}, {"v_target": -1}]
)
def test_invalid_dual_rate_configuration(kwargs):
    with pytest.raises(ValueError):
        StagedBrakingPolicy(**kwargs)

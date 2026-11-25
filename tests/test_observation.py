from dataclasses import replace

import numpy as np
import pytest

from pbs.risk.assessors import RiskAssessor
from pbs.sim.observation import GaussianObservationModel
from pbs.sim.policy import NeverBrakePolicy
from pbs.sim.simulator import run_sim
from pbs.types import RiskEstimate, Scenario, Trajectory
from pbs.utils import history_until


def track():
    times = np.arange(11) / 10
    return Trajectory(
        times,
        10 - 20 * times,
        times * 0,
        times * 0,
        times * 0,
        agent_id="car",
        accel=np.ones((len(times), 2)),
    )


def test_overlapping_histories_are_seeded_independent_measurements():
    truth = track()
    model = GaussianObservationModel(seed=42)
    prefix = model.observe(history_until(truth, 0.5), t_now=0.5)
    full = model.observe(truth, t_now=1)
    repeat = GaussianObservationModel(seed=42).observe(truth, t_now=1)
    other = model.observe(replace(truth, agent_id="other"), t_now=1)
    for field in ("x", "y", "v", "heading"):
        observed = getattr(full, field)
        np.testing.assert_array_equal(getattr(prefix, field), observed[:6])
        np.testing.assert_array_equal(getattr(repeat, field), observed)
    assert not np.array_equal(other.x, full.x)
    assert not np.array_equal(full.x, truth.x)
    assert (full.v >= 0).all() and full.accel is None
    assert full.agent_type == truth.agent_type and full.agent_id == truth.agent_id
    np.testing.assert_array_equal(full.t, truth.t)
    assert not np.shares_memory(truth.t, full.t)
    clean = GaussianObservationModel(pos_std=0, speed_std=0, heading_std=0).observe(truth, t_now=1)
    for field in ("x", "y", "v", "heading"):
        np.testing.assert_array_equal(getattr(clean, field), getattr(truth, field))
    with pytest.raises(ValueError, match="t_now"):
        model.observe(truth, t_now=0.5)


def test_observation_changes_assessment_input_but_contact_uses_truth():
    class Spy(RiskAssessor):
        def assess(self, ctx, history):
            self.last_x = history.x.copy()
            return RiskEstimate()

    truth = track()
    scenario = Scenario((0, 0, 0, 0), np.array([[0, 0], [100, 0]]), [truth])
    spy = Spy()
    clean = run_sim(scenario, spy, NeverBrakePolicy(), duration=1, vehicle_width=2)
    noisy = run_sim(
        scenario,
        spy,
        NeverBrakePolicy(),
        duration=1,
        vehicle_width=2,
        observation_model=GaussianObservationModel(pos_std=100, seed=17),
    )
    np.testing.assert_array_equal(truth.x, track().x)
    assert not np.array_equal(spy.last_x, truth.x)
    assert noisy.collision and clean.collision
    assert noisy.first_collision_t == clean.first_collision_t == pytest.approx(0.25)


@pytest.mark.parametrize(
    "settings", [{"pos_std": -1}, {"heading_std": np.nan}, {"speed_std": np.inf}]
)
def test_invalid_noise_parameters(settings):
    with pytest.raises(ValueError):
        GaussianObservationModel(**settings)

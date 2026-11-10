"""First-contact metadata, assessor replacement and collision-limited display stats."""

from dataclasses import replace

import numpy as np
import pytest

from experiments.exp06_demo_comparison import classify
from pbs.demo.assessors import AssessorBundle, make_demo_assessor
from pbs.demo.service import simulate
from pbs.risk.assessors import NullAssessor
from pbs.sim.collision import swept_contact_fraction
from pbs.sim.policy import NeverBrakePolicy
from pbs.sim.simulator import run_sim
from tests.test_scenarios import straight_scenario


def test_contact_between_frames_and_tangent_is_not_a_collision():
    assert swept_contact_fraction([0, 0], [10, 0], [5, 0], [5, 0], 1) == pytest.approx(0.4)
    assert swept_contact_fraction([0, 0], [10, 0], [5, 1], [5, 1], 1) is None
    assert swept_contact_fraction([0, 0], [0, 0], [0, 0], [0, 0], 1) == 0
    assert swept_contact_fraction([0, 0], [0, 0], [3, 0], [3, 0], 1) is None


def test_first_collision_is_earliest_across_agents_and_does_not_change_physics():
    scenario = straight_scenario(lead_v=0)
    near = replace(scenario.agents[0], x=np.full_like(scenario.agents[0].x, 20), agent_id="near")
    scenario.agents.append(near)
    log = run_sim(scenario, NullAssessor(), NeverBrakePolicy())
    assert log.first_collision_t == pytest.approx((20 - 2.5) / 15)
    assert log.collision
    assert log.ego_x[-1] == pytest.approx(
        150
    )  # Freeze is a presentation rule, not impact dynamics.
    scenario.agents.reverse()
    repeated = run_sim(scenario, NullAssessor(), NeverBrakePolicy())
    assert repeated.first_collision_t == log.first_collision_t


def test_initial_overlap_records_zero_and_safe_run_has_no_collision_time():
    scene = straight_scenario(lead_v=0)
    scene.agents[0].x[:] = 1
    assert run_sim(scene, NullAssessor(), NeverBrakePolicy()).first_collision_t == 0
    safe = straight_scenario(lead_v=20)
    assert run_sim(safe, NullAssessor(), NeverBrakePolicy()).first_collision_t is None


def test_reactive_contact_does_not_freeze_other_side_or_include_later_braking():
    result = simulate({"gap_d": 10, "rel_v": 4})
    reactive, cv = (result["systems"][n] for n in ("reactive", "cv"))
    assert 2 < reactive["first_collision_t"] < 2.1
    assert reactive["summary"]["metrics_until_t"] == reactive["first_collision_t"]
    assert reactive["summary"]["brake_duration_s"] == pytest.approx(
        reactive["first_collision_t"] - 2
    )
    assert reactive["braking"]["episodes"][-1]["release_t"] is None
    assert reactive["summary"]["min_dist"] == pytest.approx(2.5)
    assert cv["first_collision_t"] is None
    assert cv["summary"]["metrics_until_t"] == 10


def test_gap_five_is_supported_and_assessor_can_be_replaced_without_ui_changes():
    result = simulate({"gap_d": 5, "rel_v": 1})
    assert all(not s["summary"]["collision"] for s in result["systems"].values())
    calls = []

    def custom(name):
        calls.append(name)
        if name == "cv":
            return AssessorBundle(NullAssessor(), "Predictive · Custom")
        return make_demo_assessor(name)

    changed = simulate({}, assessor_factory=custom)
    assert calls == ["reactive", "cv"]
    assert changed["systems"]["cv"]["label"] == "Predictive · Custom"
    assert changed["systems"]["cv"]["summary"]["collision"]
    assert all(not f["predicted"] for f in changed["systems"]["cv"]["trace"])
    assert not changed["systems"]["reactive"]["summary"]["collision"]


def test_comparison_prioritizes_safety_and_marks_tradeoffs():
    base = dict(
        collision=False, min_dist=3.0, brake_count=2, brake_duration_s=0.5, false_brake=False
    )
    assert classify({**base, "collision": True}, base) == "predict_safety"
    assert classify(base, {**base, "collision": True}) == "reactive_safety"
    assert classify({**base, "collision": True}, {**base, "collision": True}) == "both_collision"
    assert classify(base, base) == "tie_safe"
    assert classify({**base, "brake_count": 1}, base) == "reactive_control_margin"
    assert classify({**base, "brake_count": 1}, {**base, "min_dist": 4}) == "tradeoff"


def test_both_contact_cases_have_independent_cutoffs_and_no_post_contact_episodes():
    result = simulate({"gap_d": 5, "rel_v": 2, "cut_in_t": 1.5})
    for shown in result["systems"].values():
        contact = shown["first_collision_t"]
        assert contact is not None
        assert shown["summary"]["metrics_until_t"] == contact
        assert shown["braking"]["count"][-1] == shown["summary"]["brake_count"]
        assert shown["braking"]["duration_s"][-1] == pytest.approx(
            shown["summary"]["brake_duration_s"]
        )
        assert all(e["end_t"] <= contact for e in shown["braking"]["episodes"])
        assert all(e["start_t"] < contact for e in shown["braking"]["episodes"])


def test_five_metre_comparison_cases_reproduce_the_requested_table():
    cases = {
        duration: simulate({"gap_d": 5, "rel_v": 1, "lc_T": duration}) for duration in (3, 4, 5)
    }
    faster = cases[3]["systems"]
    assert all(not side["summary"]["collision"] for side in faster.values())
    assert faster["reactive"]["summary"]["brake_count"] < faster["cv"]["summary"]["brake_count"]
    medium = cases[4]["systems"]
    assert medium["cv"]["summary"]["min_dist"] > medium["reactive"]["summary"]["min_dist"]
    assert (
        medium["cv"]["summary"]["brake_duration_s"]
        < medium["reactive"]["summary"]["brake_duration_s"]
    )
    slow = cases[5]["systems"]
    assert slow["reactive"]["summary"]["collision"]
    assert not slow["cv"]["summary"]["collision"]

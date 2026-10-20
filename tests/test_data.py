"""M1 tests — scenarios and the toy-trajectory loader. Owner: A.

Run: pytest tests/test_data.py -q      (or just `pytest -q`)
"""

import numpy as np
import pytest

from pbs.data import (
    CANONICAL,
    DEFAULT_GRID,
    DT,
    KINDS,
    LANE_WIDTH,
    canonical,
    load_toy,
    load_toy_ego,
    make_scenario,
    min_lc_duration,
    peak_lateral_acc,
    quintic_lateral,
    quintic_lateral_acc,
    quintic_lateral_vel,
    scenario_label,
    scenario_set,
)
from pbs.eval.counterfactual import is_benign
from pbs.sim.collision import R_SAFE
from pbs.types import Scenario, Trajectory


# --------------------------------------------------------------------------------------
# quintic lateral profile
# --------------------------------------------------------------------------------------
def test_quintic_hits_endpoints_and_is_flat_outside():
    t = np.array([-1.0, 0.0, 0.5, 1.5, 2.0, 3.0, 4.0])
    y = quintic_lateral(3.5, 0.0, 2.0, t, t0=1.0)
    assert y[0] == pytest.approx(3.5)  # before the lane change
    assert y[1] == pytest.approx(3.5)
    assert y[2] == pytest.approx(3.5)  # t0 has not been reached yet (t = 0.5 < 1)
    assert y[3] == pytest.approx(3.5 * (1 - 0.10352), abs=1e-4)  # quarter of the way
    assert y[4] == pytest.approx(1.75)  # τ = 0.5, the inflection point
    assert y[5] == pytest.approx(0.0)  # exactly at t0 + T
    assert y[6] == pytest.approx(0.0)  # after the lane change


def test_quintic_is_monotone_and_antisymmetric():
    t = np.linspace(0.0, 3.0, 301)
    y = quintic_lateral(LANE_WIDTH, 0.0, 3.0, t)
    assert np.all(np.diff(y) <= 1e-12)  # never moves away from the target lane
    assert quintic_lateral(0.0, 1.0, 3.0, np.array([1.5]))[0] == pytest.approx(0.5)


def test_quintic_is_c2_at_the_ends():
    T = 3.0
    edges = np.array([0.0, T])
    assert np.allclose(quintic_lateral_vel(LANE_WIDTH, 0.0, T, edges), 0.0, atol=1e-12)
    assert np.allclose(quintic_lateral_acc(LANE_WIDTH, 0.0, T, edges), 0.0, atol=1e-12)


def test_peak_lateral_acc_matches_the_sampled_profile():
    dy, T = LANE_WIDTH, 3.0
    t = np.linspace(0.0, T, 20001)
    sampled = np.abs(quintic_lateral_acc(0.0, dy, T, t)).max()
    assert sampled == pytest.approx(peak_lateral_acc(dy, T), rel=1e-4)
    assert peak_lateral_acc(dy, T) == pytest.approx(10 / np.sqrt(3) * dy / T**2)


def test_min_lc_duration_is_the_inverse_of_the_peak():
    T = min_lc_duration(LANE_WIDTH, 3.0)
    assert peak_lateral_acc(LANE_WIDTH, T) == pytest.approx(3.0, rel=1e-9)
    with pytest.raises(ValueError):
        peak_lateral_acc(LANE_WIDTH, 0.0)


# --------------------------------------------------------------------------------------
# make_scenario: structure, units, ids, params
# --------------------------------------------------------------------------------------
def test_unknown_kind_is_rejected():
    with pytest.raises(ValueError, match="unknown scenario kind"):
        make_scenario("overtake")


@pytest.mark.parametrize("kind", KINDS)
def test_every_kind_builds_a_consistent_scenario(kind):
    sc = make_scenario(
        kind,
        **{name: values[0] for name, values in DEFAULT_GRID[kind].items()},
    )
    assert isinstance(sc, Scenario)
    assert sc.kind == kind
    assert sc.dt == pytest.approx(DT)
    assert sc.lane_width == pytest.approx(LANE_WIDTH)
    assert sc.ego_start == (0.0, 0.0, 0.0, sc.params["v_ego"])
    assert sc.ego_path.ndim == 2 and sc.ego_path.shape[1] == 2
    assert sc.ego_path[-1, 0] > sc.params["v_ego"] * sc.params["duration"]
    assert len(sc.agents) == 1
    assert sc.params["duration"] == pytest.approx(15.0)

    agent = sc.agents[0]
    assert isinstance(agent, Trajectory)
    assert agent.agent_id and isinstance(agent.agent_id, str)
    assert agent.t[0] == pytest.approx(0.0)
    assert agent.t[-1] == pytest.approx(sc.params["duration"])
    assert np.all(np.diff(agent.t) == pytest.approx(DT))  # uniform 10 Hz sampling
    for array in (agent.x, agent.y, agent.v, agent.heading):
        assert array.shape == agent.t.shape
        assert np.all(np.isfinite(array))
    assert np.all(agent.v >= 0.0)
    assert np.all(agent.v > 0.0)  # nothing is required to stop in the default grid


def test_cut_in_geometry_starts_in_the_adjacent_lane_and_merges():
    sc = make_scenario("cut_in", gap_d=25.0, rel_v=3.0, cut_in_t=0.5, lc_T=3.0)
    agent = sc.agents[0]
    assert agent.agent_id == "cutter"
    assert agent.y[0] == pytest.approx(LANE_WIDTH)  # adjacent lane centre
    assert agent.y[-1] == pytest.approx(0.0)  # ego lane centre
    assert agent.x[0] == pytest.approx(25.0)  # gap_d is measured at t = 0
    # lateral motion only inside [cut_in_t, cut_in_t + lc_T]
    start = int(round(0.5 / DT))
    end = int(round(3.5 / DT))
    assert np.allclose(agent.y[: start + 1], LANE_WIDTH)
    assert np.allclose(agent.y[end:], 0.0)
    # the cutter keeps ego − rel_v along x: purely lateral conflict
    assert agent.v[0] == pytest.approx(15.0 - 3.0)
    assert np.allclose(np.diff(agent.x), (15.0 - 3.0) * DT)


def test_cut_in_rejects_physics_beyond_the_lateral_acceleration_bound():
    with pytest.raises(ValueError, match="peak lateral acceleration"):
        make_scenario("cut_in", gap_d=25.0, rel_v=2.0, cut_in_t=0.5, lc_T=1.5)
    # the same lane change is fine once the bound is relaxed explicitly
    sc = make_scenario(
        "cut_in", gap_d=25.0, rel_v=2.0, cut_in_t=0.5, lc_T=2.0, max_lat_acc=6.0
    )
    assert sc.params["peak_lat_acc"] > 3.0


def test_cut_in_rejects_a_reversing_cutter():
    with pytest.raises(ValueError, match="exceeds v_ego"):
        make_scenario("cut_in", gap_d=25.0, rel_v=20.0)


def test_lead_brake_stops_the_lead_vehicle_at_constant_deceleration():
    sc = make_scenario("lead_brake", init_gap=30.0, decel=4.0, t_brake=1.0)
    agent = sc.agents[0]
    assert agent.agent_id == "lead"
    assert np.allclose(agent.y, 0.0)
    assert agent.x[0] == pytest.approx(30.0)
    assert agent.v[0] == pytest.approx(15.0)
    # constant speed up to t_brake, then −decel until it stops
    k_brake = int(round(1.0 / DT))
    assert np.allclose(agent.v[: k_brake + 1], 15.0)
    decel = -np.diff(agent.v[k_brake:]) / DT
    assert np.all(decel >= -1e-9)
    assert decel.max() == pytest.approx(4.0, abs=1e-6)
    assert agent.v[-1] == pytest.approx(0.0)  # 15 / 4 = 3.75 s < horizon
    assert np.all(np.diff(agent.x) >= 0.0)


def test_crossing_reaches_the_ego_lane_centreline_at_arrival_t():
    sc = make_scenario("crossing", gap_d=35.0, arrival_t=2.3, lateral_v=1.5)
    agent = sc.agents[0]
    assert agent.agent_id == "crosser"
    assert np.allclose(agent.x, 35.0)  # crosses at a fixed longitudinal position
    k = int(round(2.3 / DT))
    assert agent.y[k] == pytest.approx(0.0, abs=1e-9)
    assert agent.y[0] == pytest.approx(1.5 * 2.3)  # starts arrival_t away at lateral_v
    assert agent.v[0] == pytest.approx(1.5)
    assert agent.heading[0] == pytest.approx(-np.pi / 2)  # heading is −y


# --------------------------------------------------------------------------------------
# scenario sets, rejection accounting, benign/dangerous balance
# --------------------------------------------------------------------------------------
def test_default_grid_builds_without_rejections():
    built = scenario_set()
    assert built.n_built == sum(
        int(np.prod([len(v) for v in DEFAULT_GRID[kind].values()])) for kind in KINDS
    )
    assert built.n_rejected == 0
    assert built.rejection_rate == 0.0
    assert set(built.by_kind()) == set(KINDS)


def test_strict_mode_propagates_but_counting_mode_records_rejections():
    grid = {"cut_in": {"gap_d": [25.0], "rel_v": [2.0], "lc_T": [1.5, 3.0]}}
    with pytest.raises(ValueError):
        scenario_set(kinds=("cut_in",), grid=grid, strict=True)
    counted = scenario_set(kinds=("cut_in",), grid=grid, strict=False)
    assert counted.n_built == 1 and counted.n_rejected == 1
    assert counted.rejection_rate == pytest.approx(0.5)
    assert "peak lateral acceleration" in counted.rejected[0]["reason"]
    assert counted.rejection_summary() == {"cut_in": 1}


@pytest.mark.parametrize("kind", KINDS)
def test_every_family_contains_both_dangerous_and_benign_samples(kind):
    """Spec §1.5: a family with no benign member makes the false-brake rate identically 0."""
    scenarios = scenario_set(kinds=(kind,)).by_kind()[kind]
    benign = [sc for sc in scenarios if is_benign(sc, duration=sc.params["duration"])]
    assert benign, f"{kind} has no benign scenario — the false-brake rate would be 0"
    assert len(benign) < len(scenarios), f"{kind} has no dangerous scenario"


@pytest.mark.parametrize("name", sorted(CANONICAL))
def test_canonical_scenarios_are_labelled_and_named_as_documented(name):
    sc = canonical(name)
    assert sc.kind in KINDS
    assert scenario_label(sc).startswith(sc.kind)
    expected_benign = "benign" in name
    assert is_benign(sc, duration=sc.params["duration"]) is expected_benign, name


def test_canonical_lookup_fails_loudly():
    with pytest.raises(KeyError):
        canonical("no_such_scenario")


# --------------------------------------------------------------------------------------
# toy data loader
# --------------------------------------------------------------------------------------
def test_load_toy_excludes_ego_and_uses_the_file_stem_as_id():
    agents = load_toy()
    assert len(agents) == 2
    assert [a.agent_id for a in agents] == ["agent_trajectory_001", "agent_trajectory_002"]
    assert all(a.agent_type != "ego" for a in agents)
    for agent in agents:
        assert np.all(np.diff(agent.t) > 0.0)
        assert np.all(np.isfinite(agent.x)) and np.all(np.isfinite(agent.y))
        assert agent.t[0] == pytest.approx(0.0)
        assert agent.t[-1] == pytest.approx(12.9, abs=1e-6)  # 130 samples at 10 Hz
        assert len(agent.t) == 130


def test_load_toy_ego_returns_the_ego_clip():
    ego = load_toy_ego()
    assert ego is not None
    assert ego.agent_type == "ego"
    assert ego.agent_id == "ego_trajectory"
    assert np.all(ego.v > 0.0)


def test_loader_reports_a_missing_directory(tmp_path):
    import pbs.data.loader as loader_module

    saved = loader_module.DATA_DIR
    loader_module.DATA_DIR = tmp_path
    try:
        with pytest.raises(FileNotFoundError):
            load_toy(tmp_path / "nope")
    finally:
        loader_module.DATA_DIR = saved


def test_contact_distance_is_not_already_violated_at_t_zero():
    """A scenario must not start inside the collision envelope (r_safe)."""
    for scenario in scenario_set().scenarios:
        agent = scenario.agents[0]
        assert np.hypot(agent.x[0], agent.y[0]) > R_SAFE

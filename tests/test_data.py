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
    ca_scenario_set,
    canonical,
    load_toy,
    load_toy_ego,
    make_scenario,
    min_lc_duration,
    peak_lateral_acc,
    prediction_slices,
    quintic_lateral,
    quintic_lateral_acc,
    quintic_lateral_vel,
    scenario_label,
    scenario_set,
)
from pbs.eval.counterfactual import is_benign
from pbs.sim.collision import R_SAFE
from pbs.types import Scenario, Trajectory
from pbs.utils import history_until, state_at


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
    sc = make_scenario("cut_in", gap_d=25.0, rel_v=2.0, cut_in_t=0.5, lc_T=2.0, max_lat_acc=6.0)
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
@pytest.fixture
def course_toy_dir():
    """Optional external course clip; missing files are not a loader regression."""
    from pbs.data.loader import SAMPLE_DIR, _resolve_dir

    try:
        return _resolve_dir(SAMPLE_DIR)
    except FileNotFoundError:
        pytest.skip("Optional course toy CSVs absent; copy them to data/sample_trajectories/")


def test_toy_loader_contract_without_external_course_files(tmp_path):
    """Synthetic CSV unit fixture, independent of downloaded evidence datasets."""
    header = "timestamp,x,y,heading,velocity,acceleration,agent_type\n"
    samples = {
        "ego_trajectory": "0,0,0,0,15,0,ego\n0.1,1.5,0,0,15,0,ego\n",
        "agent_trajectory_001": "0,5,1,0,10,0,vehicle\n0.1,6,1,0,10,0,vehicle\n",
        "agent_trajectory_002": "0,10,-1,0,12,0,vehicle\n0.1,11.2,-1,0,12,0,vehicle\n",
    }
    for name, rows in samples.items():
        (tmp_path / f"{name}.csv").write_text(header + rows, encoding="utf-8")
    agents = load_toy(tmp_path)
    assert [a.agent_id for a in agents] == ["agent_trajectory_001", "agent_trajectory_002"]
    assert all(a.agent_type == "vehicle" and a.accel is None for a in agents)
    np.testing.assert_allclose(agents[0].t, [0, 0.1])
    np.testing.assert_allclose(agents[0].x, [5, 6])
    np.testing.assert_allclose(agents[1].x, [10, 11.2])
    ego = load_toy_ego(tmp_path)
    assert ego.agent_id == "ego_trajectory" and ego.agent_type == "ego"
    np.testing.assert_allclose(ego.x, [0, 1.5])


def test_load_toy_excludes_ego_and_uses_the_file_stem_as_id(course_toy_dir):
    agents = load_toy(course_toy_dir)
    assert len(agents) == 2
    assert [a.agent_id for a in agents] == ["agent_trajectory_001", "agent_trajectory_002"]
    assert all(a.agent_type != "ego" for a in agents)
    for agent in agents:
        assert np.all(np.diff(agent.t) > 0.0)
        assert np.all(np.isfinite(agent.x)) and np.all(np.isfinite(agent.y))
        assert agent.t[0] == pytest.approx(0.0)
        assert agent.t[-1] == pytest.approx(12.9, abs=1e-6)  # 130 samples at 10 Hz
        assert len(agent.t) == 130


def test_load_toy_ego_returns_the_ego_clip(course_toy_dir):
    ego = load_toy_ego(course_toy_dir)
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


# --------------------------------------------------------------------------------------
# ground-truth acceleration and the CA slice (M2 support)
# --------------------------------------------------------------------------------------
def test_ground_truth_acceleration_of_every_family():
    """Synthetic scenes carry analytic (a_x, a_y); predictors may only use it for tests."""
    for scenario in scenario_set().scenarios:
        agent = scenario.agents[0]
        assert agent.accel is not None, scenario.kind
        assert agent.accel.shape == (len(agent.t), 2)
        assert np.all(np.isfinite(agent.accel))

    lead = make_scenario("lead_brake", init_gap=30.0, decel=4.0, t_brake=1.0).agents[0]
    k = int(round(1.0 / DT))
    assert np.allclose(lead.accel[: k + 1, 0], 0.0)
    braking = (lead.t > 1.0) & (lead.v > 0)
    assert np.allclose(lead.accel[braking, 0], -4.0)
    assert np.allclose(lead.accel[lead.v == 0, 0], 0.0)  # no reverse acceleration

    crossing = make_scenario("crossing").agents[0]
    np.testing.assert_allclose(crossing.accel, 0.0)


def test_cut_in_ground_truth_lateral_acceleration_matches_the_quintic():
    agent = make_scenario("cut_in", gap_d=25.0, rel_v=2.0).agents[0]
    expected = quintic_lateral_acc(LANE_WIDTH, 0.0, 3.0, agent.t, t0=0.5)
    np.testing.assert_allclose(agent.accel[:, 1], expected, atol=1e-12)


def test_cut_in_acceleration_is_zero_by_default():
    """Backward compatibility: a_cutter = 0 must reproduce the old constant-speed cutter."""
    sc = make_scenario("cut_in", gap_d=25.0, rel_v=2.0)
    agent = sc.agents[0]
    assert sc.params["a_cutter"] == 0.0
    np.testing.assert_allclose(agent.accel[:, 0], 0.0)
    np.testing.assert_allclose(np.diff(agent.x), (15.0 - 2.0) * DT)


def test_cut_in_longitudinal_acceleration_ramps_inside_the_merge_window():
    sc = make_scenario("cut_in", gap_d=25.0, rel_v=2.0, a_cutter=2.0, accel_T=2.0)
    agent = sc.agents[0]
    v0, t0, a, window = 13.0, 0.5, 2.0, 2.0

    k_in = int(round(1.5 / DT))  # inside the ramp
    k_after = int(round(4.0 / DT))  # after the ramp, lane change finished
    assert agent.accel[k_in, 0] == pytest.approx(a)
    assert agent.accel[k_after, 0] == pytest.approx(0.0)
    assert agent.v[k_after] == pytest.approx(v0 + a * window)

    t_end = 5.0  # after both the ramp and the lane change, so v = v_x
    x_expected = 25.0 + v0 * t_end + 0.5 * a * window**2 + a * window * (t_end - t0 - window)
    assert agent.x[int(round(t_end / DT))] == pytest.approx(x_expected, abs=1e-9)
    assert agent.v[int(round(t_end / DT))] == pytest.approx(v0 + a * window)


def test_cut_in_rejects_reversing_or_unphysical_longitudinal_acceleration():
    with pytest.raises(ValueError, match="reverse"):
        make_scenario("cut_in", rel_v=2.0, a_cutter=-3.0, accel_T=5.0)  # 13 − 15 < 0
    with pytest.raises(ValueError, match="longitudinal bound"):
        make_scenario("cut_in", a_cutter=4.0)


def test_ca_slice_builds_and_keeps_both_classes():
    built = ca_scenario_set()
    assert built.n_built == 12
    assert built.n_rejected == 0
    assert {sc.params["a_cutter"] for sc in built.scenarios} == {-2.0, 0.0, 2.0}
    benign = [sc for sc in built.scenarios if is_benign(sc, duration=sc.params["duration"])]
    assert 0 < len(benign) < len(built.scenarios)
    # Exact split, not just "both classes exist": the handoff doc quotes it, and
    # a_cutter = 0 is the only column that is not uniform (0 → 2 benign / 2 dangerous,
    # −2 → all dangerous, +2 → all benign).
    assert len(benign) == 6
    by_accel: dict[float, list[bool]] = {}
    for sc in built.scenarios:
        by_accel.setdefault(sc.params["a_cutter"], []).append(
            is_benign(sc, duration=sc.params["duration"])
        )
    assert by_accel[-2.0] == [False, False, False, False]
    assert sorted(by_accel[0.0]) == [False, False, True, True]
    assert by_accel[2.0] == [True, True, True, True]


def test_history_slices_carry_ground_truth_acceleration():
    agent = make_scenario("cut_in", a_cutter=2.0).agents[0]
    hist = history_until(agent, 1.0)
    assert hist.accel is not None
    assert hist.accel.shape == (len(hist.t), 2)
    assert hist.t[-1] == pytest.approx(1.0)


# --------------------------------------------------------------------------------------
# prediction benchmark slices
# --------------------------------------------------------------------------------------
def test_prediction_slices_cover_the_clip_without_leakage():
    sc = make_scenario("cut_in", gap_d=25.0, rel_v=2.0, a_cutter=2.0)
    slices = prediction_slices(sc, history=1.0, horizon=2.0, stride=0.5)
    assert len(slices) == 25
    assert slices[0].t0 == pytest.approx(1.0)
    assert slices[-1].t0 == pytest.approx(13.0)

    agent = sc.agents[0]
    for slc in slices:
        assert slc.history.t[-1] == pytest.approx(slc.t0)  # nothing from the future
        assert np.all(slc.history.t <= slc.t0 + 1e-9)
        # fixed-length sliding window [t0 − 1 s, t0], not a growing prefix from t = 0
        assert len(slc.history.t) == 11
        assert slc.history.t[0] == pytest.approx(slc.t0 - 1.0)
        assert slc.truth.shape == (20, 2)
        assert slc.truth_accel is not None and slc.truth_accel.shape == (20, 2)

    probe = slices[3]
    np.testing.assert_allclose(probe.truth[0], state_at(agent, probe.t0 + DT))
    np.testing.assert_allclose(probe.truth[-1], state_at(agent, probe.t0 + 2.0))


def test_prediction_slices_reject_non_step_windows():
    sc = make_scenario("cut_in")
    with pytest.raises(ValueError, match="not a positive multiple"):
        prediction_slices(sc, stride=0.15)
    with pytest.raises(ValueError, match="does not fit"):
        prediction_slices(sc, history=1.0, horizon=20.0)

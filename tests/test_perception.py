"""Geometry, causal tracking and optional simulator integration for 2D LiDAR."""

from dataclasses import replace

import numpy as np
import pytest

from pbs.data.perception_scenarios import adjacent_partial_occlusion, occluded_cut_in
from pbs.data.scenarios import make_scenario
from pbs.data.womd import load_clip
from pbs.perception import (
    DetectionBox,
    Lidar2DSensor,
    LidarBody,
    MultiObjectKalmanTracker,
    PerceptionPipeline,
    cluster_points,
    fit_lshape_boxes,
)
from pbs.prediction import ConstantVelocityPredictor, OraclePredictor
from pbs.risk.assessors import NullAssessor, PredictiveAssessor, RiskAssessor
from pbs.sim.observation import IdentityObservation
from pbs.sim.policy import HysteresisPolicy, NeverBrakePolicy
from pbs.sim.simulator import run_sim
from pbs.types import RiskEstimate


def box(x, y=0, heading=0):
    return DetectionBox(np.array([float(x), float(y)]), 5.0, 2.5, heading, 10)


class RecordingRisk(RiskAssessor):
    def __init__(self, ttc=np.inf):
        self.ttc = ttc
        self.calls = []

    def bind(self, scenario):
        self.calls = []

    def assess(self, ctx, history):
        self.calls.append((ctx.t_now, history))
        return RiskEstimate(ttc=self.ttc)


@pytest.mark.parametrize("heading,expected", [(0, 17.5), (np.pi / 2, 18.75)])
def test_first_beam_intersects_the_surface_not_the_center(heading, expected):
    points = Lidar2DSensor(range_noise_std=0).scan(np.zeros(2), 0, [LidarBody([20, 0], heading)])
    assert points[0] == pytest.approx([expected, 0])


def test_nearest_return_hides_far_body_and_is_independent_of_body_order():
    sensor = Lidar2DSensor(range_noise_std=0)
    near, far = LidarBody([12, 0]), LidarBody([24, 0])
    expected = sensor.scan(np.zeros(2), 0, [near])
    assert np.array_equal(sensor.scan(np.zeros(2), 0, [far, near]), expected)
    assert np.array_equal(sensor.scan(np.zeros(2), 0, [near, far]), expected)


def test_no_hits_are_empty_and_do_not_become_max_range_objects():
    sensor = Lidar2DSensor(max_range=10, range_noise_std=0)
    assert sensor.scan([0, 0], 0, []).shape == (0, 2)
    assert sensor.scan([0, 0], 0, [LidarBody([30, 0])]).shape == (0, 2)


def test_disk_and_sensor_inside_body_have_positive_surface_returns():
    sensor = Lidar2DSensor(range_noise_std=0)
    assert sensor.scan([0, 0], 0, [LidarBody([20, 0], radius=1.25)])[0, 0] == 18.75
    assert sensor.scan([0, 0], 0, [LidarBody([0, 0])])[0, 0] == 2.5


def test_scan_rotates_and_translates_with_world_pose():
    sensor = Lidar2DSensor(range_noise_std=0)
    baseline = sensor.scan([0, 0], 0, [LidarBody([20, 4], 0.3)])
    angle, shift = 1.2, np.array([100, -40])
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    rotated = sensor.scan(shift, angle, [LidarBody(np.array([20, 4]) @ rotation.T + shift, 1.5)])
    assert rotated == pytest.approx(baseline @ rotation.T + shift)


def test_far_body_gets_fewer_points_at_fixed_angular_resolution():
    sensor = Lidar2DSensor(n_beams=360, range_noise_std=0)
    assert len(sensor.scan([0, 0], 0, [LidarBody([60, 0])])) < len(
        sensor.scan([0, 0], 0, [LidarBody([10, 0])])
    )


def test_noise_is_seeded_and_reset_reproduces_an_independent_run():
    sensor = Lidar2DSensor(seed=17, range_noise_std=0.05)
    first = sensor.scan([0, 0], 0, [LidarBody([20, 0])])
    assert not np.array_equal(sensor.scan([0, 0], 0, [LidarBody([20, 0])]), first)
    sensor.reset()
    assert np.array_equal(sensor.scan([0, 0], 0, [LidarBody([20, 0])]), first)


def test_euclidean_components_ignore_input_order_and_reject_isolated_noise():
    points = np.array([[0, 0], [0.4, 0], [0.8, 0], [10, 0], [10, 0.4], [10, 0.8], [30, 9]])
    clusters = cluster_points(points[[6, 2, 5, 0, 4, 1, 3]], 0.5, 3)
    assert sorted(len(c) for c in clusters) == [3, 3]
    assert sorted(c.mean(axis=0)[0] for c in clusters) == pytest.approx([0.4, 10])


def test_lshape_fit_recovers_rotated_visible_edges_without_truth_geometry():
    angle = np.deg2rad(23)
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    local = np.vstack(
        (
            np.column_stack((np.linspace(-2.5, 2.5, 31), np.full(31, -1.25))),
            np.column_stack((np.full(31, -2.5), np.linspace(-1.25, 1.25, 31))),
        )
    )
    fitted = fit_lshape_boxes([local @ rotation.T + [20, 3]])[0]
    assert fitted.center == pytest.approx([20, 3], abs=1e-8)
    assert (fitted.length, fitted.width, fitted.heading) == pytest.approx((5, 2.5, angle))


def test_partial_face_and_sparse_fallback_do_not_invent_hidden_center_or_size():
    points = np.column_stack((np.full(8, 17.5), np.linspace(-1, 1, 8)))
    fitted = fit_lshape_boxes([points])[0]
    assert fitted.center == pytest.approx([17.5, 0])  # visible rear face, not truth x=20
    assert fitted.width == pytest.approx(0, abs=1e-10)
    sparse = np.array([[3, 2], [3.2, 2], [3, 2.4]])
    assert fit_lshape_boxes([sparse])[0].center == pytest.approx(sparse.mean(axis=0))


def test_variance_fit_keeps_noisy_nonuniform_lshape_aligned_with_visible_faces():
    sensor = Lidar2DSensor(range_noise_std=0.02, seed=7)
    points = sensor.scan([0, 0], 0, [LidarBody([14, 3.5])])
    fitted = fit_lshape_boxes([points])[0]
    assert abs(np.sin(fitted.heading)) < 0.1
    assert fitted.center == pytest.approx([14, 3.5], abs=0.3)


def test_kalman_estimates_velocity_from_positions_and_preserves_causal_history():
    tracker = MultiObjectKalmanTracker(min_hits=1)
    first = tracker.update([box(20)], 0)[0]
    assert first.history.v[0] == 0  # no truth speed at first detection
    for t in np.arange(0.1, 2.1, 0.1):
        current = tracker.update([box(20 + 10 * t)], float(t))[0]
    assert current.history.v[-1] == pytest.approx(10, abs=0.1)
    assert current.history.x[-1] == pytest.approx(40, abs=0.02)
    assert current.history.agent_id == first.history.agent_id == "lidar_1"
    assert np.array_equal(first.history.t, [0])
    assert np.array_equal(first.history.x, [20])
    assert np.linalg.eigvalsh(current.position_covariance).min() >= 0
    assert current.history.accel is None


def test_track_association_is_one_to_one_when_detection_order_changes():
    tracker = MultiObjectKalmanTracker(min_hits=1)
    first = tracker.update([box(10, -4), box(10, 4)], 0)
    next_frame = tracker.update([box(11, 4), box(11, -4)], 0.1)
    assert [t.history.agent_id for t in next_frame] == [t.history.agent_id for t in first]
    assert [t.history.y[-1] for t in next_frame] == pytest.approx([-4, 4])


def test_dropout_coasts_briefly_then_retires_and_reacquisition_has_a_new_id():
    tracker = MultiObjectKalmanTracker(max_missed=2, min_hits=1)
    tracker.update([box(0)], 0)
    tracker.update([box(1)], 0.1)
    coasting = tracker.update([], 0.2)[0]
    assert not coasting.observed
    assert coasting.history.x[-1] > coasting.history.x[-2]
    assert tracker.update([], 0.3)
    assert tracker.update([], 0.4) == []
    assert tracker.update([box(5)], 0.5)[0].history.agent_id == "lidar_2"


def test_default_confirmation_requires_two_actual_detections_not_coasted_positions():
    tracker = MultiObjectKalmanTracker()
    assert tracker.update([box(20)], 0) == []
    assert tracker.update([], 0.1) == []
    confirmed = tracker.update([box(22)], 0.2)[0]
    assert confirmed.history.v[-1] > 8
    assert confirmed.observed
    assert confirmed.history.t[-1] == 0.2


def test_saved_track_histories_are_isolated_and_window_is_bounded():
    tracker = MultiObjectKalmanTracker(history_length=3, min_hits=1)
    saved = tracker.update([box(0)], 0)[0]
    saved.history.x[:] = 1000
    for i in range(1, 6):
        current = tracker.update([box(i)], i * 0.1)[0]
    assert len(current.history.t) == 3
    assert current.history.x[-1] == pytest.approx(5, abs=0.05)
    tracker.reset()
    assert tracker.update([box(0)], 0)[0].history.agent_id == "lidar_1"


@pytest.mark.parametrize(
    "build",
    [
        lambda: Lidar2DSensor(n_beams=True),
        lambda: Lidar2DSensor(max_range=-1),
        lambda: Lidar2DSensor(range_noise_std=np.nan),
        lambda: LidarBody([np.nan, 0]),
        lambda: cluster_points(np.empty((0, 2)), min_points=0),
        lambda: fit_lshape_boxes([], angle_step_deg=0),
        lambda: MultiObjectKalmanTracker(max_missed=-1),
        lambda: MultiObjectKalmanTracker(measurement_std=0),
    ],
)
def test_invalid_configuration_is_rejected(build):
    with pytest.raises(ValueError):
        build()


def test_pipeline_rejects_duplicate_time_before_advancing_sensor_randomness():
    pipeline = PerceptionPipeline()
    pipeline.observe(t_now=0, ego_xy=[0, 0], ego_heading=0, current_agent_states=[])
    with pytest.raises(ValueError, match="increasing"):
        pipeline.observe(t_now=0, ego_xy=[0, 0], ego_heading=0, current_agent_states=[])


def test_adjacent_lane_blocker_fully_hides_initial_cutter_then_it_emerges():
    scene = occluded_cut_in()
    sensor = Lidar2DSensor(range_noise_std=0)
    bodies = [LidarBody([a.x[0], a.y[0]], a.heading[0]) for a in scene.agents]
    assert np.array_equal(sensor.scan([0, 0], 0, bodies), sensor.scan([0, 0], 0, bodies[1:]))
    pipeline = PerceptionPipeline(sensor=sensor, record_frames=True)
    run_sim(scene, NullAssessor(), NeverBrakePolicy(), duration=3, perception_model=pipeline)
    assert len(pipeline.frames[0].detections) == 1
    assert not pipeline.frames[0].tracks  # first position cannot reveal velocity
    assert len(pipeline.frames[1].tracks) == 1
    assert any(len(frame.tracks) >= 2 for frame in pipeline.frames[1:])
    assert scene.agents[1].y.min() - 1.25 > 1.25  # blocker never overlaps ego laterally


def test_straight_occlusion_fixture_has_no_true_lateral_motion():
    scene = adjacent_partial_occlusion()
    assert np.all(np.diff(scene.agents[0].y) == 0)
    assert np.all(scene.agents[0].heading == 0)


def test_invisible_objects_are_not_assessed_but_truth_collision_still_counts():
    scene = make_scenario("lead_brake", init_gap=20, rel_v=15, duration=2)
    assessor = RecordingRisk(ttc=0)
    log = run_sim(
        scene,
        assessor,
        HysteresisPolicy(),
        duration=2,
        perception_model=PerceptionPipeline(min_points=10000),
    )
    assert assessor.calls == []
    assert log.collision
    assert all(action == "cruise" for action in log.action)


def test_scans_use_actual_braking_ego_pose_and_never_reuse_truth_agent_id():
    scene = make_scenario("cut_in", duration=4)
    assessor, pipeline = RecordingRisk(ttc=0), PerceptionPipeline(record_frames=True)
    log = run_sim(scene, assessor, HysteresisPolicy(), duration=3, perception_model=pipeline)
    assert np.array([frame.ego_xy for frame in pipeline.frames]) == pytest.approx(
        np.column_stack((log.ego_x, log.ego_y))
    )
    assert log.ego_x[-1] < scene.ego_start[3] * log.t[-1]
    truth_ids = {agent.agent_id for agent in scene.agents}
    assert assessor.calls and all(h.agent_id not in truth_ids for _, h in assessor.calls)
    assert all(h.t[-1] <= t for t, h in assessor.calls)


def test_latency_delivers_old_scans_with_original_viewpoint_and_empty_startup():
    scene = make_scenario("cut_in", duration=2)
    assessor, pipeline = RecordingRisk(ttc=0), PerceptionPipeline(record_frames=True)
    run_sim(scene, assessor, HysteresisPolicy(), duration=1, latency=0.2, perception_model=pipeline)
    assert min(t for t, _ in assessor.calls) == pytest.approx(0.3)
    assert all(h.t[-1] == pytest.approx(t - 0.2) for t, h in assessor.calls)
    assert pipeline.frames[0].ego_xy == pytest.approx([0, 0])
    assert pipeline.frames[2].ego_xy == pytest.approx([3, 0])


def test_reset_and_unmodified_future_truth_give_reproducible_causal_scans():
    scene = occluded_cut_in()
    pipeline = PerceptionPipeline(record_frames=True)
    run_sim(scene, NullAssessor(), NeverBrakePolicy(), duration=1, perception_model=pipeline)
    first = [frame.points.copy() for frame in pipeline.frames]
    changed = replace(scene, agents=[replace(a, x=a.x.copy(), y=a.y.copy()) for a in scene.agents])
    for agent in changed.agents:
        agent.x[agent.t > 1] += 100
    run_sim(changed, NullAssessor(), NeverBrakePolicy(), duration=1, perception_model=pipeline)
    assert len(first) == len(pipeline.frames)
    assert all(
        np.array_equal(points, frame.points)
        for points, frame in zip(first, pipeline.frames, strict=True)
    )


def test_default_and_identity_observation_remain_identical():
    scene = make_scenario("cut_in", gap_d=14, rel_v=4, duration=4)
    left = run_sim(
        scene, PredictiveAssessor(ConstantVelocityPredictor()), HysteresisPolicy(), duration=4
    )
    right = run_sim(
        scene,
        PredictiveAssessor(ConstantVelocityPredictor()),
        HysteresisPolicy(),
        duration=4,
        observation_model=IdentityObservation(),
        perception_model=None,
    )
    for key in ("t", "ego_x", "ego_y", "ego_v", "ego_a", "ttc", "action"):
        assert np.array_equal(getattr(left, key), getattr(right, key))
    assert left.collision == right.collision


def test_conflicting_observation_modes_and_truth_oracle_are_explicit_errors():
    scene = make_scenario("cut_in")
    with pytest.raises(ValueError, match="either"):
        run_sim(
            scene,
            NullAssessor(),
            NeverBrakePolicy(),
            observation_model=IdentityObservation(),
            perception_model=PerceptionPipeline(),
        )
    with pytest.raises(ValueError, match="Oracle"):
        run_sim(
            scene,
            PredictiveAssessor(OraclePredictor()),
            NeverBrakePolicy(),
            perception_model=PerceptionPipeline(),
        )


def test_real_waymo_snapshot_runs_through_the_same_closed_loop_pipeline():
    scene = load_clip("tests/fixtures/cases/womd_dangerous_6e86a639df5d4fa1_track1782.npz")
    pipeline = PerceptionPipeline(record_frames=True)
    log = run_sim(
        scene,
        PredictiveAssessor(ConstantVelocityPredictor()),
        HysteresisPolicy(),
        duration=1,
        perception_model=pipeline,
    )
    assert len(pipeline.frames) == len(log.t)
    assert any(frame.tracks for frame in pipeline.frames)
    assert np.isfinite(log.ego_x).all()

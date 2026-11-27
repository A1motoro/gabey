"""Scene scan -> points -> detections -> tracks, with optional diagnostic frames."""

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from pbs.perception.detection import DetectionBox, cluster_points, fit_lshape_boxes
from pbs.perception.sensor import Lidar2DSensor, LidarBody
from pbs.perception.tracking import MultiObjectKalmanTracker, TrackedObject
from pbs.types import Trajectory


@dataclass(frozen=True)
class PerceptionFrame:
    t: float
    ego_xy: np.ndarray
    ego_heading: float
    points: np.ndarray
    detections: list[DetectionBox]
    tracks: list[TrackedObject]


class PerceptionPipeline:
    def __init__(
        self,
        sensor: Lidar2DSensor | None = None,
        tracker: MultiObjectKalmanTracker | None = None,
        *,
        cluster_distance: float = 1.2,
        min_points: int = 3,
        angle_step_deg: float = 1.0,
        record_frames: bool = False,
    ):
        # Validate even when the first scan contains no hits.
        cluster_points(np.empty((0, 2)), cluster_distance, min_points)
        fit_lshape_boxes([], angle_step_deg)
        self.sensor = sensor if sensor is not None else Lidar2DSensor()
        self.tracker = tracker if tracker is not None else MultiObjectKalmanTracker()
        self.cluster_distance = cluster_distance
        self.min_points = min_points
        self.angle_step_deg = angle_step_deg
        self.record_frames = record_frames
        self.reset()

    def reset(self) -> None:
        self.sensor.reset()
        self.tracker.reset()
        self.last_frame: PerceptionFrame | None = None
        self.frames: list[PerceptionFrame] = []

    def observe(
        self,
        *,
        t_now: float,
        ego_xy: np.ndarray,
        ego_heading: float,
        current_agent_states: Sequence[LidarBody],
    ) -> list[Trajectory]:
        """Only the sensor consumes footprints; later layers receive points only."""
        if not np.isfinite(t_now) or (self.last_frame is not None and t_now <= self.last_frame.t):
            raise ValueError("pipeline timestamps must be finite and strictly increasing")
        points = self.sensor.scan(ego_xy, ego_heading, current_agent_states)
        clusters = cluster_points(points, self.cluster_distance, self.min_points)
        detections = fit_lshape_boxes(clusters, self.angle_step_deg)
        tracks = self.tracker.update(detections, t_now)
        self.last_frame = PerceptionFrame(
            float(t_now),
            np.array(ego_xy, dtype=float).copy(),
            float(ego_heading),
            points,
            detections,
            tracks,
        )
        if self.record_frames:
            self.frames.append(self.last_frame)
        return [track.history for track in tracks]

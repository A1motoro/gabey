"""Constant-velocity Kalman tracks with gated, one-to-one nearest association."""

from dataclasses import dataclass, field
from numbers import Integral

import numpy as np

from pbs.perception.detection import DetectionBox
from pbs.types import Trajectory


@dataclass(frozen=True)
class TrackedObject:
    history: Trajectory
    position_covariance: np.ndarray
    observed: bool  # False: a short, explicitly predicted dropout bridge
    box: DetectionBox


@dataclass
class _Track:
    track_id: str
    state: np.ndarray
    covariance: np.ndarray
    box: DetectionBox
    missed: int = 0
    hits: int = 1
    samples: list[tuple] = field(default_factory=list)


class MultiObjectKalmanTracker:
    """Track positions without scene IDs, truth speeds or future samples.

    Each scan adds a causal filtered sample; old samples are never smoothed with
    future data. Tracks coast for max_missed scans, then disappear. Geometric
    heading has a pi ambiguity; motion supplies direction once speed >= 0.5 m/s.
    """

    def __init__(
        self,
        association_gate: float = 4.0,
        measurement_std: float = 0.2,
        acceleration_std: float = 2.0,
        initial_velocity_std: float = 10.0,
        max_missed: int = 3,
        history_length: int = 30,
        min_hits: int = 2,
    ):
        values = [association_gate, measurement_std, acceleration_std, initial_velocity_std]
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError("tracker scales must be finite and positive")
        for name, value, minimum in (
            ("max_missed", max_missed, 0),
            ("history_length", history_length, 2),
            ("min_hits", min_hits, 1),
        ):
            if isinstance(value, bool) or not isinstance(value, Integral) or value < minimum:
                raise ValueError(f"{name} must be an integer >= {minimum}")
        self.association_gate = association_gate
        self.measurement_std = measurement_std
        self.acceleration_std = acceleration_std
        self.initial_velocity_std = initial_velocity_std
        self.max_missed = max_missed
        self.history_length = history_length
        self.min_hits = min_hits
        self.reset()

    def reset(self) -> None:
        self._tracks: list[_Track] = []
        self._last_t: float | None = None
        self._next_id = 1

    def update(self, detections: list[DetectionBox], t_now: float) -> list[TrackedObject]:
        if not np.isfinite(t_now) or (self._last_t is not None and t_now <= self._last_t):
            raise ValueError("tracker timestamps must be finite and strictly increasing")
        for box in detections:
            if np.shape(box.center) != (2,) or not np.isfinite(box.center).all():
                raise ValueError("detection center must be a finite (2,) position")
            if not np.isfinite(box.heading):
                raise ValueError("detection heading must be finite")
        dt = 0.0 if self._last_t is None else float(t_now - self._last_t)
        transition = np.eye(4)
        transition[:2, 2:] = np.eye(2) * dt
        injection = np.vstack((np.eye(2) * dt * dt / 2, np.eye(2) * dt))
        process_cov = self.acceleration_std**2 * injection @ injection.T
        for track in self._tracks:
            track.state = transition @ track.state
            track.covariance = transition @ track.covariance @ transition.T + process_cov
            track.missed += 1

        # Sort all admissible distances, then consume each track/detection once.
        candidates = sorted(
            (float(np.linalg.norm(track.state[:2] - box.center)), i, j)
            for i, track in enumerate(self._tracks)
            for j, box in enumerate(detections)
        )
        used_tracks, used_detections = set(), set()
        observation = np.eye(2, 4)
        noise = np.eye(2) * self.measurement_std**2
        for distance, i, j in candidates:
            if distance > self.association_gate:
                break
            if i in used_tracks or j in used_detections:
                continue
            track, box = self._tracks[i], detections[j]
            innovation_cov = observation @ track.covariance @ observation.T + noise
            gain = np.linalg.solve(innovation_cov, observation @ track.covariance).T
            track.state += gain @ (box.center - observation @ track.state)
            residual = np.eye(4) - gain @ observation
            # Joseph form keeps covariance symmetric positive semidefinite.
            track.covariance = residual @ track.covariance @ residual.T + gain @ noise @ gain.T
            track.box, track.missed = box, 0
            track.hits += 1
            used_tracks.add(i)
            used_detections.add(j)
        for j, box in enumerate(detections):
            if j in used_detections:
                continue
            covariance = np.diag(
                [
                    self.measurement_std**2,
                    self.measurement_std**2,
                    self.initial_velocity_std**2,
                    self.initial_velocity_std**2,
                ]
            )
            self._tracks.append(
                _Track(f"lidar_{self._next_id}", np.r_[box.center, 0.0, 0.0], covariance, box)
            )
            self._next_id += 1
        self._tracks = [track for track in self._tracks if track.missed <= self.max_missed]
        result = []
        for track in self._tracks:
            x, y, vx, vy = track.state
            speed = float(np.hypot(vx, vy))
            heading = float(np.arctan2(vy, vx)) if speed >= 0.5 else track.box.heading
            track.samples.append((t_now, x, y, speed, heading))
            track.samples = track.samples[-self.history_length :]
            # A single position cannot reveal velocity. Confirm with another
            # detection before letting a predictor treat the target as stopped.
            if track.hits < self.min_hits:
                continue
            samples = np.asarray(track.samples)
            # Copies isolate the tracker from consumers and saved earlier frames.
            history = Trajectory(*[samples[:, i].copy() for i in range(5)], agent_id=track.track_id)
            result.append(
                TrackedObject(
                    history, track.covariance[:2, :2].copy(), track.missed == 0, track.box
                )
            )
        self._last_t = float(t_now)
        return result

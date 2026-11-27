"""Planar first-return ray casting. Only this layer sees scene geometry."""

from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Integral

import numpy as np

from pbs.sim.collision import VEHICLE_LENGTH, VEHICLE_WIDTH


@dataclass(frozen=True)
class LidarBody:
    """One current footprint, deliberately without identity or velocity labels.

    A radius selects a disk (e.g. the simulator's pedestrian footprint); otherwise
    length/width describe an oriented rectangle. Coordinates are in world metres.
    """

    center: np.ndarray
    heading: float = 0.0
    length: float = VEHICLE_LENGTH
    width: float = VEHICLE_WIDTH
    radius: float | None = None

    def __post_init__(self):
        center = np.asarray(self.center, dtype=float).copy()
        if center.shape != (2,) or not np.isfinite(center).all():
            raise ValueError("body center must be a finite (2,) position")
        if not np.isfinite([self.heading, self.length, self.width]).all():
            raise ValueError("body geometry must be finite")
        if min(self.length, self.width) <= 0:
            raise ValueError("body dimensions must be positive")
        if self.radius is not None and (not np.isfinite(self.radius) or self.radius <= 0):
            raise ValueError("body radius must be finite and positive")
        center.flags.writeable = False
        object.__setattr__(self, "center", center)


class Lidar2DSensor:
    """Instantaneous 360-degree scan with first returns and range-only noise.

    No object IDs, speed, heading labels or missed-beam endpoints are returned.
    Ego localization is assumed known; scan motion distortion is not modelled.
    """

    def __init__(
        self,
        n_beams: int = 720,
        max_range: float = 80.0,
        range_noise_std: float = 0.02,
        seed: int = 0,
    ):
        if isinstance(n_beams, bool) or not isinstance(n_beams, Integral) or n_beams < 4:
            raise ValueError("n_beams must be an integer >= 4")
        if not np.isfinite([max_range, range_noise_std]).all():
            raise ValueError("sensor parameters must be finite")
        if max_range <= 0 or range_noise_std < 0:
            raise ValueError("range must be positive and noise nonnegative")
        self.n_beams = int(n_beams)
        self.max_range = float(max_range)
        self.range_noise_std = float(range_noise_std)
        self.seed = seed
        self.angles = np.arange(self.n_beams) * (2 * np.pi / self.n_beams)
        self.reset()

    def reset(self) -> None:
        self._rng = np.random.default_rng(self.seed)

    @staticmethod
    def _rectangle_ranges(origin, directions, body):
        c, s = np.cos(body.heading), np.sin(body.heading)
        rotation = np.array([[c, -s], [s, c]])
        local_origin = (origin - body.center) @ rotation
        local_directions = directions @ rotation
        half = np.array([body.length, body.width]) / 2
        parallel = np.abs(local_directions) < 1e-12
        near = np.divide(
            -half - local_origin,
            local_directions,
            out=np.zeros_like(local_directions),
            where=~parallel,
        )
        far = np.divide(
            half - local_origin,
            local_directions,
            out=np.zeros_like(local_directions),
            where=~parallel,
        )
        lower, upper = np.minimum(near, far), np.maximum(near, far)
        outside = np.abs(local_origin) > half
        lower = np.where(parallel, np.where(outside, np.inf, -np.inf), lower)
        upper = np.where(parallel, np.where(outside, -np.inf, np.inf), upper)
        enter, leave = lower.max(axis=1), upper.min(axis=1)
        # If the sensor is inside a footprint, the nearest surface is the exit.
        hit = np.where(enter >= 0, enter, leave)
        return np.where((enter <= leave) & (leave >= 0), hit, np.inf)

    @staticmethod
    def _disk_ranges(origin, directions, body):
        offset = origin - body.center
        b = directions @ offset
        discriminant = b * b - (offset @ offset - body.radius**2)
        root = np.sqrt(np.maximum(discriminant, 0))
        enter, leave = -b - root, -b + root
        return np.where(
            (discriminant >= 0) & (leave >= 0), np.where(enter >= 0, enter, leave), np.inf
        )

    def scan(
        self,
        ego_xy: np.ndarray,
        ego_heading: float,
        bodies: Sequence[LidarBody],
    ) -> np.ndarray:
        """Return an unlabelled (M, 2) point cloud in the world frame."""
        origin = np.asarray(ego_xy, dtype=float)
        if origin.shape != (2,) or not np.isfinite(origin).all() or not np.isfinite(ego_heading):
            raise ValueError("ego pose must be finite with position shape (2,)")
        angles = self.angles + ego_heading
        directions = np.column_stack((np.cos(angles), np.sin(angles)))
        nearest = np.full(self.n_beams, np.inf)
        for body in bodies:
            ranges = (
                self._rectangle_ranges(origin, directions, body)
                if body.radius is None
                else self._disk_ranges(origin, directions, body)
            )
            nearest = np.minimum(nearest, ranges)
        visible = np.isfinite(nearest) & (nearest > 0) & (nearest <= self.max_range)
        distances = nearest[visible]
        if self.range_noise_std:
            distances = distances + self._rng.normal(0, self.range_noise_std, len(distances))
        valid = (distances > 0) & (distances <= self.max_range)
        return origin + distances[valid, None] * directions[visible][valid]

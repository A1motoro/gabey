"""Euclidean components and search-based L-shape enclosing boxes, using points only."""

from dataclasses import dataclass
from numbers import Integral

import numpy as np


def _points(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError("points must be a finite (M, 2) array")
    return points


@dataclass(frozen=True)
class DetectionBox:
    center: np.ndarray
    length: float
    width: float
    heading: float  # undirected geometric axis, not a measured velocity direction
    point_count: int


def cluster_points(
    points: np.ndarray,
    distance_threshold: float = 1.2,
    min_points: int = 3,
) -> list[np.ndarray]:
    """Connected components of the Euclidean neighbour graph (spatial hash).

    Beam order is irrelevant. Nearby touching objects can merge; sparse returns
    can fragment. Discarded tiny components cannot start a track.
    """
    points = _points(points)
    if not np.isfinite(distance_threshold) or distance_threshold <= 0:
        raise ValueError("distance_threshold must be finite and positive")
    if isinstance(min_points, bool) or not isinstance(min_points, Integral) or min_points < 1:
        raise ValueError("min_points must be a positive integer")
    cells = np.floor(points / distance_threshold).astype(np.int64)
    buckets: dict[tuple[int, int], list[int]] = {}
    for i, cell in enumerate(cells):
        buckets.setdefault(tuple(cell), []).append(i)
    visited = np.zeros(len(points), dtype=bool)
    clusters = []
    for start in range(len(points)):
        if visited[start]:
            continue
        visited[start] = True
        component, pending = [], [start]
        while pending:
            i = pending.pop()
            component.append(i)
            cx, cy = cells[i]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    candidates = buckets.get((cx + dx, cy + dy), [])
                    for j in candidates:
                        if (
                            not visited[j]
                            and np.linalg.norm(points[j] - points[i]) <= distance_threshold
                        ):
                            visited[j] = True
                            pending.append(j)
        if len(component) >= min_points:
            clusters.append(points[sorted(component)].copy())
    return clusters


def fit_lshape_boxes(
    clusters: list[np.ndarray],
    angle_step_deg: float = 1.0,
    *,
    criterion: str = "variance",
) -> list[DetectionBox]:
    """Search orientations using the classic L-shape minimum-variance criterion.

    Score the variance of each point's distance to its closer rectangle edge.
    Area breaks ties; criterion='area' selects the minimum-area alternative.
    This fits visible edges, not hidden ones. A one-edge cloud has unobservable
    width and a biased centre. <= 3 points use their centroid.
    """
    if not np.isfinite(angle_step_deg) or not 0 < angle_step_deg <= 45:
        raise ValueError("angle_step_deg must be in (0, 45]")
    if criterion not in ("variance", "area"):
        raise ValueError("criterion must be 'variance' or 'area'")
    angles = np.deg2rad(np.arange(0, 90, angle_step_deg))
    axes = np.column_stack((np.cos(angles), np.sin(angles)))
    across = np.column_stack((-np.sin(angles), np.cos(angles)))
    boxes = []
    for raw in clusters:
        points = _points(raw)
        if not len(points):
            continue
        anchor = points.mean(axis=0)
        local = points - anchor
        u, w = local @ axes.T, local @ across.T
        low_u, high_u = u.min(axis=0), u.max(axis=0)
        low_w, high_w = w.min(axis=0), w.max(axis=0)
        lengths, widths = high_u - low_u, high_w - low_w
        area = lengths * widths
        if criterion == "area" or len(points) <= 3:
            i = int(np.argmin(area))
        else:
            du = np.minimum(u - low_u, high_u - u)
            dw = np.minimum(w - low_w, high_w - w)
            near_u = du <= dw
            score = np.zeros(len(angles))
            for distances, mask in ((du, near_u), (dw, ~near_u)):
                count = mask.sum(axis=0)
                mean = np.divide(
                    (distances * mask).sum(axis=0),
                    count,
                    out=np.zeros(len(angles)),
                    where=count > 0,
                )
                variance = np.divide(
                    ((distances - mean) ** 2 * mask).sum(axis=0),
                    count,
                    out=np.zeros(len(angles)),
                    where=count > 0,
                )
                score += variance
            i = int(np.lexsort((area, score))[0])
        center = anchor + (low_u[i] + high_u[i]) / 2 * axes[i]
        center = center + (low_w[i] + high_w[i]) / 2 * across[i]
        if len(points) <= 3:
            center = anchor
        length, width, heading = lengths[i], widths[i], angles[i]
        if width > length:
            length, width, heading = width, length, heading + np.pi / 2
        boxes.append(
            DetectionBox(center.copy(), float(length), float(width), float(heading), len(points))
        )
    return boxes

"""Owner: P."""

import inspect

import numpy as np

from pbs.sim.collision import (
    PEDESTRIAN_RADIUS,
    VEHICLE_LENGTH,
    VEHICLE_WIDTH,
    overlaps,
    swept_contact_fraction,
    swept_min_distance,
)
from pbs.types import RiskEstimate

_OVERLAPS_HAS_AGENT_DIMS = "agent_length" in inspect.signature(overlaps).parameters
_SWEPT_HAS_AGENT_DIMS = "agent_length" in inspect.signature(swept_contact_fraction).parameters


def forecast_headings(points, origin, initial_heading):
    """Infer directions from predicted displacements, retaining heading at stops."""
    displacements = np.diff(points, axis=0, prepend=np.asarray(origin)[None, :])
    moving = np.linalg.norm(displacements, axis=1) > 1e-9
    directions = np.arctan2(displacements[:, 1], displacements[:, 0])
    if initial_heading is None:
        initial_heading = directions[moving][0] if np.any(moving) else 0.0
    angles = np.r_[initial_heading, directions]
    indices = np.maximum.accumulate(np.where(moving, np.arange(1, len(points) + 1), 0))
    return angles[indices]


def _segment_headings(
    future_heading, origin_heading, n_steps: int
) -> tuple[np.ndarray, np.ndarray]:
    """Return per-segment midpoint headings and step-end headings for swept checks."""
    arr = np.asarray(future_heading, dtype=float)
    if arr.ndim == 0:
        end = np.full(n_steps, float(arr))
        start = float(arr) if origin_heading is None else float(origin_heading)
    else:
        end = arr
        start = float(end[0]) if origin_heading is None else float(origin_heading)
    unwrapped = np.unwrap(np.r_[start, end])
    mid = 0.5 * (unwrapped[:-1] + unwrapped[1:])
    return mid, end


def _is_bool(value) -> bool:
    return isinstance(value, (bool, np.bool_))


def _validate_positive_dim(value: float, name: str) -> float:
    if _is_bool(value):
        raise ValueError(f"{name} must be finite and positive")
    try:
        val = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be finite and positive") from exc
    if not np.isfinite(val) or val <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return val


def _resolve_pad(pad: np.ndarray | float, n_steps: int, name: str) -> np.ndarray:
    if _is_bool(pad):
        raise ValueError(f"{name} must be a non-negative finite scalar or (H,) array")
    arr = np.asarray(pad)
    if arr.dtype == bool:
        raise ValueError(f"{name} must be a non-negative finite scalar or (H,) array")
    arr = np.asarray(arr, dtype=float)
    if arr.ndim == 0:
        val = float(arr)
        if not np.isfinite(val) or val < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return np.full(n_steps, val, dtype=float)
    if arr.shape != (n_steps,) or not np.isfinite(arr).all() or np.any(arr < 0.0):
        raise ValueError(f"{name} must be finite, non-negative, and scalar or shape ({n_steps},)")
    return arr


def _local(delta, heading):
    c, s = np.cos(heading), np.sin(heading)
    x, y = delta[..., 0], delta[..., 1]
    return x * c + y * s, -x * s + y * c


def _projections_sized(
    delta,
    ego_heading,
    agent_heading,
    *,
    vehicle_length: float,
    vehicle_width: float,
    agent_length,
    agent_width,
):
    x, y = delta[..., 0], delta[..., 1]
    angle = np.asarray(agent_heading) - ego_heading
    c, s = np.abs(np.cos(angle)), np.abs(np.sin(angle))
    ego_l, ego_w = vehicle_length / 2.0, vehicle_width / 2.0
    other_l, other_w = np.asarray(agent_length) / 2.0, np.asarray(agent_width) / 2.0
    for heading, length, width, rotated_l, rotated_w in (
        (ego_heading, ego_l, ego_w, other_l, other_w),
        (agent_heading, other_l, other_w, ego_l, ego_w),
    ):
        cx, sy = np.cos(heading), np.sin(heading)
        yield x * cx + y * sy, length + rotated_l * c + rotated_w * s
        yield -x * sy + y * cx, width + rotated_l * s + rotated_w * c


def _disk_entry(start: np.ndarray, end: np.ndarray, radius: float):
    velocity = end - start
    a = float(velocity @ velocity)
    c = float(start @ start) - radius**2
    if c < 0.0:
        return 0.0
    b = float(start @ velocity)
    disc = b * b - a * c
    if a == 0.0 or disc <= 0.0:
        return None
    root = float(np.sqrt(disc))
    entry, exit_time = max(0.0, (-b - root) / a), min(1.0, (-b + root) / a)
    return float(entry) if entry < exit_time else None


def _slab_entry(axes):
    enter, leave = 0.0, 1.0
    for start, end, extent in axes:
        speed = float(end - start)
        if abs(speed) < 1e-14:
            if abs(float(start)) >= float(extent) - 1e-12:
                return None
            continue
        a, b = (-float(extent) - float(start)) / speed, (float(extent) - float(start)) / speed
        enter, leave = max(enter, min(a, b)), min(leave, max(a, b))
        if enter >= leave:
            return None
    return float(enter)


def _overlaps_sized(
    delta,
    ego_heading,
    agent_heading,
    *,
    pedestrian: bool,
    vehicle_length: float,
    vehicle_width: float,
    agent_length,
    agent_width,
    pedestrian_radius,
):
    default_ego = vehicle_length == VEHICLE_LENGTH and vehicle_width == VEHICLE_WIDTH
    if pedestrian:
        same_radius = np.all(np.asarray(pedestrian_radius) == PEDESTRIAN_RADIUS)
        if same_radius and default_ego:
            return overlaps(delta, ego_heading, agent_heading, True)
        if same_radius and _OVERLAPS_HAS_AGENT_DIMS:
            return overlaps(
                delta,
                ego_heading,
                agent_heading,
                True,
                vehicle_length=vehicle_length,
                vehicle_width=vehicle_width,
            )
        x, y = _local(delta, ego_heading)
        dx = np.maximum(np.abs(x) - vehicle_length / 2.0, 0.0)
        dy = np.maximum(np.abs(y) - vehicle_width / 2.0, 0.0)
        return dx * dx + dy * dy < np.asarray(pedestrian_radius) ** 2

    same_agent = np.all(np.asarray(agent_length) == vehicle_length) and np.all(
        np.asarray(agent_width) == vehicle_width
    )
    if default_ego and same_agent:
        return overlaps(delta, ego_heading, agent_heading, False)
    if _OVERLAPS_HAS_AGENT_DIMS:
        return overlaps(
            delta,
            ego_heading,
            agent_heading,
            False,
            vehicle_length=vehicle_length,
            vehicle_width=vehicle_width,
            agent_length=agent_length,
            agent_width=agent_width,
        )
    axes = _projections_sized(
        delta,
        ego_heading,
        agent_heading,
        vehicle_length=vehicle_length,
        vehicle_width=vehicle_width,
        agent_length=agent_length,
        agent_width=agent_width,
    )
    return np.logical_and.reduce([np.abs(offset) < extent - 1e-12 for offset, extent in axes])


def _swept_contact_sized(
    p0,
    p1,
    q0,
    q1,
    *,
    ego_heading: float,
    agent_heading: float,
    pedestrian: bool,
    vehicle_length: float,
    vehicle_width: float,
    agent_length: float,
    agent_width: float,
    pedestrian_radius: float,
):
    default_ego = vehicle_length == VEHICLE_LENGTH and vehicle_width == VEHICLE_WIDTH
    if pedestrian:
        same_radius = pedestrian_radius == PEDESTRIAN_RADIUS
        if same_radius and default_ego:
            return swept_contact_fraction(
                p0,
                p1,
                q0,
                q1,
                None,
                ego_heading=ego_heading,
                agent_heading=agent_heading,
                pedestrian=True,
            )
        if same_radius and _SWEPT_HAS_AGENT_DIMS:
            return swept_contact_fraction(
                p0,
                p1,
                q0,
                q1,
                None,
                ego_heading=ego_heading,
                agent_heading=agent_heading,
                pedestrian=True,
                vehicle_length=vehicle_length,
                vehicle_width=vehicle_width,
            )
        start = np.array(_local(np.asarray(q0) - p0, ego_heading), dtype=float)
        end = np.array(_local(np.asarray(q1) - p1, ego_heading), dtype=float)
        half_l, half_w = vehicle_length / 2.0, vehicle_width / 2.0
        radius = float(pedestrian_radius)
        entries = [
            _slab_entry(zip(start, end, extents, strict=True))
            for extents in ((half_l + radius, half_w), (half_l, half_w + radius))
        ]
        for x in (-half_l, half_l):
            for y in (-half_w, half_w):
                entries.append(
                    _disk_entry(start - np.array([x, y]), end - np.array([x, y]), radius)
                )
        return min((v for v in entries if v is not None), default=None)

    same_agent = agent_length == vehicle_length and agent_width == vehicle_width
    if default_ego and same_agent:
        return swept_contact_fraction(
            p0,
            p1,
            q0,
            q1,
            None,
            ego_heading=ego_heading,
            agent_heading=agent_heading,
            pedestrian=False,
        )
    if _SWEPT_HAS_AGENT_DIMS:
        return swept_contact_fraction(
            p0,
            p1,
            q0,
            q1,
            None,
            ego_heading=ego_heading,
            agent_heading=agent_heading,
            pedestrian=False,
            vehicle_length=vehicle_length,
            vehicle_width=vehicle_width,
            agent_length=agent_length,
            agent_width=agent_width,
        )
    start, end = np.asarray(q0, dtype=float) - p0, np.asarray(q1, dtype=float) - p1
    dims = dict(
        vehicle_length=vehicle_length,
        vehicle_width=vehicle_width,
        agent_length=agent_length,
        agent_width=agent_width,
    )
    starts = _projections_sized(start, ego_heading, agent_heading, **dims)
    ends = _projections_sized(end, ego_heading, agent_heading, **dims)
    return _slab_entry((a, b, extent) for (a, extent), (b, _) in zip(starts, ends, strict=True))


def assess_risk(
    ego_future: np.ndarray,
    agent_future: np.ndarray,
    dt: float,
    r_safe: float | None = None,
    *,
    ego_heading=None,
    agent_heading=None,
    pedestrian=False,
    ego_origin=None,
    agent_origin=None,
    ego_origin_heading: float | None = None,
    agent_origin_heading: float | None = None,
    swept: bool = False,
    vehicle_length: float = VEHICLE_LENGTH,
    vehicle_width: float = VEHICLE_WIDTH,
    agent_length_pad: np.ndarray | float = 0.0,
    agent_width_pad: np.ndarray | float = 0.0,
) -> RiskEstimate:
    """First forecast conflict with strict footprint overlap.

    An explicit ``r_safe`` uses legacy circles. When ``swept=True``, each step
    interval ``[k·dt, (k+1)·dt]`` is swept linearly using ``swept_contact_fraction``
    so fast crossings cannot tunnel between discrete 0.1 s samples; both
    ``ego_origin`` and ``agent_origin`` are required to sweep the first interval.

    Optional ``vehicle_length`` / ``vehicle_width`` select vehicle footprint size
    (defaulting to the L2 5.0 m × 2.5 m baseline). Optional ``agent_length_pad`` /
    ``agent_width_pad`` add total per-step target envelope inflation (e.g. ``2·κ·σ``
    for uncertainty-aware Swept TTC).
    """
    ego = np.asarray(ego_future, dtype=float)
    agent = np.asarray(agent_future, dtype=float)
    if ego.ndim != 2 or ego.shape[1] != 2 or agent.shape != ego.shape:
        raise ValueError("future arrays must have matching (H, 2) shapes")
    if not np.isfinite(ego).all() or not np.isfinite(agent).all():
        raise ValueError("future positions must be finite")
    if _is_bool(dt) or not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and positive")
    if swept and (ego_origin is None or agent_origin is None):
        raise ValueError("swept=True requires both ego_origin and agent_origin")
    vehicle_length = _validate_positive_dim(vehicle_length, "vehicle_length")
    vehicle_width = _validate_positive_dim(vehicle_width, "vehicle_width")
    pad_l = _resolve_pad(agent_length_pad, len(ego), "agent_length_pad")
    pad_w = _resolve_pad(agent_width_pad, len(ego), "agent_width_pad")

    if not len(ego):
        if r_safe is not None and (_is_bool(r_safe) or not np.isfinite(r_safe) or r_safe < 0):
            raise ValueError("r_safe must be finite and non-negative")
        return RiskEstimate()

    if r_safe is not None:
        if _is_bool(r_safe) or not np.isfinite(r_safe) or r_safe < 0:
            raise ValueError("r_safe must be finite and non-negative")
        eff_r = float(r_safe) + 0.5 * np.maximum(pad_l, pad_w)
        if not swept:
            mask = np.linalg.norm(ego - agent, axis=1) < eff_r
            conflicts = np.flatnonzero(mask)
            if not len(conflicts):
                return RiskEstimate()
            k = int(conflicts[0])
            return RiskEstimate(ttc=(k + 1) * dt, conflict_step=k)
        p_start = ego[0] if ego_origin is None else np.asarray(ego_origin, dtype=float)
        q_start = agent[0] if agent_origin is None else np.asarray(agent_origin, dtype=float)
        if (
            p_start.shape != (2,)
            or q_start.shape != (2,)
            or not (np.isfinite(p_start).all() and np.isfinite(q_start).all())
        ):
            raise ValueError("origin positions must be finite (2,) arrays")
        ego_pts = np.vstack((p_start, ego))
        agent_pts = np.vstack((q_start, agent))
        for k in range(len(ego)):
            rk = float(eff_r[k])
            if swept_min_distance(ego_pts[k], ego_pts[k + 1], agent_pts[k], agent_pts[k + 1]) <= rk:
                fraction = swept_contact_fraction(
                    ego_pts[k], ego_pts[k + 1], agent_pts[k], agent_pts[k + 1], rk
                )
                if fraction is not None:
                    return RiskEstimate(ttc=float((k + fraction) * dt), conflict_step=k)
        return RiskEstimate()

    p_start = ego[0] if ego_origin is None else np.asarray(ego_origin, dtype=float)
    q_start = agent[0] if agent_origin is None else np.asarray(agent_origin, dtype=float)
    if (
        p_start.shape != (2,)
        or q_start.shape != (2,)
        or not (np.isfinite(p_start).all() and np.isfinite(q_start).all())
    ):
        raise ValueError("origin positions must be finite (2,) arrays")
    if ego_heading is None:
        ego_heading = forecast_headings(ego, p_start, ego_origin_heading)
    if agent_heading is None:
        agent_heading = forecast_headings(agent, q_start, agent_origin_heading)
    for heading in (ego_heading, agent_heading):
        if np.shape(heading) not in ((), (len(ego),)) or not np.isfinite(heading).all():
            raise ValueError("headings must be finite scalars or one angle per forecast step")

    agent_lengths = vehicle_length + pad_l
    agent_widths = vehicle_width + pad_w
    ped_radii = PEDESTRIAN_RADIUS + 0.5 * np.maximum(pad_l, pad_w)

    if not swept:
        mask = _overlaps_sized(
            agent - ego,
            ego_heading,
            agent_heading,
            pedestrian=bool(pedestrian),
            vehicle_length=vehicle_length,
            vehicle_width=vehicle_width,
            agent_length=agent_lengths,
            agent_width=agent_widths,
            pedestrian_radius=ped_radii,
        )
        conflicts = np.flatnonzero(mask)
        if not len(conflicts):
            return RiskEstimate()
        k = int(conflicts[0])
        return RiskEstimate(ttc=(k + 1) * dt, conflict_step=k)

    if (
        p_start.shape != (2,)
        or q_start.shape != (2,)
        or not (np.isfinite(p_start).all() and np.isfinite(q_start).all())
    ):
        raise ValueError("origin positions must be finite (2,) arrays")
    for heading in (ego_origin_heading, agent_origin_heading):
        if heading is not None and not np.isfinite(heading):
            raise ValueError("origin headings must be finite scalars")

    ego_pts = np.vstack((p_start, ego))
    agent_pts = np.vstack((q_start, agent))
    ego_mid, ego_end = _segment_headings(ego_heading, ego_origin_heading, len(ego))
    agent_mid, agent_end = _segment_headings(agent_heading, agent_origin_heading, len(ego))
    ego_half_diag = 0.5 * float(np.hypot(vehicle_length, vehicle_width))
    is_ped = bool(pedestrian)
    for k in range(len(ego)):
        ag_l = float(agent_lengths[k])
        ag_w = float(agent_widths[k])
        ped_r = float(ped_radii[k])
        broad_limit = ego_half_diag + (ped_r if is_ped else 0.5 * float(np.hypot(ag_l, ag_w)))
        dist = swept_min_distance(ego_pts[k], ego_pts[k + 1], agent_pts[k], agent_pts[k + 1])
        if dist <= broad_limit:
            fraction = _swept_contact_sized(
                ego_pts[k],
                ego_pts[k + 1],
                agent_pts[k],
                agent_pts[k + 1],
                ego_heading=float(ego_mid[k]),
                agent_heading=float(agent_mid[k]),
                pedestrian=is_ped,
                vehicle_length=vehicle_length,
                vehicle_width=vehicle_width,
                agent_length=ag_l,
                agent_width=ag_w,
                pedestrian_radius=ped_r,
            )
            if fraction is not None:
                return RiskEstimate(ttc=float((k + fraction) * dt), conflict_step=k)
            if bool(
                _overlaps_sized(
                    agent[k] - ego[k],
                    float(ego_end[k]),
                    float(agent_end[k]),
                    pedestrian=is_ped,
                    vehicle_length=vehicle_length,
                    vehicle_width=vehicle_width,
                    agent_length=ag_l,
                    agent_width=ag_w,
                    pedestrian_radius=ped_r,
                )
            ):
                return RiskEstimate(ttc=float((k + 1) * dt), conflict_step=k)
    return RiskEstimate()

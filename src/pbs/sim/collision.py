"""Vehicle footprints and swept ground-truth collision. Distances are metres."""

import numpy as np

VEHICLE_LENGTH = 5.0
VEHICLE_WIDTH = 2.5
L3_VEHICLE_LENGTH = 5.0
L3_VEHICLE_WIDTH = 2.0
R_SAFE = 2.5  # Historical centre-distance threshold for explicit circle-mode callers.
# Preserve the original equal-circle radius for pedestrian tracks.
PEDESTRIAN_RADIUS = R_SAFE / 2
HALF_LENGTH = VEHICLE_LENGTH / 2
HALF_WIDTH = VEHICLE_WIDTH / 2


def _projections(
    delta,
    ego_heading,
    agent_heading,
    *,
    vehicle_length=VEHICLE_LENGTH,
    vehicle_width=VEHICLE_WIDTH,
    agent_length=None,
    agent_width=None,
):
    """Centre offsets and combined half-extents on the four separating axes."""
    x, y = delta[..., 0], delta[..., 1]
    angle = np.asarray(agent_heading) - ego_heading
    c, s = np.abs(np.cos(angle)), np.abs(np.sin(angle))
    ego_l, ego_w = vehicle_length / 2, vehicle_width / 2
    other_l = ego_l if agent_length is None else agent_length / 2
    other_w = ego_w if agent_width is None else agent_width / 2
    # On each body's axes, project the other body's rotated half-extents.
    for heading, length, width, rotated_l, rotated_w in (
        (ego_heading, ego_l, ego_w, other_l, other_w),
        (agent_heading, other_l, other_w, ego_l, ego_w),
    ):
        cx, sy = np.cos(heading), np.sin(heading)
        yield x * cx + y * sy, length + rotated_l * c + rotated_w * s
        yield -x * sy + y * cx, width + rotated_l * s + rotated_w * c


def _local(delta, heading):
    c, s = np.cos(heading), np.sin(heading)
    x, y = delta[..., 0], delta[..., 1]
    return x * c + y * s, -x * s + y * c


def overlaps(
    delta,
    ego_heading,
    agent_heading,
    pedestrian=False,
    *,
    vehicle_length=VEHICLE_LENGTH,
    vehicle_width=VEHICLE_WIDTH,
    agent_length=None,
    agent_width=None,
):
    """Strict interior overlap, for a single pose or arrays of poses."""
    if pedestrian:
        x, y = _local(delta, ego_heading)
        dx = np.maximum(np.abs(x) - vehicle_length / 2, 0)
        dy = np.maximum(np.abs(y) - vehicle_width / 2, 0)
        return dx * dx + dy * dy < PEDESTRIAN_RADIUS**2
    axes = _projections(
        delta,
        ego_heading,
        agent_heading,
        vehicle_length=vehicle_length,
        vehicle_width=vehicle_width,
        agent_length=agent_length,
        agent_width=agent_width,
    )
    return np.logical_and.reduce([np.abs(offset) < extent - 1e-12 for offset, extent in axes])


def in_collision(
    p,
    q,
    r_safe=None,
    *,
    ego_heading=0.0,
    agent_heading=0.0,
    pedestrian=False,
    vehicle_length=VEHICLE_LENGTH,
    vehicle_width=VEHICLE_WIDTH,
    agent_length=None,
    agent_width=None,
):
    delta = np.asarray(q, dtype=float) - p
    if r_safe is not None:
        return bool(np.linalg.norm(delta) < r_safe)
    return bool(
        overlaps(
            delta,
            ego_heading,
            agent_heading,
            pedestrian,
            vehicle_length=vehicle_length,
            vehicle_width=vehicle_width,
            agent_length=agent_length,
            agent_width=agent_width,
        )
    )


def swept_min_distance(p0, p1, q0, q1) -> float:
    """Minimum simultaneous centre distance under linear interpolation."""
    relative = np.asarray(p0, dtype=float) - q0
    delta = (np.asarray(p1) - p0) - (np.asarray(q1) - q0)
    squared = float(delta @ delta)
    fraction = 0.0 if squared == 0 else float(np.clip(-(relative @ delta) / squared, 0, 1))
    return float(np.linalg.norm(relative + fraction * delta))


def _disk_entry(start, end, radius):
    velocity = end - start
    a = velocity @ velocity
    c = start @ start - radius**2
    if c < 0:
        return 0.0
    b = start @ velocity
    disc = b * b - a * c
    if a == 0 or disc <= 0:
        return None
    root = np.sqrt(disc)
    entry, exit_time = max(0.0, (-b - root) / a), min(1.0, (-b + root) / a)
    return float(entry) if entry < exit_time else None


def _slab_entry(axes):
    """Clip the relative motion to the open overlap interval on every axis."""
    enter, leave = 0.0, 1.0
    for start, end, extent in axes:
        speed = end - start
        if abs(speed) < 1e-14:
            if abs(start) >= extent - 1e-12:
                return None
            continue
        a, b = (-extent - start) / speed, (extent - start) / speed
        enter, leave = max(enter, min(a, b)), min(leave, max(a, b))
        if enter >= leave:
            return None
    return float(enter)


def swept_contact_fraction(
    p0,
    p1,
    q0,
    q1,
    r_safe=None,
    *,
    ego_heading=0.0,
    agent_heading=0.0,
    pedestrian=False,
    vehicle_length=VEHICLE_LENGTH,
    vehicle_width=VEHICLE_WIDTH,
    agent_length=None,
    agent_width=None,
):
    """First contact leading to overlap; orientation is fixed over this substep."""
    start, end = np.asarray(q0) - p0, np.asarray(q1) - p1
    if r_safe is not None:
        return _disk_entry(start, end, r_safe)
    if pedestrian:
        # The disk centre must enter a rectangle dilated by its radius:
        # two strips and the round corners.
        start, end = np.array(_local(start, ego_heading)), np.array(_local(end, ego_heading))
        half_l, half_w = vehicle_length / 2, vehicle_width / 2
        radius = PEDESTRIAN_RADIUS
        entries = [
            _slab_entry(zip(start, end, extents, strict=True))
            for extents in ((half_l + radius, half_w), (half_l, half_w + radius))
        ]
        for x in (-half_l, half_l):
            for y in (-half_w, half_w):
                entries.append(_disk_entry(start - (x, y), end - (x, y), radius))
        return min((v for v in entries if v is not None), default=None)
    dimensions = dict(
        vehicle_length=vehicle_length,
        vehicle_width=vehicle_width,
        agent_length=agent_length,
        agent_width=agent_width,
    )
    starts = _projections(start, ego_heading, agent_heading, **dimensions)
    ends = _projections(end, ego_heading, agent_heading, **dimensions)
    return _slab_entry((a, b, extent) for (a, extent), (b, _) in zip(starts, ends, strict=True))

"""Vehicle footprints and swept ground-truth collision. Distances are metres."""

import numpy as np

VEHICLE_LENGTH = 5.0
VEHICLE_WIDTH = 2.5
R_SAFE = 2.5  # Historical centre-distance threshold for explicit circle-mode callers.
# Preserve the original equal-circle radius for pedestrian tracks.
PEDESTRIAN_RADIUS = R_SAFE / 2
HALF_LENGTH = VEHICLE_LENGTH / 2
HALF_WIDTH = VEHICLE_WIDTH / 2


def _projections(delta, ego_heading, agent_heading):
    """Centre offsets and combined half-extents on the four separating axes."""
    x, y = delta[..., 0], delta[..., 1]
    angle = np.asarray(agent_heading) - ego_heading
    c, s = np.abs(np.cos(angle)), np.abs(np.sin(angle))
    along = HALF_LENGTH * (1 + c) + HALF_WIDTH * s
    across = HALF_WIDTH * (1 + c) + HALF_LENGTH * s
    for heading in (ego_heading, agent_heading):
        cx, sy = np.cos(heading), np.sin(heading)
        yield x * cx + y * sy, along
        yield -x * sy + y * cx, across


def _local(delta, heading):
    c, s = np.cos(heading), np.sin(heading)
    x, y = delta[..., 0], delta[..., 1]
    return x * c + y * s, -x * s + y * c


def overlaps(delta, ego_heading, agent_heading, pedestrian=False):
    """Strict interior overlap, for a single pose or arrays of poses."""
    if pedestrian:
        x, y = _local(delta, ego_heading)
        dx = np.maximum(np.abs(x) - HALF_LENGTH, 0)
        dy = np.maximum(np.abs(y) - HALF_WIDTH, 0)
        return dx * dx + dy * dy < PEDESTRIAN_RADIUS**2
    return np.logical_and.reduce(
        [
            np.abs(offset) < extent - 1e-12
            for offset, extent in _projections(delta, ego_heading, agent_heading)
        ]
    )


def in_collision(p, q, r_safe=None, *, ego_heading=0.0, agent_heading=0.0, pedestrian=False):
    delta = np.asarray(q, dtype=float) - p
    if r_safe is not None:
        return bool(np.linalg.norm(delta) < r_safe)
    return bool(overlaps(delta, ego_heading, agent_heading, pedestrian))


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
    p0, p1, q0, q1, r_safe=None, *, ego_heading=0.0, agent_heading=0.0, pedestrian=False
):
    """First contact leading to overlap; orientation is fixed over this substep."""
    start, end = np.asarray(q0) - p0, np.asarray(q1) - p1
    if r_safe is not None:
        return _disk_entry(start, end, r_safe)
    if pedestrian:
        # The disk centre must enter a rectangle dilated by its radius:
        # two strips and the round corners.
        start, end = np.array(_local(start, ego_heading)), np.array(_local(end, ego_heading))
        radius = PEDESTRIAN_RADIUS
        entries = [
            _slab_entry(zip(start, end, extents, strict=True))
            for extents in ((HALF_LENGTH + radius, HALF_WIDTH), (HALF_LENGTH, HALF_WIDTH + radius))
        ]
        for x in (-HALF_LENGTH, HALF_LENGTH):
            for y in (-HALF_WIDTH, HALF_WIDTH):
                entries.append(_disk_entry(start - (x, y), end - (x, y), radius))
        return min((v for v in entries if v is not None), default=None)
    starts = _projections(start, ego_heading, agent_heading)
    ends = _projections(end, ego_heading, agent_heading)
    return _slab_entry((a, b, extent) for (a, extent), (b, _) in zip(starts, ends, strict=True))

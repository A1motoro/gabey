"""M2 — Agent-centric trajectory and lane context features for L3 neural prediction. Owner: P.

All features live in the target agent's last-frame local coordinate system:
origin at (x(t0), y(t0)) and +x axis along heading(t0). Rotating or translating a
Scenario by any rigid SE(2) transform leaves both feature vectors invariant.
"""

from __future__ import annotations

from numbers import Integral

import numpy as np

from pbs.prediction._motion import prediction_inputs
from pbs.types import Trajectory

HISTORY_STEPS = 10
TRAJ_STEP_DIM = 6
TRAJ_FEATURE_DIM = HISTORY_STEPS * TRAJ_STEP_DIM
TRAJ_POS_SCALE = 20.0
TRAJ_VEL_SCALE = 20.0
LANE_LOOKAHEAD_M = (5.0, 10.0, 15.0, 20.0, 30.0, 40.0, 50.0)
LANE_FEATURE_DIM = 6 + 2 * len(LANE_LOOKAHEAD_M)
_MIN_SEG_LENGTH = 1e-6


def _is_bool(value) -> bool:
    return isinstance(value, (bool, np.bool_))


def world_to_agent(points: np.ndarray, origin: np.ndarray, heading: float) -> np.ndarray:
    """Transform world (..., 2) positions into the agent frame at (origin, heading)."""
    pts = np.asarray(points, dtype=float)
    orig = np.asarray(origin, dtype=float)
    delta = pts - orig
    c, s = float(np.cos(heading)), float(np.sin(heading))
    local_x = delta[..., 0] * c + delta[..., 1] * s
    local_y = -delta[..., 0] * s + delta[..., 1] * c
    return np.stack((local_x, local_y), axis=-1)


def agent_to_world(points: np.ndarray, origin: np.ndarray, heading: float) -> np.ndarray:
    """Inverse of :func:`world_to_agent`: local (..., 2) positions back to world frame."""
    pts = np.asarray(points, dtype=float)
    orig = np.asarray(origin, dtype=float)
    c, s = float(np.cos(heading)), float(np.sin(heading))
    world_x = orig[0] + pts[..., 0] * c - pts[..., 1] * s
    world_y = orig[1] + pts[..., 0] * s + pts[..., 1] * c
    return np.stack((world_x, world_y), axis=-1)


def _resample_history(
    history: Trajectory, dt: float, steps: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Sample (x, y, v, heading) on ``[t0 - (steps - 1)*dt, ..., t0]``.

    Missing earlier timestamps (before ``history.t[0]``) are padded by backward
    constant-velocity extrapolation from the earliest observed state so short
    histories at simulation start remain physically consistent and SE(2) invariant.
    """
    if _is_bool(steps) or not isinstance(steps, Integral) or steps < 1:
        raise ValueError("steps must be a positive integer")
    t, xy, _ = prediction_inputs(history, 0, dt)
    v = np.asarray(history.v, dtype=float)
    heading = np.asarray(history.heading, dtype=float)

    t0 = float(t[-1])
    rel_t = t - t0
    rel_query = -np.arange(steps - 1, -1, -1, dtype=float) * float(dt)

    if len(t) >= steps and np.allclose(rel_t[-steps:], rel_query, rtol=1e-6, atol=1e-7):
        return (
            xy[-steps:, 0].copy(),
            xy[-steps:, 1].copy(),
            v[-steps:].copy(),
            heading[-steps:].copy(),
        )

    unwrapped = np.unwrap(heading)
    qx = np.interp(rel_query, rel_t, xy[:, 0])
    qy = np.interp(rel_query, rel_t, xy[:, 1])
    qv = np.interp(rel_query, rel_t, v)
    qh = np.interp(rel_query, rel_t, unwrapped)

    before = rel_query < rel_t[0] - 1e-9
    if np.any(before):
        dt_back = rel_query[before] - rel_t[0]
        vx0 = float(v[0] * np.cos(heading[0]))
        vy0 = float(v[0] * np.sin(heading[0]))
        qx[before] = xy[0, 0] + vx0 * dt_back
        qy[before] = xy[0, 1] + vy0 * dt_back
        qv[before] = v[0]
        qh[before] = unwrapped[0]

    return qx, qy, qv, qh


def extract_traj_matrix(
    history: Trajectory,
    dt: float = 0.1,
    steps: int = HISTORY_STEPS,
    *,
    normalize: bool = True,
) -> np.ndarray:
    """Extract agent-centric history tensor of shape ``(steps, 6)`` in float32.

    Columns per frame are ``(dx, dy, vx, vy, cos(dtheta), sin(dtheta))`` in the
    target agent's last-frame coordinate system. When ``normalize=True`` (default,
    matching Option-B spec §3.1), displacements and velocities are scaled by ``20.0``.
    """
    qx, qy, qv, qh = _resample_history(history, dt, steps)
    origin = np.array([qx[-1], qy[-1]], dtype=float)
    theta0 = float(qh[-1])

    local_xy = world_to_agent(np.column_stack((qx, qy)), origin, theta0)
    dtheta = qh - theta0
    cos_d = np.cos(dtheta)
    sin_d = np.sin(dtheta)
    vx = qv * cos_d
    vy = qv * sin_d

    pos_scale = TRAJ_POS_SCALE if normalize else 1.0
    vel_scale = TRAJ_VEL_SCALE if normalize else 1.0

    mat = np.column_stack(
        (
            local_xy[:, 0] / pos_scale,
            local_xy[:, 1] / pos_scale,
            vx / vel_scale,
            vy / vel_scale,
            cos_d,
            sin_d,
        )
    )
    return mat.astype(np.float32)


def extract_traj_features(
    history: Trajectory,
    dt: float = 0.1,
    steps: int = HISTORY_STEPS,
    *,
    normalize: bool = True,
) -> np.ndarray:
    """Extract flattened agent-centric trajectory features of shape ``(steps * 6,)`` (float32)."""
    return extract_traj_matrix(history, dt=dt, steps=steps, normalize=normalize).reshape(-1)


def _clean_polyline(path: np.ndarray):
    """Validate and deduplicate consecutive near-identical vertices (<= 1e-6 m)."""
    arr = np.asarray(path, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 2 or len(arr) < 2 or not np.isfinite(arr).all():
        raise ValueError("ego_path must be a finite (M, 2) array with at least two points")

    kept = [arr[0]]
    for pt in arr[1:]:
        if float(np.linalg.norm(pt - kept[-1])) > _MIN_SEG_LENGTH:
            kept.append(pt)
    if len(kept) < 2:
        raise ValueError("ego_path must contain at least one non-zero length segment")

    clean = np.asarray(kept, dtype=float)
    starts = clean[:-1]
    ends = clean[1:]
    vectors = ends - starts
    lengths = np.linalg.norm(vectors, axis=1)
    tangents = vectors / lengths[:, None]
    cum_s = np.concatenate(([0.0], np.cumsum(lengths)))
    return starts, ends, lengths, cum_s, tangents


def _project_onto_path(path: np.ndarray, point: np.ndarray):
    starts, ends, lengths, cum_s, tangents = _clean_polyline(path)
    vectors = ends - starts
    raw = np.sum((point - starts) * vectors, axis=1) / (lengths**2)
    fractions = np.clip(raw, 0.0, 1.0)
    if len(fractions) == 1:
        fractions = raw
    else:
        fractions[0] = min(raw[0], 1.0)
        fractions[-1] = max(raw[-1], 0.0)
    projected = starts + fractions[:, None] * vectors
    diffs = point - projected
    distances = np.linalg.norm(diffs, axis=1)
    idx = int(np.argmin(distances))
    frac = float(fractions[idx])
    dist = float(distances[idx])
    s0 = float(cum_s[idx] + frac * lengths[idx])

    # On an interior bend vertex (clamped to 0 or 1 between two adjacent segments),
    # use the incident-segment bisector tangent and preserve true Euclidean distance
    # so d_lat never drops the tangential component or jumps discontinuously.
    at_interior_end = (idx < len(lengths) - 1) and (frac >= 1.0 - 1e-12) and (raw[idx] > 1.0)
    at_interior_start = (idx > 0) and (frac <= 1e-12) and (raw[idx] < 0.0)
    if at_interior_end or at_interior_start:
        i_prev = idx if at_interior_end else idx - 1
        i_next = idx + 1 if at_interior_end else idx
        t_sum = tangents[i_prev] + tangents[i_next]
        t_norm = float(np.linalg.norm(t_sum))
        tangent = (t_sum / t_norm) if t_norm > 1e-9 else tangents[idx]
        normal = np.array([-tangent[1], tangent[0]], dtype=float)
        if dist <= 1e-12:
            d_lat = 0.0
        else:
            side = float(diffs[idx] @ normal)
            if abs(side) <= 1e-12:
                # Fallback to segment cross sign if orthogonal to bisector normal.
                n_prev = np.array([-tangents[i_prev, 1], tangents[i_prev, 0]], dtype=float)
                side = float(diffs[idx] @ n_prev)
            d_lat = dist if side >= 0.0 else -dist
    else:
        tangent = tangents[idx]
        normal = np.array([-tangent[1], tangent[0]], dtype=float)
        d_lat = float(diffs[idx] @ normal)

    return s0, d_lat, tangent, starts, ends, lengths, cum_s, tangents


def _sample_path_extended(
    starts: np.ndarray,
    ends: np.ndarray,
    lengths: np.ndarray,
    cum_s: np.ndarray,
    tangents: np.ndarray,
    query_s: np.ndarray,
) -> np.ndarray:
    total_len = float(cum_s[-1])
    out = np.empty((len(query_s), 2), dtype=float)
    for k, s in enumerate(query_s):
        if s < 0.0:
            out[k] = starts[0] + s * tangents[0]
        elif s > total_len:
            out[k] = ends[-1] + (s - total_len) * tangents[-1]
        else:
            idx = int(np.clip(np.searchsorted(cum_s, s, side="right") - 1, 0, len(lengths) - 1))
            out[k] = starts[idx] + (s - cum_s[idx]) * tangents[idx]
    return out


def _validate_lane_width(lane_width: float) -> float:
    if _is_bool(lane_width):
        raise ValueError("lane_width must be finite and positive")
    try:
        lw = float(lane_width)
    except (TypeError, ValueError) as exc:
        raise ValueError("lane_width must be finite and positive") from exc
    if not np.isfinite(lw) or lw <= 0.0:
        raise ValueError("lane_width must be finite and positive")
    return lw


def extract_lane_features(
    history: Trajectory, ego_path: np.ndarray, lane_width: float = 3.5
) -> np.ndarray:
    """Extract 20-D lane context features relative to ``ego_path`` (float32).

    Layout (Contract 1 / Option-B spec §3.1):
    - ``[0]``: signed lateral offset ``d_lat / lane_width`` (left positive)
    - ``[1:3]``: ``(v_lon / 20.0, v_lat / 5.0)`` along/across lane tangent
    - ``[3:5]``: ``(cos(dtheta_lane), sin(dtheta_lane))``
    - ``[5]``: ``lane_width / 3.5``
    - ``[6:20]``: 7 lookahead points on ``ego_path`` at
      ``{5, 10, 15, 20, 30, 40, 50} m`` ahead of projection, expressed in the
      target agent's local frame and divided by ``40.0``.
    """
    prediction_inputs(history, 0, 0.1)
    lw = _validate_lane_width(lane_width)

    p0 = np.array([float(history.x[-1]), float(history.y[-1])], dtype=float)
    v0 = float(history.v[-1])
    theta0 = float(history.heading[-1])

    s0, d_lat, tangent, starts, ends, lengths, cum_s, tangents = _project_onto_path(ego_path, p0)
    normal = np.array([-tangent[1], tangent[0]], dtype=float)
    vel_world = np.array([v0 * np.cos(theta0), v0 * np.sin(theta0)], dtype=float)
    v_lon = float(vel_world @ tangent)
    v_lat = float(vel_world @ normal)
    lane_heading = float(np.arctan2(tangent[1], tangent[0]))
    dtheta_lane = theta0 - lane_heading

    lookahead_s = s0 + np.asarray(LANE_LOOKAHEAD_M, dtype=float)
    lookahead_world = _sample_path_extended(starts, ends, lengths, cum_s, tangents, lookahead_s)
    lookahead_local = world_to_agent(lookahead_world, p0, theta0) / 40.0

    head = np.array(
        [
            d_lat / lw,
            v_lon / 20.0,
            v_lat / 5.0,
            np.cos(dtheta_lane),
            np.sin(dtheta_lane),
            lw / 3.5,
        ],
        dtype=float,
    )
    return np.concatenate((head, lookahead_local.reshape(-1))).astype(np.float32)


def default_lane_features(history: Trajectory, lane_width: float = 3.5) -> np.ndarray:
    """SE(2)-invariant fallback lane features when no scenario ``ego_path`` is bound."""
    prediction_inputs(history, 0, 0.1)
    lw = _validate_lane_width(lane_width)
    v0 = float(history.v[-1])
    lookahead = np.column_stack(
        (np.asarray(LANE_LOOKAHEAD_M, dtype=float) / 40.0, np.zeros(len(LANE_LOOKAHEAD_M)))
    )
    head = np.array([0.0, v0 / 20.0, 0.0, 1.0, 0.0, lw / 3.5], dtype=float)
    return np.concatenate((head, lookahead.reshape(-1))).astype(np.float32)

"""A's L3: WOMD -> fixed-frame replay Scenario. No simulator/model imports.

Raw reading lazily requires ``pip install -e '.[womd]'``. Frozen NPZ clips require
only NumPy. Invalid states stay invalid; the adapter never fills gaps.
"""

from __future__ import annotations

import json
import struct
from functools import lru_cache
from pathlib import Path

import numpy as np

from pbs.types import Scenario, Trajectory

DT = 0.1
VEHICLE = 1
SCHEMA_VERSION = 1
DEFAULT_RAW_DIR = Path(__file__).resolve().parents[3] / "waymo/v1.2/validation"


def shard_paths(path) -> list[Path]:
    """Deterministic file order; partial downloads are not records."""
    path = Path(path)
    if path.is_file():
        return [path]
    paths = sorted(
        p for p in path.glob("*.tfrecord*") if p.is_file() and not p.name.endswith(".part")
    )
    if not paths:
        raise FileNotFoundError(f"No WOMD TFRecord shards: {path}")
    return paths


@lru_cache(maxsize=1)
def scenario_message_class():
    """Load official, packaged descriptors; no generated module path hacks."""
    try:
        from google.protobuf import descriptor_pb2, descriptor_pool, message_factory
    except ImportError as exc:
        raise ImportError("Raw WOMD reading needs: pip install -e '.[womd]'") from exc
    descriptors = descriptor_pb2.FileDescriptorSet.FromString(
        Path(__file__).with_name("womd.desc").read_bytes()
    )
    pool = descriptor_pool.DescriptorPool()
    for descriptor in descriptors.file:  # protoc emits dependencies first
        pool.Add(descriptor)
    return message_factory.GetMessageClass(
        pool.FindMessageTypeByName("waymo.open_dataset.Scenario")
    )


def _masked_crc(data: bytes) -> int:
    try:
        import google_crc32c
    except ImportError as exc:
        raise ImportError("CRC verification needs: pip install -e '.[womd]'") from exc
    crc = google_crc32c.value(data)
    return (((crc >> 15) | (crc << 17)) + 0xA282EAD8) & 0xFFFFFFFF


def iter_tfrecords(path, *, verify_crc: bool = True):
    """Read uncompressed TFRecords with both CRC32C checks and truncation errors."""
    with Path(path).open("rb") as handle:
        while True:
            offset = handle.tell()
            head = handle.read(12)
            if not head:
                return
            if len(head) != 12:
                raise ValueError(f"Truncated TFRecord header at {path}:{offset}")
            length, length_crc = struct.unpack("<QI", head)
            if length > 128 * 1024 * 1024:
                raise ValueError(f"Unreasonable record length at {path}:{offset}")
            if verify_crc and _masked_crc(head[:8]) != length_crc:
                raise ValueError(f"Length CRC32C mismatch at {path}:{offset}")
            payload = handle.read(length)
            tail = handle.read(4)
            if len(payload) != length or len(tail) != 4:
                raise ValueError(f"Truncated TFRecord payload at {path}:{offset}")
            if verify_crc and _masked_crc(payload) != struct.unpack("<I", tail)[0]:
                raise ValueError(f"Payload CRC32C mismatch at {path}:{offset}")
            yield payload


def read_scenarios(path=DEFAULT_RAW_DIR, limit: int | None = None, *, verify_crc=True):
    """Yield official protobuf Scenario messages, with a global optional record limit."""
    if limit is not None and (isinstance(limit, bool) or not isinstance(limit, int) or limit < 0):
        raise ValueError("limit must be a nonnegative integer or None")
    if limit == 0:
        return
    message_class = scenario_message_class()
    count = 0
    for shard in shard_paths(path):
        for payload in iter_tfrecords(shard, verify_crc=verify_crc):
            yield message_class.FromString(payload)
            count += 1
            if limit is not None and count >= limit:
                return


def sdc_index(scene) -> int:
    """SDC is identified by its official index, which is often NOT tracks[0]."""
    if not scene.HasField("sdc_track_index"):
        raise ValueError("Missing sdc_track_index; refusing to guess the ego track")
    index = int(scene.sdc_track_index)
    if not 0 <= index < len(scene.tracks):
        raise ValueError("sdc_track_index is out of range")
    return index


def track_arrays(track):
    """Return xy, body yaw, world velocity and valid; invalid numeric rows are NaN."""
    valid = np.array([s.valid for s in track.states], dtype=bool)
    xy = np.array([(s.center_x, s.center_y) for s in track.states], dtype=float)
    yaw = np.array([s.heading for s in track.states], dtype=float)
    velocity = np.array([(s.velocity_x, s.velocity_y) for s in track.states], dtype=float)
    xy[~valid], yaw[~valid], velocity[~valid] = np.nan, np.nan, np.nan
    return xy, yaw, velocity, valid


def local_xy(xy, origin, yaw):
    """Single fixed SE(2) transform at anchor, left-positive y (not moving ego axes)."""
    c, s = np.cos(yaw), np.sin(yaw)
    return (np.asarray(xy) - origin) @ np.array([[c, -s], [s, c]])


def _time_grid(scene):
    times = np.asarray(scene.timestamps_seconds, dtype=float)
    if times.ndim != 1 or len(times) < 2 or not np.isfinite(times).all():
        raise ValueError("Missing/invalid WOMD timestamps")
    # Recorded clock timestamps have sub-ms jitter; preserve frame samples and
    # snap timestamps only (no trajectory interpolation) to the nominal 10 Hz grid.
    drift = times - times[0] - np.arange(len(times)) * DT
    if not np.allclose(np.diff(times), DT, rtol=0, atol=1e-3) or np.max(np.abs(drift)) > 0.005:
        raise ValueError("Require native uniform 10 Hz WOMD timestamps")
    if any(len(tr.states) != len(times) for tr in scene.tracks):
        raise ValueError("Track/timestamp length mismatch")
    return times


def _window(scene, anchor, duration, history_steps):
    times = _time_grid(scene)
    if not isinstance(anchor, (int, np.integer)) or isinstance(anchor, bool):
        raise ValueError("anchor must be an integer frame index")
    if not isinstance(history_steps, int) or isinstance(history_steps, bool) or history_steps < 20:
        raise ValueError("Require at least 20 history steps")
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be positive")
    ahead = int(round(duration / DT))
    if not np.isclose(ahead * DT, duration, rtol=0, atol=1e-9):
        raise ValueError("duration must be a multiple of 0.1 s")
    if anchor < history_steps or anchor + ahead >= len(times):
        raise ValueError("Insufficient history/future for the requested window")
    return slice(anchor - history_steps, anchor + ahead + 1)


def _ego_window(scene, anchor, window):
    index = sdc_index(scene)
    xy, yaw, velocity, valid = track_arrays(scene.tracks[index])
    if not valid[window].all():
        raise ValueError("Ego window contains invalid states; no interpolation permitted")
    local = local_xy(xy[window], xy[anchor], yaw[anchor])
    speed = np.linalg.norm(velocity[window], axis=1)
    if not np.isfinite(local).all() or not np.isfinite(yaw[window]).all():
        raise ValueError("Nonfinite ego states")
    if speed.min() <= 3:
        raise ValueError("Ego must remain above 3 m/s")
    yaw_rate = np.max(np.abs(np.diff(np.unwrap(yaw[window])))) / DT
    if yaw_rate >= np.deg2rad(5):
        raise ValueError("Ego heading rate exceeds the straight-road filter")
    if np.max(np.abs(local[:, 1])) >= 1:
        raise ValueError("Straight ego approximation deviates by >= 1 m")
    return index, xy, yaw, velocity, local, float(yaw_rate)


def find_candidates(scene, *, duration=6.0, history_steps=20) -> list[dict]:
    """Geometric cohort discovery, never conditioned on system outcomes.

    One best anchor per target/group: prefer centerline entry 2.5 s after anchor.
    Controls remain 2..5 m to one side for the ENTIRE historical+future window.
    Cut-ins enter |lat|<=0.9 m, 5..40 m ahead, from 2..5 m to the same side
    within 5 s, and remain inside |lat|<=1.75 m for >=0.5 s. Parallel heading
    (<=20 degrees) excludes oncoming/crossing traffic. This is a lane surrogate.
    """
    times = _time_grid(scene)
    ahead = int(round(duration / DT))
    ego_index = sdc_index(scene)
    cached = [track_arrays(track) for track in scene.tracks]
    best = {}
    for anchor in range(history_steps, len(times) - ahead):
        window = _window(scene, anchor, duration, history_steps)
        try:
            _, ego_xy, ego_yaw, ego_vel, ego_local, yaw_rate = _ego_window(scene, anchor, window)
        except ValueError:
            continue
        for index, track in enumerate(scene.tracks):
            if index == ego_index or track.object_type != VEHICLE:
                continue
            xy, yaw, velocity, valid = cached[index]
            if not valid[window].all():
                continue
            delta = xy[window] - ego_xy[window]
            c, s = np.cos(ego_yaw[window]), np.sin(ego_yaw[window])
            lon = delta[:, 0] * c + delta[:, 1] * s
            lat = -delta[:, 0] * s + delta[:, 1] * c
            heading_delta = np.arctan2(
                np.sin(yaw[window] - ego_yaw[window]), np.cos(yaw[window] - ego_yaw[window])
            )
            if (
                not np.isfinite(lon).all()
                or not np.isfinite(heading_delta).all()
                or np.max(np.abs(heading_delta)) > np.deg2rad(20)
            ):
                continue
            cross, departure, group = None, None, None
            if (
                np.all((np.abs(lat) >= 2) & (np.abs(lat) <= 5))
                and np.all(np.sign(lat) == np.sign(lat[0]))
                and 5 <= lon[history_steps] <= 40
            ):
                group = "control"
            else:
                entries = np.flatnonzero((np.abs(lat[1:]) <= 0.9) & (np.abs(lat[:-1]) > 0.9)) + 1
                for k in entries:
                    if k <= history_steps or k + 5 >= len(lat) or not 5 <= lon[k] <= 40:
                        continue
                    if not np.all(np.abs(lat[k : k + 6]) <= 1.75):
                        continue
                    prior = np.arange(max(0, k - 50), k)
                    adjacent = prior[
                        (np.abs(lat[prior]) >= 2)
                        & (np.abs(lat[prior]) <= 5)
                        & (np.sign(lat[prior]) == np.sign(lat[k - 1]))
                    ]
                    if not len(adjacent) or abs(lat[history_steps]) < 2:
                        continue
                    cross, departure, group = int(k), int(adjacent[-1]), "cut_in"
                    break
            if group is None:
                continue
            v0 = float(np.linalg.norm(ego_vel[anchor - 1 : anchor + 2], axis=1).mean())
            target_vx = float(
                velocity[anchor] @ np.array([np.cos(ego_yaw[anchor]), np.sin(ego_yaw[anchor])])
            )
            row = {
                "clip": scene.scenario_id,
                "group": group,
                "track_id": int(track.id),
                "track_index": index,
                "sdc_track_index": ego_index,
                "sdc_track_id": int(scene.tracks[ego_index].id),
                "anchor": anchor,
                "t_cross_s": None if cross is None else (cross - history_steps) * DT,
                "lon_m": float(lon[history_steps if cross is None else cross]),
                "lc_T_s": None if cross is None else (cross - departure) * DT,
                "rel_v": v0 - target_vx,
                "v_ego": v0,
                "gap_d": float(lon[history_steps]),
                "lat_anchor_m": float(lat[history_steps]),
                "n_valid_frames": int(valid[window].sum()),
                "ego_max_lat_m": float(np.max(np.abs(ego_local[:, 1]))),
                "ego_max_yaw_rate_deg_s": float(np.rad2deg(yaw_rate)),
                "cross_frame": None if cross is None else window.start + cross,
            }
            score = abs((row["t_cross_s"] or 2.5) - 2.5)
            key = (index, group)
            if key not in best or score < best[key][0]:
                best[key] = score, row
    return [row for _, row in best.values()]


def real_scenario(
    scene, track_index: int, *, anchor=20, duration=6.0, history_steps=20, group="cut_in"
) -> Scenario:
    """Selected-target replay. Ego is simulated on a documented straight surrogate."""
    if group not in ("cut_in", "control"):
        raise ValueError("group must be cut_in or control")
    window = _window(scene, anchor, duration, history_steps)
    ego_index, ego_xy, ego_yaw, ego_vel, ego_local, yaw_rate = _ego_window(scene, anchor, window)
    if (
        not isinstance(track_index, (int, np.integer))
        or isinstance(track_index, bool)
        or not 0 <= track_index < len(scene.tracks)
        or track_index == ego_index
    ):
        raise ValueError("track_index must identify a non-ego track")
    track = scene.tracks[track_index]
    xy, yaw, velocity, valid = track_arrays(track)
    if track.object_type != VEHICLE or not valid[window].all():
        raise ValueError("Require a vehicle with a continuously valid window")
    pos = local_xy(xy[window], ego_xy[anchor], ego_yaw[anchor])
    speed = np.linalg.norm(velocity[window], axis=1)
    # Predictor's v/heading encode velocity direction, not potentially drifting body yaw.
    direction = np.where(
        speed > 0.2, np.arctan2(velocity[window, 1], velocity[window, 0]), yaw[window]
    )
    heading = np.unwrap(direction - ego_yaw[anchor])
    if not np.isfinite(pos).all() or not np.isfinite(speed).all() or not np.isfinite(heading).all():
        raise ValueError("Nonfinite target states")
    v0 = float(np.linalg.norm(ego_vel[anchor - 1 : anchor + 2], axis=1).mean())
    t = np.arange(-history_steps, int(round(duration / DT)) + 1) * DT
    agent = Trajectory(
        t,
        pos[:, 0],
        pos[:, 1],
        speed,
        heading,
        agent_type="vehicle",
        agent_id=f"track{track.id}",
        accel=None,
    )
    params = {
        "duration": duration,
        "v_ego": v0,
        "source": "Waymo-WOMD-v1.2-validation",
        "scenario_id": scene.scenario_id,
        "anchor": int(anchor),
        "group": group,
        "sdc_track_index": ego_index,
        "sdc_track_id": int(scene.tracks[ego_index].id),
        "track_index": int(track_index),
        "track_id": int(track.id),
        "history_steps": history_steps,
        "n_valid_frames": len(t),
        "timestamp_max_grid_error_s": float(
            np.max(np.abs(_time_grid(scene)[window] - scene.timestamps_seconds[anchor] - t))
        ),
        "ego_max_lat_m": float(np.max(np.abs(ego_local[:, 1]))),
        "ego_max_yaw_rate_deg_s": float(np.rad2deg(yaw_rate)),
        "ego_path_mode": "straight_at_anchor",
        "agent_scope": "selected_target_only",
        "gap_d": float(pos[history_steps, 0]),
        "rel_v": v0
        - float(velocity[anchor] @ np.array([np.cos(ego_yaw[anchor]), np.sin(ego_yaw[anchor])])),
    }
    # Diagnostic only: fit local quadratic position windows instead of differencing
    # noisy velocity. This is not ground-truth acceleration or quintic calibration.
    tau = np.arange(-5, 6) * DT
    quadratic = np.column_stack((np.ones(11), tau, tau**2))
    windows = np.lib.stride_tricks.sliding_window_view(pos[:, 1], 11)
    fitted_accel = 2 * (np.linalg.pinv(quadratic) @ windows.T)[2]
    params["peak_lat_acc_est_mps2"] = float(np.max(np.abs(fitted_accel)))
    params["peak_lat_acc_window_frames"] = 11
    return Scenario(
        (0.0, 0.0, 0.0, v0),
        np.array([[0.0, 0.0], [max(200.0, v0 * (duration + 3) + 60), 0.0]]),
        [agent],
        DT,
        params,
        f"real_{group}",
        3.5,
    )


def clip_evidence(scene, scenario) -> dict:
    """Freeze observed ego trajectory and validity witnesses, separate from sim inputs."""
    p = scenario.params
    window = _window(scene, p["anchor"], p["duration"], p["history_steps"])
    xy, yaw, velocity, valid = track_arrays(scene.tracks[sdc_index(scene)])
    return {
        "observed_ego_xy": local_xy(xy[window], xy[p["anchor"]], yaw[p["anchor"]]),
        "observed_ego_v": np.linalg.norm(velocity[window], axis=1),
        "ego_valid": valid[window],
        "agent_valid": track_arrays(scene.tracks[p["track_index"]])[3][window][None, :],
        "observed_timestamps_s": _time_grid(scene)[window] - scene.timestamps_seconds[p["anchor"]],
    }


def save_clip(scenario: Scenario, path, *, evidence: dict | None = None):
    """Portable snapshot; JSON metadata and numeric/Unicode arrays, never pickle."""
    if not scenario.agents:
        raise ValueError("Cannot save an empty clip")
    arrays = {
        key: np.stack([getattr(a, key) for a in scenario.agents])
        for key in ("t", "x", "y", "v", "heading")
    }
    arrays.update(
        {
            "schema_version": SCHEMA_VERSION,
            "ego_start": scenario.ego_start,
            "ego_path": scenario.ego_path,
            "dt": scenario.dt,
            "lane_width": scenario.lane_width,
            "kind": scenario.kind,
            "params_json": json.dumps(scenario.params, allow_nan=False),
            "agent_id": [a.agent_id for a in scenario.agents],
            "agent_type": [a.agent_type for a in scenario.agents],
        }
    )
    if evidence:
        if set(evidence) & set(arrays):
            raise ValueError("Evidence keys cannot overwrite Scenario fields")
        arrays.update(evidence)
    if any(np.asarray(a).dtype.hasobject for a in arrays.values()):
        raise ValueError("Object arrays are forbidden in clip snapshots")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **arrays)


def load_clip(path) -> Scenario:
    """Load/validate an NPZ without importing protobuf or accessing the raw dataset."""
    with np.load(path, allow_pickle=False) as z:
        if int(z["schema_version"]) != SCHEMA_VERSION:
            raise ValueError("Unsupported WOMD clip schema")
        params = json.loads(str(z["params_json"]))
        arrays = [z[key] for key in ("t", "x", "y", "v", "heading")]
        if (
            any(
                a.ndim != 2 or a.shape != arrays[0].shape or not np.isfinite(a).all()
                for a in arrays
            )
            or not arrays[0].size
        ):
            raise ValueError("Clip arrays must be matching, finite, nonempty (agents, time)")
        if not np.allclose(np.diff(arrays[0], axis=1), DT, rtol=0, atol=1e-9):
            raise ValueError("Clip has a time gap or nonuniform 10 Hz sampling")
        ids, types = z["agent_id"].tolist(), z["agent_type"].tolist()
        if (
            len(ids) != len(arrays[0])
            or len(types) != len(ids)
            or len(set(ids)) != len(ids)
            or not all(ids)
        ):
            raise ValueError("Require unique, nonempty agent IDs")
        if f"track{params.get('sdc_track_id')}" in ids:
            raise ValueError("Ego cannot be in Scenario.agents")
        for key in ("ego_valid", "agent_valid"):
            if key in z and not z[key].all():
                raise ValueError("Snapshot contains invalid states")
        if (
            float(z["dt"]) != DT
            or not np.isfinite(params["duration"])
            or params["duration"] <= 0
            or np.any(arrays[3] < 0)
        ):
            raise ValueError("Invalid duration, step or speed")
        if np.any(arrays[0][:, 0] > -2 + 1e-9) or np.any(
            arrays[0][:, -1] < params["duration"] - 1e-9
        ):
            raise ValueError("Clip lacks 2 s prehistory or full future")
        agents = [
            Trajectory(*(a[i].copy() for a in arrays), agent_type=types[i], agent_id=ids[i])
            for i in range(len(ids))
        ]
        start, path_array = z["ego_start"], z["ego_path"]
        if (
            start.shape != (4,)
            or not np.isfinite(start).all()
            or start[3] < 0
            or not np.allclose(start[:3], 0)
            or path_array.ndim != 2
            or path_array.shape[1] != 2
            or len(path_array) < 2
            or not np.isfinite(path_array).all()
            or not np.allclose(path_array[0], 0)
            or np.any(np.linalg.norm(np.diff(path_array, axis=0), axis=1) == 0)
            or path_array[1, 0] <= 0
            or abs(path_array[1, 1]) > 1e-8
            or not np.isfinite(z["lane_width"])
            or float(z["lane_width"]) <= 0
        ):
            raise ValueError("Invalid fixed-frame ego start/path or lane width")
        kind = str(z["kind"])
        if kind not in ("real_cut_in", "real_control") or kind != f"real_{params['group']}":
            raise ValueError("Clip kind/group mismatch")
        return Scenario(
            tuple(start.tolist()),
            path_array.copy(),
            agents,
            float(z["dt"]),
            params,
            kind,
            float(z["lane_width"]),
        )


def load_evidence(path) -> dict:
    with np.load(path, allow_pickle=False) as z:
        return {
            key: z[key].copy()
            for key in (
                "observed_ego_xy",
                "observed_ego_v",
                "ego_valid",
                "agent_valid",
                "observed_timestamps_s",
            )
            if key in z
        }

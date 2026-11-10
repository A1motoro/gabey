"""Compute once in Python; present the actual decisions in any renderer."""

import numpy as np

from pbs.demo.assessors import AssessorFactory, RecordingAssessor, make_demo_assessor
from pbs.demo.scene import DEFAULTS, PRESETS, RANGES, SceneAdapter, SyntheticCutInAdapter
from pbs.eval.counterfactual import is_benign
from pbs.eval.metrics import brake_activity, evaluate
from pbs.sim.collision import R_SAFE
from pbs.sim.policy import HysteresisPolicy
from pbs.sim.simulator import run_sim
from pbs.utils import path_heading, state_at

DURATION = 10.0
HORIZON = 30


def get_config():
    return {
        "defaults": DEFAULTS,
        "ranges": RANGES,
        "presets": PRESETS,
        "fixed": {
            "v_ego": 15,
            "duration": DURATION,
            "dt": 0.1,
            "horizon_s": 3,
            "ttc_threshold": 2,
            "policy": "hysteresis",
            "t_on": 2.0,
            "t_off": 3.0,
            "a_brake": 6,
            "r_safe": R_SAFE,
        },
    }


def _track(traj):
    return {
        "agent_id": traj.agent_id,
        "t": traj.t.tolist(),
        "x": traj.x.tolist(),
        "y": traj.y.tolist(),
        "heading": traj.heading.tolist(),
        "v": traj.v.tolist(),
    }


def simulate(
    config,
    *,
    adapter: SceneAdapter | None = None,
    observation_factory=None,
    computation_delay_factory=None,
    assessor_factory: AssessorFactory = make_demo_assessor,
):
    """A fresh observation model per side; seeded models can supply matched noise.

    Each side receives independent observation and timing models from factories.
    Only zero computation delay is supported; nonzero requests fail explicitly.
    """
    scene = (adapter or SyntheticCutInAdapter()).load(config)
    scenario = scene.scenario
    if not np.isclose(scenario.dt, 0.1):
        raise ValueError("Demo adapters must resample to dt=0.1 s.")
    benign = bool(is_benign(scenario, duration=DURATION))
    result = {
        "version": 1,
        "params": scenario.params,
        "fixed": get_config()["fixed"],
        "source": scene.source,
        "road_lines": (
            [np.asarray(line).tolist() for line in scene.road_lines]
            if scene.road_lines is not None
            else None
        ),
        "ego_path": scenario.ego_path.tolist(),
        "lane_width": scenario.lane_width,
        "ego_reference": _track(scene.ego_reference) if scene.ego_reference else None,
        "agents": [_track(a) for a in scenario.agents],
        "systems": {},
    }
    for name in ("reactive", "cv"):
        bundle = assessor_factory(name)
        recorder = RecordingAssessor(bundle.assessor, bundle.predictor)
        model = observation_factory() if observation_factory else None
        timing = computation_delay_factory() if computation_delay_factory else None
        log = run_sim(
            scenario,
            recorder,
            HysteresisPolicy(2.0, 3.0),
            duration=DURATION,
            horizon=HORIZON,
            observation_model=model,
            computation_delay_model=timing,
        )
        result["t"] = log.t.tolist()
        # Retain per-agent traces so adding a dataset with multiple agents needs no schema change.
        distances = [
            min(
                (float(np.linalg.norm(np.array([x, y]) - state_at(a, t))) for a in scenario.agents),
                default=None,
            )
            for t, x, y in zip(log.t, log.ego_x, log.ego_y, strict=True)
        ]
        headings = [path_heading(scenario.ego_path, 0)] * len(log.t)
        for frame in recorder.frames:
            headings[int(round(frame["t"] / scenario.dt))] = frame["ego_heading"]
        activity = brake_activity(log)
        summary = {
            **evaluate(log, benign),
            "brake_count": activity["total_count"],
            "brake_duration_s": activity["total_duration_s"],
            "metrics_until_t": float(log.t[-1]),
        }
        if log.first_collision_t is not None:
            # Stop presentation statistics at contact, not after the unmodelled impact.
            contact = log.first_collision_t
            clipped = []
            for episode in activity["episodes"]:
                if episode["start_t"] >= contact - 1e-9:
                    continue
                finish = min(episode["end_t"], contact)
                released = episode["release_t"]
                clipped.append(
                    {
                        **episode,
                        "end_t": finish,
                        "release_t": (
                            released if released is not None and released < contact else None
                        ),
                        "duration_s": finish - episode["start_t"],
                    }
                )
            activity["episodes"] = clipped
            activity["total_count"] = len(clipped)
            activity["total_duration_s"] = sum(e["duration_s"] for e in clipped)
            for i, t in enumerate(log.t):
                if t >= contact:
                    activity["count"][i] = len(clipped)
                    activity["duration_s"][i] = activity["total_duration_s"]
            summary.update(
                brake_count=len(clipped),
                brake_duration_s=activity["total_duration_s"],
                brake_onset_t=clipped[0]["start_t"] if clipped else None,
                min_dist=min(
                    [R_SAFE]
                    + [
                        d
                        for t, d in zip(log.t, distances, strict=True)
                        if t <= contact and d is not None
                    ]
                ),
                false_brake=bool(benign and clipped),
                metrics_until_t=contact,
            )
        result["systems"][name] = {
            "label": bundle.label,
            "first_collision_t": log.first_collision_t,
            "x": log.ego_x.tolist(),
            "y": log.ego_y.tolist(),
            "v": log.ego_v.tolist(),
            "a": log.ego_a.tolist(),
            "heading": headings,
            "ttc": [float(x) if np.isfinite(x) else None for x in log.ttc],
            "action": log.action,
            "distance": distances,
            "trace": recorder.frames,
            "executed_steps": log.executed_steps,
            "braking": activity,
            "summary": {
                k: (
                    None
                    if isinstance(v, float) and not np.isfinite(v)
                    else v.item() if isinstance(v, np.generic) else v
                )
                for k, v in summary.items()
            },
        }
    onsets = [result["systems"][s]["summary"]["brake_onset_t"] for s in ("reactive", "cv")]
    result["lead_time_s"] = (
        float(onsets[0] - onsets[1]) if all(t is not None for t in onsets) else None
    )
    return result

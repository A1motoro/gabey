"""Compare truth observations with white-box LiDAR, and export a scrub-able demo.

Run from the repository root. No raw Waymo, server, GPU or extra dependencies.
These are illustrative cases, not a population-level safety benchmark.
"""

# Direct-script execution needs the source path and a noninteractive plot backend.
# ruff: noqa: E402

import argparse
import csv
import json
import sys
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon

from pbs.data.perception_scenarios import adjacent_partial_occlusion, occluded_cut_in
from pbs.data.scenarios import make_scenario
from pbs.data.womd import load_clip
from pbs.perception import Lidar2DSensor, PerceptionPipeline
from pbs.prediction import NeuralPredictor
from pbs.risk.assessors import NullAssessor, PredictiveAssessor
from pbs.sim.policy import HysteresisPolicy, NeverBrakePolicy
from pbs.sim.simulator import run_sim
from pbs.systems import make_system
from pbs.utils import state_at

LABELS = {
    "cut_in": "无遮挡切入",
    "occluded_cut_in": "邻道遮挡后切入",
    "adjacent_partial_occlusion": "邻道直行、局部遮挡",
    "waymo": "Waymo 轨迹重建",
}
SYSTEMS = ("reactive", "cv", "ca", "ctrv", "lr", "nn", "nn_ua")


def _assessor(name):
    if name in ("nn", "nn_ua"):
        return PredictiveAssessor(
            NeuralPredictor(), uncertainty_scale=1.0 if name == "nn_ua" else 0.0
        )
    return make_system(name)


def _scenarios(quick):
    duration = 4.0 if quick else 8.0
    return {
        "cut_in": make_scenario("cut_in", gap_d=14, rel_v=4, duration=duration),
        "occluded_cut_in": occluded_cut_in(duration=duration),
        "adjacent_partial_occlusion": adjacent_partial_occlusion(duration=duration),
        "waymo": load_clip(
            ROOT / "tests/fixtures/cases/womd_dangerous_6e86a639df5d4fa1_track1782.npz"
        ),
    }


def _first_brake(log):
    executed = log.executed_steps
    indices = [i for i, action in enumerate(log.action[:executed]) if action == "brake"]
    return float(log.t[indices[0]]) if indices else None


def _rounded(points):
    return np.asarray(points).round(4).tolist()


def _frame_data(scene, log, frame, index):
    truth = []
    for agent in scene.agents:
        center = state_at(agent, frame.t)
        heading = float(np.interp(frame.t, agent.t, np.unwrap(agent.heading)))
        truth.append({"id": agent.agent_id, "center": _rounded(center), "heading": heading})
    return {
        "t": round(frame.t, 4),
        "ego": {
            "xy": _rounded(frame.ego_xy),
            "heading": frame.ego_heading,
            "v": round(float(log.ego_v[index]), 4),
        },
        "ttc": float(log.ttc[index]) if np.isfinite(log.ttc[index]) else None,
        "action": log.action[index],
        "points": _rounded(frame.points),
        "detections": [
            {
                "center": _rounded(box.center),
                "length": box.length,
                "width": box.width,
                "heading": box.heading,
            }
            for box in frame.detections
        ],
        "tracks": [
            {
                "id": track.history.agent_id,
                "observed": track.observed,
                "trail": _rounded(np.column_stack((track.history.x, track.history.y))),
                "position_covariance": _rounded(track.position_covariance),
            }
            for track in frame.tracks
        ],
        "truth": truth,
    }


def _corners(center, length, width, heading):
    local = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * [length / 2, width / 2]
    c, s = np.cos(heading), np.sin(heading)
    return local @ np.array([[c, s], [-s, c]]) + center


def _overview(runs, output):
    keys = ("cut_in", "occluded_cut_in", "adjacent_partial_occlusion")
    titles = ("Unoccluded cut-in", "Occluded cut-in", "Adjacent straight target")
    fig, axes = plt.subplots(2, 3, figsize=(15, 6), layout="constrained")
    for column, key in enumerate(keys):
        system = "cv" if "cv" in runs[key]["systems"] else next(iter(runs[key]["systems"]))
        frames = runs[key]["systems"][system]["frames"]
        for row, timestamp in enumerate((0.0, 1.5)):
            frame = min(frames, key=lambda item: abs(item["t"] - timestamp))
            ax = axes[row, column]
            origin = np.asarray(frame["ego"]["xy"])
            for y in (-1.75, 1.75, 5.25):
                ax.axhline(y - origin[1], color="#d0d6dd", lw=1, ls="--")
            ax.add_patch(
                Polygon(
                    _corners([0, 0], 5, 2.5, frame["ego"]["heading"]),
                    facecolor="#3875cb",
                    alpha=0.8,
                )
            )
            for body in frame["truth"]:
                ax.add_patch(
                    Polygon(
                        _corners(np.asarray(body["center"]) - origin, 5, 2.5, body["heading"]),
                        facecolor="#e8edf2",
                        edgecolor="#697787",
                        ls="--",
                    )
                )
            points = np.asarray(frame["points"])
            if len(points):
                points = points - origin
                ax.scatter(points[:, 0], points[:, 1], s=9, c="#089a96", zorder=4)
            for box in frame["detections"]:
                ax.add_patch(
                    Polygon(
                        _corners(
                            np.asarray(box["center"]) - origin,
                            box["length"],
                            box["width"],
                            box["heading"],
                        ),
                        fill=False,
                        edgecolor="#dc7b21",
                        lw=1.7,
                    )
                )
            for track in frame["tracks"]:
                trail = np.asarray(track["trail"]) - origin
                ax.plot(trail[:, 0], trail[:, 1], color="#8c49bd", lw=1, alpha=0.6)
                ax.scatter(*trail[-1], s=28, color="#8c49bd", zorder=5)
            ax.set(
                xlim=(-6, 35),
                ylim=(-3, 9),
                aspect="equal",
                title=f"{titles[column]} | t={frame['t']:.1f}s",
                xlabel="x relative to ego (m)",
                ylabel="y relative to ego (m)",
            )
            ax.spines[["top", "right"]].set_visible(False)
    legend = [
        Line2D([], [], color=color, marker=marker, ls=style, label=label)
        for color, marker, style, label in (
            ("#3875cb", "s", "", "Ego"),
            ("#697787", "", "--", "Truth (display only)"),
            ("#089a96", ".", "", "LiDAR returns"),
            ("#dc7b21", "", "-", "Visible box"),
            ("#8c49bd", "o", "-", "Filtered track"),
        )
    ]
    fig.legend(handles=legend, loc="outside lower center", ncol=5, frameon=False)
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true", help="4s synthetic cases, 2s Waymo")
    parser.add_argument("--systems", nargs="+", choices=SYSTEMS, default=["reactive", "cv", "lr"])
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--beams", type=int, default=720)
    parser.add_argument(
        "--noise", type=float, default=0.02, help="range noise standard deviation (m)"
    )
    parser.add_argument(
        "--output-dir", type=Path, default=ROOT / "outputs/figures/lidar-perception"
    )
    parser.add_argument("--open", action="store_true", help="open the generated HTML viewer")
    args = parser.parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    runs, rows = {}, []
    for key, scene in _scenarios(args.quick).items():
        duration = min(float(scene.agents[0].t[-1]), 2.0 if args.quick and key == "waymo" else 8.0)
        no_brake = run_sim(scene, NullAssessor(), NeverBrakePolicy(), duration=duration)
        runs[key] = {"label": LABELS[key], "systems": {}}
        for system in dict.fromkeys(args.systems):
            for mode in ("truth", "lidar"):
                pipeline = (
                    None
                    if mode == "truth"
                    else PerceptionPipeline(
                        sensor=Lidar2DSensor(
                            args.beams, range_noise_std=args.noise, seed=args.seed
                        ),
                        record_frames=True,
                    )
                )
                log = run_sim(
                    scene,
                    _assessor(system),
                    HysteresisPolicy(),
                    duration=duration,
                    horizon=40,
                    perception_model=pipeline,
                )
                row = {
                    "scenario": key,
                    "system": system,
                    "observations": mode,
                    "collision": log.collision,
                    "first_collision_t": log.first_collision_t,
                    "first_brake_t": _first_brake(log),
                    "min_center_distance_m": log.min_dist,
                    "duration_s": duration,
                    "no_brake_collision_within_duration": no_brake.collision,
                    "brake_in_benign_case": not no_brake.collision
                    and _first_brake(log) is not None,
                    "horizon_steps": 40,
                    "seed": args.seed,
                    "beams": args.beams,
                    "range_noise_std_m": args.noise,
                }
                rows.append(row)
                if pipeline is not None:
                    runs[key]["systems"][system] = {
                        "summary": row,
                        "truth_summary": rows[-2],
                        "frames": [
                            _frame_data(scene, log, frame, i)
                            for i, frame in enumerate(pipeline.frames)
                        ],
                    }
                print(
                    f"{key:28} {system:8} {mode:5} collision={log.collision} "
                    f"first_brake={row['first_brake_t']}"
                )
    payload = {
        "config": {"seed": args.seed, "beams": args.beams, "noise": args.noise},
        "runs": runs,
    }
    serialized = json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    (args.output_dir / "scans.json").write_text(serialized, encoding="utf-8")
    with (args.output_dir / "summary.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    template = (ROOT / "examples/lidar_viewer.html").read_text(encoding="utf-8")
    viewer = args.output_dir / "lidar_demo.html"
    viewer.write_text(
        template.replace("__LIDAR_DATA__", serialized.replace("<", "\\u003c")), encoding="utf-8"
    )
    _overview(runs, args.output_dir / "overview.png")
    print(f"Viewer: {viewer.resolve()}")
    if args.open:
        webbrowser.open(viewer.resolve().as_uri())
    return payload


if __name__ == "__main__":
    main()

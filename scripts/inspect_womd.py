"""Inspect a raw WOMD record, official SDC metadata and map/track BEV (no TensorFlow)."""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from pbs.data.womd import DEFAULT_RAW_DIR, read_scenarios, sdc_index, track_arrays  # noqa: E402


def main(args):
    scene = next(
        (
            scene
            for scene in read_scenarios(args.path)
            if args.scenario_id is None or scene.scenario_id == args.scenario_id
        ),
        None,
    )
    if scene is None:
        raise ValueError("Scenario ID not found")
    index = sdc_index(scene)
    print("scenario_id:", scene.scenario_id)
    print("scenario fields:", [f.name for f in scene.DESCRIPTOR.fields])
    print("track fields:", [f.name for f in scene.tracks[0].DESCRIPTOR.fields])
    print("state fields:", [f.name for f in scene.tracks[0].states[0].DESCRIPTOR.fields])
    print(
        f"SDC index={index}, track_id={scene.tracks[index].id}; "
        f"tracks={len(scene.tracks)}, timestamps={len(scene.timestamps_seconds)}"
    )
    ego_xy, _, _, valid = track_arrays(scene.tracks[index])
    center = ego_xy[valid].mean(axis=0)
    fig, ax = plt.subplots(figsize=(10, 7))
    for feature in scene.map_features:
        if feature.HasField("lane"):
            lane = np.array([(p.x, p.y) for p in feature.lane.polyline])
            if len(lane) and np.any(np.linalg.norm(lane - center, axis=1) < 150):
                ax.plot(lane[:, 0], lane[:, 1], color="#c7c7c7", lw=0.8)
    count = 0
    for i, track in enumerate(scene.tracks):
        xy, _, _, valid = track_arrays(track)
        if track.object_type == 1 and valid.any():
            ax.plot(
                xy[:, 0],
                xy[:, 1],
                color="#146cba" if i == index else "#d47a36",
                lw=2.5 if i == index else 0.8,
                alpha=1 if i == index else 0.5,
                label="SDC (metadata index)" if i == index else None,
            )
            if count < 8:
                print(f"vehicle track{track.id}: {valid.sum()}/{len(valid)} valid")
            count += 1
    ax.set(
        xlabel="World x [m]",
        ylabel="World y [m]",
        aspect="equal",
        title=f"WOMD {scene.scenario_id} | SDC index {index}",
    )
    ax.set_xlim(center[0] - 120, center[0] + 120)
    ax.set_ylim(center[1] - 90, center[1] + 90)
    ax.legend()
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)
    print("BEV:", args.output)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", type=Path, default=DEFAULT_RAW_DIR)
    parser.add_argument("--scenario-id")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/figures/exp07_womd_bev.png")
    main(parser.parse_args())

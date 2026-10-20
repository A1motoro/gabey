"""M1 — trajectory loading. Owner: A.

Course sample trajectories: one CSV per agent, columns

    timestamp, x, y, heading, velocity, acceleration, agent_type

Units m / rad / m·s⁻¹ / m·s⁻², 10 Hz. ``agent_type`` is ``"ego"`` for the ego and
``"vehicle"`` for the other agents; the file is one scenario (1 ego + 2 agents), not three
scenarios. The ``acceleration`` column is deliberately **not** loaded: ``Trajectory`` is
frozen at W4 without an acceleration field, and the data audit showed the velocity column
is noisy enough (±2 m/s² frame-to-frame) that differentiation is not trustworthy —
predictors fit velocity/acceleration by window least squares over raw positions instead.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np

from pbs.types import Trajectory

DATA_DIR = Path(__file__).resolve().parents[3] / "data"
SAMPLE_DIR = DATA_DIR / "sample_trajectories"
COLUMNS = ("timestamp", "x", "y", "heading", "velocity", "acceleration", "agent_type")


def _resolve_dir(data_dir) -> Path:
    """Return a directory that actually contains CSVs (tolerates the un-moved layout)."""
    directory = Path(data_dir)
    if directory.is_dir() and any(directory.glob("*.csv")):
        return directory
    if DATA_DIR.is_dir() and any(DATA_DIR.glob("*.csv")):
        print(
            f"[pbs.data] {directory} has no CSVs; falling back to {DATA_DIR} "
            "(course data was left in data/ instead of data/sample_trajectories/)",
            file=sys.stderr,
        )
        return DATA_DIR
    raise FileNotFoundError(
        f"no *.csv trajectories under {directory}"
        "\n  copy the course files into data/sample_trajectories/ (see README '数据说明')"
    )


def _to_trajectory(path: Path, agent_id: str | None = None) -> Trajectory:
    """Read one agent CSV into a Trajectory."""
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"empty trajectory file: {path}")
    missing = [column for column in COLUMNS if column not in rows[0]]
    if missing:
        raise ValueError(f"{path.name}: missing columns {missing}")

    t = np.array([float(row["timestamp"]) for row in rows])
    if np.any(np.diff(t) <= 0):
        raise ValueError(f"{path.name}: timestamps are not strictly increasing")
    return Trajectory(
        t=t,
        x=np.array([float(row["x"]) for row in rows]),
        y=np.array([float(row["y"]) for row in rows]),
        v=np.array([float(row["velocity"]) for row in rows]),
        heading=np.array([float(row["heading"]) for row in rows]),
        agent_type=(rows[0]["agent_type"] or "vehicle").strip(),
        agent_id=agent_id or path.stem,
    )


def load_toy(data_dir=SAMPLE_DIR) -> list[Trajectory]:
    """Read the course CSVs (``timestamp, x, y, heading, velocity, acceleration,
    agent_type``) and return the **surrounding agents**, ego excluded.

    ``agent_id`` = file stem (``"agent_trajectory_001"``, …), stable across runs so
    ``OraclePredictor`` can look a trajectory up. Put ego in ``Scenario.ego_start`` /
    ``Scenario.ego_path`` instead: keeping ego inside ``Scenario.agents`` makes the
    ground-truth minimum-distance statistic identically zero and confuses the Oracle.
    Use :func:`load_toy_ego` if you need the ego clip itself (e.g. the exp06 case study).
    """
    directory = _resolve_dir(data_dir)
    agents: list[Trajectory] = []
    for path in sorted(directory.glob("*.csv")):
        trajectory = _to_trajectory(path)
        if trajectory.agent_type == "ego" or path.stem.lower().startswith("ego"):
            continue
        agents.append(
            Trajectory(
                t=trajectory.t,
                x=trajectory.x,
                y=trajectory.y,
                v=trajectory.v,
                heading=trajectory.heading,
                agent_type=trajectory.agent_type,
                agent_id=path.stem,
            )
        )
    if not agents:
        raise FileNotFoundError(f"no non-ego agents under {directory}")
    return agents


def load_toy_ego(data_dir=SAMPLE_DIR) -> Trajectory | None:
    """The ego clip of the course sample scenario, or ``None`` if it is not present.

    Added (not renamed) for the W8 real-clip case study; ``Scenario`` still takes the ego
    as ``ego_start`` + ``ego_path`` so that the ego is *simulated* rather than replayed.
    """
    directory = _resolve_dir(data_dir)
    for path in sorted(directory.glob("*.csv")):
        trajectory = _to_trajectory(path)
        if trajectory.agent_type == "ego" or path.stem.lower().startswith("ego"):
            return trajectory
    return None


def load_nuscenes_tracks(scene_id: str) -> list[Trajectory]:
    """Optional (L3): nuScenes-mini instance tracks, 2 Hz → 10 Hz interpolation."""
    raise NotImplementedError(
        "TODO(A): optional L3 — nuScenes-mini tracks; "
        "every claim must first hold on the synthetic scenario family"
    )

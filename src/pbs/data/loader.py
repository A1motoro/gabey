"""M1 — trajectory loading. Owner: A."""

from pathlib import Path

from pbs.types import Trajectory

DATA_DIR = Path(__file__).resolve().parents[3] / "data"


def load_toy(data_dir: Path = DATA_DIR / "sample_trajectories") -> list[Trajectory]:
    """Read course CSVs (timestamp, x, y, heading, velocity, acceleration, agent_type)."""
    raise NotImplementedError("TODO(A): W3–W4")


def load_nuscenes_tracks(scene_id: str) -> list[Trajectory]:
    """Optional: nuScenes-mini instance tracks, 2 Hz → 10 Hz interpolation."""
    raise NotImplementedError("TODO(A): optional, W8+")

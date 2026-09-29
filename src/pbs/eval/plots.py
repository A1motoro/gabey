"""M4 — figures. Owner: D.

Headline figure: Reactive / CV / CA / Oracle failure boundaries overlaid on (gap_d, rel_v).
"""

import matplotlib.pyplot as plt

from pbs.types import SimLog


def plot_timeline(log: SimLog, ax=None, label: str = "ego"):
    """Ego speed vs time with brake intervals shaded. Overlay several systems on one ax."""
    ax = ax or plt.gca()
    (line,) = ax.plot(log.t, log.ego_v, label=f"{label} v [m/s]")
    step = log.t[1] - log.t[0]
    for ti, a in zip(log.t, log.action, strict=True):
        if a == "brake":
            ax.axvspan(ti, ti + step, color=line.get_color(), alpha=0.12, lw=0)
    ax.set_xlabel("t [s]")
    ax.legend()
    return ax


def plot_boundaries(df, systems, ax=None):
    """One contour (collision/no-collision frontier) per system on (gap_d, rel_v)."""
    raise NotImplementedError("TODO(D): W8–W10")

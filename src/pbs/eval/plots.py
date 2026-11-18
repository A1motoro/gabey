"""M4 — figures. Owner: D.

Headline figure: Reactive / CV / CA / Oracle failure boundaries overlaid on (gap_d, rel_v).
"""

import matplotlib.pyplot as plt
import numpy as np

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


SYSTEM_COLORS = {
    "reactive": "#4C78A8",
    "cv": "#F58518",
    "ca": "#54A24B",
    "ctrv": "#E45756",
    "oracle": "#B279A2",
    "lr": "#72B7B2",
}
_MARKERS = {"reactive": "x", "cv": "o", "ca": "+", "ctrv": "s", "oracle": "D", "lr": "P"}


def plot_boundaries(df, systems, ax=None, threshold=None, latency=None, labels=None):
    """One collision/no-collision contour per system on (gap_d, rel_v).

    ``df`` must contain a single threshold and latency per system, or pass ``threshold``
    and ``latency`` to select that working point. Collision cells are always marked.
    The 0.5 contour is drawn only when a system has at least four collision cells;
    two isolated cells would otherwise be painted as a region. A system with no
    collisions is named in the legend and has no mark.
    """
    from pbs.eval.sweep import failure_boundary

    ax = ax or plt.gca()
    labels = list(labels) if labels is not None else list(systems)
    for name, label in zip(systems, labels, strict=True):
        cells = failure_boundary(df, name, threshold=threshold, latency=latency)
        color = SYSTEM_COLORS.get(name, None)
        hits = cells.loc[cells["collision"]]
        # Two isolated collision cells make contour invent a blob. Draw the cells
        # always, and the 0.5 contour only once the failures form a region.
        if len(hits) >= 4:
            pivot = (
                cells.pivot(index="rel_v", columns="gap_d", values="collision")
                .astype(float)
                .sort_index()
                .sort_index(axis=1)
            )
            xs = pivot.columns.to_numpy(dtype=float)
            ys = pivot.index.to_numpy(dtype=float)
            field = pivot.to_numpy(dtype=float)
            if (
                field.shape[0] >= 2
                and field.shape[1] >= 2
                and np.nanmin(field) < 0.5 <= np.nanmax(field)
            ):
                ax.contour(xs, ys, field, levels=[0.5], colors=[color], linewidths=2.0)
        if len(hits):
            ax.scatter(
                hits["gap_d"],
                hits["rel_v"],
                s=42,
                color=color,
                marker=_MARKERS.get(name, "x"),
                facecolors="none",
                linewidths=1.5,
                zorder=3,
                label=label,
            )
        else:
            ax.plot([], [], color=color, linewidth=2.0, label=f"{label} (0 collisions)")
    if "gap_d" in df.columns and len(df):
        pad_x = 0.08 * (float(df["gap_d"].max()) - float(df["gap_d"].min()) + 1.0)
        pad_y = 0.08 * (float(df["rel_v"].max()) - float(df["rel_v"].min()) + 1.0)
        ax.set_xlim(float(df["gap_d"].min()) - pad_x, float(df["gap_d"].max()) + pad_x)
        ax.set_ylim(float(df["rel_v"].min()) - pad_y, float(df["rel_v"].max()) + pad_y)
    ax.set_xlabel("gap_d [m]")
    ax.set_ylabel("rel_v [m/s]")
    ax.grid(alpha=0.3)
    ax.legend()
    return ax


def plot_stopping_distance(ax, t_react: float, a_brake: float = 6.0, rel_max: float | None = None):
    """Analytic gap needed to cancel ``rel_v``: d = rel_v·t_react + rel_v²/(2a)."""
    upper = rel_max if rel_max is not None else max(float(ax.get_ylim()[1]), 1.0)
    rel = np.linspace(0.0, max(upper, 0.0), 80)
    gap = rel * t_react + rel**2 / (2.0 * a_brake)
    ax.plot(
        gap,
        rel,
        color="black",
        linestyle="--",
        linewidth=1.0,
        label=f"stopping gap, t={t_react:.1f}s",
    )
    return ax

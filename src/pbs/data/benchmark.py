"""M1 — fixed prediction-benchmark slices. Owner: A.

Turns a :class:`~pbs.types.Scenario` into a deterministic list of
``(history, ground truth future)`` pairs so that every predictor (CV / CA / CTRV / LR /
Oracle) is scored on **exactly the same** windows. This is what ``exp01`` should consume
instead of the course clip: the clip has no lane change and its speed column is
piecewise constant, so it cannot show whether an acceleration model helps.

Two rules that make the benchmark honest:

* ``history`` contains only samples with ``t <= t0`` (no future leakage), matching what
  ``run_sim`` hands to an assessor.
* ``truth`` is the agent's true future at ``t0 + k·dt``, ``k = 1..H`` — the same time
  convention as ``Predictor.predict``.

The benchmark does not import ``pbs.prediction`` / ``pbs.sim`` / ``pbs.eval``.
"""

from dataclasses import dataclass

import numpy as np

from pbs.types import Scenario, Trajectory
from pbs.utils import history_until


@dataclass(frozen=True)
class PredictionSlice:
    """One forecast problem: history at ``t0`` and the true future after it."""

    t0: float
    history: Trajectory
    truth: np.ndarray  # (H, 2) true positions at t0 + k·dt, k = 1..H
    truth_accel: np.ndarray | None = None  # (H, 2) GT acceleration, if the scenario has it


def _steps(name: str, value: float, dt: float) -> int:
    """Validate that ``value`` is a whole number of simulation steps and return it."""
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive, got {value!r}")
    steps = int(round(value / dt))
    if steps < 1 or not np.isclose(steps * dt, value, rtol=0, atol=1e-9):
        raise ValueError(
            f"{name}={value!r} s is not a positive multiple of scenario.dt={dt!r} s"
        )
    return steps


def prediction_slices(
    scenario: Scenario,
    *,
    history: float = 1.0,
    horizon: float = 2.0,
    stride: float = 0.5,
    agent_index: int = 0,
) -> list[PredictionSlice]:
    """Deterministic forecast windows over one scenario.

    Parameters
    ----------
    history:
        Seconds of past observations each slice may use (window ``[t0 − history, t0]``).
    horizon:
        Forecast length: truth rows are ``t0 + k·dt`` for ``k = 1..H``, ``H = horizon/dt``.
    stride:
        Spacing between consecutive ``t0`` values (must be a multiple of ``dt``).
    agent_index:
        Which agent of the scenario to forecast (M1 scenarios have exactly one).

    Returns
    -------
    list[PredictionSlice]
        Ordered by ``t0``; the first window starts at ``t0 = history`` (a full past is
        required) and the last ends at the scenario duration.

    Notes
    -----
    ``history`` and ``horizon`` are in **seconds**, not steps — predictors take steps.
    Use ``len(slice.history.t)`` and ``len(slice.truth)`` for the step counts.
    """
    dt = float(scenario.dt)
    n_history = _steps("history", history, dt)
    n_ahead = _steps("horizon", horizon, dt)
    step = _steps("stride", stride, dt)

    agent = scenario.agents[agent_index]
    duration = float(scenario.params.get("duration", agent.t[-1]))
    if agent.t[0] > 1e-9 or agent.t[-1] < duration - 1e-9:
        raise ValueError(
            f"agent {agent.agent_id!r} must cover [0, {duration}] s "
            f"(got [{agent.t[0]}, {agent.t[-1]}])"
        )
    if n_history * dt > duration - n_ahead * dt + 1e-9:
        raise ValueError(
            f"history={history} s + horizon={horizon} s does not fit in duration={duration} s"
        )

    first = n_history
    last = int(round((duration - horizon) / dt))
    offsets = np.arange(1, n_ahead + 1) * dt
    accel = agent.accel

    slices: list[PredictionSlice] = []
    for k in range(first, last + 1, step):
        t0 = k * dt
        future = t0 + offsets
        truth = np.column_stack(
            (np.interp(future, agent.t, agent.x), np.interp(future, agent.t, agent.y))
        )
        truth_accel = None
        if accel is not None:
            truth_accel = np.column_stack(
                (
                    np.interp(future, agent.t, accel[:, 0]),
                    np.interp(future, agent.t, accel[:, 1]),
                )
            )
        slices.append(
            PredictionSlice(
                t0=float(t0),
                history=history_until(agent, t0),
                truth=truth,
                truth_accel=truth_accel,
            )
        )
    return slices

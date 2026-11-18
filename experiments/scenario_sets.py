"""Scenario sets for D's experiments, built with A's ``pbs.data`` generators.

Logs are ``LOG_PAD`` seconds longer than the simulation. The risk horizon is 3 s, and
without the pad Oracle holds the last sample and treats a moving agent as stopped.
"""

import numpy as np

from pbs.data import (
    LANE_WIDTH,
    V_EGO,
    ca_scenario_set,
    quintic_lateral,
    quintic_lateral_vel,
    scenario_set,
)
from pbs.eval.sweep import scenario_id
from pbs.types import Scenario, Trajectory

SIM_DURATION = 15.0
LOG_PAD = 3.0
LOG_DURATION = SIM_DURATION + LOG_PAD
QUICK_GAPS = (8.0, 14.0, 22.0, 32.0, 48.0)
QUICK_RELS = (-4.0, 0.0, 4.0, 8.0, 12.0)
FULL_GAPS = tuple(float(value) for value in np.linspace(6.0, 60.0, 15))
FULL_RELS = tuple(float(value) for value in np.linspace(-4.0, 12.0, 17))
LC_T = 3.0


def boundary_grid(quick: bool = False) -> list[Scenario]:
    """Dense ``make_scenario("cut_in")`` sweep on the (gap_d, rel_v) plane at one lc_T."""
    gaps = QUICK_GAPS if quick else FULL_GAPS
    rels = QUICK_RELS if quick else FULL_RELS
    grid = {"cut_in": {"gap_d": list(gaps), "rel_v": list(rels), "lc_T": [LC_T]}}
    return scenario_set(("cut_in",), grid, duration=LOG_DURATION).scenarios


def tradeoff_set() -> tuple[list[Scenario], dict]:
    """A's default grid (all three kinds), the CA slice, and aborted cut-ins.

    A's grid has no aborted cut-in: a cutter that leans toward the ego lane, then goes
    back. Those are the cases where CV extrapolation predicts a conflict that never
    happens, so they are added here and counted separately.
    """
    default = scenario_set(strict=False, duration=LOG_DURATION)
    ca = ca_scenario_set(duration=LOG_DURATION)
    aborts = abort_cut_ins()
    scenarios, seen = [], set()
    for scenario in [*default.scenarios, *ca.scenarios, *aborts]:
        sid = scenario_id(scenario)
        if sid not in seen:
            seen.add(sid)
            scenarios.append(scenario)
    counts = {
        "default_grid": default.n_built,
        "default_grid_rejected": default.n_rejected,
        "ca_slice": ca.n_built,
        "abort_cut_ins": len(aborts),
        "unique": len(scenarios),
    }
    return scenarios, counts


def abort_cut_ins() -> list[Scenario]:
    return [make_abort(gap, rel) for gap in (0.0, 1.5, 3.0) for rel in (0.0, -2.0)]


def make_abort(gap_d: float, rel_v: float, *, v_ego: float = V_EGO) -> Scenario:
    """Cutter moves from y = 3.5 to 2.8 m and back, staying outside the 2.5 m footprint."""
    t = np.arange(int(round(LOG_DURATION / 0.1)) + 1) * 0.1
    y_low = LANE_WIDTH - 0.7
    y = (
        quintic_lateral(LANE_WIDTH, y_low, 1.5, t, t0=0.5)
        + quintic_lateral(y_low, LANE_WIDTH, 1.5, t, t0=2.0)
        - y_low
    )
    vy = quintic_lateral_vel(LANE_WIDTH, y_low, 1.5, t, t0=0.5) + quintic_lateral_vel(
        y_low, LANE_WIDTH, 1.5, t, t0=2.0
    )
    vx = np.full_like(t, v_ego - rel_v)
    agent = Trajectory(
        t, gap_d + vx * t, y, np.hypot(vx, vy), np.arctan2(vy, vx), agent_id="cutter"
    )
    return Scenario(
        ego_start=(0.0, 0.0, 0.0, float(v_ego)),
        ego_path=np.array([[0.0, 0.0], [v_ego * LOG_DURATION + 100.0, 0.0]]),
        agents=[agent],
        dt=0.1,
        kind="cut_in",
        lane_width=LANE_WIDTH,
        params={"gap_d": float(gap_d), "rel_v": float(rel_v), "maneuver": "abort"},
    )


def kind_label(scenario: Scenario) -> str:
    if scenario.params.get("maneuver") == "abort":
        return "cut_in_abort"
    if scenario.kind == "cut_in" and scenario.params.get("a_cutter"):
        return "cut_in_ca"
    return scenario.kind


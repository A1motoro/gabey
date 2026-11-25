"""M1 — parametric synthetic scenarios. Owner: A.

Physical plausibility first: lane changes follow a quintic (minimum-jerk) lateral profile

    y(τ) = y0 + Δy·(10τ³ − 15τ⁴ + 6τ⁵),   τ = (t − t0)/T ∈ [0, 1]

whose peak lateral acceleration is (10/√3)·|Δy|/T² ≈ 5.77·|Δy|/T², reached at
τ ≈ 0.211 (mirrored at τ ≈ 0.789). Parameter combinations whose peak exceeds
``max_lat_acc`` are **rejected** by :func:`make_scenario` and *counted* by
:func:`scenario_set` — never silently clipped — so the report can state what fraction of
the grid is physically admissible.

Geometry (all kinds): the ego starts at x = 0 on a straight lane centreline y = 0 with
heading +x. Lateral offsets are measured from that centreline; ``|offset| < lane_width/2``
(= 1.75 m) is "inside the ego lane", the predicate the Reactive baseline uses.

===========  ================================================  ==============
kind         params                                            conflict type
===========  ================================================  ==============
``cut_in``    gap_d, rel_v, cut_in_t, lc_T                     lateral
``lead_brake`` init_gap, decel, t_brake, rel_v                 longitudinal
``crossing``  gap_d, arrival_t, lateral_v                      lateral
===========  ================================================  ==============

``gap_d``      centre-to-centre longitudinal gap at t = 0 [m]
``rel_v``      ego speed − other speed [m/s]; > 0 ⇒ ego is faster (closing)
``cut_in_t``   time the lane change starts [s]; ``lc_T`` is its duration [s]
``init_gap``   centre-to-centre gap to the lead at t = 0 [m]; ``decel`` [m/s²] starts at
               ``t_brake`` and lasts until the lead stops
``arrival_t``  time at which the crossing agent reaches the ego-lane centreline [s];
               it starts at lateral offset ``lateral_v · arrival_t`` and moves at
               constant ``lateral_v`` [m/s]

Every scenario set MUST contain benign cases (dangerous-looking but collision-free under
``NeverBrakePolicy``, e.g. cut-in with rel_v ≤ 0 or a large gap_d). Otherwise the
false-brake rate is identically 0 and the trade-off curve cannot be drawn. Label a
scenario with :func:`pbs.eval.counterfactual.is_benign`; this module deliberately does not
import :mod:`pbs.sim` (M1 must stay below M3 in the data flow).

Times are sampled at ``dt``; agent trajectories always cover the whole scenario duration,
so no agent is frozen by endpoint extrapolation.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field

import numpy as np

from pbs.types import Scenario, Trajectory

KINDS = ("cut_in", "lead_brake", "crossing")

MAX_LAT_ACC = 3.0  # m/s²; comfort/aggressive bound — sensitivity 2.0/3.0/4.5 in exp00
MAX_LONG_ACC = 3.0  # m/s²; longitudinal comfort bound for the cut-in acceleration slice
DT = 0.1  # s, frozen interface
LANE_WIDTH = 3.5  # m, frozen interface
DURATION = 15.0  # s, default horizon of one scenario
V_EGO = 15.0  # m/s (= 54 km/h), default ego speed

_PEAK_SHAPE = 10.0 / np.sqrt(3.0)  # peak of the dimensionless quintic acceleration


# --------------------------------------------------------------------------------------
# quintic (minimum-jerk) lateral profile
# --------------------------------------------------------------------------------------
def _tau(t, t0: float, T: float) -> np.ndarray:
    """Normalised time, clamped to [0, 1] (outside the lane change the profile is flat)."""
    if T <= 0:
        raise ValueError(f"lane-change duration T must be > 0, got {T}")
    return np.clip((np.asarray(t, dtype=float) - t0) / T, 0.0, 1.0)


def quintic_lateral(y0: float, y1: float, T: float, t, t0: float = 0.0) -> np.ndarray:
    """Lateral position of a minimum-jerk lane change starting at t0 with duration T.

    y(t) = y0 for t ≤ t0, y1 for t ≥ t0 + T; C²-continuous (zero lateral velocity and
    acceleration at both ends).
    """
    tau = _tau(t, t0, T)
    return y0 + (y1 - y0) * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)


def quintic_lateral_vel(y0: float, y1: float, T: float, t, t0: float = 0.0) -> np.ndarray:
    """d/dt of :func:`quintic_lateral` [m/s]; 0 outside the lane change."""
    tau = _tau(t, t0, T)
    return (y1 - y0) / T * (30 * tau**2 - 60 * tau**3 + 30 * tau**4)


def quintic_lateral_acc(y0: float, y1: float, T: float, t, t0: float = 0.0) -> np.ndarray:
    """d²/dt² of :func:`quintic_lateral` [m/s²]; 0 outside the lane change."""
    tau = _tau(t, t0, T)
    return (y1 - y0) / T**2 * (60 * tau - 180 * tau**2 + 120 * tau**3)


def peak_lateral_acc(dy: float, T: float) -> float:
    """max |d²y/dt²| of the quintic profile [m/s²] — closed form, no sampling."""
    if T <= 0:
        raise ValueError(f"lane-change duration T must be > 0, got {T}")
    return _PEAK_SHAPE * abs(dy) / T**2


def min_lc_duration(dy: float, max_lat_acc: float = MAX_LAT_ACC) -> float:
    """Shortest lane-change duration that keeps the peak lateral acceleration ≤ bound."""
    if max_lat_acc <= 0:
        raise ValueError(f"max_lat_acc must be > 0, got {max_lat_acc}")
    return float(np.sqrt(_PEAK_SHAPE * abs(dy) / max_lat_acc))


# --------------------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------------------
def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _times(duration: float, dt: float = DT) -> np.ndarray:
    _require(duration > 0, f"duration must be > 0, got {duration}")
    _require(dt > 0, f"dt must be > 0, got {dt}")
    return np.arange(0.0, duration + 0.5 * dt, dt)


def _trajectory(
    t: np.ndarray,
    x,
    y,
    vx,
    vy,
    agent_id: str,
    agent_type: str = "vehicle",
    ax=None,
    ay=None,
) -> Trajectory:
    """Build a Trajectory from kinematic arrays (heading = atan2(vy, vx)).

    ``ax``/``ay`` are the analytic ground-truth accelerations (same grid as ``t``).
    They are stored in ``Trajectory.accel`` for M2 validation; predictors must still
    estimate acceleration from observations, never read this field at test time.
    """
    shape = np.shape(t)

    def _as_array(value) -> np.ndarray:
        return np.array(np.broadcast_to(np.asarray(value, dtype=float), shape))

    vx, vy = _as_array(vx), _as_array(vy)
    accel = None
    if ax is not None:
        accel = np.column_stack((_as_array(ax), _as_array(0.0 if ay is None else ay)))
    return Trajectory(
        t=t,
        x=_as_array(x),
        y=_as_array(y),
        v=np.hypot(vx, vy),
        heading=np.arctan2(vy, vx),
        agent_type=agent_type,
        agent_id=agent_id,
        accel=accel,
    )


def _ego_path(v_ego: float, duration: float) -> np.ndarray:
    """Straight lane centreline, long enough for the ego to never run off its end."""
    length = max(200.0, float(v_ego) * float(duration) + 100.0)
    return np.array([[0.0, 0.0], [length, 0.0]])


def _base_params(**params) -> dict:
    """JSON/DataFrame friendly record of every resolved parameter (for run_matrix)."""
    return {
        key: (float(value) if isinstance(value, int | float | np.floating)
              and not isinstance(value, bool) else value)
        for key, value in params.items()
    }


# --------------------------------------------------------------------------------------
# scenario builders
# --------------------------------------------------------------------------------------
def _cut_in(
    gap_d: float = 25.0,
    rel_v: float = 2.0,
    cut_in_t: float = 0.5,
    lc_T: float = 3.0,
    *,
    a_cutter: float = 0.0,
    accel_T: float | None = None,
    v_ego: float = V_EGO,
    lane_width: float = LANE_WIDTH,
    duration: float = DURATION,
    dt: float = DT,
    from_left: bool = True,
    max_lat_acc: float = MAX_LAT_ACC,
    max_lon_acc: float = MAX_LONG_ACC,
    agent_id: str = "cutter",
) -> Scenario:
    """Adjacent-lane vehicle merges into the ego lane ahead of the ego.

    Lateral: quintic from the adjacent lane centre to ``y = 0`` over ``lc_T``, starting
    at ``cut_in_t``. Longitudinal: constant speed ``v_ego − rel_v`` unless ``a_cutter``
    is non-zero, in which case the speed ramps at ``a_cutter`` for ``accel_T`` seconds
    (default: the lane-change window) and stays constant afterwards.

    ``a_cutter`` is what makes the **constant-acceleration (CA)** predictor different
    from CV on this family: with ``a_cutter = 0`` the two are identical along x.
    """
    _require(gap_d > 0, f"cut_in: gap_d must be > 0 m, got {gap_d}")
    _require(cut_in_t >= 0, f"cut_in: cut_in_t must be ≥ 0 s, got {cut_in_t}")
    _require(lc_T > 0, f"cut_in: lc_T must be > 0 s, got {lc_T}")
    _require(lane_width > 0, f"cut_in: lane_width must be > 0 m, got {lane_width}")
    _require(
        abs(a_cutter) <= max_lon_acc,
        f"cut_in: |a_cutter|={abs(a_cutter):.2f} m/s² exceeds the {max_lon_acc:.2f} m/s² "
        "longitudinal bound",
    )
    window = lc_T if accel_T is None else float(accel_T)
    _require(window > 0, f"cut_in: accel_T must be > 0 s, got {window}")
    v_cut = v_ego - rel_v
    _require(v_cut >= 0, f"cut_in: rel_v={rel_v} exceeds v_ego={v_ego} (cutter would reverse)")
    v_end = v_cut + a_cutter * window
    _require(
        v_end >= 0,
        f"cut_in: a_cutter={a_cutter:g} m/s² over accel_T={window:.2f} s would make the "
        f"cutter reverse ({v_cut:.2f} → {v_end:.2f} m/s); lower |a_cutter| or accel_T",
    )

    y0 = lane_width if from_left else -lane_width  # start in the adjacent lane centre
    dy = -y0  # merge to the ego-lane centreline (y = 0)
    peak = peak_lateral_acc(dy, lc_T)
    _require(
        peak <= max_lat_acc,
        f"cut_in: peak lateral acceleration {peak:.2f} m/s² of a {lc_T:.2f} s lane change "
        f"exceeds the {max_lat_acc:.2f} m/s² bound "
        f"(minimum lc_T = {min_lc_duration(dy, max_lat_acc):.2f} s); "
        "lower |Δy|, lengthen lc_T, or raise max_lat_acc explicitly",
    )

    t = _times(duration, dt)
    # Longitudinal speed ramps only inside [cut_in_t, cut_in_t + window]:
    #   v_x = v_cut + a·τ,  τ = clip(t − cut_in_t, 0, window)
    #   x   = gap_d + v_cut·t + ½·a·τ² + a·window·max(t − cut_in_t − window, 0)
    # The last term keeps the speed gained during the ramp after it ends.
    tau = np.clip(t - cut_in_t, 0.0, window)
    x = (
        gap_d
        + v_cut * t
        + 0.5 * a_cutter * tau**2
        + a_cutter * window * np.clip(t - cut_in_t - window, 0.0, None)
    )
    vx = v_cut + a_cutter * tau
    y = quintic_lateral(y0, 0.0, lc_T, t, t0=cut_in_t)
    vy = quintic_lateral_vel(y0, 0.0, lc_T, t, t0=cut_in_t)
    ax = np.where((t >= cut_in_t) & (t <= cut_in_t + window), a_cutter, 0.0)
    ay = quintic_lateral_acc(y0, 0.0, lc_T, t, t0=cut_in_t)
    return Scenario(
        ego_start=(0.0, 0.0, 0.0, float(v_ego)),
        ego_path=_ego_path(v_ego, duration),
        agents=[_trajectory(t, x, y, vx, vy, agent_id, ax=ax, ay=ay)],
        dt=dt,
        kind="cut_in",
        lane_width=float(lane_width),
        params=_base_params(
            gap_d=gap_d,
            rel_v=rel_v,
            cut_in_t=cut_in_t,
            lc_T=lc_T,
            a_cutter=a_cutter,
            accel_T=window,
            v_ego=v_ego,
            v_cutter=v_cut,
            v_cutter_end=v_end,
            peak_lat_acc=peak,
            from_left=from_left,
            duration=duration,
        ),
    )


def _lead_brake(
    init_gap: float = 30.0,
    decel: float = 4.0,
    t_brake: float = 1.0,
    rel_v: float = 0.0,
    *,
    v_ego: float = V_EGO,
    lane_width: float = LANE_WIDTH,
    duration: float = DURATION,
    dt: float = DT,
    max_lat_acc: float = MAX_LAT_ACC,  # accepted and ignored: no lateral motion
    agent_id: str = "lead",
) -> Scenario:
    """Lead vehicle in the ego lane brakes at a constant ``decel`` from ``t_brake`` on."""
    _require(init_gap > 0, f"lead_brake: init_gap must be > 0 m, got {init_gap}")
    _require(decel > 0, f"lead_brake: decel must be > 0 m/s², got {decel}")
    _require(t_brake >= 0, f"lead_brake: t_brake must be ≥ 0 s, got {t_brake}")
    _require(decel <= 10.0, f"lead_brake: decel={decel} m/s² is not a road vehicle")
    v_lead = v_ego - rel_v
    _require(v_lead >= 0, f"lead_brake: rel_v={rel_v} exceeds v_ego={v_ego}")

    t = _times(duration, dt)
    tau = np.clip(t - t_brake, 0.0, None)
    stopped_at = v_lead / decel  # time after t_brake until the lead vehicle stops
    v_lead_t = np.where(t <= t_brake, v_lead, np.maximum(0.0, v_lead - decel * tau))
    # travelled distance: constant for t ≤ t_brake, then v·τ − ½aτ² (frozen once stopped)
    tau_eff = np.minimum(tau, stopped_at)
    travelled = np.where(
        t <= t_brake,
        v_lead * t,
        v_lead * t_brake + v_lead * tau_eff - 0.5 * decel * tau_eff**2,
    )
    x = init_gap + travelled
    # GT acceleration: 0 while cruising, −decel while the lead is still moving, 0 once stopped.
    ax = np.where((t > t_brake) & (v_lead_t > 0.0), -decel, 0.0)
    return Scenario(
        ego_start=(0.0, 0.0, 0.0, float(v_ego)),
        ego_path=_ego_path(v_ego, duration),
        agents=[_trajectory(t, x, 0.0, v_lead_t, 0.0, agent_id, ax=ax, ay=0.0)],
        dt=dt,
        kind="lead_brake",
        lane_width=float(lane_width),
        params=_base_params(
            init_gap=init_gap,
            decel=decel,
            t_brake=t_brake,
            rel_v=rel_v,
            v_ego=v_ego,
            v_lead=v_lead,
            duration=duration,
        ),
    )


def _crossing(
    gap_d: float = 35.0,
    arrival_t: float = 2.3,
    lateral_v: float = 1.5,
    *,
    v_ego: float = V_EGO,
    lane_width: float = LANE_WIDTH,
    duration: float = DURATION,
    dt: float = DT,
    max_lat_acc: float = MAX_LAT_ACC,  # accepted and ignored: constant lateral speed
    agent_type: str = "pedestrian",
    agent_id: str = "crosser",
) -> Scenario:
    """Agent crosses the ego path at fixed x, reaching the centreline at ``arrival_t``."""
    _require(gap_d > 0, f"crossing: gap_d must be > 0 m, got {gap_d}")
    _require(arrival_t > 0, f"crossing: arrival_t must be > 0 s, got {arrival_t}")
    _require(lateral_v > 0, f"crossing: lateral_v must be > 0 m/s, got {lateral_v}")

    t = _times(duration, dt)
    y0 = lateral_v * arrival_t  # lateral offset at t = 0
    return Scenario(
        ego_start=(0.0, 0.0, 0.0, float(v_ego)),
        ego_path=_ego_path(v_ego, duration),
        agents=[
            _trajectory(
                t,
                np.full_like(t, gap_d),
                y0 - lateral_v * t,
                np.zeros_like(t),
                np.full_like(t, -lateral_v),
                agent_id,
                agent_type=agent_type,
                ax=0.0,
                ay=0.0,
            )
        ],
        dt=dt,
        kind="crossing",
        lane_width=float(lane_width),
        params=_base_params(
            gap_d=gap_d,
            arrival_t=arrival_t,
            lateral_v=lateral_v,
            v_ego=v_ego,
            lateral_offset_0=y0,
            duration=duration,
        ),
    )


_BUILDERS = {"cut_in": _cut_in, "lead_brake": _lead_brake, "crossing": _crossing}


def make_scenario(kind: str, **params) -> Scenario:
    """Build one scenario. Agents get unique ``agent_id`` (OraclePredictor relies on it).

    Unknown ``kind`` or an unphysical/over-aggressive parameter combination raises
    ``ValueError`` with the offending numbers — nothing is silently clipped.
    """
    if kind not in _BUILDERS:
        raise ValueError(f"unknown scenario kind {kind!r}; expected one of {KINDS}")
    return _BUILDERS[kind](**params)


# --------------------------------------------------------------------------------------
# scenario sets (grids) — with explicit rejection accounting
# --------------------------------------------------------------------------------------
@dataclass
class ScenarioSet:
    """A built grid plus the parameter combinations that were rejected, and why."""

    scenarios: list[Scenario] = field(default_factory=list)
    rejected: list[dict] = field(default_factory=list)

    @property
    def n_built(self) -> int:
        return len(self.scenarios)

    @property
    def n_rejected(self) -> int:
        return len(self.rejected)

    @property
    def rejection_rate(self) -> float:
        total = self.n_built + self.n_rejected
        return self.n_rejected / total if total else 0.0

    def by_kind(self) -> dict[str, list[Scenario]]:
        out: dict[str, list[Scenario]] = {k: [] for k in KINDS}
        for scenario in self.scenarios:
            out.setdefault(scenario.kind, []).append(scenario)
        return out

    def rejection_summary(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for row in self.rejected:
            counts[row["kind"]] = counts.get(row["kind"], 0) + 1
        return counts


# Physically admissible default grid: every combination here is accepted at
# max_lat_acc = 3.0 (shortest lane change = 3.0 s → 2.24 m/s²). It is deliberately
# *not* a dense grid — D's run_matrix owns the dense sweeps.
DEFAULT_GRID = {
    "cut_in": {
        "gap_d": [10.0, 20.0, 30.0, 40.0, 60.0],
        "rel_v": [-2.0, 0.0, 1.0, 2.0, 4.0, 6.0],
        "lc_T": [3.0, 4.0],
    },
    "lead_brake": {
        "init_gap": [20.0, 35.0, 50.0, 80.0, 100.0],
        "decel": [0.3, 1.0, 3.0, 6.0],
    },
    "crossing": {
        "gap_d": [15.0, 25.0, 35.0, 50.0],
        "arrival_t": [1.0, 2.5, 4.0, 6.0],
    },
}

# Slice that gives the constant-acceleration (CA) predictor something to learn from:
# the cutter now accelerates/decelerates during the merge (a_cutter ≠ 0), so CA and CV
# stop being identical. Kept separate from DEFAULT_GRID so D's main matrix (96 scenarios)
# and every documented denominator stay unchanged.
CA_GRID = {
    "cut_in": {
        "gap_d": [20.0, 30.0],
        "rel_v": [0.0, 2.0],
        "a_cutter": [-2.0, 0.0, 2.0],
    },
}


def scenario_set(
    kinds=KINDS,
    grid: dict | None = None,
    *,
    max_lat_acc: float = MAX_LAT_ACC,
    strict: bool = True,
    **common,
) -> ScenarioSet:
    """Build every combination of ``grid`` per kind.

    ``strict=True`` propagates the first rejected combination (use it in tests);
    ``strict=False`` records rejections in :attr:`ScenarioSet.rejected` so a report can
    quote "x % of the grid is physically inadmissible".
    ``common`` is passed to every builder (e.g. ``v_ego=20.0``).
    """
    grid = DEFAULT_GRID if grid is None else grid
    built = ScenarioSet()
    for kind in kinds:
        spec = grid.get(kind, {})
        keys = sorted(spec)
        for values in itertools.product(*(spec[key] for key in keys)):
            params = dict(zip(keys, values, strict=True))
            try:
                scenario = make_scenario(
                    kind, max_lat_acc=max_lat_acc, **params, **common
                )
            except ValueError as err:
                if strict:
                    raise
                built.rejected.append({"kind": kind, **params, "reason": str(err)})
                continue
            built.scenarios.append(scenario)
    return built


def ca_scenario_set(**common) -> ScenarioSet:
    """The CA slice: 12 cut-ins with ``a_cutter ∈ {−2, 0, +2} m/s²`` (see :data:`CA_GRID`).

    Same builders, same physics filter, same benign/dangerous accounting as the default
    grid — only the longitudinal acceleration is switched on.
    """
    return scenario_set(("cut_in",), CA_GRID, **common)


def scenario_label(scenario: Scenario) -> str:
    """Short human label, e.g. ``cut_in(gap_d=25,rel_v=3.0,lc_T=3.0)``."""
    keep = {
        "cut_in": ("gap_d", "rel_v", "cut_in_t", "lc_T"),
        "lead_brake": ("init_gap", "decel", "t_brake"),
        "crossing": ("gap_d", "arrival_t", "lateral_v"),
    }.get(scenario.kind, tuple(scenario.params))
    body = ",".join(f"{k}={scenario.params[k]:g}" for k in keep if k in scenario.params)
    if scenario.kind == "cut_in" and scenario.params.get("a_cutter"):
        body += f",a_cutter={scenario.params['a_cutter']:g}"
    return f"{scenario.kind}({body})"


# Named scenarios used by docs, figures and tests. Each family has at least one
# dangerous and one benign member; the benign ones are the "dangerous-looking but safe"
# cases that make the counterfactual false-brake rate non-trivial (spec §1.5).
CANONICAL: dict[str, dict] = {
    # lateral conflict, ego is closing → contact
    "cut_in_danger": dict(kind="cut_in", gap_d=25.0, rel_v=3.0, cut_in_t=0.5, lc_T=3.0),
    # cutter merges 25 m ahead at the same speed: nothing ever closes → benign
    "cut_in_benign_equal_speed": dict(
        kind="cut_in", gap_d=25.0, rel_v=0.0, cut_in_t=0.5, lc_T=3.0
    ),
    # cutter merges 12 m ahead but is faster: looks dangerous, is benign
    "cut_in_benign_fast_cutter": dict(
        kind="cut_in", gap_d=12.0, rel_v=-1.5, cut_in_t=0.5, lc_T=3.0
    ),
    # longitudinal conflict, ego never brakes → rear-end
    "lead_brake_danger": dict(kind="lead_brake", init_gap=30.0, decel=4.0, t_brake=1.0),
    # light braking far away: looks like a conflict, stays clear inside 15 s
    "lead_brake_benign": dict(kind="lead_brake", init_gap=100.0, decel=0.3, t_brake=1.0),
    # ego reaches the crossing point exactly when the pedestrian does
    "crossing_danger": dict(kind="crossing", gap_d=35.0, arrival_t=2.3, lateral_v=1.5),
    # pedestrian finishes crossing long before the ego arrives
    "crossing_benign": dict(kind="crossing", gap_d=35.0, arrival_t=6.0, lateral_v=1.5),
}


def canonical(name: str) -> Scenario:
    """Build one of :data:`CANONICAL` by name."""
    if name not in CANONICAL:
        raise KeyError(f"unknown canonical scenario {name!r}; expected one of {sorted(CANONICAL)}")
    spec = dict(CANONICAL[name])
    return make_scenario(spec.pop("kind"), **spec)

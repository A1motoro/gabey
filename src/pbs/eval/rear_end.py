"""Rear-end sandwich coupling analysis (post-processing over ``SimLog``).

Evaluates whether an AEB manoeuvre that avoids (or fails to avoid) a forward
conflict induces a secondary rear-end collision from a human-driven following
vehicle with perception-reaction delay ``reaction_time`` and bounded braking
deceleration ``max_decel``.

Zero-intrusion contract:
- Consumes ``pbs.types.SimLog`` directly without modifying ``pbs.sim`` or
  ``pbs.types``.
- Supports both L2 binary ``SimLog`` runs from ``pbs.sim.simulator.run_sim``
  and continuous staged-braking ``SimLog`` runs (either from future L3 policies
  or via the self-contained :func:`rollout_staged_ego` helper provided here).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import pandas as pd

from pbs.risk.assessors import RiskAssessor
from pbs.sim.collision import (
    VEHICLE_LENGTH,
    VEHICLE_WIDTH,
    in_collision,
    swept_contact_fraction,
    swept_min_distance,
)
from pbs.sim.observation import ObservationModel
from pbs.sim.simulator import MissingObservationError, _validate, ego_rollout
from pbs.types import RiskContext, Scenario, SimLog
from pbs.utils import history_until, path_heading, path_point, state_at

BRAKE_ACTIONS = frozenset({"brake", "soft_brake", "hard_brake"})
BRAKE_ACCEL_EPS = 0.05  # m/s² threshold for physical brake-light illumination


def _is_bool(value) -> bool:
    return isinstance(value, (bool, np.bool_))


def _require_finite(value: float, name: str) -> float:
    if _is_bool(value):
        raise ValueError(f"{name} must be a finite float, not bool.")
    try:
        out = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite float.") from exc
    if not math.isfinite(out):
        raise ValueError(f"{name} must be finite, got {value!r}.")
    return out


@dataclass(frozen=True)
class FollowerConfig:
    """Parameters of the human-driven following vehicle behind the ego."""

    init_rear_gap: float = 12.0  # initial bumper-to-bumper net headway g_0 [m]
    reaction_time: float = 1.0  # human perception-reaction delay tau_rear [s]
    max_decel: float = 6.0  # physical tire-road braking limit [m/s²]
    max_accel: float = 1.5  # mild recovery acceleration cap [m/s²]
    max_jerk: float | None = 10.0  # pedal/hydraulic jerk limit [m/s³]; None = step
    standstill_gap: float = 1.5  # minimum desired standstill buffer s_min [m]
    gap_gain: float = 0.25  # proportional gain on gap deficit below g_0 [s⁻²]
    ego_length: float = VEHICLE_LENGTH  # [m]
    follower_length: float = VEHICLE_LENGTH  # [m]
    substep_dt: float = 0.01  # continuous-time integration step [s]
    model: str = "kinematic"  # "kinematic" or "idm"
    truncate_at_front_collision: bool = True

    def __post_init__(self) -> None:
        gap = _require_finite(self.init_rear_gap, "init_rear_gap")
        tau = _require_finite(self.reaction_time, "reaction_time")
        decel = _require_finite(self.max_decel, "max_decel")
        accel = _require_finite(self.max_accel, "max_accel")
        standstill = _require_finite(self.standstill_gap, "standstill_gap")
        gain = _require_finite(self.gap_gain, "gap_gain")
        ego_l = _require_finite(self.ego_length, "ego_length")
        fol_l = _require_finite(self.follower_length, "follower_length")
        sub_dt = _require_finite(self.substep_dt, "substep_dt")
        if gap <= 0.0:
            raise ValueError(
                "init_rear_gap must be strictly positive (already in collision if <= 0)."
            )
        if tau < 0.0:
            raise ValueError("reaction_time must be nonnegative.")
        if decel <= 0.0:
            raise ValueError("max_decel must be positive.")
        if accel < 0.0:
            raise ValueError("max_accel must be nonnegative.")
        if standstill < 0.0 or gain < 0.0:
            raise ValueError("standstill_gap and gap_gain must be nonnegative.")
        if ego_l <= 0.0 or fol_l <= 0.0:
            raise ValueError("ego_length and follower_length must be positive.")
        if sub_dt <= 0.0 or sub_dt > 0.1:
            raise ValueError("substep_dt must be in (0, 0.1] seconds.")
        if self.max_jerk is not None:
            jerk = _require_finite(self.max_jerk, "max_jerk")
            if jerk <= 0.0:
                raise ValueError("max_jerk must be positive or None.")
        if self.model not in ("kinematic", "idm"):
            raise ValueError(
                f"Unknown follower model {self.model!r}; expected 'kinematic' or 'idm'."
            )
        if not _is_bool(self.truncate_at_front_collision):
            raise ValueError("truncate_at_front_collision must be a bool.")


@dataclass(frozen=True)
class StagedPolicyConfig:
    """L3 staged hysteresis + jerk-limited longitudinal braking specification.

    ``a_recover`` defaults to ``0.0`` so cruising holds speed identically to
    ``HysteresisPolicy`` (isolating staged braking + jerk limiting without
    post-conflict re-acceleration); pass ``a_recover > 0`` to enable speed recovery.
    """

    t_on_hard: float = 1.5
    t_off_hard: float = 2.0
    t_on_soft: float = 3.0
    t_off_soft: float = 3.5
    a_hard: float = 6.0  # m/s² magnitude
    a_soft: float = 2.5  # m/s² magnitude
    a_recover: float = 0.0  # m/s²
    max_jerk: float = 10.0  # m/s³

    def __post_init__(self) -> None:
        vals = (
            _require_finite(self.t_on_hard, "t_on_hard"),
            _require_finite(self.t_off_hard, "t_off_hard"),
            _require_finite(self.t_on_soft, "t_on_soft"),
            _require_finite(self.t_off_soft, "t_off_soft"),
            _require_finite(self.a_hard, "a_hard"),
            _require_finite(self.a_soft, "a_soft"),
            _require_finite(self.a_recover, "a_recover"),
            _require_finite(self.max_jerk, "max_jerk"),
        )
        t_on_h, t_off_h, t_on_s, t_off_s, a_h, a_s, a_rec, jerk = vals
        if not (0.0 < t_on_h < t_off_h <= t_on_s < t_off_s):
            raise ValueError(
                "Require 0 < t_on_hard < t_off_hard <= t_on_soft < t_off_soft."
            )
        if not (0.0 < a_s < a_h):
            raise ValueError("Require 0 < a_soft < a_hard.")
        if a_rec < 0.0 or jerk <= 0.0:
            raise ValueError("Require a_recover >= 0 and max_jerk > 0.")


@dataclass
class RearEndLog:
    """Detailed bidirectional outcome and trajectories of one coupled follower run."""

    t: np.ndarray
    ego_s: np.ndarray
    ego_v: np.ndarray
    ego_a: np.ndarray
    rear_s: np.ndarray
    rear_v: np.ndarray
    rear_a: np.ndarray
    bumper_gap: np.ndarray
    front_collision: bool
    first_front_collision_t: float | None
    rear_collision: bool
    first_rear_collision_t: float | None
    rear_impact_speed_diff: float
    min_bumper_gap: float
    rear_brake_onset_t: float | None
    rear_peak_decel: float
    ego_brake_onset_t: float | None
    ego_peak_decel: float
    ego_peak_jerk: float
    ego_speed_loss: float
    safe_both: bool
    outcome: str


def _validate_sim_log(log: SimLog) -> int:
    t = np.asarray(log.t, dtype=float)
    ego_x = np.asarray(log.ego_x, dtype=float)
    ego_y = np.asarray(log.ego_y, dtype=float)
    ego_v = np.asarray(log.ego_v, dtype=float)
    ego_a = np.asarray(log.ego_a, dtype=float)
    if t.ndim != 1 or len(t) < 2:
        raise ValueError("SimLog.t must be a 1-D array with at least 2 samples.")
    n = len(t)
    if any(arr.shape != (n,) for arr in (ego_x, ego_y, ego_v, ego_a)):
        raise ValueError("SimLog state arrays must all have matching 1-D shape.")
    if not all(np.isfinite(arr).all() for arr in (t, ego_x, ego_y, ego_v, ego_a)):
        raise ValueError("SimLog state arrays must be finite.")
    if np.any(np.diff(t) <= 0.0):
        raise ValueError("SimLog.t must be strictly increasing.")
    if np.any(ego_v < -1e-9):
        raise ValueError("SimLog.ego_v must be nonnegative.")
    if len(log.action) < n - 1:
        raise ValueError("SimLog.action must cover all executed intervals.")
    end = n - 1 if log.executed_steps is None else int(log.executed_steps)
    if end < 1 or end > n - 1:
        raise ValueError(f"Invalid executed_steps={log.executed_steps!r} for len(t)={n}.")
    return end


def _reconstruct_ego_profile(
    log: SimLog,
    substep_dt: float,
    *,
    truncate_at_front_collision: bool = True,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Interpolate ``SimLog`` onto a fine grid preserving exact within-step stopping."""
    exec_steps = _validate_sim_log(log)
    t_coarse = np.asarray(log.t[: exec_steps + 1], dtype=float)
    x_coarse = np.asarray(log.ego_x[: exec_steps + 1], dtype=float)
    y_coarse = np.asarray(log.ego_y[: exec_steps + 1], dtype=float)
    v_coarse = np.maximum(0.0, np.asarray(log.ego_v[: exec_steps + 1], dtype=float))
    a_coarse = np.asarray(log.ego_a[: exec_steps + 1], dtype=float)

    t0 = float(t_coarse[0])
    t_end = float(t_coarse[-1])
    if truncate_at_front_collision and log.collision and log.first_collision_t is not None:
        fc_t = float(log.first_collision_t)
        if not math.isfinite(fc_t) or fc_t < t0 - 1e-9:
            raise ValueError(f"Invalid first_collision_t={log.first_collision_t!r}.")
        t_end = min(t_end, max(t0, fc_t))

    if t_end <= t0 + 1e-12:
        return (
            np.array([t0], dtype=float),
            np.array([0.0], dtype=float),
            np.array([float(v_coarse[0])], dtype=float),
            np.array([float(a_coarse[0])], dtype=float),
        )

    n_fine = max(1, int(math.ceil((t_end - t0) / substep_dt - 1e-12)))
    t_fine = np.linspace(t0, t_end, n_fine + 1)

    chord_ds = np.hypot(np.diff(x_coarse), np.diff(y_coarse))
    s_knots = np.concatenate(([0.0], np.cumsum(chord_ds)))

    idx = np.clip(
        np.searchsorted(t_coarse, t_fine, side="right") - 1,
        0,
        len(t_coarse) - 2,
    )
    h = t_fine - t_coarse[idx]
    dt_k = t_coarse[idx + 1] - t_coarse[idx]
    s0 = s_knots[idx]
    v0 = v_coarse[idx]
    v1 = v_coarse[idx + 1]
    a0 = a_coarse[idx]
    total_ds = chord_ds[idx]

    decel_mask = (a0 < -1e-12) & (v0 > 0.0)
    ego_s = np.empty_like(t_fine)
    ego_v = np.empty_like(t_fine)
    ego_a = np.empty_like(t_fine)

    if np.any(decel_mask):
        h_d = h[decel_mask]
        dt_d = dt_k[decel_mask]
        v0_d = v0[decel_mask]
        a0_d = a0[decel_mask]
        ds_d = total_ds[decel_mask]
        t_to_zero = v0_d / (-a0_d)
        h_stop = np.minimum(dt_d, t_to_zero)
        eff_h = np.minimum(h_d, h_stop)
        kin_full = v0_d * h_stop + 0.5 * a0_d * h_stop * h_stop
        scale = np.where(kin_full > 1e-12, ds_d / np.maximum(kin_full, 1e-12), 1.0)
        ego_s[decel_mask] = s0[decel_mask] + scale * (v0_d * eff_h + 0.5 * a0_d * eff_h * eff_h)
        still_moving = h_d < t_to_zero - 1e-12
        ego_v[decel_mask] = np.where(still_moving, np.maximum(0.0, v0_d + a0_d * eff_h), 0.0)
        ego_a[decel_mask] = np.where(still_moving, a0_d, 0.0)

    other_mask = ~decel_mask
    if np.any(other_mask):
        h_o = h[other_mask]
        dt_o = dt_k[other_mask]
        v0_o = v0[other_mask]
        v1_o = v1[other_mask]
        a0_o = a0[other_mask]
        ds_o = total_ds[other_mask]
        frac = np.where(dt_o > 0.0, h_o / np.maximum(dt_o, 1e-12), 0.0)
        kin_full = v0_o * dt_o + 0.5 * a0_o * dt_o * dt_o
        ego_s[other_mask] = s0[other_mask] + np.where(
            kin_full > 1e-12,
            (ds_o / np.maximum(kin_full, 1e-12)) * (v0_o * h_o + 0.5 * a0_o * h_o * h_o),
            frac * ds_o,
        )
        ego_v[other_mask] = np.maximum(0.0, (1.0 - frac) * v0_o + frac * v1_o)
        ego_a[other_mask] = a0_o

    return t_fine, ego_s, ego_v, ego_a


def _ego_summary_metrics(
    log: SimLog,
    t_eval_end: float,
) -> tuple[float | None, float, float, float]:
    """Compute ego brake onset, peak decel, peak jerk, and speed loss up to ``t_eval_end``."""
    exec_steps = _validate_sim_log(log)
    mask = np.asarray(log.t[: exec_steps + 1], dtype=float) <= t_eval_end + 1e-9
    t_sub = np.asarray(log.t[: exec_steps + 1], dtype=float)[mask]
    v_sub = np.asarray(log.ego_v[: exec_steps + 1], dtype=float)[mask]
    a_sub = np.asarray(log.ego_a[: exec_steps + 1], dtype=float)[mask]
    act_len = min(exec_steps, max(0, len(t_sub) - 1))

    onset = None
    for k in range(act_len):
        ti = float(t_sub[k])
        if ti >= t_eval_end - 1e-9:
            break
        if log.action[k] in BRAKE_ACTIONS or float(a_sub[k]) < -BRAKE_ACCEL_EPS:
            onset = ti
            break

    executed_a = a_sub[:act_len] if act_len > 0 else a_sub[:1]
    peak_decel = float(max(0.0, float(np.max(-executed_a)))) if len(executed_a) else 0.0

    if len(executed_a) >= 2:
        dts = np.diff(t_sub[:act_len])
        jerks = np.abs(np.diff(executed_a, prepend=0.0)) / np.r_[t_sub[1] - t_sub[0], dts]
        peak_jerk = float(np.max(jerks))
    elif len(executed_a) == 1 and len(t_sub) >= 2:
        peak_jerk = float(abs(executed_a[0]) / float(t_sub[1] - t_sub[0]))
    else:
        peak_jerk = 0.0

    speed_loss = float(max(0.0, float(v_sub[0]) - float(np.min(v_sub)))) if len(v_sub) else 0.0
    return onset, peak_decel, peak_jerk, speed_loss


def _commanded_follower_accel(
    *,
    cfg: FollowerConfig,
    v0_init: float,
    delayed_gap: float,
    delayed_rear_v: float,
    delayed_ego_v: float,
    delayed_ego_a: float,
) -> float:
    """Compute target longitudinal acceleration of the human follower from delayed cues."""
    dv_closing = delayed_rear_v - delayed_ego_v  # > 0 means follower is closing in

    if cfg.model == "kinematic":
        ego_braking = delayed_ego_a < -BRAKE_ACCEL_EPS
        gap_deficit = max(0.0, cfg.init_rear_gap - delayed_gap)
        if not ego_braking and dv_closing <= 1e-9 and gap_deficit <= 1e-9:
            # Cruise equilibrium when ego has not braked and headway is unperturbed.
            if delayed_rear_v < v0_init - 1e-6 and delayed_gap > cfg.init_rear_gap:
                return min(cfg.max_accel, (v0_init - delayed_rear_v) / 1.0)
            return 0.0

        s_buf = min(cfg.standstill_gap, 0.45 * cfg.init_rear_gap)
        eff_gap = max(delayed_gap - s_buf, 0.2)
        a_match = min(0.0, delayed_ego_a) if ego_braking else 0.0
        a_kin = -(max(0.0, dv_closing) ** 2) / (2.0 * eff_gap)
        a_gap = -cfg.gap_gain * gap_deficit
        a_cmd = a_match + a_kin + a_gap
        return float(np.clip(a_cmd, -cfg.max_decel, cfg.max_accel))

    # Equilibrium-calibrated Delayed IDM:
    # Choose v_des and s_eq so that at (v = v0_init, dv = 0, gap = init_rear_gap), a_idm == 0.
    if v0_init <= 1e-9:
        return 0.0
    free_ratio_4 = 0.64  # (v0 / v_des)^4 = 0.64 -> (s_eq / g0)^2 = 0.36 -> s_eq = 0.6 * g0
    v_des = v0_init / (free_ratio_4 ** 0.25)
    s_eq = 0.6 * cfg.init_rear_gap
    b_comf = min(3.0, cfg.max_decel)
    a_acc = max(cfg.max_accel, 0.5)
    s_star = s_eq + max(
        0.0,
        delayed_rear_v * dv_closing / (2.0 * math.sqrt(a_acc * b_comf)),
    )
    eff_gap = max(delayed_gap, 0.1)
    a_idm = a_acc * (1.0 - (delayed_rear_v / v_des) ** 4 - (s_star / eff_gap) ** 2)
    if delayed_ego_a < -BRAKE_ACCEL_EPS:
        # Brake-light salience adds anticipatory matching when lead brakes hard.
        a_idm = min(a_idm, 0.7 * delayed_ego_a - (max(0.0, dv_closing) ** 2) / (2.0 * eff_gap))
    return float(np.clip(a_idm, -cfg.max_decel, cfg.max_accel))


def simulate_following_vehicle(
    log: SimLog,
    config: FollowerConfig | None = None,
    **overrides,
) -> RearEndLog:
    """Simulate a delayed human-driven following vehicle behind ``log``'s ego trajectory.

    Parameters
    ----------
    log:
        Closed-loop simulation output from ``run_sim`` (or :func:`rollout_staged_ego`).
    config:
        Optional :class:`FollowerConfig`; keyword ``overrides`` construct or update it.
    """
    if config is None:
        cfg = FollowerConfig(**overrides)
    elif overrides:
        merged = {field: getattr(config, field) for field in config.__dataclass_fields__}
        merged.update(overrides)
        cfg = FollowerConfig(**merged)
    else:
        cfg = config

    t, ego_s, ego_v, ego_a = _reconstruct_ego_profile(
        log,
        cfg.substep_dt,
        truncate_at_front_collision=cfg.truncate_at_front_collision,
    )
    n = len(t)
    half_sum = 0.5 * (cfg.ego_length + cfg.follower_length)
    v0 = float(ego_v[0])

    rear_s = np.empty(n, dtype=float)
    rear_v = np.empty(n, dtype=float)
    rear_a = np.zeros(n, dtype=float)
    bumper_gap = np.empty(n, dtype=float)

    rear_s[0] = float(ego_s[0]) - half_sum - cfg.init_rear_gap
    rear_v[0] = v0
    bumper_gap[0] = cfg.init_rear_gap

    ego_onset, ego_peak_decel, ego_peak_jerk, ego_speed_loss = _ego_summary_metrics(
        log, float(t[-1])
    )

    if n == 1:
        front_col = bool(log.collision)
        outcome = "front_only" if front_col else "safe_both"
        return RearEndLog(
            t=t,
            ego_s=ego_s,
            ego_v=ego_v,
            ego_a=ego_a,
            rear_s=rear_s,
            rear_v=rear_v,
            rear_a=rear_a,
            bumper_gap=bumper_gap,
            front_collision=front_col,
            first_front_collision_t=log.first_collision_t if front_col else None,
            rear_collision=False,
            first_rear_collision_t=None,
            rear_impact_speed_diff=0.0,
            min_bumper_gap=float(cfg.init_rear_gap),
            rear_brake_onset_t=None,
            rear_peak_decel=0.0,
            ego_brake_onset_t=ego_onset,
            ego_peak_decel=ego_peak_decel,
            ego_peak_jerk=ego_peak_jerk,
            ego_speed_loss=ego_speed_loss,
            safe_both=not front_col,
            outcome=outcome,
        )

    t0 = float(t[0])
    rear_collision = False
    first_rear_t: float | None = None
    impact_dv = 0.0
    min_gap_until_rear = float(bumper_gap[0])
    rear_brake_onset: float | None = None
    prev_a = 0.0

    for k in range(n - 1):
        dt_k = float(t[k + 1] - t[k])
        t_query = float(t[k]) - cfg.reaction_time
        if t_query < t0 - 1e-12:
            d_gap = cfg.init_rear_gap
            d_rv = v0
            d_ev = v0
            d_ea = 0.0
        else:
            tq = max(t0, t_query)
            j = int(np.clip(np.searchsorted(t[: k + 1], tq, side="right") - 1, 0, k))
            j_next = min(j + 1, k)
            denom_t = float(t[j_next] - t[j])
            w = (tq - float(t[j])) / denom_t if denom_t > 0.0 else 0.0
            d_gap = float((1.0 - w) * bumper_gap[j] + w * bumper_gap[j_next])
            d_rv = float((1.0 - w) * rear_v[j] + w * rear_v[j_next])
            d_ev = float((1.0 - w) * ego_v[j] + w * ego_v[j_next])
            d_ea = float(ego_a[j])

        a_target = _commanded_follower_accel(
            cfg=cfg,
            v0_init=v0,
            delayed_gap=d_gap,
            delayed_rear_v=d_rv,
            delayed_ego_v=d_ev,
            delayed_ego_a=d_ea,
        )
        if cfg.max_jerk is not None:
            max_da = cfg.max_jerk * dt_k
            a_applied = float(np.clip(a_target, prev_a - max_da, prev_a + max_da))
        else:
            a_applied = a_target
        a_applied = float(np.clip(a_applied, -cfg.max_decel, cfg.max_accel))

        v_curr = float(rear_v[k])
        if v_curr <= 1e-12 and a_applied < 0.0:
            a_applied = 0.0

        rear_a[k] = a_applied
        prev_a = a_applied
        if rear_brake_onset is None and a_applied < -BRAKE_ACCEL_EPS:
            rear_brake_onset = float(t[k])

        if a_applied < 0.0 and v_curr > 0.0:
            h_stop = min(dt_k, v_curr / (-a_applied))
            ds = v_curr * h_stop + 0.5 * a_applied * h_stop * h_stop
            v_next = max(0.0, v_curr + a_applied * h_stop)
        else:
            ds = v_curr * dt_k + 0.5 * a_applied * dt_k * dt_k
            v_next = max(0.0, v_curr + a_applied * dt_k)

        rear_s[k + 1] = float(rear_s[k]) + ds
        rear_v[k + 1] = v_next
        g0_step = float(bumper_gap[k])
        g1_step = float(ego_s[k + 1] - rear_s[k + 1]) - half_sum
        bumper_gap[k + 1] = g1_step

        if not rear_collision:
            if g1_step < -1e-12:
                rear_collision = True
                denom = g0_step - g1_step
                frac = float(np.clip(g0_step / denom, 0.0, 1.0)) if denom > 1e-15 else 0.0
                first_rear_t = float(t[k]) + frac * dt_k
                v_r_hit = (1.0 - frac) * v_curr + frac * v_next
                v_e_hit = (1.0 - frac) * float(ego_v[k]) + frac * float(ego_v[k + 1])
                impact_dv = float(max(0.0, v_r_hit - v_e_hit))
                min_gap_until_rear = 0.0
            else:
                min_gap_until_rear = min(min_gap_until_rear, max(0.0, g1_step))

    rear_a[-1] = rear_a[-2]
    front_col = bool(log.collision)
    safe_both = (not front_col) and (not rear_collision)
    if safe_both:
        outcome = "safe_both"
    elif front_col and rear_collision:
        outcome = "both_collisions"
    elif front_col:
        outcome = "front_only"
    else:
        outcome = "rear_only"

    rear_peak_decel = float(max(0.0, float(np.max(-rear_a[:-1]))))
    return RearEndLog(
        t=t,
        ego_s=ego_s,
        ego_v=ego_v,
        ego_a=ego_a,
        rear_s=rear_s,
        rear_v=rear_v,
        rear_a=rear_a,
        bumper_gap=bumper_gap,
        front_collision=front_col,
        first_front_collision_t=log.first_collision_t if front_col else None,
        rear_collision=rear_collision,
        first_rear_collision_t=first_rear_t,
        rear_impact_speed_diff=impact_dv,
        min_bumper_gap=float(min_gap_until_rear),
        rear_brake_onset_t=rear_brake_onset,
        rear_peak_decel=rear_peak_decel,
        ego_brake_onset_t=ego_onset,
        ego_peak_decel=ego_peak_decel,
        ego_peak_jerk=ego_peak_jerk,
        ego_speed_loss=ego_speed_loss,
        safe_both=safe_both,
        outcome=outcome,
    )


# --------------------------------------------------------------------------------------
# Self-contained L3 staged gentle braking rollout -> standard SimLog (zero sim edits)
# --------------------------------------------------------------------------------------
def _advance_continuous(
    v: float, a_cmd: float, dt: float, v_cap: float | None = None
) -> tuple[float, float]:
    """Integrate arc displacement and speed over ``dt`` with stop/cruise-cap clamping."""
    if abs(a_cmd) <= 1e-12:
        return v * dt, v
    if a_cmd < 0.0:
        if v <= 0.0:
            return 0.0, 0.0
        h = min(dt, v / (-a_cmd))
        return v * h + 0.5 * a_cmd * h * h, max(0.0, v + a_cmd * h)
    # Positive recovery acceleration capped at target cruising speed v_cap
    if v_cap is not None and v >= v_cap - 1e-12:
        return v * dt, v
    if v_cap is not None:
        h_cap = min(dt, (v_cap - v) / a_cmd)
        ds_acc = v * h_cap + 0.5 * a_cmd * h_cap * h_cap
        v_after = min(v_cap, v + a_cmd * h_cap)
        ds_rem = v_after * max(0.0, dt - h_cap)
        return ds_acc + ds_rem, v_after
    return v * dt + 0.5 * a_cmd * dt * dt, v + a_cmd * dt


def rollout_staged_ego(
    scenario: Scenario,
    assessor: RiskAssessor,
    staged: StagedPolicyConfig | None = None,
    *,
    duration: float = 19.0,
    horizon: int = 40,
    r_safe: float | None = None,
    latency: float = 0.0,
    collision_dt: float = 0.01,
    observation_model: ObservationModel | None = None,
) -> SimLog:
    """Produce a standard ``SimLog`` under staged gentle + emergency braking + jerk limit.

    Uses the exact read-only ``RiskContext`` and swept ground-truth collision primitives
    from ``pbs.sim`` without altering ``src/pbs/sim/simulator.py`` or ``policy.py``.
    """
    cfg = StagedPolicyConfig() if staged is None else staged
    dt = float(scenario.dt)
    steps, path = _validate(
        scenario, duration, horizon, r_safe, latency, collision_dt, cfg.a_hard
    )
    n = steps + 1
    t = np.arange(n) * dt

    assessor.bind(scenario)
    path_distances = np.concatenate(
        [[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    )
    path_delta = np.diff(path, axis=0)
    path_angles = np.arctan2(path_delta[:, 1], path_delta[:, 0])
    agent_angles = {a.agent_id: np.unwrap(a.heading) for a in scenario.agents}
    broad_limit = r_safe if r_safe is not None else np.hypot(VEHICLE_LENGTH, VEHICLE_WIDTH)

    def headings_at(arcs):
        indices = np.searchsorted(path_distances, arcs, side="right") - 1
        return path_angles[np.clip(indices, 0, len(path_angles) - 1)]

    ego_x, ego_y, ego_v, ego_a = (np.zeros(n) for _ in range(4))
    ttc = np.full(n, np.inf)
    action: list[str] = []

    s = 0.0
    v_target = float(scenario.ego_start[3])
    v = v_target
    prev_a = 0.0
    state = "cruise"
    min_dist, collision = np.inf, False
    first_collision_t = None
    min_until_contact = np.inf

    for k in range(n):
        ego_xy = path_point(path, s)
        ego_x[k], ego_y[k], ego_v[k] = ego_xy[0], ego_xy[1], v

        ctx = RiskContext(
            t_now=float(t[k]),
            ego_xy=ego_xy,
            ego_heading=path_heading(path, s),
            ego_v=v,
            ego_future=ego_rollout(scenario.ego_path, s, v, horizon, dt),
            ego_path=scenario.ego_path,
            lane_width=scenario.lane_width,
            horizon=horizon,
            dt=dt,
            r_safe=r_safe,
            ego_future_heading=headings_at(s + v * dt * np.arange(1, horizon + 1)),
        )
        for agent in scenario.agents:
            true_xy = state_at(agent, t[k])
            min_dist = min(min_dist, float(np.linalg.norm(ego_xy - true_xy)))
            overlap = in_collision(
                ego_xy,
                true_xy,
                r_safe,
                ego_heading=ctx.ego_heading,
                agent_heading=float(np.interp(t[k], agent.t, agent_angles[agent.agent_id])),
                pedestrian=agent.agent_type == "pedestrian",
            )
            collision |= overlap
            if overlap and first_collision_t is None:
                first_collision_t = float(t[k])
            history = history_until(agent, t[k] - latency)
            if not len(history.t):
                raise MissingObservationError(f"No observable history for {agent.agent_id!r}.")
            if observation_model is not None:
                reference_t = history.t.copy()
                history = observation_model.observe(history, t_now=float(t[k]))
                if history.agent_id != agent.agent_id or not np.array_equal(history.t, reference_t):
                    raise ValueError("Observation model must preserve agent ID and timestamps.")
            est = assessor.assess(ctx, history)
            if est.considered:
                if np.isnan(est.ttc) or est.ttc < 0:
                    raise ValueError(f"Invalid TTC from assessor for {agent.agent_id!r}.")
                ttc[k] = min(ttc[k], est.ttc)

        if first_collision_t is None or first_collision_t == t[k]:
            min_until_contact = min(min_until_contact, min_dist)
        if k == steps:
            action.append(action[-1] if action else "cruise")
            break

        # Three-stage hysteresis transition (ISO 15622 / L3 StagedBrakingPolicy spec)
        cur_ttc = float(ttc[k])
        if state == "hard_brake":
            if cur_ttc > cfg.t_off_soft:
                state = "cruise"
            elif cur_ttc > cfg.t_off_hard:
                state = "soft_brake"
        elif state == "soft_brake":
            if cur_ttc < cfg.t_on_hard:
                state = "hard_brake"
            elif cur_ttc > cfg.t_off_soft:
                state = "cruise"
        else:
            if cur_ttc < cfg.t_on_hard:
                state = "hard_brake"
            elif cur_ttc < cfg.t_on_soft:
                state = "soft_brake"
            else:
                state = "cruise"

        if state == "hard_brake":
            act_label = "hard_brake"
            a_des = -cfg.a_hard if v > 0.0 else 0.0
        elif state == "soft_brake":
            act_label = "soft_brake"
            a_des = -cfg.a_soft if v > 0.0 else 0.0
        elif v < v_target - 1e-6 and cfg.a_recover > 0.0:
            act_label = "recover"
            a_des = cfg.a_recover
        else:
            act_label = "cruise"
            a_des = 0.0

        max_da = cfg.max_jerk * dt
        a_cmd = float(np.clip(a_des, prev_a - max_da, prev_a + max_da))
        if v <= 0.0 and a_cmd < 0.0:
            a_cmd = 0.0
        if v >= v_target - 1e-9 and a_cmd > 0.0:
            a_cmd = 0.0

        action.append(act_label)
        ego_a[k] = a_cmd
        prev_a = a_cmd

        ds, next_v = _advance_continuous(v, a_cmd, dt, v_cap=v_target)
        if s + ds > path_distances[-1] + 1e-8:
            raise ValueError("ego_path ends before the next simulated position.")

        offsets = list(np.linspace(0.0, dt, int(np.ceil(dt / collision_dt)) + 1))
        if a_cmd < 0.0 and 0.0 < v / (-a_cmd) < dt:
            offsets.append(v / (-a_cmd))
        before_contact = first_collision_t is None
        intervals = []
        for agent in scenario.agents:
            knots = agent.t[(agent.t > t[k]) & (agent.t < t[k + 1])] - t[k]
            times = np.unique(np.concatenate([offsets, knots]))
            positions = [
                path_point(path, s + _advance_continuous(v, a_cmd, float(h), v_cap=v_target)[0])
                for h in times
            ]
            truth = [state_at(agent, t[k] + h) for h in times]
            mid = (times[:-1] + times[1:]) / 2.0
            ego_headings = headings_at(
                [
                    s + _advance_continuous(v, a_cmd, float(h), v_cap=v_target)[0]
                    for h in mid
                ]
            )
            other_headings = np.interp(t[k] + mid, agent.t, agent_angles[agent.agent_id])
            for j in range(len(times) - 1):
                distance = swept_min_distance(
                    positions[j], positions[j + 1], truth[j], truth[j + 1]
                )
                min_dist = min(min_dist, distance)
                if before_contact:
                    intervals.append(
                        (
                            times[j],
                            times[j + 1],
                            positions[j],
                            positions[j + 1],
                            truth[j],
                            truth[j + 1],
                            distance,
                        )
                    )
                if before_contact and distance <= broad_limit:
                    fraction = swept_contact_fraction(
                        positions[j],
                        positions[j + 1],
                        truth[j],
                        truth[j + 1],
                        r_safe,
                        ego_heading=float(ego_headings[j]),
                        agent_heading=float(other_headings[j]),
                        pedestrian=agent.agent_type == "pedestrian",
                    )
                    if fraction is None:
                        continue
                    collision = True
                    contact = float(t[k] + times[j] + fraction * (times[j + 1] - times[j]))
                    if first_collision_t is None or contact < first_collision_t:
                        first_collision_t = contact
        if before_contact and first_collision_t is not None:
            cutoff = first_collision_t - t[k]
            for start, end, p0, p1, q0, q1, distance in intervals:
                if start >= cutoff:
                    continue
                if end > cutoff:
                    f = (cutoff - start) / (end - start)
                    distance = swept_min_distance(p0, p0 + f * (p1 - p0), q0, q0 + f * (q1 - q0))
                min_until_contact = min(min_until_contact, distance)
        s += ds
        v = next_v

    return SimLog(
        t=t,
        ego_x=ego_x,
        ego_y=ego_y,
        ego_v=ego_v,
        ego_a=ego_a,
        action=action,
        ttc=ttc,
        min_dist=float(min_dist),
        collision=bool(collision),
        executed_steps=steps,
        first_collision_t=first_collision_t,
        min_dist_until_contact=min_until_contact,
    )


def summarize_bidirectional_logs(
    logs_by_variant: dict[str, list[tuple[str, str, SimLog]]],
    follower_config: FollowerConfig | None = None,
) -> pd.DataFrame:
    """Evaluate ``(scenario_id, label, SimLog)`` collections under a follower configuration.

    Returns one row per ``(variant, scenario_id)`` with front, rear, and joint safety metrics.
    """
    cfg = FollowerConfig() if follower_config is None else follower_config
    rows = []
    for variant, entries in logs_by_variant.items():
        for sid, label, sim_log in entries:
            res = simulate_following_vehicle(sim_log, cfg)
            rows.append(
                {
                    "variant": variant,
                    "scenario_id": sid,
                    "label": label,
                    "init_rear_gap": cfg.init_rear_gap,
                    "reaction_time": cfg.reaction_time,
                    "follower_model": cfg.model,
                    "front_collision": res.front_collision,
                    "rear_collision": res.rear_collision,
                    "safe_both": res.safe_both,
                    "outcome": res.outcome,
                    "min_bumper_gap": res.min_bumper_gap,
                    "rear_impact_speed_diff": res.rear_impact_speed_diff,
                    "ego_brake_onset_t": (
                        np.nan if res.ego_brake_onset_t is None else res.ego_brake_onset_t
                    ),
                    "rear_brake_onset_t": (
                        np.nan if res.rear_brake_onset_t is None else res.rear_brake_onset_t
                    ),
                    "ego_peak_decel": res.ego_peak_decel,
                    "ego_peak_jerk": res.ego_peak_jerk,
                    "ego_speed_loss": res.ego_speed_loss,
                    "rear_peak_decel": res.rear_peak_decel,
                }
            )
    return pd.DataFrame(rows)


def _simulate_follower_grid_for_log(
    log: SimLog,
    gap_vec: np.ndarray,
    tau_vec: np.ndarray,
    cfg: FollowerConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Vectorized batch evaluation of ``(gap_vec, tau_vec)`` pairs on a single ``SimLog``.

    Returns ``(rear_collision, min_bumper_gap, impact_dv)`` each of shape ``(M,)``.
    """
    t, ego_s, ego_v, ego_a = _reconstruct_ego_profile(
        log,
        cfg.substep_dt,
        truncate_at_front_collision=cfg.truncate_at_front_collision,
    )
    m = len(gap_vec)
    n = len(t)
    if n == 1:
        return np.zeros(m, dtype=bool), gap_vec.copy(), np.zeros(m, dtype=float)

    half_sum = 0.5 * (cfg.ego_length + cfg.follower_length)
    v0 = float(ego_v[0])
    cols = np.arange(m)

    gap_hist = np.empty((n, m), dtype=float)
    rv_hist = np.empty((n, m), dtype=float)

    rear_s = float(ego_s[0]) - half_sum - gap_vec
    rear_v = np.full(m, v0, dtype=float)
    gap_hist[0] = gap_vec
    rv_hist[0] = rear_v

    prev_a = np.zeros(m, dtype=float)
    rear_collision = np.zeros(m, dtype=bool)
    min_gap = gap_vec.copy()
    impact_dv = np.zeros(m, dtype=float)

    t0 = float(t[0])
    step_dt = float(t[1] - t[0])
    s_buf_vec = np.minimum(cfg.standstill_gap, 0.45 * gap_vec)

    for k in range(n - 1):
        dt_k = float(t[k + 1] - t[k])
        t_query = float(t[k]) - tau_vec
        pre_mask = t_query < t0 - 1e-12

        d_gap = gap_vec.copy()
        d_rv = np.full(m, v0, dtype=float)
        d_ev = np.full(m, v0, dtype=float)
        d_ea = np.zeros(m, dtype=float)

        if not np.all(pre_mask):
            act_cols = cols[~pre_mask]
            tq = np.maximum(t0, t_query[~pre_mask])
            u = (tq - t0) / step_dt
            j = np.clip(np.floor(u).astype(np.intp), 0, k)
            j = np.where((j < k) & (t[np.minimum(j + 1, k)] <= tq), j + 1, j)
            j = np.where((j > 0) & (t[j] > tq), j - 1, j)
            j_next = np.minimum(j + 1, k)
            denom_t = t[j_next] - t[j]
            w = np.where(
                denom_t > 0.0,
                np.clip((tq - t[j]) / np.maximum(denom_t, 1e-12), 0.0, 1.0),
                0.0,
            )
            d_gap[~pre_mask] = (1.0 - w) * gap_hist[j, act_cols] + w * gap_hist[j_next, act_cols]
            d_rv[~pre_mask] = (1.0 - w) * rv_hist[j, act_cols] + w * rv_hist[j_next, act_cols]
            d_ev[~pre_mask] = (1.0 - w) * ego_v[j] + w * ego_v[j_next]
            d_ea[~pre_mask] = ego_a[j]

        dv_closing = d_rv - d_ev
        if cfg.model == "kinematic":
            ego_braking = d_ea < -BRAKE_ACCEL_EPS
            gap_deficit = np.maximum(0.0, gap_vec - d_gap)
            equil = (~ego_braking) & (dv_closing <= 1e-9) & (gap_deficit <= 1e-9)
            eff_gap = np.maximum(d_gap - s_buf_vec, 0.2)
            a_match = np.where(ego_braking, np.minimum(0.0, d_ea), 0.0)
            a_kin = -(np.maximum(0.0, dv_closing) ** 2) / (2.0 * eff_gap)
            a_gap = -cfg.gap_gain * gap_deficit
            a_raw = np.clip(a_match + a_kin + a_gap, -cfg.max_decel, cfg.max_accel)
            a_rec = np.where(
                (d_rv < v0 - 1e-6) & (d_gap > gap_vec),
                np.minimum(cfg.max_accel, v0 - d_rv),
                0.0,
            )
            a_target = np.where(equil, a_rec, a_raw)
        else:
            if v0 <= 1e-9:
                a_target = np.zeros(m, dtype=float)
            else:
                v_des = v0 / (0.64 ** 0.25)
                s_eq = 0.6 * gap_vec
                b_comf = min(3.0, cfg.max_decel)
                a_acc = max(cfg.max_accel, 0.5)
                s_star = s_eq + np.maximum(
                    0.0, d_rv * dv_closing / (2.0 * math.sqrt(a_acc * b_comf))
                )
                eff_gap = np.maximum(d_gap, 0.1)
                a_idm = a_acc * (1.0 - (d_rv / v_des) ** 4 - (s_star / eff_gap) ** 2)
                lead_brake = 0.7 * d_ea - (np.maximum(0.0, dv_closing) ** 2) / (2.0 * eff_gap)
                a_idm = np.where(d_ea < -BRAKE_ACCEL_EPS, np.minimum(a_idm, lead_brake), a_idm)
                a_target = np.clip(a_idm, -cfg.max_decel, cfg.max_accel)

        if cfg.max_jerk is not None:
            max_da = cfg.max_jerk * dt_k
            a_applied = np.clip(a_target, prev_a - max_da, prev_a + max_da)
        else:
            a_applied = a_target
        a_applied = np.clip(a_applied, -cfg.max_decel, cfg.max_accel)
        a_applied = np.where((rear_v <= 1e-12) & (a_applied < 0.0), 0.0, a_applied)
        prev_a = a_applied

        dec = (a_applied < 0.0) & (rear_v > 0.0)
        h_stop = np.where(dec, np.minimum(dt_k, rear_v / np.maximum(-a_applied, 1e-12)), dt_k)
        ds = np.where(
            dec,
            rear_v * h_stop + 0.5 * a_applied * h_stop * h_stop,
            rear_v * dt_k + 0.5 * a_applied * dt_k * dt_k,
        )
        v_next = np.where(
            dec,
            np.maximum(0.0, rear_v + a_applied * h_stop),
            np.maximum(0.0, rear_v + a_applied * dt_k),
        )

        g0_step = gap_hist[k]
        rear_s = rear_s + ds
        g1_step = float(ego_s[k + 1]) - rear_s - half_sum
        gap_hist[k + 1] = g1_step
        rv_hist[k + 1] = v_next

        newly_hit = (~rear_collision) & (g1_step < -1e-12)
        if np.any(newly_hit):
            denom = g0_step[newly_hit] - g1_step[newly_hit]
            frac = np.where(
                denom > 1e-15,
                np.clip(g0_step[newly_hit] / np.maximum(denom, 1e-15), 0.0, 1.0),
                0.0,
            )
            v_r_hit = (1.0 - frac) * rear_v[newly_hit] + frac * v_next[newly_hit]
            v_e_hit = (1.0 - frac) * float(ego_v[k]) + frac * float(ego_v[k + 1])
            impact_dv[newly_hit] = np.maximum(0.0, v_r_hit - v_e_hit)
            min_gap[newly_hit] = 0.0
            rear_collision[newly_hit] = True

        still_clear = ~rear_collision
        if np.any(still_clear):
            min_gap[still_clear] = np.minimum(
                min_gap[still_clear], np.maximum(0.0, g1_step[still_clear])
            )
        rear_v = v_next

    return rear_collision, min_gap, impact_dv


def sweep_rear_headway_plane(
    sim_logs: dict[str, SimLog | list[SimLog]],
    rear_gaps: list[float] | tuple[float, ...],
    reaction_times: list[float] | tuple[float, ...],
    *,
    base_config: FollowerConfig | None = None,
) -> pd.DataFrame:
    """Evaluate rear-end & bidirectional safety across a ``(init_rear_gap, reaction_time)`` grid."""
    cfg0 = FollowerConfig() if base_config is None else base_config
    gaps = [float(g) for g in rear_gaps]
    taus = [float(tau) for tau in reaction_times]
    if not gaps or not taus:
        raise ValueError("rear_gaps and reaction_times must be non-empty.")
    # Validate every grid coordinate via FollowerConfig
    for g in gaps:
        FollowerConfig(**{**cfg0.__dict__, "init_rear_gap": g})
    for tau in taus:
        FollowerConfig(**{**cfg0.__dict__, "reaction_time": tau})

    gap_grid, tau_grid = np.meshgrid(gaps, taus, indexing="ij")
    gap_vec = gap_grid.ravel()
    tau_vec = tau_grid.ravel()
    m = len(gap_vec)

    rows = []
    for variant, item in sim_logs.items():
        logs = [item] if isinstance(item, SimLog) else list(item)
        if not logs:
            raise ValueError(f"Empty SimLog list for variant {variant!r}.")
        n_logs = len(logs)
        front_flags = np.array([bool(lg.collision) for lg in logs], dtype=bool)
        rear_mat = np.empty((n_logs, m), dtype=bool)
        gap_mat = np.empty((n_logs, m), dtype=float)
        dv_mat = np.empty((n_logs, m), dtype=float)

        for idx_log, lg in enumerate(logs):
            rc, mg, dv = _simulate_follower_grid_for_log(lg, gap_vec, tau_vec, cfg0)
            rear_mat[idx_log] = rc
            gap_mat[idx_log] = mg
            dv_mat[idx_log] = dv

        safe_mat = (~front_flags[:, None]) & (~rear_mat)
        front_rate = float(np.mean(front_flags))
        rear_rates = np.mean(rear_mat, axis=0)
        safe_rates = np.mean(safe_mat, axis=0)
        any_rear = np.any(rear_mat, axis=0)
        all_safe = np.all(safe_mat, axis=0)
        mean_gaps = np.mean(gap_mat, axis=0)
        max_dvs = np.max(dv_mat, axis=0)

        for idx_cell in range(m):
            rows.append(
                {
                    "variant": variant,
                    "init_rear_gap": float(gap_vec[idx_cell]),
                    "reaction_time": float(tau_vec[idx_cell]),
                    "n_scenarios": n_logs,
                    "front_collision_rate": front_rate,
                    "rear_collision_rate": float(rear_rates[idx_cell]),
                    "safe_both_rate": float(safe_rates[idx_cell]),
                    "any_rear_collision": bool(any_rear[idx_cell]),
                    "all_safe_both": bool(all_safe[idx_cell]),
                    "mean_min_bumper_gap": float(mean_gaps[idx_cell]),
                    "max_rear_impact_dv": float(max_dvs[idx_cell]),
                }
            )
    return pd.DataFrame(rows)

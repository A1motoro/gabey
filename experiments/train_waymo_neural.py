"""P-T2 — Train Option-B PhysicsAnchoredResMLP and run {128, 256, 512, 1024} verification.

Implements the architecture, two-stage loss, augmentation, 5-dimensional verification
protocol, and selection rule defined in ``docs/neural_predictor_option_b.md``:
- Stage 1 (Warmup, default 25 epochs): time-weighted Smooth L1 loss (1.2x on [0.5 s, 2.5 s]).
- Stage 2 (NLL Calibration, default 35 epochs): 2D Gaussian NLL + 0.05 monotonic sigma penalty.
- Data augmentation: 50% left-right mirror flip + 30% physically consistent observation noise.
- Capacity sweep: evaluates H_dim in ``--hidden-dims`` (default 128 256 512 1024) on:
  1. Held-out validation ADE/FDE at 2 s and 4 s
  2. Validation Gaussian NLL and 1-sigma lateral coverage calibration error
  3. Cross-domain synthetic generalization on A's 14 exp01 scenarios (23 origins each)
  4. Observation-noise robustness (pos_std=0.15 m, heading_std=0.03 rad, speed_std=0.25 m/s)
  5. Pure-NumPy single-step CPU latency (MLP-only & end-to-end) and .npz size

Supports both Waymo ``.npz`` datasets from A-T2 (``--train-npz``, ``--val-npz``) and an
unblocked diverse synthetic corpus (``--synthetic`` or automatic fallback when Waymo
``.npz`` files are absent).
"""

from __future__ import annotations

import os

# Avoid Windows Conda MKL + PyTorch libomp duplicate runtime abort.
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")

import argparse
import json
import shutil
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, TensorDataset

from pbs.data import ca_scenario_set, make_scenario, prediction_slices
from pbs.data.scenarios import quintic_lateral, quintic_lateral_vel
from pbs.prediction import ConstantVelocityPredictor
from pbs.prediction.features import (
    HISTORY_STEPS,
    LANE_FEATURE_DIM,
    TRAJ_POS_SCALE,
    TRAJ_VEL_SCALE,
    extract_lane_features,
    extract_traj_matrix,
    world_to_agent,
)
from pbs.prediction.neural import (
    LN_EPS,
    MODEL_DT,
    MODEL_HORIZON,
    SCHEMA_VERSION,
    SIGMA_MAX,
    SIGMA_MIN,
    NeuralPredictor,
    gaussian_nll_2d,
)
from pbs.types import Scenario, Trajectory
from pbs.utils import history_until

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRAIN_NPZ = ROOT / "data/waymo_train.npz"
DEFAULT_VAL_NPZ = ROOT / "data/waymo_val.npz"
DEFAULT_EXPORT_DIR = ROOT / "src/pbs/prediction/weights"
DEFAULT_MODELS_DIR = ROOT / "outputs/models"
DEFAULT_LOG_DIR = ROOT / "outputs/logs/l3_neural_capacity"
DEFAULT_FIG_DIR = ROOT / "outputs/figures/l3-predictors"

TARGET_1SIGMA_COV = 0.6827


class PhysicsAnchoredResMLP(nn.Module):
    """Exact Option-B §4.1 architecture with parameter count 4*H^2 + 251*H + 160 (D=80)."""

    def __init__(
        self,
        input_dim: int,
        hidden_dim: int = 128,
        *,
        n_res_blocks: int = 2,
        horizon: int = MODEL_HORIZON,
        dt: float = MODEL_DT,
        sigma_min: float = SIGMA_MIN,
        sigma_max: float = SIGMA_MAX,
    ):
        super().__init__()
        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.n_res_blocks = n_res_blocks
        self.horizon = horizon
        self.dt = dt
        self.sigma_min = sigma_min
        self.sigma_max = sigma_max

        tau = torch.arange(1, horizon + 1, dtype=torch.float32) * float(dt)
        self.register_buffer("tau", tau)

        self.in_fc = nn.Linear(input_dim, hidden_dim)
        self.in_ln = nn.LayerNorm(hidden_dim, eps=LN_EPS)

        self.blocks = nn.ModuleList()
        for _ in range(n_res_blocks):
            self.blocks.append(
                nn.ModuleDict(
                    {
                        "fc1": nn.Linear(hidden_dim, hidden_dim),
                        "ln": nn.LayerNorm(hidden_dim, eps=LN_EPS),
                        "fc2": nn.Linear(hidden_dim, hidden_dim),
                    }
                )
            )

        self.mean_head = nn.Linear(hidden_dim, 2 * horizon)
        self.std_head = nn.Linear(hidden_dim, 2 * horizon)

        nn.init.zeros_(self.mean_head.weight)
        nn.init.zeros_(self.mean_head.bias)
        nn.init.normal_(self.std_head.weight, mean=0.0, std=1e-3)
        nn.init.constant_(self.std_head.bias, -0.5)

    def forward(
        self, feats: torch.Tensor, v_local: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        h = F.silu(self.in_ln(self.in_fc(feats)))
        for blk in self.blocks:
            h = h + blk["fc2"](F.silu(blk["ln"](blk["fc1"](h))))
        delta_mu = self.mean_head(h).view(-1, self.horizon, 2)
        z_sigma = self.std_head(h).view(-1, self.horizon, 2)
        cv_anchor = self.tau[None, :, None] * v_local[:, None, :]
        mean_local = cv_anchor + delta_mu
        std_local = torch.clamp(
            F.softplus(z_sigma) + self.sigma_min, min=self.sigma_min, max=self.sigma_max
        )
        return mean_local, std_local

    def state_as_numpy_dict(self, *, use_lane: bool) -> dict[str, np.ndarray]:
        arrays: dict[str, np.ndarray] = {
            "schema_version": np.int32(SCHEMA_VERSION),
            "use_lane": np.int32(1 if use_lane else 0),
            "input_dim": np.int32(self.input_dim),
            "hidden_dim": np.int32(self.hidden_dim),
            "n_res_blocks": np.int32(self.n_res_blocks),
            "horizon": np.int32(self.horizon),
            "dt": np.float64(self.dt),
            "history_steps": np.int32(HISTORY_STEPS),
            "sigma_min": np.float64(self.sigma_min),
            "sigma_max": np.float64(self.sigma_max),
            "ln_eps": np.float64(LN_EPS),
            "in_w": self.in_fc.weight.detach().cpu().numpy().astype(np.float32),
            "in_b": self.in_fc.bias.detach().cpu().numpy().astype(np.float32),
            "in_ln_w": self.in_ln.weight.detach().cpu().numpy().astype(np.float32),
            "in_ln_b": self.in_ln.bias.detach().cpu().numpy().astype(np.float32),
            "mean_w": self.mean_head.weight.detach().cpu().numpy().astype(np.float32),
            "mean_b": self.mean_head.bias.detach().cpu().numpy().astype(np.float32),
            "std_w": self.std_head.weight.detach().cpu().numpy().astype(np.float32),
            "std_b": self.std_head.bias.detach().cpu().numpy().astype(np.float32),
        }
        for idx, blk in enumerate(self.blocks, start=1):
            arrays[f"b{idx}_w1"] = blk["fc1"].weight.detach().cpu().numpy().astype(np.float32)
            arrays[f"b{idx}_b1"] = blk["fc1"].bias.detach().cpu().numpy().astype(np.float32)
            arrays[f"b{idx}_ln_w"] = blk["ln"].weight.detach().cpu().numpy().astype(np.float32)
            arrays[f"b{idx}_ln_b"] = blk["ln"].bias.detach().cpu().numpy().astype(np.float32)
            arrays[f"b{idx}_w2"] = blk["fc2"].weight.detach().cpu().numpy().astype(np.float32)
            arrays[f"b{idx}_b2"] = blk["fc2"].bias.detach().cpu().numpy().astype(np.float32)
        return arrays

    def export_npz(self, path: Path, *, use_lane: bool, metadata: dict | None = None) -> None:
        arrays = self.state_as_numpy_dict(use_lane=use_lane)
        if metadata:
            arrays["metadata_json"] = np.array(json.dumps(metadata))
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, **arrays)


def _stage1_loss(mean: torch.Tensor, target: torch.Tensor, weights: torch.Tensor) -> torch.Tensor:
    per_step = F.smooth_l1_loss(mean, target, reduction="none").mean(dim=-1)
    return (per_step * weights[None, :]).mean()


def _stage2_loss(mean: torch.Tensor, std: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    diff = target - mean
    nll = (
        0.5 * (diff[:, :, 0] / std[:, :, 0]) ** 2
        + 0.5 * (diff[:, :, 1] / std[:, :, 1]) ** 2
        + torch.log(std[:, :, 0])
        + torch.log(std[:, :, 1])
    ).mean()
    mono_violation = F.relu(std[:, :-1, :] - std[:, 1:, :])
    mono_reg = 0.05 * (mono_violation**2).sum(dim=(1, 2)).mean()
    return nll + mono_reg


def _rotate_2d_torch(xy: torch.Tensor, angle: torch.Tensor) -> torch.Tensor:
    c = torch.cos(angle)
    s = torch.sin(angle)
    x_new = xy[..., 0] * c - xy[..., 1] * s
    y_new = xy[..., 0] * s + xy[..., 1] * c
    return torch.stack((x_new, y_new), dim=-1)


def _augment_batch(
    bx_traj: torch.Tensor,
    bx_lane: torch.Tensor | None,
    by: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor | None, torch.Tensor]:
    """Option-B §4.3 augmentations: 50% mirror flip + 30% physically consistent noise.

    Observation noise at t0 shifts past/future positions by -eps_0 and rotates
    the agent frame by -dtheta_0 while keeping the last frame invariant at
    (dx=0, dy=0, vy=0, cos=1, sin=0), matching ``extract_traj_matrix`` at inference.
    """
    traj = bx_traj.clone()
    lane = None if bx_lane is None else bx_lane.clone()
    y = by.clone()
    bsz = traj.shape[0]
    dev = traj.device

    # 1. 50% left-right mirror flip across +x axis
    flip = torch.rand(bsz, device=dev) < 0.5
    if flip.any():
        traj[flip, :, 1] = -traj[flip, :, 1]
        traj[flip, :, 3] = -traj[flip, :, 3]
        traj[flip, :, 5] = -traj[flip, :, 5]
        y[flip, :, 1] = -y[flip, :, 1]
        if lane is not None:
            lane[flip, 0] = -lane[flip, 0]
            lane[flip, 2] = -lane[flip, 2]
            lane[flip, 4] = -lane[flip, 4]
            lane[flip, 7::2] = -lane[flip, 7::2]

    # 2. 30% physically consistent sensor noise (pos 0.08 m, vel 0.15 m/s, heading 0.02 rad)
    noisy = torch.rand(bsz, device=dev) < 0.3
    if noisy.any():
        n_cnt = int(noisy.sum().item())
        eps_pos = torch.randn(n_cnt, HISTORY_STEPS, 2, device=dev) * 0.08
        eps0 = eps_pos[:, -1:, :]  # (n_cnt, 1, 2)
        dtheta0 = torch.randn(n_cnt, 1, device=dev) * 0.02

        # Past frames shift by (eps_k - eps_0), then rotate by -dtheta_0; t0 stays (0, 0).
        past_xy_m = traj[noisy, :-1, 0:2] * TRAJ_POS_SCALE + (eps_pos[:, :-1, :] - eps0)
        traj[noisy, :-1, 0:2] = _rotate_2d_torch(past_xy_m, -dtheta0) / TRAJ_POS_SCALE
        traj[noisy, -1, 0:2] = 0.0

        # Future GT shifts by -eps_0 and rotates by -dtheta_0 into the noisy observed t0 frame.
        y[noisy] = _rotate_2d_torch(y[noisy] - eps0, -dtheta0)

        # Speed jitter on past frames and longitudinal speed at t0 (vy at t0 remains 0).
        traj[noisy, :-1, 2:4] += (
            torch.randn(n_cnt, HISTORY_STEPS - 1, 2, device=dev) * (0.15 / TRAJ_VEL_SCALE)
        )
        traj[noisy, -1, 2] += torch.randn(n_cnt, device=dev) * (0.15 / TRAJ_VEL_SCALE)
        traj[noisy, -1, 3] = 0.0
        traj[noisy, -1, 4] = 1.0
        traj[noisy, -1, 5] = 0.0

        if lane is not None:
            lane[noisy, 0] += eps0[:, 0, 1] / 3.5
            lane[noisy, 1] += torch.randn(n_cnt, device=dev) * (0.15 / 20.0)
            lane[noisy, 2] += torch.randn(n_cnt, device=dev) * (0.15 / 5.0)
            c0 = torch.cos(-dtheta0[:, 0])
            s0 = torch.sin(-dtheta0[:, 0])
            old_c, old_s = lane[noisy, 3].clone(), lane[noisy, 4].clone()
            lane[noisy, 3] = old_c * c0 + old_s * s0
            lane[noisy, 4] = old_s * c0 - old_c * s0
            look = lane[noisy, 6:].view(n_cnt, -1, 2) * 40.0 - eps0
            lane[noisy, 6:] = (_rotate_2d_torch(look, -dtheta0) / 40.0).reshape(n_cnt, -1)

    return traj, lane, y


def _arc_straight_frame(s: np.ndarray, radius: float, max_sweep: float = 0.65 * np.pi):
    """C^1 circular arc capped at ``max_sweep`` rad followed by straight tangent exit."""
    r_abs = abs(radius)
    sign = 1.0 if radius >= 0.0 else -1.0
    s_arc_max = r_abs * max_sweep
    s_arr = np.asarray(s, dtype=float)
    s_clamped = np.minimum(s_arr, s_arc_max)
    s_extra = np.maximum(0.0, s_arr - s_arc_max)

    theta = sign * (s_clamped / r_abs)
    cx = radius * np.sin(theta)
    cy = radius * (1.0 - np.cos(theta))
    tx = np.cos(theta)
    ty = np.sin(theta)
    cx = cx + s_extra * tx
    cy = cy + s_extra * ty
    nx, ny = -ty, tx
    in_arc = s_arr <= s_arc_max
    return cx, cy, tx, ty, nx, ny, in_arc


def _make_curved_scenario(
    rng: np.random.Generator,
    *,
    radius: float,
    v_agent: float,
    a_lon: float,
    lat_start: float,
    lat_end: float,
    lc_start: float,
    lc_dur: float,
    lane_width: float,
    duration: float = 11.0,
    dt: float = MODEL_DT,
) -> Scenario:
    t = np.arange(0.0, duration + 0.5 * dt, dt)
    s_path = np.arange(0.0, max(260.0, v_agent * duration + 120.0), 1.0)
    px, py, _, _, _, _, _ = _arc_straight_frame(s_path, radius)
    ego_path = np.column_stack((px, py))

    v_t = np.maximum(1.0, v_agent + a_lon * np.clip(t - lc_start, 0.0, lc_dur))
    s_t = np.concatenate(([0.0], np.cumsum(0.5 * (v_t[:-1] + v_t[1:]) * dt))) + rng.uniform(
        10.0, 45.0
    )
    if abs(lat_end - lat_start) > 1e-6:
        d_lat = quintic_lateral(lat_start, lat_end, lc_dur, t, t0=lc_start)
        d_lat_dot = quintic_lateral_vel(lat_start, lat_end, lc_dur, t, t0=lc_start)
    else:
        d_lat = np.full_like(t, lat_start)
        d_lat_dot = np.zeros_like(t)

    cx, cy, tx, ty, nx, ny, in_arc = _arc_straight_frame(s_t, radius)
    x = cx + d_lat * nx
    y = cy + d_lat * ny
    scale = np.where(in_arc, np.maximum(0.2, 1.0 - d_lat / radius), 1.0)
    vx = v_t * scale * tx + d_lat_dot * nx
    vy = v_t * scale * ty + d_lat_dot * ny
    speed = np.hypot(vx, vy)
    heading = np.unwrap(np.arctan2(vy, vx))
    agent = Trajectory(t, x, y, speed, heading, agent_type="vehicle", agent_id="curved_agent")
    return Scenario(
        ego_start=(0.0, 0.0, 0.0, float(v_agent)),
        ego_path=ego_path,
        agents=[agent],
        dt=dt,
        kind="curved",
        lane_width=float(lane_width),
        params={"duration": duration, "radius": radius},
    )


def _make_abort_scenario(
    rng: np.random.Generator,
    *,
    v_agent: float,
    lane_width: float,
    from_left: bool,
    peak_shift: float,
    t_start: float,
    half_dur: float,
    duration: float = 11.0,
    dt: float = MODEL_DT,
) -> Scenario:
    t = np.arange(0.0, duration + 0.5 * dt, dt)
    sign = 1.0 if from_left else -1.0
    y0 = sign * lane_width
    y_mid = sign * (lane_width - peak_shift)
    y_in = quintic_lateral(y0, y_mid, half_dur, t, t0=t_start)
    vy_in = quintic_lateral_vel(y0, y_mid, half_dur, t, t0=t_start)
    y_out = quintic_lateral(y_mid, y0, half_dur, t, t0=t_start + half_dur)
    vy_out = quintic_lateral_vel(y_mid, y0, half_dur, t, t0=t_start + half_dur)
    first_half = t <= t_start + half_dur
    y = np.where(first_half, y_in, y_out)
    vy = np.where(first_half, vy_in, vy_out)
    x = rng.uniform(12.0, 40.0) + v_agent * t
    vx = np.full_like(t, v_agent)
    agent = Trajectory(
        t, x, y, np.hypot(vx, vy), np.arctan2(vy, vx), agent_type="vehicle", agent_id="abort_agent"
    )
    ego_path = np.array([[0.0, 0.0], [max(250.0, v_agent * duration + 100.0), 0.0]])
    return Scenario(
        ego_start=(0.0, 0.0, 0.0, float(v_agent)),
        ego_path=ego_path,
        agents=[agent],
        dt=dt,
        kind="abort_cut_in",
        lane_width=float(lane_width),
        params={"duration": duration},
    )


def _make_cruise_scenario(
    rng: np.random.Generator, *, lane_width: float, duration: float = 11.0, dt: float = MODEL_DT
) -> Scenario:
    t = np.arange(0.0, duration + 0.5 * dt, dt)
    v_agent = float(rng.uniform(4.0, 22.0))
    lane_idx = int(rng.choice([-1, 0, 1]))
    y0 = lane_idx * lane_width + float(rng.uniform(-0.35, 0.35))
    x0 = float(rng.uniform(8.0, 60.0))
    a_gentle = float(rng.uniform(-0.8, 0.8))
    v_t = np.maximum(0.5, v_agent + a_gentle * np.clip(t - 1.0, 0.0, 3.0))
    x = x0 + np.concatenate(([0.0], np.cumsum(0.5 * (v_t[:-1] + v_t[1:]) * dt)))
    y = np.full_like(t, y0)
    agent = Trajectory(
        t, x, y, v_t, np.zeros_like(t), agent_type="vehicle", agent_id="cruise_agent"
    )
    ego_path = np.array([[0.0, 0.0], [max(260.0, 25.0 * duration + 100.0), 0.0]])
    return Scenario(
        ego_start=(0.0, 0.0, 0.0, 15.0),
        ego_path=ego_path,
        agents=[agent],
        dt=dt,
        kind="cruise",
        lane_width=float(lane_width),
        params={"duration": duration},
    )


def _extract_windows_from_scenario(
    sc: Scenario,
    *,
    stride_steps: int = 4,
    horizon: int = MODEL_HORIZON,
) -> tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray]]:
    agent = sc.agents[0]
    n = len(agent.t)
    hists, lanes, futures = [], [], []
    for idx in range(HISTORY_STEPS - 1, n - horizon, stride_steps):
        sl = slice(idx - HISTORY_STEPS + 1, idx + 1)
        hist = Trajectory(
            agent.t[sl],
            agent.x[sl],
            agent.y[sl],
            agent.v[sl],
            agent.heading[sl],
            agent.agent_type,
            agent.agent_id,
        )
        traj_mat = extract_traj_matrix(hist, dt=sc.dt, steps=HISTORY_STEPS, normalize=True)
        lane_vec = extract_lane_features(hist, sc.ego_path, lane_width=sc.lane_width)
        origin = np.array([agent.x[idx], agent.y[idx]], dtype=float)
        theta0 = float(agent.heading[idx])
        fut_world = np.column_stack(
            (agent.x[idx + 1 : idx + 1 + horizon], agent.y[idx + 1 : idx + 1 + horizon])
        )
        fut_local = world_to_agent(fut_world, origin, theta0).astype(np.float32)
        hists.append(traj_mat)
        lanes.append(lane_vec)
        futures.append(fut_local)
    return hists, lanes, futures


def build_synthetic_corpus(
    seed: int = 42,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Build diverse scenario-split synthetic train/val sets (80/20 by scenario ID)."""
    rng = np.random.default_rng(seed)
    scenarios: list[Scenario] = []

    for _ in range(180):
        lane_w = float(rng.uniform(3.2, 3.9))
        v_ego = float(rng.uniform(10.0, 22.0))
        rel_v = float(rng.uniform(-3.0, min(7.0, v_ego - 3.0)))
        lc_t = float(rng.uniform(2.8, 5.2))
        cut_in_t = float(rng.uniform(0.3, 3.8))
        a_cut = float(rng.uniform(-2.5, 2.5))
        from_left = bool(rng.integers(0, 2))
        gap_d = float(rng.uniform(12.0, 50.0))
        try:
            sc = make_scenario(
                "cut_in",
                gap_d=gap_d,
                rel_v=rel_v,
                cut_in_t=cut_in_t,
                lc_T=lc_t,
                a_cutter=a_cut,
                v_ego=v_ego,
                lane_width=lane_w,
                from_left=from_left,
                duration=11.0,
            )
        except ValueError:
            continue
        scenarios.append(sc)

    for _ in range(90):
        v_ego = float(rng.uniform(9.0, 22.0))
        rel_v = float(rng.uniform(-2.0, min(5.0, v_ego - 3.0)))
        decel = float(rng.uniform(0.6, 6.8))
        t_brake = float(rng.uniform(0.4, 3.8))
        init_gap = float(rng.uniform(15.0, 70.0))
        lane_w = float(rng.uniform(3.2, 3.9))
        scenarios.append(
            make_scenario(
                "lead_brake",
                init_gap=init_gap,
                decel=decel,
                t_brake=t_brake,
                rel_v=rel_v,
                v_ego=v_ego,
                lane_width=lane_w,
                duration=11.0,
            )
        )

    for _ in range(50):
        lat_v = float(rng.uniform(0.8, 2.8))
        arr_t = float(rng.uniform(1.5, 5.5))
        gap_d = float(rng.uniform(18.0, 55.0))
        sc = make_scenario(
            "crossing",
            gap_d=gap_d,
            arrival_t=arr_t,
            lateral_v=lat_v,
            lane_width=float(rng.uniform(3.2, 3.9)),
            duration=11.0,
        )
        if rng.random() < 0.5:
            ag = sc.agents[0]
            ag.y = -ag.y
            ag.heading = -ag.heading
        scenarios.append(sc)

    for _ in range(80):
        scenarios.append(_make_cruise_scenario(rng, lane_width=float(rng.uniform(3.2, 3.9))))

    for _ in range(50):
        lw = float(rng.uniform(3.2, 3.9))
        scenarios.append(
            _make_abort_scenario(
                rng,
                v_agent=float(rng.uniform(9.0, 20.0)),
                lane_width=lw,
                from_left=bool(rng.integers(0, 2)),
                peak_shift=float(rng.uniform(0.8, 1.7)),
                t_start=float(rng.uniform(0.5, 2.5)),
                half_dur=float(rng.uniform(1.4, 2.4)),
            )
        )

    for _ in range(90):
        lw = float(rng.uniform(3.2, 3.9))
        radius = float(rng.choice([-1.0, 1.0]) * rng.uniform(45.0, 240.0))
        v_ag = float(rng.uniform(8.0, 18.0))
        a_lon = float(rng.uniform(-1.5, 1.5))
        if rng.random() < 0.5:
            side = float(rng.choice([-1.0, 1.0]))
            lat_start, lat_end = side * lw, 0.0
        else:
            lat_start = float(rng.choice([-1.0, 0.0, 1.0])) * lw
            lat_end = lat_start
        scenarios.append(
            _make_curved_scenario(
                rng,
                radius=radius,
                v_agent=v_ag,
                a_lon=a_lon,
                lat_start=lat_start,
                lat_end=lat_end,
                lc_start=float(rng.uniform(0.6, 3.0)),
                lc_dur=float(rng.uniform(3.0, 4.8)),
                lane_width=lw,
            )
        )

    perm = rng.permutation(len(scenarios))
    n_val = max(1, int(round(0.2 * len(scenarios))))
    val_idx = set(perm[:n_val].tolist())

    def collect(indices: list[int]) -> dict[str, np.ndarray]:
        h_all, l_all, f_all = [], [], []
        for idx in indices:
            h, ln, fu = _extract_windows_from_scenario(
                scenarios[idx], stride_steps=4, horizon=MODEL_HORIZON
            )
            h_all.extend(h)
            l_all.extend(ln)
            f_all.extend(fu)
        return {
            "hist_traj": np.stack(h_all).astype(np.float32),
            "lane_ctx": np.stack(l_all).astype(np.float32),
            "future_xy": np.stack(f_all).astype(np.float32),
        }

    train_indices = [i for i in range(len(scenarios)) if i not in val_idx]
    val_indices = sorted(val_idx)
    return collect(train_indices), collect(val_indices)


def _infer_is_unnormalized(hist: np.ndarray, future: np.ndarray) -> bool:
    """Speed-invariant physical ratio check: |future[:,0,0]| / |hist[:,-1,2]| is ~0.1 vs ~2.0."""
    vx = np.abs(hist[:, -1, 2].astype(np.float64))
    dx1 = np.abs(future[:, 0, 0].astype(np.float64))
    moving = (vx > 1e-4) & (dx1 > 0.05)
    if np.any(moving):
        ratio = float(np.median(dx1[moving] / vx[moving]))
        return ratio < 0.5
    return float(np.max(np.abs(hist[:, :, :4]))) > 3.0


def load_npz_dataset(path: Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as z:
        hist = np.asarray(z["hist_traj"], dtype=np.float32).copy()
        lane = np.asarray(z["lane_ctx"], dtype=np.float32).copy()
        future = np.asarray(z["future_xy"], dtype=np.float32).copy()
        normalized = bool(int(z["traj_normalized"])) if "traj_normalized" in z else None

    n = len(hist)
    if (
        n == 0
        or hist.shape != (n, HISTORY_STEPS, 6)
        or lane.shape != (n, LANE_FEATURE_DIM)
        or future.shape != (n, MODEL_HORIZON, 2)
    ):
        raise ValueError(f"Dataset {path} must be non-empty and match Contract 1 shapes")
    if not (np.isfinite(hist).all() and np.isfinite(lane).all() and np.isfinite(future).all()):
        raise ValueError(f"Dataset {path} contains non-finite values")

    if normalized is False or (normalized is None and _infer_is_unnormalized(hist, future)):
        hist[:, :, 0:2] /= TRAJ_POS_SCALE
        hist[:, :, 2:4] /= TRAJ_VEL_SCALE

    return {"hist_traj": hist, "lane_ctx": lane, "future_xy": future}


def _evaluate_exp01_synthetic(
    predictor: NeuralPredictor,
    *,
    pos_noise_std: float = 0.0,
    heading_noise_std: float = 0.0,
    speed_noise_std: float = 0.0,
    seed: int = 0,
) -> tuple[float, float]:
    """Evaluate 2 s ADE and FDE on A's exact 14 exp01 scenarios (23 origins each)."""
    scenarios = ca_scenario_set().scenarios + [
        make_scenario("lead_brake"),
        make_scenario("crossing"),
    ]
    rng = np.random.default_rng(seed)
    ades, fdes = [], []
    n_2s = 20
    for sc in scenarios:
        predictor.bind(sc)
        for sl in prediction_slices(sc, history=1.0, horizon=3.0, stride=0.5):
            h = sl.history
            if pos_noise_std > 0.0 or heading_noise_std > 0.0 or speed_noise_std > 0.0:
                h = Trajectory(
                    h.t.copy(),
                    h.x + rng.normal(0.0, pos_noise_std, size=len(h.x)),
                    h.y + rng.normal(0.0, pos_noise_std, size=len(h.y)),
                    np.maximum(0.0, h.v + rng.normal(0.0, speed_noise_std, size=len(h.v))),
                    h.heading + rng.normal(0.0, heading_noise_std, size=len(h.heading)),
                    h.agent_type,
                    h.agent_id,
                )
            pred = predictor.predict(h, len(sl.truth), sc.dt)
            err = np.linalg.norm(pred[:n_2s] - sl.truth[:n_2s], axis=1)
            ades.append(float(err.mean()))
            fdes.append(float(err[-1]))
    return float(np.mean(ades)), float(np.mean(fdes))


def _cv_exp01_ade_2s() -> float:
    scenarios = ca_scenario_set().scenarios + [
        make_scenario("lead_brake"),
        make_scenario("crossing"),
    ]
    cv = ConstantVelocityPredictor()
    ades = []
    for sc in scenarios:
        for sl in prediction_slices(sc, history=1.0, horizon=3.0, stride=0.5):
            pred = cv.predict(sl.history, len(sl.truth), sc.dt)
            ades.append(float(np.linalg.norm(pred[:20] - sl.truth[:20], axis=1).mean()))
    return float(np.mean(ades))


def _benchmark_latency_ms(
    predictor: NeuralPredictor, input_dim: int, iters: int
) -> tuple[float, float]:
    """Return (mlp_only_ms, end_to_end_predict_ms) averaged over ``iters`` single-step calls."""
    rng = np.random.default_rng(0)
    feat = rng.standard_normal(input_dim).astype(np.float32)
    anchor = np.zeros((MODEL_HORIZON, 2), dtype=np.float64)
    for _ in range(50):
        predictor._forward_single(feat, anchor)
    t0 = time.perf_counter()
    for _ in range(iters):
        predictor._forward_single(feat, anchor)
    mlp_ms = (time.perf_counter() - t0) * 1000.0 / max(1, iters)

    sc = make_scenario("cut_in", gap_d=25.0, rel_v=2.0)
    hist = history_until(sc.agents[0], 1.5)
    predictor.bind(sc)
    for _ in range(20):
        predictor.predict(hist, MODEL_HORIZON, MODEL_DT)
    e2e_iters = max(200, iters // 2)
    t1 = time.perf_counter()
    for _ in range(e2e_iters):
        predictor.predict(hist, MODEL_HORIZON, MODEL_DT)
    e2e_ms = (time.perf_counter() - t1) * 1000.0 / e2e_iters
    return float(mlp_ms), float(e2e_ms)


def train_and_verify_one(
    train_data: dict[str, np.ndarray],
    val_data: dict[str, np.ndarray],
    *,
    use_lane: bool,
    hidden_dim: int,
    output_path: Path,
    epochs_warmup: int,
    epochs_nll: int,
    batch_size: int,
    lr: float,
    seed: int,
    device: str,
    data_source: str,
    bench_iters: int,
    cv_synth_ade_2s: float,
) -> dict:
    torch.manual_seed(seed)
    np.random.seed(seed)

    flat_traj_dim = HISTORY_STEPS * 6
    input_dim = flat_traj_dim + (LANE_FEATURE_DIM if use_lane else 0)
    model = PhysicsAnchoredResMLP(
        input_dim,
        hidden_dim=hidden_dim,
        n_res_blocks=2,
        horizon=MODEL_HORIZON,
        dt=MODEL_DT,
        sigma_min=SIGMA_MIN,
        sigma_max=SIGMA_MAX,
    ).to(device)
    n_params = int(sum(p.numel() for p in model.parameters()))

    step_weights = torch.ones(MODEL_HORIZON, dtype=torch.float32, device=device)
    step_weights[4:25] = 1.2

    train_ds = TensorDataset(
        torch.from_numpy(train_data["hist_traj"]),
        torch.from_numpy(train_data["lane_ctx"]),
        torch.from_numpy(train_data["future_xy"]),
    )
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True, drop_last=False)

    val_traj_t = torch.from_numpy(val_data["hist_traj"]).to(device)
    val_lane_t = torch.from_numpy(val_data["lane_ctx"]).to(device)
    val_y_t = torch.from_numpy(val_data["future_xy"]).to(device)
    val_v_t = val_traj_t[:, -1, 2:4] * TRAJ_VEL_SCALE
    val_flat = val_traj_t.reshape(len(val_traj_t), flat_traj_dim)
    val_feats_t = torch.cat((val_flat, val_lane_t), dim=1) if use_lane else val_flat

    total_epochs = epochs_warmup + epochs_nll
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=max(1, total_epochs))

    best_val_loss = float("inf")
    best_state = None
    best_epoch = 0
    t_start = time.perf_counter()

    for epoch in range(1, total_epochs + 1):
        model.train()
        for b_traj, b_lane, b_y in train_loader:
            b_traj = b_traj.to(device)
            b_lane = b_lane.to(device) if use_lane else None
            b_y = b_y.to(device)
            b_traj, b_lane, b_y = _augment_batch(b_traj, b_lane, b_y)

            b_v = b_traj[:, -1, 2:4] * TRAJ_VEL_SCALE
            b_flat = b_traj.reshape(len(b_traj), flat_traj_dim)
            b_feats = (
                torch.cat((b_flat, b_lane), dim=1) if use_lane and b_lane is not None else b_flat
            )
            optimizer.zero_grad(set_to_none=True)
            mu, std = model(b_feats, b_v)
            loss = (
                _stage1_loss(mu, b_y, step_weights)
                if epoch <= epochs_warmup
                else _stage2_loss(mu, std, b_y)
            )
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            optimizer.step()
        scheduler.step()

        model.eval()
        with torch.no_grad():
            v_mu, v_std = model(val_feats_t, val_v_t)
            val_loss = float(
                _stage1_loss(v_mu, val_y_t, step_weights).item()
                if epochs_nll == 0
                else _stage2_loss(v_mu, v_std, val_y_t).item()
            )
            if (epoch > epochs_warmup or epochs_nll == 0) and val_loss < best_val_loss:
                best_val_loss = val_loss
                best_epoch = epoch
                best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}

    if best_state is not None:
        model.load_state_dict(best_state)
    model.eval()
    train_time_s = time.perf_counter() - t_start

    with torch.no_grad():
        val_mu_pt, val_std_pt = model(val_feats_t, val_v_t)
        val_mu_pt_np = val_mu_pt.cpu().numpy().astype(np.float64)
        val_std_pt_np = val_std_pt.cpu().numpy().astype(np.float64)

    metadata = {
        "data_source": data_source,
        "use_lane": use_lane,
        "hidden_dim": int(hidden_dim),
        "n_params": n_params,
        "n_train": int(len(train_data["hist_traj"])),
        "n_val": int(len(val_data["hist_traj"])),
        "best_epoch": int(best_epoch),
        "seed": int(seed),
    }
    model.export_npz(output_path, use_lane=use_lane, metadata=metadata)

    # Pure-NumPy parity check
    np_predictor = NeuralPredictor(output_path, use_lane=use_lane)
    val_mu_np, val_std_np = np_predictor.forward_batch(
        val_data["hist_traj"],
        val_data["lane_ctx"] if use_lane else None,
        traj_normalized=True,
    )
    max_diff = max(
        float(np.max(np.abs(val_mu_np - val_mu_pt_np))),
        float(np.max(np.abs(val_std_np - val_std_pt_np))),
    )
    if max_diff > 1e-3:
        raise RuntimeError(f"PyTorch/NumPy parity mismatch: max_diff={max_diff}")

    # 5-dimensional verification metrics (Option-B spec §4.3)
    gt = val_data["future_xy"].astype(np.float64)
    disp = np.linalg.norm(val_mu_np - gt, axis=-1)
    val_ade_2s = float(disp[:, :20].mean())
    val_fde_2s = float(disp[:, 19].mean())
    val_ade_4s = float(disp.mean())
    val_fde_4s = float(disp[:, -1].mean())
    val_nll = float(gaussian_nll_2d(val_mu_np, val_std_np, gt).mean())
    lat_err = np.abs(val_mu_np[:, :, 1] - gt[:, :, 1])
    cov_1sigma_lat = float(np.mean(lat_err <= val_std_np[:, :, 1]))
    cov_1sigma_err = abs(cov_1sigma_lat - TARGET_1SIGMA_COV)

    synth_ade_2s, synth_fde_2s = _evaluate_exp01_synthetic(np_predictor)
    # Match C's GaussianObservationModel(pos_std=0.15, heading_std=0.03, speed_std=0.25).
    noisy_ade_2s, _ = _evaluate_exp01_synthetic(
        np_predictor,
        pos_noise_std=0.15,
        heading_noise_std=0.03,
        speed_noise_std=0.25,
        seed=seed,
    )

    mlp_latency_ms, e2e_latency_ms = _benchmark_latency_ms(np_predictor, input_dim, bench_iters)
    size_mb = float(output_path.stat().st_size / (1024.0 * 1024.0))

    score = 0.5 * val_ade_2s + 0.3 * val_ade_4s + 0.2 * synth_ade_2s
    eligible = (0.60 <= cov_1sigma_lat <= 0.78) and (synth_ade_2s < cv_synth_ade_2s)

    return {
        "variant": "nn_lane" if use_lane else "nn_traj",
        "hidden_dim": int(hidden_dim),
        "n_params": n_params,
        "size_mb": round(size_mb, 4),
        "mlp_latency_ms": round(mlp_latency_ms, 5),
        "latency_ms": round(e2e_latency_ms, 5),
        "val_ade_2s": round(val_ade_2s, 4),
        "val_fde_2s": round(val_fde_2s, 4),
        "val_ade_4s": round(val_ade_4s, 4),
        "val_fde_4s": round(val_fde_4s, 4),
        "val_nll": round(val_nll, 4),
        "cov_1sigma_lat": round(cov_1sigma_lat, 4),
        "cov_1sigma_err": round(cov_1sigma_err, 4),
        "synth_ade_2s": round(synth_ade_2s, 4),
        "synth_fde_2s": round(synth_fde_2s, 4),
        "cv_synth_ade_2s": round(cv_synth_ade_2s, 4),
        "noisy_ade_2s": round(noisy_ade_2s, 4),
        "score": round(score, 4),
        "eligible": bool(eligible),
        "best_epoch": int(best_epoch),
        "train_time_s": round(train_time_s, 2),
        "numpy_parity_max_diff": round(max_diff, 7),
        "checkpoint": str(output_path),
    }


def select_best_candidate(rows: list[dict], *, max_size_mb: float = 0.0) -> dict:
    """Apply Option-B §4.3 selection rule with 1.5% Occam's-razor tie-breaker."""
    sized = [r for r in rows if max_size_mb <= 0.0 or r["size_mb"] <= max_size_mb]
    candidates = sized if sized else rows
    eligible = [r for r in candidates if r["eligible"]]
    pool = eligible if eligible else candidates
    min_score = min(r["score"] for r in pool)
    near_best = [r for r in pool if (r["score"] - min_score) / max(min_score, 1e-9) < 0.015]
    return min(near_best, key=lambda r: (r["n_params"], r["score"]))


def plot_capacity_verification(df: pd.DataFrame, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.2))
    for variant, grp in df.groupby("variant"):
        grp = grp.sort_values("hidden_dim")
        dims = grp["hidden_dim"].astype(str)
        axes[0].plot(dims, grp["val_ade_2s"], "o-", label=f"{variant} 2s")
        axes[0].plot(dims, grp["val_ade_4s"], "s--", label=f"{variant} 4s")
        axes[1].plot(dims, grp["cov_1sigma_lat"] * 100.0, "o-", label=variant)
        axes[2].plot(dims, grp["synth_ade_2s"], "o-", label=f"{variant} clean")
        axes[2].plot(dims, grp["noisy_ade_2s"], "s--", label=f"{variant} noisy(0.15m)")
        axes[3].plot(dims, grp["mlp_latency_ms"], "o-", label=f"{variant} MLP")
        axes[3].plot(dims, grp["latency_ms"], "s--", label=f"{variant} e2e")

    axes[0].set(title="Validation ADE vs Width", xlabel="Hidden Dim", ylabel="ADE [m]")
    axes[1].axhspan(60.0, 78.0, color="green", alpha=0.12, label="Target [60%, 78%]")
    axes[1].axhline(68.27, color="black", linestyle=":", linewidth=1.0, label="Gaussian 68.3%")
    axes[1].set(title="1σ Lateral Calibration", xlabel="Hidden Dim", ylabel="Coverage [%]")
    axes[2].set(title="Synthetic Generalization & Noise", xlabel="Hidden Dim", ylabel="ADE 2s [m]")
    axes[3].set(title="Pure-NumPy Single-Step Latency", xlabel="Hidden Dim", ylabel="Latency [ms]")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train-npz", type=Path, default=DEFAULT_TRAIN_NPZ)
    parser.add_argument("--val-npz", type=Path, default=DEFAULT_VAL_NPZ)
    parser.add_argument("--synthetic", action="store_true")
    parser.add_argument("--hidden-dims", type=int, nargs="+", default=[128, 256, 512, 1024])
    parser.add_argument("--epochs-warmup", type=int, default=25)
    parser.add_argument("--epochs-nll", type=int, default=35)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--device", default="auto", choices=("auto", "cpu", "cuda"))
    parser.add_argument("--models", default="both", choices=("lane", "traj", "both"))
    parser.add_argument("--bench-iters", type=int, default=2000)
    parser.add_argument(
        "--max-export-size-mb",
        type=float,
        default=2.0,
        help="Max size [MB] for git-packaged checkpoint (Contract 1 < 2 MB; 0 disables cap)",
    )
    parser.add_argument("--models-dir", type=Path, default=DEFAULT_MODELS_DIR)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--fig-dir", type=Path, default=DEFAULT_FIG_DIR)
    parser.add_argument("--export-best-to", type=Path, default=DEFAULT_EXPORT_DIR)
    args = parser.parse_args()

    device = (
        ("cuda" if torch.cuda.is_available() else "cpu") if args.device == "auto" else args.device
    )
    if not args.synthetic and args.train_npz.is_file() and args.val_npz.is_file():
        train_data = load_npz_dataset(args.train_npz)
        val_data = load_npz_dataset(args.val_npz)
        data_source = f"waymo_npz:{args.train_npz.name}"
    else:
        train_data, val_data = build_synthetic_corpus(seed=args.seed)
        data_source = "synthetic_unblocked"

    cv_synth_ade_2s = _cv_exp01_ade_2s()
    variants = []
    if args.models in ("lane", "both"):
        variants.append((True, "nn_lane", "waymo_nn_lane.npz"))
    if args.models in ("traj", "both"):
        variants.append((False, "nn_traj", "waymo_nn_traj.npz"))

    all_rows: list[dict] = []
    winners: dict[str, dict] = {}
    for use_lane, variant_name, export_filename in variants:
        variant_rows = []
        for h_dim in args.hidden_dims:
            ckpt_path = args.models_dir / f"waymo_{variant_name}_h{h_dim}.npz"
            row = train_and_verify_one(
                train_data,
                val_data,
                use_lane=use_lane,
                hidden_dim=h_dim,
                output_path=ckpt_path,
                epochs_warmup=args.epochs_warmup,
                epochs_nll=args.epochs_nll,
                batch_size=args.batch_size,
                lr=args.lr,
                seed=args.seed,
                device=device,
                data_source=data_source,
                bench_iters=args.bench_iters,
                cv_synth_ade_2s=cv_synth_ade_2s,
            )
            variant_rows.append(row)
            all_rows.append(row)
            print(json.dumps(row, indent=2), flush=True)

        best = select_best_candidate(variant_rows, max_size_mb=args.max_export_size_mb)
        best["selected"] = True
        for row in variant_rows:
            row.setdefault("selected", False)
        args.export_best_to.mkdir(parents=True, exist_ok=True)
        shutil.copy2(best["checkpoint"], args.export_best_to / export_filename)
        winners[variant_name] = best
        print(
            f"[SELECTED] {variant_name}: h={best['hidden_dim']} "
            f"(score={best['score']}, cov_1sigma={best['cov_1sigma_lat']}, "
            f"size={best['size_mb']} MB, mlp_ms={best['mlp_latency_ms']}, "
            f"e2e_ms={best['latency_ms']}) -> {args.export_best_to / export_filename}",
            flush=True,
        )

    args.log_dir.mkdir(parents=True, exist_ok=True)
    df = pd.DataFrame(all_rows)
    df.to_csv(args.log_dir / "verification_summary.csv", index=False)
    plot_capacity_verification(df, args.fig_dir / "neural_capacity_verification.png")
    winner_keys = (
        "hidden_dim",
        "score",
        "val_ade_2s",
        "val_ade_4s",
        "val_nll",
        "cov_1sigma_lat",
        "synth_ade_2s",
        "noisy_ade_2s",
        "mlp_latency_ms",
        "latency_ms",
        "size_mb",
    )
    summary = {
        "data_source": data_source,
        "device": device,
        "hidden_dims": args.hidden_dims,
        "epochs_warmup": args.epochs_warmup,
        "epochs_nll": args.epochs_nll,
        "max_export_size_mb": args.max_export_size_mb,
        "cv_synth_ade_2s": round(cv_synth_ade_2s, 4),
        "winners": {k: {wk: v[wk] for wk in winner_keys} for k, v in winners.items()},
    }
    (args.log_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()

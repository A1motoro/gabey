"""M2 — Lightweight heteroscedastic Physics-Anchored ResMLP predictor (Option B). Owner: P.

Architecture (Option-B spec §4.1, adaptive to any hidden width H_dim in {128, 256, 512, 1024}):
  - Stem:       h0 = SiLU(LayerNorm(Linear(D -> H_dim)(x)))
  - ResBlock i: h_i = h_{i-1} + Linear(H_dim -> H_dim)(SiLU(LayerNorm(Linear(H_dim -> H_dim)(h))))
  - Mean Head:  delta_mu = Linear(H_dim -> 2*H_model)(h_last)  -> mu_local = p_cv + delta_mu
  - Std Head:   z_sigma  = Linear(H_dim -> 2*H_model)(h_last)  -> clip(softplus(z) + 0.05, 0.05, 5)

Training runs in PyTorch (``experiments/train_waymo_neural.py``); runtime inference here
uses pure NumPy matrix math on compact ``.npz`` checkpoints or in-memory weight dicts.
"""

from __future__ import annotations

import re
from collections.abc import Mapping
from pathlib import Path

import numpy as np

from pbs.prediction._motion import prediction_inputs
from pbs.prediction.base import Predictor
from pbs.prediction.constant import ConstantVelocityPredictor
from pbs.prediction.features import (
    HISTORY_STEPS,
    LANE_FEATURE_DIM,
    TRAJ_FEATURE_DIM,
    TRAJ_POS_SCALE,
    TRAJ_VEL_SCALE,
    _clean_polyline,
    _is_bool,
    _validate_lane_width,
    agent_to_world,
    default_lane_features,
    extract_lane_features,
    extract_traj_matrix,
)
from pbs.types import Scenario, Trajectory

SCHEMA_VERSION = 1
MODEL_HORIZON = 40
MODEL_DT = 0.1
SIGMA_MIN = 0.05
SIGMA_MAX = 5.0
LN_EPS = 1e-5

WEIGHTS_DIR = Path(__file__).resolve().parent / "weights"
DEFAULT_TRAJ_WEIGHTS = WEIGHTS_DIR / "waymo_nn_traj.npz"
DEFAULT_LANE_WEIGHTS = WEIGHTS_DIR / "waymo_nn_lane.npz"
_BLOCK_KEY_RE = re.compile(r"^b(\d+)_(w1|b1|ln_w|ln_b|w2|b2)$")
_REQUIRED_WEIGHT_KEYS = (
    "in_w",
    "in_b",
    "in_ln_w",
    "in_ln_b",
    "mean_w",
    "mean_b",
    "std_w",
    "std_b",
)


def _silu(x: np.ndarray) -> np.ndarray:
    """Overflow-safe SiLU activation ``x * sigmoid(x)``."""
    pos = x >= 0.0
    sig = np.empty_like(x)
    sig[pos] = 1.0 / (1.0 + np.exp(-x[pos]))
    exp_x = np.exp(x[~pos])
    sig[~pos] = exp_x / (1.0 + exp_x)
    return x * sig


def _layer_norm(x: np.ndarray, weight: np.ndarray, bias: np.ndarray, eps: float) -> np.ndarray:
    mean = np.mean(x, axis=-1, keepdims=True)
    var = np.mean((x - mean) ** 2, axis=-1, keepdims=True)
    return (x - mean) / np.sqrt(var + eps) * weight + bias


def _softplus(z: np.ndarray) -> np.ndarray:
    return np.logaddexp(0.0, z)


def _validate_positive_scalar(value: float, name: str) -> float:
    if _is_bool(value):
        raise ValueError(f"{name} must be finite and positive")
    try:
        v = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be finite and positive") from exc
    if not np.isfinite(v) or v <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return v


def _snap_scalar(raw_val) -> float:
    """Snap float32-quantized scalar metadata (e.g. 0.10000000149) cleanly to decimal float."""
    arr = np.asarray(raw_val)
    val = float(arr.item() if arr.ndim == 0 else arr)
    if arr.dtype == np.float32:
        return float(f"{val:.7g}")
    return val


def gaussian_nll_2d(
    pred_mean: np.ndarray, pred_std: np.ndarray, target: np.ndarray
) -> np.ndarray:
    """Per-step 2D diagonal Gaussian negative log-likelihood in nats.

    Parameters are matching ``(..., 2)`` arrays (e.g. ``(H, 2)`` or ``(N, H, 2)``).
    Returns an array of shape ``pred_mean.shape[:-1]``.
    """
    mu = np.asarray(pred_mean, dtype=float)
    std = np.asarray(pred_std, dtype=float)
    y = np.asarray(target, dtype=float)
    if mu.shape != std.shape or mu.shape != y.shape or mu.ndim < 1 or mu.shape[-1] != 2:
        raise ValueError("pred_mean, pred_std, and target must have identical (..., 2) shapes")
    if not (np.isfinite(mu).all() and np.isfinite(std).all() and np.isfinite(y).all()):
        raise ValueError("Gaussian NLL inputs must be finite")
    if np.any(std <= 0.0):
        raise ValueError("pred_std must be strictly positive")
    sq = ((y - mu) / std) ** 2
    return 0.5 * np.sum(sq, axis=-1) + np.sum(np.log(std), axis=-1) + np.log(2.0 * np.pi)


def _discover_block_indices(weights: Mapping[str, np.ndarray]) -> list[int]:
    found: set[int] = set()
    for key in weights:
        match = _BLOCK_KEY_RE.match(key)
        if match:
            found.add(int(match.group(1)))
    if not found:
        return []
    ordered = sorted(found)
    if ordered != list(range(1, len(ordered) + 1)):
        raise ValueError(f"Residual block indices must be contiguous starting at 1, got {ordered}")
    return ordered


def _parse_and_validate_weights(raw: Mapping[str, np.ndarray]):
    missing = [k for k in _REQUIRED_WEIGHT_KEYS if k not in raw]
    if missing:
        raise ValueError(f"Missing required neural weight keys: {missing}")

    in_w = np.ascontiguousarray(np.asarray(raw["in_w"], dtype=np.float32))
    if in_w.ndim != 2 or in_w.shape[0] < 1 or in_w.shape[1] < 1:
        raise ValueError("in_w must be 2D (hidden_dim >= 1, input_dim >= 1)")
    hidden_dim, input_dim = int(in_w.shape[0]), int(in_w.shape[1])

    mean_w = np.ascontiguousarray(np.asarray(raw["mean_w"], dtype=np.float32))
    std_w = np.ascontiguousarray(np.asarray(raw["std_w"], dtype=np.float32))
    if (
        mean_w.ndim != 2
        or mean_w.shape != std_w.shape
        or mean_w.shape[1] != hidden_dim
        or mean_w.shape[0] < 2
        or mean_w.shape[0] % 2 != 0
    ):
        raise ValueError("mean_w and std_w must have matching shape (2*horizon, hidden_dim)")

    horizon = int(mean_w.shape[0] // 2)
    in_b = np.asarray(raw["in_b"], dtype=np.float32).copy()
    in_ln_w = np.asarray(raw["in_ln_w"], dtype=np.float32).copy()
    in_ln_b = np.asarray(raw["in_ln_b"], dtype=np.float32).copy()
    mean_b = np.asarray(raw["mean_b"], dtype=np.float32).copy()
    std_b = np.asarray(raw["std_b"], dtype=np.float32).copy()

    if (
        in_b.shape != (hidden_dim,)
        or in_ln_w.shape != (hidden_dim,)
        or in_ln_b.shape != (hidden_dim,)
        or mean_b.shape != (2 * horizon,)
        or std_b.shape != (2 * horizon,)
    ):
        raise ValueError("Stem or head parameter shapes are inconsistent")

    block_indices = _discover_block_indices(raw)
    blocks = []
    all_arrays = [in_w, in_b, in_ln_w, in_ln_b, mean_w, mean_b, std_w, std_b]
    for i in block_indices:
        keys = (f"b{i}_w1", f"b{i}_b1", f"b{i}_ln_w", f"b{i}_ln_b", f"b{i}_w2", f"b{i}_b2")
        if any(k not in raw for k in keys):
            raise ValueError(f"Residual block {i} is missing required keys")
        w1 = np.ascontiguousarray(np.asarray(raw[f"b{i}_w1"], dtype=np.float32))
        b1 = np.asarray(raw[f"b{i}_b1"], dtype=np.float32).copy()
        ln_w = np.asarray(raw[f"b{i}_ln_w"], dtype=np.float32).copy()
        ln_b = np.asarray(raw[f"b{i}_ln_b"], dtype=np.float32).copy()
        w2 = np.ascontiguousarray(np.asarray(raw[f"b{i}_w2"], dtype=np.float32))
        b2 = np.asarray(raw[f"b{i}_b2"], dtype=np.float32).copy()
        if (
            w1.shape != (hidden_dim, hidden_dim)
            or b1.shape != (hidden_dim,)
            or ln_w.shape != (hidden_dim,)
            or ln_b.shape != (hidden_dim,)
            or w2.shape != (hidden_dim, hidden_dim)
            or b2.shape != (hidden_dim,)
        ):
            raise ValueError(f"Residual block {i} shapes are inconsistent")
        blocks.append((w1, b1, ln_w, ln_b, w2, b2))
        all_arrays.extend((w1, b1, ln_w, ln_b, w2, b2))

    if not all(np.isfinite(arr).all() for arr in all_arrays):
        raise ValueError("Checkpoint contains non-finite weights")

    return (
        hidden_dim,
        input_dim,
        horizon,
        (in_w, in_b, in_ln_w, in_ln_b, mean_w, mean_b, std_w, std_b),
        blocks,
    )


def forward_resmlp_numpy(
    feat: np.ndarray,
    weights: Mapping[str, np.ndarray],
    cv_anchor: np.ndarray,
    *,
    sigma_min: float = SIGMA_MIN,
    sigma_max: float = SIGMA_MAX,
    ln_eps: float = LN_EPS,
) -> tuple[np.ndarray, np.ndarray]:
    """Adaptive-width pure-NumPy forward pass for ``PhysicsAnchoredResMLP``.

    Automatically infers ``H_dim = weights["in_w"].shape[0]`` and the number of
    residual blocks (``b1_*``, ``b2_*``, ...), so switching between
    ``128 / 256 / 512 / 1024`` requires zero code changes.
    """
    s_min = _validate_positive_scalar(sigma_min, "sigma_min")
    s_max = _validate_positive_scalar(sigma_max, "sigma_max")
    if s_max < s_min:
        raise ValueError("sigma_max must be >= sigma_min")
    eps = _validate_positive_scalar(ln_eps, "ln_eps")

    x = np.asarray(feat, dtype=np.float32)
    if x.ndim not in (1, 2) or not np.isfinite(x).all():
        raise ValueError("feat must be a finite 1D (D,) or 2D (N, D) array")
    anchor = np.asarray(cv_anchor, dtype=np.float64)
    if not np.isfinite(anchor).all():
        raise ValueError("cv_anchor must be finite")

    _, input_dim, horizon, stem_heads, blocks = _parse_and_validate_weights(weights)
    in_w, in_b, in_ln_w, in_ln_b, mean_w, mean_b, std_w, std_b = stem_heads
    if x.shape[-1] != input_dim:
        raise ValueError(f"feat dimension {x.shape[-1]} != weights input dim {input_dim}")

    if x.ndim == 1:
        h = _silu(_layer_norm(in_w @ x + in_b, in_ln_w, in_ln_b, eps))
        for w1, b1, ln_w, ln_b, w2, b2 in blocks:
            z = _silu(_layer_norm(w1 @ h + b1, ln_w, ln_b, eps))
            h = h + (w2 @ z + b2)
        delta_mu = (mean_w @ h + mean_b).reshape(horizon, 2).astype(np.float64)
        z_sigma = (std_w @ h + std_b).reshape(horizon, 2).astype(np.float64)
    else:
        h = _silu(_layer_norm(x @ in_w.T + in_b, in_ln_w, in_ln_b, eps))
        for w1, b1, ln_w, ln_b, w2, b2 in blocks:
            z = _silu(_layer_norm(h @ w1.T + b1, ln_w, ln_b, eps))
            h = h + (z @ w2.T + b2)
        delta_mu = (h @ mean_w.T + mean_b).reshape(len(x), horizon, 2).astype(np.float64)
        z_sigma = (h @ std_w.T + std_b).reshape(len(x), horizon, 2).astype(np.float64)

    if anchor.shape != delta_mu.shape:
        raise ValueError(
            f"cv_anchor shape {anchor.shape} does not match model output {delta_mu.shape}"
        )

    mean_local = anchor + delta_mu
    std_local = np.clip(_softplus(z_sigma) + s_min, s_min, s_max)
    return mean_local, std_local


class NeuralPredictor(Predictor):
    """Pure-NumPy inference wrapper for the L3 Option-B Physics-Anchored ResMLP.

    Supports loading either from a ``.npz`` checkpoint path or directly from an
    in-memory weight dictionary (automatically adapting to any hidden width
    ``H_dim`` in ``{128, 256, 512, 1024}``).

    After each :meth:`predict` call:
    - ``self.latest`` holds the ``(H, 2)`` world-frame mean trajectory.
    - ``self.latest_std`` holds the ``(H, 2)`` local ``(sigma_lon, sigma_lat)`` stds.
    """

    def __init__(
        self,
        weights_path: str | Path | Mapping[str, np.ndarray] | None = None,
        *,
        weights: Mapping[str, np.ndarray] | None = None,
        use_lane: bool = True,
        name: str | None = None,
    ):
        if isinstance(weights_path, Mapping):
            if weights is not None:
                raise ValueError("Pass either weights_path or weights, not both")
            weights = weights_path
            weights_path = None

        self.use_lane = bool(use_lane)
        self.name = ("nn" if self.use_lane else "nn_traj") if name is None else str(name)
        self._ego_path: np.ndarray | None = None
        self._lane_width: float = 3.5
        self.latest: np.ndarray | None = None
        self.latest_std: np.ndarray | None = None
        self._cv_fallback = ConstantVelocityPredictor()

        if weights is not None:
            self.weights_path: Path | None = None
            self._init_from_mapping(weights)
        else:
            path = (
                Path(weights_path)
                if weights_path is not None
                else (DEFAULT_LANE_WEIGHTS if self.use_lane else DEFAULT_TRAJ_WEIGHTS)
            )
            self.weights_path = path
            if not path.is_file():
                raise FileNotFoundError(f"NeuralPredictor weights not found: {path}")
            with np.load(path, allow_pickle=False) as z:
                self._init_from_mapping({k: z[k] for k in z.files})

    def _init_from_mapping(self, raw: Mapping[str, np.ndarray]) -> None:
        if "schema_version" in raw and int(raw["schema_version"]) != SCHEMA_VERSION:
            raise ValueError(f"Unsupported neural weights schema_version={raw['schema_version']}")
        if "use_lane" in raw and bool(int(raw["use_lane"])) != self.use_lane:
            raise ValueError(
                f"Checkpoint use_lane={bool(int(raw['use_lane']))} "
                f"does not match use_lane={self.use_lane}"
            )

        hidden_dim, input_dim, horizon, stem_heads, blocks = _parse_and_validate_weights(raw)
        self.hidden_dim = hidden_dim
        self.input_dim = input_dim

        self.history_steps = int(raw["history_steps"]) if "history_steps" in raw else HISTORY_STEPS
        if self.history_steps < 1:
            raise ValueError("history_steps must be >= 1")
        expected_in = self.history_steps * 6 + (LANE_FEATURE_DIM if self.use_lane else 0)
        if self.input_dim != expected_in:
            raise ValueError(f"Checkpoint input_dim={self.input_dim} != expected {expected_in}")

        self.model_horizon = int(raw["horizon"]) if "horizon" in raw else horizon
        if self.model_horizon != horizon:
            raise ValueError("Checkpoint horizon disagrees with mean_w output rows")

        self.model_dt = _validate_positive_scalar(
            _snap_scalar(raw["dt"]) if "dt" in raw else MODEL_DT, "dt"
        )
        self.sigma_min = _validate_positive_scalar(
            _snap_scalar(raw["sigma_min"]) if "sigma_min" in raw else SIGMA_MIN, "sigma_min"
        )
        self.sigma_max = _validate_positive_scalar(
            _snap_scalar(raw["sigma_max"]) if "sigma_max" in raw else SIGMA_MAX, "sigma_max"
        )
        if self.sigma_max < self.sigma_min:
            raise ValueError("Invalid checkpoint sigma bounds")
        self.ln_eps = _validate_positive_scalar(
            _snap_scalar(raw["ln_eps"]) if "ln_eps" in raw else LN_EPS, "ln_eps"
        )

        in_w, in_b, in_ln_w, in_ln_b, mean_w, mean_b, std_w, std_b = stem_heads
        self._in_w = in_w
        self._in_wt = np.ascontiguousarray(in_w.T)
        self._in_b = in_b
        self._in_ln_w = in_ln_w
        self._in_ln_b = in_ln_b
        self._mean_w = mean_w
        self._mean_wt = np.ascontiguousarray(mean_w.T)
        self._mean_b = mean_b
        self._std_w = std_w
        self._std_wt = np.ascontiguousarray(std_w.T)
        self._std_b = std_b

        self.n_res_blocks = len(blocks)
        self._blocks = [
            (w1, np.ascontiguousarray(w1.T), b1, ln_w, ln_b, w2, np.ascontiguousarray(w2.T), b2)
            for w1, b1, ln_w, ln_b, w2, b2 in blocks
        ]
        self._model_tau = np.arange(1, self.model_horizon + 1, dtype=np.float64) * self.model_dt

    def bind(self, scenario: Scenario) -> None:
        """Bind scenario road geometry (read-only copy; never reads ``scenario.agents``)."""
        self.latest = None
        self.latest_std = None
        if not self.use_lane:
            self._ego_path = None
            return
        _clean_polyline(scenario.ego_path)
        lw = _validate_lane_width(scenario.lane_width)
        self._ego_path = np.asarray(scenario.ego_path, dtype=float).copy()
        self._lane_width = lw

    def _forward_single(
        self, feat_1d: np.ndarray, cv_anchor: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        x = feat_1d.astype(np.float32, copy=False)
        h = _silu(
            _layer_norm(self._in_w @ x + self._in_b, self._in_ln_w, self._in_ln_b, self.ln_eps)
        )
        for w1, _, b1, ln_w, ln_b, w2, _, b2 in self._blocks:
            z = _silu(_layer_norm(w1 @ h + b1, ln_w, ln_b, self.ln_eps))
            h = h + (w2 @ z + b2)
        delta_mu = (
            (self._mean_w @ h + self._mean_b).reshape(self.model_horizon, 2).astype(np.float64)
        )
        z_sigma = (self._std_w @ h + self._std_b).reshape(self.model_horizon, 2).astype(np.float64)
        mean_local = cv_anchor + delta_mu
        std_local = np.clip(_softplus(z_sigma) + self.sigma_min, self.sigma_min, self.sigma_max)
        return mean_local, std_local

    def _forward_pretransposed(
        self, feats: np.ndarray, cv_anchor: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        x = feats.astype(np.float32, copy=False)
        h = _silu(
            _layer_norm(x @ self._in_wt + self._in_b, self._in_ln_w, self._in_ln_b, self.ln_eps)
        )
        for _, w1t, b1, ln_w, ln_b, _, w2t, b2 in self._blocks:
            z = _silu(_layer_norm(h @ w1t + b1, ln_w, ln_b, self.ln_eps))
            h = h + (z @ w2t + b2)
        delta_mu = (
            (h @ self._mean_wt + self._mean_b)
            .reshape(len(x), self.model_horizon, 2)
            .astype(np.float64)
        )
        z_sigma = (
            (h @ self._std_wt + self._std_b)
            .reshape(len(x), self.model_horizon, 2)
            .astype(np.float64)
        )
        mean_local = cv_anchor + delta_mu
        std_local = np.clip(_softplus(z_sigma) + self.sigma_min, self.sigma_min, self.sigma_max)
        return mean_local, std_local

    def forward_batch(
        self,
        hist_traj: np.ndarray,
        lane_ctx: np.ndarray | None = None,
        *,
        traj_normalized: bool = True,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Vectorized local-frame prediction on Contract-1 arrays."""
        traj = np.asarray(hist_traj, dtype=np.float32)
        if traj.ndim != 3 or traj.shape[1:] != (self.history_steps, 6):
            raise ValueError(
                f"hist_traj must have shape (N, {self.history_steps}, 6), got {traj.shape}"
            )
        if not np.isfinite(traj).all():
            raise ValueError("hist_traj must be finite")

        if traj_normalized:
            traj_in = traj
            v_local = traj[:, -1, 2:4].astype(np.float64) * TRAJ_VEL_SCALE
        else:
            traj_in = traj.copy()
            traj_in[:, :, 0:2] /= TRAJ_POS_SCALE
            traj_in[:, :, 2:4] /= TRAJ_VEL_SCALE
            v_local = traj[:, -1, 2:4].astype(np.float64)

        flat_traj = traj_in.reshape(len(traj), self.history_steps * 6)
        if self.use_lane:
            if lane_ctx is None:
                raise ValueError("lane_ctx is required when use_lane=True")
            lane = np.asarray(lane_ctx, dtype=np.float32)
            if lane.ndim != 2 or lane.shape != (len(traj), LANE_FEATURE_DIM):
                raise ValueError(
                    f"lane_ctx must have shape ({len(traj)}, {LANE_FEATURE_DIM}), got {lane.shape}"
                )
            if not np.isfinite(lane).all():
                raise ValueError("lane_ctx must be finite")
            feats = np.concatenate((flat_traj, lane), axis=1)
        else:
            feats = flat_traj

        cv_anchor = self._model_tau[None, :, None] * v_local[:, None, :]
        return self._forward_pretransposed(feats, cv_anchor)

    def _resample_horizon(
        self, mean_local: np.ndarray, std_local: np.ndarray, horizon: int, dt: float
    ) -> tuple[np.ndarray, np.ndarray]:
        if horizon <= self.model_horizon and np.isclose(dt, self.model_dt, rtol=1e-6, atol=1e-7):
            return mean_local[:horizon].copy(), std_local[:horizon].copy()

        tau = np.arange(1, horizon + 1, dtype=np.float64) * float(dt)
        model_t = np.arange(0, self.model_horizon + 1, dtype=np.float64) * self.model_dt
        pts_full = np.vstack((np.zeros((1, 2), dtype=np.float64), mean_local))
        std_full = np.vstack((np.full((1, 2), self.sigma_min, dtype=np.float64), std_local))

        out_mean = np.empty((horizon, 2), dtype=np.float64)
        out_std = np.empty((horizon, 2), dtype=np.float64)
        t_max = float(model_t[-1])
        within = tau <= t_max + 1e-12
        if np.any(within):
            for axis in range(2):
                out_mean[within, axis] = np.interp(tau[within], model_t, pts_full[:, axis])
                out_std[within, axis] = np.interp(tau[within], model_t, std_full[:, axis])

        beyond = ~within
        if np.any(beyond):
            dt_extra = tau[beyond] - t_max
            if self.model_horizon >= 2:
                v_end = (mean_local[-1] - mean_local[-2]) / self.model_dt
                s_rate = np.maximum(0.0, (std_local[-1] - std_local[-2]) / self.model_dt)
            else:
                v_end = mean_local[-1] / self.model_dt
                s_rate = np.zeros(2, dtype=np.float64)
            out_mean[beyond] = mean_local[-1] + dt_extra[:, None] * v_end[None, :]
            out_std[beyond] = np.clip(
                std_local[-1] + dt_extra[:, None] * s_rate[None, :],
                self.sigma_min,
                self.sigma_max,
            )
        return out_mean, out_std

    def predict(self, history: Trajectory, horizon: int, dt: float) -> np.ndarray:
        """Predict world-frame ``(horizon, 2)`` positions and update ``latest`` / ``latest_std``."""
        t, xy, _ = prediction_inputs(history, horizon, dt)
        if horizon == 0:
            empty = np.empty((0, 2), dtype=float)
            self.latest = empty.copy()
            self.latest_std = empty.copy()
            return empty

        # Option-B spec §3.1 degradation rule: single stationary frame falls back to CV.
        if len(t) == 1 and abs(float(history.v[-1])) <= 1e-12:
            pred_world = self._cv_fallback.predict(history, horizon, dt)
            self.latest = pred_world.copy()
            self.latest_std = np.full((horizon, 2), self.sigma_min, dtype=float)
            return pred_world

        traj_mat = extract_traj_matrix(
            history, dt=self.model_dt, steps=self.history_steps, normalize=True
        )
        v_local = traj_mat[-1, 2:4].astype(np.float64) * TRAJ_VEL_SCALE
        cv_anchor = self._model_tau[:, None] * v_local[None, :]
        if self.use_lane:
            if self._ego_path is not None:
                lane_vec = extract_lane_features(
                    history, self._ego_path, lane_width=self._lane_width
                )
            else:
                lane_vec = default_lane_features(history, lane_width=self._lane_width)
            feat_1d = np.concatenate((traj_mat.reshape(-1), lane_vec))
        else:
            feat_1d = traj_mat.reshape(-1)

        mean_local_full, std_local_full = self._forward_single(feat_1d, cv_anchor)
        mean_local, std_local = self._resample_horizon(
            mean_local_full, std_local_full, horizon, dt
        )
        origin = xy[-1]
        theta0 = float(history.heading[-1])
        pred_world = agent_to_world(mean_local, origin, theta0)

        self.latest = pred_world.copy()
        self.latest_std = std_local.copy()
        return pred_world


def verify_feature_dims() -> tuple[int, int]:
    """Return expected feature dimensions for trajectory-only and lane-aware modes."""
    return TRAJ_FEATURE_DIM, TRAJ_FEATURE_DIM + LANE_FEATURE_DIM


__all__ = [
    "DEFAULT_LANE_WEIGHTS",
    "DEFAULT_TRAJ_WEIGHTS",
    "HISTORY_STEPS",
    "LN_EPS",
    "MODEL_DT",
    "MODEL_HORIZON",
    "NeuralPredictor",
    "SCHEMA_VERSION",
    "SIGMA_MAX",
    "SIGMA_MIN",
    "forward_resmlp_numpy",
    "gaussian_nll_2d",
]

"""Core data structures. FROZEN at W4: add fields only, never rename or remove.

Units: m / s / m·s⁻¹ / rad. World frame everywhere (never grid cells). Sim step dt = 0.1 s.
Position arrays are (N, 2) with columns (x, y).
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Trajectory:
    """一个 agent 的一段轨迹；单位：s, m, m/s, rad"""

    t: np.ndarray  # (N,)
    x: np.ndarray  # (N,)
    y: np.ndarray  # (N,)
    v: np.ndarray  # (N,)
    heading: np.ndarray  # (N,)
    agent_type: str = "vehicle"
    agent_id: str = ""  # 供 OraclePredictor 查询真实未来


@dataclass
class Scenario:
    """一次仿真的完整输入"""

    ego_start: tuple  # (x, y, heading, v)
    ego_path: np.ndarray  # (M, 2) 参考路径，同时是本车道中心线
    agents: list[Trajectory]
    dt: float = 0.1
    params: dict = field(default_factory=dict)  # 供 boundary search 扫描
    kind: str = "cut_in"
    lane_width: float = 3.5  # m；Reactive baseline 用它判定 agent 是否在本车道内


@dataclass
class SimLog:
    """一次仿真的完整输出"""

    t: np.ndarray
    ego_x: np.ndarray
    ego_y: np.ndarray
    ego_v: np.ndarray
    ego_a: np.ndarray
    action: list[str]  # 每步 'cruise' / 'brake'
    ttc: np.ndarray  # 每步估计的 TTC（所有 agent 取 min），无冲突为 inf
    min_dist: float
    collision: bool


# ---- P ↔ C contract: risk assessment -------------------------------------------------


@dataclass
class RiskContext:
    """C 在每个仿真步交给 RiskAssessor 的全部信息（只读）。"""

    t_now: float
    ego_xy: np.ndarray  # (2,)
    ego_heading: float
    ego_v: float
    ego_future: np.ndarray  # (H, 2) C 的 ego rollout，第 k 行对应 t_now + (k+1)·dt
    ego_path: np.ndarray  # (M, 2) 本车道中心线
    lane_width: float
    horizon: int
    dt: float
    r_safe: float  # 由 C 决定（footprint 近似），P 只读


@dataclass
class RiskEstimate:
    """RiskAssessor 对单个 agent 的输出。"""

    ttc: float = np.inf  # 首次冲突时间 [s]；无冲突为 inf
    conflict_step: int | None = None  # 首次冲突在 horizon 中的下标
    considered: bool = True  # Reactive：agent 不在本车道时为 False（被忽略）

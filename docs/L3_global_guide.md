# `gabey` L3 全局开发总纲与四人并行开工手册

> **版本**：L3 Frozen Spec v1.0（2026-11-23）  
> **基准分支**：`main`（`a3f91cb`，PR #13 合入后的统一 L2 基线）  
> **适用对象**：全组 4 人 —— **A (Persistent · M1)**、**P (华华 · M2)**、**C (前背痛 · M3)**、**D (yj · M4)**  
> **交付死线**：**12/8（周二）** Final 展示 + Live Demo（10 + 10 min） · **12/13（周日）** `gabey-v1.0.zip` + 8–12 页最终报告  
> **最高原则**：**只加不改，严格向后兼容 L2**。所有 L3 新能力均通过**新增类或可选关键字参数**开启，必须保证现有 348 项 `pytest` 与 9 项前端 JS 测试 100% 不破。

---

## 1. 全局架构：从 L2 到 L3 我们升级什么？

| 维度 | L2 已冻结基线（默认不变） | L3 升级规格（本次四人共同目标） | 负责模块 |
| --- | --- | --- | --- |
| **车辆碰撞几何** | `5.0 m × 2.5 m` 定向矩形 OBB（`r_safe=None`） | **`5.0 m × 2.0 m` 乘用车定向矩形 OBB**（通过可选参数 `vehicle_width=2.0` 开启，L2 默认仍为 `2.5 m`） | C (`pbs.sim`) + P (`pbs.risk`) |
| **纵向制动策略** | 二值滞回 `HysteresisPolicy(t_on=2.0, t_off=3.0, a=6.0)`，松刹不恢复车速 | **三级滞回 + 双档 Jerk 限幅 `StagedBrakingPolicy`**：恢复加速 `+1.5` / 舒适预制动 `-2.5` / 紧急制动 `-6.0 m/s²`，双档加加速度限幅 `jerk_soft = 5.0 m/s³`、`jerk_hard = 20.0 m/s³` | C (`pbs.sim.policy`) |
| **预测器谱系** | `Reactive → CV / CA / CTRV / LR → Oracle` | 新增 **Waymo 轻量神经网络（方案 B：单模态均值 + 异方差位置标准差 $\sigma$）**，支持 `NN-Traj`（纯轨迹）与 `NN-Lane`（含相对参考车道）消融；GPU PyTorch 训练 → 纯 NumPy `.npz` 推理 | P (`pbs.prediction`) |
| **风险评估 (TTC)** | 4.0 s 时域（`horizon=40`）+ 步间连续扫掠（Swept TTC） | 新增 **不确定度感知的 Swept TTC（Uncertainty-Aware TTC）**：当 `uncertainty_scale = κ > 0` 时，按 $+\kappa\sigma_k$ 动态膨胀目标车碰撞包络 | P (`pbs.risk`) |
| **观测噪声模型** | 理想零噪声真值观测 | **因果缓存高斯观测噪声 `GaussianObservationModel`**：对 `(x, y, v, heading)` 注入噪声并按 `(agent_id, timestamp)` 缓存，不污染真值碰撞 | C (`pbs.sim.observation`) |
| **运行期安全兜底与超时保护** | `Reactive` 与 `Predictive` 完全独立；无在线超时/缺测降级 | **帧级 Simplex 安全协调器 `FrameFailsafeCoordinator`**：明确 `Reactive` 为全预测系统的最终 AEB 兜底；① 整帧 `50 ms` 共享预算并行调度 + `frame_id` 丢弃迟到帧；② 基于采样时间戳按 `agent_id` 追踪缺测/过期（区分 `TTC=+∞` 与 `unknown`，默认 `0.20 s` 宽限 + `2` 帧恢复防抖）；③ Hold 锁目标档位并统一走 `StagedBrakingPolicy` 同一 `prev_a`/双档 Jerk；④ 独立 `latency_margin` 偏移有限 TTC（详见 [reactive_failsafe_and_watchdog.md](reactive_failsafe_and_watchdog.md) 与 [T6_failsafe_design_review.md](T6_failsafe_design_review.md)） | C (`pbs.sim.failsafe`) |
| **场景与评测轨道** | 合成场景（96 默认 + 12 CA + 6 中止换道，19 s 含 4 s 缓冲） | **双轨评测**：① 合成网格（Failure Boundary、跨域泛化、抗噪退化、分级制动舒适性）；② **Waymo 真实弯道/切入场景集**（放开直线限制，曲线 `ego_path`） | A (`pbs.data`) + D (`pbs.eval`) |
| **评估指标与反事实口径** | `collision`, `false_brake`, `min_dist_until_contact`, `brake_onset_t`, `rescue_rate`, `lead_time`；15 s 窗口二分类 `benign` | ① 新增 **纵向舒适性与效率指标**：`max_decel`、`peak_jerk`、`rms_jerk`、`speed_loss`、`end_speed`、`hard_brake`；② 新增 **物理末端安全判据与反事实三分类**（`dangerous` / `terminal_unsafe` / `benign`，结合末端 ICS 与 `4.0 s` 2D Swept TTC 外推，消除 15 s 截断伪良性误刹惩罚，详见 [terminal_safety_counterfactual.md](terminal_safety_counterfactual.md)） | D (`pbs.eval`) |
| **2.5D Live Demo** | 固定 `Reactive vs CV` + 合成直线切入 | **任意双系统下拉切换**（含 `Neural` / `Neural-UA` / `CA` / `CTRV`）+ **一键切换合成切入与预导出 Waymo 真实片段 JSON** + **分级制动状态 & $1\sigma$ 预测带渲染** | C (`pbs.demo`) + A (`waymo/*.json`) |

---

## 2. 四人并行开工契约（Day 1 即可各自写代码，互不阻塞）

为了让 A、P、C、D 四个人**今天就能同时拉分支写代码**，以下 7 个跨模块接口契约在开工前直接锁死：

```
A (M1 pbs.data)                     P (M2 pbs.prediction / pbs.risk)
──────────────────                  ────────────────────────────────
[契约 1] Waymo 训练切片 .npz ──────► 1. features.py + neural.py (方案 B)
[契约 2] Waymo 精选场景 *.json       2. 导出 weights/*.npz (< 2 MB)
         + 收编 abort_cut_ins        3. PredictiveAssessor(uncertainty_scale)
         │                                        │ [契约 3: latest_std & 尺寸]
         ▼                                        ▼
C (M3 pbs.sim / pbs.demo)           D (M4 pbs.eval / pbs.systems / experiments)
─────────────────────────           ───────────────────────────────────────────
1. 车身尺寸参数化 (vehicle_width) ──► 1. 注册 nn_traj / nn / nn_ua
2. StagedBrakingPolicy (+Jerk 限幅)  2. [契约 4] 识别 _BRAKE_ACTIONS 算舒适性指标
3. GaussianObservationModel (缓存)   3. [契约 5] run_matrix 支持 staged + 配对噪声
4. 2.5D Demo 多系统 + Waymo JSON ◄── 4. 跑全量 L2 + Waymo 路考 + 抗噪/舒适性实验
```

### 契约 1（A ↔ P）：训练数据 `.npz` 与统一特征格式
- **以目标车末帧 $t_0$ 为原点、末帧航向 $\theta_0$ 为 $+x$ 轴**的 Agent-Centric 局部坐标系：
  - `hist_traj`: `(N, 10, 6)`，`float32` —— 过去 1.0 s（10 帧，$[t_0-0.9, t_0]$）每帧局部 `(dx, dy, vx, vy, cos Δθ, sin Δθ)`；
  - `lane_ctx`: `(N, 20)`，`float32` —— 目标车当前相对 `scenario.ego_path` 的归一化横向偏移 $d_{\text{lat}}/\text{lane\_width}$、纵向/横向速度 $(v_{\text{lon}}/20, v_{\text{lat}}/5)$、相对航向 $(\cos\Delta\theta_{\text{lane}}, \sin\Delta\theta_{\text{lane}})$、`lane_width/3.5`，以及前方 $\{5, 10, 15, 20, 30, 40, 50\}\text{ m}$ 处 `ego_path` 上的 7 个点在目标车局部系下的 `(dx/40, dy/40)`（共 $6 + 14 = 20$ 维）；
  - `future_xy`: `(N, 40, 2)`，`float32` —— 未来 4.0 s（40 帧）在目标车 $t_0$ 局部系下的真实坐标 $(\Delta x_k, \Delta y_k)$。
- **不阻塞策略**：P 先写 `src/pbs/prediction/features.py` 实现 `extract_traj_features(history)` 和 `extract_lane_features(history, ego_path, lane_width)`，先用合成场景切片跑通模型训练与单测；A 导出 Waymo `.npz` 后 P 直接重训替换权重文件即可。

### 契约 2（A ↔ C / D）：精选 Waymo 场景离线 JSON Schema
- 存放在 `src/pbs/demo/scenes/waymo/*.json`（3–5 个代表性场景，单文件 `< 100 KB`，**直接入库，运行时零 TensorFlow / Waymo SDK 依赖**）：
  ```json
  {
    "scene_id": "waymo_cut_in_01",
    "label": "Waymo · Cut-in #01",
    "dt": 0.1,
    "duration": 6.0,
    "lane_width": 3.7,
    "ego_start": [0.0, 0.0, 0.0, 14.2],
    "ego_path": [[0.0, 0.0], [1.42, 0.01], "... (末端延伸 >= v_ego*4.0 + 30 m)"],
    "ego_reference": {"agent_id": "ego", "t": [...], "x": [...], "y": [...], "v": [...], "heading": [...]},
    "agents": [{"agent_id": "1042", "agent_type": "vehicle", "t": [-1.0, "...", 10.0], "x": [...], "y": [...], "v": [...], "heading": [...]}],
    "road_lines": [[[-20.0, -1.85], [150.0, -1.85]], ...],
    "source": {"kind": "waymo_open_motion", "record_id": "...", "transforms": {"time_origin_s": 1.0, "units": "m, s, rad", "frame": "ego_t0 +x"}}
  }
  ```

### 契约 3（P ↔ C / D）：`NeuralPredictor` 与 `RiskContext` 接口
- `RiskContext` 末尾新增两个带默认值的可选字段：`vehicle_length: float = 5.0`、`vehicle_width: float = 2.5`；
- `NeuralPredictor.predict(history, horizon, dt)` 返回 `(H, 2)` 世界坐标均值轨迹，并同时设置实例属性：
  - `self.latest`: `(H, 2)` 世界坐标均值轨迹；
  - `self.latest_std`: `(H, 2)` 沿预测航向的纵向与横向位置标准差 $(\sigma_{\text{lon}, k}, \sigma_{\text{lat}, k})$。

### 契约 4（C ↔ D）：分级制动与容错动作标签
- `StagedBrakingPolicy` 输出动作标签集合为 `{"cruise", "recover", "coast_zero", "soft_brake", "hard_brake"}`（其中 `"coast_zero"` 为 C-T6 缺测首帧平滑撤力标签；L1/L2 为 `{"cruise", "brake"}`）；
- 凡统计“是否处于制动指令”（`evaluate`、`brake_activity`、`is_braking_action`），统一判定 `action in {"brake", "soft_brake", "hard_brake"}`（`"coast_zero"` 不计为制动）。

### 契约 5（D ↔ P / C）：系统注册名
- `src/pbs/systems.py` 统一使用以下系统 key：
  - 既有：`reactive`, `cv`, `ca`, `ctrv`, `lr`, `oracle`
  - L3 新增：`nn_traj`（纯轨迹神经网络）、`nn`（含车道特征神经网络，$\kappa=0$）、`nn_ua`（含车道特征 + 不确定度包络膨胀，$\kappa=1.0$）。

### 契约 6（D ↔ C / A）：反事实三分类口径（PR #18 双窗口 `late_contact` + 物理末端安全 ICS/TTC）
- **合成场景双窗口三分类（已由 PR #18 合入 `main`）**：`never_brake_label(scenario, decision_duration=15.0, duration=19.0)` 将场景分为 `label ∈ {"dangerous", "late_contact", "benign"}`：
  - `"dangerous"`：`[0, 15 s]` 决策窗内无制动碰撞（进入 `collision_rate` / `rescue_rate` 分母）；
  - `"late_contact"`：`[0, 15 s]` 内未撞，但延长 `+4.0 s` 预测时域至 `19.0 s` 结果窗内发生无制动碰撞（**既不计入 `false_brake_rate` 分母，也不稀释 `collision_rate` 分母**，单独汇报 `n_late_contact` 与 `late_contact_outcomes()`）；
  - `"benign"`：`[0, 19 s]` 内未撞（仅统计前 `15 s` 决策窗内下达的制动为 `false_brake`）。
- **物理末端安全判据（ICS + 4.0 s 2D Swept TTC 外推，详见 [docs/terminal_safety_counterfactual.md](terminal_safety_counterfactual.md)）**：当场景日志无法向后延长 `4.0 s` 真值仿真（如固定长度真实片段或单窗口 `duration=15.0 s` 调用）时，通过检查窗口末端 $t = T$ 的不可避免碰撞态（ICS：$d_{\text{gap}} \le v_{\text{rel}}^2 / (2 a_{\text{brake}})$）与 `4.0 s` 2D Swept TTC 外推，将末端紧迫冲突统一归入第三类（`terminal_unsafe` / `late_contact`），与 PR #18 的三分类分母规则完全一致。

### 契约 7（C ↔ P / D）：帧级 Simplex 安全兜底（`FrameFailsafeCoordinator`）
- 完整架构、评审修正与验收规范见：**[docs/reactive_failsafe_and_watchdog.md](reactive_failsafe_and_watchdog.md)** 与 **[docs/T6_failsafe_design_review.md](T6_failsafe_design_review.md)**；
- 架构上明确 **`ReactiveAssessor` 是所有预测型 AEB 系统的最终兜底机制**；科研对比实验（`exp02/03/05`）仍使用独立无兜底基线，运行期容错由 C 在 `src/pbs/sim/failsafe.py` 与 `src/pbs/sim/_failsafe_channels.py` 提供帧级协调器 `FrameFailsafeCoordinator`（并在 `src/pbs/types.py` 新增 `FrameDecision` 与 `SimLog.failsafe_decisions`，`ObservationModel.observe` 支持返回 `Trajectory | None`）：
  1. **整帧 50 ms 共享预算与并行调度**（Reactive 与预测通道并行，按 `frame_id` 丢弃迟到结果、忙碌跳过防积压，提供 `VirtualChannels` 确定性可控时钟与 `AsyncChannels` `spawn` 独立持久进程双后端，`run_sim` 托管 `bind / step_frame / close` 生命周期）；
  2. **按 `agent_id` 追踪缺测/过期**（用采样时间戳算年龄抗旧帧重发，严格区分有效 `TTC=+∞` 与 `unknown`；缺测默认 `missing_grace_s=0.20 s` 宽限、`recovery_confirm_frames=2` 帧恢复防抖；高风险前车失联保留制动不被正常目标解除）；
  3. **Hold 锁档位并统一走 L3 执行链路**（废除外部复制 `prev_a`，正常帧/缺测帧/恢复帧共用 `StagedBrakingPolicy` 内部同一个 `prev_a`、`brake_mode()`、`mode_override`、双档 Jerk 限幅与速度前瞻卸力）；
  4. **独立延迟裕度补偿**（`TTC_eff = max(0, TTC_raw - latency_margin)`，与 `missing_grace_s` 独立配置调参）。

---

## 3. A（M1 · `pbs.data`）开发清单与规范

### A-T1：Waymo TFRecord 通用场景加载（`src/pbs/data/waymo.py`）
1. **去掉直线限制**：删除旧 `waymo_l1_replay.py` 中 `max_lateral > 1.5` 的过滤，支持任意弯道与直道场景。
2. **坐标变换与 `ego_path` 构造**：
   - 以 `current_time_index`（$t = 0$）自车位置为 `(0, 0)`、初始航向为 `+x` 轴平移旋转全体轨迹和 `road_line` 折线；
   - 以自车真实轨迹去重零长线段、并在末端沿切线延伸 `max(50.0, v_ego * 5.0)` 米作为 `Scenario.ego_path`；确保 `ego_start[:2] == ego_path[0]` 且 `ego_start[2] == path_heading(ego_path, 0)`（通过 `simulator._validate`）。
3. **周车轨迹过滤**：保留在 `[-1.0 s, duration + 4.0 s]` 内有效（允许 $\le 2$ 帧短暂缺失做线性插值）的车辆/行人轨迹。
4. **函数签名**：
   ```python
   def load_waymo_scenario(
       tfrecord_path: str | Path,
       record_index: int = 0,
       *,
       duration: float = 6.0,
       log_pad: float = 4.0,
       min_ego_speed: float = 2.0,
   ) -> tuple[Scenario, list[np.ndarray], Trajectory]: ...
   ```

### A-T2：导出 GPU 训练与验证数据集（`scripts/extract_waymo_npz.py`）
1. **按场景 ID 划分 Train / Val（80% / 20%）**，严禁同一场景的不同时间步同时出现在训练集和验证集。
2. 调用与 P 共享的 `extract_traj_features` 和 `extract_lane_features`，滑窗提取 `1.0 s (10 帧) → 4.0 s (40 帧)` 样本（过滤全窗口位移 `< 1.0 m` 的纯静止车辆），保存为 `data/waymo_train.npz` 与 `data/waymo_val.npz`。

### A-T3：导出 3–5 个精选 Waymo 场景 JSON（`src/pbs/demo/scenes/waymo/*.json`）
- 至少包含：① 典型相邻车道切入 `waymo_cut_in_01.json`、② 弯道跟驰/切入 `waymo_curve_02.json`、③ 邻道贴近未切入良性场景 `waymo_benign_03.json`；
- 在 `src/pbs/data/waymo.py` 中提供 `load_waymo_json(path_or_id) -> tuple[Scenario, list[np.ndarray], Trajectory, dict]` 供 C 的 Demo 与 D 的 `exp07` 直接调用。

### A-T4：收编 `abort_cut_ins` 至 `src/pbs/data/scenarios.py`
- 将 `make_abort` 与 `abort_cut_ins` 迁入 `src/pbs/data/scenarios.py` 并从 `pbs.data` 导出；`experiments/scenario_sets.py` 改为从 `pbs.data` 导入。

### A 的验收测试（`tests/test_waymo_data.py`）
- [ ] 内置 `src/pbs/demo/scenes/waymo/*.json` 全部通过 `run_sim` 校验（含 `horizon=40` 与曲线 `ego_path`）；
- [ ] `abort_cut_ins` 在 `vehicle_width=2.5` 与 `vehicle_width=2.0` 下均为 `benign=True`。

---

## 4. P（M2 · `pbs.prediction` & `pbs.risk`）开发清单与规范

### P-T1：跨场景统一特征提取器（`src/pbs/prediction/features.py`）
- 实现契约 1 中的 `extract_traj_features(history, dt=0.1, steps=10) -> (60,)` 与 `extract_lane_features(history, ego_path, lane_width=3.5) -> (20,)`；
- 保证平移与旋转不变性（对 `Scenario` 整体旋转任意角度，提取出的特征向量逐位相等）。

### P-T2：方案 B 异方差不确定度神经网络与 `{128, 256, 512, 1024}` 容量验证（`src/pbs/prediction/neural.py`）
> 完整架构、两阶段训练法与四档容量选优协议见专项文档：**[docs/neural_predictor_option_b.md](neural_predictor_option_b.md)**。
1. **模型结构（支持纯 NumPy `.npz` 自适应宽度推理导出）**：
   - 输入：`60` 维（`use_lane=False`）或 `80` 维（`use_lane=True`）；
   - 主干：3 层残差 MLP（2 个 ResBlock，LayerNorm + SiLU，隐藏维 $H_{\text{dim}} \in \{128, 256, 512, 1024\}$）：
     - 以局部恒速外推 $\hat{p}_k^{\text{cv}} = v_{\text{local}}(t_0) \cdot (k \cdot dt)$ 为物理残差锚点，网络输出位置修正量 $\Delta\mu_k \in \mathbb{R}^2$ 与不确定度原值 $z_{\sigma, k} \in \mathbb{R}^2$；
     - 局部均值 $\hat{p}_k = \hat{p}_k^{\text{cv}} + \Delta\mu_k$，局部标准差 $\sigma_k = \text{clip}(\text{softplus}(z_{\sigma, k}) + 0.05, 0.05, 5.0)$。
2. **GPU 训练与 `{128, 256, 512, 1024}` 四档容量验证脚本（`experiments/train_waymo_neural.py`）**：
   - 固定随机种子，用「Stage 1 Smooth L1 均值预热 (25 Epochs) + Stage 2 Gaussian NLL 联合校准 (35 Epochs)」依次训练 $H_{\text{dim}} \in \{128, 256, 512, 1024\}$ 四档模型；
   - 按 5 个维度（Waymo 验证集 ADE/FDE、Gaussian NLL 与 $1\sigma$ 覆盖率校准偏差、A 的合成切片跨域 ADE/FDE、观测加噪鲁棒性、纯 NumPy CPU 单步推理延迟）自动生成对比表 `verification_summary.csv` 与验证图，并将综合最优档位导出至 `src/pbs/prediction/weights/waymo_nn_lane.npz` 和 `waymo_nn_traj.npz` 入库。
3. **`NeuralPredictor` 类接口**：
   - `predict(history, horizon, dt) -> (H, 2)`：纯 NumPy 矩阵前向推理（根据 `.npz` 矩阵维度自动适配任意 $H_{\text{dim}}$），将局部均值旋回世界坐标返回，同时更新 `self.latest` 与 `self.latest_std`（`(H, 2)`）。

### P-T3：不确定度感知的 Swept TTC 与车身尺寸透传（`src/pbs/risk/`）
1. **车身尺寸透传**：
   - `assess_risk(...)` 新增可选参数 `vehicle_length: float = VEHICLE_LENGTH, vehicle_width: float = VEHICLE_WIDTH, agent_length_pad: np.ndarray | float = 0.0, agent_width_pad: np.ndarray | float = 0.0`；
   - `ReactiveAssessor` 与 `PredictiveAssessor` 从 `ctx.vehicle_length` / `ctx.vehicle_width` 读取当前车长/车宽。
2. **不确定度包络膨胀（Uncertainty-Aware Swept TTC）**：
   - `PredictiveAssessor(predictor, *, uncertainty_scale: float = 0.0, name: str | None = None)`：
   - 当 `uncertainty_scale > 0.0` 且 `getattr(predictor, "latest_std", None)` 非空时，在第 $k$ 步扫掠区间将目标车的有效长宽膨胀 $2 \kappa \sigma_{\text{lon}, k}$ 与 $2 \kappa \sigma_{\text{lat}, k}$（单侧膨胀上限 `<= 1.5 m`）。

### P-T4：扩展 `exp01` 开环预测与校准度评测
- 在合成切片与 Waymo 验证集上对比 `CV / CA / CTRV / LR / NN-Traj / NN-Lane / Oracle` 的 **ADE / FDE / Gaussian NLL**，结果更新至 `docs/l3-validation.md`。

### P 的验收测试（`tests/test_neural_predictor.py` & `tests/test_risk.py`）
- [ ] `NeuralPredictor` 满足 SE(2) 刚体旋转平移等变性、`horizon=0` 返回 `(0, 2)`、不修改输入；
- [ ] `uncertainty_scale=1.0` 时 `TTC(nn_ua) <= TTC(nn)`；`ctx.vehicle_width=2.0` 正确透传。

---

## 5. C（M3 · `pbs.sim` & `pbs.demo`）开发清单与规范

### C-T1：车辆尺寸参数化（支持 L3 `5.0 m × 2.0 m`）
- `src/pbs/sim/collision.py`：保留 `VEHICLE_LENGTH = 5.0`、`VEHICLE_WIDTH = 2.5`，新增 `L3_VEHICLE_WIDTH = 2.0`；为 `_projections`、`overlaps`、`in_collision`、`swept_contact_fraction` 追加关键字参数 `vehicle_length: float = VEHICLE_LENGTH, vehicle_width: float = VEHICLE_WIDTH`（以及供 P 包络膨胀使用的可选 `agent_length: float | None = None, agent_width: float | None = None`）。
- `src/pbs/types.py`：`RiskContext` 追加 `vehicle_length: float = 5.0, vehicle_width: float = 2.5`。
- `src/pbs/sim/simulator.py`：`run_sim(...)` 追加 `vehicle_length: float = VEHICLE_LENGTH, vehicle_width: float = VEHICLE_WIDTH` 并透传至 `RiskContext` 与碰撞函数。

### C-T2：三级滞回分级制动 + 双档 Jerk 限幅（`StagedBrakingPolicy`）
- 完整数学定义、建压时间线与参考代码见专项规范：**[docs/staged_braking_and_jerk.md](staged_braking_and_jerk.md)**。
- 在 `src/pbs/sim/policy.py` 新增 `StagedBrakingPolicy`（对齐 ISO 15622 / Euro NCAP 分级逻辑）：
  - **状态与目标加速度**：
    - `hard_brake`（`TTC < t_on_hard=1.5 s`，退出 `t_off_hard=2.0 s`）：目标 `-6.0 m/s²`；
    - `soft_brake`（非制动态 `1.5 s ≤ TTC < t_on_soft=3.0 s`，或 `hard_brake` 退出后 `2.0 s < TTC ≤ t_off_soft=3.5 s`）：目标 `-2.5 m/s²`；
    - `recover` / `cruise`（非制动态 `TTC ≥ 3.0 s`，或制动态 `TTC > 3.5 s` 退出制动后）：若 $v < v_{\text{target}}$ 则以 `+1.5 m/s²` 温和恢复至 $v_{\text{target}}$，否则 `0.0 m/s²`。
  - **双档 Jerk 限幅与前瞻收力**：
    - 处于 `hard_brake`（紧急避险优先）：使用 **`jerk_hard = 20.0 m/s³`**（每 0.1 s 步变化 `2.0 m/s²`，0.3 s 快速建压至 `-6.0 m/s²`；若已处于 `-2.5 m/s²` 预制动底压则仅需 **0.2 s** 踩满）；
    - 处于 `soft_brake` / `recover` / `cruise`（舒适平顺优先）：使用 **`jerk_soft = 5.0 m/s³`**（每 0.1 s 步变化 `0.5 m/s²`，符合 ISO 15622 舒适限值）；接近刹停或目标巡航车速前按 `jerk_soft` 提前卸力（`_speed_limit_accel`），消除单帧 Jerk 尖峰；同时保留 `max_jerk=` 兼容参数。
- `simulator.py` 支持连续加速度运动学步进（含 `a > 0` 恢复加速与步内刹停/达速截断）。

### C-T3：因果缓存高斯观测噪声（`GaussianObservationModel`）
- 在 `src/pbs/sim/observation.py` 新增 `GaussianObservationModel(pos_std=0.15, heading_std=0.03, speed_std=0.25, seed=0)`；
- 按 `(agent_id, round(float(t), 6))` 缓存每个历史采样点的随机扰动，确保同一历史时刻被不同仿真步重复读取时测量值恒定，且绝不污染真值碰撞检查。

### C-T4：弯道 `ego_path` 弧长预计算优化（`src/pbs/utils.py`）
- `path_point(path, s, *, cum=None)` 与 `path_heading(path, s, *, cum=None)` 支持传入预计算的累积弧长数组 `cum`，消除 Waymo 多点弯道在 0.01 s 碰撞扫掠下的重复计算开销。

### C-T5：2.5D 浏览器 Live Demo 升级（`src/pbs/demo/`）
- 支持左右下拉切换任意两个系统（`reactive`, `cv`, `ca`, `ctrv`, `lr`, `nn`, `nn_ua`, `oracle`）；
- 支持一键切换「合成切入」与 A 导出的「Waymo 真实弯道/切入片段 JSON」（`WaymoJsonAdapter`）；
- 支持切换 `L2 滞回二值制动 / L3 三级温和制动`、`车宽 2.0 m / 2.5 m`、`观测噪声档位`，并在画布上绘制 `latest_std` 对应的 $1\sigma$ 预测不确定度带及面板上的 `Max Decel` / `Peak Jerk` 指标。

### C-T6：帧级 Simplex 安全兜底、缺测状态机与统一 L3 执行链路（`src/pbs/sim/failsafe.py`）
- 完整规格、评审依据与四项仿真验收见：**[docs/reactive_failsafe_and_watchdog.md](reactive_failsafe_and_watchdog.md)** 与 **[docs/T6_failsafe_design_review.md](T6_failsafe_design_review.md)**；
- 新增 `FrameFailsafeCoordinator`（及 `FailsafeConfig` / `FrameDecision`）：
  1. **整帧 50 ms 共享预算与非阻塞截止**：每帧固定快照与 `frame_id`，Reactive 与预测通道并行（预测可按帧批量处理），到点只取帧号匹配的已就绪结果；迟到帧丢弃、忙碌时跳过防积压；同时支持可控虚拟时钟（确定性仿真与单测）与异步 worker（真实模型压测）双后端；
  2. **双通道共同缺测与按 `agent_id` 失联追踪**：严格区分有效 `TTC=+∞` 与 `unknown`/过期（从最后可信采样时间戳算观测年龄，重发旧帧不清零）；已在制动则保留原档位，巡航/恢复首帧缺测将目标加速度归零、满 `missing_grace_s=0.20 s` 进入软刹；高风险前车失联保留制动不被其他正常目标解除；松刹需连续 `2` 帧新鲜有效观测；
  3. **统一 L3 动力学执行链路**：在 `StagedBrakingPolicy` 内部拆出统一 `_apply_mode_accel(mode, v, dt)`，Hold 仅锁定制动档位，使正常帧、缺测帧与恢复帧共用同一个 `prev_a`、双档 Jerk 限幅（`5.0`/`20.0 m/s³`）与前瞻停车卸力（刚进硬刹 `-2` 丢帧仍继续建压到 `-6`，恢复 `+1.5` 丢帧按 `+1.0 → +0.5 → 0` 平滑撤力）；
  4. **独立延迟裕度参数**：对有效有限 TTC 施加 `TTC_eff = max(0, TTC_raw - latency_margin)`，与 `missing_grace_s` 独立配置与调参；L2 默认路径遇空历史仍抛 `MissingObservationError` 保持零回归。

### C 的验收测试
- [ ] `tests/test_collision.py`、`tests/test_policy.py`、`tests/test_observation.py`、`tests/test_failsafe.py`、`tests/test_demo.py` 全绿，L2 默认 348 项测试零回归。

---

## 6. D（M4 · `pbs.eval` & `pbs.systems` & `experiments`）开发清单与规范

### D-T1：注册 L3 系统（`src/pbs/systems.py`）
- 注册 `"nn_traj"`、`"nn"`、`"nn_ua"`（若 P 的 `NeuralPredictor` 尚未合入，可先用工厂延迟导入避免循环依赖）。

### D-T2：舒适性与通行效率评估指标（`src/pbs/eval/metrics.py`）
- **制动动作统一识别（重要 · 对接 C-T2 与 C-T6）**：
  - 直接复用 `pbs.sim.policy.is_braking_action(action)`（或 `_BRAKE_ACTIONS = {"brake", "soft_brake", "hard_brake"}`）替换原有的 `action == "brake"`；
  - 注意 C 的 `StagedBrakingPolicy` 还会输出非制动动作 `"cruise"`、`"recover"`（恢复巡航加速）以及 C-T6 缺测首帧撤力标签 **`"coast_zero"`**（目标加速度归零）——这三者均**不计入制动/误刹**（`is_braking_action` 为 `False`），切勿将 `"coast_zero"` 或 `"recover"` 当作未知动作报错；
- `evaluate(log, benign, *, decision_end=None)` 新增返回字段（均截止到 `first_collision_t` 或 `executed_steps`）：
  - `max_decel` (`float`, $\max(0, \max(-a_k))$)
  - `peak_jerk` (`float`, $\max |a_k - a_{k-1}| / \Delta t$)
  - `rms_jerk` (`float`, $\sqrt{\text{mean}(( (a_k - a_{k-1})/\Delta t )^2)}$)
  - `speed_loss` (`float`, $\max(0, v_0 - \min v_k)$)
  - `end_speed` (`float`, 有效末帧车速)
  - `hard_brake` (`bool`, 是否触发二级急刹)

### D-T2b：反事实三分类与物理末端安全检查（`src/pbs/eval/counterfactual.py`）
- 合成场景双窗口三分类（`decision_duration=15.0, duration=19.0` → `label ∈ {"dangerous", "late_contact", "benign"}`）已由 D 在 PR #18 合入 `main`；
- 针对固定长度场景或单窗口末端物理安全检查（ICS：$d_{\text{gap}} \le d_{\text{stop}}$ + `4.0 s` 2D Swept TTC 前瞻外推），完整数学定义与统一三分类接口见专项规范：**[docs/terminal_safety_counterfactual.md](terminal_safety_counterfactual.md)**。

### D-T3：`run_matrix` 支持分级制动、配对噪声与反事实三分类（`src/pbs/eval/sweep.py`）
- `policy_kind` 支持 `"staged"`（构造 `StagedBrakingPolicy`）；
- 新增 `observation_factory: Callable[[int], ObservationModel] | None = None`：对同一 `(scenario_id, latency, seed)` 下的不同系统派生相同的确定性子种子，实现严格配对的抗噪对比；
- 新增 `check_terminal_safety: bool = False, terminal_safe_ttc: float = 4.0`：开启后按 `classify_scenario` 缓存并记录 `scenario_class`、`terminal_ics`、`terminal_ttc`；`tradeoff()` 与 `rescue_rate()` 严格按 `scenario_class == "dangerous"` / `"benign"` 划分分母，并在 `tradeoff()` 输出 `n_terminal_unsafe`（旧 CSV 无此列时无缝回退）；
- 输出 DataFrame 增加 `vehicle_width`、`max_decel`、`peak_jerk`、`rms_jerk`、`speed_loss`、`hard_brake`、`scenario_class`、`terminal_ics`、`terminal_ttc` 列。

### D-T4：全量 L2 基准与 L3 最终报告实验脚本
1. **全量 L2 基准图**：跑完 15×17（255 场景）× 6 阈值的 `exp02`、`exp03`、`exp05`；
2. **Waymo 真实场景闭环评测（`experiments/exp07_waymo_benchmark.py`）**：在 Waymo 真实弯道/切入场景集上对比各系统碰撞率、误刹率、提前量与舒适性；
3. **抗噪鲁棒性与分级制动消融（`experiments/exp08_l3_ablation.py`）**：产出 ① 观测噪声 $\sigma_{\text{pos}} \in \{0.0, 0.10, 0.25\}\text{ m}$ 退化曲线；② L2 二值滞回 vs L3 三级温和制动（`5.0×2.0 m`）的安全性–舒适性（Jerk / 速度损失）对比表。

### D 的验收测试（`tests/test_counterfactual.py` & `tests/test_sweep.py`）
- [ ] 验证新舒适性指标数值、`policy_kind="staged"` 矩阵运行与配对噪声种子一致性；
- [ ] 验证 `classify_scenario` 对 ICS / 末端 TTC ≤ 4.0 s 样本判为 `"terminal_unsafe"`、对远处慢逼近样本保留 `"benign"`，且 `tradeoff()` 分母纯净、不误计 `false_brake`。

---

## 7. 协作流程、合入顺序与门禁要求

### 推荐分支与 PR 合入顺序（互不锁死）
1. **第一波（底座先行，Day 1–3）**：
   - C 提 PR：`feat/c-l3-sim-policy-noise`（完成 C-T1 尺寸参数化、C-T2 分级制动、C-T3 观测噪声、C-T4 弧长优化、C-T6 Simplex 安全兜底与看门狗降级）；
   - A 提 PR：`feat/a-l3-waymo-data`（完成 A-T1 Waymo 弯道加载、A-T3 精选 Waymo JSON 入库、A-T4 收编 `abort_cut_ins`，并把 `.npz` 发给 P）。
2. **第二波（模型与评估接入，Day 4–7）**：
   - P 提 PR：`feat/p-l3-neural-ua`（完成 P-T1 特征、P-T2 方案 B `NeuralPredictor` 与 `.npz` 权重、P-T3 不确定度包络膨胀 TTC、P-T4 `exp01`）；
   - D 提 PR：`feat/d-l3-eval-metrics`（完成 D-T1 系统注册、D-T2 舒适性指标、D-T2b 反事实物理末端安全三分类、D-T3 `run_matrix` 扩展）。
3. **第三波（Demo 联调与终稿实验，Day 8–12）**：
   - C 提 PR：`feat/c-l3-demo-upgrade`（完成 C-T5 2.5D Demo 多系统 + Waymo JSON + $1\sigma$ 预测带）；
   - D 提 PR：`feat/d-l3-final-experiments`（跑完 `exp02/03/05` 全量 + `exp07` Waymo 路考 + `exp08` 抗噪与舒适性消融，更新报告图表）。

### 每次提 PR 前的统一自检门禁
```bash
PYTHONPATH="$PWD/src" pytest -q -k 'not test_load_toy_excludes_ego_and_uses_the_file_stem_as_id and not test_load_toy_ego_returns_the_ego_clip'
node --test tests/test_presentation_events.cjs
ruff check src tests experiments examples
git diff --check
```

---

## 8. 明确标记的未办事项（Backlog / 不纳入本次开发）

- `[未办]` `experiments/exp04_latency.py` 保持现状（观测噪声实验已由 D 的 `exp08` 覆盖）。
- `[未办]` `src/pbs/data/loader.py::load_nuscenes_tracks` 保持 `NotImplementedError`（真实数据统一采用 Waymo）。
- `[未办]` `src/pbs/sim/simulator.py` 非零 `ComputationDelayModel` 异步执行队列保持 `NotImplementedError`（运行期帧级 50 ms 预算裁剪、缺测/过期状态机与延迟裕度保护已由 C-T6 `FrameFailsafeCoordinator` 覆盖）。

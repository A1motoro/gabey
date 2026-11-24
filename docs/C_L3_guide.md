# C（M3 · `pbs.sim` & `pbs.demo`）L3 开发指导文档

> **维护者**：C（前背痛）  
> **基准分支**：`main`（`a3f91cb`，PR #13 合入后的统一 L2 基线）  
> **目标节点**：2026-12-08 Final 展示 + Live Demo / 2026-12-13 代码包与报告终稿  
> **核心原则**：**只加不改，严格向后兼容**。L2 默认行为（`5.0 m × 2.5 m` 矩形、`HysteresisPolicy(2.0, 3.0, 6.0)`、无噪声、348 项既有测试）必须保持 100% 不破坏；所有 L3 物理与控制升级均通过**可选参数或新类**显式开启。

---

## 1. L3 总览：C 负责哪 6 件事？

依据 `docs/spec.md`、`docs/interfaces.md` 与团队 L3 共识，C 负责仿真底座升级与 2.5D Live Demo 升级，共拆分为 6 个可独立单测的子任务（按推荐开发顺序排列）：

| 顺序 | 子任务 | 主要文件 | 状态 / 对接方 |
| --- | --- | --- | --- |
| **T1** | 车辆尺寸参数化（L3 支持 `5.0 m × 2.0 m`） | `src/pbs/sim/collision.py`、`src/pbs/types.py`、`src/pbs/sim/simulator.py`、`src/pbs/risk/ttc.py`、`src/pbs/risk/assessors.py` | ✅ 已实现（`feat/c-l3-sim-policy-noise`） · P / D |
| **T2** | 三级滞回分级制动 + 双档 Jerk 限幅（`StagedBrakingPolicy`） | `src/pbs/sim/policy.py`、`src/pbs/sim/simulator.py` | ✅ 已实现（含 `COAST_ZERO` / `brake_mode` / `mode_override`） · D |
| **T3** | 因果高斯观测噪声模型（`GaussianObservationModel`） | `src/pbs/sim/observation.py` | ✅ 已实现（协议放宽 `Trajectory \| None` 支持 T6 丢包） · P / D |
| **T4** | 弯道 `ego_path` 弧长预计算与性能优化 | `src/pbs/utils.py`、`src/pbs/sim/simulator.py` | ✅ 已实现 · A（Waymo 弯道） |
| **T5** | 2.5D 浏览器 Live Demo 升级（多预测器切换 + Waymo 真实片段加载） | `src/pbs/demo/scene.py`、`service.py`、`assessors.py`、`server.py`、`static/*` | 🚧 进行中（`feat/c-l3-demo-upgrade`） · A / P / 展示 |
| **T6** | 帧级 Simplex 安全兜底、缺测状态机与统一 L3 执行链路（`FrameFailsafeCoordinator`） | `src/pbs/sim/failsafe.py`、`src/pbs/sim/_failsafe_channels.py`、`src/pbs/sim/policy.py`、`src/pbs/sim/simulator.py`、`src/pbs/types.py`、`tests/test_failsafe.py` | ✅ 已实现（待提交并合并） · P / D |

---

## 2. 逐项规格与接口设计

### T1：车辆尺寸参数化（支持 L3 `5.0 m × 2.0 m`）

**背景**：L2 锁定了 `VEHICLE_LENGTH = 5.0`、`VEHICLE_WIDTH = 2.5`（对应既有 348 项测试和已定稿 L2 基线）。L3 为更贴近真实乘用车横向余量，启用 **`5.0 m × 2.0 m`**，但不得直接改动全局默认常量 `VEHICLE_WIDTH`。

**改动方案（只加可选参数）**：
1. **`src/pbs/sim/collision.py`**：
   - 保留原有常量 `VEHICLE_LENGTH = 5.0`、`VEHICLE_WIDTH = 2.5`，新增 `L3_VEHICLE_LENGTH = 5.0`、`L3_VEHICLE_WIDTH = 2.0`。
   - 为 `_projections`、`overlaps`、`in_collision`、`swept_contact_fraction` 追加关键字可选参数：
     ```python
     *,
     vehicle_length: float = VEHICLE_LENGTH,
     vehicle_width: float = VEHICLE_WIDTH,
     ```
   - 内部不再硬编码模块级 `HALF_LENGTH` / `HALF_WIDTH`，改用 `half_l = vehicle_length / 2`、`half_w = vehicle_width / 2`（默认仍为 `2.5` 和 `1.25`，L2 零变化）。
2. **`src/pbs/types.py`**：
   - `RiskContext` 末尾追加可选字段（满足 W4 冻结“只加不改”约定）：
     ```python
     vehicle_length: float = 5.0
     vehicle_width: float = 2.5
     ```
3. **`src/pbs/sim/simulator.py`**：
   - `run_sim(...)` 末尾追加关键字参数：
     ```python
     vehicle_length: float = VEHICLE_LENGTH,
     vehicle_width: float = VEHICLE_WIDTH,
     ```
   - 校验两项均为有限正数；将它们写入 `ctx`、`broad_limit = np.hypot(vehicle_length, vehicle_width)`，并透传给 `in_collision` 与 `swept_contact_fraction`。
4. **协同 P 更新风险侧透传（`src/pbs/risk/ttc.py` & `assessors.py`）**：
   - `assess_risk(...)` 追加 `vehicle_length=VEHICLE_LENGTH, vehicle_width=VEHICLE_WIDTH` 并透传给 `overlaps` / `swept_contact_fraction`；
   - `PredictiveAssessor` 与 `ReactiveAssessor` 从 `ctx.vehicle_length` / `ctx.vehicle_width` 读取当前仿真车身尺寸（避免真值碰撞用 2.0 m、风险评估仍用 2.5 m 的口径割裂）。

---

### T2：分级制动策略 + 双档 Jerk 限幅（`StagedBrakingPolicy`）

> 完整数学定义、逐帧建压时间线与参考代码见 **[docs/staged_braking_and_jerk.md](staged_braking_and_jerk.md)**。

**工程与行业依据**：
- 对齐 **ISO 15622（ACC）** 与 **Euro NCAP AEB** 分级逻辑：
  - **巡航 / 温和恢复（`cruise` / `recover`）**：无威胁时保持目标巡航速度 `v_target`；若此前避险降速且 `TTC > t_off_soft`，以舒适加速度 **`+1.5 m/s²`**（≤ ISO 15622 的 2.0 m/s² 上限）平滑加速恢复至 `v_target`，解决 L2“一刹永慢”问题。
  - **一级舒适预制动（`soft_brake`）**：`t_on_hard ≤ TTC < t_on_soft`（默认 `1.5 s ≤ TTC < 3.0 s`），施加舒适减速度 **`-2.5 m/s²`**（ISO 15622 舒适减速上限 2.0–3.0 m/s²），化解早期切入风险且不引起明显前冲不适。
  - **二级紧急制动（`hard_brake`）**：`TTC < t_on_hard`（默认 `< 1.5 s`），释放阈值 `t_off_hard = 2.0 s`，施加紧急减速度 **`-6.0 m/s²`**（与 L2 最大减速度极限严格可比）。
- **双档 Jerk 限幅（兼顾舒适与极限安全）**：
  - **舒适档 `jerk_soft = 5.0 m/s³`**（用于 `soft_brake`、`recover`、`cruise` 及松刹过程，每 0.1 s 步变化 `0.5 m/s²`，符合 ISO 15622 舒适上限）；
  - **紧急档 `jerk_hard = 20.0 m/s³`**（用于 `hard_brake`，每 0.1 s 步变化 `2.0 m/s²`，从 `0` 到 `-6.0 m/s²` 仅需 `0.3 s` 快速建压；若已有 `-2.5 m/s²` 预制动底压则仅需 **`0.2 s`** 踩满）。

**接口设计**：
1. **`src/pbs/sim/policy.py`** 新增类 `StagedBrakingPolicy`：
   ```python
   SOFT_BRAKE, HARD_BRAKE, RECOVER = "soft_brake", "hard_brake", "recover"

   class StagedBrakingPolicy:
       def __init__(
           self,
           *,
           t_on_soft: float = 3.0,
           t_off_soft: float = 3.5,
           t_on_hard: float = 1.5,
           t_off_hard: float = 2.0,
           a_soft: float = 2.5,
           a_hard: float = 6.0,
           a_recover: float = 1.5,
           jerk_soft: float = 5.0,
           jerk_hard: float = 20.0,
           v_target: float | None = None,
       ): ...

       def reset(self, *, v_target: float | None = None) -> None: ...
       def decide_accel(self, ttc: float, *, v: float, dt: float) -> tuple[str, float]:
           """返回 (action_label, actual_accel [m/s²])，内部维护滞回状态与上一帧加速度以执行双档 Jerk 限幅。"""
   ```
   - **状态转移滞回规则**（严格防抖，边界相等保持原状态）：
     - 任意状态下，只要 `ttc < t_on_hard` → 进入 `hard_brake`（目标加速度 `-a_hard`）；
     - 处于 `hard_brake` 且 `ttc > t_off_hard` → 若 `ttc < t_on_soft` 降级为 `soft_brake`，否则若 `ttc > t_off_soft` 退出制动；
     - 处于 `cruise`/`recover` 且 `ttc < t_on_soft` → 进入 `soft_brake`（目标加速度 `-a_soft`）；
     - 处于 `soft_brake` 且 `ttc > t_off_soft` → 退出制动（若 `v < v_target - 1e-6` 进入 `recover`，目标加速度 `+a_recover`；否则 `cruise`，目标 `0.0`）。
   - **Jerk 限幅与速度钳位**：
     - 限幅：`a = np.clip(a_target, prev_a - max_jerk * dt, prev_a + max_jerk * dt)`；
     - 若 `v == 0` 且 `a < 0`，强制 `a = 0.0`；若 `a > 0` 且 `v + a * dt > v_target`，截断使步末速度精确到达 `v_target`。
   - **动作标签兼容约定**：`action` 列记录 `"cruise"` / `"recover"` / `"soft_brake"` / `"hard_brake"`；建议提供一个辅助函数 `is_braking_action(act: str) -> bool`（`"brake"`, `"soft_brake"`, `"hard_brake"` 均为 `True`），供 D 的 `evaluate()` / `brake_activity()` 统一识别。
2. **`src/pbs/sim/simulator.py`** 运动学步进升级：
   - 新增通用恒加速度步进函数 `advance_kinematics(v: float, a: float, dt: float) -> tuple[float, float]`：
     - 若 `a < 0`：与现有 `advance_motion` 相同，在步内刹停时刻 $h = \min(dt, v / (-a))$ 处停止，不倒车；
     - 若 `a >= 0`：返回 `(v * dt + 0.5 * a * dt * dt, v + a * dt)`。
   - 在 `run_sim` 主循环中：
     - 检测到策略实现了 `decide_accel(ttc, v=v, dt=dt)` 时走连续加速度分支；否则走原 `decide(ttc)` 二值分支（L1/L2 零回归风险）。
     - **碰撞子步扫掠注意**：当 `a > 0`（恢复加速）或 `a < 0`（任意减速度）时，`offsets` 均复用 `advance_kinematics(v, a, h)` 计算自车弧长位置，拐点交叉时刻解二次方程 $v t + \frac{1}{2} a t^2 = \text{distance}$ 即可。
     - `reset` 时若策略接受 `v_target`，传入 `float(scenario.ego_start[3])`。

---

### T3：因果高斯观测噪声模型（`GaussianObservationModel`）

**设计红线**（见 `src/pbs/sim/observation.py` 协议注释）：
- **绝不污染真值**：`run_sim` 碰撞判定永远读原始 `scenario.agents` 真值；噪声只作用于交给 `assessor.assess()` 的 `history` 切片。
- **时间一致性缓存（最容易踩的坑）**：同一个 `(agent_id, timestamp)` 在不同仿真步被反复读取历史时，必须返回**完全相同的噪声偏移**，不能每次 `observe()` 重新随机抽样！

**接口设计**（新增于 `src/pbs/sim/observation.py`）：
```python
class GaussianObservationModel:
    """因果、按 (agent_id, timestamp) 缓存的高斯测量噪声模型。"""

    def __init__(
        self,
        *,
        pos_std: float = 0.15,      # 位置噪声标准差 [m]（典型视觉/雷达融合误差 0.1–0.3 m）
        heading_std: float = 0.03,  # 航向噪声标准差 [rad]（约 1.7°）
        speed_std: float = 0.25,    # 速度噪声标准差 [m/s]
        seed: int = 0,
    ): ...

    def observe(self, history: Trajectory, *, t_now: float) -> Trajectory:
        # 对 history.t 中每个时间戳，以 (history.agent_id, round(float(ti), 6)) 为 key 查缓存；
        # 未命中时用 self._rng 采样 (dx, dy, dv, dheading) 并缓存；
        # 叠加后裁剪 v >= 0，返回新 Trajectory（保留原 t、agent_id、agent_type，accel 置 None）。
```
- **推荐噪声档位**（供 D 跑抗噪对比实验）：
  - 无噪：`pos_std=0.0, heading_std=0.0, speed_std=0.0`
  - 轻噪（雷达/融合）：`pos_std=0.10 m, heading_std=0.02 rad, speed_std=0.15 m/s`
  - 中噪（纯视觉/远距）：`pos_std=0.25 m, heading_std=0.05 rad, speed_std=0.40 m/s`

---

### T4：Waymo 弯道 `ego_path` 性能优化

**问题定位**：目前 [src/pbs/utils.py](file:///Users/qianbt/Downloads/1CUHKSZstudy学习/AIE19/AIE1903/gabey/src/pbs/utils.py#L54-L65) 的 `path_point(path, s)` 和 `path_heading(path, s)` 每次调用都会重新计算整条折线的 `_cum_arclength(path)`。在 Waymo 真实弯道（`ego_path` 有 100–200 个点）上，`simulator.py` 每 0.01 s 碰撞子步循环调用 `path_point` 会导致显著性能退化。

**改动方案**：
- 在 `src/pbs/utils.py` 中为 `path_point(path, s, *, cum=None)` 与 `path_heading(path, s, *, cum=None)` 增加可选参数 `cum: np.ndarray | None = None`（兼容原调用）；
- 在 `simulator.py` 中复用已算好的 `path_distances`，全循环传入 `cum=path_distances`，并把 `ego_rollout` 改为向量化 `np.interp` 一次插出全部 `H` 个点，消除弯道仿真瓶颈。

---

### T5：2.5D 浏览器 Live Demo 升级（面向 12/8 现场展示）

**升级目标**（复用现有 `SceneAdapter`、`AssessorBundle`、`road_lines` 架构，最小化前端重写）：

1. **后端接口升级（`src/pbs/demo/`）**：
   - `assessors.py`：扩展 `make_demo_assessor(name)`，支持从 `pbs.systems.SYSTEMS` 构造任意系统：`reactive`、`cv`、`ca`、`ctrv`、`lr`、`oracle`（以及 P 交付后自动接通的 `neural`）。
   - `scene.py`：新增 `WaymoJsonAdapter(SceneAdapter)`，读取 A 预先导出存放在仓库内（如 `src/pbs/demo/scenes/waymo/*.json`，单文件 < 100 KB，无需装 TensorFlow/Waymo SDK）的精选 Waymo 真实弯道/切入片段，返回包含真实 `road_lines`、曲线 `ego_path` 与多 agent 轨迹的 `DemoScene`。
   - `service.py`：
     - `simulate(config)` 接受新增可选字段：
       - `left_system`（默认 `"reactive"`）、`right_system`（默认 `"cv"`）
       - `scene_id`（默认 `"synthetic"`，或某个 Waymo 片段 ID）
       - `policy`（`"hysteresis"` L2 二值 / `"staged"` L3 分级制动）
       - `vehicle_width`（默认 L3 `2.0`，也可切换 `2.5`）
       - `noise_level`（`"none"` / `"low"` / `"medium"`，接入 `GaussianObservationModel`）
     - **支持方案 B 不确定度预测可视化**：`RecordingAssessor` 读取并记录预测器属性 `latest_std`（形状 `(H, 2)`，由 P 的 `NeuralPredictor` 在每次 `predict()` 后更新；传统确定性预测器无此属性则为 `None`），前端在开启预测线时一并绘制 $1\sigma$ 预测不确定度椭圆/阴影带。
     - 合成场景 `SyntheticCutInAdapter` 的 `duration` 增加 `LOG_PAD = 4.0`（生成到 `14.0 s`、仿真 `10.0 s`），避免在 Demo 里切到 `Oracle` 时最后几秒端点冻结。
2. **前端 UI 升级（`src/pbs/demo/static/`）**：
   - 保持现有 `DESIGN.md` 极简双栏对比视觉风格；
   - 顶部/参数区新增：**场景源选择器**（Synthetic Cut-in / Waymo 精选弯道切入片段）、**左/右系统下拉框**、**策略切换（L2 滞回 / L3 分级）** 与 **噪声开关**；
   - 速度图与状态徽章支持显示 `SOFT BRAKE`（琥珀色）/ `HARD BRAKE`（红色）/ `RECOVER`（浅绿），指标栏增加 **Max Decel** 与 **Peak Jerk**；
   - 画布渲染已原生支持 `road_lines` 折线与任意 `heading`，只需确认弯道下相机跟随与多 agent 绘制居中流畅即可。

---

### T6：帧级 Simplex 安全兜底、缺测状态机与统一 L3 执行链路（`FrameFailsafeCoordinator`）

> 完整规格、四项设计评审修正与验收用例见：**[docs/reactive_failsafe_and_watchdog.md](reactive_failsafe_and_watchdog.md)** 与 **[docs/T6_failsafe_design_review.md](T6_failsafe_design_review.md)**。

**背景与架构定位**：
- 明确 **`ReactiveAssessor` 是所有预测型 AEB 系统的最终安全兜底机制**，但不能采用「单目标同步调用 + 外部冻结 `prev_a`」的简化实现；
- 不修改既有独立的 `ReactiveAssessor` / `PredictiveAssessor`（保持 `exp02/03/05` 独立消融纯净），在 `src/pbs/sim/failsafe.py` 新增帧级协调器并协同 `StagedBrakingPolicy`：
  1. **整帧 50 ms 共享预算与并行非阻塞截止**：每帧固定观测快照与 `frame_id`，Reactive 与预测通道并行（预测按帧批量处理多目标）；到 50 ms 截止点只取帧号匹配的已就绪结果，迟到帧丢弃、忙碌跳过防积压；同时支持可控虚拟时钟（确定性仿真与单测）与异步 worker（真实模型压测）双后端；
  2. **双通道共同缺测与按 `agent_id` 失联追踪**：严格区分有效 `TTC = +∞` 与 `unknown` / 过期（从最后可信采样时间戳算观测年龄，旧时间戳重发不清零）；已在制动保持原档位，巡航/恢复首帧缺测先下发 `coast_zero`（目标加速度 `0.0`）、满 `missing_grace_s = 0.20 s` 进入软刹；高风险前车失联保留制动不被其他正常目标解除；松刹需连续 `recovery_confirm_frames = 2` 帧新鲜有效观测并满足滞回退出条件；
  3. **Hold 锁档位 + 统一 L3 动力学执行链路**：废除外部复制 `_last_accel`，在 `StagedBrakingPolicy` 内部统一经过同一个 `prev_a`、双档 Jerk 限幅（`5.0` / `20.0 m/s³`）与 `_speed_limit_accel` 前瞻卸力（刚进硬刹 `-2` 丢帧继续走到 `-4 → -6`，恢复 `+1.5` 丢帧按 `+1.0 → +0.5 → 0` 平滑撤力）；
  4. **独立延迟裕度与兼容边界**：对有效有限 TTC 施加 `TTC_eff = max(0, TTC_raw - latency_margin)`，与 `missing_grace_s` 独立配置与调参；L2 默认路径遇空历史仍抛 `MissingObservationError` 保持零回归。

---

## 3. 与 A / P / D 的接口交接点

- **C ← A**：A 提供 2–5 个预导出 Waymo 精选场景 JSON（约定字段：`timestamps`, `ego_path`, `ego_start`, `agents`, `road_lines`, `lane_width`, `meta`），保证无缺帧、时间从 0 起、已转世界局部米制单位。
- **C ← P**：P 交付 `NeuralPredictor` 注册进 `pbs.systems.SYSTEMS["neural"]` 后，C 的 Demo 自动出现在下拉列表中，并可直接接入 `FrameFailsafeCoordinator` 进行帧级并行容错运行；同时 P 协助在 `assess_risk` / `assessors.py` 中透传 `ctx.vehicle_length` / `ctx.vehicle_width`。
- **C → D**：C 交付 `StagedBrakingPolicy`、`GaussianObservationModel`、`FrameFailsafeCoordinator`、`run_sim(..., vehicle_width=2.0)` 以及 `is_braking_action()` 后，D 即可在 `pbs.eval` 中实现舒适性指标（`max_decel`、`peak_jerk`、`rms_jerk`、`speed_loss`、`false_brake_severity`）并跑出 L3 全套对比图与容错验证。

---

## 4. C 的 L3 验收与测试清单

每完成一个子任务需补对应 `pytest` 测试并确保全仓回归 + Ruff 通过：

- [ ] **`tests/test_collision.py` & `tests/test_simulator.py`**：验证 `vehicle_width=2.0` 与默认 `2.5` 下的 OBB 重叠、扫掠接触及 `RiskContext` 尺寸透传；验证默认调用 L2 全部 348 项测试零回归。
- [ ] **`tests/test_policy.py` & `tests/test_simulator.py`**：
  - `StagedBrakingPolicy` 状态机滞回边界（`cruise → soft_brake → hard_brake → soft_brake → recover → cruise`）；
  - Jerk 限幅下 `max(|Δa/dt|) <= max_jerk + 1e-9`；
  - 恢复加速精确停在 `v_target` 不超速、制动减到 0 不倒车；
  - 加速阶段碰撞子步扫掠准确捕捉接触。
- [ ] **`tests/test_observation.py`（新增）**：验证 `GaussianObservationModel` 固定 `seed` 可复现、同一 `(agent_id, t)` 多次读取噪声完全一致、不修改原 `Scenario` 真值、`v >= 0`、`accel is None`。
- [ ] **`tests/test_failsafe.py`（新增）**：按 [T6_failsafe_design_review.md](T6_failsafe_design_review.md) 四项验收逐条验证：① 可控时钟模拟 Reactive 8 ms / 预测 80 ms 在 50 ms 截止发令且 80 ms 迟到帧按 `frame_id` 丢弃、多目标共享 50 ms 预算、任务卡死不阻塞；② 对比 `missing_grace_s ∈ {0.10, 0.20, 0.30} s`、旧时间戳重发不清零年龄、仅高风险前车失联时其他目标不能解除制动、两帧新鲜恢复防抖；③ 逐帧检验进入缺测/连续缺测/恢复/停车四处边界的实际 Jerk 上限（`-2 → -4 → -6` 硬刹继续建压、`+1.5 → +1.0 → +0.5 → 0` 恢复平滑撤力）；④ `latency_margin = 0.20 s` 的进入/退出与边界相等保持、与 `missing_grace_s` 独立调参。
- [ ] **`tests/test_demo.py` & `tests/test_demo_extensions.py`**：验证双侧任意系统切换（如 `cv` vs `ca`、`reactive` vs `oracle`）、`policy="staged"`、`vehicle_width=2.0`、`WaymoJsonAdapter` 离线加载、JSON 严格无 `NaN/Inf`。
- [ ] **质量门禁**：`ruff check` 0 警告、`git diff --check` 无空白错误、`pytest` 全绿、`node --test tests/test_presentation_events.cjs` 全绿。

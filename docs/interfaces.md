# Interfaces（维护者：C）

> W4 冻结。冻结后**只加不改**：可新增字段 / 可选参数，不得重命名、删除或改变语义。改动需在 PR 中 @ 全员。设计依据见 [spec.md](spec.md)。

[L2 基线规范](l2-baseline.md)定义滞回设置、定向矩形和距离口径。位置字段表示车身中心。L2 将 `run_sim` 和 `assess_risk` 的 `r_safe` 默认值从 2.5 改为 `None`。省略参数会选择 L2 几何。复现 L1 时显式传入 `r_safe=2.5`。车辆真值扫掠在每个子步内以中点航向做平移扫掠，旋转和加速度为子步近似。

## 约定

- 单位：m / s / m·s⁻¹ / rad；统一世界坐标（不使用 grid cell）。
- 仿真步长 `dt = 0.1 s`；位置数组 `(N, 2)`，列为 `(x, y)`。
- `Scenario.ego_path` 同时是本车道中心线，车道宽 `Scenario.lane_width`（默认 3.5 m）。
- 每个 agent 必须有唯一 `agent_id`（Oracle 依赖）。

### M1 追加：Waymo L3（2026-11-23，A）

- 新增 `load_womd_tracks(path, track_index=None, anchor=20, duration=6, history_steps=20,
  group="cut_in", scenario_id=None)`。NPZ 返回快照内周车；多记录 TFRecord 必须指定 `scenario_id`，
  避免默默选错片段。旧 `load_nuscenes_tracks` 占位接口保留兼容。
- `womd.real_scenario(scene, track_index, anchor=..., group=...)` 返回固定 anchor 坐标的
  `Scenario(kind="real_cut_in"/"real_control")`；`ego_start=(0,0,0,v0)`、直线 `ego_path` 足够长。
  本次仅保留 ego 直行窗口和选定单目标，不把这些真实 kind 加入合成 `KINDS`/96 场景分母。
- 官方 `sdc_track_index` 校验后排除 ego；整段 ego/目标 `valid=True`；无跨空洞插值。
  `t=[-2,6]`、81 帧；保留真实微抖时间戳并对齐名义 10 Hz，元数据记录误差。
- `Trajectory.v/heading` 编码 Waymo 速度向量；`accel=None`。拟合横向加速度只放诊断元数据。
- `save_clip/load_clip` 使用 NumPy 数组和 JSON、`allow_pickle=False`。快照与实验不需要
  protobuf/TFRecord；只有原始读取需要 `pip install -e ".[womd]"`。
- L3 冻结回放与审计显式传 `duration=sc.params["duration"], r_safe=2.5`，保留圆形兼容配置；
  L2 主线继续使用默认矩形，数值不能混用。Forecast 评测的 20 帧含当前帧（跨度 1.9 s），
  真值从下一帧开始；GRU 的窗口预处理与回退计数仍由 P 明确实现。
- `params.cohort="illustrative"` 的危险/误刹说明案例按结果选择，必须与首批统计分开。
  数字与复现见 [L3_waymo_case_study.md](L3_waymo_case_study.md)。

## 数据结构 — `pbs.types`

| 类型 | 用途 | 字段 |
| --- | --- | --- |
| `Trajectory` | agent 轨迹 | `t, x, y, v, heading: (N,)`, `agent_type`, `agent_id` |
| `Scenario` | 仿真输入 | `ego_start=(x,y,heading,v)`, `ego_path: (M,2)`, `agents`, `dt`, `params`, `kind`, `lane_width` |
| `SimLog` | 仿真输出 | `t, ego_x, ego_y, ego_v, ego_a, ttc: (n,)`, `action`, `min_dist`, `collision`。可选字段 `executed_steps`、`first_collision_t`、`min_dist_until_contact` 见[日志字段说明](l2-baseline.md#日志与距离口径) |
| `RiskContext` | C → P，每步只读 | `t_now, ego_xy, ego_heading, ego_v, ego_future: (H,2), ego_path, lane_width, horizon, dt, r_safe, ego_future_heading` |
| `RiskEstimate` | P → C | `ttc`（无冲突 inf）, `conflict_step`, `considered`（Reactive 忽略时 False） |

## P / C 边界

```
C (pbs.sim)                                    P (pbs.risk, pbs.prediction)
────────────                                   ────────────────────────────
r_safe, footprint, in_collision (ground truth)
ego_rollout → RiskContext ───────────────────► RiskAssessor.assess(ctx, history) → RiskEstimate
history_until(agent, t − latency) ───────────►   Reactive: in-lane only, range / range-rate
min over agents of est.ttc ◄──────────────────   Predictive: predictor.predict → assess_risk
policy.decide(ttc) → brake / cruise
```

## 函数接口

| 模块 | 签名 | Owner | 状态 |
| --- | --- | --- | --- |
| M1 | `load_toy() -> list[Trajectory]` | A | ✅ |
| M1 | `load_womd_tracks(path, ...) -> list[Trajectory]`；`womd.real_scenario(scene, track_index, ...) -> Scenario` | A | ✅ L3：Waymo 原始 / NPZ，ego 排除 |
| M1 | `womd.load_clip(path)` / `save_clip(scenario, path, evidence=...)`；`read_scenarios(path)` | A | ✅ L3：NPZ 不需 protobuf / 原始数据；原始读取验证 CRC32C |
| M1 | `make_scenario(kind, **params) -> Scenario` | A | ✅ |
| M1 | `quintic_lateral(y0, y1, T, t, t0=0) -> ndarray`；`peak_lateral_acc(dy, T)` | A | ✅ |
| M1 | `quintic_lateral_vel / quintic_lateral_acc`（同上签名）；`min_lc_duration(dy, max_lat_acc)` | A | ✅ 新增 |
| M1 | `scenario_set(kinds, grid, max_lat_acc, strict) -> ScenarioSet`；`canonical(name) -> Scenario` | A | ✅ 新增 |
| M1 | `ca_scenario_set() -> ScenarioSet`；`CA_GRID`（12 个带纵向加速度的 cut-in） | A | ✅ 新增（CA） |
| M1 | `prediction_slices(scenario, history, horizon, stride) -> list[PredictionSlice]` | A | ✅ 新增（CA） |
| M2 | `Predictor.predict(history, horizon, dt) -> (horizon, 2)`；可选 `bind(scenario)` | P | 接口 ✅；`StaticPredictor` 占位 |
| M2 | `ConstantVelocity / ConstantAccel / CTRV / LinearRegression / Oracle` | P | CV / CA / CTRV / Oracle ✅ L2；LR ✅ L3 |
| Risk | `assess_risk(ego_future, agent_future, dt, r_safe=None, *, ego_heading=None, agent_heading=None, pedestrian=False, ego_origin=None, agent_origin=None, ego_origin_heading=None, agent_origin_heading=None, swept=False) -> RiskEstimate` | P | `swept=False` 时第 k 行时间为 `(k+1)*dt`；`swept=True` 时必须同时提供 `ego_origin` 与 `agent_origin`，按步间线性扫掠（中点航向）返回连续 TTC `(k + fraction)·dt`。几何规则见 [L2 基线](l2-baseline.md#制动与碰撞) |
| Risk | `RiskAssessor.assess(ctx, history) -> RiskEstimate`；`ReactiveAssessor`、`PredictiveAssessor(predictor)` | P | ✅ L2；Predictive 将延迟观测对齐当前时间；L2 矩形模式（`r_safe=None`）自动传起点并启用 `swept=True`，L1 圆形模式保持离散 |
| M3 | `run_sim(scenario, assessor, policy, duration, horizon, r_safe, latency, collision_dt, …) -> SimLog` | C | 已实现 L2 闭环 |
| M3 | `ego_rollout(path, s, v, horizon, dt)`、`in_collision(p, q, r_safe)` | C | ✅ |
| M3 | `HysteresisPolicy(t_on, t_off, a_brake)`、`NeverBrakePolicy`、`TTCThresholdPolicy(ttc_threshold, a_brake)` | C | 依次用于 L2 主实验、无制动对照和 L1 复现 |
| M4 | `make_system(name)`；`SYSTEMS`、`CORE` | D | ✅ |
| M4 | `is_benign(scenario) -> bool`；`evaluate(log, benign) -> dict` | D | ✅ L2；`evaluate` 额外返回 `first_collision_t` 与 `min_dist_until_contact`，并按首次接触截断 `brake_onset_t` |
| M4 | `run_matrix(..., policy_kind="threshold", hysteresis_gap=1.0, a_brake=6.0)`、`tradeoff`、`failure_boundary`、`safe_area`、`rescue_rate`、`plot_boundaries` | D | ✅ L2；支持滞回策略并记录 `policy`/`t_on`/`t_off`/`geometry`/`horizon`/`first_collision_t`/`min_dist_until_contact`；`exp04` latency 仍为 L3 |

### M2 追加：P L2（2026-11-18）

- 原 `predict(history, horizon, dt)`、`bind(scenario)`、模型名称与时间语义不变。
  CA 新增可选构造参数 `m=10`（整数 ≥ 3）；CTRV 新增 `m=10`（整数 ≥ 2）。
- CA 从最近至多 m 帧位置按真实时间间隔作二次最小二乘拟合，不读取 `Trajectory.accel`
  或对 `v` 差分；不足 3 帧使用 CV。沿拟合速度方向减速至零后停止该方向运动，
  垂直方向加速度继续外推；最后两个位置相同或刹车拟合反向过冲时保持原位。
- CTRV 从最近至多 m 帧 `heading` 解缠后拟合转率，以最后观测的 `v`/`heading`
  外推圆弧；不足 2 帧或 `|yaw_rate| < 1e-8 rad/s` 使用原二维 CV。
- Oracle 在 bind 时检查唯一非空 ID 并复制真值，以最后观测时间为起点查询，插值与
  `state_at` 一致（超出范围保持端点）。只有真值覆盖所有请求时刻时才可解释为零预测误差上界；
  用于长时域或延迟实验时，A/D 应保证足够的真值覆盖。重新 bind 会替换原场景映射。
- 三个模型复用 L1 输入校验，允许 `horizon=0` 返回 `(0, 2)`，不修改输入。
  延迟对齐继续由已有 `PredictiveAssessor` 完成，其网格间插值近似保持不变。

### M2 追加：P L3（2026-11-18）

- `LinearRegressionPredictor(m=10)` 已实现，模型名 `lr` 和原 `predict`/`bind` 接口不变。
  m 为整数 ≥ 2；只取最近至多 m 个位置样本，按真实采样时间拟合 x(t)、y(t) 直线。
- 查询时间仍为 `t_last + (k+1)·dt`。位置使用拟合截距而非强制经过最后噪声观测，
  以保留窗口平滑效果；至少 2 帧时不使用速度/航向元数据或 `Trajectory.accel`。
- 单帧按速度/航向退化为 CV；输入校验与 `(0, 2)` 空时域行为复用 L1。
  不读取未来、不修改输入，延迟对齐由现有 `PredictiveAssessor` 完成。
- D 的 `make_system("lr")` 已可直接使用。`CORE` 四边界集合保持原定义。

### M1 补充约定（2026-10，A）

- 合成 `Scenario.params` 必含 `duration`（默认 15 s，这是日志长度）与 `peak_lat_acc`。闭环实验显式传入仿真时长；`run_sim` 默认 10 s。D 的 L2 比较仿真 19 s、日志 23 s：刹车在前 15 s 计分，碰撞看到 19 s（[l2-baseline.md](l2-baseline.md)）。若为 Oracle 生成额外未来轨迹，仍使用[实验设定的评分时长](l2-baseline.md#c-的复现配置与-d-的接入)。
- 合成 `Scenario.agents` 恒为 1 个元素；`agent_id ∈ {"cutter", "lead", "crosser"}`。
- 合成 `Scenario.kind ∈ {"cut_in", "lead_brake", "crossing"}`；`make_scenario` 对未知 kind / 超限横向加速度 / 反向目标速度 **抛 `ValueError`**，不静默裁剪。真实 Waymo kind 单独见前文 L3 约定。
- `load_toy()` 不含 ego（`agent_id` = 文件 stem）；ego 用新增的 `load_toy_ego()`。

### M1 追加：CA 支持（2026-11-18，A）

- `Trajectory.accel: (N, 2) | None` —— **只加不改**的可选字段，列 = `(a_x, a_y)` [m/s²]。
  合成场景由 builder 解析填充；`load_toy()` 为 `None`（速度列有噪声，不要求导）。
  **M2 只能把它当测试真值，不能在 `predict()` 里读**。
- `make_scenario("cut_in", a_cutter=…, accel_T=…)`：纵向加速度只作用于
  `[cut_in_t, cut_in_t + accel_T]`（默认 `accel_T = lc_T`），之后保持增速；`a_cutter=0`
  与旧行为逐位一致。`|a_cutter| ≤ 3 m/s²`，坡段结束速度 < 0 时抛 `ValueError`。
- `CA_GRID` / `ca_scenario_set()`：12 个 cut-in（`gap_d ∈ {20, 30}`、`rel_v ∈ {0, 2}`、
  `a_cutter ∈ {−2, 0, 2}`），6 benign / 6 dangerous。**不并入 `DEFAULT_GRID`**（D 的 96 场景矩阵不变）。
- `prediction_slices(scenario, history=1.0, horizon=2.0, stride=0.5)`：返回
  `PredictionSlice(t0, history, truth, truth_accel)`；`history` 是**固定长度的滑动窗口**
  `[t0 − history, t0]`（`history=1.0` 时恒为 11 个点，随 `t0` 平移，不是从 0 秒起的全部前缀），
  只含 `t ≤ t0` 的观测，`truth[k]` 是 `t0 + (k+1)·dt` 的真值位置，`truth_accel` 同网格。时间单位是秒。

## C 第一版实现补充（2026-10-20）

本节记录 C 第一版的兼容扩展和时间语义。
详见 [C baseline 接入与降级策略](C_baseline.md)。提交 PR 时请提示 A/P/D 审阅以下行为。

- 新增 `HysteresisPolicy(t_on=2, t_off=3, a_brake=6)`，支持 `decide(ttc)` 与 `reset()`。
  run_sim 对提供 reset 的策略每次重置；原单阈值和 NeverBrake 保持可用。
- run_sim 原参数顺序不变，末尾追加 `collision_dt=0.01` 秒：步内线性扫掠的最大子步，
  加速度运动仍有近似，按实验需要验证收敛。
- SimLog 末尾追加可选字段 `executed_steps: int | None = None`。
  新日志 K+1 行状态，仅前 K 个动作执行。最后一行 action 保持上一步值，ego_a=0，
  ttc 记录终点评估，不调用策略或额外移动。evaluate 已兼容；旧日志为 None 时保持旧统计。
- ego_a 是步开始的控制加速度，不是整步平均加速度；步内停车后保持静止。
- 校验时间步长、轨迹格式、唯一 ID、真值覆盖、自车起点与路径方向，路径需覆盖实际运动及 rollout。
- history_until 在没有历史时返回空 Trajectory，不再补未来首帧。
  run_sim 抛出 MissingObservationError（ValueError 子类），不输出正常安全日志。
- A 需提供延迟所需预历史；P 定义其最小历史长度，并将预测对齐到 ctx.t_now+(k+1)*dt。
  原 Predictor 接口以最后观测为基准，非零延迟的时间对齐仍由 P 完成。
- 记录碰撞后继续到指定时长，collision 永久保留 True；不模拟碰撞冲击。

## 可选 2D LiDAR 感知接口（2026-11-27）

`run_sim(..., *, perception_model: PerceptionPipeline | None = None)` 是可选扩展，
不修改 `Trajectory` / `Scenario` / `SimLog` 字段和既有默认行为。

- 每个运行开始重置感知器。传入每步实际自车位姿与当前周车 footprint；只有射线传感器读取 footprint。
- 聚类、L-Shape 拟合与 KF 跟踪只接收无标签点云 / 检测框，输出自己的 `lidar_*` 轨迹 ID。
- 轨迹默认至少两次实际检出才确认；单次检出不能提供运动速度。已有航迹最多外推 3 个丢失扫描，再删除。
- 全盲区、尚未建轨的目标不评估；真值碰撞仍逐步检查，不能把未检出当成没有碰撞。
- `latency` 延迟交付保存的扫描结果，自车位姿保持扫描时视角；启动暂无交付结果时允许空目标列表。
  此模式只要求真值覆盖 `[0, duration]`，不借用真值预历史初始化感知。原轨迹模式仍要求延迟预历史。
- `observation_model` 与 `perception_model` 同时传入时报错；Oracle 的真值 ID 接口与无标签轨迹不兼容，明确报错。
- 轨迹的 `accel=None`；检测尺寸与位置协方差保存在独立诊断帧，风险端仍使用原固定车身。
  感知协方差不自动加入 P 的 NN 预测不确定度。语义分类未实现，风险端按通用车辆处理。

入口、复现及限制见 [2D LiDAR 新手说明](2d_lidar_guide.md)。

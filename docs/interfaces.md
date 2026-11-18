# Interfaces（维护者：C）

> W4 冻结。冻结后**只加不改**：可新增字段 / 可选参数，不得重命名、删除或改变语义。改动需在 PR 中 @ 全员。设计依据见 [spec.md](spec.md)。

## 约定

- 单位：m / s / m·s⁻¹ / rad；统一世界坐标（不使用 grid cell）。
- 仿真步长 `dt = 0.1 s`；位置数组 `(N, 2)`，列为 `(x, y)`。
- `Scenario.ego_path` 同时是本车道中心线，车道宽 `Scenario.lane_width`（默认 3.5 m）。
- 每个 agent 必须有唯一 `agent_id`（Oracle 依赖）。

## 数据结构 — `pbs.types`

| 类型 | 用途 | 字段 |
| --- | --- | --- |
| `Trajectory` | agent 轨迹 | `t, x, y, v, heading: (N,)`, `agent_type`, `agent_id` |
| `Scenario` | 仿真输入 | `ego_start=(x,y,heading,v)`, `ego_path: (M,2)`, `agents`, `dt`, `params`, `kind`, `lane_width` |
| `SimLog` | 仿真输出 | `t, ego_x, ego_y, ego_v, ego_a, ttc: (n,)`, `action`, `min_dist`, `collision` |
| `RiskContext` | C → P，每步只读 | `t_now, ego_xy, ego_heading, ego_v, ego_future: (H,2), ego_path, lane_width, horizon, dt, r_safe` |
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
| M1 | `make_scenario(kind, **params) -> Scenario` | A | ✅ |
| M1 | `quintic_lateral(y0, y1, T, t, t0=0) -> ndarray`；`peak_lateral_acc(dy, T)` | A | ✅ |
| M1 | `quintic_lateral_vel / quintic_lateral_acc`（同上签名）；`min_lc_duration(dy, max_lat_acc)` | A | ✅ 新增 |
| M1 | `scenario_set(kinds, grid, max_lat_acc, strict) -> ScenarioSet`；`canonical(name) -> Scenario` | A | ✅ 新增 |
| M1 | `ca_scenario_set() -> ScenarioSet`；`CA_GRID`（12 个带纵向加速度的 cut-in） | A | ✅ 新增（CA） |
| M1 | `prediction_slices(scenario, history, horizon, stride) -> list[PredictionSlice]` | A | ✅ 新增（CA） |
| M2 | `Predictor.predict(history, horizon, dt) -> (horizon, 2)`；可选 `bind(scenario)` | P | 接口 ✅；`StaticPredictor` 占位 |
| M2 | `ConstantVelocity / ConstantAccel / CTRV / LinearRegression / Oracle` | P | CV / CA / CTRV / Oracle ✅ L2；LR ✅ L3 |
| Risk | `assess_risk(ego_future, agent_future, dt, r_safe) -> RiskEstimate`（纯函数） | P | ✅ L1；严格距离 `< r_safe`，第 k 行时间 `(k+1)*dt` |
| Risk | `RiskAssessor.assess(ctx, history) -> RiskEstimate`；`ReactiveAssessor`、`PredictiveAssessor(predictor)` | P | ✅ L1；Predictive 将延迟观测的预测对齐当前时间 |
| M3 | `run_sim(scenario, assessor, policy, duration, horizon, r_safe, latency) -> SimLog` | C | ✅ skeleton |
| M3 | `ego_rollout(path, s, v, horizon, dt)`、`in_collision(p, q, r_safe)` | C | ✅ |
| M3 | `NeverBrakePolicy`、`TTCThresholdPolicy(ttc_threshold, a_brake)` | C | ✅ |
| M4 | `make_system(name)`；`SYSTEMS`、`CORE` | D | ✅ |
| M4 | `is_benign(scenario) -> bool`；`evaluate(log, benign) -> dict` | D | ✅ |
| M4 | `run_matrix`、`tradeoff`、`failure_boundary`、`safe_area`、`rescue_rate`、`plot_boundaries` | D | ✅ L2；`exp04` latency 仍为 L3 |

### M2 追加：P L2（2026-11-18）

- 上表的 M2 状态更新：CV、CA、CTRV、Oracle 已实现；LR 仍为 L3 TODO。
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

- `Scenario.params` 必含 `duration`（默认 15 s）与 `peak_lat_acc`。**`run_sim(..., duration=scenario.params["duration"])`**——
  `run_sim` 的默认值是 10 s，不传会把慢速收敛组合误判成 benign。
- `Scenario.agents` 恒为 1 个元素（每类场景单目标）；`agent_id ∈ {"cutter", "lead", "crosser"}`。
- `Scenario.kind ∈ {"cut_in", "lead_brake", "crossing"}`；`make_scenario` 对未知 kind / 超限横向加速度 / 反向目标速度 **抛 `ValueError`**，不静默裁剪。
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

本节为 C 本次兼容扩展与集成说明；上表保留原骨架记录，不表示 A/P TODO 已完成。
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

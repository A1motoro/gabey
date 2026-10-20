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
| M1 | `load_toy() -> list[Trajectory]` | A | TODO |
| M1 | `make_scenario(kind, **params) -> Scenario` | A | TODO |
| M1 | `quintic_lateral(y0, y1, T, t, t0=0) -> ndarray`；`peak_lateral_acc(dy, T)` | A | TODO / ✅ |
| M2 | `Predictor.predict(history, horizon, dt) -> (horizon, 2)`；可选 `bind(scenario)` | P | 接口 ✅；`StaticPredictor` 占位 |
| M2 | `ConstantVelocity / ConstantAccel / CTRV / LinearRegression / Oracle` | P | CV ✅；其余 TODO（L2/L3） |
| Risk | `assess_risk(ego_future, agent_future, dt, r_safe) -> RiskEstimate`（纯函数） | P | ✅ L1；严格距离 `< r_safe`，第 k 行时间 `(k+1)*dt` |
| Risk | `RiskAssessor.assess(ctx, history) -> RiskEstimate`；`ReactiveAssessor`、`PredictiveAssessor(predictor)` | P | ✅ L1；Predictive 将延迟观测的预测对齐当前时间 |
| M3 | `run_sim(scenario, assessor, policy, duration, horizon, r_safe, latency) -> SimLog` | C | ✅ skeleton |
| M3 | `ego_rollout(path, s, v, horizon, dt)`、`in_collision(p, q, r_safe)` | C | ✅ |
| M3 | `NeverBrakePolicy`、`TTCThresholdPolicy(ttc_threshold, a_brake)` | C | ✅ |
| M4 | `make_system(name)`；`SYSTEMS`、`CORE` | D | ✅ |
| M4 | `is_benign(scenario) -> bool`；`evaluate(log, benign) -> dict` | D | ✅ |
| M4 | `run_matrix`、`tradeoff`、`failure_boundary`、`safe_area`、`plot_boundaries` | D | TODO |

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

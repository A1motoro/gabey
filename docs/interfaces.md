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
| M2 | `ConstantVelocity / ConstantAccel / CTRV / LinearRegression / Oracle` | P | TODO |
| Risk | `assess_risk(ego_future, agent_future, dt, r_safe) -> RiskEstimate`（纯函数） | P | TODO |
| Risk | `RiskAssessor.assess(ctx, history) -> RiskEstimate`；`ReactiveAssessor`、`PredictiveAssessor(predictor)` | P | 接口 ✅；`NullAssessor` 占位 |
| M3 | `run_sim(scenario, assessor, policy, duration, horizon, r_safe, latency) -> SimLog` | C | ✅ skeleton |
| M3 | `ego_rollout(path, s, v, horizon, dt)`、`in_collision(p, q, r_safe)` | C | ✅ |
| M3 | `NeverBrakePolicy`、`TTCThresholdPolicy(ttc_threshold, a_brake)` | C | ✅ |
| M4 | `make_system(name)`；`SYSTEMS`、`CORE` | D | ✅ |
| M4 | `is_benign(scenario) -> bool`；`evaluate(log, benign) -> dict` | D | ✅ |
| M4 | `run_matrix`、`tradeoff`、`failure_boundary`、`safe_area`、`plot_boundaries` | D | TODO |

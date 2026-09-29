# Project Spec v2 — Predictive Braking Simulator

> v2（2026-09-29）：在 v1 基础上合并评审意见——三层 baseline、counterfactual false brake、多边界叠加图、quintic 场景、TTC 由 P 实现。

## 0. 研究问题

> **Prediction 能把 AEB 的安全工作区扩大多少，又在哪里仍然失效？**

- **课程**：AIE1903 · Topic 9 — Predictive Braking Simulator (★★★) · 4 人
- **底线（不可动）**：W4 接口冻结 + walking skeleton 端到端跑通。所有增强排在其后，做不完按 §1.3 砍。

## 1. Scope

### 1.1 In scope

| 项 | 内容 |
| --- | --- |
| Baselines（三层） | **Reactive**（无预测）→ **Predictive**（CV / CA / CTRV / LR）→ **Oracle**（真实未来，上界） |
| 决策 | TTC 阈值二值刹车 (cruise / brake) |
| 仿真 | 2D 离散时间闭环；ego 自主仿真，agent log replay |
| 场景 | 参数化合成：`cut_in`（quintic 换道 + 横向加速度约束）、`lead_brake`、`crossing`；**必须含 benign 样本** |
| 评估 | ADE/FDE、碰撞率、counterfactual 误报率、最小间距、刹车时机、cut-in **lead time** |
| 核心图 | Reactive / CV / CA / Oracle 四条 failure boundary 叠加在 (gap_d, rel_v) 平面 |
| 加分 | latency sensitivity τ ∈ [0, 300] ms |

### 1.2 Out of scope

CARLA / ROS / Docker / GPU；perception；横向控制 (steering)；LSTM / Transformer；3D 与车辆动力学（点质量 + 圆形 footprint）。nuScenes mini 仅作可选增强，优先级低于场景物理合理性。

### 1.3 分层交付 (fallback ladder)

| 层级 | 内容 | 砍掉后仍成立的结论 |
| --- | --- | --- |
| L0 底线 | 接口冻结 + skeleton | — |
| L1 保底 | CV + ADE/FDE；**Reactive vs CV** 在 cut_in 上的 TTC 刹车与 timeline；lead time | "prediction 让刹车提前了 x 秒" |
| L2 核心 | + CA、CTRV、Oracle；counterfactual 误报 + trade-off；**四边界叠加图** | "安全区扩大了多少 / 预测误差代价多少" |
| L3 加码 | LR predictor；latency 实验；nuScenes | 可随时裁剪 |

### 1.4 关键定义：Reactive ≠ CV（proposal 中须写死）

标准 reactive AEB 的 TTC = range / range-rate，本身就是视线方向的匀速假设。若不加区分，它与 2D CV 在数学上近似等价，主线曲线会消失。因此：

| | Reactive | Predictive |
| --- | --- | --- |
| 关注对象 | 仅**已在本车道内**的 agent | 所有 agent |
| 横向运动 | 忽略，只看纵向距离与接近率 | 外推，**在切入完成前**就预判进入本车道 |

**Prediction 的价值 = 提前多少秒察觉切入 (lead time)。** CTRV 建模 turn rate，换道中横向外推优于 CV。

### 1.5 关键定义：False brake（counterfactual）

同一场景跑 `NeverBrakePolicy`：不碰撞 → 该场景 **benign**；benign 场景中任何刹车都是 false brake。场景集必须包含"看起来危险但实际安全"的 cut-in（如 rel_v ≤ 0、大 gap_d），否则误报率恒为 0。

## 2. Modules

数据流：`M1 scenario → M2 prediction → risk (TTC) → M3 sim/decision → M4 evaluation`

| 模块 | 包 | Owner | 内容 |
| --- | --- | --- | --- |
| M1 | `pbs.data` | A | `load_toy`、`make_scenario`、`quintic_lateral`、横向加速度过滤 |
| M2 | `pbs.prediction` | P | `ConstantVelocity`、`ConstantAccel`、`CTRV`、`LinearRegression`（最后做，可砍）、`Oracle` |
| Risk | `pbs.risk` | P | `assess_risk`（纯函数）、`ReactiveAssessor`、`PredictiveAssessor` |
| M3 | `pbs.sim` | C | `run_sim`、ego rollout、`r_safe`/footprint、真值碰撞、latency、policy |
| M4 | `pbs.eval`、`pbs.systems` | D | `is_benign`、`evaluate`、`run_matrix`、trade-off、boundary、`safe_area`、plots |

**P/C 边界**：P 只实现 `RiskContext + history → RiskEstimate` 的纯函数；`r_safe`、footprint、仿真步进、ego rollout 归 C，经 `RiskContext` 只读传给 P。接口细节见 `docs/interfaces.md`。

## 3. 实验矩阵 (M4)

`systems × kind × param grid × ttc_threshold [× latency]`，一次运行一行 DataFrame，其余全部是 groupby。

| 实验 | 系统 | 输出 |
| --- | --- | --- |
| exp01 | CV / CA / CTRV / LR | ADE/FDE vs horizon |
| exp02 | CORE = Reactive / CV / CA / Oracle | 碰撞率 vs 误报率 trade-off |
| exp03 | CORE | **四边界叠加图**；面积差 = prediction 收益 / 预测误差代价；对照刹停距离 `d = v·t_react + v²/(2a)` |
| exp04 | CV（+Reactive） | latency 敏感性 |
| exp05 | Reactive vs CV / CTRV | cut-in lead time 分布 |

## 4. Roles

| 代号 | 角色 | 关键交付 | 报告章节 |
| --- | --- | --- | --- |
| **C** | 统筹 + 仿真 | `run_sim`、`r_safe`/footprint、latency、集成、接口文档 | 系统架构、决策逻辑 |
| **P** | 预测 + 风险评估 | 4 个 predictor + Oracle、`assess_risk`、Reactive/Predictive assessor | 预测方法、Reactive vs Predictive |
| **A** | 场景 | quintic cut-in、benign/dangerous 场景集、三类场景参数化 | 场景设计与物理合理性 |
| **D** | 评估与实验 | counterfactual、实验矩阵、trade-off、边界叠加图、demo | 实验结果、局限性 |

- Review 轮换：D 审 C、P 审 A、C 审 P、A 审 D。D 是 C 的 backup。
- C 仍是 critical path，但预测侧逻辑已移交 P。

### Q&A 准备

- **C**：为什么 receding horizon？`r_safe` 如何确定？latency 如何实现？
- **P**：你的 Reactive baseline 和 CV 有什么本质区别？CTRV 在什么时候优于 CV？前车减速时 CV 让刹车偏早还是偏晚？
- **A**：quintic 换道的峰值横向加速度是多少？为什么场景集必须有 benign 样本？为什么不能用日志中的 ego 轨迹？
- **D**：false brake 如何定义？两条边界之间的面积意味着什么？边界是否与刹停距离吻合？

## 5. Milestones

| 周 | C | P | A | D | 里程碑 |
| --- | --- | --- | --- | --- | --- |
| W3–W4 | 接口冻结；skeleton ✅ | CV + `assess_risk` | `load_toy` + quintic `cut_in` | timeline + `is_benign` ✅ | **接口冻结 + 端到端** |
| W5–W6 | footprint/`r_safe`、latency、集成 | Reactive、CA、CTRV | benign/dangerous 集、`lead_brake`、`crossing` | `run_matrix`、基础指标 | L1 完成 |
| **W7** | 端到端 demo | ADE 曲线 + Reactive vs CV | 场景说明 | 初步 trade-off | **Proposal**：写死 §1.4 / §1.5；evidence = ADE 曲线 + 同一 cut-in 上 Reactive vs CV 的 timeline |
| W8–W10 | 调试 | Oracle、LR（可选） | 参数网格 | 四边界图、lead time、latency | L2 完成 |
| W11–W12 | 代码冻结 | 报告 | 报告 | 图表定稿 | 初稿 + 交叉审阅 |
| **W13–W14** | | | | | **Final**：20 min + 10 min Q&A |

## 6. 风险

整体翻车风险：**低～中**（工作量净增：+2 baseline、+1 predictor、场景更复杂、实验配置 1 → 4 组）。

| 风险 | 应对 |
| --- | --- |
| C 延期 | skeleton 已跑通；D 为 backup |
| Reactive 与 CV 结果重合 | §1.4 的定义 + `tests/test_risk.py::test_cv_sees_cut_in_before_reactive` 守护 |
| 误报率恒 0 | §1.5；场景集强制包含 benign 样本 |
| 场景代表性被质疑 | quintic + 横向加速度上限，报告被过滤的参数比例 |
| scope 失控 | 按 §1.3 从 L3 往下砍 |

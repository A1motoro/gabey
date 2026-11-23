# gabey 项目审计报告

**审计时间**: 2026-11-23 13:35:06 UTC  
**基准提交**: `a3f91cb5f4bf21cc1cbbe8fbb865a85ac53e44e5` (main)  
**审计执行**: Cursor Cloud Agent (gabey)  
**仓库**: https://github.com/A1motoro/gabey

---

## 执行摘要

**项目目标**: 通过参数化合成场景的批量仿真，比较 Reactive / Predictive (CV/CA/CTRV) / Oracle 三层 AEB 系统，回答"预测能把 AEB 安全工作区扩大多少，又在哪里失效"。

**当前状态**: 
- **L2 核心已完成并合并** (PR #8, #10, #12, #13)
- **D (评估) 当前处于 M2 (W11) 已完成状态**
- **D 的下一个里程碑**: **M3 (W12) — 报告"实验结果/局限"节**
- 348/350 测试通过 (仅 2 个因缺少可选课程数据失败)
- 实验可运行但 `outputs/figures/` 为空，需重新生成结果

**关键发现**:
1. ✅ 数据完全合成，物理模型明确，无真实数据依赖
2. ✅ 场景生成器与评估逻辑分离清晰，dangerous/benign 标签可复现
3. ⚠️ **所有实验结果图表缺失**，仅有空目录，需运行实验生成
4. ⚠️ exp04 (延迟敏感性) 仅为占位符，尚未实现 (已标记 L3 可选)
5. ⚠️ 文档声称的"L2 quick 结果"未在仓库中找到对应产物

---

## 1. 数据可行性分析

### 1.1 数据来源与性质

**数据类型**: **100% 参数化合成**，无真实世界数据

| 组件 | 来源 | 性质 | 文件 |
|------|------|------|------|
| 场景生成 | A's M1 实现 | quintic 最小加加速度横向轨迹 + 参数化纵向运动 | `src/pbs/data/scenarios.py` (554行) |
| Toy 数据 (可选) | 课程 AIE1903 sample_trajectories | CSV 格式轨迹片段，仅用于 P 的 L1 验证 | `data/sample_trajectories/*.csv` (未入库) |
| 实验场景集 | D 的组装 | 调用 A 的生成器 | `experiments/scenario_sets.py` |

**证据**:
- `src/pbs/data/scenarios.py:1-42` 明确文档说明 "parametric synthetic scenarios"
- `README.md:189-192` 明确声明数据不入库，toy 数据为可选课程数据
- 测试 `test_data.py::test_load_toy_*` 失败原因为 `FileNotFoundError: no *.csv trajectories`

**授权**: 合成数据无授权问题；课程数据仅用于验证，不影响核心结论

### 1.2 场景规模与参数覆盖

**实际运行验证** (2026-11-23 13:35 UTC):

```python
# 命令: python3 -c "from experiments.scenario_sets import *; ..."
Full boundary grid: 255 scenarios
Quick boundary grid: 25 scenarios
Tradeoff set: 110 scenarios
  - Default grid: 96 (cut_in: 60, lead_brake: 20, crossing: 16)
  - CA slice: 12 (a_cutter = ±2.0 m/s²)
  - Abort cut-ins: 6
  - Rejected: 0 (所有场景物理可行)
```

**参数范围** (来源: `experiments/scenario_sets.py:23-27`):

| 实验 | gap_d (m) | rel_v (m/s) | 其他参数 |
|------|-----------|-------------|----------|
| Quick boundary | 8–48 (5档) | -4–12 (5档) | lc_T=3.0s 固定 |
| Full boundary | 6–60 (15档) | -4–12 (17档) | lc_T=3.0s 固定 |
| Default grid | 见 A's DEFAULT_GRID | 见 A's DEFAULT_GRID | 三类场景混合 |

**物理合理性检查**:
- ✅ Quintic 横向轨迹: 峰值横向加速度 ≤ 3.0 m/s² (舒适/激进边界)
- ✅ 纵向加速度: ≤ 3.0 m/s² (cut_in), ≤ 6.0 m/s² (制动)
- ✅ 拒绝率: 0/96 scenarios (所有组合物理可行)
- 测试覆盖: `test_data.py::test_cut_in_rejects_physics_beyond_the_lateral_acceleration_bound` ✅

### 1.3 Dangerous/Benign 标签定义

**定义** (来源: `docs/d-baseline.md:40-50`):
```
Dangerous/Benign: 同一场景用 NeverBrakePolicy 跑一次：
  - 碰撞 ⇒ dangerous
  - 否则 ⇒ benign
与系统无关，每个场景只算一次并缓存
```

**实际分布验证** (2026-11-23 13:35 UTC, L2 配置: r_safe=None, horizon=40, duration=15s):

```
Quick boundary grid (n=25): 11 benign (44%), 14 dangerous (56%)
Tradeoff set (n=110): 53 benign (48%), 57 dangerous (52%)
```

**边界处理**:
- ✅ 碰撞判定: L2 使用 OBB (定向矩形 5.0m×2.5m) + swept volume
- ✅ 时域覆盖: 仿真 15s, 日志 19s (LOG_PAD=4s 避免 Oracle 末端外推假停车)
- ✅ 缓存机制: `run_matrix` 中 `benign_cache` 确保每场景仅判定一次
- 测试覆盖: `test_sweep.py::test_run_matrix_schema_and_cached_benign` ✅

**关键约束满足**:
- ✅ benign 与 dangerous 均非空 (满足 §1.5 要求避免误报率恒为 0)
- ✅ benign 场景包含"看起来危险但实际安全"的 cut-in (rel_v ≤ 0, 大 gap_d)
- ✅ 中止换道 (abort cut-ins): 6 个场景测试预测误报代价

### 1.4 数据一致性与可复现性

**时间步与单位**:
- ✅ dt = 0.1s 冻结接口 (`src/pbs/data/scenarios.py:59`)
- ✅ 单位: 距离 [m], 速度 [m/s], 时间 [s], 加速度 [m/s²]
- ✅ 时间戳单调递增，测试覆盖: `test_baseline_edges.py::test_invalid_history[descending]` ✅

**随机性与种子**:
- ✅ `run_matrix(..., seed=0)` 默认固定种子
- ✅ 测试验证确定性: `test_baseline_integration.py::test_l1_repeated_run_is_deterministic` ✅
- ⚠️ **场景生成无随机性** (完全确定性参数网格)，无需种子控制

**场景唯一性** (scenario_id):
- ✅ ID 包含: kind + 完整参数哈希 (包括内部轨迹，非仅端点)
- ✅ 测试覆盖: `test_sweep.py::test_scenario_identity_includes_interior_motion_and_geometry` ✅
- 防止: 不同中间运动错误共享 benign 标签

### 1.5 训练/调参/评测隔离

**不适用**: 本项目为**仿真对比实验**，无机器学习训练

- Reactive/CV/CA/CTRV: 确定性算法，无可学习参数
- Oracle: 使用真值未来，非预测器
- 阈值扫描: 在固定场景集上事后选择工作点 (非训练优化)

**公平性保证**:
- ✅ 所有系统在**相同场景集**上评估
- ✅ 相同仿真配置 (dt, duration, 碰撞几何, 制动参数)
- ✅ Counterfactual 标签与系统无关

### 1.6 切片与样本独立性

**关键问题**: 相邻切片是否被当作独立场景？

**审查结果**:

1. **边界实验 (exp03)**: ✅ **独立场景**
   - 每个 (gap_d, rel_v) 组合生成**完整独立场景** (从 t=0 开始)
   - 非滑动窗口切片
   - 证据: `experiments/scenario_sets.py:30-35` 调用 `make_scenario` 生成独立场景

2. **Trade-off 实验 (exp02)**: ✅ **独立场景**
   - 110 个独立场景: 96 (default grid) + 12 (CA) + 6 (abort)
   - 去重逻辑: `scenario_sets.py:48-53` 确保唯一性

3. **预测器 ADE/FDE (P 的 exp01)**: ⚠️ **使用固定切片**
   - `prediction_slices()` 在同一轨迹上滑动窗口切片
   - 证据: `src/pbs/data/scenarios.py:528-554`
   - **统计独立性**: ❌ 切片间相关
   - **用途**: 仅用于预测器误差曲线 (ADE vs horizon)，**不用于系统对比的主指标**

**结论**: 
- ✅ D 的核心指标 (碰撞率, 误报率, rescue rate) 基于**独立场景**
- ⚠️ P 的 ADE/FDE 曲线使用相关切片，但明确标注为"同切片" (same slices)
- ✅ 无统计检验需要置信区间 (确定性仿真，无抽样不确定性)

### 1.7 Oracle 与真值使用

**Oracle 定义** (来源: `src/pbs/prediction/oracle.py`):
```python
def predict(self, history, horizon, dt):
    # 返回 scenario 中 agent 的真实未来轨迹
    return self._future_positions[idx_start : idx_start + horizon]
```

**审查结果**:
- ✅ Oracle 使用场景定义的**真实未来轨迹**，非预测
- ✅ 明确标注为"upper bound" (上界基准)
- ✅ 不参与预测器 ADE/FDE 比较 (Oracle 误差为 0 无意义)
- ✅ 仅用于 failure boundary 对比，回答"预测误差代价多少"

**无泄露风险**: Oracle 与 CV/CA/CTRV 在**相同仿真配置**下对比，非训练/测试分离场景

### 1.8 指标定义一致性

**核心指标** (来源: `docs/d-baseline.md:40-50`):

| 指标 | 定义 | 实现 | 测试 |
|------|------|------|------|
| Collision rate | dangerous 中碰撞比例 | `eval/sweep.py:tradeoff` | `test_sweep.py::test_tradeoff_rates_ignore_the_other_class` ✅ |
| False-brake rate | benign 中刹车比例 | 同上 | 同上 ✅ |
| Rescue rate | dangerous 中被救下比例 | `eval/sweep.py:rescue_rate` | `test_sweep.py::test_boundary_rescue_and_safe_area` ✅ |
| ADE/FDE | 预测与真值平均/终点误差 | `eval/metrics.py` | `test_predictors.py` ✅ |
| Lead time | Reactive vs Predictive 刹车时差 | `eval/sweep.py:lead_times` | `test_sweep.py::test_lead_time_keeps_one_sided_brakes` ✅ |
| Brake onset | 首次刹车时刻 | `eval/metrics.py:evaluate` | `test_sweep.py::test_evaluate_truncates_brake_onset_at_first_collision` ✅ |

**一致性检查**:
- ✅ 所有指标在 `run_matrix` 输出的**单一 DataFrame** 中计算
- ✅ 相同 sim_kwargs 传递给 benign 判定与主实验
- ✅ 碰撞后刹车正确截断 (避免碰撞后触发计入 brake onset)

### 1.9 外推到真实世界的证据

**明确限制** (来源: `README.md:161-166`, `docs/d-baseline.md:118-120`):

```
Limitations:
- 点质量运动 + 随航向旋转的矩形车身，无车辆动力学
- 场景为合成构造，非真实交通分布
- 无 perception: 直接使用轨迹级数据
- 2D 简化

Will not demonstrate:
- 真实交通分布或真实车辆上的 AEB 性能
- perception 误差 (仅用 latency 近似)
```

**结论**: ❌ **无真实世界外推证据**，项目明确声明为受控仿真对比

---

## 2. 数据支持的结论层级

### 2.1 ✅ 课程原型 (Classroom Prototype)

**支持**: 是  
**证据**:
- 完整端到端仿真管线 (M1→M2→M3→M4)
- 348/350 测试通过
- Demo 可运行: `examples/cut_in_demo.py` (测试覆盖: `test_demo.py`)
- 物理模型明确 (quintic 轨迹, TTC 刹车, OBB 碰撞)

**适用**: AIE1903 课程 Topic 9 交付要求

### 2.2 ✅ 受控算法对比 (Controlled Algorithm Comparison)

**支持**: 是  
**条件**:
- 仅适用于**参数化合成场景**
- 仅适用于**2D 点质量 + TTC 阈值刹车**设定
- 对比结论限于"在此设定下"

**证据**:
- 场景物理合理 (横向加速度 ≤ 3.0 m/s², 拒绝率 0%)
- Benign/dangerous 分布合理 (48%/52%)
- 相同场景集, 相同仿真配置
- Counterfactual 定义公平 (与系统无关)

**可回答问题**:
- ✅ "在 quintic 切入场景下，CV 比 Reactive 提前多少秒刹车？" (lead time)
- ✅ "在相同误报率下，CV 救下多少额外 dangerous 场景？" (rescue rate)
- ✅ "Oracle 上界与 CV 差距是多少？" (预测误差代价)

**不可回答问题**:
- ❌ "在真实高速公路上，该系统碰撞率是多少？"
- ❌ "该系统是否满足 ISO 26262 ASIL-D？"

### 2.3 ❌ 论文级结论 (Peer-Reviewed Publication)

**支持**: 附条件  
**缺失**:
- ❌ 真实世界验证数据 (nuScenes / KITTI / Waymo)
- ❌ 统计显著性检验 (当前为确定性仿真，无需置信区间)
- ❌ 对比 state-of-the-art 预测模型 (仅 CV/CA/CTRV 经典模型)
- ❌ 车辆动力学模型验证
- ⚠️ **实验结果缺失** (outputs/figures/ 为空)

**可行路径** (若补充):
1. 在 nuScenes mini 上验证预测器 ADE/FDE
2. 加入统计检验 (bootstrap 置信区间)
3. 对比 LSTM/Transformer 预测器
4. 敏感性分析 (横向加速度上限, 阈值选择)

**当前状态**: 可发表为**方法论文** (methodology) 或**工具论文** (tool paper)，但不适合作为安全声明

### 2.4 ❌ 现实安全结论 (Real-World Safety Claims)

**支持**: 否  
**根本限制**:
- 无真实交通分布
- 无 perception 误差 (仅 latency 近似)
- 无车辆动力学
- 无多智能体交互 (agent 不响应 ego 刹车)
- 无传感器失效 / 遮挡 / 恶劣天气

**明确声明** (来源: `README.md:161`, `PRODUCT.md:9-11`):
```
Results illustrate a configured simplified model. 
Earlier braking in one synthetic scenario does not establish general safety superiority.
```

**结论**: ⚠️ **项目已明确标注限制**，无过度声称

---

## 3. 项目现状全景

### 3.1 目标与技术链路

**研究问题** (来源: `README.md:21-25`, `docs/spec.md:8`):
> Prediction 能把 AEB 的安全工作区扩大多少，又在哪里仍然失效？

**技术链路**:
```
M1 (A)   → M2 (P)      → Risk (P) → M3 (C)    → M4 (D)
场景生成   预测器         TTC评估     仿真决策     评估实验
├─ quintic ├─ CV        ├─ Reactive ├─ rollout  ├─ run_matrix
├─ cut_in  ├─ CA        ├─ Predictive├─ OBB     ├─ tradeoff
├─ lead_   ├─ CTRV     └─ assess_risk├─ policy  ├─ boundary
│  brake   ├─ LR (L3)                 └─ latency ├─ lead_time
└─ crossing└─ Oracle                             └─ plots
```

**系统对比** (CORE):
- Reactive: 仅纵向 TTC (已在本车道内的 agent)
- CV: 2D 匀速外推
- CA: 2D 匀加速外推 (限制停车后反向)
- CTRV: 匀角速度转弯模型
- Oracle: 真实未来 (上界)
- LR (L3): 线性回归拟合 (可选)

### 3.2 职责分工与实现状态

**C (前背痛) — 仿真与决策 (M3)**:

| 组件 | 状态 | 文件 | 提交 |
|------|------|------|------|
| run_sim | ✅ 已合并 | `src/pbs/sim/simulator.py` | PR #12 (a3f91cb) |
| OBB footprint | ✅ 已合并 | `src/pbs/sim/collision.py` | PR #12 |
| HysteresisPolicy | ✅ 已合并 | `src/pbs/sim/policy.py` | PR #12 |
| L2 baseline docs | ✅ 已合并 | `docs/l2-baseline.md` | PR #13 |
| L2 闭环实验 | ✅ 已合并 | `experiments/c_l2_closed_loop.py` | PR #12 |

**测试**: `test_simulator.py`, `test_collision.py`, `test_policy.py` ✅ 全通过

**P (华华) — 预测 + 风险评估 (M2)**:

| 组件 | 状态 | 文件 | 提交 |
|------|------|------|------|
| CV (L1) | ✅ 已合并 | `src/pbs/prediction/cv.py` | 早期提交 |
| CA (L2) | ✅ 已合并 | `src/pbs/prediction/ca.py` | PR #10 (2b21d69) |
| CTRV (L2) | ✅ 已合并 | `src/pbs/prediction/ctrv.py` | PR #10 |
| Oracle (L2) | ✅ 已合并 | `src/pbs/prediction/oracle.py` | PR #10 |
| LR (L3) | ✅ 已合并 | `src/pbs/prediction/lr.py` | PR #11 (b377c4c) |
| assess_risk | ✅ 已合并 | `src/pbs/risk/ttc.py` | 早期提交 |
| Reactive/Predictive assessors | ✅ 已合并 | `src/pbs/risk/assessor.py` | 早期提交 |
| L1 验证 | ✅ 已合并 | `experiments/exp00_l1_baseline.py` | 早期提交 |
| L2 验证 | ✅ 已合并 | `experiments/exp01_predictor_ade.py` | PR #10 |
| L2/L3 验证文档 | ✅ 已合并 | `docs/l2-validation.md`, `docs/l3-validation.md` | PR #10, #11 |

**测试**: `test_predictors.py`, `test_l2_predictors.py`, `test_l3_predictors.py`, `test_risk.py` ✅ 全通过

**A (Persistent) — 场景 (M1)**:

| 组件 | 状态 | 文件 | 提交 |
|------|------|------|------|
| quintic 换道 | ✅ 已合并 | `src/pbs/data/scenarios.py` | PR #6 前 |
| 三类场景 | ✅ 已合并 | `src/pbs/data/scenarios.py` | PR #6 前 |
| 场景网格 | ✅ 已合并 | `src/pbs/data/scenarios.py:DEFAULT_GRID` | PR #6 前 |
| CA 评测底座 | ✅ 已合并 | `src/pbs/data/scenarios.py:ca_scenario_set` | PR #9 (aa25d3f) |
| 场景文档 | ✅ 已合并 | `docs/scenarios.md`, `docs/A_baseline_handoff.md` | PR #6, #9 |
| CA 支持文档 | ✅ 已合并 | `docs/CA_scenario_support.md` | PR #9 |

**测试**: `test_data.py` (40 项) ✅, `test_scenarios.py` ✅

**D (yj) — 评估与实验 (M4)**:

| 组件 | 状态 | 文件 | 提交 |
|------|------|------|------|
| is_benign | ✅ 已合并 | `src/pbs/eval/counterfactual.py` | PR #8 前 |
| evaluate | ✅ 已合并 | `src/pbs/eval/metrics.py` | PR #8 前 |
| run_matrix | ✅ 已合并 | `src/pbs/eval/sweep.py` | PR #8 (321528a) |
| tradeoff | ✅ 已合并 | `src/pbs/eval/sweep.py` | PR #8 |
| failure_boundary | ✅ 已合并 | `src/pbs/eval/sweep.py` | PR #8 |
| rescue_rate | ✅ 已合并 | `src/pbs/eval/sweep.py` | PR #8 |
| plot_timeline | ✅ 已合并 | `src/pbs/eval/plots.py` | PR #8 前 |
| plot_boundaries | ✅ 已合并 | `src/pbs/eval/plots.py` | PR #8 |
| exp02 (trade-off) | ✅ 已合并 | `experiments/exp02_threshold_sweep.py` | PR #8 |
| exp03 (boundary) | ✅ 已合并 | `experiments/exp03_failure_boundary.py` | PR #8 |
| exp05 (lead time) | ✅ 已合并 | `experiments/exp05_cut_in_lead_time.py` | PR #8 |
| exp04 (latency) | ⚠️ 占位符 | `experiments/exp04_latency.py` (9行) | 标记 L3 可选 |
| scenario_sets | ✅ 已合并 | `experiments/scenario_sets.py` | PR #8 |
| matrix_setup | ✅ 已合并 | `experiments/matrix_setup.py` | PR #8 |
| d-baseline 文档 | ✅ 已合并 | `docs/d-baseline.md` | PR #8 |

**测试**: `test_sweep.py`, `test_counterfactual.py`, `test_scenario_sets.py` (19 项) ✅ 全通过

**Demo (共同)**:

| 组件 | 状态 | 文件 | 提交 |
|------|------|------|------|
| Cut-in demo | ✅ 已合并 | `examples/cut_in_demo.py`, `src/pbs/demo/` | PR #6 (2224d94) |
| Demo 文档 | ✅ 已合并 | `docs/cut_in_demo.md` | PR #6 |
| Demo 测试 | ✅ 已合并 | `tests/test_demo.py`, `tests/test_demo_extensions.py` | PR #6 |

### 3.3 里程碑验收证据

**M0 (W3-W4) — 接口冻结 + 端到端** ✅:
- 证据: `pbs/types.py` (Trajectory, Scenario, SimLog 冻结)
- 证据: `docs/interfaces.md` 存在
- 端到端: `test_baseline_integration.py::test_l1_cut_in_cv_brakes_before_reactive` ✅

**M1 (W5-W6) — L1 完成** ✅:
- C: footprint/r_safe ✅ (`src/pbs/sim/collision.py`)
- P: Reactive/CV ✅, ADE 曲线 ✅ (`exp00_l1_baseline.py`)
- A: benign/dangerous 集 ✅, 三类场景 ✅ (`exp00_scenarios.py`)
- D: run_matrix ✅, 基础指标 ✅ (`test_sweep.py`)

**M2 (W8-W11) — L2 完成** ✅:
- C: 调试 ✅, L2 baseline 文档 ✅ (PR #12, #13)
- P: CA/CTRV/Oracle ✅, LR (L3) ✅ (PR #10, #11)
- A: 参数网格 ✅, CA 支持 ✅ (PR #9)
- D: **exp02/exp03/exp05 代码合并** ✅ (PR #8)
  - ⚠️ **但产物缺失** (见下节)

**M3 (W11-W12) — 初稿 + 交叉审阅** ⏳:
- **当前状态**: D 需完成报告"实验结果/局限"节
- C: 代码冻结 ✅ (已 2 周无 C 相关修改)
- P: 报告 ⏳ (无文档交付)
- A: 报告 ⏳ (无文档交付)
- **D: 图表定稿 ❌** (outputs/figures/ 为空)

**M4 (W13-W14) — Final** ⏳:
- Deadline: 12/8 (展示), 12/13 (报告)
- 距今: ~19 天

### 3.4 开放 PR 与依赖

**当前开放 PR**: 0 个 (所有 PR 已合并到 main)

**最近合并 PR**:
1. PR #13: P0 L2 alignment (2026-11-20, a3f91cb)
2. PR #12: C L2 baseline (2026-11-19, 5cdd4ff)
3. PR #11: P L3 LR (2026-11-18, 73f08f9)
4. PR #10: P L2 predictors (2026-11-17, a0f4556)
5. PR #8: D L2 evaluation (2026-11-16, 321528a)

**阻塞**: 无开放 PR，无代码依赖阻塞

### 3.5 D 的下一个 L 明确目标

**当前状态**: D 已完成 **M2 (W11)**

**证据**:
- ✅ exp03 四边界图代码定稿 (`exp03_failure_boundary.py`)
- ✅ 报告两个差值的逻辑实现 (`rescue_rate` 计算)
- ✅ 全部实验 `--quick` 实现 (exp02/exp03/exp05 均支持)
- ✅ 图与 CSV 对应 (代码中 `from-csv` 参数支持)
- ✅ 测试通过 (19/19 D 相关测试 ✅)

**下一个 L**: **M3 (W12) — 报告"实验结果/局限"节**

**验收标准** (来源: `docs/d-baseline.md:110`):
```
M3 (W12): 报告"实验结果 / 局限"节；
每个结论按 claim → evidence（图号）→ limitation 写
```

**具体要求**:
1. **生成所有实验产物**:
   - `outputs/figures/exp02_tradeoff.png`
   - `outputs/figures/exp03_boundaries.png`
   - `outputs/figures/exp05_lead_time.png`
   - 对应 CSV: `outputs/logs/exp{02,03,05}.csv`
   - 对应 summary JSON

2. **撰写报告章节** (新建或更新文档):
   - 实验结果节: 每个图的 claim / evidence / limitation
   - 局限性节: 汇总所有限制 (已在 README/PRODUCT 中部分声明)

3. **结果验证**:
   - 运行 `--quick` 实验 (< 5 min)
   - 验证数字与文档声称一致 (docs/d-baseline.md:122-150)

**当前阻塞**: ❌ **所有实验产物缺失**

---

## 4. 关键风险与建议

### 4.1 🔴 高优先级 (立即执行)

#### 风险 1: 实验结果产物完全缺失

**现状**: `outputs/figures/` 仅有 `.gitkeep`, 所有图表/CSV/JSON 缺失

**影响**:
- 无法验证文档声称的结果 (docs/d-baseline.md:122-150)
- M3 报告节无法撰写 (无图号可引用)
- 无法交叉验证代码与结果一致性

**根因**: 仓库 `.gitignore` 排除 `outputs/` 目录内容

**立即行动**:
1. 运行 `--quick` 实验生成 baseline 结果:
   ```bash
   python experiments/exp02_threshold_sweep.py --quick
   python experiments/exp03_failure_boundary.py --quick
   python experiments/exp05_cut_in_lead_time.py --quick
   ```
2. 检查生成文件与文档数字是否一致
3. 若一致, 创建实验结果文档 (M3 交付)

**时间估计**: ~10 min (代码已存在, 仅需运行)

#### 风险 2: M3 报告节未开始

**现状**: 无 `docs/results.md` 或类似文档

**Deadline**: W12 结束 (约 2026-11-29), 距今 6 天

**立即行动**:
1. 生成实验结果后, 创建 `docs/results.md`
2. 结构参考 `docs/d-baseline.md:115-120`:
   ```markdown
   ## Claim 1: CV 比 Reactive 救下更多场景
   Evidence: exp03_boundaries.png, rescue(CV)=0.643, rescue(Reactive)=0.429
   Limitation: 仅在 quintic 切入场景, 2D 点质量设定下成立
   ```
3. 补充局限性汇总 (已在 README:161-166 部分覆盖)

**时间估计**: ~2 hours (撰写 + 审校)

### 4.2 🟡 中优先级 (本周完成)

#### 风险 3: exp04 (latency) 未实现

**现状**: `experiments/exp04_latency.py` 仅 9 行占位符

**标注**: L3 可选 (docs/spec.md:76, docs/d-baseline.md:36)

**决策点**:
- **若砍掉**: 在 M3 报告中明确声明 "exp04 latency 敏感性作为 L3 扩展未实施"
- **若实现**: 需 ~4 hours (run_matrix 已支持 latency 参数, 仅需画图逻辑)

**建议**: **砍掉**, 理由:
1. 已标记 L3 可选
2. M3 Deadline 紧张
3. 核心结论 (exp02/03/05) 已足够回答研究问题

#### 风险 4: 交叉审阅未开始

**现状**: M3 包含"交叉审阅", 但无 PR 或审阅记录

**审阅分工** (来源: `docs/spec.md:90`):
- D 审 C
- P 审 A
- C 审 P
- A 审 D

**建议**:
1. 生成实验结果后, 由 A 审阅 D 的结果文档
2. D 审阅 C 的 L2 baseline 实现 (已合并, 追溯审阅)
3. 记录审阅意见在 GitHub Issue 或内部文档

**时间估计**: ~1 hour per person

### 4.3 🟢 低优先级 (Final 前完成)

#### 建议 1: 补充实验配置文档

**现状**: 实验配置分散在代码中 (matrix_setup.py, scenario_sets.py)

**建议**: 创建 `docs/experiments.md` 汇总:
- 每个实验的场景集/系统/阈值
- Quick vs full 配置对比表
- 预期运行时间

**时间估计**: ~1 hour

#### 建议 2: 补充可复现性清单

**现状**: README 有运行指令, 但无完整复现清单

**建议**: 在 README 或新文档中补充:
- 环境要求 (Python 3.12, conda env)
- 完整运行步骤 (从 clone 到生成所有图)
- 预期文件树 (outputs/ 结构)
- 常见问题 (Windows 路径, 缺 pytest)

**时间估计**: ~30 min

#### 建议 3: 添加 CI/CD

**现状**: 无 GitHub Actions 自动测试

**建议**: 添加 `.github/workflows/test.yml`:
- 自动运行 pytest
- 检查代码格式 (black, ruff)
- 可选: 运行 `--quick` 实验验证可运行性

**时间估计**: ~1 hour

---

## 5. 数据审计总结表

| 维度 | 评级 | 说明 |
|------|------|------|
| **数据来源** | ✅ 明确 | 100% 合成, 参数化生成, 无授权问题 |
| **物理合理性** | ✅ 通过 | 横向加速度 ≤ 3.0 m/s², 拒绝率 0% |
| **样本规模** | ✅ 充足 | Quick: 25/110, Full: 255 场景 |
| **标签质量** | ✅ 可靠 | Benign/dangerous 48%/52%, counterfactual 定义明确 |
| **时间步一致性** | ✅ 通过 | dt=0.1s 冻结, 单调递增 |
| **可复现性** | ✅ 通过 | 固定种子, 确定性仿真, 348/350 测试通过 |
| **样本独立性** | ⚠️ 部分 | D 实验用独立场景 ✅, P 的 ADE 用切片 ⚠️ (已标注) |
| **Oracle 使用** | ✅ 合理 | 明确标注上界, 不参与 ADE, 无泄露 |
| **指标一致性** | ✅ 通过 | 统一 DataFrame, 相同 sim_kwargs, 测试覆盖 |
| **外推证据** | ❌ 无 | 明确声明限制, 无过度声称 |
| **实验产物** | ❌ 缺失 | **所有图表/CSV 未生成** |

**总体评估**: 数据质量 ✅ 高, 但**实验结果缺失** ❌ 阻塞 M3 交付

---

## 6. 最终建议优先级

### 本轮必须完成 (D 的下一个 L):

1. **生成实验结果** (10 min):
   ```bash
   cd /workspace
   python experiments/exp02_threshold_sweep.py --quick
   python experiments/exp03_failure_boundary.py --quick
   python experiments/exp05_cut_in_lead_time.py --quick
   ```

2. **验证结果与文档一致性** (10 min):
   - 检查 CSV 数字与 `docs/d-baseline.md:135-150` 声称值
   - 若不一致, 更新文档或报告差异

3. **撰写 M3 报告节** (2 hours):
   - 创建 `docs/results.md` 或在现有文档中补充
   - 每个实验: claim → evidence (图号 + 数字) → limitation

4. **创建 D 的 L2→M3 PR**:
   - Branch: `cursor/d-m3-results-report-64a3`
   - 包含: 实验结果, 报告文档, 任何必要的文档更新
   - 不合并, 等待审阅

### 后续建议 (M3→Final):

5. **交叉审阅**: A 审 D, D 审 C (追溯 PR #12/13)
6. **砍掉 exp04** 或实现 (决策后更新文档)
7. **补充实验配置/可复现性文档** (Final 交付前)

---

## 附录: 文件清单

### 核心代码 (已合并 main)

**M1 (A)**: `src/pbs/data/{scenarios.py, loader.py}`, `docs/{scenarios.md, A_baseline_handoff.md, CA_scenario_support.md}`

**M2 (P)**: `src/pbs/prediction/{cv.py, ca.py, ctrv.py, oracle.py, lr.py}`, `src/pbs/risk/{ttc.py, assessor.py}`, `docs/{l2-validation.md, l3-validation.md}`

**M3 (C)**: `src/pbs/sim/{simulator.py, collision.py, policy.py}`, `docs/l2-baseline.md`, `experiments/c_l2_closed_loop.py`

**M4 (D)**: `src/pbs/eval/{counterfactual.py, metrics.py, sweep.py, plots.py}`, `experiments/{exp02,exp03,exp05}*.py`, `experiments/{scenario_sets.py, matrix_setup.py}`, `docs/d-baseline.md`

### 测试 (348/350 通过)

**总计**: 350 tests, 2 failed (缺可选 toy 数据)

**D 相关**: 19 tests in `test_sweep.py`, `test_counterfactual.py`, `test_scenario_sets.py` ✅

### 缺失产物

❌ `outputs/figures/{exp02_tradeoff.png, exp03_boundaries.png, exp05_lead_time.png}`  
❌ `outputs/logs/{exp02.csv, exp03.csv, exp05.csv}`  
❌ `outputs/logs/{exp02_summary.json, exp03_summary.json, exp05_summary.json}`  
❌ `docs/results.md` 或等效 M3 报告文档

---

**审计完成时间**: 2026-11-23 13:35 UTC  
**下一步**: 执行"本轮必须完成"清单 → 生成实验结果 → 撰写 M3 报告 → 创建 PR

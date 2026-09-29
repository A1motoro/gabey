# gabey — Predictive Braking Simulator

> 给定周围车辆的历史轨迹，预测它们的未来运动，估计 **time-to-collision (TTC)** 并决定是否刹车；通过参数化场景的批量仿真，找出该 AEB 系统的 **failure boundary**。

## 课程与团队

- **课程**：AIE1903 Exploration on Basic Autonomous Driving · Topic 9 — Predictive Braking Simulator (★★★)
- **团队**（4 人）：

| 代号 | 成员 | 角色 | 模块 |
| --- | --- | --- | --- |
| C | _TBD_ | 统筹 + 仿真与决策 | M3 `pbs.sim` |
| P | _TBD_ | 预测 + 风险评估 | M2 `pbs.prediction`、`pbs.risk` |
| A | _TBD_ | 场景 | M1 `pbs.data` |
| D | _TBD_ | 评估与实验 | M4 `pbs.eval`、`pbs.systems` |

完整方案见 [docs/spec.md](docs/spec.md)。

## Motivation

传统 reactive AEB 只对已进入本车道的目标、按纵向 range / range-rate 计算 TTC；对**正在切入**的车辆，它要等切入完成才会反应。Prediction 可以提前外推横向运动，但预测误差又会带来刹车过晚或误报。本项目要回答：

> **Prediction 能把 AEB 的安全工作区扩大多少，又在哪里仍然失效？**

我们比较三层系统——**Reactive**（无预测）、**Predictive**（CV / CA / CTRV / LR）、**Oracle**（真实未来）——并在 (gap_d, rel_v) 平面上画出各自的 failure boundary。

## 快速开始

```bash
conda activate aie1903_ad          # 课程环境
pip install -r requirements.txt
pip install -e .                   # 使 `import pbs` 可用
pytest                             # smoke test
python experiments/exp01_predictor_ade.py
```

## 目录结构

```
src/pbs/
  types.py        Trajectory / Scenario / SimLog（W4 冻结）
  data/           M1  load_toy, make_scenario (quintic cut-in)
  prediction/     M2  CV / CA / CTRV / LR / Oracle predictors
  risk/           TTC: assess_risk, Reactive / Predictive assessors
  sim/            M3  run_sim, ego rollout, footprint, policy, latency
  eval/           M4  counterfactual, metrics, experiment matrix, plots
  systems.py      被比较的 AEB 系统注册表
experiments/      一个实验一个脚本，产出 outputs/figures/
tests/            pytest
docs/             interfaces / ai_disclosure / contributions
data/             不入库（见「数据说明」）
```

数据流：`M1 scenario → M2 prediction → risk (TTC) → M3 sim/decision → M4 evaluation`

## 核心概念

- **TTC (time-to-collision)**：在预测时域内，ego 与 agent 距离首次小于 `r_safe` 的时间；无冲突为 `inf`。
- **Reactive vs Predictive**：Reactive 只看本车道内目标的纵向接近；Predictive 外推所有目标的 2D（含横向）运动。两者之差 = 提前察觉切入的 **lead time**。
- **Counterfactual false brake**：同一场景下若"永不刹车"也不会碰撞，则该场景 benign，其中任何刹车都是误报。
- **Receding horizon**：每个仿真步都重新预测，不复用上一步结果。
- **Log replay**：agent 按日志轨迹回放，不响应 ego；ego 自主仿真，因此能响应刹车决策。
- **ADE / FDE**：预测轨迹与真值的平均 / 终点位移误差。

## 实验复现

| 脚本 | 内容 | 报告图 |
| --- | --- | --- |
| `experiments/exp01_predictor_ade.py` | CV / CA / CTRV / LR 的 ADE/FDE vs horizon | _TBD_ |
| `experiments/exp02_threshold_sweep.py` | 各系统碰撞率 vs 误报率 trade-off | _TBD_ |
| `experiments/exp03_failure_boundary.py` | **Reactive / CV / CA / Oracle 四边界叠加图** | _TBD_ |
| `experiments/exp04_latency.py` | 感知延迟 τ ∈ [0, 300] ms 敏感性 | _TBD_ |
| `experiments/exp05_cut_in_lead_time.py` | Reactive vs Predictive 的 cut-in lead time | _TBD_ |

## 结果预览

_待补充：四边界叠加图、trade-off 曲线、Reactive vs CV timeline。_

## Limitations

- 点质量 + 圆形 footprint，无车辆动力学；仅纵向控制，无 steering。
- 场景为合成构造（quintic 换道 + 横向加速度上限），非真实交通分布。
- 无 perception：直接使用轨迹级数据。
- 2D 简化。

## 分工与贡献

见 [docs/contributions.md](docs/contributions.md)；接口约定见 [docs/interfaces.md](docs/interfaces.md)。

## AI 工具使用声明

按课程 AI Tool Usage Policy 记录于 [docs/ai_disclosure.md](docs/ai_disclosure.md)。

## 数据说明

`data/` 不入库。

- **Toy 数据**：从课程仓库 `AIE1903_AD/data/sample_trajectories/` 复制到本仓库 `data/sample_trajectories/`。
- **nuScenes mini**（可选，L3）：自行下载；所有结论须在合成场景上独立成立。

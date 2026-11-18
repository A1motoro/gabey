# gabey — Predictive Braking Simulator

> 给定周围车辆的历史轨迹，预测它们的未来运动，估计 **time-to-collision (TTC)** 并决定是否刹车；通过参数化场景的批量仿真，找出该 AEB 系统的 **failure boundary**。

## 课程与团队

- **课程**：AIE1903 Exploration on Basic Autonomous Driving · Topic 9 — Predictive Braking Simulator (★★★)
- **团队**（4 人）：

| 代号 | 成员 | 角色 | 模块 |
| --- | --- | --- | --- |
| C | 前背痛 | 仿真与决策 | M3 `pbs.sim` |
| P | 华华 | 预测 + 风险评估 | M2 `pbs.prediction`、`pbs.risk` |
| A | Persistent | 场景 | M1 `pbs.data` |
| D | yj | 评估与实验 + 统筹 | M4 `pbs.eval`、`pbs.systems` |

完整方案见 [docs/spec.md](docs/spec.md)（v3：实现从 W7 起步，排期见 §5，具体形式见 §7，最终交付物见 §8）。

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
python experiments/exp00_l1_baseline.py  # P 的 L1 验证（CV / TTC / Reactive）
python experiments/exp00_scenarios.py    # A 的 M1 场景基线（96 个场景 + no-braking 判定）
python experiments/exp02_threshold_sweep.py --quick   # D 的 L2：碰撞率–误报率
python experiments/exp03_failure_boundary.py --quick  # D 的 L2：四条 failure boundary
```

### P 的 L1 基线验证

L1 已实现二维 CV、首次冲突 TTC、Reactive 与 Predictive assessor。P 的独立验证入口：

```bash
conda activate aie1903_ad
python -m pytest -o cache_dir=outputs/logs/l1-baseline/.pytest_cache
python experiments/exp00_l1_baseline.py
```

Windows 未激活环境时，可使用 `conda run --no-capture-output -n aie1903_ad python ...`。
直接调用环境内的 `python.exe` 可能缺少原生数值库所需的环境设置。

验证脚本用独立的解析轨迹样例生成 CV ADE/FDE 曲线和 Reactive/CV cut-in timeline，
产出位于 `outputs/figures/l1-baseline/`。样例用于验证 P 的方法，不替代 A 的场景模块。
D 的 L2 实验矩阵见 [docs/d-baseline.md](docs/d-baseline.md) §9：`exp02`、`exp03`、`exp05` 已支持 `--quick`。
CA、CTRV、Oracle 已接入；LR 与 `exp04` 仍是 L3。场景由 A 的 `make_scenario` / `scenario_set` 生成，exp02 另加 6 个中止换道。
测试结果和复现说明见 [docs/l1-validation.md](docs/l1-validation.md)。

## 目录结构

```
src/pbs/
  types.py        Trajectory / Scenario / SimLog（W4 冻结）
  data/           M1  load_toy, make_scenario（三类参数化场景 + quintic 换道）
  prediction/     M2  CV / CA / CTRV / LR / Oracle predictors
  risk/           TTC: assess_risk, Reactive / Predictive assessors
  sim/            M3  run_sim, ego rollout, footprint, policy, latency
  eval/           M4  counterfactual, metrics, experiment matrix, plots
  systems.py      被比较的 AEB 系统注册表
  demo.py         单场景多系统对比动画（v3 新增，W9–W10）
experiments/      一个实验一个脚本，产出 outputs/figures/
tests/            pytest
docs/             interfaces / scenarios / d-baseline / ai_disclosure / contributions
data/             不入库（见「数据说明」）
```

数据流：`M1 scenario → M2 prediction → risk (TTC) → M3 sim/decision → M4 evaluation`

## 2.5D Cut-in 演示

```bash
python examples/cut_in_demo.py
```

在本地浏览器同步对比 Reactive 与 Predictive · CV，支持播放、逐帧、参数调整及结果摘要。
原版入口 `/`；投屏增强版 `/presentation` 增加车间虚线/TTC、关键事件跳转和自动暂停。
无需真实数据或前端构建工具。运行、测试、演示步骤以及真实数据/噪声扩展接口见
[Demo 使用说明](docs/cut_in_demo.md)。

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
| `experiments/exp00_l1_baseline.py` | P 的 L1：CV ADE/FDE + Reactive/CV cut-in timeline | `l1-baseline/cv_ade_fde.png`、`l1-baseline/cut_in_timeline.png` |
| `experiments/exp00_scenarios.py` | **M1 场景基线**：三类场景的几何、quintic 横向加速度过滤、96 场景 danger map | `outputs/figures/exp00_*.png` |
| `experiments/exp01_predictor_ade.py` | CV / CA / CTRV / LR 的 ADE/FDE vs horizon | _TBD_ |
| `experiments/exp02_threshold_sweep.py` | 各系统碰撞率 vs 误报率 trade-off（`--quick`） | `exp02_tradeoff.png` |
| `experiments/exp03_failure_boundary.py` | **Reactive / CV / CA / Oracle 四边界叠加图**（`--quick`） | `exp03_boundaries.png` |
| `experiments/exp04_latency.py` | 感知延迟 τ ∈ [0, 300] ms 敏感性 | _TBD_（L3） |
| `experiments/exp05_cut_in_lead_time.py` | Reactive vs CV / CTRV 的 cut-in lead time（`--quick`） | `exp05_lead_time.png` |

A 的 M1 同时是 CA 的评测底座：`ca_scenario_set()`（12 个带纵向加速度的切入）、`Trajectory.accel` 真值、`prediction_slices()` 固定切片。用法与坑见 [docs/CA_scenario_support.md](docs/CA_scenario_support.md)。

## 结果预览

P 的 L1 固定合成样例：Reactive 在 2.0 s 开始刹车，CV 在 1.7 s 开始刹车，提前 0.3 s；
两者均未碰撞。该样例用于方法验证，不能据此推断所有场景的安全收益。
曲线、参数与测试记录见 [L1 验证报告](docs/l1-validation.md)。
A 的 M1 场景基线：运行 `python experiments/exp00_scenarios.py` 后，`outputs/figures/` 下得到三张图：

| 文件 | 内容 |
| --- | --- |
| `exp00_scenario_geometry.png` | 三类冲突各一个 dangerous 样本（含不刹车时的接触点） |
| `exp00_lateral_accel.png` | quintic 的 y/ẏ/ÿ + 横向加速度过滤（最短换道时长 2.60 s） |
| `exp00_danger_map.png` | 96 个场景的 no-braking danger map（四边界图的底板） |

四边界、trade-off 和 lead time 的 `--quick` 结果见 [D 的评估说明](docs/d-baseline.md) §9。

## Limitations

- 点质量 + 圆形 footprint，无车辆动力学；仅纵向控制，无 steering。
- 场景为合成构造（quintic 换道 + 横向加速度上限），非真实交通分布。
- 无 perception：直接使用轨迹级数据。
- 2D 简化。

## 交付与日程

| Deadline | 日期 | 交付物 |
| --- | --- | --- |
| DL1 | 10/27（W7 二） | Proposal 展示 10 + 5 min |
| DL2 | 11/1（W7 日） | Proposal 文档 4–6 页 |
| DL3 | 12/8（W13 二） | Final 展示 + live demo 10 + 10 min |
| DL4 | 12/13（W13 日） | Code package（`gabey-v1.0.zip`）+ 报告 8–12 页 |

内部里程碑：M0 端到端（W7）→ M1 = L1（W9）→ M2 = L2 + 功能冻结（W11）→ M3 报告初稿（W12）。

## 分工与贡献

见 [docs/contributions.md](docs/contributions.md)；接口约定见 [docs/interfaces.md](docs/interfaces.md)；D 的评估设计见 [docs/d-baseline.md](docs/d-baseline.md)。

## AI 工具使用声明

按课程 AI Tool Usage Policy，每人一份：[C](docs/ai_disclosure/C.md) · [P](docs/ai_disclosure/P.md) · [A](docs/ai_disclosure/A.md) · [D](docs/ai_disclosure/D.md)（说明见 [docs/ai_disclosure/README.md](docs/ai_disclosure/README.md)）。

## 数据说明

`data/` 不入库。

- **Toy 数据**：从课程仓库 `AIE1903_AD/data/sample_trajectories/` 复制到本仓库 `data/sample_trajectories/`。
  （若直接放在 `data/` 根下也能读：`load_toy()` 会回退并在 stderr 提示。）
- **nuScenes mini**（可选，L3）：自行下载；所有结论须在合成场景上独立成立。

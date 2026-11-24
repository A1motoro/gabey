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

完整方案见 [docs/spec.md](docs/spec.md)（当前 v2；模块分工与排期见 §2、§4、§5）；当前进度见 [docs/status.md](docs/status.md)。后续主线闭环实验使用 [L2 基线规范](docs/l2-baseline.md)的滞回制动和定向矩形车身。A 的冻结 Waymo 案例显式使用圆形兼容参数，见下方 L3 说明。

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
python experiments/exp02_threshold_sweep.py --quick   # D：L2 基线的碰撞率–误报率（旧口径加 --legacy-l1）
python experiments/exp03_failure_boundary.py --quick  # D：L2 基线的四条 failure boundary（旧口径加 --legacy-l1）
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
产出位于 `outputs/figures/l1-baseline/`。样例用于验证 P 的方法，不替代 A 的场景模块或
D 的实验矩阵。CA、CTRV、Oracle 已在 L2 实现，LR 已在 P 的 L3 实现。
D 的已合入实验见 [docs/d-baseline.md](docs/d-baseline.md) §9。当前 `exp02`、`exp03`、`exp05` 默认已对齐 [L2 基线规范](docs/l2-baseline.md)（定向矩形 + 滞回制动 + 4 s 时域 + Swept TTC），传入 `--legacy-l1` 可复现原单阈值与圆形配置。
`exp04` 属于 L3。场景由 A 的 `make_scenario` / `scenario_set` 生成，exp02 另加 6 个中止换道。
测试结果和复现说明见 [docs/l1-validation.md](docs/l1-validation.md)。

### C 的 L2 闭环仿真

在仓库根目录运行 `python experiments/c_l2_closed_loop.py`，复现 A 的 12 个 CA 切入场景与 P 的评估器闭环对比。配置见 [L2 基线规范](docs/l2-baseline.md)，结果见 [C 的 L2 仿真报告](docs/C_L2_closed_loop.md)。

### P 的 L2 预测器验证

CA 用最近 10 帧位置拟合二维速度/加速度并限制停车后的反向外推；CTRV 用解缠航向
估计转率，零转率退化为 CV；Oracle 按唯一 agent ID 插值真实未来。
保持已有 `predict(history, horizon, dt)`、`bind(scenario)` 和 Risk/P/C 接口。

```bash
python -m pytest tests/test_predictors.py tests/test_l2_predictors.py -q
python experiments/exp01_predictor_ade.py
```

`exp01` 比较 CV / CA / CTRV / Oracle，使用 A 的 CA 场景集及 lead-brake/crossing 固定切片，
输出 CSV、ADE/FDE 图与摘要到 `outputs/figures/l2-predictors/`。
适用条件、实测结果及 Oracle 真值覆盖限制见 [docs/l2-validation.md](docs/l2-validation.md)。

### P 的 L3：LR 预测器

`LinearRegressionPredictor(m=10)` 用最近位置按真实时间间隔拟合二维线性趋势，
使用拟合截距平滑观测噪声；单帧退化为 CV，不读取未来或真值加速度。

```bash
python -m pytest tests/test_l3_predictors.py -q
python experiments/exp01_predictor_ade.py --include-lr
```

L3 的五模型同切片结果输出至 `outputs/figures/l3-predictors/`；原 exp01 默认四模型入口保留。
验证与模型限制见 [docs/l3-validation.md](docs/l3-validation.md)。

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
  demo/           本地浏览器单场景多系统对比
experiments/      一个实验一个脚本，产出 outputs/figures/
tests/            pytest
docs/             interfaces / scenarios / d-baseline / results / reproducibility / ai_disclosure / contributions
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

- **TTC (time-to-collision)**：在预测时域内，ego 与 agent 的预测车身首次发生面积重叠 的时间；无冲突为 `inf`。
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
| `experiments/exp00_cv_ade.py` | **M1 上的 CV 基线**：在 `ca_scenario_set()` 的 12×25 固定切片上给 CV 打分（不改预测器） | `outputs/figures/exp00_cv_ade.png` |
| `experiments/exp01_predictor_ade.py` | P 的 L2：CV / CA / CTRV / Oracle 同切片 ADE/FDE | `l2-predictors/exp01_predictor_ade.png` |
| `experiments/exp02_threshold_sweep.py` | 各系统碰撞率 vs 误报率 trade-off（`--quick`） | `exp02_tradeoff.png` |
| `experiments/exp03_failure_boundary.py` | **Reactive / CV / CA / Oracle 四边界叠加图**（`--quick`） | `exp03_boundaries.png` |
| `experiments/exp04_latency.py` | 感知延迟 τ ∈ [0, 300] ms 敏感性 | _TBD_（L3） |
| `experiments/exp05_cut_in_lead_time.py` | Reactive vs CV / CTRV 的 cut-in lead time（`--quick`） | `exp05_lead_time.png` |
| `experiments/exp07_real_clip.py` | **A 的 L3**：Waymo 5+5 首批、真实危险/误刹说明案例、CV ADE/FDE | `outputs/figures/exp07_real_{timeline,ade,vs_synth}.png` |

A 的 M1 同时是 CA 的评测底座：`ca_scenario_set()`（12 个带纵向加速度的切入）、`Trajectory.accel` 真值、`prediction_slices()` 固定切片。用法与坑见 [docs/CA_scenario_support.md](docs/CA_scenario_support.md)。

## 结果预览

P 的 L1 固定合成样例使用单阈值和圆形碰撞模型。Reactive 在 2.0 s 开始刹车，CV 在 1.7 s 开始刹车，提前 0.3 s。两者均未碰撞。曲线、参数与测试记录见 [L1 验证报告](docs/l1-validation.md)。

A 的 M1 场景基线：运行 `python experiments/exp00_scenarios.py` 后，`outputs/figures/` 下得到三张图：

| 文件 | 内容 |
| --- | --- |
| `exp00_scenario_geometry.png` | 三类冲突各一个 dangerous 样本（含不刹车时的接触点） |
| `exp00_lateral_accel.png` | quintic 的 y/ẏ/ÿ + 横向加速度过滤（最短换道时长 2.60 s） |
| `exp00_danger_map.png` | 96 个场景的 no-braking danger map（四边界图的底板） |

四边界、trade-off 和 lead time 的 `--quick` 结果见 [D 的评估说明](docs/d-baseline.md) §9；按 claim → evidence → limitation 写的结果与局限见 [docs/results.md](docs/results.md)，复现步骤见 [docs/reproducibility.md](docs/reproducibility.md)。

## Limitations

- 点质量运动 + 随航向旋转的矩形车身（长 5.0 m、宽 2.5 m），无车辆动力学；仅纵向控制，无 steering。
- 主网格为合成构造（quintic 换道 + 横向加速度上限）；L3 真实回放为少量选定窗口，非真实交通分布。
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

按课程 AI Tool Usage Policy 记录于 [docs/ai_disclosure.md](docs/ai_disclosure.md)。

## 数据说明

`data/` 不入库。

- **Toy 数据**：从课程仓库 `AIE1903_AD/data/sample_trajectories/` 复制到本仓库 `data/sample_trajectories/`。
  （若直接放在 `data/` 根下也能读：`load_toy()` 会回退并在 stderr 提示。）
- **Waymo Open Motion v1.2 validation**（L3）：本机原始分片在 `waymo/v1.2/validation/`，
  不入 git；仓库内 `tests/fixtures/` 的 12 个真实 NPZ 快照可独立复现，不需要 TensorFlow 或原始分片。
  nuScenes 的历史 loader 占位接口保留兼容。

### A 的 L3 真实数据交付

```powershell
python -m pytest tests/test_womd.py -q
python experiments/exp07_real_clip.py
```

扫描 2,321 条记录得到 40 个切入目标窗口和 435 个对照窗口。首批按几何顺序选 5+5，
均不触发刹车；完整切入审计另外冻结了危险和误刹说明案例。
危险案例 `6e86a639df5d4fa1`：Reactive 3.2 s、CV 2.9 s 刹车，CV 提前 **0.3 s**，两者都避免碰撞。
另一个 benign 案例两者均在 4.8 s 误刹。说明案例按结果挑选，**不进入首批统计**。
本实验默认比较 Reactive/CV，显式传 `r_safe=2.5` 保持冻结案例的圆形兼容配置。
主线 L2 使用默认矩形 + Swept TTC，不能混用两套结果。CA/CTRV/LR 已合入 main，可通过 `--systems` 加入；
Oracle 因真值未覆盖完整 6 s 仿真 + 3 s 预测而跳过，GRU 仍未注册。

已保存三张主图、地图/片段 BEV、两组表、说明案例表、ADE/FDE、逐帧日志和全候选审计。
方法、实际数字、英文报告段落见 [L3 case study](docs/L3_waymo_case_study.md)，
给 P 的契约和 A 的答辩说明见 [L3 A handoff](docs/L3_A_handoff.md)。

需要从原始数据重新选片时：

```powershell
python -m pip install -e ".[womd]"
python scripts/inspect_womd.py
python scripts/export_womd_clips.py
python scripts/audit_womd_cut_ins.py
python experiments/exp07_real_clip.py
```

本车按 `sdc_track_index` 标识；每窗 81 个连续有效帧，`t=[-2,6]`，只对齐微小时间戳抖动、不插值轨迹。
使用固定 anchor 坐标与直线 ego 路径，周车不响应本车，且只回放选中的单目标。
这些结果支持接口迁移和案例行为，不能作为总体安全收益或学习模型评测的证据。

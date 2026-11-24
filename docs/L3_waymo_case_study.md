# A 的 L3 交付：Waymo 真实轨迹回放（2026-11-23）

已完成 Waymo 原始读取、几何筛选、真实快照、Reactive/CV 回放、预测误差与报告图。
真实危险说明案例中，CV 比 Reactive 提前 **0.3 s** 刹车；这是一个案例的结果。
完整筛选还发现误刹，不能只展示成功案例。本实验默认保留 Reactive/CV 对比。
合入最新 main 后，CA/CTRV/LR/Oracle 已实现；CA 等模型可用 `--systems` 加入，
Oracle 因冻结真值未覆盖完整仿真和预测时域而跳过，GRU 尚未注册。

## 数据和筛选

本机 `waymo/v1.2/validation/` 的 8 个分片共 **2,321** 条记录，读取时检查 TFRecord
长度与两处 CRC32C。**428** 条不满足严格的时间网格门槛，未参与几何筛选。
剩余记录中有 **40 个切入目标窗口、435 个相邻车道对照窗口**；这是候选数，不是独立场景数。
完整台账与来源帧保存在 `tests/fixtures/womd_candidates.csv` 和 `womd_manifest.json`。

本车从官方 `sdc_track_index` 读取，绝不能固定为 `tracks[0]`：第一条记录的本车索引实测为 **10**。
每个窗口含 20 个历史步和 60 个未来步，`t=[-2,6]`、81 帧；ego 和目标全程 `valid=True`。
原始时间戳允许相邻间隔误差 ≤1 ms、整段累计网格偏差 ≤5 ms；只对齐时间轴，不重采样位置。
快照另外保存真实时间戳和有效性标记。12 份快照最大对齐误差 **1.45 ms**。

只保留 ego 速度 >3 m/s、航向变化率 <5°/s、固定 anchor 坐标下横向偏离 <1 m 的窗口。
切入判据是在逐帧 ego 坐标下由同侧 `2≤|lat|≤5 m` 进入 `|lat|≤0.9 m`，
进入时 `5≤lon≤40 m`，间隔 ≤5 s，进入后至少 0.5 s 留在 `|lat|≤1.75 m`；
目标与 ego 航向差 ≤20°。对照需整个历史和未来窗口都位于同侧 `2≤|lat|≤5 m`。
这是直路的车道代理；地图用于 BEV 核验，没有实施地图车道匹配。

**两种选样用途必须分开**：

- **Primary cohort**：按分片、记录、目标 ID 顺序取首批 5 个切入和 5 个对照，场景 ID 互异；不看任何刹车结果。
- **Illustrative cases**：再对全部 40 个切入窗口（39 个场景）回放，冻结首个危险案例与首个误刹案例。
  它们按结果挑选，只用于说明，不进入首批的统计或 ADE 图。

## 已测结果

统一参数：显式圆形兼容配置 `r_safe=2.5 m`，6 s 仿真、3 s 预测、`dt=0.1`、延迟 0、
`HysteresisPolicy(t_on=2, t_off=3, a_brake=6)`。
ego 从 anchor 附近三帧的平均速度开始，沿直线自主仿真；只回放选中的单个目标。
main 的 L2 默认值已升级为定向矩形 + Swept TTC。本实验与审计、测试均显式传入 `r_safe=2.5`，
避免默认参数变更改写冻结证据；这些数字不是 L2 矩形配置的评测结果。

| 集合 / 说明案例 | 永不刹车 | Reactive 首次刹车 | CV 首次刹车 | 结论 |
| --- | --- | --- | --- | --- |
| 首批 5 个切入 | 5 个均无碰撞 | 均未刹车 | 均未刹车 | 切入本身不代表危险；这组没有可计算的提前量 |
| 首批 5 个对照 | 5 个均无碰撞 | 均未刹车 | 均未刹车 | 此样本内误刹 0/5，不能外推到真实交通 |
| `6e86a639df5d4fa1`，target 1782 | 碰撞 | **3.2 s** | **2.9 s** | CV 提前 0.3 s；两者都未碰撞，最小中心距约 4.882 m |
| `b51cfccc51400468`，target 0 | 无碰撞 | **4.8 s** | **4.8 s** | 按既有反事实定义，两者均发生误刹 |

危险说明案例的永不刹车回放首次接触约 **4.722 s**，最小中心距 **0.890 m**；
这些是本项目圆形包络模型中的接触，不是原始视频中发生了车祸。

全部 40 个切入目标窗口中，39 个 benign、1 个 dangerous；两系统各在 2 个窗口刹车，
各有 1 个误刹，均未碰撞。完整逐窗结果是 `tests/fixtures/womd_cut_in_audit.json`。
窗口来自 39 个场景，并非完全独立的 40 个交通事件；这里只报计数。

首批预测评测固定在 `t0=0,0.5,…,3 s` 的 **7 个相同起点**，每次给恰好 20 帧（含当前帧，跨度 1.9 s），
未来从 `t0+0.1 s` 开始，禁止未来泄漏。

| CV 预测时域 | 切入 ADE / FDE（m） | 对照 ADE / FDE（m） |
| --- | --- | --- |
| 1 s | 0.186 / 0.423 | 0.161 / 0.373 |
| 2 s | 0.550 / 1.377 | 0.508 / 1.339 |
| 3 s | 1.057 / 2.677 | 1.055 / 2.881 |

先对每个片段的 7 个窗口取平均，再对 5 个片段取平均；图上的误差棒是片段间标准差，非置信区间。
首批 ego 速度范围 5.05–16.84 m/s；切入初始间距 9.06–32.37 m，接近速度 -1.63–0.41 m/s。
危险说明案例为 14.25 m/s、25.17 m、3.49 m/s，与合成网格有局部重叠，但许多真实点在网格外。
`lc_T_s` 仅是从 `|lat|≥2` 到 `|lat|≤0.9` 的部分过渡时间，**不能当成完整 quintic 换道时长**。
`peak_lat_acc_est_mps2` 是对固定坐标下的位置用 11 帧二次拟合得到的诊断值，不是真值或物理标定。

## 图、表与复现

三张主图：`outputs/figures/exp07_real_timeline.png`、`exp07_real_ade.png`、`exp07_real_vs_synth.png`。
另有原始地图 BEV 和冻结片段 BEV。Timeline 中橙色入口线用逐帧 ego 坐标定义，
底部曲线是固定 anchor 坐标；它不等于 Reactive 的固定路径 `±1.75 m` 车道边界。
距离图同时展示不刹车反事实曲线，灰色 TTC 空白表示无有限预测冲突。
首批两张表为 `outputs/logs/exp07_real_clip.csv` 与 `exp07_real_control.csv`；
说明案例单独在 `exp07_real_cases.csv`，其余逐片段 ADE/FDE、逐帧日志和参数在同目录。

只使用仓库内快照，队友无需原始数据或 protobuf：

```powershell
python -m pip install -r requirements.txt
python -m pip install -e .
python -m pytest tests/test_womd.py -q
python experiments/exp07_real_clip.py
```

从原始分片重建全部结果：

```powershell
python -m pip install -e ".[womd]"
python scripts/inspect_womd.py
python scripts/export_womd_clips.py
python scripts/audit_womd_cut_ins.py
python experiments/exp07_real_clip.py
```

官方 proto 的固定版本与 Apache-2.0 许可证在 `vendor/`，预编译 descriptor 在 `pbs.data` 包内。
只有维护者重新编译它才需要 `grpcio-tools` 和网络，常规运行不需要 TensorFlow。
API 和给 P 的输入契约见 [interfaces.md](interfaces.md)；A 的讲解要点见 [L3_A_handoff.md](L3_A_handoff.md)。

2026-11-23 初次交付（`e1fa151`）本机具备课程 toy CSV 和 `[womd]` 额外依赖，验收：`pytest -q -p no:cacheprovider` 为 **266 passed / 2 skipped**（18.33 s），
两个 skip 是既有 CTRV/Oracle 未实现项；新增 L3 测试 **39 项**（38 项快照/适配测试 + 1 项可选 CRC 测试）。
`ruff check src tests` 和新增脚本检查通过，`git diff --check` 通过，三张图已视觉核验。
另在项目外临时目录运行 exp07，逐行 CSV/JSON 与默认产物一致；
NPZ 测试还明确阻断了 protobuf 导入和原始 Waymo 文件访问。

在 L3 分支合并 main（`a3f91cb`）后，本机具备课程 toy CSV，PR #16 集成验收：
**389 passed**（38.60 s），无跳过；这是本分支合并 main 的验收，不能据此推断所有克隆环境。
Ruff 检查通过。默认 exp07 重跑保持两份说明案例时刻一致；可选 CA 接入通过，
Oracle 的未来真值覆盖检查阻止把端点保持当成真实未来。

## PR #16 评审后的干净克隆说明

组员在 `7d6c1ba` 的干净克隆（无课程 toy CSV）上实测 **387 passed / 2 failed**。
失败是 `test_load_toy` / `test_load_toy_ego` 两项依赖外部课程文件的集成测试，不是 L3 回归。
本次修复使缺少课程 CSV 时这两项明确 skip，并增加自包含 CSV 单元测试，
保证没有外部数据也会测试 parser、ego 排除和稳定 agent ID。已有 CSV 的解析/内容错误仍然失败。
未安装可选 google-crc32c 时，独立 CRC 测试也会 skip；NPZ 快照与默认 exp07 不需要 protobuf。
`389 passed` 是此前特定环境的历史记录。
2026-11-24 基于 main `e7453f1` 的后续修复实测：有课程 CSV 时 **408 passed**（67.88 s）；
纯 Git 克隆、不含课程 CSV/原始 TFRecord 时 **406 passed / 2 skipped**（51.22 s），无失败。
两种环境均安装了基础与 `[womd]` 依赖；干净克隆的默认 exp07 也运行成功。

## 可直接放进英文报告的段落

> We validated the M1 interface on Waymo Open Motion v1.2 validation trajectories. Eight shards
> contained 2,321 records; our conservative clock and straight-road filters yielded 40 cut-in
> target windows and 435 adjacent-lane control windows. The first five distinct scenes per group,
> selected without consulting system outcomes, produced no collision or braking in six-second
> replay with the explicitly selected historical 2.5 m circle-contact model. On these clips, CV achieved mean 3-second ADE/FDE of 1.057/2.677 m for cut-ins and
> 1.055/2.881 m for controls. We additionally audited all 40 cut-in windows from 39 scenes and
> explicitly selected two illustrative cases. In the single dangerous replay, CV initiated braking
> at 2.9 s and Reactive at 3.2 s; both avoided contact. In a benign replay both braked at 4.8 s,
> which counts as false braking under our counterfactual definition. These illustrations are
> excluded from primary-cohort statistics. Agents are replayed without responding to the ego,
> the ego follows a straight-path surrogate, and only the selected target is simulated. The
> evidence demonstrates interface transfer and case-specific behavior, not population-level
> safety gains, statistical significance, or validation of learned predictors. These results
> retain the historical circle configuration and are separate from the main L2 OBB baseline.

引用：Scott Ettinger et al., *Large Scale Interactive Motion Forecasting for Autonomous Driving:
The Waymo Open Motion Dataset*, 2021，核实入口 [arXiv:2104.10133](https://arxiv.org/abs/2104.10133)。

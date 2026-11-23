# 仓库状态更新 — 2026-11-23：A 的 L3 已交付

**A 的 L3 已从“只有数据”推进到可独立复现的真实数据证据**：Waymo 原始 CRC 读取、
官方 SDC 索引、有效窗口、5+5 首批和 2 个说明案例共 12 个 NPZ；三张主图、BEV、
两组 CSV、说明案例表、CV ADE/FDE、全候选审计、测试与交接说明。

8 分片 / 2321 条记录 → 40 个切入目标窗口、435 个对照窗口；首批 5+5 都 benign，均无刹车。
完整切入审计中 1 dangerous / 39 benign（39 个场景）；真实危险说明案例 CV 2.9 s、Reactive 3.2 s，
均避免碰撞；另有两者均误刹的说明案例。说明案例不进入首批统计。
**CA / Oracle 未实现，GRU 未注册，不能把本次交付说成四系统或学习模型评测完成。**

复现：`python experiments/exp07_real_clip.py`；报告和限制见
[L3_waymo_case_study.md](L3_waymo_case_study.md)，给 P 的输入契约与 A 的答辩说明见
[L3_A_handoff.md](L3_A_handoff.md)。原始分片继续被 ignore；小型 `exp07` 图表已允许纳入代码包。
此前 demo 测试的 Windows 路径、端口预留与子进程编码问题已在测试侧修正。

最终本机验收：**266 passed / 2 skipped**（18.33 s）；新增 L3 39 项测试通过，
Ruff（src/tests 与新增脚本）通过，项目外目录复现通过。

---

# 以下保留原 2026-11-23 21:30 (+08:00) 的历史快照

> 下文的 L3 “代码 0%”、旧测试失败与权限阻塞是此前状态，不代表上述交付后的结果。

> 一次性快照，下次更新直接覆盖。Git / pytest / Waymo 数据三项是本机实测（命令见 §9）；
> 其余数字引自各自文档并在表内标注出处。
> 上一份状态记录是 [ai_disclosure.md](ai_disclosure.md) 的 2026-11-18 条。

## 1. 一页结论

| 问题 | 答案 |
| --- | --- |
| 代码在哪 | 分支 `feat/m1-ca-review-fixes` @ `aa25d3f`，比 `origin/main` @ `3b77312` 多 **1 个 commit**，**还没合进 main** |
| 测试怎么样 | **226 passed / 2 skipped / 1 failed**；唯一失败是测试自己写了 Windows 不存在的 `cwd="/tmp"`，不是代码问题 |
| 做到哪一层 | **L1 基本完成**（差 exp05 的 lead-time 全量）；**L2 一行没写**；**L3 只有数据落地，代码没动** |
| 主图（四边界）能画吗 | **不能**。CV 之外三个 predictor（CA / CTRV / Oracle）还是 `NotImplementedError`，`run_matrix` / `tradeoff` / `failure_boundary` / `plot_boundaries` 也还是空函数 |
| 离截止多远 | 12-08 最终汇报 + demo（**15 天**）；12-13 代码包 + 报告（**20 天**） |
| 最该做的事 | L2 的 **CA + Oracle + `run_matrix` + exp03 四边界**；L3 按计划自己的判断线降档 |

一句话：**M1（场景）和 M3（仿真）能用了，M2 只有 CV，M4 只有零散指标，所以"prediction 扩大了多少安全区"这个主结论目前还画不出来。**

## 2. Git 状态

```
分支        feat/m1-ca-review-fixes
HEAD        aa25d3f  fix(M1): CA slice counts 6/6, sliding history window, CV baseline  (2026-11-18 21:54)
origin/main 3b77312  Merge pull request #7 from A1motoro/feat/m1-ca-support
本地工作区  已跟踪文件 0 处修改，14 个未跟踪文件（快照前是 22 个，8 个 Waymo 分片已转忽略），无 stash
```

- 本分支 = `origin/main` + 1 个 commit（10 文件 / +265 −13），即 **PR #7 评审要求的改动已经推上去了，但还没被合入**。
- 远端未合入 main 的分支只有两个：**`origin/feat/m1-ca-review-fixes`（本案）** 与 `origin/exp/p-visual-lab`（P 的 10-28 可视化脚本 `experiments/viz_baseline.py`，741 行，一直没合）。
- 本地 `main` 落后 `origin/main` 4 个 commit（从没切回去更新过，不影响当前分支）。
- **远端新鲜度未核实**：本会话沙箱禁止 SSH（`ssh.exe: couldn't create signal pipe`），`git ls-remote` 失败；仓库最后一次 fetch 是 **2026-11-18 21:47**。要确认这 5 天有没有人推东西，需要在正常终端里 `git fetch --all --prune`。

### 未跟踪文件（22 → 14 个）

| 类别 | 文件 | 建议 |
| --- | --- | --- |
| 原始数据（**2.06 GB**） | `waymo/v1.2/validation/*.tfrecord-00xxx-of-00150`（8 个分片） | **不提交**。已在 `.gitignore` 加 `/waymo/v1.2/`、`*.tfrecord*`（本次改动） |
| L3 计划与工具 | `docs/L3_waymo_plan.md`（885 行）、`waymo/womd_dl.py`、`waymo/_verify.py` | 建议提交（可复现的数据获取与校验，不含数据本体） |
| 期中材料 | `docs/proposal.md`、`docs/proposal_writing.md`、`docs/section2_zh.md`、`docs/references_candidates.md`、`docs/related_work.md`、`docs/midterm_plan.md`、`gabey_proposal_A.docx` | 建议提交（DL2 的历史证据） |
| 审计/规范 | `docs/data_audit.md`、`project_guidelines.md`、`scripts/csv_to_html.py` | 建议提交（后两者是课程原文与工具） |
| 工具产物 | `.acl_reports/acl-report-….jsonl` | 不提交。已加 `.acl_reports/` 到 `.gitignore` |

## 3. 本机复核结果

环境：系统 Python 3.11.3（`%LOCALAPPDATA%\Programs\Python\Python311`）、pytest 9.1.1、`pbs` 为 editable 安装。
注意：本会话里 `conda run -n aie1903_ad` 被沙箱拒绝，且该环境内**没有 pytest**，所以 [l1-validation.md](l1-validation.md) 里那套 conda 命令这次没法原样复现；代码本身不受影响。

```powershell
python -m pytest -q -p no:cacheprovider     # 226 passed, 2 skipped, 1 failed, 22.3 s
python waymo\_verify.py waymo\v1.2\validation
```

- **2 skipped**：`test_ctrv_degenerates_to_cv`、`test_oracle_exact` —— 就是 P 的 L2 TODO，属预期。
- **1 failed**：`tests/test_demo.py::test_http_assets_errors_and_results`。该测试最后一步用 `subprocess.run(..., cwd="/tmp")` 启动 `examples/cut_in_demo.py` 验证端口占用报错；Windows 没有 `/tmp`，于是 `NotADirectoryError`。**同一测试前半段（静态资源、`/api/*` 状态码、CORS 403、路径穿越 404）全部通过**。这是 11-18 就记录过的既存问题，本机与当时结果逐项一致。修法二选一：`cwd=Path(tempfile.gettempdir())`，或直接把那段 `cwd` 删掉（脚本不依赖工作目录）。
- **Waymo 数据完整**：8 个分片、合计 **2,321 条** `Scenario` 记录、逐个 `OK`（TFRecord 长度前缀、CRC 边界、无截断）。
- `outputs/` 里已有 4 张 exp00 图 + CSV/汇总（11-18 生成）；`docs/l1-validation.md` 引用的 `outputs/figures/l1-baseline/` 在这台机器上不存在。
- **本次没能补图**：本会话沙箱进程对 `outputs/`、`docs/` 只有只读权限（`CodexSandboxUsers: ReadAndExecute`），`exp00_l1_baseline.py` 在 `output_dir.mkdir()` 处 `PermissionError`。这不是仓库问题；在自己终端里跑同一条命令即可生成。

## 4. 模块进度

| 模块 | 负责人 | 已能跑 | 还是空的 |
| --- | --- | --- | --- |
| **M1 场景** `pbs.data` | A | `load_toy` / `load_toy_ego`；`make_scenario` 三类（`cut_in` + `a_cutter`、`lead_brake`、`crossing`）；quintic + 横向加速度过滤；`DEFAULT_GRID` 96 场景（**52 dangerous / 44 benign**，1 个 marginal）；`ca_scenario_set()` 12 场景（**6 / 6**）；`prediction_slices()` 滑动窗口 | `load_nuscenes_tracks`（L3，已决定换 Waymo）；`womd.py` 不存在 |
| **M2 预测** `pbs.prediction` | P | `ConstantVelocityPredictor`（唯一能跑的） | **`ConstantAccel` / `CTRV` / `LinearRegression` / `Oracle` 全部 `NotImplementedError`**；GRU 不在本仓库（计划里提到的是 P 分支上的东西） |
| **Risk** `pbs.risk` | P | `assess_risk`（首次冲突 TTC）、`ReactiveAssessor`、`PredictiveAssessor` | — |
| **M3 仿真** `pbs.sim` | C | `run_sim`（闭环、receding horizon、步内 0.01 s 扫掠碰撞）、`ego_rollout`、`path_point/heading`、`TTCThresholdPolicy`、`HysteresisPolicy`、观测延迟 `ObservationModel`、`MissingObservationError` | 计算延迟 ≠ 0（`run_sim` 明确抛 `NotImplementedError`，`timing.py` 只留了缝） |
| **M4 评估** `pbs.eval` | D | `is_benign`、`evaluate`、`ade`/`fde`、`SYSTEMS`/`CORE`、`make_system` | **`run_matrix` / `tradeoff` / `failure_boundary` / `safe_area` / `plot_boundaries` 全是空函数** → exp02/03/04 无从谈起 |
| **Demo** `pbs.demo` | C | 交互式 cut-in 对比（`/`）、投屏增强版（`/presentation`）、事件跳转、参数面板；3 个测试文件覆盖 HTTP/前端资源/交互契约 | — |

实验脚本现状：

| 脚本 | 状态 |
| --- | --- |
| `exp00_scenarios.py` | ✅ 已产出 3 图 + CSV + 汇总（96 场景） |
| `exp00_cv_ade.py` | ✅ CV 在 12×25 切片上的 ADE（0.169 / 0.334 / 0.334 m） |
| `exp00_l1_baseline.py` | ✅ 固定样例：Reactive 2.0 s、CV 1.7 s 刹车，提前 **0.3 s** |
| `exp01`…`exp05` | ❌ 全部 `raise NotImplementedError` |
| `exp06_demo_comparison.py` | ✅ demo 侧的对照导出 |
| `exp07`（真实片段） | ❌ 不存在 |

## 5. 分层交付对照（spec §1.3）

| 层 | 定义 | 现状 | 缺什么 |
| --- | --- | --- | --- |
| **L1 保底** | CV + ADE/FDE；Reactive vs CV 的 TTC 刹车与 timeline；lead time | **约 90%** | 只有固定样例与 demo 的 0.3 s；缺"全场景 lead-time 分布"（`exp05`），缺 CV 在 M1 全网格上的 ADE/FDE（`exp01`） |
| **L2 核心** | + CA、CTRV、Oracle；counterfactual 误报 + trade-off；**四边界叠加图** | **0%**（底座已备好） | 3 个 predictor、`run_matrix`、`tradeoff`、`failure_boundary`、`exp02`、`exp03` |
| **L3 加码** | LR；latency 实验；真实数据 | **数据 100%、代码 0%** | proto 未编译（`import waymo_open_dataset` 失败）、`pbs/data/womd.py` 不存在、loader 未改名、`exp04`/`exp07` 空 |

## 6. 和计划的偏差

- **L3（Waymo）计划自带的判断线已经踩过**：计划 §9 写"11-22 还没跑通真实片段的 `run_sim` → 跳档 3"（只做 loader + 单测 + 一张 BEV，不做定量结论），"11-26 仍无图 → 档 4（明写不做）"。今天是 11-23，**Step 0 的 proto 编译都还没开始**，所以已经落在档 3 的窗口里。
- **L2 才是被评分的主图，却排在最迟**：spec 的 W8–W10（四边界图）本该在 11 月中，现在 0%。CA 的评测底座（A 的 11-18 交付）已经把 CA 要用的场景/真值/切片都准备好了，**卡点只在 P 的 `ConstantAccelPredictor`(+Oracle) 和 D 的 `run_matrix`**。
- 报告里要写的 `r_safe = 2.5` 语义（`collision.py:9` 仍标着 TODO）从 10 月挂到现在，直接决定"碰撞"与"边界"的口径，**写报告前必须定稿**。

## 7. 文档不同步（都不影响代码，但会被老师看到）

| 位置 | 现在写的 | 实际 |
| --- | --- | --- |
| `README.md`「数据说明」、`docs/spec.md` §1.3 | L3 = nuScenes mini | 已换成 **Waymo Open Motion**（[L3_waymo_plan.md](L3_waymo_plan.md)） |
| `README.md` 实验表 | exp01–exp05 标 `_TBD_` | 仍准确，但没写"L1 只有固定样例" |
| `docs/interfaces.md` | M4 `run_matrix`… 标 TODO | 仍准确 |
| `docs/contributions.md` | 只到 W11 | 本周（W12）无人提交记录；本周日应补 |
| `docs/A_baseline_handoff.md` §8 | `load_nuscenes_tracks` 保持 `NotImplementedError` | 与 Waymo 计划冲突，等 L3 落地再改 |

## 8. 建议的下一步（按截止倒排）

| 优先级 | 做什么 | 谁 | 验收 |
| --- | --- | --- | --- |
| P0 | `ConstantAccelPredictor` + `OraclePredictor` 落地（CA 用手/位置窗口最小二乘估 a，别差速度列） | P | `test_predictors.py` 两个 skip 变绿；CA 在 `a_cutter≠0` 切片上 ADE 明显低于 0.334 m |
| P0 | `run_matrix` + `tradeoff` + `failure_boundary` + `safe_area` | D | `exp02` 出 trade-off 曲线，`exp03` 出四边界 |
| P0 | `exp01`、`exp05` 从空函数变成脚本 | P / D | 两张图进 `outputs/figures/`，README 表格去掉 `_TBD_` |
| P1 | `r_safe` 语义定稿（车体包络推导 vs 圆心距） | C | `collision.py` 的 TODO 消失；96 场景 danger 计数变化写进文档 |
| P1 | 12-08 demo 打磨：用四边界图 + 真实片段（若做）串一条 10 分钟叙事 | C + 全员 | 预演一次 10 分钟 |
| P2 | L3 档 3：编译 proto → `inspect_womd.py` → 一张 BEV + `load_womd_tracks` 单测 | A | 删掉 TFRecord 后 `tests/test_womd.py` 仍绿 |
| P2 | exp05 lead-time 分布 + exp05 图 | P / D | 支撑"提前 X 秒"这句话 |
| P3 | `test_demo.py` 的 `cwd="/tmp"` 修掉 | C | `pytest` 全绿（227 passed） |

## 9. 命令与产物（按需）

> 前三行是本快照的实测依据；`exp00_*` 三条会写 `outputs/`，需在可写的终端运行（见 §3 末尾）。

```powershell
git status --porcelain -uall                        # 22 个未跟踪文件
python -m pytest -q -p no:cacheprovider             # 226 / 2 / 1
python waymo\_verify.py waymo\v1.2\validation       # 8 分片 / 2321 记录 / 全 OK
python experiments\exp00_scenarios.py               # 96 场景（52/44，1 marginal）
python experiments\exp00_cv_ade.py                  # CV: 0.169 / 0.334 / 0.334 m
python experiments\exp00_l1_baseline.py             # Reactive 2.0 s / CV 1.7 s
python examples\cut_in_demo.py                      # 浏览器 demo，/ 与 /presentation
```

# D 的 Evaluation Baseline（M4 · `pbs.eval` / `pbs.systems`）

> `pbs.eval` 与 `exp02` / `exp03` / `exp05` 已对齐 [L2 基线规范](l2-baseline.md)（定向矩形 `r_safe=None` + 滞回制动 `HysteresisPolicy(t_on, t_on+1.0, a_brake=6.0)` + `horizon=40` + Swept TTC）；传入 `--legacy-l1` 可复现原 PR #8 的单阈值与圆形配置。

PR #8 审核修正（2026-11-18）：没有阈值满足误刹预算时，先选择最低误刹率；
场景 ID 纳入完整轨迹、路径和几何参数，避免不同中间运动错误共享 benign 标签；
Reactive/CA 的 x/+ 碰撞点保持可见；README 链接恢复到仓库实际存在的 spec 和 AI 记录。
exp02/03/05 快速实验已重跑，§9 的主要数字保持一致，图与 CSV 已核对。
审核修正后全仓回归 300 passed / 1 deselected（已知 Windows Demo 端口测试），
变更 Python 文件 Black / Ruff 与 `git diff --check` 通过。

> 2026-11-23：exp02 / exp03 / exp05 的 L2 模式改用**两个窗口**——刹车决策在前 15 s 内计分，碰撞结果看到 19 s——并把场景分成 dangerous / late_contact / benign 三类（§3、§6-Q2）。这是 D 在评估侧的实现，**定义变更尚待 A、C 确认**；`--legacy-l1` 仍是单一 15 s 窗口。

> Owner：D（yj）· 配合现有 [spec.md](spec.md) v2 与 [interfaces.md](interfaces.md)。
> P 的 baseline 回答"系统怎么判断风险"；D 的 baseline 回答"**怎么公平地判定一个系统好不好**"。本文记录 D 的度量、数据表、实验、验收标准与 L2 验证结果。

## 1. 职责边界

| 输入（来自谁） | D 做什么 | 输出（给谁） |
| --- | --- | --- |
| `Scenario` 集与参数网格（A）<br>`run_sim → SimLog`（C）<br>`SYSTEMS` 中的 assessor（P） | 跑实验矩阵 → 每次仿真一行 → groupby 出指标与图 | 报告 F1–F5（全组）、proposal Evidence 节、final demo 的结论数字 |

D **不**实现场景、预测或仿真逻辑；发现上游问题时提 issue 给对应 owner，不在 `pbs.eval` 里打补丁绕过。

## 2. 本次 L2 实现状态

| 组件 | 文件 | 状态 | 说明 |
| --- | --- | --- | --- |
| `SYSTEMS` / `CORE` / `make_system` | `pbs/systems.py` | ✅ | `CORE = reactive, cv, ca, oracle`。CA / CTRV / Oracle 由 P 实现（见 `docs/l2-validation.md`）；LR 仍是 L3 |
| `is_benign` | `eval/counterfactual.py` | ✅ | NullAssessor + NeverBrakePolicy 下不碰撞 ⇒ benign |
| `never_brake_label` | `eval/counterfactual.py` | ✅ | 返回三类标签与无制动首次接触时刻。dangerous 的判定就是 `not is_benign(duration=决策窗口)` |
| `ade` / `fde` | `eval/metrics.py` | ✅ | P 的 exp00 已复用 |
| `evaluate(log, benign, decision_end=None)` | `eval/metrics.py` | ✅ | collision / min_dist / brake_onset_t / false_brake / benign；给出 `decision_end` 时，只有在此之前下达的刹车才计为误刹 |
| `plot_timeline` | `eval/plots.py` | ✅ | ego v(t) + 刹车阴影，可多系统叠加 |
| `run_matrix` | `eval/sweep.py` | ✅ L2 | §4 schema；无制动标签按场景缓存，并与主实验共用 `sim_kwargs`；`decision_duration` 给出决策窗口，不传则为单一窗口 |
| `tradeoff` / `select_operating_point` | `eval/sweep.py` | ✅ L2 | §5.1；工作点为 false-brake ≤ 5% 下碰撞率最低的阈值 |
| `late_contact_outcomes` | `eval/sweep.py` | ✅ | 各系统在 late_contact 场景里的刹车与碰撞计数（这些场景不进入任何比率） |
| `failure_boundary` / `safe_area` / `rescue_rate` | `eval/sweep.py` | ✅ L2 | §5.2。主指标是 rescue rate，不是含 benign 格子的面积 |
| `plot_boundaries` | `eval/plots.py` | ✅ L2 | 碰撞格必画；碰撞格 ≥ 4 时才叠 0.5 等高线，避免两个孤立点被画成一块区域 |
| exp02 / exp03 / exp05 | `experiments/` | ✅ L2 `--quick` | 见 §9。exp01 归 P，exp04 仍是可砍的 L3 |
| 测试 | `tests/test_sweep.py`、`test_scenario_sets.py`、`test_counterfactual.py` | ✅ | 合并 main 后全库 294 passed，2 failed 为缺 `data/sample_trajectories`（2026-11-18，课程环境 `aie1903_ad`） |

## 3. 核心定义（proposal 与报告中写死）

| 概念 | 定义 | 备注 |
| --- | --- | --- |
| **Dangerous / Late contact / Benign** | 同一场景用 `NeverBrakePolicy`：15 s 内接触 ⇒ dangerous；15 s 内无接触、19 s 内接触 ⇒ late_contact；19 s 内无接触 ⇒ benign | 与系统无关，每个场景只算一次并缓存。dangerous 与原单窗口标签逐场景相同。late_contact 不进入任何分母，单独报告 |
| **Collision rate** | dangerous 场景中 19 s 内仍碰撞的比例 | 分母**只含 dangerous**；benign 场景不可能因刹车而碰撞（agent log replay）。刹车只把接触推迟到 15 s 之后的，仍算碰撞 |
| **False-brake rate** | benign 场景中前 15 s 内出现过 brake 的比例 | 分母只含 benign |
| **Brake onset** | 第一次 action = brake 的时刻 | 从未刹车记为 `None`；取整段 19 s |
| **Lead time** | `onset(Reactive) − onset(Predictive)`，同一场景同一阈值 | 仅在两者都刹车时有定义；一方未刹车的场景单独计数，不丢弃 |
| **Failure boundary** | (gap_d, rel_v) 网格上，某系统在**固定工作点**下碰撞 / 不碰撞的分界 | 工作点见 §6-Q1 |
| **Rescue rate**（替代 safe_area 的主指标） | dangerous 格子中被该系统救下的比例 | 用 dangerous 格子作分母，避免 benign 格子把所有系统的面积一起抬高 |
| **ADE / FDE** | 预测与真值的平均 / 终点位移误差 | 与 P 共用实现 |

两个窗口的理由：系统在 TTC 低于 `t_on` 时刹车，`t_on` 最大 3.0 s，小于 4 s 预测时域。所以前 15 s 内下达的刹车，所依据的预测接触时刻不晚于 18 s，落在 19 s 的结果窗口内，这个预测对不对由 19 s 的无制动对照判定。只看 15 s 会把为 15 s 之后的真实接触而刹的车记成误刹（方向一），也会把仅被推迟到 15 s 之后的碰撞记成救下（方向二）。实测与拆分见 [results.md](results.md)。

状态：以上是 D 的评估脚本在 L2 模式下的口径。[scenarios.md](scenarios.md) §10 第 5 条、[interfaces.md](interfaces.md) 的 `Scenario.params` 一条和 [l2-baseline.md](l2-baseline.md) 仍写单一 15 s 窗口，归 A、C 维护，本次未改；`--legacy-l1` 与 C 的 `c_l2_closed_loop.py` 也仍是单一 15 s 窗口。

## 4. 实验矩阵数据表（`run_matrix` 输出 schema）

一次 `run_sim` = 一行；所有图都是这张表的 groupby / pivot。

| 列 | 类型 | 来源 |
| --- | --- | --- |
| `scenario_id` | str | `kind` + 参数哈希，保证可复现 |
| `kind` | str | `Scenario.kind` |
| `gap_d`, `rel_v`, …（展开 `params`） | float | A |
| `system` | str | `SYSTEMS` 的 key |
| `policy` | str | `"hysteresis"`（L2 默认）或 `"threshold"`（L1） |
| `threshold`, `t_on`, `t_off` | float | 进入/退出 TTC 阈值 [s]；`threshold = t_on` 兼容旧 groupby |
| `latency` | float | `run_sim(latency=)` [s] |
| `geometry`, `horizon` | str / int | `"obb"` / `"circle"` 与预测步数（L2 默认 40） |
| `label` | str | `never_brake_label`（按 `scenario_id` 缓存）：`dangerous` / `late_contact` / `benign` |
| `benign` | bool | `label == "benign"` |
| `never_brake_contact_t` | float / NaN | 无制动对照的首次接触时刻 [s]；benign 为 NaN |
| `decision_duration`, `outcome_duration` | float | 决策窗口与结果窗口 [s]；单一窗口时两者相等 |
| `collision`, `false_brake` | bool | `evaluate`；`collision` 覆盖结果窗口，`false_brake` 只看决策窗口 |
| `first_collision_t` | float / NaN | `evaluate` [s]，首次接触时刻 |
| `min_dist`, `min_dist_until_contact` | float | `evaluate` [m]；L2 报告优先采用 `min_dist_until_contact` |
| `brake_onset_t` | float / NaN | `evaluate` [s]（已按 `first_collision_t` 截断碰撞后刹车） |
| `seed`, `git_rev` | str | 复现用 |

- 结果写 `outputs/logs/expXX.csv`；画图脚本只读 CSV，不重跑仿真。
- 无制动标签必须与主实验使用**相同的 `duration` / `r_safe`**，否则标签与主实验不一致；`run_matrix` 统一传 `sim_kwargs`（其中 `duration` 是结果窗口），决策窗口用 `decision_duration` 传入。
- 规模估计：网格 15 × 17 = 255 场景 × 4 系统 × 5 阈值 ≈ 5 100 次仿真；`--quick` 用 5 × 5 粗网格 × 2 阈值，目标 < 1 min。

## 5. D 负责的实验

| 实验 | 问题 | 自变量 | 因变量 | 图 | 周 |
| --- | --- | --- | --- | --- | --- |
| exp05（与 P） | prediction 让刹车提前几秒？ | cut_in 场景集；Reactive vs CV / CTRV | lead time 分布 + 未刹车计数 | F2 | W8 |
| exp02 | 每个系统的安全–误报 trade-off？ | `CORE` × 阈值 0.5–3.0 s | collision rate vs false-brake rate，每系统一条曲线 | F3 | W8 → W11 |
| exp03 | prediction 把安全区扩大多少？ | `CORE` × (gap_d, rel_v) 网格，固定工作点 | 四条 boundary 叠加 + rescue rate；叠 1D 制动下界 `d = 5.0 + v_rel²/(2×6)` | **F4** | W9 → W11 |
| exp04（可砍） | 感知延迟的代价？ | latency 0–300 ms × {Reactive, CV} | collision rate / rescue rate vs latency | F5 | W11 |

### 5.1 `tradeoff`
按 `(system, threshold)` 分组：`collision_rate = mean(collision | dangerous)`，`false_brake_rate = mean(false_brake | benign)`；late_contact 行两边都不进，行数记在 `n_late_contact`。没有 `label` 列的旧表按 dangerous = ¬benign 读。若某系统所有阈值下误报率都是 0，说明 benign 集不够"像危险"，回到 A 加样本（spec §1.5），而不是改指标。

### 5.2 `failure_boundary` / `rescue_rate`
在固定工作点下对 (gap_d, rel_v) pivot 出 collision 0/1 矩阵，用 `contour(level=0.5)` 画分界；`rescue_rate = 1 − collisions / dangerous_cells`。报告两个差值：
- `rescue(CV) − rescue(Reactive)` = **value of prediction**
- `rescue(Oracle) − rescue(CV)` = **cost of prediction error**

## 6. 待决问题（W7 前需全组拍板）

| # | 问题 | D 的建议 | 需要谁 |
| --- | --- | --- | --- |
| Q1 | boundary 用什么工作点比较？同一阈值对 Reactive 和 CV 并不公平（误报率不同） | 主图：每个系统取 **false-brake rate ≤ 5%** 时的最小碰撞率阈值；附图：统一阈值 2.0 s | 全组 |
| Q2 | 仿真 duration 是否覆盖所有碰撞？10 s 后才会撞的场景会被误标为 benign | 用 `gap_d / max(rel_v)` 估算并留余量，或延长到 15 s | A、C |
| Q3 | 碰撞严重程度：目前只有 bool | 请 C 在 `SimLog` **新增**（只加不改）`collision_t`，D 由此算 impact speed，作为副指标 | C |
| Q4 | 碰撞后仿真是否继续？现在会继续跑，`min_dist` 不受影响，但 timeline 图会误导 | 画图时在 `collision_t` 处截断；不改仿真语义 | C |
| Q5 | 阈值网格与参数网格的范围 | 阈值 {0.5, 1.0, 1.5, 2.0, 2.5, 3.0} s；参数网格由 A 定，D 只要求 benign 与 dangerous 都 ≥ 20% | A |

Q2 实测（2026-11-23）：单一 15 s 窗口下，exp02 的 53 个 benign 里有 6 个在 15–19 s 之间接触，其中 3 个在 15.00–15.14 s 接触，四个系统都在 13.1–13.3 s 刹车并被记为误刹，Oracle 的 3 例误刹全部来自这 3 个场景。反方向也有 1 例：`lead_brake`（init_gap 100、decel 1.0）在 `t_on = 1.0 s` 时四个系统都只把接触从 14.78 s 推迟到 15.09–15.21 s，却被记为救下。

Q2 处理（2026-11-23，D 的评估侧实现，**待 A、C 确认**）：L2 模式改为 §3 的两个窗口与三类场景。exp02 变为 57 dangerous / 6 late_contact / 47 benign，Oracle 误刹为 0；quick 边界网格无 late_contact（14 / 0 / 11），完整网格为 154 / 6 / 95。`experiments/check_outcome_window.py` 把结果窗口再延长到 25 s 复跑：exp02 没有碰撞标记变化、前 15 s 内的刹车没有变化，只有 1 个 benign 场景（`lead_brake` init_gap 80、decel 0.3）在 23.36 s 接触，且没有系统在前 15 s 内为它刹车。数字与拆分见 [results.md](results.md)。需要 A、C 决定的是：是否接受这个口径并相应修改 scenarios.md §10 第 5 条、interfaces.md 与 l2-baseline.md 的表述；不接受则 D 把 `experiments/scenario_sets.py` 的 `OUTCOME_PAD` 设回 0：两个窗口相等时就是原来的单一 15 s 窗口。

## 7. 验收标准（对应 spec v3 里程碑）

| 里程碑 | D 的验收 |
| --- | --- |
| **M0（W7）** | `run_matrix` 能在 A 的 `cut_in` 上对 {reactive, cv} 跑出 CSV；每个场景的 `benign` 只算一次；单测覆盖 schema 列名 |
| **M1（W9）** | exp05 lead time 图 + exp02 trade-off 初版；benign 与 dangerous 均非空，误报率不恒为 0 |
| **M2（W11）** | exp03 四边界图定稿，报告两个差值；全部实验 `--quick` < 5 min；图与 CSV 一一对应 |
| **M3（W12）** | 报告"实验结果 / 局限"节；每个结论按 claim → evidence（图号）→ limitation 写 |

M3 草稿见 [results.md](results.md)（quick 模式，F2 / F3 / F4）；复现步骤见 [reproducibility.md](reproducibility.md)。两个窗口下三条 `--quick` 实测合计 246 s、245 s（单一 15 s 窗口时为 190 s、195 s），仍在 M2 的 5 min 之内，余量变小。

## 8. 给 proposal 用的 Evidence & Evaluation 草稿

> **Our final evidence will be** a four-way failure-boundary overlay (Reactive / CV / CA / Oracle) on the (gap_d, rel_v) plane for synthetic cut-ins, together with the collision-rate vs false-brake-rate trade-off and the cut-in lead-time distribution, all regenerated by one command from a cached results table.
>
> - **Will demonstrate**：在点质量、log replay、TTC 阈值刹车的设定下，prediction 救下了多少 dangerous 场景、提前多少秒察觉切入，以及预测误差与误报的代价。
> - **Will not demonstrate**：真实交通分布或真实车辆上的 AEB 性能；perception 误差（仅用 latency 近似）；转向避让。
> - **Fairness**：危险 / 良性由与系统无关的 counterfactual 判定；系统间在相同误报率下比较，而非仅在相同阈值下比较。

## 9. L2 quick 结果（2026-11-20，定向矩形 + 滞回 + Swept TTC）

> 本节数字已按 §3 的两个窗口在 2026-11-23 重跑（`63ea148`）。边界网格与 exp05 的数字与 2026-11-20 相同；exp02 的误刹率与 `t_on = 1.0 s` 的碰撞率有变化，对照见 [results.md](results.md)。

场景全部来自 A 的 `pbs.data`，组装在 `experiments/scenario_sets.py`。所有场景都用 `duration=23` 生成、仿真 19 s、刹车决策在前 15 s 内计分：仿真比决策窗口长一个 4 s 预测时域（`horizon=40`），日志再长 `LOG_PAD=4.0 s`，避免 Oracle 把轨迹末端当成停车。默认采用 L2 闭环配置：`r_safe=None`（5.0 m × 2.5 m 定向矩形）、`HysteresisPolicy(t_on, t_on + 1.0, a_brake=6.0)` 与 Predictive Swept TTC；追加 `--legacy-l1` 可复现原 PR #8 的圆形 + 单阈值配置（碰撞与 rescue 数值一致；由于 `evaluate()` 现按 `first_collision_t` 截断碰撞后触发的刹车，`exp05` 在旧口径下会移除 1 例碰撞后 Reactive 刹车统计）。

- exp03 / exp05：`make_scenario("cut_in", lc_T=3.0)` 在 (gap_d, rel_v) 平面上的密网格。`--quick` 为 5×5（gap 8–48 m，rel_v −4–12 m/s）。A 的 `DEFAULT_GRID` 注明了密扫由 D 负责。`exp03` 叠加的理论参照曲线已修正为零延迟 1D 物理制动极限 $d = 5.0 + v_{\text{rel}}^2 / (2 \times 6.0)$（L2 矩形车长 5.0 m；`--legacy-l1` 下为 $2.5 + v_{\text{rel}}^2 / (2 \times 6.0)$）。
- exp02：A 的 `scenario_set()`（切入 60、前车制动 20、横穿 16，无被拒组合）加 `ca_scenario_set()` 中 `a_cutter = ±2` 的 8 个，再加 6 个中止换道，共 110 个。中止换道是 A 的生成器没有的一类：切入车向本车道靠近后返回，在 L2 矩形几何与 L1 圆形几何下均保持 benign。

```bash
python experiments/exp02_threshold_sweep.py --quick
python experiments/exp03_failure_boundary.py --quick
python experiments/exp05_cut_in_lead_time.py --quick
```

边界网格 25 个切入里 **14 个 dangerous、11 个 benign**。在 false-brake ≤ 5% 的工作点上（这组粗网格里与固定 `t_on = 2.0 s` / `t_off = 3.0 s` 的 rescue 相同）：

| 系统 | rescue rate | 含义 |
| --- | ---: | --- |
| Reactive | 0.429 | 6 / 14 dangerous 救下 |
| CV | 0.643 | 9 / 14；rescue(CV) − rescue(Reactive) = **0.214**，prediction 的收益 |
| CA | 0.643 | 与 CV 相同；这段无纵向加减速的 quintic 上常加速度没有分开 |
| Oracle | 0.857 | 12 / 14；rescue(Oracle) − rescue(CV) = **0.214**，预测误差的代价 |

exp02 共 **57 个 dangerous、6 个 late_contact、47 个 benign**。`t_on = 2.0 s`（`t_off = 3.0 s`）时碰撞率严格递减：Reactive **0.246**、CV **0.123**、CA **0.070**、Oracle **0.035**；误报率 Reactive **0.021**（1/47）、CV / CA **0.085**（4/47）、Oracle **0**。`t_on = 1.0 s`（`t_off = 2.0 s`）时四个系统的碰撞率在 0.386–0.491。6 个 late_contact 场景里四个系统在两个阈值下都刹车且都没有碰撞，不计入任何比率。分类来看（`t_on = 2.0 s`）：

- 切入（A 的网格，28 dangerous / 4 late_contact / 28 benign）：四个系统误报率均为 0；碰撞率 Reactive 0.250、CV 0.143、CA 0.071、Oracle 0.000。
- 前车制动（16 dangerous / 2 late_contact / 2 benign）：误报率均为 0；CA 与 Oracle 无碰撞，CV 与 Reactive 各 1 例（0.0625）。
- 横穿（9 dangerous / 7 benign）：预测系统（CV / CA / Oracle）碰撞率均为 0.222（2/9）、误报率均为 0；Reactive 碰撞率 0.667（6/9）、误报率 0.143（1/7）。
- CA 切片（`a_cutter = ±2`，4 dangerous / 4 benign）：所有系统碰撞率与误报率均为 0。
- 中止换道（6 benign）：CV 和 CA 的误报率在 2.0 s 时均为 0.667（4/6），Reactive 和 Oracle 为 0。去掉中止换道后，CV / CA 的误报率从 0.085 降到 0（0/41），Reactive 为 1/41，`exp02_summary.json` 里两组数都有保留。

exp05（`t_on = 2.0 s` / `t_off = 3.0 s`）需同时报告全体双刹中位数与显著提前中位数：双方都刹车（`both`，各 9 例）的**全体中位数**为 CV **0.4 s**、CTRV **0.5 s**；预测显著更早（`lead_time > 0.3 s`）子集的**中位数**为 CV **1.0 s**、CTRV **1.2 s**。另有 5 例为 `predictive_only`（仅预测系统在接触前触发有效刹车），11 例 `neither`。

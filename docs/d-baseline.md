# D 的 Evaluation Baseline（M4 · `pbs.eval` / `pbs.systems`）

> Owner：D（yj）· 2026-10-20 · 配合 [spec.md](spec.md) v3 §3 / §5 / §8 与 [interfaces.md](interfaces.md)。
> P 的 baseline 回答"系统怎么判断风险"；D 的 baseline 回答"**怎么公平地判定一个系统好不好**"。本文定义 D 的度量、数据表、实验与验收标准，不含新代码。

## 1. 职责边界

| 输入（来自谁） | D 做什么 | 输出（给谁） |
| --- | --- | --- |
| `Scenario` 集与参数网格（A）<br>`run_sim → SimLog`（C）<br>`SYSTEMS` 中的 assessor（P） | 跑实验矩阵 → 每次仿真一行 → groupby 出指标与图 | 报告 F1–F5（全组）、proposal Evidence 节、final demo 的结论数字 |

D **不**实现场景、预测或仿真逻辑；发现上游问题时提 issue 给对应 owner，不在 `pbs.eval` 里打补丁绕过。

## 2. 现状（`main` @ d3fe1ec，2026-10-21；未含 PR #3）

| 组件 | 文件 | 状态 | 说明 |
| --- | --- | --- | --- |
| `SYSTEMS` / `CORE` / `make_system` | `pbs/systems.py` | ✅ | `CORE = reactive, cv, ca, oracle`。CA / CTRV / Oracle 已实现；LR 仍是 L3 |
| `is_benign` | `eval/counterfactual.py` | ✅ | NullAssessor + NeverBrakePolicy 下不碰撞 ⇒ benign |
| `ade` / `fde` | `eval/metrics.py` | ✅ | P 的 exp00 已复用 |
| `evaluate(log, benign)` | `eval/metrics.py` | ✅ | collision / min_dist / brake_onset_t / false_brake / benign |
| `plot_timeline` | `eval/plots.py` | ✅ | ego v(t) + 刹车阴影，可多系统叠加 |
| `run_matrix` | `eval/sweep.py` | ✅ L2 | §4 schema；`is_benign` 按场景缓存，并与主实验共用 `sim_kwargs` |
| `tradeoff` / `select_operating_point` | `eval/sweep.py` | ✅ L2 | §5.1；工作点为 false-brake ≤ 5% 下碰撞率最低的阈值 |
| `failure_boundary` / `safe_area` / `rescue_rate` | `eval/sweep.py` | ✅ L2 | §5.2。主指标是 rescue rate，不是含 benign 格子的面积 |
| `plot_boundaries` | `eval/plots.py` | ✅ L2 | 碰撞格必画；碰撞格 ≥ 4 时才叠 0.5 等高线，避免两个孤立点被画成一块区域 |
| exp02 / exp03 / exp05 | `experiments/` | ✅ L2 `--quick` | 见 §9。exp01 归 P，exp04 仍是可砍的 L3 |
| 测试 | `tests/test_sweep.py`、`test_counterfactual.py`、`test_predictors.py` | ✅ | 全库 125 passed（2026-11-18，课程环境 `aie1903_ad`） |

## 3. 核心定义（proposal 与报告中写死）

| 概念 | 定义 | 备注 |
| --- | --- | --- |
| **Dangerous / Benign** | 同一场景用 `NeverBrakePolicy` 跑一次：碰撞 ⇒ dangerous，否则 benign | 与系统无关，每个场景只算一次并缓存 |
| **Collision rate** | dangerous 场景中仍碰撞的比例 | 分母**只含 dangerous**；benign 场景不可能因刹车而碰撞（agent log replay） |
| **False-brake rate** | benign 场景中出现过 brake 的比例 | 分母只含 benign |
| **Brake onset** | 第一次 action = brake 的时刻 | 从未刹车记为 `None` |
| **Lead time** | `onset(Reactive) − onset(Predictive)`，同一场景同一阈值 | 仅在两者都刹车时有定义；一方未刹车的场景单独计数，不丢弃 |
| **Failure boundary** | (gap_d, rel_v) 网格上，某系统在**固定工作点**下碰撞 / 不碰撞的分界 | 工作点见 §6-Q1 |
| **Rescue rate**（替代 safe_area 的主指标） | dangerous 格子中被该系统救下的比例 | 用 dangerous 格子作分母，避免 benign 格子把所有系统的面积一起抬高 |
| **ADE / FDE** | 预测与真值的平均 / 终点位移误差 | 与 P 共用实现 |

## 4. 实验矩阵数据表（`run_matrix` 输出 schema）

一次 `run_sim` = 一行；所有图都是这张表的 groupby / pivot。

| 列 | 类型 | 来源 |
| --- | --- | --- |
| `scenario_id` | str | `kind` + 参数哈希，保证可复现 |
| `kind` | str | `Scenario.kind` |
| `gap_d`, `rel_v`, …（展开 `params`） | float | A |
| `system` | str | `SYSTEMS` 的 key |
| `threshold` | float | `TTCThresholdPolicy.ttc_threshold` [s] |
| `latency` | float | `run_sim(latency=)` [s] |
| `benign` | bool | `is_benign`（按 `scenario_id` 缓存） |
| `collision`, `false_brake` | bool | `evaluate` |
| `min_dist` | float | `evaluate` [m] |
| `brake_onset_t` | float / NaN | `evaluate` [s] |
| `seed`, `git_rev` | str | 复现用 |

- 结果写 `outputs/logs/expXX.csv`；画图脚本只读 CSV，不重跑仿真。
- `is_benign` 必须与主实验使用**相同的 `duration` / `r_safe`**，否则 benign 标签与主实验不一致；建议 `run_matrix` 统一传 `sim_kwargs`。
- 规模估计：网格 15 × 17 = 255 场景 × 4 系统 × 5 阈值 ≈ 5 100 次仿真；`--quick` 用 5 × 5 粗网格 × 2 阈值，目标 < 1 min。

## 5. D 负责的实验

| 实验 | 问题 | 自变量 | 因变量 | 图 | 周 |
| --- | --- | --- | --- | --- | --- |
| exp05（与 P） | prediction 让刹车提前几秒？ | cut_in 场景集；Reactive vs CV / CTRV | lead time 分布 + 未刹车计数 | F2 | W8 |
| exp02 | 每个系统的安全–误报 trade-off？ | `CORE` × 阈值 0.5–3.0 s | collision rate vs false-brake rate，每系统一条曲线 | F3 | W8 → W11 |
| exp03 | prediction 把安全区扩大多少？ | `CORE` × (gap_d, rel_v) 网格，固定工作点 | 四条 boundary 叠加 + rescue rate；叠刹停距离 `d = v·t_react + v²/(2a)` | **F4** | W9 → W11 |
| exp04（可砍） | 感知延迟的代价？ | latency 0–300 ms × {Reactive, CV} | collision rate / rescue rate vs latency | F5 | W11 |

### 5.1 `tradeoff`
按 `(system, threshold)` 分组：`collision_rate = mean(collision | ¬benign)`，`false_brake_rate = mean(false_brake | benign)`。若某系统所有阈值下误报率都是 0，说明 benign 集不够"像危险"，回到 A 加样本（spec §1.5），而不是改指标。

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

## 7. 验收标准（对应 spec v3 里程碑）

| 里程碑 | D 的验收 |
| --- | --- |
| **M0（W7）** | `run_matrix` 能在 A 的 `cut_in` 上对 {reactive, cv} 跑出 CSV；每个场景的 `benign` 只算一次；单测覆盖 schema 列名 |
| **M1（W9）** | exp05 lead time 图 + exp02 trade-off 初版；benign 与 dangerous 均非空，误报率不恒为 0 |
| **M2（W11）** | exp03 四边界图定稿，报告两个差值；全部实验 `--quick` < 5 min；图与 CSV 一一对应 |
| **M3（W12）** | 报告"实验结果 / 局限"节；每个结论按 claim → evidence（图号）→ limitation 写 |

## 8. 给 proposal 用的 Evidence & Evaluation 草稿

> **Our final evidence will be** a four-way failure-boundary overlay (Reactive / CV / CA / Oracle) on the (gap_d, rel_v) plane for synthetic cut-ins, together with the collision-rate vs false-brake-rate trade-off and the cut-in lead-time distribution, all regenerated by one command from a cached results table.
>
> - **Will demonstrate**：在点质量、log replay、TTC 阈值刹车的设定下，prediction 救下了多少 dangerous 场景、提前多少秒察觉切入，以及预测误差与误报的代价。
> - **Will not demonstrate**：真实交通分布或真实车辆上的 AEB 性能；perception 误差（仅用 latency 近似）；转向避让。
> - **Fairness**：危险 / 良性由与系统无关的 counterfactual 判定；系统间在相同误报率下比较，而非仅在相同阈值下比较。

## 9. L2 quick 结果（2026-11-18）

实验网格在 `experiments/cut_in_grid.py`，不替换 A 的 `make_scenario`。完成换道的几何与 P 的 L1 fixture 相同；另外 6 个中止换道用来制造“看起来要进来、实际不撞”的 benign 样本，A 的 `gap_d > 0` 网格不含这组。日志比仿真长一个 3 s 预测时域，避免 Oracle 把轨迹末端当成停车。`--quick`：完成换道 5×5（gap 8–48 m，rel_v −4–12 m/s），仿真 15 s。三条命令合计约 30 s。

```bash
python experiments/exp02_threshold_sweep.py --quick
python experiments/exp03_failure_boundary.py --quick
python experiments/exp05_cut_in_lead_time.py --quick
```

完成换道 25 个场景里 12 个 dangerous、13 个 benign。exp02 另加 6 个中止换道（向本车道靠近后返回，最近距离仍大于 `r_safe`），否则完成换道上的误报率恒为 0：相对速度 ≤ 0 时没有系统会刹车。

在 false-brake ≤ 5% 的工作点上（这组粗网格里与固定 2.0 s 的 rescue 相同）：

| 系统 | rescue rate | 含义 |
| --- | ---: | --- |
| Reactive | 0.583 | |
| CV | 0.833 | rescue(CV) − rescue(Reactive) = **0.25**，prediction 的收益 |
| CA | 0.833 | 与 CV 相同；这段 quintic 上常加速度没有分开 |
| Oracle | 1.000 | rescue(Oracle) − rescue(CV) = **0.17**，预测误差的代价 |

exp02 在 2.0 s（含中止换道）：Reactive 碰撞率 0.42、误报 0；CV / CA 碰撞率 0.17、误报 0.11；Oracle 碰撞率 0、误报 0。Oracle 在 1.0 s 仍有 0.25 的碰撞率，阈值并不是免费的。

exp05 的 lead time 是双峰的，不能只报中位数。双方都刹车时中位数是 −0.1 s：切入完成后只剩纵向跟车，Reactive 的连续 TTC 比离散预测早大约一个仿真步。CV 真正更早的那些场景中位数是 0.9 s（CTRV 1.25 s），而且就是 CV 躲开、Reactive 撞上的格子。另有 2 个场景只有预测系统刹车，但刹得太晚，仍然碰撞。

# 场景设计（M1 · `pbs.data`）— Owner: A

> 本页的场景基线图和历史统计按圆形判定复现。将同一生成器用于 L2 闭环实验时，按 [L2 基线规范](l2-baseline.md)重新判断碰撞及良性场景。

> 一句话：**我们不用真实交通分布，而是用一组参数可控、物理上站得住脚的合成场景；每个场景都要能被问一句"完全不刹车会不会撞"，因为这是后面四条 failure boundary 的共同分母。**
>
> 代码：[`src/pbs/data/scenarios.py`](../src/pbs/data/scenarios.py) · [`src/pbs/data/loader.py`](../src/pbs/data/loader.py)
> 验证脚本：[`experiments/exp00_scenarios.py`](../experiments/exp00_scenarios.py)
> 产物：[`outputs/figures/exp00_*.png`](../outputs/figures)、[`outputs/logs/exp00_scenarios.csv`](../outputs/logs/exp00_scenarios.csv)

---

## 1. 我们在造什么，不造什么

| | 内容 |
| --- | --- |
| **造** | 三类冲突的运动学骨架：切入（`cut_in`）、前车制动（`lead_brake`）、横穿（`crossing`）。参数可网格化，用于画 failure boundary。 |
| **不造** | 真实交通统计分布、多车交互、传感器噪声、车辆动力学、3D。 |
| **诚实说法** | "在给定运动学参数范围内，prediction 改变了什么"——**不是**"复现了真实交通"。这一条必须写进 report 的 limitations。 |

之所以必须自己造：boundary 图需要在 (gap_d, rel_v) 平面上做**可控**扫描；真实数据只有一段低速跟车片段（`docs/data_audit.md`），数量与覆盖度都不够。真实数据的角色是**定性案例 + loader 验证**（W8 的 `exp06`），不是主图。

## 2. 坐标、单位与几何约定

```
y
↑                       车道中心线：y = +3.5（相邻车道）、y = 0（本车道）、y = −3.5
│   ┌────────────┐
│   │            │  ← 车道边界：y = ±1.75（= ±lane_width/2）
│   │   cutter   │
│   │     ↘      │
├───┼────────────┼──────────────►  x      ego 从 x = 0 出发，朝 +x
│   │   ego →    │
└───┴────────────┴─────────────
```

- 世界坐标，单位 m / s / m·s⁻¹ / rad；`dt = 0.1 s`（冻结接口）。
- `lane_width = 3.5 m`，`|横向偏移| < 1.75 m` ⇒ "在本车道内"——**这正是 Reactive baseline 用来过滤目标的判据**（`Scenario.lane_width` 是它唯一的输入）。
- `r_safe = 2.5 m` 由 M3（C）拥有，本模块**不 import `pbs.sim`**（M1 必须在数据流的更下层）。
- 每类场景只有 **1 个 agent**，`agent_id` 唯一（`"cutter"` / `"lead"` / `"crosser"`），OraclePredictor 依赖它。
- agent 轨迹覆盖**整个** scenario duration，所以不会被 `state_at` 的端点外插冻结。
- ego **不是** replay：它由 M3 仿真、可以响应刹车；只有 agent 是 log replay。

## 3. 三类场景

| `kind` | 参数 | 冲突方向 | 物理含义 |
| --- | --- | --- | --- |
| `cut_in` | `gap_d`, `rel_v`, `cut_in_t`, `lc_T` | 横向 | 相邻车道车辆打灯切入本车道 |
| `lead_brake` | `init_gap`, `decel`, `t_brake`, `rel_v` | 纵向 | 本车道前车减速/急刹 |
| `crossing` | `gap_d`, `arrival_t`, `lateral_v` | 横向 | 行人/车辆横穿本车路径 |

### 3.1 `cut_in`

```text
v_x(t) = (v_ego − rel_v) + a_cutter · τ,        τ = clip(t − cut_in_t, 0, accel_T)
x_cut(t) = gap_d + (v_ego − rel_v)·t + ½·a_cutter·τ²
           + a_cutter·accel_T·max(t − cut_in_t − accel_T, 0)      # 坡段结束后保持增速
y_cut(t) = λ(t − cut_in_t;  lc_T)               # 横向：quintic，从 ±3.5 m 到 0
```

- `gap_d`：**t = 0 时**的中心纵向间距；`rel_v = v_ego − v_cutter`，**> 0 表示本车更快（在追近）**。
- `cut_in_t`：开始换道的时刻；`lc_T`：换道持续时长。
- `a_cutter`：为 CA 新增的**纵向加速度**，默认 `0`（= 原来的全程匀速，向后兼容）；`accel_T` 默认等于 `lc_T`，即"边换道边加速/减速"。`|a_cutter| ≤ 3 m/s²`，且坡段结束时速度不得为负（否则报 `ValueError`）。
- 换道完成后 `y = 0`、纵向按上面的 `v_x` 前进 ⇒ 若 `rel_v > 0`，间距线性缩小。

> **为什么加 `a_cutter`**：没有它时 cut-in 的纵向运动是纯匀速，**CA（constant acceleration）和 CV 在这一维上完全一样**，exp01 会得到"CA ≈ CV"的假结论。只有 `a_cutter ≠ 0` 时这一族才真正区分两个预测器（见 §6.1）。

### 3.2 `lead_brake`

```text
v_lead(t) = v_ego − rel_v                         (t ≤ t_brake)
          = max(0, v_lead − decel·(t − t_brake))  (t > t_brake)
x_lead(t) = init_gap + ∫₀ᵗ v_lead
```

- `decel` 施加到停住为止（停车后不再倒车），所以持续时间足够长时前车会**完全停住**。
- 若 `rel_v = 0`（默认），刹车前两车同速、间距不变 —— 间距的收窄**全部**来自前车减速。

### 3.3 `crossing`

```text
x_cross(t) = gap_d                    # 在固定纵向位置横穿
y_cross(t) = lateral_v · (arrival_t − t)
```

- `arrival_t`：横穿者到达**本车道中心线**（y = 0）的时刻；t = 0 时它在侧面 `lateral_v·arrival_t` 处。
- 匀速横穿，单位横向量 `lateral_v`（行人 1.5 m/s 量级）。
- 危险窗口的宽度约 `± r_safe / lateral_v ≈ ±1.67 s`：本车到达横穿点（`gap_d / v_ego`）与 `arrival_t` 相差不到这个量级时才可能撞上 —— 这就是 danger map 里那条"对角带"的来源。

## 4. quintic 换道与"物理合理性"过滤

最小 jerk（quintic）横向剖面：

```text
y(τ) = y₀ + Δy·(10τ³ − 15τ⁴ + 6τ⁵),      τ = (t − t₀)/T ∈ [0, 1]
```

它满足两端 **速度、加速度都为 0**（C² 连续，乘客不会感到"突然一掰"）。对 τ 求二阶导：

```text
d²y/dt² = (Δy/T²)·60τ(1 − τ)(1 − 2τ)
```

在 `τ ≈ 0.2113`（以及镜像的 0.7887）取到峰值：

```text
a_lat,peak = (10/√3)·|Δy|/T² ≈ 5.7735·|Δy|/T²
```

| Δy | T | a_lat,peak | 说明 |
| --- | --- | --- | --- |
| 3.5 m | 2.0 s | 5.05 m/s² | 太猛，被拒 |
| 3.5 m | 2.60 s | 3.00 m/s² | 边界（`MAX_LAT_ACC`） |
| 3.5 m | 3.0 s | 2.25 m/s² | 默认，可接受 |
| 3.5 m | 4.0 s | 1.26 m/s² | 舒适 |

**过滤规则**：`peak > max_lat_acc` 的组合**不做裁剪**，而是
- `make_scenario(..., strict=True)`（默认）→ 抛 `ValueError`，错误信息里带上实际值与最小可行 `lc_T`；
- `scenario_set(..., strict=False)` → 记进 `ScenarioSet.rejected`，可统计"网格里百分之多少物理上不可接受"。

阈值不是拍出来的：`docs/data_audit.md` §4 要求用**文献区间 + 敏感度分析**替代单一阈值，所以脚本对 2.0 / 3.0 / 4.5 m/s² 各跑一遍：

| `max_lat_acc` | 接受的最短 `lc_T` | 在 `lc_T ∈ {1.5, 2, 2.5, 3, 4, 5}` 中接受几个 |
| --- | --- | --- |
| 2.0 m/s² | 3.18 s | 2 / 6 |
| **3.0 m/s²** | **2.60 s** | **3 / 6** |
| 4.5 m/s² | 2.12 s | 4 / 6 |

> 结论用法：主结果用 3.0 m/s²；若把阈值放到 4.5，网格里可行的换道更短、切入更"凶"，边界会移动——**移动的方向和幅度**就是报告里要写的那段敏感度。

## 5. 场景集（网格）与命名场景

默认网格在 `scenarios.py::DEFAULT_GRID`，**全部通过过滤**（`rejected = 0`，所以 D 的 `run_matrix` 直接调用不会踩 `ValueError`）：

| kind | 扫描的参数 | 组合数 |
| --- | --- | --- |
| `cut_in` | `gap_d ∈ {10, 20, 30, 40, 60}`，`rel_v ∈ {−2, 0, 1, 2, 4, 6}`，`lc_T ∈ {3, 4}` | 60 |
| `lead_brake` | `init_gap ∈ {20, 35, 50, 80, 100}`，`decel ∈ {0.3, 1, 3, 6}` | 20 |
| `crossing` | `gap_d ∈ {15, 25, 35, 50}`，`arrival_t ∈ {1, 2.5, 4, 6}` | 16 |

另有一小片 **CA 切片**（`CA_GRID`，见 §6.1）：`cut_in` 的 `gap_d ∈ {20, 30}`、`rel_v ∈ {0, 2}`、`a_cutter ∈ {−2, 0, +2}` = 12 个场景。它**不并入**默认网格，所以 D 的 96 场景矩阵和所有文档分母都不变。

命名场景（`CANONICAL`）给文档、图、测试共用，**每个家族各一 dangerous + 至少一 benign**：

| 名字 | 参数 | 首次接触 t | margin | 判定 |
| --- | --- | --- | --- | --- |
| `cut_in_danger` | gap 25 m, rel_v 3 m/s, lc_T 3 s | 7.51 s | −2.50 m | dangerous |
| `cut_in_benign_equal_speed` | gap 25 m, rel_v 0 | — | +22.50 m | benign |
| `cut_in_benign_fast_cutter` | gap 12 m, rel_v −1.5 | — | +10.00 m | benign（**看着危险，其实安全**） |
| `lead_brake_danger` | gap 30 m, decel 4 m/s² | 4.71 s | −2.50 m | dangerous |
| `lead_brake_benign` | gap 100 m, decel 0.3 m/s² | — | +68.10 m | benign（**看着危险，其实安全**） |
| `crossing_danger` | x 35 m, arrival 2.3 s, 1.5 m/s | 2.17 s | −2.45 m | dangerous |
| `crossing_benign` | x 35 m, arrival 6.0 s | — | +2.97 m | benign |

> `margin = min_dist − r_safe`（0.01 s 扫掠）。碰撞后 ego 会**穿过** agent（不模拟冲击），所以 dangerous 的 `margin` 可以到 −2.50 m（= −r_safe），表示"轨迹直接穿过目标"；benign 的 `margin` 就是真正的最小安全余量。**首次接触时刻**看第二列。

## 6. benign / dangerous：为什么每个家族都必须有 benign

**定义（spec §1.5）**：同一场景用 `NeverBrakePolicy` + `NullAssessor` 跑一遍——
不碰撞 ⇒ 该场景 **benign**；benign 场景里**任何**一次刹车都是 counterfactual 的**误刹**（false brake）。

- 没有 benign 的场景集 ⇒ 误报率恒等于 0 ⇒ exp02 的 **碰撞率 vs 误报率 trade-off 曲线画不出来**，只能退化成一条碰撞率曲线。
- 所以"看着危险但实际安全"的样本（`rel_v ≤ 0` 的切入、很远很轻的前车减速、行人早早过完马路）是**功能性需求**，不是凑数。

默认网格的实测构成（`python experiments/exp00_scenarios.py` 生成）：

| kind | 场景数 | dangerous | benign |
| --- | --- | --- | --- |
| `cut_in` | 60 | 27 | 33 |
| `lead_brake` | 20 | 16 | 4 |
| `crossing` | 16 | 9 | 7 |
| **合计** | **96** | **52** | **44** |

`tests/test_data.py::test_every_family_contains_both_dangerous_and_benign_samples` 对每个家族断言"既有既无"，防止以后改网格时把某一类改成全 dangerous。

> **数字变过一次（2026-11-18）**：合并 C 的 `collision_dt = 0.01 s` 子步扫掠后，`crossing` 从 8/8 变成 9/7（总数 51/45 → 52/44）。根因是 `crossing(gap_d=35, arrival_t=4.0)` 的真实最小中心距 **2.4876 m**，只比 `r_safe = 2.5 m` 少 **1.24 cm**：0.1 s 网格看不见，0.01 s 扫掠看见了。这类点一格里只有一个，CSV 的 `marginal` 列会标出来；报告里要么单列"刀刃场景"，要么从主图剔除，不能假装边界是干净的。
>
> 同时 `min_dist` 的口径也统一了：CSV 里 `min_dist` 是 0.01 s 扫掠值（`collision` 就按它判），`min_dist_coarse` 才是旧的 0.1 s 网格值。两者在刀刃点上会给出相反的结论——这正是之前"CSV 说撞了，但 min_dist 还是 2.60 > 2.5"的原因。

### 6.1 为 CA 准备的切片（`CA_GRID`）

```python
from pbs.data import ca_scenario_set, prediction_slices

built = ca_scenario_set()          # 12 个 cut-in：a_cutter ∈ {−2, 0, +2}
slices = prediction_slices(canonical("cut_in_danger"), history=1.0, horizon=2.0)
slices[0].history.t[-1]            # == slices[0].t0，history 里没有未来
slices[0].truth                    # (20, 2) 真值未来，k = 1..20
slices[0].truth_accel              # (20, 2) 解析加速度真值（仅用于验证）
```

| 你拿到什么 | 在哪 | 给谁用 |
| --- | --- | --- |
| `make_scenario("cut_in", a_cutter=±2)` | `scenarios.py` | P 的 CA 需要"纵向非匀速"的切入 |
| `Trajectory.accel`（N×2 解析真值） | `types.py` + 三个 builder | P 验证"估出来的 a"对不对；**预测器不许在测试时读它** |
| `prediction_slices(...)` | `data/benchmark.py` | exp01 的固定切片：同一 `t0`、同一 horizon，所有 predictor 同题 |
| `ca_scenario_set()` | `scenarios.py` | 12 个场景，6 benign / 6 dangerous（−2 全危险、0 各半、+2 全安全） |

三条给 P 的硬约定（详见 [CA_scenario_support.md](CA_scenario_support.md)）：

1. 加速度从**位置**的窗口最小二乘估，不要对 `v` 列差分（课程数据速度列分片常数，两点差分能造出 ±2 m/s²）。
2. 预测到"前车已停"之后要 clamp，否则 CA 会让前车倒车。
3. `exp01` 用 `prediction_slices` 的合成切片当主数据；`load_toy()` 那段 12.9 s 没有换道、加速度不可辨识，只能当定性案例。

## 7. 怎么读 danger map（`outputs/figures/exp00_danger_map.png`）

- 标记 = 网格点：红叉 = 不刹车会撞，空心绿圈 = benign。
- `cut_in` 面板里的虚线是**解析参照**：假设 target 已经在本车道中心，则接触时刻 `t* = (gap_d − r_safe)/rel_v`；`t* = duration` 给出 `gap_d = r_safe + rel_v·duration`。**仿真得到的危险区比这条线略大**，因为接触发生时 target 还在换道中途（横向还没归零）。所以：**边界必须靠仿真算，不能靠公式替代**；公式只用来 sanity check（"边界离解析线大约一个网格格距"是正常的）。
- `crossing` 面板的对角线是"本车与行人同时到达"；危险带宽度 ≈ `r_safe / lateral_v`。
- `lead_brake` 的 benign 区很小（只有"远 + 轻"），这与直觉一致：纯纵向冲突是 reactive AEB 的主场，prediction 在这里能加的边际收益小 —— 这本身就是一个可写进报告的现象。

## 8. A 的 Q&A（spec §4 给的三个必答题）

**Q1：quintic 换道的峰值横向加速度是多少？**

`(10/√3)·|Δy|/T²`。默认 `Δy = 3.5 m`、`T = 3 s` → **2.25 m/s²**；我们的上限是 3.0 m/s²，对应最短 `T = 2.60 s`。并给出 2.0/3.0/4.5 三档敏感度（表格见 §4）。

**Q2：为什么场景集必须有 benign 样本？**

因为误刹用 counterfactual 定义：benign 场景里"不刹车也不会撞"，此时任何刹车都是误报。场景集若全是 dangerous，误报率恒为 0，trade-off 曲线无法画出，exp02/exp03 的主结论就少了横轴。我们的网格 **46%** 是 benign（52 dangerous / 44 benign），且三类各含"看着危险其实安全"的样本。

**Q3：为什么不能用日志里的 ego 轨迹？**

因为 ego 必须**能被刹住**：我们的问题是"AEB 决策改变结果"，ego 必须闭环仿真（`run_sim` 每步按决策更新速度）；只有周围 agent 才按日志 replay（它们不响应 ego 的刹车）。用日志 ego 轨迹等于假定了"无论怎么决策，本车都照原样开"，那么碰撞与最小间距就和策略无关，整个项目的因果链断掉。另外课程数据的 ego 速度列有非物理抖动（`docs/data_audit.md` §2.3），也不适合直接回放。

## 9. 复现与产物

```bash
python experiments/exp00_scenarios.py         # 生成下面 5 个文件（约 10 s）
pytest tests/test_data.py -q                  # M1 的单元测试
```

| 产物 | 内容 |
| --- | --- |
| `outputs/figures/exp00_scenario_geometry.png` | 三类冲突各一个 dangerous 样本的几何图（含接触点） |
| `outputs/figures/exp00_lateral_accel.png` | quintic 的 y / ẏ / ÿ 剖面 + 横向加速度过滤曲线 |
| `outputs/figures/exp00_danger_map.png` | 三类场景的 danger map（no-braking baseline） |
| `outputs/logs/exp00_scenarios.csv` | 96 行：参数、是否碰撞、`min_dist`（0.01 s 扫掠）、`margin`、`marginal`、`contact_t`、解析接触时刻 |
| `outputs/logs/exp00_summary.md` | 上表的自动汇总（本文档的数字都来自它） |

## 10. 局限（写进 report）

1. **合成、非校准**：cut-in 家族没有用真实换道数据校准（数据里没有一次换道，见 data_audit §4）；物理合理性靠文献界 + 敏感度论证。
2. **无车辆动力学**：点质量 + 圆形 footprint（M3），无转向、无轮胎、无刹车距离的非线性。
3. **单 agent、无交互**：每类场景只有 1 个目标，agent 不响应 ego。
4. **2D、无 perception**：直接用轨迹级数据，不涉及检测/跟踪误差。
5. **`duration = 15 s` 是判据的一部分**：`benign` 的含义是"15 s 内不撞"，不是"永不撞"。长尾收敛很慢的参数组合（大 gap、小 rel_v）会被归为 benign —— 这是**定义**，报告要写清楚，D 画边界时也必须用同一个 duration。
6. **边界是"分辨率相关"的**：`crossing(35, 4.0)` 的 margin 只有 −1.2 cm，0.1 s 网格下会被判成 benign。我们保留它但标成 `marginal`；报告里必须写"边界位置有 ±网格步长的数值不确定性"，不能把边界当成精确曲线。`CA_GRID` 是额外的一小片场景，不改变默认网格的 52/44 分母。

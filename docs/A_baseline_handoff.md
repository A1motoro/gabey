# A 的 baseline：做完了什么、东西在哪、怎么理解

> 这份文档是给你（A）一个人的入口。**按 §3 的顺序读，20 分钟能全部过一遍。**
> 结论先放这儿：**你的 M1 场景模块（L1 + L2 全部）已经实现并通过测试；跑一条命令就能得到 3 张图 + 2 个数据文件，全部在 `outputs/` 下。**

---

## 1. 先说清一件事：你的 "baseline" 指哪个

`docs/spec.md` 里 "baseline" 有两个意思，别混：

| 说法 | 指什么 | 谁的 | 状态 |
| --- | --- | --- | --- |
| **三层系统 baseline** | Reactive → Predictive(CV/CA/CTRV/LR) → Oracle，四条 failure boundary | P（预测/风险）+ D（实验） | P/D 的 `predictor`、`assess_risk` 还没实现（`NOT IMPLEMENTED`，测试里是 skip） |
| **你的场景 baseline（本次交付）** | M1 `pbs.data`：参数化合成场景 + 物理合理性过滤 + **no-braking 参照**（哪些场景"不刹车会撞"） | **A（你）** | ✅ 完成 |

为什么这两件事是一体的：四条边界都要画在同一个 (gap_d, rel_v) 平面上、都要区分"该刹"与"误刹"。**这个平面和这个分母就是你的模块提供的。** 没有它，P 和 D 什么都画不出来。

所以"你的 baseline 做出来的东西"= **一组可复现的场景 + 每个场景的 no-braking 判定 + 三张诊断图**。它们不是最终图，而是最终图的底板。

---

## 2. 交付清单：新增/修改了哪些文件

### 2.1 代码（你的模块）

| 文件 | 状态 | 内容 |
| --- | --- | --- |
| [`src/pbs/data/scenarios.py`](../src/pbs/data/scenarios.py) | **新写**（原为 `NotImplementedError`） | quintic 换道、3 类场景构造、参数网格、物理过滤、命名场景 |
| [`src/pbs/data/loader.py`](../src/pbs/data/loader.py) | **新写**（原为 `NotImplementedError`） | `load_toy()`（排除 ego）、`load_toy_ego()`、路径容错 |
| [`src/pbs/data/__init__.py`](../src/pbs/data/__init__.py) | 新写 | 统一导出（`from pbs.data import ...`） |

### 2.2 验证脚本（产生你能看见的东西）

| 文件 | 内容 |
| --- | --- |
| [`experiments/exp00_scenarios.py`](../experiments/exp00_scenarios.py) | 建 96 个场景 → 每个跑一次"不刹车" → 出 3 图 + CSV + 汇总 MD |

### 2.3 测试（你的模块的验收标准）

| 文件 | 内容 |
| --- | --- |
| [`tests/test_data.py`](../tests/test_data.py) | 31 个测试：quintic 端点/C² 连续/峰值公式、三类场景几何、过滤拒绝、网格计数、每类必须既有 dangerous 又有 benign、loader 正确性 |

### 2.4 文档

| 文件 | 内容 |
| --- | --- |
| `docs/scenarios.md` | **技术说明书**：公式推导、参数表、过滤阈值、danger map 读法、A 的三个 Q&A、局限 |
| `docs/A_baseline_handoff.md` | 本文：位置 + 理解路线 |
| `docs/interfaces.md` | 补上 M1 的状态与新增函数（O = 已实现） |
| `docs/contributions.md` | 填了 A 的 W4 一行 |
| `README.md` | 实验表加了 `exp00` 一行 |

### 2.5 产物（每次运行重新生成，不进 git）

```
outputs/figures/exp00_scenario_geometry.png   三类冲突各一例的几何图（含"接触点"）
outputs/figures/exp00_lateral_accel.png       quintic 的 y/ẏ/ÿ + 横向加速度过滤曲线
outputs/figures/exp00_danger_map.png          96 个场景的 danger map（红叉=会撞，绿圈=安全）
outputs/logs/exp00_scenarios.csv              96 行明细：参数 + 是否碰撞 + 最小距离
outputs/logs/exp00_summary.md                 汇总表（本文档所有数字的来源）
```

---

## 3. 5 分钟跑起来

在仓库根目录（`...\AIE1903_Project\gabey`）：

```powershell
python experiments\exp00_scenarios.py      # 约 10 秒，重写上面 5 个产物
python -m pytest tests -q                  # 38 passed, 6 skipped（6 个是 P 的 TODO）
```

预期输出：

```
scenarios built : 96  (rejected 0)
  cut_in       60 scenarios |  27 dangerous |  33 benign
  lead_brake   20 scenarios |  16 dangerous |   4 benign
  crossing     16 scenarios |   8 dangerous |   8 benign
rejection sensitivity (max_lat_acc -> accepted lc_T):
  2 m/s^2 -> 2/6  (min lc_T 3.18 s)
  3 m/s^2 -> 3/6  (min lc_T 2.60 s)
  4.5 m/s^2 -> 4/6  (min lc_T 2.12 s)
```

> 环境说明：你的部分只需要 `numpy` + `matplotlib`（都已装）。`pandas` 只有 D 的实验矩阵需要，当前没装；`pytest` 已装。
> 如果 `import pbs` 报错，用 `pip install -e .`，或直接跑脚本（脚本内部已把 `src/` 加进路径）。

---

## 4. 三张图怎么读（每张一句话）

| 图 | 一句话 | 你要能指出来的细节 |
| --- | --- | --- |
| `exp00_scenario_geometry.png` | 三类冲突"长什么样" | 左：cutter 从 y=3.5 换到 y=0，红★是**不刹车时首次进入 r_safe=2.5 m** 的时刻（t=7.60 s，中心距 2.20 m）。中：前车在 t=1 s 开始 4 m/s² 刹停，红★=t 4.80 s 追尾。右：行人在 x=35 m 处横穿，红★=t 2.20 s 撞上。 |
| `exp00_lateral_accel.png` | 为什么"换道时长"不能随便取 | 左：quintic 的归一化 y/ẏ/ÿ；**ÿ 峰值 5.77**（=10/√3），在 τ=0.211。右：峰值横向加速度 ∝ 1/T²；3 m/s² 这条虚线把平面切成"可接受/太猛"，对应 `T_min=2.60 s`。 |
| `exp00_danger_map.png` | 哪些参数组合"不刹车会撞" | 红叉=危险、绿圈=benign。左图虚线 = "target 已在本车道"的解析边界；实测危险区**比它略大**（因为接触时 target 还在换道中途）。中图 benign 区很小（纯纵向冲突，reactive AEB 的主场）。右图危险带在"同时到达"对角线附近。 |

> 这张 `exp00_danger_map.png` 就是 EX03 那张"四边界叠加图"的**底板**：D 会把每个网格点换成"Reactive / CV / CA / Oracle 是否碰撞"，叠加在同一个平面上。你的红叉/绿圈是最底下的那一层。

---

## 5. 它在整条流水线里的位置（给 P / D 的接口）

```
你的模块 M1                 P 的模块 M2/Risk              C 的模块 M3              D 的模块 M4
─────────────               ─────────────────             ─────────────           ──────────────
make_scenario(kind, ...)    ConstantVelocity.predict()    run_sim(...)            is_benign(sc)
  └─ Scenario ───────────►   Reactive/PredictiveAssessor   ego 闭环 + 碰撞判定  ──►  evaluate / run_matrix
       .agents[0]              .assess(ctx, history)         .collision               trade-off / boundary
       .params                 └─ RiskEstimate
```

**四个必须知道的约定**（写代码的 P/D 会用错的地方）：

1. **`run_sim` 的 `duration` 必须传 `scenario.params["duration"]`**（我们的是 15 s，而 `run_sim` 默认 10 s）。不传的话，部分 dangerous 场景会被误判成 benign。
2. `Scenario.params` 里存着**全部解析后的参数**（含 `gap_d`、`rel_v`、`duration`、`peak_lat_acc`），D 建 DataFrame 时直接 `{**sc.params, "kind": sc.kind}` 就是一行。
3. `lane_width = 3.5` 同时是"本车道中心线间距"和**Reactive baseline 的过滤阈值**（`|偏移| < 1.75` 才算"在本车道"）。
4. `agent_id` 唯一且稳定（`"cutter"` / `"lead"` / `"crosser"`），`OraclePredictor` 靠它取真实未来。

调用示例：

```python
from pbs.data import make_scenario, scenario_set, canonical

sc = make_scenario("cut_in", gap_d=25.0, rel_v=3.0, cut_in_t=0.5, lc_T=3.0)
log = run_sim(sc, assessor, policy, duration=sc.params["duration"])   # ← duration 别丢

built = scenario_set()                 # 96 个场景，全部通过物理过滤；built.by_kind() 分类
sc = canonical("cut_in_benign_fast_cutter")   # 文档/图里用的命名场景

# 物理上不可接受的组合不会被悄悄改掉：
make_scenario("cut_in", gap_d=25, rel_v=2, lc_T=1.5)   # ValueError: 峰值 8.98 > 3.0
```

---

## 6. 四个可以直接写进报告的结论（都有证据文件）

| 结论 | 证据 |
| --- | --- |
| 场景集 47% 是 benign；三类各含"看着危险、其实安全"的样本，因此误报率的分母非空 | `outputs/logs/exp00_summary.md` §2、§4 |
| 换道物理约束：`a_peak=(10/√3)|Δy|/T²`，3.5 m 车道下 3 s 换道 = 2.25 m/s²，最短 2.60 s | `exp00_lateral_accel.png`、`docs/scenarios.md` §4 |
| 阈值 2.0/3.0/4.5 m/s² 下可接受的换道时长比例 2/6、3/6、4/6 —— 结论的敏感度可量化 | 脚本 stdout、`exp00_summary.md` §3 |
| cut-in 的危险区比"target 已归位"的解析边界略大（接触发生在换道中途），所以边界必须仿真 | `exp00_danger_map.png` 左图 |

---

## 7. 我替你做的默认选择（你可能想改，改之前先看测试）

| 选择 | 默认值 | 在哪改 | 为什么这么定 |
| --- | --- | --- | --- |
| 本车速度 | `v_ego = 15 m/s`（54 km/h） | `scenarios.py::V_EGO` | 城市工况；边界图固定它、只扫 (gap_d, rel_v) |
| 场景时长 | `duration = 15 s` | `DURATION` 或每次传入 | 覆盖 60 m/6 m/s 的最慢危险组合（9.6 s） |
| 换道横向位移 | `= lane_width = 3.5 m`（相邻车道中心 → 本车道中心） | `_cut_in(lane_width=...)` | 与车道定义一致；改它就等于改车道宽 |
| 切入来自左侧 | `from_left=True` | `_cut_in(from_left=False)` | 左右对称，选一侧即可 |
| 横向加速度上限 | `MAX_LAT_ACC = 3.0 m/s²` | 常量 / `max_lat_acc=` | 舒适~激进的中间值，配 2.0/4.5 敏感度 |
| 横穿者类型 | `agent_type="pedestrian"`，1.5 m/s | `_crossing(lateral_v=...)` | 行人的典型步速 |
| 默认网格 | 见 `DEFAULT_GRID` | 该字典 | 恰好每个组合都通过过滤，D 直接可用 |

**如果你改了 `DEFAULT_GRID` 或常量**：`pytest tests/test_data.py -q` 会立刻告诉你有没有破坏"每类都有 benign + dangerous"这条硬约束。

---

## 8. 这部分之外、还没有的东西（别以为是 bug）

| 缺什么 | 谁的 | 影响 |
| --- | --- | --- |
| 5 个 predictor（CV/CA/CTRV/LR/Oracle） | P | `experiments/exp01/exp05` 现在还跑不了 |
| `assess_risk`、`ReactiveAssessor` | P | 四条边界里除了你的 no-braking 参照，都还画不出来 |
| `run_matrix`、`tradeoff`、`failure_boundary`、`plot_boundaries` | D | 四边界叠加图 |
| `load_nuscenes_tracks` | A（你，L3 可选） | 保持 `NotImplementedError`，按 spec §1.3 可随时裁剪 |

你的部分**不依赖**上面任何一个 —— 这也是为什么你的 baseline 现在就能自证：`is_benign` / `run_sim` 只用到 C 已完成的骨架。

---

## 9. 自检清单（老师随时可能抽一段代码让你解释）

| 会被问 | 你要能说清 | 对应测试 |
| --- | --- | --- |
| quintic 为什么用 5 次多项式？ | 6 个边界条件（起终点位置/速度/加速度）恰好定 6 个系数；两端加速度为 0，乘客不会"被掰一下" | `test_quintic_is_c2_at_the_ends` |
| 峰值横向加速度怎么来的？ | 二阶导 `60τ(1−τ)(1−2τ)`，极值在 τ=0.211 ⇒ 5.7735·Δy/T² | `test_peak_lateral_acc_matches_the_sampled_profile` |
| 为什么超限要报错而不是截断？ | 截断会让报告里的"换道时长"和实际轨迹不一致，等于偷偷改了实验条件；报错+计数才能写"网格里 x% 不可行" | `test_cut_in_rejects_physics_...`、`test_strict_mode_...` |
| 为什么必须有 benign 场景？ | 误刹 = benign 场景中的刹车；没有 benign，误报率恒 0，trade-off 没有横轴 | `test_every_family_contains_both_...` |
| 为什么 ego 不用日志轨迹？ | ego 必须闭环、能被刹住；agent 才 replay（不响应 ego） | 见 `docs/scenarios.md` §8 Q3 |
| `lane_width` 有什么用？ | 既是几何（相邻车道中心距），也是 Reactive 的"是否在本车道"判据（±1.75 m） | `test_cut_in_geometry_...` |

---

## 10. 术语小抄

| 词 | 一句话 |
| --- | --- |
| **TTC** | time-to-collision：预测时域内首次距离小于 `r_safe` 的时间；无冲突 = ∞ |
| **Reactive** | 无预测：只看**已在本车道**的目标，纵向 `range/range-rate` |
| **Predictive** | 外推所有目标的 2D 运动（含横向），因此**切入还没完成**就能察觉 |
| **lead time** | Reactive 与 Predictive 首次报警的时间差 = "prediction 提前了多少秒" |
| **benign** | 同一场景不刹车也不会撞 ⇒ 其中的任何刹车都是误报 |
| **false brake** | benign 场景里发生的刹车（counterfactual 定义） |
| **danger map** | 在参数平面上标出"不刹车会撞 / 不会撞"的图（本模块的产物） |
| **failure boundary** | 换成真实系统后，同一平面上"该系统的安全区边界" |

---

## 11. 你的下一个 30 分钟

1. 跑 §3 的两条命令，确认产物时间戳变了。
2. 打开 `outputs/figures/exp00_danger_map.png`，用 §4 的读法自己复述一遍三块图。
3. 打开 `docs/scenarios.md` §8，把三个 Q&A 各讲一遍（口头，30 秒一题）。
4. 填 `README.md` 团队表里的姓名（现在是 `_TBD_`），并在 `docs/contributions.md` 里把 A 的贡献写成"具体产出"。
5. 如果要开始联调：把 §5 的调用示例发给 P 和 D，特别是**"`duration` 必须传 `sc.params['duration']`"** 这一条。

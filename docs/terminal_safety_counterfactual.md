# L3 反事实良性判据升级：物理末端安全检查与三分类规范

> **维护者**：D（yj · `pbs.eval.counterfactual` / `metrics` / `sweep`） · **协同校验**：C（前背痛 · `pbs.sim`）& A（Persistent · `pbs.data`）  
> **所属阶段**：L3 评估口径升级（严格遵循「只加不改，向后兼容 L2」原则，L2 既有 348 项测试零回归）

---

## 1. 问题背景：15 s 有限时域截断导致的「伪良性（False Benign）」

在 L1/L2 基线中，反事实良性判定 [src/pbs/eval/counterfactual.py](../src/pbs/eval/counterfactual.py) 采用单窗口二分类定义：
- 在 `[0, duration]`（默认 `15.0 s`）内运行 `NeverBrakePolicy`；
- 未发生碰撞即判为 `benign = True`，否则为 `dangerous`；
- 在 [src/pbs/eval/metrics.py](../src/pbs/eval/metrics.py) 中，凡 `benign == True` 场景下触发的任意制动均记为 `false_brake`（误刹）。

该定义存在明显的**有限时域截断盲区（Horizon Truncation Blind Spot）**：若不刹车在 `15.0 s` 内尚未撞上，但在第 `15.1 s ~ 19.0 s` 即将发生碰撞（甚至在第 `15.0 s` 末端已进入物理不可挽救状态），当前口径仍会将其标为 `benign`。这会导致：
- 在第 13–15 秒正确识别逼近风险并提前刹车的系统反而被记为 **false brake（误刹惩罚）**；
- 拖延至窗口结束仍不刹车的保守策略反而获得「无碰撞 + 无误刹」的虚假高分。

### 1.1 在 `DEFAULT_GRID`（96 场景，`duration = 15.0 s`）上的实测拆解

对 `DEFAULT_GRID` 全部 96 个场景在 L2 矩形几何（`5.0 m × 2.5 m`，`a_brake = 6.0 m/s²`）下检查 $t = 15.0\text{ s}$ 末端状态，在原 43 个 `benign` 场景中共有 **16 个**在末端仍处于同车道逼近（$v_{\text{rel}} > 0$），按物理紧迫度清晰分化为两类：

| 分档 | 场景示例 | $t=15\text{ s}$ 净车距 $d_{\text{gap}}$ | $v_{\text{rel}}$ | 末端 $\text{TTC}_{\text{end}}$ | 刹停距离 $d_{\text{stop}}$ | 物理判定 |
| --- | --- | :---: | :---: | :---: | :---: | --- |
| **末端已极度危险 / ICS**（6 个） | `cut_in(gap_d=20, rel_v=1, lc_T=3/4)`（2 个） | **`0.0 m`** | `1.0 m/s` | `≈ 0.0 s` | `0.08 m` | **ICS**（已无法刹停，下一帧即撞） |
| | `lead_brake(init_gap=35, decel=0.3)`（1 个） | **`0.6 m`** | `4.2 m/s` | **`0.1 s`** | `1.47 m` | **ICS**（$d_{\text{gap}} < d_{\text{stop}}$） |
| | `cut_in(gap_d=40, rel_v=2, lc_T=3/4)`（2 个） | `5.0 m` | `2.0 m/s` | **`2.5 s`** | `0.33 m` | 已侵入刹车阈值区（$t_{\text{on}} \le 3.0\text{ s}$） |
| | `lead_brake(init_gap=50, decel=0.3)`（1 个） | `15.6 m` | `4.2 m/s` | **`3.7 s`**（含减速约 `3.4 s`） | `1.47 m` | 已侵入 `4.0 s` 预测时域 |
| **末端仍远在安全裕度之外**（10 个） | `lead_brake(init_gap=80/100, decel=0.3)`（2 个） | `45.6 / 65.6 m` | `4.2 m/s` | `10.9 / 15.6 s` | `1.47 m` | `[0, 15 s]` 内不应急刹，保持 `benign` |
| | `cut_in(gap_d=30/40/60, rel_v=1)` 等（8 个） | `10.0 ~ 40.0 m` | `1.0 ~ 2.0 m/s` | `10.0 ~ 40.0 s` | `0.08 ~ 0.33 m` | `[0, 15 s]` 内不应急刹，保持 `benign` |

### 1.2 为什么选择「物理末端安全判据」而非改写场景定义？

- 现实道路中前车完全可能一路刹停（`lead_brake`），不能为了凑 `benign` 而强行限制物理场景行为；
- 同时，直接把所有 $v_{\text{rel}} > 0$ 的场景一律排除出 `benign` 会导致 `lead_brake` 家族失去全部 `benign` 样本，且不符合「前车远在 10 秒之外时 15 s 内不该误刹」的工程常识；
- 因此，引入**基于物理末端安全（ICS + 2D Swept TTC 外推）的三分类口径**，在不改动任何物理场景生成器的前提下，精准剔除截断边界处的伪良性样本。

---

## 2. 物理末端安全判据的数学定义

设在 `[0, T]` 内运行 `NeverBrakePolicy` 得到 `log`。若 `log.collision == True`，直接判为 `dangerous`。  
若 `log.collision == False`，在末端时刻 $t = T$ 取出自车状态 $(x_e, y_e, \theta_e, v_e)$ 与每个 agent 的末端状态 $(x_a, y_a, \theta_a, v_a)$（及可选真值加速度 $\mathbf{a}_a = (a_{x,a}, a_{y,a})$），执行以下两条物理检查（**满足任一条即判为末端不安全**）：

### 2.1 判据 A：不可避免碰撞态（ICS, Inevitable Collision State）

对每个 agent，将其相对位置 $\Delta \mathbf{p} = (x_a - x_e, y_a - y_e)$ 投影到自车末端航向局部系 $(\Delta s_{\text{lon}}, \Delta s_{\text{lat}})$：
- 定义同走廊判定：
  - OBB 模式：$|\Delta s_{\text{lat}}| \le \frac{w_{\text{ego}} + w_{\text{agent}}}{2} + 0.25\text{ m}$（行人取 $w_{\text{agent}} = 0.8\text{ m}$，车辆取当前 `vehicle_width`）；
  - 圆形模式：$|\Delta s_{\text{lat}}| \le r_{\text{safe}}$。
- 定义沿自车航向的净车距与相对接近速度：
  $$d_{\text{gap}}(T) = \begin{cases} \Delta s_{\text{lon}} - \frac{L_{\text{ego}} + L_{\text{agent}}}{2}, & \text{OBB 模式} \\ \|\Delta \mathbf{p}\|_2 - r_{\text{safe}}, & \text{圆形模式} \end{cases}$$
  $$v_{\text{rel}}(T) = v_e - v_a \cos(\theta_a - \theta_e)$$
- 当目标在自车前方（$\Delta s_{\text{lon}} > 0$）、处于同走廊且正在接近（$v_{\text{rel}}(T) > 0$）时，计算自车以最大紧急减速度 $a_{\text{brake}}$（默认 `6.0 m/s²`）将相对速度降为 $0$ 所需的最小刹停距离：
  $$d_{\text{stop}}(T) = \frac{v_{\text{rel}}(T)^2}{2 a_{\text{brake}}}$$
- **ICS 触发条件**：
  $$d_{\text{gap}}(T) \le d_{\text{stop}}(T) + d_{\text{margin}} \quad (\text{默认 } d_{\text{margin}} = 0.0\text{ m})$$
  满足此式说明即使在 $t = T$ 立刻全力制动，物理上也已无法避免碰撞。

### 2.2 判据 B：末端 2D Swept TTC 前瞻检查

为统一覆盖纵向追尾（`lead_brake`）、切入（`cut_in`）、横向横穿（`crossing`）以及 L3 Waymo 弯道场景，从 $t = T$ 起继续进行 $T_{\text{safe}}$ 秒（步数 $H_{\text{term}} = \text{round}(T_{\text{safe}} / \Delta t)$）的运动学外推：
1. **自车外推**：保持末速 $v_e(T)$ 沿 `scenario.ego_path` 前进（若路径剩余弧长不足，则从路径末点沿末段切线线性延伸）；
2. **Agent 外推**：
   - 若 `agent.accel` 非空（合成场景提供解析真值加速度），在末端速度方向上按常加速度外推，速度降为 $0$ 后保持静止（避免前车减速倒车）；
   - 若 `agent.accel` 为 `None`（如 Waymo 真实数据），按末端速度 $(v_a \cos\theta_a, v_a \sin\theta_a)$ 恒速外推；
3. **2D 扫掠碰撞检测**：直接复用 [src/pbs/risk/ttc.py](../src/pbs/risk/ttc.py) 中的 `assess_risk(..., swept=True)`（透传当前 `r_safe` / `vehicle_length` / `vehicle_width` 与 `pedestrian` 标志），取所有 agent 的最小 $\text{TTC}_{\text{end}}$。
4. **末端前瞻冲突触发条件**：
   $$\text{TTC}_{\text{end}} \le T_{\text{safe}}$$

> **`T_safe` 默认取值**：`terminal_safe_ttc = 4.0 s`。  
> **物理对齐依据**：与 L2/L3 预测器统一时域 `horizon = 40 × 0.1 s = 4.0 s` 严格一致，同时完整覆盖二值滞回（$t_{\text{on}} \le 3.0\text{ s}$）与 L3 三级预制动（$t_{\text{on,soft}} = 3.0\text{ s}$）的全部合法触发区间。

---

## 3. 三分类评估口径（为何不直接并入 `dangerous`？）

若将「15 s 内未撞、但末端不安全」的样本直接归入 `dangerous`，会产生严重的统计偏差：因为这批样本在 `[0, 15 s]` 内即使 `NeverBrakePolicy` 也不会触发 `collision = True`，将其混入 `dangerous` 会**人为稀释 `collision_rate` 分母、虚高 `rescue_rate`**。

因此，L3 评估层统一采用**三分类口径**：

| `scenario_class` | 判定条件 | 计入 `collision_rate` / `rescue_rate` 分母？ | 计入 `false_brake_rate` 分母？ | 说明 |
| --- | --- | :---: | :---: | --- |
| **`"dangerous"`** | `[0, T]` 内 `NeverBrakePolicy` 发生碰撞 | ✅ 是 | ❌ 否 | 窗口内真实碰撞样本 |
| **`"terminal_unsafe"`** | `[0, T]` 内未撞，但 $t = T$ 触发 **ICS** 或 **$\text{TTC}_{\text{end}} \le T_{\text{safe}}$** | ❌ 否 | ❌ 否 | 截断边界冲突样本：刹车**不算误刹**，单独汇报 `n_terminal_unsafe` |
| **`"benign"`** | `[0, T]` 内未撞，且 $t = T$ 物理安全（非 ICS 且 $\text{TTC}_{\text{end}} > T_{\text{safe}}$） | ❌ 否 | ✅ 是 | 真正良性样本：在此类样本中刹车才计为 `false_brake` |

### 在 `DEFAULT_GRID`（96 场景，`duration = 15.0 s`，`T_safe = 4.0 s`）下的口径变化

| 指标 | 原 L2 二分类口径 | L3 三分类口径（`T_safe = 4.0 s`） |
| --- | :---: | :---: |
| `n_dangerous` | 53 | 53（不变，分母纯净） |
| `n_benign` | 43 | 37（剔除 6 个截断伪良性，`lead_brake` 仍保留 2 个真 benign） |
| `n_terminal_unsafe` | — | **6**（2 个 `cut_in(20,1)` + 2 个 `cut_in(40,2)` + 2 个 `lead_brake(35/50, 0.3)`） |

---

## 4. 接口契约与代码修改清单

严格遵循 **只加不改、向后兼容 L2** 原则：既有 L2 调用不传新参数时行为完全不变，L3 评测通过 `check_terminal_safety=True` 启用三分类。

### 4.1 `src/pbs/eval/counterfactual.py`
- 新增结果数据类与三分类入口：
  ```python
  from dataclasses import dataclass
  from typing import Literal

  ScenarioClass = Literal["dangerous", "benign", "terminal_unsafe"]

  @dataclass(frozen=True)
  class CounterfactualResult:
      scenario_class: ScenarioClass
      collision: bool
      terminal_ics: bool
      terminal_ttc: float  # inf if no predicted conflict within terminal_safe_ttc

      @property
      def is_benign(self) -> bool:
          return self.scenario_class == "benign"

  def classify_scenario(
      scenario: Scenario,
      *,
      terminal_safe_ttc: float = 4.0,
      a_brake: float = DEFAULT_A_BRAKE,
      ics_margin: float = 0.0,
      **sim_kwargs,
  ) -> CounterfactualResult: ...
  ```
- 兼容扩展 `is_benign`：
  ```python
  def is_benign(
      scenario: Scenario,
      *,
      check_terminal_safety: bool = False,
      terminal_safe_ttc: float = 4.0,
      a_brake: float = DEFAULT_A_BRAKE,
      **sim_kwargs,
  ) -> bool:
      if not check_terminal_safety:
          return not run_sim(scenario, NullAssessor(), NeverBrakePolicy(), **sim_kwargs).collision
      return classify_scenario(
          scenario,
          terminal_safe_ttc=terminal_safe_ttc,
          a_brake=a_brake,
          **sim_kwargs,
      ).is_benign
  ```

### 4.2 `src/pbs/eval/sweep.py`
1. `run_matrix(...)` 新增可选关键字参数：
   - `check_terminal_safety: bool = False`
   - `terminal_safe_ttc: float = 4.0`
2. 缓存逻辑：
   - 当 `check_terminal_safety=True` 时，调用 `classify_scenario(scenario, terminal_safe_ttc=terminal_safe_ttc, a_brake=float(a_brake), **sim_kwargs)`；
   - 当 `check_terminal_safety=False` 时，保持原有 L2 二分类（`scenario_class = "benign" if benign else "dangerous"`，`terminal_ics = False`，`terminal_ttc = np.inf`）。
3. 输出 DataFrame 新增三列（保留原 `benign` 布尔列）：
   - `scenario_class` (`"dangerous" | "benign" | "terminal_unsafe"`)
   - `terminal_ics` (`bool`)
   - `terminal_ttc` (`float`)
4. `tradeoff(df)` 与 `rescue_rate(df)`：
   - 若 DataFrame 含有 `scenario_class` 列：
     - `dangerous = group.loc[group["scenario_class"] == "dangerous"]`
     - `benign = group.loc[group["scenario_class"] == "benign"]`
     - `terminal_unsafe = group.loc[group["scenario_class"] == "terminal_unsafe"]`
     - 在 `tradeoff()` 返回表中新增 `n_terminal_unsafe: int(len(terminal_unsafe))`。
   - 若为旧版 CSV（无 `scenario_class` 列），无缝回退到 `~group["benign"]` / `group["benign"]` 且 `n_terminal_unsafe = 0`。

### 4.3 `src/pbs/eval/metrics.py` 与 `src/pbs/eval/__init__.py`
- `evaluate(log, benign)` 函数签名保持不变：当 `scenario_class == "terminal_unsafe"` 时，`benign` 为 `False`，因此 `false_brake = benign and braked` 自然为 `False`，不会将合法刹车误判为误刹；
- `src/pbs/eval/__init__.py` 补充导出 `CounterfactualResult` 与 `classify_scenario`。

---

## 5. 验收测试清单（`tests/test_counterfactual.py` & `tests/test_sweep.py`）

- [ ] **向后兼容**：默认 `is_benign(scenario)` 与 `run_matrix(...)` 不传 `check_terminal_safety` 时，全量既有 348 项 `pytest` 零回归；
- [ ] **ICS 拦截**：`lead_brake(init_gap=35.0, decel=0.3)` 与 `cut_in(gap_d=20.0, rel_v=1.0)` 在 `classify_scenario(..., duration=15.0)` 下判为 `"terminal_unsafe"` 且 `terminal_ics == True`；
- [ ] **末端 TTC 拦截**：`cut_in(gap_d=40.0, rel_v=2.0)`（末端 TTC ≈ 2.5 s）与 `lead_brake(init_gap=50.0, decel=0.3)`（末端 TTC < 4.0 s）判为 `"terminal_unsafe"` 且 `terminal_ttc <= 4.0`；
- [ ] **真良性保留**：`lead_brake(init_gap=100.0, decel=0.3)`、`cut_in(gap_d=60.0, rel_v=1.0)`、`cut_in_benign_fast_cutter`、`crossing_benign` 均保持 `"benign"`；
- [ ] **横穿边界拦截**：构造在 $t = 15.5\text{ s}$ 碰撞的 `crossing` 场景，验证 2D Swept 外推能正确判为 `"terminal_unsafe"`；
- [ ] **矩阵分母纯净性**：在 `tradeoff()` 中，`terminal_unsafe` 样本既不计入 `n_dangerous` 也不计入 `n_benign`，准确计入 `n_terminal_unsafe`，且在其中刹车时 `false_brake == False`。

# L3 三级滞回制动与双档 Jerk 限幅规范（`StagedBrakingPolicy`）

> **维护者**：C（前背痛 · `pbs.sim.policy` & `simulator`） · **协同评估**：D（yj · `pbs.eval.metrics`）
> **所属阶段**：L3 物理与舒适性升级（保持 L2 `HysteresisPolicy(2.0, 3.0, 6.0)` 默认不变）

---

## 1. 为什么要做“三级滞回 + 双档 Jerk 限幅”？

在 L2 基线中，自车采用二值阶跃制动（`cruise = 0 m/s²`，`brake = -6.0 m/s²`），存在三个明显的物理与驾驶体验局限：
1. **瞬时阶跃冲击（Jerk = $60\text{ m/s}^3$）**：在 $\Delta t = 0.1\text{ s}$ 内直接从 $0$ 跳到 $-6.0\text{ m/s}^2$，未反映液压/线控制动系统的建压过程，且对乘员产生剧烈前冲。
2. **轻微风险也满额急刹**：只要 $\text{TTC} < 2.0\text{ s}$ 就一脚 $-6.0\text{ m/s}^2$ 到底；在许多早期切入或中止换道（`abort_cut_ins`）场景中，原本只需温和预减速即可化解风险，满额急刹不仅舒适性差，还容易引发后车追尾。
3. **“一刹永慢”**：L2 松开刹车后仅维持降速后的低速，不恢复目标巡航车速，导致一次早期点刹永久改变后续交通交互。

同时，如果对所有阶段只使用**单一 Jerk 限幅**（如 $10\text{ m/s}^3$），会出现两难：对舒适减速来说 $10\text{ m/s}^3$ 依然偏冲；而对紧急制动（AEB）来说，从 $0$ 建压到 $-6.0\text{ m/s}^2$ 需要 $0.6\text{ s}$，会额外损失近 $2\text{ m}$ 的极限刹停距离。

因此，L3 引入**三级滞回状态机 + 双档 Jerk 限幅（舒适档 $5.0\text{ m/s}^3$ / 紧急档 $20.0\text{ m/s}^3$）**，兼顾日常平顺性与极限避撞安全性。

---

## 2. 行业与工程标准依据（可直接写入最终报告）

| 参数 | 设定值 | 行业标准与车辆动力学依据 |
| --- | --- | --- |
| **一级舒适预制动减速度 $a_{\text{soft}}$** | $-2.5\text{ m/s}^2$ | **ISO 15622（ACC 自适应巡航）** 与交通工程舒适制动上限为 $2.0 \sim 3.0\text{ m/s}^2$；乘员无明显前倾不适，足以通过早期干预拉开车距。 |
| **二级紧急制动减速度 $a_{\text{hard}}$** | $-6.0\text{ m/s}^2$ | **Euro NCAP AEB** 与干燥沥青路面乘用车典型有效紧急减速度（$6 \sim 8\text{ m/s}^2$），与本仓库 L2 基线最大减速度严格对齐，保证物理极限可比。 |
| **温和恢复加速度 $a_{\text{recover}}$** | $+1.5\text{ m/s}^2$ | **ISO 15622** 规定巡航恢复加速上限 $\le 2.0\text{ m/s}^2$；风险解除后温和恢复至初始巡航速度 $v_{\text{target}}$。 |
| **舒适档加加速度限幅 $J_{\text{soft}}$** | $5.0\text{ m/s}^3$ | **ISO 15622** 规定 ACC 正常调节的最大加加速度均值为 $2.5 \sim 5.0\text{ m/s}^3$（每 $0.1\text{ s}$ 步变化 $0.5\text{ m/s}^2$），用于预制动、松刹回退与恢复巡航。 |
| **紧急档加加速度限幅 $J_{\text{hard}}$** | $20.0\text{ m/s}^3$ | 现代线控制动（iBooster / ESP）紧急全力建压时间通常为 **$0.15 \sim 0.30\text{ s}$**；取 $20.0\text{ m/s}^3$（每 $0.1\text{ s}$ 步变化 $2.0\text{ m/s}^2$）时，从 $0$ 踩满到 $-6.0\text{ m/s}^2$ 耗时 **$0.3\text{ s}$**，从预制动 $-2.5\text{ m/s}^2$ 切入急刹仅需 **$0.2\text{ s}$**，几乎不损失极限刹停距离。 |

---

## 3. 三级滞回状态机与双档 Jerk 数学定义

### 3.1 四个控制状态与目标加速度

设自车初始巡航目标车速为 $v_{\text{target}} = v_{\text{ego}}(0)$，当前估计的最小冲突时间为 $\text{TTC}_k \in [0, \infty]$：

| 状态名（`action` 标签） | 进入条件 | 退出/保持滞回条件 | 目标加速度 $a_{\text{target}}$ | 适用 Jerk 限幅 $J$ |
| --- | --- | --- | :---: | :---: |
| **`hard_brake`**（二级急刹） | 任意状态下 $\text{TTC}_k < t_{\text{on,hard}}$（**`1.5 s`**） | $\text{TTC}_k \le t_{\text{off,hard}}$（**`2.0 s`**）时保持 `hard_brake` | $-6.0\text{ m/s}^2$ | **$J_{\text{hard}} = 20.0\text{ m/s}^3$** |
| **`soft_brake`**（一级预制动） | 1. 处于 `cruise`/`recover` 且 $1.5\text{ s} \le \text{TTC}_k < t_{\text{on,soft}}$（**`3.0 s`**）<br>2. 处于 `hard_brake` 且 $2.0\text{ s} < \text{TTC}_k \le t_{\text{off,soft}}$（**`3.5 s`**） | $\text{TTC}_k \le t_{\text{off,soft}}$（**`3.5 s`**）且 $\text{TTC}_k \ge 1.5\text{ s}$ 时保持 `soft_brake` | $-2.5\text{ m/s}^2$ | **$J_{\text{soft}} = 5.0\text{ m/s}^3$** |
| **`recover`**（温和恢复） | 退出制动（或处于非制动态且 $\text{TTC}_k \ge 3.0\text{ s}$），且当前车速 $v_k < v_{\text{target}} - 10^{-6}$ | $v_k$ 到达 $v_{\text{target}}$ 后自动转为 `cruise` | $+1.5\text{ m/s}^2$ | **$J_{\text{soft}} = 5.0\text{ m/s}^3$** |
| **`cruise`**（定速巡航） | 非制动态且 $v_k \ge v_{\text{target}} - 10^{-6}$ | $\text{TTC}_k \ge 3.0\text{ s}$ 时保持 `cruise` | $0.0\text{ m/s}^2$ | **$J_{\text{soft}} = 5.0\text{ m/s}^3$** |

> **滞回防抖原则**：所有阈值边界相等时（如 $\text{TTC}_k = 1.5\text{ s}$ 或 $2.0\text{ s}$ 或 $3.0\text{ s}$ 或 $3.5\text{ s}$）**保持上一帧的制动等级不变**，防止在临界 TTC 附近来回抖动。

---

### 3.2 双档 Jerk 限幅与速度边界钳位公式

在每个仿真步 $k$（步长 $\Delta t = 0.1\text{ s}$），给定上一帧实际执行的加速度 $a_{k-1}$（初始 $a_{-1} = 0.0$）和当前状态对应的 $(a_{\text{target}}, J)$：

1. **按档位 Jerk 限幅**：
   $$J_k = \begin{cases} J_{\text{hard}} = 20.0\text{ m/s}^3, & \text{if state}_k = \text{\texttt{"hard\_brake"}} \\ J_{\text{soft}} = 5.0\text{ m/s}^3, & \text{otherwise} \end{cases}$$
   $$a_{\text{raw}} = \text{clip}\big(a_{\text{target}},\; a_{k-1} - J_k \cdot \Delta t,\; a_{k-1} + J_k \cdot \Delta t\big)$$

2. **物理边界钳位（防倒车与防超速）**：
   - **静止钳位**：若当前车速 $v_k \le 0$ 且 $a_{\text{raw}} < 0$，则车辆已刹停，令 $a_k = 0.0$；
   - **恢复达速钳位**：若 $a_{\text{raw}} > 0$ 且 $v_k + a_{\text{raw}} \cdot \Delta t > v_{\text{target}}$，则令：
     $$a_k = \min\left(a_{\text{raw}},\; \frac{v_{\text{target}} - v_k}{\Delta t}\right)$$
     使步末车速精确收敛至 $v_{\text{target}}$，不发生超调；
   - 其余情况 $a_k = a_{\text{raw}}$，并更新内部状态 `self.prev_a = a_k`。

---

### 3.3 在 $\Delta t = 0.1\text{ s}$ 下的典型建压与释放时间线

- **场景 A（预测器提前发现切入：巡航 → 一级预制动 → 二级急刹）**：
  1. **进入预制动**（$J_{\text{soft}} = 5.0\text{ m/s}^3$，每步 $-0.5\text{ m/s}^2$）：
     $$0.0 \to -0.5 \to -1.0 \to -1.5 \to -2.0 \to -2.5\text{ m/s}^2 \quad (\text{耗时 } 0.5\text{ s 平滑建压})$$
  2. **若风险继续升级进入急刹**（切换 $J_{\text{hard}} = 20.0\text{ m/s}^3$，每步 $-2.0\text{ m/s}^2$）：
     $$-2.5 \to -4.5 \to -6.0\text{ m/s}^2 \quad (\text{因已有预制动底压，仅需 } \mathbf{0.2\text{ s}} \text{ 即达满额急刹！})$$
  > **核心物理洞察**：这也从动力学上解释了**为什么预测型 AEB（CV / CA / Neural）比 Reactive 更安全**——预测器不仅让刹车启动时刻更早，还通过一级预制动提前完成了“预建压（Pre-fill）”，一旦需要二级急刹，只需 $0.2\text{ s}$ 就能达到 $-6.0\text{ m/s}^2$！
- **场景 B（Reactive 晚发现：巡航直接跳 → 二级急刹）**：
  - 使用 $J_{\text{hard}} = 20.0\text{ m/s}^3$（每步 $-2.0\text{ m/s}^2$）：
    $$0.0 \to -2.0 \to -4.0 \to -6.0\text{ m/s}^2 \quad (\text{耗时 } 0.3\text{ s 建压})$$
- **场景 C（风险解除：从 `-2.5 m/s²` 预制动松刹并恢复车速）**：
  - 使用 $J_{\text{soft}} = 5.0\text{ m/s}^3$（每步 $+0.5\text{ m/s}^2$）：
    $$-2.5 \to -2.0 \to -1.5 \to -1.0 \to -0.5 \to 0.0 \to +0.5 \to +1.0 \to +1.5\text{ m/s}^2$$
    全程平顺过渡，无任何阶跃顿挫。

---

## 4. 代码实现模板（供 C 直接落入 `src/pbs/sim/policy.py`）

```python
SOFT_BRAKE, HARD_BRAKE, RECOVER = "soft_brake", "hard_brake", "recover"
BRAKE_ACTIONS = frozenset({BRAKE, SOFT_BRAKE, HARD_BRAKE})


def is_braking_action(action: str) -> bool:
    """True for L1/L2 'brake' and L3 'soft_brake' / 'hard_brake'."""
    return action in BRAKE_ACTIONS


class StagedBrakingPolicy:
    """Three-stage hysteresis AEB controller with dual-rate jerk limiting.

    States:
      - 'cruise':     target 0.0 m/s²       (jerk_soft)
      - 'recover':    target +a_recover     (jerk_soft, capped at v_target)
      - 'soft_brake': target -a_soft        (jerk_soft)
      - 'hard_brake': target -a_hard        (jerk_hard)
    """

    def __init__(
        self,
        *,
        t_on_soft: float = 3.0,
        t_off_soft: float = 3.5,
        t_on_hard: float = 1.5,
        t_off_hard: float = 2.0,
        a_soft: float = 2.5,
        a_hard: float = 6.0,
        a_recover: float = 1.5,
        jerk_soft: float = 5.0,
        jerk_hard: float = 20.0,
        v_target: float | None = None,
    ):
        params = (
            t_on_soft,
            t_off_soft,
            t_on_hard,
            t_off_hard,
            a_soft,
            a_hard,
            a_recover,
            jerk_soft,
            jerk_hard,
        )
        if not all(math.isfinite(x) for x in params):
            raise ValueError("Policy parameters must be finite.")
        if not (0 < t_on_hard < t_off_hard <= t_on_soft < t_off_soft):
            raise ValueError("Require 0 < t_on_hard < t_off_hard <= t_on_soft < t_off_soft.")
        if not (0 < a_soft < a_hard and a_recover >= 0 and 0 < jerk_soft <= jerk_hard):
            raise ValueError("Invalid acceleration or jerk bounds.")
        self.t_on_soft, self.t_off_soft = float(t_on_soft), float(t_off_soft)
        self.t_on_hard, self.t_off_hard = float(t_on_hard), float(t_off_hard)
        self.a_soft, self.a_hard, self.a_recover = float(a_soft), float(a_hard), float(a_recover)
        self.a_brake = self.a_hard  # compatibility attribute for maximum braking capability
        self.jerk_soft, self.jerk_hard = float(jerk_soft), float(jerk_hard)
        self._default_v_target = None if v_target is None else float(v_target)
        self.reset(v_target=self._default_v_target)

    def reset(self, *, v_target: float | None = None) -> None:
        self.state = CRUISE
        self.prev_a = 0.0
        self.v_target = self._default_v_target if v_target is None else float(v_target)

    def _update_stage(self, ttc: float, v: float) -> None:
        if math.isnan(ttc) or ttc < 0:
            raise ValueError("TTC must be nonnegative or positive infinity.")
        if self.state == HARD_BRAKE:
            if ttc > self.t_off_hard:
                if ttc <= self.t_off_soft:
                    self.state = SOFT_BRAKE
                else:
                    self.state = self._non_braking_state(v)
        elif self.state == SOFT_BRAKE:
            if ttc < self.t_on_hard:
                self.state = HARD_BRAKE
            elif ttc > self.t_off_soft:
                self.state = self._non_braking_state(v)
        else:
            if ttc < self.t_on_hard:
                self.state = HARD_BRAKE
            elif ttc < self.t_on_soft:
                self.state = SOFT_BRAKE
            else:
                self.state = self._non_braking_state(v)

    def _non_braking_state(self, v: float) -> str:
        if self.v_target is not None and self.a_recover > 0 and v < self.v_target - 1e-6:
            return RECOVER
        return CRUISE

    def decide_accel(self, ttc: float, *, v: float, dt: float) -> tuple[str, float]:
        if not (math.isfinite(v) and v >= 0 and math.isfinite(dt) and dt > 0):
            raise ValueError("Require nonnegative finite speed v and positive finite dt.")
        if self.v_target is None:
            self.v_target = float(v)
        self._update_stage(ttc, v)

        if self.state == HARD_BRAKE:
            target_a, max_j = -self.a_hard, self.jerk_hard
        elif self.state == SOFT_BRAKE:
            target_a, max_j = -self.a_soft, self.jerk_soft
        elif self.state == RECOVER:
            target_a, max_j = self.a_recover, self.jerk_soft
        else:
            target_a, max_j = 0.0, self.jerk_soft

        max_da = max_j * dt
        a = max(self.prev_a - max_da, min(self.prev_a + max_da, target_a))
        if v <= 0.0 and a < 0.0:
            a = 0.0
        elif a > 0.0 and self.v_target is not None and v + a * dt > self.v_target:
            a = max(0.0, (self.v_target - v) / dt)
        self.prev_a = float(a)
        return self.state, float(a)
```

---

## 5. 与 D（`pbs.eval.metrics`）的舒适性指标对接约定

D 在 `evaluate(log, benign)` 中对有效执行帧 $0 \le k < k_{\text{end}}$（截止于首次接触 `first_collision_t` 或 `executed_steps`）统计以下指标：

1. **制动触发判定**：`action in {"brake", "soft_brake", "hard_brake"}`（可直接调用 `is_braking_action(action)`）。
2. **`max_decel` `[m/s²]`**：$\max(0.0, \max_{0 \le k < k_{\text{end}}} (-a_k))$
   - 若 $\le 2.5\text{ m/s}^2$，说明全程仅靠舒适预制动即完成避险。
3. **`peak_jerk` & `rms_jerk` `[m/s³]`**：
   - 步间加加速度序列 $j_k = \frac{a_k - a_{k-1}}{\Delta t}$（其中 $k = 0 \dots k_{\text{end}}-1$，规定 $a_{-1} = 0.0$ 以包含首步建压冲击！）；
   - `peak_jerk` $= \max_k |j_k|$（L2 二值急刹为 $60.0\text{ m/s}^3$；L3 舒适预制动 $\le 5.0\text{ m/s}^3$，紧急制动 $\le 20.0\text{ m/s}^3$）；
   - `rms_jerk` $= \sqrt{\frac{1}{k_{\text{end}}} \sum_{k=0}^{k_{\text{end}}-1} j_k^2}$。
4. **`speed_loss` `[m/s]` & `end_speed` `[m/s]`**：
   - `speed_loss` $= \max(0.0, v_0 - \min_{0 \le k \le k_{\text{end}}} v_k)$；
   - `end_speed` $= v_{k_{\text{end}}}$（展示 L3 在风险解除后恢复巡航车速的能力）。
5. **`hard_brake` `bool`**：是否出现 `"hard_brake"`（用于区分 benign 场景中的“温和预减速”与“严重误急刹”）。

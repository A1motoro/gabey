# L3 运行期安全兜底规范：帧级 Simplex 保护、缺测状态机与统一 L3 执行链路

> **维护者**：C（前背痛 · `pbs.sim.failsafe` & `pbs.sim.policy` & `pbs.sim.simulator`） · **协同接口**：P（华华 · `pbs.risk`）& D（yj · `pbs.eval`）  
> **设计评审依据**：[docs/T6_failsafe_design_review.md](T6_failsafe_design_review.md)  
> **所属阶段**：L3 仿真运行期容错升级（严格遵循「只加不改，向后兼容 L2」原则，不破坏现有独立基线评测与 348 项既有测试）

---

## 1. 设计定位：为什么 `Reactive` 是全系统的最终兜底，以及为什么必须按「帧级」设计？

预测型 AEB（`CV` / `CA` / `CTRV` / `LR` / `NN-Traj` / `NN` / `NN-UA`）虽然能提前发现邻车道切入，但高度依赖多帧历史完整性、车道上下文与稳定算力；一旦任一输入或算力环节失效，确定性、单帧微秒级的 **`ReactiveAssessor` 是所有预测型 AEB 系统的最终安全兜底机制**。

在进入编码前，[T6_failsafe_design_review.md](T6_failsafe_design_review.md) 明确了四个必须落实的核心设计约束（修正了早期「单目标同步调用 + 外部复制 `_last_accel`」草案的缺陷）：
1. **帧级并行与共享预算（Frame-Level Budget）**：不能按单目标同步串行计时（否则 3 辆车各 40 ms 累计 120 ms 超出 `dt = 100 ms`，且卡死会阻塞控制侧）。必须整帧共享一个 `frame_budget_s = 0.05 s`，并行调度 Reactive 与预测通道，到点只读帧号匹配的已就绪结果。
2. **显式区分「有效无冲突（`TTC = +∞`）」与「观测缺失/过期（`unknown`）」**：两条通道共享车辆历史与自车路径，切换算法无法补回共同缺失的输入；必须基于采样时间戳计算观测年龄并按 `agent_id` 追踪失联状态，防止有风险前车失联被其他正常目标误解除。
3. **Hold 保持「制动档位」而非冻结「上一帧加速度」**：绝不在外部直接复制 `prev_a`（否则刚进硬刹 `-2.0 m/s²` 丢帧会卡死建压、恢复 `+1.5 m/s²` 丢帧直降 0 会触发超标 Jerk）。正常帧、缺测帧与恢复帧必须**共用 `StagedBrakingPolicy` 内部同一个 `prev_a`、双档 Jerk 限幅与速度边界前瞻卸力**。
4. **延迟裕度统一偏移有限 TTC，且与缺测宽限解耦**：对有效且有限的 TTC 执行 `TTC_eff = max(0, TTC_raw - latency_margin)`（既提前刹车又延后松刹，滞回带宽保持 `0.5 s`，边界相等保持原档位）；`latency_margin` 与缺测宽限 `missing_grace_s` 必须作为两个独立参数分别配置和调参。

> **评测解耦 vs 部署兜底原则**：
> - `exp02` / `exp03` / `exp05` 等科研消融实验继续使用独立的 `ReactiveAssessor` 与 `PredictiveAssessor`，以精确分离预测提前量收益与预测误差代价；
> - L3 运行期容错通过帧级协调器 `FrameFailsafeCoordinator` 统一包裹任意预测系统与 `ReactiveAssessor`，零侵入 P 的预测器实现。

---

## 2. 四项核心机制的详细规范

### 2.1 帧级并行调度与 50 ms 整帧预算（问题 1）

以当前控制周期 `dt = 0.10 s` 为背景，设定整帧共享预算 **`frame_budget_s = 0.05 s`**（Reactive 与预测通道均纳入该预算）：

1. **帧快照与帧号（`frame_id`）**：每帧开始时固定观测快照与单调递增 `frame_id`；预测通道可按帧批量处理全部目标，控制侧并行运行 Reactive。
2. **防积压机制（Skip-While-Busy / 单槽不排队）**：若上一帧预测任务尚未完成，新帧**直接跳过提交**（绝不使用 FIFO 队列堆叠过时任务），本帧由 Reactive 兜底；上一帧的迟到结果完成时因 `result.frame_id != current_frame_id` 被直接丢弃，绝不覆盖后续帧。
3. **50 ms 截止点逐目标融合真值表**（在观测新鲜的前提下）：

| Reactive 通道 | 预测通道 | 融合判定状态 | 有效 TTC 与来源记录 |
| --- | --- | --- | --- |
| 就绪且有效 | 就绪且有效 | 至少一方有有限 TTC → `valid_threat`；否则 `valid_clear` | 取参与评估（`considered=True`）通道的最小 TTC（`source="fused"`）；邻车道时 Reactive 返回 `considered=False`，不干扰预测提前量也不增加误刹 |
| 就绪且有效 | 超时 / 异常 / 忙跳过 / 无效输出 | `valid_threat` 或 `valid_clear` | 采用 Reactive 结果（`source="reactive_fallback"`），累计对应降级原因 |
| 超时 / 异常 | 就绪且有效 | `valid_threat` 或 `valid_clear` | 采用预测结果（`source="primary_only"`） |
| 不可用 / 超时 | 不可用 / 超时 | **`unknown`** | 标记该目标风险未知，交由第 2.2 节缺测状态机处理 |

4. **双执行后端（兼顾离线确定性仿真与真实模型压测）**：
   - **可控时钟 / 确定性后端（默认用于 `run_sim` 消融与单元测试）**：支持注入可控虚拟时钟与通道完成时刻（如 Reactive 第 8 ms、预测第 80 ms），100% 确定性复现 50 ms 截止发令、80 ms 迟到帧号丢弃、多目标共享预算与任务永不完成等场景，不依赖脆弱的真实 `time.sleep`；
   - **真实异步执行器后端（可选用于真实模型接入后的端到端耗时验证）**：单个持久 worker（`max_workers=1` + 最新槽位覆盖不排队）在截止点按剩余预算非阻塞取回已就绪结果。

---

### 2.2 观测新鲜度、按目标失联追踪与多目标仲裁（问题 2）

1. **基于采样时间戳的观测年龄（抗旧帧重发）**：
   - 设基线已知通信延迟为 `base_latency`（默认 `0.0 s`），对每个 `agent_id` 记录最近一次收到**新鲜有效观测**时的采样时间戳 `last_fresh_sample_t`；
   - 若本帧该目标无样本、或 `sample_t <= last_fresh_sample_t`（重复发送旧样本，**绝不清零计时**）、或超额年龄 `stale_age = (t_now - base_latency) - sample_t > max_obs_age_s`（默认 `0.05 s`），则该目标本帧判定为 **`obs_unavailable`（缺失/过期）**，并累加其缺测持续时长 `missing_duration = max(0.0, (t_now - base_latency) - last_fresh_sample_t)`。
   - **与 `run_sim` 既有 `MissingObservationError` 的向后兼容边界**：L2 默认路径（未开启 T6 容错模式）遇空历史继续抛出 `MissingObservationError`，保持 348 项测试零回归；仅当显式开启 T6 容错模式 / 缺测注入观测模型时，将空历史或过期重发转为显式 `obs_unavailable` 送入下表状态机。

2. **失联与缺测响应策略（按状态转移）**：

| 失联前的状态 | 首个缺测周期 | 缺测持续时 |
| --- | --- | --- |
| **已在 `soft_brake` 或 `hard_brake`** | **保留原制动档位**，继续按对应档位建立所需制动力 | 继续制动，接近停车时沿用 L3 前瞻卸力平滑停车逻辑 |
| **正在 `cruise` 或 `recover`（最近有效观测无近迫风险）** | 进入 `coast_zero`：**将目标加速度设为 `0.0`**，按 `jerk_soft` 平滑撤去驱动力，同时继续等待观测 | 连续缺测满 `missing_grace_s`（默认待标定 **`0.20 s`**，验收对比 `0.10 / 0.20 / 0.30 s`）后进入 **`soft_brake`**，逐渐减速至停 |
| **有风险的前车失联，其他目标仍可见** | 按 `agent_id` **保留失联前车对应的制动档位要求**，同时继续评估所有可见目标 | 其他可见目标要求更强制动时**立即升级**；其他目标的正常观测（`TTC = +∞`）**绝不能解除**失联前车触发的制动 |

> **说明（区分高风险前车失联 vs 无关背景车离场）**：仅当某个目标失联前已持有制动需求（或全帧感知整体失联）时保留/触发降级制动；无关远距背景车正常驶出视野不触发误刹。

3. **恢复防抖（Recovery Confirmation）**：
   - **升级立即响应**：任一新鲜有效观测发现更高风险时，**当帧立即允许升级制动**（零延迟）；
   - **松刹/降级防抖**：从缺测保持或高制动档位准备松刹/降级时，要求连续 **`recovery_confirm_frames = 2`** 帧（待标定参数）新鲜有效观测，且同时满足原有滞回退出条件。
   - 本规范假定执行端始终可读取自车速度 `v` 并执行纵向加速度指令。

---

### 2.3 统一 L3 制动执行链路：Hold 锁档位，统一算 Jerk（问题 3）

**废除外部复制 `_last_accel` 的旧 `FailSafeHoldPolicy` 设计**。改为在 `StagedBrakingPolicy`（`src/pbs/sim/policy.py`）内将「目标档位决策」与「动力学加速度执行」拆为清晰的单一路径：

1. **统一档位到动力学参数映射**（正常帧、缺测帧、恢复帧共用同一个 `self.prev_a` 与 `_speed_limit_accel`）：

| 最终下发档位 `mode` | 目标加速度 $a_{\text{target}}$ | 适用 Jerk 限幅 | 速度边界保护 |
| --- | :---: | :---: | --- |
| `hard_brake` | `-6.0 m/s²` | `jerk_hard = 20.0 m/s³` | 接近 `v = 0` 前按 `jerk_soft` 提前卸力 |
| `soft_brake` | `-2.5 m/s²` | `jerk_soft = 5.0 m/s³` | 接近 `v = 0` 前按 `jerk_soft` 提前卸力 |
| `coast_zero`（首帧缺测撤力） | `0.0 m/s²` | `jerk_soft = 5.0 m/s³` | 仅平滑归零，不触发加速 |
| `recover` | `+1.5 m/s²` | `jerk_soft = 5.0 m/s³` | 接近 `v_target` 前按 `jerk_soft` 提前收力 |
| `cruise` | `0.0 m/s²` | `jerk_soft = 5.0 m/s³` | — |

2. **关键边界的逐帧行为保证**（`dt = 0.10 s`）：
   - **刚进硬刹即丢帧**（`prev_a = -2.0 m/s²`）：仲裁器保持 `mode = "hard_brake"`，统一执行器按 `jerk_hard = 20.0` 继续建压：`-2.0 → -4.0 → -6.0 m/s²`，绝不被锁死在 `-2.0`；
   - **正在恢复即丢帧**（`prev_a = +1.5 m/s²`）：仲裁器下发 `mode = "coast_zero"`（目标 `0.0`），统一执行器按 `jerk_soft = 5.0` 平滑撤力：`+1.5 → +1.0 → +0.5 → 0.0 m/s²`；若在 `+0.5` 时宽限满升级为 `soft_brake`，直接从 `+0.5 → 0.0 → -0.5 → ... → -2.5` 连续过渡，全过程绝不产生 Jerk 尖峰。

---

### 2.4 延迟裕度补偿与独立参数契约（问题 4）

对任意有效且有限的 $\text{TTC}_{\text{raw}}$ 执行统一补偿后送入原有滞回状态机：
$$\text{TTC}_{\text{eff}} = \max\!\big(0.0,\; \text{TTC}_{\text{raw}} - \text{latency\_margin}\big)$$

以 `latency_margin = 0.20 s` 为例（等于退出阈值时保持原档位，两档滞回带宽均维持 `0.5 s`）：

| 档位 | 原始 TTC 进入条件 | 原始 TTC 退出条件 |
| --- | --- | --- |
| `soft_brake` | $\text{TTC}_{\text{raw}} < 3.2\text{ s}$（原 `3.0 s`） | $\text{TTC}_{\text{raw}} > 3.7\text{ s}$（原 `3.5 s`） |
| `hard_brake` | $\text{TTC}_{\text{raw}} < 1.7\text{ s}$（原 `1.5 s`） | $\text{TTC}_{\text{raw}} > 2.2\text{ s}$（原 `2.0 s`） |

- `latency_margin = 0.0` 时严格复现既有 L3 切换序列；`+∞` 保持 `+∞`；缺测状态走第 2.2 节失效策略，不经过此公式。
- **参数严格解耦**：`latency_margin`（补偿有效有限 TTC）与 `missing_grace_s`（缺测多久后进入软刹）即使实验数值同为 `0.20 s`，也必须作为两个独立参数分别配置和调参。

---

## 3. 代码模块规划与实际实现接口（归属 C，已落地）

- **新增文件**：
  - `src/pbs/sim/failsafe.py`：`FailsafeConfig`、`_Track`（按 `agent_id` 追踪采样时间戳、需求档位与恢复确认计数）、`FrameFailsafeCoordinator`；
  - `src/pbs/sim/_failsafe_channels.py`：`VirtualChannels`（确定性虚拟时钟后端，支持整帧标量耗时与逐目标耗时序列）与 `AsyncChannels`（基于 `multiprocessing.get_context("spawn")` 的独立持久 daemon 工作进程，单槽 `Queue(maxsize=1)` + `Pipe(duplex=False)`，`close()` 强制回收阻塞子进程）；
  - `src/pbs/sim/__init__.py`：统一导出 `FailsafeConfig`、`FrameDecision`、`FrameFailsafeCoordinator`；
  - `tests/test_failsafe.py`：覆盖 4 项评审验收与闭环回归（共 17 项用例，含参数化变体）。
- **协同小改（零侵入 `src/pbs/risk/`）**：
  - `src/pbs/types.py`：新增 `FrameDecision` 数据类，并在 `SimLog` 末尾追加可选字段 `failsafe_decisions`；
  - `src/pbs/sim/observation.py`：`ObservationModel.observe` 协议返回类型放宽为 `Trajectory | None`（允许 T6 注入丢包与子集采样，L2 默认不变）；
  - `src/pbs/sim/policy.py`：新增常量 `COAST_ZERO = "coast_zero"`（`is_braking_action("coast_zero") == False`），在 `StagedBrakingPolicy` 拆出 `brake_mode(ttc, previous) -> str` 与 `_apply_mode_accel(mode, *, v, dt)`，并在 `decide_accel(..., mode_override="normal")` 支持 T6 直接锁定目标档位；
  - `src/pbs/sim/simulator.py`：`run_sim` 新增可选关键字参数 `failsafe_coordinator=None`（默认 `None` 时 100% 走既有 L1/L2/L3 路径，遇空历史仍抛 `MissingObservationError`）；传入时由 `run_sim` 托管 `bind / step_frame / close` 全生命周期，在 `finally` 中确保异步子进程干净释放。

### 核心数据结构与实际接口签名

```python
from collections import Counter
from dataclasses import dataclass
from typing import Literal

FailsafeMode = Literal["normal", "coast_zero", "soft_brake", "hard_brake"]

@dataclass(frozen=True)
class FailsafeConfig:
    frame_budget_s: float = 0.05       # 整帧共享预算（Reactive + 预测均计入）
    max_obs_age_s: float = 0.05        # 超过该超额观测年龄即判过期
    missing_grace_s: float = 0.20      # 巡航/恢复态缺测宽限（独立于 latency_margin）
    recovery_confirm_frames: int = 2   # 松刹/降级所需连续新鲜有效帧数
    latency_margin: float = 0.0        # 有限有效 TTC 的延迟裕度偏移（独立调参）
    base_latency: float = 0.0          # 已知基线通信延迟，用于校准观测超额年龄

@dataclass(frozen=True)
class FrameDecision:
    frame_id: int
    effective_ttc: float               # 供正常滞回使用的有效 TTC（全局缺测时为 NaN）
    mode_override: str                 # "normal" 走原滞回；其余为 coast_zero/soft_brake/hard_brake
    source: str                        # "fused" / "reactive_fallback" / "primary_only" / "mixed" / "clear" / "failsafe_hold"
    missing_agents: tuple[str, ...]
    dropped_stale_results: int
    elapsed_s: float
    agent_sources: dict[str, str]      # 各可见目标通道来源

class FrameFailsafeCoordinator:
    """帧级 Simplex 协调器：并行预算裁剪 + 帧号校验 + 按目标失联追踪 + 多目标仲裁。"""
    def __init__(
        self,
        primary,
        *,
        reactive=None,
        config: FailsafeConfig | None = None,
        executor_mode: Literal["deterministic", "async"] = "deterministic",
        clock=None,
    ): ...

    def bind(self, scenario, *, policy=None, base_latency=None) -> None: ...
    def close(self) -> None: ...
    def step_frame(
        self,
        frame_id: int,
        ctx,
        agent_histories: dict,         # agent_id -> Trajectory | None（None 表示本帧缺测）
        *,
        current_policy_mode: str,
        channel_timings: dict | None = None,  # 确定性后端可注入 {reactive_s, primary_s}（标量或逐目标列表）
    ) -> FrameDecision: ...
```

> **诊断统计**：`coordinator.fallback_reasons` 为 `Counter`，键为二元组 `(channel_name, reason)`，覆盖 `timeout`、`busy`、`exception`、`invalid_output`、`bind_error`、`unavailable`、`worker_exit`；`coordinator.dropped_stale_results` 累计被帧号丢弃的迟到帧总数。

---

## 4. 仿真验收测试清单（`tests/test_failsafe.py`，与评审文档四项验收逐条对齐）

- [ ] **验收 1（并行与 50 ms 整帧预算）**：
  - 可控时钟模拟 Reactive 第 8 ms 完成、预测第 80 ms 完成 → 第 50 ms 用 Reactive 发出本帧决定，第 80 ms 迟到结果按 `frame_id` 丢弃且不覆盖下一帧；
  - 多目标（如 3 辆车各 40 ms）共享 50 ms 整帧预算，总耗时不得静默超标；
  - 预测任务永不完成（卡死/忙跳过）与 Reactive 超出 50 ms（双通道均不可用转 `unknown`）均不阻塞控制侧；
  - 真实模型接入后补测端到端异步耗时。
- [ ] **验收 2（双通道缺测、旧时间戳与单目标失联）**：
  - 同一组合成切入与良性场景对比 `missing_grace_s ∈ {0.10, 0.20, 0.30} s`，记录碰撞、最小间距、误刹与速度损失；
  - 旧时间戳重复重发不清零观测年龄，满宽限后触发 `soft_brake`；
  - 仅高风险前车失联、其他无关目标仍可见 → 保留前车制动档位，其他目标的 `TTC = +∞` 不得解除制动，其他目标更危险时立即升级；
  - 危险目标重现当帧即升级制动；松刹/降级需连续 `recovery_confirm_frames = 2` 帧新鲜有效观测并满足滞回退出条件。
- [ ] **验收 3（统一 L3 实际加速度与逐帧 Jerk 检验）**：
  - 刚进硬刹（`prev_a = -2.0 m/s²`）丢帧 → 按 `jerk_hard = 20.0 m/s³` 继续走 `-2 → -4 → -6`，直至接近停车边界平滑卸力停稳（无倒车）；
  - 恢复加速（`prev_a = +1.5 m/s²`）丢帧 → 按 `jerk_soft = 5.0 m/s³` 依次走 `+1.0 → +0.5 → 0.0`，中途升级软刹平滑衔接；
  - 对实际加速度序列计算逐帧 `|a[k] - a[k-1]| / dt`，在进入缺测、连续缺测、恢复观测、接近停车四处边界均不超对应档位 Jerk 上限。
- [ ] **验收 4（延迟裕度与独立调参）**：
  - `latency_margin = 0.20 s` 下让 TTC 先降后升，验证软刹 `3.2 / 3.7 s`、硬刹 `1.7 / 2.2 s` 的进入/退出帧及恰好等于退出阈值时保持原档位；
  - `latency_margin = 0.0` 严格复现既有 L3 切换序列；`+∞` 保持 `+∞`；缺测态仍走第 2.2 节策略，不与 `missing_grace_s` 耦合。

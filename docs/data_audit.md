# 数据审计 — `data/*.csv`（2026-10-20）

> **本文件与期中 DL1/DL2 无关**，属 11 月开工参考。期中只需知道一句：**数据已到位，格式对得上，不挡期中。**
> 审计对象：`data/ego_trajectory.csv`、`data/agent_trajectory_001.csv`、`data/agent_trajectory_002.csv`。
> 结论摘要：**loader 已被解锁（好消息）**；**不能用来校准 cut-in 几何（坏消息）**；**但意外得到一段真实"前车急刹 + 本车追尾"的冲突片段，可直接支撑主线论证（最大的好消息）**。

---

## 1. 清单与格式

| 文件 | 行数 | 时长 | `agent_type` | 内容 |
| --- | --- | --- | --- | --- |
| `ego_trajectory.csv` | 130 个采样 | 0–12.9 s @ 10 Hz | `ego` | 本车 |
| `agent_trajectory_001.csv` | 130 | 0–12.9 s | `vehicle` | 前方慢车（后来急刹） |
| `agent_trajectory_002.csv` | 130 | 0–12.9 s | `vehicle` | 右前方转向车 |

列：`timestamp, x, y, heading, velocity, acceleration, agent_type` —— 与 `load_toy` docstring 完全一致。单位 m / rad / m·s⁻¹；`dt` 严格 0.1 s、无 NaN、无重复时间戳、时间单调。**单位与 `pbs.types.Trajectory` 契约匹配，A 不需要做任何换算。**

> 注：3 个文件 = **1 个场景（1 ego + 2 agents）**，不是 3 个独立场景。`data/*` 已在 `.gitignore`（`data/*` + `!data/.gitkeep`），所以队友必须各自复制一份，不要试图提交。

## 2. 三个必须立刻做的集成决定

### 2.1 路径不匹配（现在 `load_toy()` 一定会 `FileNotFoundError`）

`src/pbs/data/loader.py` 默认 `DATA_DIR / "sample_trajectories"`，而文件在 `data/` 根下。**二选一**：

```powershell
# 方案 A（推荐，与 README / interfaces 的写法一致）
New-Item -ItemType Directory -Force data\sample_trajectories
Move-Item data\*.csv data\sample_trajectories\

# 方案 B（不改文件，改默认参数）
# load_toy(data_dir: Path = DATA_DIR) 并把 README 的路径一起改掉
```

### 2.2 `load_toy()` 必须**排除 ego**

`Scenario.agents` 里放 ego 会让 Oracle 的 `agent_id` 查询和最小距离统计同时出错（ego 和 agent 重合时距离恒为 0）。建议：

```python
# 语义：返回"周围 agent"，不含 ego
agent_id = 文件名 stem（如 "agent_trajectory_001"）
agent_type = CSV 里的 agent_type 列（"vehicle" / "ego"）
# 若将来要 ego → 另加 load_toy_ego()（interfaces 约定"只加不改"，允许新增函数）
```

### 2.3 本车速度列有噪声 → 预测器不要做两点差分

| 对象 | `|dv/dt|` p50 | p95 | max | 说明 |
| --- | --- | --- | --- | --- |
| ego | 2.18 m/s² | 6.40 | **8.67** | 平滑行驶却算出 8.7 m/s²，说明 `velocity` 列有抖动 |
| agent_001 | 0.00 | 1.60 | 1.60 | 速度是分片常数（只有 21 个不同值），加速度分片 −0.2 / −0.8 / −1.2 / −1.6 |
| agent_002 | 0.00 | 0.24 | 0.36 | 几乎匀速，末端开始转向（yaw rate 最高 12.6 °/s） |

**后果**：CA / 加速度估计若用 `(v[-1]-v[-2])/dt`，会拿到 ±2 m/s² 的假加速度，把"预测变差"全归因到数据噪声。
**对策**：所有 predictor 一律用**窗口最小二乘**估计速度/加速度（对最近 `m = 5~10` 帧拟合），不要两点差分。这一条恰好也是 `LinearRegressionPredictor` 的做法——可以把它从"L3 可选"提升为**默认估计器**，写进 report 的"实现选择"里。
另外 `heading` 列无抖动（ego yaw rate ≤ 0.77 °/s），CTRV 的 yaw-rate 估计不受影响。

## 3. 这批数据**能**支撑什么

### 3.1 `load_toy` + ADE/FDE（原本的 Day-1 任务，现在变成 12 行）

2 个 agent × 130 帧，按 horizon 1–30 步（0.1–3 s）、每 5 步取一个窗口 → 每个 horizon 约 25 个样本/agent。**够画一条带误差棒的曲线，但样本量小（要写清 N）**，不要拿它做显著性结论。

### 3.2 【新增，价值最高】一段真实的"前车急刹 → 本车追尾"片段

用 `agent_001` 反算的几何（ego 朝向下的纵向/横向距离）：

| t [s] | ego v | a1 v | 纵向间距 lon | 横向偏移 lat | 中心距 | 保险杠间距（按 4.5 m 车长） |
| --- | --- | --- | --- | --- | --- | --- |
| 0.0 | 5.09 | 4.00 | 15.21 | 1.64 | 15.30 | 10.71 |
| 5.0 | 5.09 | 4.00 | 10.06 | 0.64 | 10.08 | 5.56 |
| 9.0 | 5.20 | 4.00 | 6.01 | 0.44 | 6.03 | 1.51 |
| **10.5** | ~4.9 | 4.00 | **4.50** | — | 4.5 | **0.00（首次接触）** |
| 11.1 | ~5.0 | **4.00 → 开始刹** | — | — | — | — |
| 12.7 | — | 1.04 | — | — | **0.64（最小）** | **−5.31（重叠）** |

关键事实：

- t=0 时以**恒速假设**算出的 reactive TTC = `gap / closing = 15.21 / 1.09 = 13.9 s` → **"非常安全"**；
- 前车在 11.1 s 才以 −1.6 m/s² 刹车，而两车在 **10.5 s 几何上已经接触**；
- 全片最小中心距 0.64 m，ego 全程**未刹车**（v 一直在 4.4–5.6 m/s 之间抖）。

**这就是你们整个 project 论点的一个现成真实案例**：reactive 的 TTC 建立在前车**保持车速**的假设上，前车一刹车，13.9 s 的"安全"在 10 s 内归零。于是这一小段数据可以支撑三件事：

1. **reactive vs CA vs oracle 的真实对照**：CA 能从 −1.6 m/s² 的减速度里提前预判，reactive 只能等间距塌到阈值以下（那时已经来不及）。这正好回答 spec §4 给 P 的必答题"前车减速时 CV 让刹车偏早还是偏晚"。
2. **一个"该刹却没刹"的 demo**：把 `NeverBrakePolicy` 跑一遍确认它碰撞（`is_benign == False`），再用三种系统各跑一遍，展示各自是否避免碰撞、何时刹。比合成场景更有说服力，且成本极低（数据已经在手）。
3. **反方向的诚实对照**：这是一个**纯纵向**场景（lat < 1.64 m 且基本不变），**横向预测毫无用处**。把它和合成 cut-in 场景并排放，正好说明"prediction 的价值取决于冲突类型：纵向冲突靠加速度模型，横向冲突靠 2D 外推"——这比只说"prediction 有好处"高一个层次。

> 建议做成 `experiments/exp06_real_clip.py`，放在 W8（11-08 前），不要塞进本周冲刺。

## 4. 这批数据**不能**支撑什么（重要，别在 proposal 里夸口）

| 想说的话 | 能不能 | 原因 |
| --- | --- | --- |
| "用真实轨迹校准了 cut-in 几何（换道时长 / 峰值横向加速度）" | ❌ | **全片没有一次切入本车道的换道**。a1 的横向偏移从 1.64 → 0.44 m 是 ego 相对漂移造成的；a2 始终在 −4 到 −7 m 的右侧车道，末端是**转向**（yaw rate 12.6 °/s），不是换道。把 a2 的横向轨迹硬拟合成 quintic，RMSE 1.70 m、隐含峰值横向加速度 0.051 m/s²（明显不是换道）。 |
| "真实交通数据验证了 AEB 行为" | ❌ | 1 个场景、2 个 agent、12.9 s；低速（1.0–5.6 m/s，约 4–20 km/h），与你们 10–25 m/s 的边界网格差一个量级。 |
| "这是真实采集数据" | ⚠️ **先确认来源** | 统计特征像**课程生成的样例**：严格 0.1 s 规则采样、a1 速度只有 21 个不同值且加速度分片常数、ego 速度列有非物理抖动。写报告前问 TA，措辞用 **"course-provided sample trajectories"**，不要写 "real-world driving data"。 |

**因此 `docs/midterm_plan.md` §6.2 原计划（用真实数据校准 quintic 参数）作废**，替换为：
- quintic 的 `MAX_LAT_ACC = 3.0 m/s²` 改为**引用文献范围 + 敏感性分析**（2.0 / 3.0 / 4.5 各跑一遍，看结论是否翻转），并在 limitations 里写"未用真实换道数据校准"；
- 真实数据的角色改为**案例研究 + loader 验证**（§3.2）。

## 5. 对 proposal 措辞的影响

- §2 资源表：课程数据的"借了什么"改成 *a single real low-speed following/braking conflict for a qualitative case study and loader validation*，删掉 "lane-change statistics"。
- §4 "will demonstrate"：加一条"on a real (course-provided) low-speed conflict clip, a constant-velocity reactive TTC declares 13.9 s of clearance at t = 0 while the conflict occurs at 10.5 s — a case where acceleration-aware prediction matters and lateral prediction does not"。
- §4 "will not demonstrate"：加一条"our scenarios are **not** calibrated against real cut-in statistics; the cut-in family is synthetic and its kinematic plausibility is argued from literature bounds plus a sensitivity analysis"。
- §5 schedule：W8（11-08）加 `exp06_real_clip.py`。

## 6. 一句话交接

> 数据到位的意义不是"终于有真实数据了"，而是：① `load_toy` 从"阻塞项"变成 12 行；② 白送一个真实纵向冲突案例，让"reactive 的恒速假设会骗人"这句话有据可依；③ 但同时**打掉了**"用真实换道校准合成场景"这条路——cut-in 那一半必须老老实实写成合成 + 文献界 + 敏感性。

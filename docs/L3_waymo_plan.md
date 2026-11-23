# A 的 L3：真实数据（Waymo Open Motion）交叉验证 — 逐步执行手册

> **一句话**：把一段**真实**的切入（cut-in）搬进你已有的 `Scenario`，用同一套 `run_sim` 跑 Reactive / CV / （P 的 GRU），证明"prediction 让刹车提前了 X 秒"这句话在真实车载采集的数据上也能成立，同时把误刹算清楚。
>
> - 读者：A（M1 `pbs.data` 负责人）。会写 Python，不熟悉 Waymo。
> - 定位：`docs/spec.md` §1.3 的 **L3 加码**，可随时裁剪；**不影响主图**。
> - 时间盒：**6 个半天**，硬截止 **2026-11-29**（proposal §5 的 scope freeze）。
> - 版本：v2（2026-11-18）——**由 nuScenes 改为 Waymo Open Motion**，原 `L3_nuscenes_plan.md` 作废。
> - 完成一步就在 §13 打勾。

> **2026-11-23 实现更新**：A 的真实数据适配、首批 5+5、全候选审计、两个说明案例、三图与测试已完成。
> 实际结果见 [L3_waymo_case_study.md](L3_waymo_case_study.md)，实际命令见 README。
> 本手册旧骨架有三项纠正：SDC 用 `sdc_track_index`，时间戳有微小时钟抖动，几何切入不等于 dangerous。
> 原定四系统中的 CA/Oracle 和 GRU 仍待 P；本次仅报告实际已实现的 Reactive/CV。

---

## 0. TL;DR 与怎么用

### 0.1 三句话版本

1. Waymo 换掉的是**场景从哪来**（M1），不是**要不要仿真**（M3 照跑）。
2. 你只需要产出一个东西：`Scenario(kind="real_cut_in")`，它和 `make_scenario("cut_in", ...)` 的输出**结构完全一样**。
3. 产出五件：1 个 npz 快照、3 张图、1 张表、1 段报告文字、**两组片段（切入组 + 不切入的对照组）**。

### 0.2 如果你只有一天

只做 Step 0 → 2 → 5 → 8 → 10，然后画一张 timeline。其余全砍，报告里写"one real clip, qualitative only"。

### 0.3 三条纪律

| 纪律 | 为什么 |
|---|---|
| **几 GB 的 TFRecord 不进 git**，只提交 `tests/fixtures/*.npz`（几十 KB） | `data/*` 已 ignore；队友拉不动仓库 |
| **每个半天结束必须有一样能看见的东西**（一张图 / 一个绿色测试 / 一行数字） | 否则 11-29 你只有"在做了" |
| **主结论永远来自合成场景**，真实数据只负责"检验能否迁移" | 组员原话：合成场景画失效边界，真实场景检验预测器能不能迁移到车载采集的数据 |

---

## 1. L3 是什么，为什么换成 Waymo

### 1.1 位置

```
M1 场景来源 ──► M2 预测 ──► risk(TTC) ──► M3 仿真/决策 ──► M4 评估
   ↑
   └── 只有这一格被换成真实数据，后面三格一个字都不改
```

- 代码现状：`src/pbs/data/loader.py` 第 118 行 `load_nuscenes_tracks()` 抛 `NotImplementedError` → **改名/改实现为 `load_womd_tracks()`**。
- `src/pbs/data/__init__.py` 已导出它，跟着改一行。
- 你不动 `pbs.prediction` / `pbs.sim` / `pbs.eval`，一行都不动。

### 1.2 三种数据，一张表看懂

| | 合成 quintic（主线） | 课程 toy clip | **Waymo WOMD（本次）** |
|---|---|---|---|
| 谁产生 | 你，`make_scenario()` | 课程给的样例 | 真车采集（凤凰城/山景城等，车载） |
| 时间分辨率 | 10 Hz（`dt=0.1`） | 10 Hz | **10 Hz 原生**（不用插值！） |
| 一段多长 | 15 s，任意 | 12.9 s | **9.1 s / 91 帧** |
| 有切入吗 | 有（你造的） | **没有** | 有（要自己找，见 Step 3） |
| 车道 | `lane_width=3.5` 自己定 | 无 | **有 map_features 车道几何** |
| 遮挡/丢帧 | 无（解析式） | 无 | **有 `valid` 标记**，必须处理 |
| 真值 | 解析式，含加速度 | 只有 CSV 列 | 位置/朝向/速度 + 观测有效性 |
| 能撑 | 参数扫描、四边界图 | loader 验证、一段纵向急刹案例 | **真实切入的定性案例 + 运动学参数依据 + 预测器迁移检查** |
| 撑不了 | 代表性（"自证"） | 切入、代表性 | 统计结论（样本太少） |

**为什么不用 nuScenes 了（一句话）**：它的目标框标注只有 **2 Hz**（0.5 s 一格）。"刹车提前量"这个项目的核心量就是 0.1–0.5 s 量级，2 Hz 的粒度正好把这个结论吃掉——插值能让时间轴变细，但**不会增加真实观测**。Waymo 是 10 Hz，且带 `valid` 标记和车道几何，正好对应我们需要的三样东西。

（nuScenes 留在 §9 的**降级档**里当演示素材，不删；proposal 里 [9] 那条引用保留。）

### 1.3 有真实数据，为什么还要仿真

一句话：**录像只能回放，不能改**。

- 真实片段里那位司机当时是那么开的，你没法让他"早 0.5 秒踩"。而你要问的是"**换成我们的系统**会怎样"。
- "误刹"必须靠反事实（同一场景不刹车会不会撞），真实数据只有一条已发生的时间线。
- 主图是 (间距 × 相对速度) 平面上的边界，要几百个点；真实片段只给你几个点。
- 四个系统必须在**完全相同**的输入上比，只有风险判断不同（controlled comparison）。

**真实数据的三个合法角色（= 三条验收）**：

1. **迁移检查**：合成场景里成立的东西，在真实轨迹上还成立吗？（组员最想看的）
2. **参数依据**：真实片段的 `v_ego`、`rel_v`、换道时长、峰值横向加速度，落在 `MAX_LAT_ACC=3.0`、`V_EGO=15` 的邻域里 → 合成网格不是拍脑袋。
3. **误刹对照**：一组"邻车**不**切入"的片段（对照组），看系统会不会乱刹。

### 1.4 你**不需要**做的事

| 不做 | 为什么 |
|---|---|
| 读相机 / 激光 | 我们不做感知，spec §1.2 明确 out of scope |
| 训任何模型 | GRU 是 P 的，你只负责**喂够 20 帧历史** |
| 做数据集级评测 | 你只要 10–20 个片段，不是 10 万个 |
| 用真实数据重新校准 quintic | `docs/data_audit.md` §4 已论证走不通，别回头 |
| 把 GRU 写进 `pbs.prediction` | 那是 P 的文件，冲突概率高 |

---

## 2. 数据字典：一个 Waymo `Scenario` 里有什么

WOMD 的文件是 **TFRecord**，每条记录是一个序列化的 `Scenario`（protobuf）。一个 `Scenario` ≈ 一个 9.1 s 的十字路口/路段。

| 字段 | 含义 | 你要的 |
|---|---|---|
| `scenario_id` | 片段 id | 文件名/图标题 |
| `tracks` | 所有物体（**含本车**） | `id`、`object_type`、`states[91]` |
| `tracks[i].states[j]` | 第 j 帧的状态 | `center_x/center_y`、`heading`、`velocity_x/velocity_y`、**`valid`** |
| `map_features` | 车道/路沿/人行横道几何 | `lane` 的 `polyline`（车道中心线） |
| `dynamic_map_states` | 红绿灯状态 | 本章不用 |

三条必须记住的：

1. **时间轴不是从 0 开始的**。91 帧 = 9.1 s，官方"当前时刻"是 **index 10**（1 s 历史 + 8 s 未来）。我们要的 anchor 一般比 10 更靠后，因为 **GRU 要 20 帧历史**（见 §2.1）。
2. **`valid=False` 表示这一帧没观测到**（遮挡/出画），不是"速度为零"。绝不能当 0 用，也不能跨空洞插值。
3. **`tracks` 里包含本车（SDC）**。把它塞进 `Scenario.agents` 会让 `OraclePredictor` 和最小距离统计同时出错（和 `load_toy` 排除 ego 是同一个坑）。

### 2.1 和 GRU 对齐：为什么 anchor 必须 ≥ 20

| 事实 | 数字 | 后果 |
|---|---|---|
| GRU 输入 | **20 帧历史**（2.0 s @ 10 Hz） | anchor 必须在第 20 帧之后 |
| 官方预测起点 | index 10（只有 1 s 历史） | **不能直接用官方起点**，要自己重新切窗口 |
| 一段总长 | 91 帧（index 0–90） | `anchor + 仿真步数 ≤ 90` |
| 3 s 预测时域 | 30 帧 | `anchor ≤ 60` |

所以 anchor 的合法区间是 **`[20, 90 − duration·10]`**。取 `duration = 6.0 s` → anchor ∈ `[20, 30]`，**11 个可选 anchor**。选哪个：让"跨线时刻"落在 `anchor + 2~3 s` 附近。

> 这一段就是组员说的"在有完整轨迹的片段里重新切窗口，筛选越线前有足够有效历史、越线后也有足够未来帧的样本"。**它决定了这件事成不成**，别跳过。

---

## 3. 最终交付物清单

| # | 文件 | 进 git？ | 怎么验收 |
|---|---|---|---|
| 1 | `src/pbs/data/womd.py` | ✅ | `python -c "import pbs.data.womd"` 不报错 |
| 2 | `src/pbs/data/loader.py`：`load_womd_tracks` 落地 | ✅ | 返回非空 `list[Trajectory]` |
| 3 | `src/pbs/data/__init__.py` 导出改名 | ✅ | `from pbs.data import load_womd_tracks` 可用 |
| 4 | `scripts/inspect_womd.py` | ✅ | 打印字段 + 一张 BEV |
| 5 | `scripts/export_womd_clips.py` | ✅ | 生成 `tests/fixtures/womd_cut_in_<id>.npz` 等 |
| 6 | `tests/fixtures/womd_*.npz` | ✅（每份几十 KB） | — |
| 7 | `tests/test_womd.py` | ✅ | `pytest tests/test_womd.py -q` 全绿，**不需要** TFRecord |
| 8 | `experiments/exp07_real_clip.py` | ✅ | 3 张图 + 2 个 csv |
| 9 | `docs/L3_waymo_case_study.md` | ✅ | 一页，含 will / will not |
| 10 | `README.md`（数据说明 + 实验表）、`docs/interfaces.md`、`docs/contributions.md`、`docs/references_candidates.md`（加 Waymo 引用） | ✅ | diff 看得到 |

---

## 4. 逐步执行

> 每步格式：**目标 / 动手 / ✅ 验收 / ⚠️ 坑**。仓库根目录 = `...\AIE1903_Project\gabey`。

### Step 0 — 环境与数据（半天）

**目标**：能读出第一个 `Scenario`，且不把几 GB 塞进 git。

**先做决定：走哪条读数据的路**

| 路线 | 适合 | 代价 |
|---|---|---|
| **A. 官方包**（`pip install waymo-open-dataset-tf-2-12-0` + `tensorflow`） | Linux / WSL2 / 队友的机器 | Windows 上装不上（TF 2.11 起不支持原生 Windows；官方 wheel 基本只有 Linux） |
| **B. 纯 Python**（`grpcio-tools` 编 proto + 12 行 TFRecord reader） | **你的 Windows** | 多一步 `protoc`，但只多 5 分钟 |

**推荐 B**。理由和上次一样：不要把 deadline 花在依赖冲突上。

```powershell
# B 路线：只需要两个 pip 包
pip install grpcio-tools protobuf

# 1) 拿到 proto（只拷 protos 目录，仓库不用全克隆）
#    https://github.com/waymo-research/waymo-open-dataset/tree/master/src/waymo_open_dataset/protos
#    放到   vendor/waymo_open_dataset/protos/*.proto

# 2) 编译成 Python 类（一次性）
python -m grpc_tools.protoc -Ivendor --python_out=vendor `
  vendor\waymo_open_dataset\protos\scenario.proto

# 3) 下载数据 → 见【附录 A】。Waymo 必须 Google 登录 + 同意条款，
#    匿名直链是 403（别在这上面耗时间）；
#    拿不到就走 A.4 的 Argoverse 2 兜底：公开 S3、无需账号、单场景 200 KB。
#    目标：data/womd/validation.tfrecord-00000-of-00xxx（1 个分片就够）

# 4) 确认被忽略（应该没有输出）
git status --short | Select-String womd
```

- ✅ **验收**：`python -c "from waymo_open_dataset.protos import scenario_pb2; print('ok')"`（在 `vendor` 进 `sys.path` 后）不报错；`data/womd/` 在 `git status` 里看不见。
- ⚠️ **坑**：`protos/` 里 `scenario.proto` 会 import 同目录的 `map.proto` / `vector.proto`，**整个目录一起拷**，保持相对路径，否则 `protoc` 报 `File not found`。WOMD 的版本（v1.1 / v1.2 / v1.3）要和 proto 对得上，**别混**。

---

### Step 1 — 读出第一个 Scenario，确认字段与 SDC（1 小时）

**目标**：不写业务逻辑，先确认"字段名和我以为的一样"。proto 各版本字段会漂，所以这一步是**自检**，不是走过场。

```python
from waymo_open_dataset.protos import scenario_pb2

def read_scenarios(path, limit=None):
    """Minimal TFRecord reader: uint64 length, 4B crc, payload, 4B crc."""
    with open(path, "rb") as fh:
        n = 0
        while limit is None or n < limit:
            head = fh.read(8)
            if len(head) < 8:
                return
            (length,) = struct.unpack("<Q", head)
            fh.read(4)                       # CRC of length, unchecked
            payload = fh.read(length)
            fh.read(4)                       # CRC of payload, unchecked
            yield scenario_pb2.Scenario.FromString(payload)
            n += 1

sc = next(read_scenarios("data/womd/validation.tfrecord-00000-of-00xxx"))
print([f.name for f in sc.DESCRIPTOR.fields])
print([f.name for f in sc.tracks[0].DESCRIPTOR.fields])
print([f.name for f in sc.tracks[0].states[0].DESCRIPTOR.fields])
print(len(sc.tracks), "tracks;", len(sc.tracks[0].states), "states")
```

**确认 SDC（本车）**：必须使用 **`tracks[sc.sdc_track_index]`**。本机首条记录的索引为 10，不能固定成 0：

- `tracks[sc.sdc_track_index]` 的速度连续、91 帧基本 valid；
- 它的轨迹长度量级是几百米（不是 3 m）；
- 画出来它是一条"开车"的曲线，不是有人原地站着。

实现统一用 `sdc_index(sc)` 校验元数据；字段缺失或越界时直接报错，拒绝猜测本车。

- ✅ **验收**：打印出 `tracks` / `states` 的真实字段名；数出 91 个 state；确认官方 `sdc_track_index` 指向 SDC。
- ⚠️ **坑**：`object_type` 是 enum（`TYPE_VEHICLE` / `TYPE_PEDESTRIAN` / `TYPE_CYCLIST` / `TYPE_OTHER`），打印出来是数字，要 `.name`；`scenario_id` 是字符串但可能含 `-`，别当文件名直接用。

---

### Step 2 — 先看一眼：BEV 图（1 小时）

**目标**：建 `Scenario` 之前，先确认"数据在我脑子里长什么样"。

**动手**：画一个片段：ego 轨迹 + 所有 `TYPE_VEHICLE` 的轨迹 + `map_features` 里 `lane` 的 polyline。`ax.set_aspect("equal")`。

- ✅ **验收**：能看出车道线、ego 在哪条车道、哪辆车正在换道。
- ⚠️ **坑**：地图 polyline 点很多（一条车道几十个点），`for lane in sc.map_features: if lane.HasField("lane")` 再取 `lane.lane.polyline`；`set_aspect("equal")` 不设会把直角路口画成斜的。

---

### Step 3 — 找两组片段（半天，**最容易卡住**）

**目标**：不是找"1 个片段"，而是找**两组**：

| 组 | 定义 | 用途 | 最少数量 |
|---|---|---|---|
| **切入组** | 邻车从相邻车道横跨到 ego 车道，且在 ego 前方 | 看 prediction 能不能提前看见 | 3–5 段 |
| **对照组** | 邻车**始终**在相邻车道，ego 正常通过 | 看系统会不会**误刹** | 3–5 段 |

**为什么必须有对照组**：只有切入组的话，任何系统"刹了"都算对，误刹率恒为 0——这和你们合成场景里"必须含 benign 样本"（spec §1.5）是同一个道理，只是换成真实数据。

**切入判据（可复现，写进文档）**。在 **ego 车体系**下逐帧算（用当帧 ego 位姿）：

```
dx, dy = 目标 - ego
lon =  dx·cos(yaw) + dy·sin(yaw)      # 前向距离
lat = -dx·sin(yaw) + dy·cos(yaw)      # 左正右负

判为"切入"当且仅当存在帧 k：
  ① |lat[k]| ≤ 0.9 m               （已经进本车道）
  ② lon[k] ∈ [5, 40] m             （在正前方；远了没意义，近了已经撞了）
  ③ ∃ j < k：t[k]−t[j] ≤ 5 s，|lat[j]| ≥ 2.0 m，sign(lat[j]) == sign(lat[k-1])
                                    （5 s 内从相邻车道横跨 > 2 m）
  ④ object_type == TYPE_VEHICLE
  ⑤ ego 速度 > 3 m/s，且 ego 航向变化率 < 5 °/s（排除路口转弯/停车场）
```

**有车道几何的话（推荐）**：把 `lat` 换成"到 ego 所在 lane polyline 的横向距离"，比"相对 ego 航向"更干净，弯道上也成立。

**输出**：`outputs/logs/exp07_candidates.csv`，列 `clip, group, track_id, t_cross_s, lon_m, lc_T_s, v_rel, n_valid_frames`。

- ✅ **验收**：两组各 ≥ 3 行；每行都能对着 ①–⑤ 念出理由。
- ⚠️ **坑**
  - **断帧**：`valid=False` 的帧不能插值，按"连续 valid 段"切，只保留段长 ≥ 需要的窗口（约 50 帧）。
  - 一条都没有 → 立刻走 §10 的**降级档 2**（换成真实前车急刹），别硬找。
  - 别只挑"好看"的片段——挑的时候就要记下 `n_valid_frames`，后面 GRU 的 20 帧门槛靠它。

---

### Step 4 — 选 anchor 与窗口（1 小时）

**目标**：定下四个数。

| 参数 | 值 | 理由 |
|---|---|---|
| `need_history` | **20 帧**（2.0 s） | GRU 的硬要求；也是 window 预测器的拟合窗口 |
| `duration` | **6.0 s** | 91 帧太短，6 s 是"能看清刹车过程"和"anchor 有得选"的折中 |
| `need_future` | `duration·10 = 60` 帧 | `_validate` 要求 agent 覆盖到 `t = duration` |
| `anchor` | `t_cross − 2~3 s` 且 ∈ `[20, 30]` | 前面留 2–3 s"看起来安全"的平静期 |

**时间轴**：`t[j] = (j − anchor) / 10.0`，于是 `t ∈ [−2.0, +6.0]`，**正好满足** `_validate` 的 `agent.t[0] ≤ -latency`（latency ≤ 2 s 都能做）。

- ✅ **验收**：窗口内 ego 与 cutter 的所有帧都 `valid=True`；`anchor + 60 ≤ 90`。
- ⚠️ **坑**：别等画完图才发现 cutter 在 anchor+40 就出画了——那时 `truth must cover duration` 会让你把三张图全重做。

---

### Step 5 — 转到 ego 车体系 + 定 ego_path（半天，**概念最关键**）

**目标**：把场景坐标变成"ego 在原点、朝 +x"，与 `make_scenario()` 输出同一个世界。

```python
c, s = np.cos(-yaw_s), np.sin(-yaw_s)      # yaw_s = ego 在 anchor 帧的朝向
d = xy_world - ego_xy_s
xy = np.column_stack((c * d[:, 0] - s * d[:, 1],
                      s * d[:, 0] + c * d[:, 1]))
```

`ego_start = (0.0, 0.0, 0.0, v0)`，`v0` 用 anchor 附近 3 帧的速度平均（**不要**单点差分）。

**`ego_path` 三选一，按代价从低到高**：

| 方案 | 做法 | 什么时候用 |
|---|---|---|
| **1. 直线**（默认） | `[[0,0],[L,0]]`，`L = max(200, v0·duration + 100)` | ego 在窗口内横向偏离 < 1.0 m（先做这个检查） |
| 2. ego 自己的轨迹 | 把 ego 的 91 帧轨迹当折线（**必须**再外推 60 m） | 轻微弯道，且偏离 < 3 m |
| 3. 车道中心线 | 从 `map_features` 取 ego 所在 lane 的 polyline | 明显弯道；最准，但要写 lane 匹配 |

`_validate` 的硬要求：`ego_path[0] == ego_start[:2]`、heading 与首段同向、长度够 `ego_rollout` 看 3 s 前方。

- ✅ **验收**：BEV 图里 ego 沿 x 轴；cutter 从 `y ≈ ±3.5 m` 斜插到 `y ≈ 0`；直线方案的 `max|ego_y| < 1.0 m`。
- ⚠️ **坑**
  - **不要**把场景坐标直接当世界坐标。Reactive 的"本车道"按 ego 航向投影算，坐标系差 90° 会让所有车都"不在本车道"，TTC 恒为 ∞，你还以为是 bug。
  - Waymo 的 y 轴方向与我们的 `lat` 符号不要混：`lat > 0` = ego 左侧。
  - `heading` 会 ±π 跳变 → `np.unwrap`。

---

### Step 6 — 处理 `valid` 与建 `Trajectory`（半天）

**目标**：**不插值**（10 Hz 原生），只做清洗 + 转换。这比 nuScenes 那版省掉整整一步。

```python
Trajectory(
    t=t, x=xy[:, 0], y=xy[:, 1],
    v=np.hypot(vx, vy), heading=np.unwrap(np.arctan2(vy, vx)),
    agent_type="vehicle",     # 由 object_type 映射
    agent_id=f"track{track_id}",
    accel=None,               # 真实数据没有可靠加速度，别填
)
```

- 速度直接用 Waymo 给的 `velocity_x/velocity_y` 转到 ego 帧，**不要**对位置差分（差分会把观测噪声放大，`docs/data_audit.md` 已经吃过一次这个亏）。
- `valid=False` 的帧：**整段丢弃**，不要补。留下的段必须连续覆盖 `[−2.0, +6.0]`。

- ✅ **验收**：每个 agent 的 `t` 严格递增、无 NaN、长度 = 81（−2.0 到 +6.0 是 8.1 s → 81 个点）；`v` 与 Waymo 原始速度（转到 ego 帧后）逐点一致。
- ⚠️ **坑**
  - `len(t) = 81` 这个数要自己数清楚：`anchor−20` 到 `anchor+60` 共 81 帧，差一帧就会在后面 `_validate` 或 `prediction_slices` 报错。
  - `agent_type` 别一律写 `"vehicle"`——行人/自行车在后面 `crossing` 类片段里要用。
  - `accel=None` 不要填 0：`None` 是"未知"，0 是"确定没加速"，`prediction_slices` 会据此决定给不给 `truth_accel`。

---

### Step 7 — 建成 `Scenario`（半天）

```python
Scenario(
    ego_start=(0.0, 0.0, 0.0, v0),
    ego_path=ego_path,
    agents=[cutter],                       # 可只放 cutter，也可放全部车辆
    dt=0.1,
    params={"duration": 6.0, "v_ego": v0, "rel_v": rel_v, "lc_T_s": lc_T,
            "peak_lat_acc": peak_lat_acc, "source": "Waymo-WOMD",
            "scenario_id": sc.scenario_id, "anchor": anchor,
            "group": "cut_in"},
    kind="real_cut_in",
    lane_width=3.5,
)
```

四条硬约束（`simulator._validate` 直接抛错）：

| 约束 | 报错 | 修法 |
|---|---|---|
| `ego_path[0] == ego_start[:2]`，heading 与首段同向 | `position/heading must match` | 路径从原点起、沿 +x |
| `duration` 是 `dt` 的整数倍 | `duration must be an integer multiple` | 6.0 |
| **每个 agent** `t[-1] >= duration` | `truth must cover duration` | 窗口截到 `anchor+60` |
| `latency > 0` 时 `t[0] <= -latency` | `MissingObservationError` | 窗口从 `anchor−20` 起（自动满足） |

- ✅ **验收**：`run_sim(sc, ReactiveAssessor(), NeverBrakePolicy(), duration=6.0)` 不抛异常，`len(log.t) == 61`。
- ⚠️ **坑**
  - `kind` 用新值 `"real_cut_in"` / `"real_control"`。**不要**并进 `KINDS` / `DEFAULT_GRID` / `scenario_set()`：那 96 个场景是 D 的分母。
  - `agent_id` 必须唯一且稳定（`track{id}`）；`OraclePredictor` 靠它查真值。
  - 对照组片段的 `kind` 用 `"real_control"`，别硬塞成 `cut_in`——后面统计误刹要按组分开。

---

### Step 8 — 冻结成 npz（1 小时，**必做**）

**目标**：让队友、CI、未来的你**都不需要**几 GB 的 TFRecord。

```python
np.savez_compressed(
    "tests/fixtures/womd_cut_in_<scenario_id>.npz",
    ego_start=sc.ego_start, ego_path=sc.ego_path,
    t=..., x=..., y=..., v=..., heading=..., agent_type=..., agent_id=...,
    params_json=np.array(json.dumps(sc.params)),
    kind=np.array(sc.kind), lane_width=np.array(sc.lane_width), dt=np.array(sc.dt),
)
```

放 `tests/fixtures/`，**不要**放 `data/`（`data/*` 被 ignore，提交不上）。

- ✅ **验收**：每份 npz < 200 KB；把 `data/womd` 改名后，`pytest tests/test_womd.py` 仍全绿。
- ⚠️ **坑**：`np.savez` 存 `dict` 会变成 object array，`allow_pickle=False` 读不回来 → `params` 先 `json.dumps`。

---

### Step 9 — 端到端：系统对比 + ADE/FDE（半天）

**9.1 先核对系统清单**（30 秒，别跳过）

```powershell
python -c "from pbs.systems import SYSTEMS; print(list(SYSTEMS))"
```

- 输出里**没有** `gru` → GRU 还在 P 的分支上（当前 `src/pbs/prediction/` 里只有 CV/CA/CTRV/LR/Oracle）。先跑 `reactive / cv / ca / oracle`，等 P 合并后加一行：

```python
# pbs/systems.py（P 的文件，您只加一行，改前打个招呼）
SYSTEMS["gru"] = lambda: PredictiveAssessor(GRUPredictor.load(...))
```

- 输出里**有** `gru` → 直接加进对比列表。**注意 GRU 的输入约定**（20 帧 × 2D 位置+速度）：你在 Step 4 留的 20 帧历史就是为了它。

**9.2 仿真对比**（刹车提前量与误刹）

```python
sc = load_clip("tests/fixtures/womd_cut_in_<id>.npz")
benign = is_benign(sc)                       # 不刹车也不撞 ⇒ benign
for name in ("reactive", "cv", "ca", "oracle"):   # + "gru"
    log = run_sim(sc, make_system(name), HysteresisPolicy(), duration=sc.params["duration"])
    print(name, evaluate(log, benign))
```

**9.3 ADE / FDE**（预测误差，真值是录像里的未来）

```python
slices = prediction_slices(sc, history=2.0, horizon=3.0, stride=0.5)
for name in ("cv", "ca"):                    # + "gru"
    predictor = make_system(name).predictor
    errs = [(ade(predictor.predict(sl.history, 30, 0.1), sl.truth),
             fde(predictor.predict(sl.history, 30, 0.1), sl.truth)) for sl in slices]
```

- ✅ **验收**：切入组里 `reactive` 的 `brake_onset_t` 明显晚于 `cv`（或干脆不刹）；对照组里**所有系统都不刹**；`slices` 数量 ≥ 5。
- ⚠️ **坑**
  - 用 `HysteresisPolicy` 而不是 `TTCThresholdPolicy`：后者会点刹抖动（`docs/midterm_plan.md` 附录 A.3 第 1 条），`brake_onset_t` 全是噪声。
  - **GRU 历史不足的回退要计数**：若 GRU 内部有"历史不够就退回 CV"的逻辑，让 P 给出一个计数器；拿不到就在 clip 级报告"N 个片段因历史不足被排除"。这正是组员提醒的那件事——**别让整组结果里有一半其实是 CV 在预测**。
  - `is_benign` 内部也跑一次 `run_sim`，别在循环里重复算。

---

### Step 10 — 三张图 + 两张表（半天）

详见 §6。

```powershell
python experiments\exp07_real_clip.py
```

- ✅ **验收**：三张图能在 30 秒内讲完；`exp07_real_clip.csv` 里每个系统一行，`exp07_real_control.csv` 里对照组单独一张。
- ⚠️ **坑**：中文标题变方块 → 图内全英文；`matplotlib.use("Agg")`。

---

### Step 11 — 单测（半天，和 Step 10 并做）

```powershell
pytest tests\test_womd.py -q
```

8 个测试见 §5.4。

- ✅ **验收**：8 个全绿；**把 `data/womd` 改名后仍然全绿**。
- ⚠️ **坑**：测试里不要 `import scenario_pb2`（否则 CI 要先编 proto）；只读 npz。

---

### Step 12 — 文档与同步（半天）

1. `docs/L3_waymo_case_study.md`：一页，含 §8 的 will / will not。
2. `README.md`：数据说明把"nuScenes mini（可选，L3）"改成 Waymo；实验表加 `exp07_real_clip.py`。
3. `docs/interfaces.md`：函数表加 `M1 | load_womd_tracks(path) / real_scenario(...) | A | ✅ L3（可选）`。
4. `docs/references_candidates.md` + `proposal.md` §2：加 **Waymo Open Motion Dataset** 的引用（Ettinger et al., "Waymo Open Dataset: An autonomous driving dataset", 2019 / WOMD 2021 blog）。**别只改代码不改引用**。
5. `docs/contributions.md`、`docs/ai_disclosure.md` 各一行。

- ✅ **验收**：`git diff` 只有该改的行；`pytest -q` 全绿；`ruff check src tests` 干净。
- ⚠️ **坑**：`docs/A_baseline_handoff.md` §8 那张表还写着"`load_nuscenes_tracks` 保持 `NotImplementedError`"——**改成 Waymo 并打勾**，否则文档和代码互相打脸。

---

## 5. 代码骨架

### 5.1 `src/pbs/data/womd.py`

```python
"""M1 — Waymo Open Motion Dataset reader (L3, optional). Owner: A.

TFRecord of `Scenario` protos. No TensorFlow: a 12-line record reader + protoc-generated
`scenario_pb2` is enough. Time: seconds. Length: metres. Frame: ego frame at anchor.
"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from pbs.data.scenarios import DT, LANE_WIDTH
from pbs.types import Scenario, Trajectory

SDC_INDEX = 0          # verified in Step 1
HISTORY = 20           # frames the GRU needs (2.0 s @ 10 Hz)
DT_WOMD = 0.1          # 10 Hz


def read_scenarios(path, limit: int | None = None):
    """Yield Scenario protos from a TFRecord file (CRC deliberately unchecked)."""
    from waymo_open_dataset.protos import scenario_pb2

    with Path(path).open("rb") as fh:
        n = 0
        while limit is None or n < limit:
            head = fh.read(8)
            if len(head) < 8:
                return
            (length,) = struct.unpack("<Q", head)
            fh.read(4)
            payload = fh.read(length)
            fh.read(4)
            yield scenario_pb2.Scenario.FromString(payload)
            n += 1


def valid_span(states) -> np.ndarray:
    """Boolean mask of observed frames (`valid` means 'seen', not 'stationary')."""
    return np.array([st.valid for st in states], dtype=bool)


def track_arrays(track):
    """(N,2) centres, (N,) yaw, (N,2) velocities, (N,) valid for one track."""
    xy = np.array([[st.center_x, st.center_y] for st in track.states], dtype=float)
    yaw = np.unwrap(np.array([st.heading for st in track.states], dtype=float))
    vel = np.array([[st.velocity_x, st.velocity_y] for st in track.states], dtype=float)
    return xy, yaw, vel, valid_span(track.states)


def to_ego_frame(xy, ego_xy0, yaw0):
    c, s = np.cos(-yaw0), np.sin(-yaw0)
    d = np.asarray(xy, float) - np.asarray(ego_xy0, float)
    return np.column_stack((c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]))


def anchor_window(valid_ego, valid_agents, *, duration=6.0, history=HISTORY,
                  lo=20, hi=None):
    """Largest anchor in [lo, hi] whose whole [anchor-history, anchor+duration*10] is valid."""
    n_future = int(round(duration / DT_WOMD))
    hi = 90 - n_future if hi is None else min(hi, 90 - n_future)
    for anchor in range(lo, hi + 1):
        window = slice(anchor - history, anchor + n_future + 1)
        if valid_ego[window].all() and valid_agents[window].all():
            return anchor
    return None


# scan_cut_ins(...) / build_scenario(...) / save_clip(...) / load_clip(...) 按 Step 3-8 写。
```

### 5.2 `scripts/inspect_womd.py`

```python
"""Print proto fields, one scenario summary, and a BEV figure."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vendor"))
from pbs.data.womd import read_scenarios, track_arrays  # noqa: E402

RECORD = Path(__file__).resolve().parents[1] / "data" / "womd" / "validation.tfrecord-00000-of-00xxx"
sc = next(read_scenarios(RECORD, limit=1))

print("scenario_id:", sc.scenario_id)
print("scenario fields:", [f.name for f in sc.DESCRIPTOR.fields])
print("track fields:   ", [f.name for f in sc.tracks[0].DESCRIPTOR.fields])
print("state fields:   ", [f.name for f in sc.tracks[0].states[0].DESCRIPTOR.fields])

for tr in sc.tracks:
    xy, yaw, vel, valid = track_arrays(tr)
    print(f"track {tr.id:>3} {tr.object_type} valid={valid.sum():>2} "
          f"x=[{np.nanmin(xy[:, 0]):.0f},{np.nanmax(xy[:, 0]):.0f}]")
```

### 5.3 `scripts/export_womd_clips.py` 与 `experiments/exp07_real_clip.py`

沿用 §4 Step 8/9 的调用，输出固定为：

```
tests/fixtures/womd_cut_in_<id>.npz      × 3–5
tests/fixtures/womd_control_<id>.npz     × 3–5
outputs/figures/exp07_real_timeline.png
outputs/figures/exp07_real_ade.png
outputs/figures/exp07_real_vs_synth.png
outputs/logs/exp07_real_clip.csv
outputs/logs/exp07_real_control.csv
```

### 5.4 `tests/test_womd.py` — 8 个测试

| # | 测试 | 断言 |
|---|---|---|
| 1 | `test_fixture_loads` | npz 可读，`Scenario` 字段齐全 |
| 2 | `test_ego_starts_at_origin` | `ego_start[:3] == (0,0,0)`，`ego_path[0] == (0,0)` |
| 3 | `test_time_grid_is_minus2_to_6` | `t[0] ≈ −2.0`、`t[-1] ≈ 6.0`、`len(t) == 81`、`diff ≈ 0.1` |
| 4 | `test_no_nan_unique_ids` | 无 NaN；`agent_id` 唯一 |
| 5 | `test_history_is_long_enough_for_gru` | 每个 agent 在 `t ≤ 0` 有 **≥ 20** 个连续采样点 ← **GRU 契约** |
| 6 | `test_scenario_passes_simulator_validation` | `run_sim(..., NeverBrakePolicy)` 不抛异常，`len(log.t) == 61` |
| 7 | `test_cut_in_group_has_conflict` / `test_control_group_is_benign` | 切入组 never-brake 会撞；对照组 never-brake 不撞 ← **两个方向都守** |
| 8 | `test_npz_roundtrip` | `save_clip` → `load_clip` 逐位相等 |

第 5、7 两个最值钱：一个守 GRU 的输入契约，一个守"两组片段真的不同"。

---

## 6. 三张图 + 两张表

| 图 | 文件 | 画什么 | 一句话 |
|---|---|---|---|
| 1 | `exp07_real_timeline.png` | 上：ego 速度；下：ego–cutter 中心距。竖线 = Reactive / CV / GRU 的 `brake_onset_t`，另加一条 = 跨线时刻 | "同一个真实切入，CV/GRU 比 Reactive 早 X 秒刹车" |
| 2 | `exp07_real_ade.png` | ADE vs 预测时域（1/2/3 s），每条曲线一个预测器，带误差棒 | "真实数据上 GRU 比 CV 好多少（或不好）" |
| 3 | `exp07_real_vs_synth.png` | 把真实片段的 `(rel_v, 初始间距)` 点叠在 `exp00_danger_map.png` 的合成网格上 | "合成网格不是凭空画的，真实切入就落在这片区域" |
| 表 1 | `exp07_real_clip.csv` | 切入组：每系统一行（collision / min_dist / brake_onset_t / lead_time / ADE@3s） | 报告数字来源 |
| 表 2 | `exp07_real_control.csv` | 对照组：每系统一行（是否刹车 ⇒ 误刹） | 误刹的分母 |

第 3 张图是**加分关键**：它把真实数据和合成场景接上，直接回答"场景代表性"那条风险。

---

## 7. 时间表（6 个半天，锚定 11-29）

| 日期 | 做什么 | 当天必须有 |
|---|---|---|
| 11-19 上午 | Step 0–1 环境、proto、读出一个 Scenario | 字段打印 + SDC 确认 |
| 11-19 下午 | Step 2 BEV | 一张 BEV png |
| 11-20 | Step 3 两组片段 | `exp07_candidates.csv` 两组各 ≥3 行 |
| 11-21 | Step 4–5 anchor + ego 帧 | 单测 2/3 绿 |
| 11-22 | Step 6–7 Trajectory/Scenario | `run_sim` 跑通（单测 6 绿） |
| 11-25 | Step 8–9 npz + 四系统 + ADE | 三张图 + 两张 csv |
| 11-26 | Step 11 单测全绿 + Step 12 文档 | `pytest` 全绿，case study 一页 |
| 11-27~28 | 缓冲（就是用来救 Step 3 的） | — |
| **11-29** | **scope freeze** | 不再加东西 |

---

## 8. 报告怎么写（模板）

### 8.1 will demonstrate

> On N real cut-ins replayed from the Waymo Open Motion Dataset (10 Hz), the reactive baseline's longitudinal TTC stays infinite until the cutter crosses the ego-lane boundary at t = X s, while the 2-D constant-velocity predictor raises the alarm Y s earlier; driven by the same TTC policy and the same braking limit, the ego brakes Z s later under the reactive baseline. On N′ no-cut-in control clips, neither system brakes. Prediction error (ADE/FDE at 1–3 s) is measured on the same replayed futures, so the comparison between CV and the learned predictor uses identical inputs.

### 8.2 will not demonstrate

> Agents are **replayed** and do not react to the ego's braking, so these numbers describe AEB performance **under real-trajectory replay**, not closed-loop traffic interaction. The sample is small (N ≤ 5 per group); we do not claim statistical significance, nor that the synthetic grid reproduces real traffic statistics. We do not calibrate the quintic lane change against real data.

### 8.3 图注模板

> **Figure X.** One real cut-in from the Waymo Open Motion Dataset (`scenario_id`). Shaded: the interval in which the cutter is outside the ego-lane boundary (|y| > 1.75 m). The reactive TTC remains infinite throughout that interval; the CV estimate drops below the 2 s threshold at t = Y s. Ego starts braking at 2.0 s (reactive) and 1.5 s (CV). Ego path is the straight-line approximation (maximum ego lateral deviation 0.4 m over the clip).

---

## 9. 降级档

| 档 | 内容 | 触发 | 报告里怎么写 |
|---|---|---|---|
| **1 全量** | 切入组 + 对照组 + 四系统 + 三张图 | 正常 | §8.1 |
| **2 换冲突类型** | 真实**前车急刹**片段（纵向），Reactive vs CA | Step 3 找不到切入 | "on a real longitudinal conflict, a constant-velocity TTC reports clearance while the gap collapses" |
| **3 只做 loader** | `load_womd_tracks` + 单测 + 一张 BEV | 时间不够 | "the M1 contract was validated on Waymo scenarios; no quantitative claim" |
| **4 不做** | 保持 `NotImplementedError`，limitations 明写 | 11-26 仍未跑通 | "not attempted; all conclusions rest on synthetic scenarios" |

**判断线**：11-22 还没跑通 `run_sim` → 跳档 3；11-26 还没图 → 跳档 4。

---

## 10. 12 个坑

| # | 现象 | 原因 | 修法 |
|---|---|---|---|
| 1 | `ModuleNotFoundError: waymo_open_dataset` | proto 没编 / `vendor` 不在 `sys.path` | Step 0 的命令 + `sys.path.insert(0, "vendor")` |
| 2 | `protoc` 报 `File not found` | `protos/` 没整目录拷 | 保持相对路径整目录拷 |
| 3 | 所有 TTC = ∞，Reactive 从不刹车 | 场景坐标没转到 ego 帧 | Step 5 |
| 4 | `truth must cover duration` | cutter 中途 `valid=False` 或出画 | 缩 `duration` 或换 anchor |
| 5 | `MissingObservationError` | 历史不足 20 帧 | anchor ≥ 20 |
| 6 | `ego_path is too short` | 路径 < `v·(duration + 3 s)` | `max(200, v*duration + 100)` |
| 7 | 速度看着像台阶 | 用了位置差分 | 用 `velocity_x/velocity_y` |
| 8 | 轨迹里出现瞬移 | 跨 `valid=False` 插值 | 按连续 valid 段切 |
| 9 | `oracle` 和 `cv` 一样 | `agent_id` 撞了 / 真值没覆盖 | `track{id}`；检查窗口 |
| 10 | GRU 一半在"回退到 CV" | 历史不足 20 帧 | 单测 5 + 报告排除数 |
| 11 | 队友 clone 后跑不了测试 | npz 放 `data/` | 放 `tests/fixtures/` |
| 12 | 图里中文变方块 | 没设字体 | 图内全英文 |

---

## 11. 老师/组员可能问的 8 个问题

| 问 | 答 |
|---|---|
| 为什么用 Waymo 而不是 nuScenes？ | 10 Hz 原生（nuScenes 标注 2 Hz，正好吃掉"提前量"这个量级）；带 `valid` 与车道几何；车载采集。nuScenes 留在降级档当演示 |
| 官方预测起点只有 1 s 历史，你们怎么要 20 帧？ | 不用官方起点。我们在 91 帧里重新切 anchor（≥ 第 20 帧），代价是每段能用的未来变短（6 s） |
| 有真实数据为什么还仿真？ | 录像不能改。要回答"换成我们的系统会怎样"、"这次刹车是不是误报"，必须有反事实 |
| 你的切入怎么定义的？ | ①–⑤ 五条判据，写在 `docs/L3_waymo_case_study.md`，可复现 |
| 邻车不响应你们的刹车，结论还成立吗？ | 成立，但表述必须准确：这是**真实轨迹回放下的 AEB 表现**，不是闭环交互。§8.2 明写 |
| N=5 能说明什么？ | 不能做统计。它证明"迁移检查"这一步做了，且 direction 一致；边界仍由合成场景负责 |
| 会不会过拟合？ | 不训任何模型；GRU 是 P 在合成数据上训的，真实片段只做**留出测试** |
| 做不完怎么办？ | §9 四档降级，最差保持 `NotImplementedError` + limitations 明写 |

---

## 12. 和队友的接口

| 对谁 | 你要保证 | 你要通知 |
|---|---|---|
| **P（预测/GRU）** | 每个片段 `t ≤ 0` 有 **≥ 20 个连续 valid 采样点**；`dt=0.1`；无 NaN；`accel=None` | 有多少片段因历史不足被排除；GRU 回退次数要能统计 |
| **C（仿真）** | `duration` 写进 `params`；`ego_path` 够长；`dt=0.1` | 新增 `kind ∈ {"real_cut_in", "real_control"}`，**没有**进 `KINDS` |
| **D（评估）** | 每个片段能给出 `benign`；两组分开 | 真实片段**不进** `run_matrix` 的 96 格，单独出图 |

---

## 13. 完成打勾

```
[x] Step 0  官方 proto descriptor 已编译并随包分发；8 个 validation shard 到位
[x] Step 1  字段打印 + SDC 官方索引确认（不固定为 tracks[0]）
[x] Step 2  BEV 图（含 lane polyline）
[x] Step 3  首批切入 5 + 对照 5；完整候选台账（40 切入 / 435 对照）
[x] Step 4  anchor ∈ [20,30]，窗口全 valid，duration=6.0
[x] Step 5  ego 帧变换 + ego_path 选定（直线度 < 1 m）
[x] Step 6  Trajectory：t ∈ [−2, +6]，81 点；只对齐时间戳，不插值轨迹
[x] Step 7  Scenario 通过 _validate（run_sim 不抛异常）
[x] Step 8  12 个 npz 各 < 200 KB，落在 tests/fixtures/，含两份说明案例
[x] Step 9  Reactive/CV 跑完；完整审计 40 切入中 1 dangerous / 39 benign
[ ] Step 9b CA/Oracle/GRU 待 P 实现并接入；不属于 A 的数据交付未完成项
[x] Step 10 三张主图 + 两组表 + 说明案例表 / ADE / 逐帧日志
[x] Step 11 NPZ 单测全绿；自动测试禁止 protobuf 与原始数据访问时仍能读快照
[x] Step 12 case study 文档 + README/interfaces/contributions/references 同步
[x] 收尾   A_baseline_handoff.md §8 的 Waymo 项改成 ✅；nuScenes 旧占位接口保留兼容
```

---

# 附录 A：数据获取（Step 0 的完整版）

> 这一节是 2026-11-18 在**你这台机器上实测**出来的结论，不是抄文档。

## A.0 先判断你卡在哪

| 症状 | 去哪 |
|---|---|
| `waymo.com` 打不开 / 一直转圈 | 网络问题 → A.4 直接换 Argoverse |
| 页面能开，但没有登录入口 / 下载按钮点了没反应 | 没同意条款 → A.2 第 2 步 |
| 登录了，找不到 Motion / 不知道勾哪个文件 | A.2 第 4 步 |
| 下到一半断、速度几十 KB/s | A.3 用 curl 续传 |
| 全都不行 | A.4（200 KB 一个场景，立刻能开始） |

## A.1 为什么 Waymo 不能"找个直链下"

实测结果：

| 探测 | 结果 | 含义 |
|---|---|---|
| `https://waymo.com/open/download/` | **200**（923 KB HTML） | 页面本身能打开 |
| 该 HTML 里搜 `.tfrecord-` / `motion_v1.2` | **0 处命中** | 文件清单是**登录后由后端下发**的，静态页里没有 |
| `storage.googleapis.com/waymo_open_dataset_v_1_2_0/motion_v1.2/...` | **403** | 匿名读对象被拒 |
| 同桶根目录列表 | **403** | 匿名连列目录都不行 |

**结论：必须先登录并同意条款，没有能绕过去的直链。** 所以别去搜"Waymo 网盘直链/镜像"——那只会浪费时间。

## A.2 官方路径：5 步，别跳

1. **Google 账号**（任意）。
2. **同意条款**：<https://waymo.com/open/terms/> —— 这一步和下载页是**两个页面**。没同意时，下载页的按钮是灰的/点了没反应，这是最常见的卡点。
3. **登录**：<https://waymo.com/open/download/> 右上角 Sign in。
4. 选 **Motion** 区块 → 版本 **v1.2** → **validation** → **只勾 1 个分片**（命名形如 `validation.tfrecord-00000-of-00xxx`，以页面为准）。
5. 点该文件的下载，**复制链接**（签名 URL）。

| 别做 | 为什么 |
|---|---|
| 点 "Download all" | 那是全量（几十 GB 起） |
| 拿浏览器硬下 | 断了不能续，几 GB 白费 |
| 把签名链接存起来慢慢用 | 签名**会过期**（几十分钟到几小时），过期就回页面重点 |

## A.3 用 curl 续传（Windows 10/11 自带 `curl.exe`）

```powershell
curl.exe -L -C - --retry 8 --retry-delay 5 --retry-all-errors `
  -o "data\womd\validation.tfrecord-00000-of-00xxx" "<粘贴签名链接>"
```

- `-C -`：断点续传。断了**重跑同一条命令**就接着下。
- `--retry-all-errors`：签名过期以外的瞬时错误自动重试。
- 下完检查：`Get-Item data\womd\* | Select-Object Name,Length`，应当是 GB 量级（不是几 KB——几 KB 多半是 HTML 错误页，打开看第一行就知道）。

## A.4 兜底：Argoverse 2（公开 S3，无需账号）——已实测跑通

组员把它列为第一备选，而它恰好是**唯一一个不需要任何登录就能拿到的真车数据**。我在这台机器上实测：目录能列、文件能下、单场景只有 200 KB。

**为什么它够用**：10 Hz；每段 11 s（110 帧），**前 50 帧是历史**，比 GRU 要的 20 帧宽裕一倍多；带车道中心线。

**存储布局**（实测）：

```
s3://argoverse/datasets/av2/motion-forecasting/
├── train/  val/  test/                  # 每个场景一个目录
├── <split>/<scenario_id>/
│   ├── scenario_<scenario_id>.parquet          (~80 KB)
│   └── log_map_archive_<scenario_id>.json      (~120 KB)
└── av2_mf_*_annotations.parquet         # 测试集标注，你不需要
```

**取一个场景**（把 `<id>` 换成 A.4 下面列出来的任意一个）：

```powershell
$id = '02b57064-6fe3-41a4-8ad5-24e51abbdac4'     # 已实测存在的 val 场景
$b  = "https://argoverse.s3.amazonaws.com/datasets/av2/motion-forecasting/val/$id"
curl.exe -L -o "data\av2\$id.parquet"  "$b/scenario_$id.parquet"
curl.exe -L -o "data\av2\$id.map.json" "$b/log_map_archive_$id.json"
```

**再列 1000 个候选 id**：

```powershell
curl.exe -s "https://argoverse.s3.amazonaws.com/?list-type=2&prefix=datasets/av2/motion-forecasting/val/&delimiter=/" -o av2_list.xml
Select-String -Path av2_list.xml -Pattern '<Prefix>.*</Prefix>' | ForEach-Object { ($_ -split 'val/')[1] -replace '/.*','' }
```

**读它**（`pip install pyarrow`；`pandas` 你已经有了）：

```python
import json

import pandas as pd

df = pd.read_parquet("data/av2/<id>.parquet")
print(df.columns.tolist())        # track_id/object_type/x/y/vx/vy/heading/observed/timestep/...
ego_id = df["focal_track_id"].iloc[0]
ego = df[df.track_id == ego_id].sort_values("timestep")
print(ego[["timestep", "x", "y", "vx", "vy", "heading"]].head())

with open("data/av2/<id>.map.json", encoding="utf-8") as fh:
    m = json.load(fh)             # m["lane_segments"] -> centerline / left/right boundary
```

| 注意 | 说明 |
|---|---|
| `observed=False` | 和 Waymo 的 `valid=False` 是**同一件事**：未来/未观测帧，不能当 0 用 |
| 有 `s5cmd` 的话 | `s5cmd --no-sign-request cp "s3://argoverse/datasets/av2/motion-forecasting/val/<id>/*" .` |
| **千万别** | `cp .../val/*`——那是 2.5 万个场景、几十 GB |
| 目录名 | bucket 叫 `argoverse`（不是 `argoai-argoverse`，那个会 `NoSuchBucket`） |

## A.5 关键：换数据源**不阻塞**你

```
reader（womd.py / av2.py）  ──►  Scenario  ──►  后面全一样
        ↑ 只有这一层因数据源而变
```

所以正确的干法是：**先用合成场景把 Step 5–11 全部跑通**（`Scenario` 之上的代码一行都不用改），再补 reader。数据晚到一天，你的进度不受影响。

> 甚至可以手搓一个"假 clip"（把 `make_scenario("cut_in")` 的输出存成 npz，再从文件读回来）来验 Step 8/9/11 的链路。这一步建议你现在就做，10 分钟。

## A.6 团队分工

一人一个分片 / 一批场景 id，别所有人撞同一堵墙。谁先在 Windows 上跑通任一路径，就把命令贴群里。

## A.7 验收（哪条路都一样）

```
[ ] 文件存在且大小合理（Waymo 分片 GB 级 / AV2 场景 200 KB 级）
[ ] 能读出第一条记录（read_scenarios(path, limit=1) 不抛异常）
[ ] data/ 在 git status 里看不见
[ ] 能在 90 秒内画出这个场景的 BEV
```

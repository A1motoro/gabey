# Cut-in 2.5D Demo：运行、测试与扩展

## 启动

首次下载仓库后，使用 Python 3.10 或以上版本，在 `gabey` 根目录安装依赖并启动：

```bash
python -m pip install -r requirements.txt
python examples/cut_in_demo.py
```

浏览器打开 `http://127.0.0.1:8765`；推荐的展示版地址为
`http://127.0.0.1:8765/presentation`。停止服务：在终端按 Ctrl+C。
如果使用课程 conda 环境，先激活环境，再执行上述命令；部分系统需将 `python` 改为 `python3`。
组员各自在电脑上启动服务即可使用，服务默认仅监听本机，不提供局域网或公网访问。

## AI 使用记录

2026-11-11，C 使用 OpenAI Codex 实现并整理此 demo、扩展接口、测试和说明。
发布前自动检查：Python 回归 216 passed、2 skipped；另有两项因缺少课程 CSV 未执行；
JavaScript 播放事件测试 9 passed。已进行浏览器交互与布局检查，团队人工代码复核仍待完成。
这些检查验证实现行为，不构成真实车辆安全结论。此前的 AI 使用记录保持在
`docs/ai_disclosure.md`，此处单独记录本次 demo 的产出，避免混入尚未提交的报告相关记录。

支持从任意目录传入脚本绝对路径。离线运行，无须 npm、GPU、在线地图或真实数据文件。
端口占用时用 `--port 8766`；不自动打开浏览器用 `--no-browser`。

## 浏览器手动检查（约两分钟）

1. **标准场景**：初始 gap=20 m、rel_v=5 m/s、cut-in start=0.5 s、duration=3 s。
   点击 Play，旁车应平滑切入；本车只沿原车道减速，不转向。
2. **关键时刻**：拖动时间轴至 1.70 s，左侧应为 CRUISE、右侧为 BRAKE，CV TTC=1.9 s。
   在 2.00 s 左侧开始 BRAKE。首次触发时速度尚未变化是正常的：动作影响下一个状态。
   可聚焦时间轴用左右方向键微调，或点击 Step +0.1 s。
3. **结束结果**：拖至 10 s，两侧 FINISHED。Reactive 首刹 2.0 s，CV 首刹 1.7 s，
   提前 0.30 s；两侧都不碰撞。全程最小中心距分别约 3.24 m 和 3.00 m。
   这组数据说明更早触发，不说明 CV 在所有场景都更安全。
4. **安全切入**：选择 Faster cutter · benign，再点击 Run simulation。
   gap 仍为 20 m，相对速度为 −2 m/s；两侧全程保持 15 m/s、没有刹车，TTC 显示 —。
   结束时首刹为 Not triggered，提前量不可计算。
5. **碰撞展示**：设 gap=10 m、rel_v=4 m/s，其余保持默认，重新运行。
   结果应如实报告 Reactive 碰撞。发生碰撞的一侧应标为 COLLISION · FROZEN，并停在首次接触时刻；另一侧继续播放。
6. **交互一致性**：测试暂停、重播、逐帧、0.5×/2×、预测线开关。
   两边始终使用同一时钟；改变播放速度不改变结果。修改参数后，旧结果保持不变，
   直到点击 Run simulation；修改期间有 Unapplied changes 提示。
7. **布局**：检查 1440×900、1920×1080；窄于 760 px 时道路上下排列。
   没有横向滚动；关键时刻预测线、动作标记、尾灯和 TTC 应对应同一仿真步。

TTC 的 — 表示预测时域内没有检测到冲突，不是“未来永远安全”。Reactive 使用解析纵向 TTC，
其有限 TTC 可以大于 3 s；CV 在 3 s 离散预测范围内寻找冲突。中心距离不是车身间距。
播放时车辆位置和速度连续插值；TTC、中心距离、动作、预测线和曲线使用前一个 0.1 s 仿真采样点。
终点显示的 FINISHED 不表示又执行了一次策略；最后状态只用于展示。

## 自动化测试

在 `gabey` 根目录、已安装 pytest 和项目依赖的 Python 环境里运行：

```bash
PYTHONPATH="$PWD/src" python -m pytest tests/test_demo.py -q
```

预期 **16 passed**。包含：与直接仿真一致、预测数据来源、因果历史、无穷 TTC 的 JSON 表示、
安全与碰撞案例、观测隔离、适配器元数据、HTTP 错误和端口占用。
受限沙箱不允许绑定本地端口时，HTTP 测试会明确 skip；正常本机终端不应跳过该项。

完整回归：

```bash
PYTHONPATH="$PWD/src" python -m pytest -q
```

当前仓库未包含 `data/sample_trajectories/` 的课程 CSV，以下两个既有测试需要先补齐课程数据：
`test_load_toy_excludes_ego_and_uses_the_file_stem_as_id`、`test_load_toy_ego_returns_the_ego_clip`。
未补齐时可以运行其余回归：

```bash
PYTHONPATH="$PWD/src" python -m pytest -q -k 'not test_load_toy_excludes_ego_and_uses_the_file_stem_as_id and not test_load_toy_ego_returns_the_ego_clip'
```

若 matplotlib 提示缓存目录不可写，可在命令前添加 `MPLCONFIGDIR=/tmp/pbs-mpl`。
`PYTHONPATH` 使用绝对路径，确保既有测试从临时目录启动子进程时也能导入 pbs。

## 实现位置和 API

- `examples/cut_in_demo.py`：与工作目录无关的启动入口。
- `src/pbs/demo/scene.py`：`SceneAdapter.load(config) -> DemoScene`、合成场景及参数约束。
- `src/pbs/demo/service.py`：独立运行两侧闭环仿真，包装 assessor/predictor 记录真实决策输入与输出。
- `src/pbs/demo/server.py`：标准库 HTTP 服务。`static/` 是离线 HTML/CSS/Canvas 界面。
- `src/pbs/sim/observation.py`：观测模型协议。`run_sim(..., observation_model=None)` 保持旧行为。

`GET /api/config` 返回默认值、预设、范围和固定参数。
`POST /api/simulate` 接受参数对象，例如 `{"gap_d":20,"rel_v":5,"cut_in_t":0.5,"lc_T":3}`。
响应包含版本、时间轴、场景来源、道路、agent 参考轨迹、可选本车参考轨迹、两侧仿真和摘要。
每侧 `trace` 按帧保留 agent ID、当前可见历史、实际预测、considered 和 TTC。
所有非有限 TTC 为 JSON null；`summary` 是全程结果，只在播放完成后显示。

## 后期接入真实 data

新增适配器，实现 `load(config)` 并返回 `DemoScene`，通过
`simulate(config, adapter=YourAdapter())` 接入。首版网页只暴露合成场景；确定数据集后再添加对应路由和选择器。

适配器需要执行并记录：时间归零、坐标系转换、m/s/rad 单位转换、车辆 ID 检查、
缺失值处理与 0.1 s 对齐。agent 轨迹需覆盖 0–10 s，包含速度和航向；本车固定路径需足够长。
风险评估仍按观测时刻截取历史，不能把记录中的未来交给预测器。

`DemoScene` 包括：

- `scenario`：现有 `Scenario`；本车由策略驱动，旁车按记录回放。
- `source`：数据来源及 transforms。测量轨迹标为 reference/measured，不自动宣称是真值。
- `road_lines`：可选世界坐标道路折线；缺失时使用示意道路。
- `ego_reference`：可选原始本车参考记录，供未来展示叠加；不用于替代闭环本车。

保留现有固定路径、纵向制动、点质量/简化距离碰撞约定。接口不代表已经支持任意地图、
道路动力学或他车对本车行为的交互反应。

## 后期加入 noise 和 delay

实现 `observe(history, *, t_now) -> Trajectory`，将实例传给 `run_sim`。
传入的是参考轨迹的独立历史切片；可修改位置/速度测量，但必须保留 ID 和时间戳。
每个 agent 的同一时间戳应复用同一个噪声样本；不要在每次读取历史时重新随机生成。
服务层接受 `observation_factory`，为两个系统分别创建模型；相同种子可提供匹配的干扰。
参考轨迹仍用于碰撞检查；真实数据中这只能理解为“相对参考记录的评估”。

现有 `latency` 是旧观测延迟，CV assessor 会时间对齐。**计算延迟尚未实现**：
后续应在仿真执行层加入待执行决策队列，并让车辆在等待期间继续运动，明确过期决策处理规则。
不要通过 HTTP sleep、动画降速或现有 latency 参数声称实现了计算延迟。

## 建议演示顺序

先播放标准场景；暂停到 1.7 s，解释“CV 看见横向趋势，Reactive 此时仍忽略车道外目标”；
逐帧到 2.0 s；播放到结束比较首刹时间。再运行 Faster cutter，说明系统不会仅因为旁车切入
就必然刹车。最后说明结果来自合成二维闭环模型，2.5D 只负责展示。


## 刹车次数与相对速度说明

每次从非 BRAKE 状态进入 BRAKE 算一次刹车事件；连续多帧 BRAKE 只算一次。
页面实时显示当前已发生的事件数及累计 BRAKE 指令持续时间，结束摘要显示总数。
展开 Brake / release event log 可以查看每段开始、松刹时刻和持续时间。
若仿真结束时仍在 BRAKE，标为 Held at run end，不能把结束时刻当作实际松刹。
终点复制的 action 不参与统计。这些是策略指令统计，不是物理踏板次数；车辆停住时
仍保持 BRAKE 的时段也计入指令时间。

默认标准场景 Reactive 为 3 次、CV 为 5 次；两侧累计指令时间均约 0.80 s。
安全切入预设两侧都是 0 次、0 s。重播不会累加新次数，拖回时间轴会恢复当时的计数。
计数和持续时间测试可运行：

```bash
PYTHONPATH="$PWD/src" python -m pytest tests/test_brake_activity.py tests/test_demo.py -q
```

预期 23 passed（允许本地 socket 时）。持续时间与次数必须一起看：延迟松刹可能使同一次
刹车更长，而不增加次数；当前统计本身不代表已经实现计算或松刹延迟。

relative speed = ego 纵向速度 − agent 纵向速度；值越大表示本车追近越快，旁车越慢。
页面根据当前待应用输入显示 agent 纵向速度和“不刹车、保持初始纵向速度”的预计追上时间
`gap_d / rel_v`（rel_v > 0）。这是初始参数的解释，不是实际闭环轨迹预测，也不是碰撞时间。
若本车在旁车完成换道前就超过旁车，场景可能变成切到本车后方，并非更大的相对速度
必然更危险。换道开始时刻、持续时间和纵向间距需要一起考虑。


## 当前 Demo 的迟滞策略

两侧均使用现有 `HysteresisPolicy(2.0, 3.0)`，每次独立仿真创建独立策略实例。
巡航时 TTC < 2.0 s 才开始刹车；刹车时 TTC > 3.0 s 才解除；等于阈值保持原状态。
原 `TTCThresholdPolicy` 保留供其他基线实验使用，Demo 已不调用它。
迟滞不能保证只刹一次：CV 预测冲突消失时 TTC 为无穷大，也会解除刹车，之后可能再触发。


## 独立碰撞停帧、首次刹车与 assessor 替换

发生碰撞后，该侧车辆、旁车、镜头、面板时间和指标一同冻结在首次接触时刻；另一侧继续。
拖动到碰撞前可以回看，重播或重新仿真会恢复动画。全局播放时钟仍可走到 10 s。
安全一侧在另一侧冻结后采用独立跟随镜头。每侧顶部实时显示 First brake 和 panel t；
首次刹车尚未发生时不提前显示未来时刻。速度图的两种颜色虚线竖标记表示首次刹车。

首次接触来自 `SimLog.first_collision_t`，使用与原碰撞判定相同的分段线性扫掠，
包含采样帧之间的碰撞；时间是进入严格距离阈值的边界时刻，属于 ≤0.01 s 分段近似。
绘图位置在 0.1 s 状态间插值，碰撞帧位置有相应插值误差。
基础模拟器仍完成全程，保持原实验行为；新增元数据没有引入撞击或碰后车辆动力学。
网页的摘要、刹车次数、指令时间和事件记录截止到各侧首次接触或 10 s；
因此不直接比较一个碰撞侧的短时制动总量与一个安全侧的全程制动总量来判断舒适性。
碰撞侧显示的是接触前速度，不是把速度伪造为零。

风险模块的独立入口是 `src/pbs/demo/assessors.py`：

```python
from pbs.demo.assessors import AssessorBundle, make_demo_assessor
from pbs.demo.service import simulate

def factory(name):
    if name == "cv":
        return AssessorBundle(MyAssessor(), "Predictive · MyAssessor")
    return make_demo_assessor(name)

result = simulate(params, assessor_factory=factory)
```

`MyAssessor` 实现现有 `RiskAssessor.bind(scenario)` 与 `assess(ctx, history)`；每次构造新实例。
要把自定义模型用于网页，可修改 `make_demo_assessor` 对应分支，不需改动画。
可选 `predictor` 应提供实际推理后 `latest` 二维轨迹数组，供 wrapper 记录；
没有暴露预测轨迹时留空，界面不会绘制伪造轨迹。显示标题从 bundle.label 读取。

## 252 组条件对比

```bash
python experiments/exp06_demo_comparison.py
```

预先固定粗网格：gap=[5,10,15,20,30,40] m，relative speed=[−2,0,1,2,4,6,8] m/s，
切入开始=[0.5,1.5] s，换道时长=[3,4,5] s，共252组。其余均沿用当前demo固定设置。
输出 `outputs/logs/cut-in-demo/comparison.csv`、`comparison.md` 和 `summary.json`。
判定优先看碰撞；两边都未碰撞时，再对最小中心距、制动次数、制动时间和误刹做
Pareto比较（容差分别0.10 m、0次、0.05 s），互有优劣就标为权衡，不给武断的总分。
碰撞行只用来比较安全结果；制动时长的统计截止时间不同，不能当作舒适性对照。
测试仅覆盖给定合成场景，不代表真实交通总体。

新增预设 5 m gap · slow merge 为 gap=5、relative speed=1、start=0.5、lc_T=5；
可观察 Reactive 先冻结、CV 继续行驶。5 m gap · faster merge 将 lc_T 改为3，
两侧均未碰撞，而 Reactive 刹车次数更少。完整对比条件见生成表格。

新增接口和接触时间验证：

```bash
PYTHONPATH="$PWD/src" python -m pytest tests/test_demo_extensions.py tests/test_demo.py tests/test_brake_activity.py tests/test_simulator.py tests/test_collision.py tests/test_policy.py -q
```


## 原版与投屏增强版

- 原版：`http://127.0.0.1:8765/`。保留现有道路比例和播放操作。
- 增强版：`http://127.0.0.1:8765/presentation`。两版共用 `/api/simulate` 和同一个仿真程序，
  版本选择不会改变场景、assessor 或迟滞阈值。增强版不增加新的结果判优逻辑。

增强版使用更紧凑的道路取景和更大的指标文字，车辆图形依旧按相同世界尺度绘制，
不会为了放大车辆而更改碰撞半径。旁车标签在车上方，本车标签在车下方，降低遮挡。
车间浅色虚线连接当前显示位置，关联到上方的当前评估 TTC 标注；橙色虚线仍是未来预测轨迹。
TTC 数值来自记录的 assessor 输出，不是前端用距离/速度另算。Reactive 尚未考虑目标时显示
Outside ego lane · not assessed；没有预测冲突时显示 No assessed conflict；碰撞后显示接触时间。
运动插值与 0.1 s 决策采样的区别沿用原说明。

### 增强版的操作顺序

1. 选择预设或在默认展开的 Adjust parameters 修改参数，然后点击 Run simulation。两版都明确提醒此步骤，
   不会在改动滑块后静默替换正在观看的旧结果。
2. 选择播放区的 Pause at key events，点击 Play；标准场景自动在 1.70 s 的 CV 首刹暂停。
   此时右侧显示 Assessed TTC 1.90 s，左侧显示目标尚未纳入评估。
3. 再点 Play，在 2.00 s 的 Reactive 首刹暂停；继续播放不会再次卡在同一个事件。
4. Previous/Next event 或各事件按钮可以直接跳转并暂停。缺少相应事件时按钮置灰。
   同刻发生的事件合并到一次暂停，文本同时列出。拖回过去后再次播放可以重新经过事件。
5. 使用 5 m gap · slow merge 预设，可依次停在 CV 首刹 1.70 s、Reactive 首刹 3.00 s、
   Reactive 接触约3.12 s。接触暂停用于讲解，继续播放后只有 CV 动画继续，碰撞侧保持冻结。
6. 选择 Full process 后，可像原版一样连续播放。选择 Faster cutter · benign，所有刹车/碰撞按钮
   均显示 none；正常播放不受影响。

可用 N/P 跳到下一/上一事件；在非输入框、按钮或链接区域用空格播放/暂停；
原生按钮聚焦时空格保留浏览器原生按钮操作。参数输入时不截获键盘。
跳转不重新运行模型，不改变仿真结果。事件提示只描述动作，不宣称哪方更好。

### Noise 与 computation delay 模块

- `pbs.sim.observation.ObservationModel`：观测接口；显式无噪声实现 `IdentityObservation`。
  `simulate(..., observation_factory=...)` 为两侧创建独立实例，真实/参考轨迹不被污染。
- `pbs.sim.timing.ComputationDelayModel.delay_seconds(*, t_now)`：一次完整决策 tick 的总计算耗时。
  `NoComputationDelay` 返回0。`run_sim(..., computation_delay_model=...)` 可直接注入；
  demo 使用 `simulate(..., computation_delay_factory=...)` 分别构造两侧模型。

```python
from pbs.demo.service import simulate
from pbs.sim.observation import IdentityObservation
from pbs.sim.timing import NoComputationDelay

result = simulate({}, observation_factory=IdentityObservation,
                  computation_delay_factory=NoComputationDelay)
```

此调用与默认结果相同。当前非零计算延迟会明确抛出 NotImplementedError；
无效的负数、无穷或非数值会被拒绝。只有执行过的决策 tick 才调用耗时模型，
不会按每辆 agent 或终点展示行重复计费。
未来要实现非零值，需要决策执行队列、等待期间车辆状态推进和过期决策规则。
目前不提供尚未工作的 noise/delay 网页控件，也不通过 sleep 或播放降速伪造仿真延迟。
已有 `latency` 仍是观测延迟，`HysteresisPolicy` 仍是风险阈值迟滞，两者都不等于计算耗时。

### 新增验证

```bash
PYTHONPATH="$PWD/src" python -m pytest tests/test_timing_interface.py tests/test_demo.py -q
node --test tests/test_presentation_events.cjs
```

Node 仅用于开发期验证事件导航，不是启动网站的依赖。事件逻辑测试覆盖同时事件合并、
快速播放跨过多个事件时只停最近事件、继续不重复暂停、回看/重播及无事件场景。
浏览器还需检查两种播放模式切换、碰撞独立冻结、版本切换和两版相同参数的指标一致。


## 展示版参数区与播放模式（更新）

仅更新 `/presentation`，原版不变。Scenario controls 保留四个预设：

| 预设 | gap (m) | relative speed (m/s) | start (s) | 换道时长 (s) |
|---|---:|---:|---:|---:|
| Standard cut-in | 20 | 5 | 0.5 | 3 |
| Faster cutter · benign | 20 | −2 | 0.5 | 3 |
| 5 m gap · faster merge | 5 | 1 | 0.5 | 3 |
| 5 m gap · slow merge | 5 | 1 | 0.5 | 5 |

选择预设会立即填入全部四个滑块；滑块默认展开，仍可继续手动修改。
手动修改不会改写预设模板。点击 Run simulation 后才应用到仿真，并重置播放时间与事件。
参数区可以自行收起，但不再默认隐藏。

播放区使用两个互斥单选选项替代旧的 Auto-pause 勾选框：

- Pause at key events：在首刹和接触事件暂停，点击 Play 继续。
- Full process：连续播放到10 s，不在事件处停顿；碰撞侧仍按原规则独立冻结。

切换模式保留当前位置，不自动开始或重新运行。Replay 从头按当前模式播放。
事件跳转按钮在两种模式里都可手动使用，点击后仍会定位并暂停。

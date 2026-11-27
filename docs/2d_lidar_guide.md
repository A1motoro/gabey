# 2D 激光雷达：新手说明与复现

这次完成的是附件中的**加码一：白盒 2D 激光雷达感知栈**。以 main `8f399f9` 为基础，代码在 `codex/2d-lidar` 分支。队友的 `feat/ext-rear-end-safety` 是防后车追尾，本次没有修改或合入该分支的实现。

## 先运行看看

在仓库根目录、已有课程 Python 环境中运行：

```powershell
python experiments/exp08_lidar_perception.py --open
```

这会生成并打开一个可以播放、拖动时间、切换场景和预测器的本地网页。没有服务器，也不需要 GPU、原始 Waymo 数据或额外安装依赖。

如果系统没有自动打开，双击 `outputs/figures/lidar-perception/lidar_demo.html`。
灰色是真实车身，绿色是扫描点，橙色是检测框，紫色是跟踪轨迹。灰色只供人看对照，不输入预测器。

如果直接打开 `examples/lidar_viewer.html`，会显示演示入口；点击“播放”后打开已经生成的 `lidar_demo.html` 并自动播放。首次使用仍需运行上面的 Python 命令生成数据。

```powershell
# 较短的检查
python experiments/exp08_lidar_perception.py --quick --systems reactive cv

# 接上队友已经合入 main 的预测器，不改预测器本身
python experiments/exp08_lidar_perception.py --quick --systems ca ctrv nn nn_ua

# 看更少激光束 / 更大测距噪声会怎样
python experiments/exp08_lidar_perception.py --beams 360 --noise 0.05

# 验证核心感知与文件导出
python -m pytest tests/test_perception.py tests/test_lidar_export.py -q
```

生成的 `summary.csv` 记录首次刹车、碰撞和不刹车对照；`scans.json` 保存每帧点云、检测框、轨迹及位置协方差；`overview.png` 是可用于讲解的俯视图。

## 它大概怎么做到的？

把这辆自车想成**拿着一圈测距手电筒的人**。车不是直接“知道”周围车辆的位置和速度，而是经过下面四步：

1. **打一圈光，留下点。** 默认每帧发出 720 个方向的光束，最多看 80 米。光束碰到车身边缘，就记下一个坐标。前面的车会挡住后面的车，所以每束光只记录最近的接触点；没碰到任何车就没有点。
2. **把靠得近的点分成一组。** 例如一辆车反射出一小串点，就把这串点当成一个候选目标。这里叫“聚类”。它并不知道这辆车在真实场景里叫什么。
3. **用矩形框住这一组点。** 尝试不同角度，让点尽量贴着矩形边缘，这叫 L-Shape 框拟合。只扫到半边车，估计的中心就可能偏；完全看不到时，不会凭空补出一个车框。
4. **记住它，估计速度。** 把当前目标和上一帧附近的目标对应起来，用卡尔曼滤波平滑位置，并根据位置随时间的变化估计速度。连续扫描形成 `Trajectory`，再交给项目已有的预测和刹车模块。

整个流程就是：

```text
扫描到的点 → 分成几组 → 估计车辆框 → 连续跟踪成轨迹 → 预测未来 → 判断是否刹车
```

**2D 的意思是只在俯视平面里做这些事情**，就像在纸上看车的位置，不模拟车高、坡度或真实三维激光反射。

## 为什么会出错？这些错误有什么研究价值？

- **遮挡：** 旁边的车挡住另一辆车，目标要等露出来才被发现，留给刹车的时间可能变短。
- **中心漂移：** 第一帧只能看见一小角，下一帧多看见了一条边，框的中心会移动。预测器可能把这种“观测框变化”当成“车辆正在横向运动”。
- **距离远、点少：** 远处的车占的视角小，同样数量的光束只能扫到几个点，框和速度更容易抖。
- **滤波与确认延迟：** 平滑能减少抖动，但也可能让运动变化晚一点体现出来。第一次检出没有速度信息，默认至少两次实际检出才输出轨迹；已有目标短暂丢失时最多外推 3 帧。

这让原来“直接读正确轨迹”的实验更接近感知存在误差的情况。它用于检验预测器的鲁棒性，而不是保证更早刹车或更高安全性。

## 实际实现对应哪些文件？

| 文件 | 作用 |
| --- | --- |
| `src/pbs/perception/sensor.py` | 从自车当前位姿发射射线，与车辆旋转矩形 / 行人圆形求最近交点，添加可复现的测距噪声 |
| `src/pbs/perception/detection.py` | 欧氏距离连通分组；搜索 0–90° 的框方向，默认使用 L-Shape 最小方差准则，也支持最小面积准则 |
| `src/pbs/perception/tracking.py` | 一对一最近邻关联；状态 `[x,y,vx,vy]` 的卡尔曼滤波；轨迹确认、短暂丢失与删除 |
| `src/pbs/perception/pipeline.py` | 串起四层，输出因果轨迹，可选保存诊断帧 |
| `src/pbs/sim/simulator.py` | 新增可选 `perception_model` 接口；默认仍使用原来的轨迹观测 |
| `src/pbs/data/perception_scenarios.py` | 独立的双车遮挡场景，不改变原有场景种类和网格 |
| `experiments/exp08_lidar_perception.py` | 同场景真值 / 雷达观测对比，导出 CSV、JSON、图和网页 |

感知层与真值层的边界很明确：**仿真器必须知道真实车身，才能生成光束反射；聚类、框拟合和跟踪只接收点，不读取真实车辆 ID、速度或未来轨迹。** 跟踪器自己生成 `lidar_1` 等 ID。碰撞评分仍使用真实车身，避免“没看到所以不算撞了”的错误。

代码接入示例：

```python
from pbs.perception import PerceptionPipeline
from pbs.risk.assessors import PredictiveAssessor
from pbs.prediction import ConstantVelocityPredictor
from pbs.sim.policy import HysteresisPolicy
from pbs.sim.simulator import run_sim

perception = PerceptionPipeline(record_frames=True)
log = run_sim(
    scenario,
    PredictiveAssessor(ConstantVelocityPredictor()),
    HysteresisPolicy(),
    duration=6.0,
    perception_model=perception,
)
```

如使用 `latency=0.2`，仿真会延迟交付原来视角下的扫描，而不是用自车现在的位置重画旧数据。启动时没有已交付的扫描就暂时不评估目标。`observation_model` 与 `perception_model` 不能同时启用。Oracle 依赖真实 ID 查询未来，所以不能直接用于无标签雷达轨迹；真值 Oracle 基准仍可以单独运行。

## 已验证的案例与限制

默认配置：720 束、0.02 m 测距标准差、seed=7、0.1 s 仿真步长、4 s 预测时域、原有滞回二值制动。合成案例运行 8 s，Waymo 案例运行 6 s。

| 案例 / 系统 | 直接使用真值的首次刹车 | 使用雷达的首次刹车 | 说明 |
| --- | --- | --- | --- |
| 无遮挡切入 / CV | 1.0 s | 1.1 s | 两者均未碰撞；雷达观测带来一帧晚刹 |
| 邻道直行、局部遮挡 / CV | 不刹车 | 1.9 s | 不刹车对照在 8 s 内不碰撞，雷达版本出现误刹 |
| 邻道直行、局部遮挡 / LR | 不刹车 | 2.1 s | 平滑预测仍可能被观测变化误导 |
| Waymo 重建 / CV | 2.2 s | 2.0 s | 两者均未碰撞；单案例不能据此判断总体收益 |

双车遮挡测试还验证：初始切入车完全没有激光反射，只有邻道遮挡车被扫描到；目标随后露出并建轨。遮挡车的内侧边缘为 y=1.40 m，始终在自车车身外，不是本车道挡路的前车。

以上数字由脚本生成，不是人为指定漂移或刹车时间。这里使用 L2 矩形 + 4 s 时域，因此 Waymo 数字与旧 exp07 的圆形 + 3 s 时域不能混用。`brake_in_benign_case` 只按本次 4 / 6 / 8 s 运行窗口判断，不替代 D 主线的 15 s 决策 + 19 s 结果窗口评分。

当前仍有清楚的简化：

- 这是仿真射线点云。Waymo 部分用已有轨迹重建车身，不是读取真实激光雷达原始点云。
- 不做车辆语义分类；感知轨迹按通用车辆送入现有风险接口。行人扫描使用圆形，但风险端仍按通用车辆处理。
- 检测框尺寸和协方差可导出；现有风险接口继续使用固定 5.0 × 2.5 m 车身，感知协方差没有自动混进 NN 的预测不确定度。
- 单边点云的隐藏中心和宽度无法唯一确定；附近车辆可能被分成同一簇，最近邻关联也可能换 ID。
- 假定自车定位准确；没有道路背景点、雨雾、扫描期间运动畸变、传感器计算耗时或真实车辆动力学。
- 两次检出确认会增加启动延迟；已有目标最多外推 3 帧，长时间全盲区不会继续无限“记住”车辆。

验收采用几何解析值、旋转 / 平移一致性、噪声复现、真实遮挡、轨迹因果性、闭环自车扫描位置、延迟交付、真值碰撞独立评分、Waymo 快照和自包含文件导出检查。全仓库测试结果与 AI 使用记录见 `docs/ai_disclosure.md`。

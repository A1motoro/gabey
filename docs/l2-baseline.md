# L2 闭环仿真基线

L2 的闭环实验统一使用 **TTC 滞回制动**和**随航向旋转的矩形车身**。L1 的历史验证和复现入口使用原来的单阈值策略与圆形判定。预测误差实验 `exp01` 使用固定轨迹切片，比较 ADE/FDE，无需运行制动策略。

| 设置 | L1 历史闭环验证 | L2 闭环基线 |
| --- | --- | --- |
| 制动策略 | `TTCThresholdPolicy(ttc_threshold=2.0, a_brake=6.0)` | `HysteresisPolicy(t_on=2.0, t_off=3.0, a_brake=6.0)` |
| 车辆碰撞几何 | 显式传入 `r_safe=2.5`，两中心距离小于 2.5 m 计为重叠 | `r_safe=None`，车辆用长 5.0 m、宽 2.5 m 的定向矩形 |
| 复现入口 | `python experiments/exp00_l1_baseline.py` | `python experiments/c_l2_closed_loop.py` |
| 结果 | [P 的 L1 验证](l1-validation.md) | [C 的 L2 闭环结果](C_L2_closed_loop.md) |

## 制动与碰撞

- 巡航时 TTC 严格小于 2.0 s 才进入刹车。刹车时 TTC 严格大于 3.0 s 才解除。等于任一阈值时保持当前状态。TTC 为正无穷时回到巡航。每次独立仿真从巡航开始，制动减速度为 6.0 m/s²。
- 矩形以轨迹位置为中心，随车辆航向旋转。仅相切时面积为零，进入重叠才记作碰撞。首次接触时间记录为进入重叠的边界时刻。真值碰撞采用默认 0.01 s 最大碰撞子步的分段扫掠。行人保留原等圆模型的 1.25 m 半径。
- Predictive 在离散预测时刻用车身轮廓评估冲突。Reactive 保留本车道筛选，只用车身沿车道的纵向投影计算净空和纵向 TTC。两种方法的碰撞标签都由实际运动和真值几何判定。
- 对照实验只把制动策略换为 `NeverBrakePolicy`。在相同场景、时长和几何下，若无制动对照发生碰撞，场景记为 dangerous，否则记为 benign。良性场景里只要实际执行过刹车，就记为误刹。

## C 的复现配置与 D 的接入

C 的验收用 A 的 `ca_scenario_set()` 共 12 个场景，对比 Reactive、CV、CA、CTRV、Oracle 和无制动对照。仿真 15 s，步长 0.1 s，预测时域 4 s（40 步），延迟为零。场景生成到 19 s，覆盖 Oracle 在最后一帧的未来查询。时域和阈值敏感性实验要在输出中记录实际使用的参数。比如 D 扫描阈值时，逐行记录进入和退出阈值，并在同一配置下比较各系统。

以下调用以一个已生成的 `scene` 为例。旁车真值和自车路径都需覆盖本次运行所请求的时刻。

```python
from pbs.sim.policy import HysteresisPolicy
from pbs.sim.simulator import run_sim
from pbs.systems import make_system

log = run_sim(
    scene,
    make_system("ca"),
    HysteresisPolicy(t_on=2.0, t_off=3.0, a_brake=6.0),
    duration=15.0,
    horizon=40,
    r_safe=None,
    collision_dt=0.01,
)
```

## 日志与距离口径

| `SimLog` 字段 | 含义 | L2 用法 |
| --- | --- | --- |
| `collision` | 是否发生碰撞 | 碰撞率和反事实分类 |
| `first_collision_t` | 首次接触时刻。无碰撞时为 `None` | 事件时间、碰撞案例的统计截止点 |
| `min_dist_until_contact` | 到首次接触为止的最小中心距，单位 m。无碰撞时统计到仿真结束 | L2 报告的最近中心距 |
| `min_dist` | 整段仿真的最小中心距，包含继续积分得到的碰撞后数据 | 旧结果兼容和排查 |
| `executed_steps` | 实际执行的控制步数 | 末行状态可以显示 TTC，动作统计只读执行过的指令 |

中心距衡量两车中心之间的距离。仿真没有冲击动力学，首次接触后车辆可能继续穿行。比较 L2 的接触前间距时，D 的输出应直接读取 `min_dist_until_contact`，并附上 `first_collision_t` 和 `collision`。旧日志没有截止接触的距离时，记为缺失。现有 `evaluate()` 返回的 `min_dist` 仍按全程定义。

例如初始车距 20 m、相对速度 2 m/s、旁车以 −2 m/s² 减速切入时，无制动对照在 3.371 s 接触。截止接触的中心距为 5.010 m，继续积分后的全程最小中心距为 0 m。原始逐帧日志可用来追踪这段差异，报告应引用 5.010 m。

API 签名和向后兼容字段见[接口文档](interfaces.md)。

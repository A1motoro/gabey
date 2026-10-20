# C baseline：接入、验证与降级策略思考

更新：2026-10-20。

## 1. 本轮范围

用户确认 A/P 正式实现尚未提供，后续由 D 统一集成实验。本轮完成 C 的可调用仿真组件与接口验证，不是全组预测刹车实验。

已实现：统一 run_sim、滞回与原策略兼容、步内停车积分、输入与观测检查、步内最小距离、执行动作元数据。
2/3 秒阈值与 6 m/s² 减速度是可配置默认值，不是最优结论；r_safe 的几何依据仍待团队确认。

## 2. D 的正式接入方式

```python
from pbs.data.scenarios import make_scenario
from pbs.risk.assessors import PredictiveAssessor
from pbs.prediction.constant import ConstantVelocityPredictor
from pbs.sim.policy import HysteresisPolicy
from pbs.sim.simulator import run_sim, MissingObservationError

# 以下需 A/P 完成对应实现，当前仓库仍有 NotImplementedError。
scenario = make_scenario('cut_in', gap_d=30, rel_v=5, cut_in_t=2, lc_T=3)
assessor = PredictiveAssessor(ConstantVelocityPredictor())
policy = HysteresisPolicy(t_on=2, t_off=3, a_brake=6)
try:
    log = run_sim(scenario, assessor, policy,
                  duration=10, horizon=40, latency=0, collision_dt=0.01)
except MissingObservationError as exc:
    # 批量框架应保存参数与失败原因，不能计为安全样本或静默丢弃。
    print('Invalid observation:', exc)
```

先以 latency=0 集成，再验证延迟。horizon=40、dt=0.1 时预测 4 秒，长于解除阈值 3 秒。

A 提供：

- ego_start=(x,y,heading,v)，位置和方向与路径起点及首段一致，速度非负。
- 轨迹字段为等长有限一维 NumPy 数组，时间递增，agent_id 唯一且非空。
- 真值覆盖仿真时长；Oracle 需要额外未来覆盖时由 A/P 明确保证。
- 仿真起点可看到截至 -latency 的历史；预测器所需更多历史由 P 约定。
- 路径覆盖每步实际运动及自车预测，不依赖路径末端夹断。

P 提供：

- bind(scenario) 与 assess(ctx, history) -> RiskEstimate。
- TTC 非负，允许正无穷；NaN/负值报错。
- considered=False 表示主动排除，不代表缺失观测。
- 对齐到 ctx 的绝对未来时刻；C 不替 P 补算延迟。
- 普通评估器不得利用 bind 得到的未来真值；Oracle 是明确例外。

## 3. SimLog 时间语义

运行 2 秒、步长 0.1 秒：21 行状态，executed_steps=20。

- action[:20] 对应实际执行区间；末行 action 延续上一步，未执行。
- 末行 ego_a=0，TTC 仍作评估，但不调用策略或额外前进。
- ego_a 是步开始控制加速度，步内停车后余下时间静止。
- 新增字段放在 dataclass 末尾且有默认值，旧构造方式仍可用。
- evaluate 对新日志忽略未执行末行动作；executed_steps=None 的旧日志保持原逻辑。
- D 的其他统计同样应只读取实际执行的动作区间。

## 4. 独立演示与验证

在团队 Python 环境准备依赖后，从 gabey 根目录运行：

```bash
python -m pip install -e '.[dev]'
python -m pytest -q
PYTHONPATH=src python examples/c_smoke.py
```

演示使用 ScriptedRiskForDemo 与人工直线场景：前一秒给低 TTC，后一秒给高 TTC。
它是 C 的测试刺激，不是 P 的算法或安全性实验。输出 outputs/logs/c_smoke.csv；
CSV 增加 executed 列说明动作是否执行，原 SimLog 的字段仍保持兼容。
该输出目录按现有 .gitignore 不入库。

本轮实测：

- 38 passed、6 skipped；跳过项是原有 P 预测/风险 TODO。
- 新增/修改 Python 文件 Ruff 检查通过。
- 演示 21 行、20 个执行区间，最终 x=21 m、v=9 m/s，无模型碰撞。
- 覆盖解析停车距离、步内停车、多车 TTC、历史截止、初始碰撞、步内穿越、
  轨迹中间节点、路径转角、加速度扫掠收敛、滞回边界/重置、终点与旧日志兼容、非法输入。
- 测试使用临时独立环境，未向课程环境安装 pytest；D 仍需在其环境执行集成实验。

## 5. 当前 baseline：缺失观测时报错

用户选择第一版缺失观测时报错并记为无效。history_until 不以未来首帧补空历史；
run_sim 抛出 MissingObservationError。真值覆盖不足、路径不足和非法参数同样明确报错。

D 应报告有效场景数及失败原因。无效运行不能算作零碰撞成功案例，也不能静默删除。
比较系统时应保持相同样本集，必要时修复输入后重跑。

注意：当前验证针对空历史与真值覆盖，不是完整感知质量监控；观测最大年龄、
每种预测器的最小历史窗口仍需 P 规定，不能声称已处理所有观测质量问题。

## 6. 降级策略思考（未启用）

| 候选 | 可能优点 | 风险与限制 |
| --- | --- | --- |
| 短时保持上一步动作 | 不因一次缺失立即松开已有刹车 | 先前巡航时可能忽略新危险，需要严格限时 |
| 最后有效轨迹短期外推 | 桥接短暂空窗 | 不确定性随信息年龄增大，不能当真值 |
| 保守减速 | 减少未知条件下高速继续前进 | 可能误刹车，不能保证避撞 |
| 缺失超时终止并标无效 | 保持实验可解释性 | 不覆盖真实车辆持续运行的降级行为 |

后续若实现，应先共同确定：

1. 空历史、过期观测、主动排除如何区分；新增显式有效性/年龄信息，不伪装为 inf。
2. 最长桥接时间与恢复条件。
3. 单个目标失效如何影响整个系统，避免被其他有效目标掩盖。
4. 单独记录降级状态与事件，不和滞回状态混淆。
5. 构造可重复的观测丢失实验，比较碰撞、误刹车、制动时长、恢复时间和无效次数。

这些策略本轮均未启用；当前不擅自沿用旧 TTC 或执行紧急刹车。

## 7. 已知边界与交接

- 仅二维路径跟随与纵向控制，其他车辆不响应自车。
- 碰撞后继续记录但不模拟冲击，碰撞后运动不是物理碰撞结果。
- 碰撞子步默认最大 0.01 秒，同时在停车、路径折点和其他车辆轨迹节点处分段。
  对加速度运动仍有线性近似，接近阈值时缩小 collision_dt 验证稳定性。
- 尚未证明滞回在完整场景集中的收益；本轮证明的是实现符合规则及基础运动测试。
- 所有对照系统使用同一积分、几何、碰撞与日志规则。
- 本次涉及共享历史工具、SimLog 可选字段及 D 的 evaluate，PR 必须列出并请 D 审 C，A/P 确认输入条件。
- A/P 模块未代写，本轮未 commit/push。旧 proposal 指导与讨论总结为设计历史；当前实现以本文件与 interfaces 补充为准。

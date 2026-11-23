# A 的 L3：交接与答辩速查

A 完成的是 **M1 真实数据适配和验证证据**。L2 主实验、CA/Oracle/GRU 的实现仍由相应负责人完成。
结果与英文报告段落见 [L3_waymo_case_study.md](L3_waymo_case_study.md)。

## 你需要会讲的五件事

1. **为何不直接回放本车？** 我们要比较不同刹车决策，本车必须由 `run_sim` 自主推进；
   周车才按记录回放。观测本车轨迹另存在 `observed_ego_xy`，用于检查直线路径误差。
2. **为何不用 tracks[0]？** WOMD 明确给出 `sdc_track_index`，它才是本车标识。
   本机首条数据的本车在 index 10；危险案例的本车在 index 26。
3. **如何处理丢帧？** `valid=False` 的值不是 0。必须整段 ego/目标都有效才导出，
   不跨空洞插值。固定窗口含 81 帧、20 个历史步，时间从 -2 到 +6 s。
4. **为什么 5 个切入不刹车，另一案例却刹车？** “切入”是几何分类；“危险”是
   永不刹车是否碰撞的反事实分类。首批 5 个切入均 benign。说明案例是全候选审计后挑出的，
   已明确标注，不能混入随机或前瞻性的样本统计。
5. **能声称什么？** 单个真实危险回放里 CV 提前 0.3 s，两个系统都避免碰撞。
   另一个 benign 案例两者都误刹。不能声称 CV 总是更安全或代表真实交通分布。

## 文件分工

| 文件 | 你要理解的功能 |
| --- | --- |
| `src/pbs/data/womd.py` | TFRecord CRC、SDC、有效帧、固定坐标变换、几何筛选、NPZ |
| `scripts/export_womd_clips.py` | 按几何顺序冻结首批 5+5；记录全候选与 SHA256 |
| `scripts/audit_womd_cut_ins.py` | 对全部 40 个切入窗口核验，另存危险/误刹说明案例 |
| `experiments/exp07_real_clip.py` | 调现有系统、policy、sim、metrics；不实现新的预测器 |
| `tests/test_womd.py` | 真实快照、坐标不变性、无未来泄漏、无原始数据依赖 |

固定变换只做一次：`xy_local = R(-ego_heading_anchor) (xy_world - ego_xy_anchor)`。
速度使用 Waymo 的 `velocity_x/y`；`Trajectory.heading` 编码速度方向，因为 CV 用 `v * [cos,sin]`。
极低速时回退到框朝向。`Trajectory.accel=None`，额外的横向加速度估计只放进诊断元数据。

## 给 P 的接入契约

```python
from pbs.data.womd import load_clip
from pbs.sim.simulator import run_sim
from pbs.sim.policy import HysteresisPolicy
from pbs.systems import make_system

sc = load_clip("tests/fixtures/cases/womd_dangerous_6e86a639df5d4fa1_track1782.npz")
log = run_sim(sc, make_system("cv"), HysteresisPolicy(), duration=sc.params["duration"])
```

- `Scenario.kind` 为 `real_cut_in` 或 `real_control`；这些类型不进入 `KINDS` 或合成 96 场景的分母。
- `Scenario.agents` 在本次证据中只有选中的一个目标，ego 已排除。
- 每个目标的 ID 如 `track1782`，稳定且唯一。`t[-1]=6`，显式传 `duration=6`。
- 在仿真 t=0 时已有 21 个可观测帧；GRU 若用 20 帧，应取最后 20 帧（含当前时刻，跨度 1.9 s）。
  `forecast_windows()` 就用此约定，所有预测时域共用 7 个起点。
- 后续接入 GRU 时必须记录其回退次数；当前没有运行 GRU，因此没有学习模型结果。
- Oracle 若接入，需说明 t>3 时完整 3 s 预测会超出冻结的 t=6 真值；不得把端点保持当成真实未来。
  它应明确截断评测、扩展有效真值或采用团队约定，不能静默补未来。
- 新系统注册并实现后，用 `python experiments/exp07_real_clip.py --systems reactive cv ca oracle gru`
  重新运行；脚本只对明确 `NotImplementedError`/未注册项记 unavailable，其他错误直接失败。

## A 的验收入口

```powershell
python -m pytest tests/test_womd.py -q
python experiments/exp07_real_clip.py
```

原始多 GB 分片保持 ignore。提交时携带 `vendor/`、`womd.desc`、`tests/fixtures/`、
实验脚本、测试、文档和小型 `exp07` 图表；不要提交原始 Waymo 数据。

# 给 P：CA 需要的场景、真值和切片（A 交付，2026-11-18）

> 一句话：**CA 的代码还是你的（`ConstantAccelPredictor`），我不碰；我给你的是"能让 CA 和 CV 分出高下"的场景 + 真值 + 固定评测切片。** 没有这部分，`exp01` 在现有 96 个场景上会得到"CA ≈ CV"的假结论——因为 cut-in 纵向一直是匀速、crossing 也是匀速，CA 和 CV 在那些维度上数学恒等。

## 1. 你拿到什么

| 东西 | 位置 | 用途 |
| --- | --- | --- |
| `make_scenario("cut_in", a_cutter=±2)` | `src/pbs/data/scenarios.py` | 切入车在换道窗口内加速/减速，CA 终于有纵向加速度可估 |
| `CA_GRID` / `ca_scenario_set()` | 同上 | 12 个场景：`gap_d ∈ {20,30}` × `rel_v ∈ {0,2}` × `a_cutter ∈ {−2,0,+2}`，**7 benign / 5 dangerous** |
| `Trajectory.accel`（N×2） | `src/pbs/types.py` + 三个 builder | **解析真值**加速度。只用于验证你的估计器，`predict()` 里不许读 |
| `prediction_slices(...)` | `src/pbs/data/benchmark.py` | 固定 `t0`、固定 horizon 的 `(history, truth)` 切片；所有 predictor 同题 |
| 40 个 M1 测试 | `tests/test_data.py` | 已覆盖 GT 加速度、a_cutter 运动学、切片无泄漏 |

不变量（别担心踩坏）：

- `DEFAULT_GRID` **没动**，还是 96 个场景（52 dangerous / 44 benign），D 的实验矩阵和文档分母不受影响。
- `a_cutter` 默认 `0`，旧场景逐位不变；`lc_T` / 横向 quintic 的物理过滤照旧。
- `prediction_slices` 只依赖 `dt` 和 `scenario.params["duration"]`，不 import 任何 predictor。

## 2. 三条硬约定

1. **加速度从位置估，不要对 `v` 差分。** 课程数据的 `v` 列是分片常数（`docs/data_audit.md` §2.3），两点差分会造出 ±2 m/s² 的假加速度。用最近 5–10 帧位置的窗口最小二乘拟合 `p(t) = p0 + v0·t + ½a·t²`（同时得到 v 和 a）。
2. **预测到目标停车之后要 clamp。** `lead_brake` 里前车 3.75 s 就停住（decel=4），CA 外推会继续倒车。真值速度不会为负，预测也不应该。
3. **`exp01` 用合成切片当主数据，真实片段只做定性案例。** `load_toy()` 只有 12.9 s、没有一次换道，加速度不可辨识；拿它画 ADE 曲线，CA 会因为"没东西可学"而输给 CV。

## 3. 怎么用（CA 实现后把 `NotImplementedError` 换掉即可）

```python
import numpy as np
from pbs.data import ca_scenario_set, prediction_slices
from pbs.eval.metrics import ade
from pbs.prediction import ConstantAccelPredictor

predictor = ConstantAccelPredictor()
for sc in ca_scenario_set().scenarios:
    for sl in prediction_slices(sc, history=1.0, horizon=2.0, stride=0.5):
        pred = predictor.predict(sl.history, horizon=len(sl.truth), dt=sc.dt)
        print(f"a={sc.params['a_cutter']:+g}  t0={sl.t0:>4.1f}s  ADE={ade(pred, sl.truth):.3f} m")
```

现在 CA 还没实现，可以先把上面的 predictor 换成 `ConstantVelocityPredictor()`——能跑通就说明切片本身没问题，而且你会看到 **CV 在 `a_cutter ≠ 0` 的场景上 ADE 明显变大**，这就是留给 CA 的空间。

我用 CV 在全部 12 个场景 × 25 个切片上实测过（`history=1.0 s`、`horizon=2.0 s`）：

| `a_cutter` | CV 的 ADE 均值 | 切片数 |
| --- | --- | --- |
| `0` | 0.169 m | 100 |
| `+2` | 0.334 m | 100 |
| `−2` | 0.334 m | 100 |

加速度让 CV 的误差**翻倍**。如果 CA 估得准，它在坡段内和坡段后的切片上应该明显低于 0.334、向 0.17 靠；如果毫无改善，问题在估计器（窗口长度 / 噪声），不在场景。注意：**坡段刚开始的那几个切片**（历史一半匀速一半加速）CA 可能暂时比 CV 差，这是正常的，别把这一点优化掉。

切片长什么样：

```python
sl = prediction_slices(sc, history=1.0, horizon=2.0)[0]
sl.t0                 # 1.0 s：第一个窗口（需要 1 s 历史）
sl.history.t[-1]      # == 1.0：history 里没有未来，和 run_sim 给 assessor 的观测一致
len(sl.history.t)     # 11 个采样（10 Hz × 1 s）
sl.truth.shape        # (20, 2)：t0 + k·dt, k = 1..20
sl.truth_accel.shape  # (20, 2)：同网格的 GT 加速度，用来查你的 a 估得准不准
```

一个 15 s 场景默认出 25 个切片（`t0 = 1.0 … 13.0 s`，stride 0.5 s）。

## 4. 预期结论（写报告时可以直接用，但要用你自己的数据复核）

| 场景 | CA 应该比 CV | 原因 / 风险 |
| --- | --- | --- |
| `lead_brake` 刹车开始后 | **明显更好** | 恒定减速度正是 CA 的模型；CV 会一直按原速外推 |
| `lead_brake` 刹车刚开始的那 1–2 个切片 | **更差** | 历史窗口一半匀速一半减速，CA 估出来的是"平均减速度"，会低估 |
| `cut_in` 换道窗口内 | **横向更好**（quintic 有 ÿ） | 但纵向只有 `a_cutter ≠ 0` 时才不同 |
| `cut_in` 换道结束后 | 与 CV 相同 | `y=0`、`ÿ=0`、`a_cutter` 已经归零 |
| 目标停车后 | 取决于你是否 clamp | 不 clamp 会得到"前车倒车"的非物理预测，ADE 爆炸 |

这四条正好覆盖了报告里"CA 什么时候有用、什么时候反而更差"的那一段，也给了 D 的四边界图一个物理解释：**CA 只在纵向加速度可辨识的时候移动边界**。

## 5. 验证命令

```powershell
python -m pytest tests/test_data.py -q     # 40 passed
python experiments/exp00_scenarios.py      # 96 个场景，52/44
```

如果 `pytest` 挂了，先看 `tests/test_data.py::test_ground_truth_acceleration_of_every_family` 和 `test_prediction_slices_cover_the_clip_without_leakage`——它们分别守护"真值对不对"和"切片有没有偷看未来"。

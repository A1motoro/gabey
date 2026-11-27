# 加码方向二验证报告：防后车追尾纵向安全博弈（Rear-End Sandwich）

> **模块定位**：基于已冻结 `SimLog` 契约的零侵入后处理耦合仿真与双向安全评测（[`src/pbs/eval/rear_end.py`](../src/pbs/eval/rear_end.py)、[`experiments/exp09_rear_end_safety.py`](../experiments/exp09_rear_end_safety.py)、[`tests/test_rear_end.py`](../tests/test_rear_end.py)）。不修改 `pbs.types`、`pbs.sim` 等任何既有模块代码。

---

## 1. 研究问题与物理建模

单车前向 AEB 的核心盲点在于**风险转移（Risk Transfer）**：自车若因晚发现切入或采用二值满制动（`-6.0 m/s²`）猛烈急刹，即便勉强避免前向追尾，也极易导致后方存在人类感知反应延迟（$\tau_{\text{rear}}$）的跟驰车辆追尾自车。

### 1.1 后车跟驰与反应延迟模型（`FollowerConfig`）
- **初始稳态平衡**：$t = 0$ 时，后车位于自车正后方净保险杠间距 $d_{\text{rear}, 0} > 0\text{ m}$ 处（中心距 $d_{\text{rear}, 0} + L_{\text{veh}}$，$L_{\text{veh}} = 5.0\text{ m}$），初速度与自车相等（$v_{\text{rear}}(0) = v_{\text{ego}}(0)$）；当自车不制动时，后车严格保持匀速等距巡航（加速度恒为 `0`）。
- **因果延迟与物理限幅**：后车驾驶员在时刻 $t$ 仅能感知到 $t - \tau_{\text{rear}}$ 时刻的车距、相对接近速度与自车制动状态，最大制动减速度受路面附着极限约束（$a_{\text{rear, max}} = 6.0\text{ m/s}^2$），踏板与液压建压受加加速度上限约束（$J_{\max} = 10.0\text{ m/s}^3$）。同时支持运动学防御跟驰（`model="kinematic"`）与稳态校准延迟智能驾驶员模型（`model="idm"`）交叉验证。
- **双向安全判定**：在 `0.01 s` 连续子步长积分下，同时统计**前向碰撞**（`front_collision`）、**后向被追尾**（`rear_collision`，净保险杠间距 `< 0`）与**双向完全安全**（`safe_both = not front_collision and not rear_collision`）。当前向发生碰撞时，按仓库统一准则在 `first_collision_t` 处截断，避免碰撞后穿模轨迹污染后方统计。

---

## 2. 复现命令

```powershell
conda activate aie1903_ad
python -m pytest tests/test_rear_end.py -q -p no:cacheprovider
python experiments/exp09_rear_end_safety.py          # 完整 89 切入场景 + 21x17 后车网格 + Waymo 案例
python experiments/exp09_rear_end_safety.py --quick  # 快速冒烟模式
```

产出路径：
- 图表：`outputs/figures/rear-end/exp09_{bidirectional_outcomes,rear_safety_boundary,sandwich_timeline}.png`
- 数据与摘要：`outputs/logs/rear-end/exp09_{cut_in_outcomes.csv,rear_plane.csv,waymo_plane.csv,summary.json}`

---

## 3. 核心实测结果（Claim → Evidence → Limitation）

### 3.1 合成切入基准集（89 个切入场景：63 危险 / 25 良性 / 1 晚接触；名义后车 $d_{\text{rear}, 0} = 9.5\text{ m}, \tau_{\text{rear}} = 1.1\text{ s}$）

| 系统 × 制动策略 | 危险切入前撞率 ↓ | 前车救下但被后车追尾率 ↓ | **双向完全安全率 (`safe_both`) ↑** | IDM 口径双向安全率 ↑ | 平均峰值 Jerk ($\text{m/s}^3$) ↓ | $(d_{\text{rear}, 0}, \tau_{\text{rear}})$ 50% 安全面积 ↑ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| `Reactive + Binary` | 36.5% | 17.5% | 46.0% | 46.0% | 48.6 | 70.6% |
| `Reactive + Staged` | 42.9% | 9.5% | 47.6% | 47.6% | 8.1 | 72.8% |
| `CV + Binary` | 11.1% | **33.3%** | 55.6% | 60.3% | 60.0 | 77.3% |
| `CV + Staged` | 12.7% | 25.4% | **61.9%** | 58.7% | **10.0** | **82.1%** |
| `CTRV + Staged` | 11.1% | 30.2% | 58.7% | 52.4% | **10.0** | **82.4%** |
| **`NN-UA + Staged`** | **6.3%** | **14.3%** | **79.4%** | **81.0%** | **10.0** | **89.1%** |
| `Oracle + Staged`（上界） | 1.6% | 12.7% | 85.7% | 85.7% | 10.0 | 91.9% |

### 3.2 三条关键因果结论（Claims & Evidence）
1. **没有预测提前量，单靠分级温和制动无法提升整体安全（`Reactive + Staged` 消融）**：
   `Reactive` 直到切入车进入本车道才首次产生有限 TTC，此时冲突已迫在眉睫；引入 Jerk 限幅与温和档反而使前向碰撞率从 `36.5%` 上升至 `42.9%`，证明分级温和制动必须以预测提前量为前提。
2. **仅有预测但采用二值满刹会将前向碰撞转化为后车追尾（`CV + Binary` 风险转移效应）**：
   `CV + Binary` 将前向碰撞率从 `36.5%` 压低至 `11.1%`，但一触发即施加 `-6.0 m/s²`（峰值 Jerk `60.0 m/s³`），导致**被后车追尾比例翻倍至 `33.3%`**（每救下约 3 次前撞就有 1 次引发后车追尾）。
3. **“轨迹预测提前量 + 不确定度包络 + 三级温和制动”实现前后双赢（`CV / NN-UA + Staged`）**：
   预测器提前捕捉横向切入趋势，使 TTC 平滑落入 `[1.5 s, 3.0 s]` 预制动窗口，以 `-2.5 m/s²` 温和减速提前唤醒后车驾驶员反应时钟并保留净减速度余量。其中 `NN-UA + Staged` 将双向完全安全率从 `46.0%`（`Reactive`）和 `55.6%`（`CV + Binary`）大幅提升至 **`79.4%`**（IDM 口径 **`81.0%`**），逼近 `Oracle + Staged` 上界（`85.7%`）；在真实 Waymo 危险切入案例（`6e86a639df5d4fa1`）上，后方安全跟驰平面面积亦从 `87.7%`（`Reactive + Binary`）扩展至 **`98.0%`（`CV + Staged`）**与 **`99.7%`（`NN-UA + Staged`）**。

### 3.3 适用边界与局限性（Limitations）
- 后车为单车道一维纵向跟驰模型，不模拟后车横向紧急变道避让或多车连环碰撞动力学传递；
- 前车切入轨迹仍采用开环日志回放（不因自车温和点刹而二次改变切入意图）；
- 统计比率基于参数化切入网格与单条冻结 Waymo 危险案例，代表受控机理边界而非自然交通流真实事故率。

# P 的 L3 验证：方案 B 异方差神经网络、不确定度 TTC 与 LR 基线

> **最新更新（2026-11-24，分支 `feat/p-l3-neural-ua`）**：在合并 PR #15 / #16 后的 `main` (`3f141f7`) 上完成 P 的 L3 升级（`P-T1`～`P-T4`），对齐 [L3 全局开发总纲](L3_global_guide.md) 与 Option-B 规格。原有 L1 / L2 / L3(LR) 行为与既有测试 100% 保持兼容。

---

## 第一部分：方案 B 异方差神经网络与不确定度感知 TTC（2026-11-24）

### 1. 模块交付与接口边界（只加不改）

- **P-T1 跨场景统一特征（`src/pbs/prediction/features.py`）**：
  - `extract_traj_features(history, dt=0.1, steps=10) -> (60,)` 与 `extract_traj_matrix(...) -> (10, 6)`：以目标车末帧 $t_0$ 为原点、末帧航向 $\theta_0$ 为 $+x$ 轴提取过去 `1.0 s`（10 帧）局部 `(dx/20, dy/20, vx/20, vy/20, cos Δθ, sin Δθ)`；不足 `0.9 s` 时按首帧速度反向匀速外推补齐；
  - `extract_lane_features(history, ego_path, lane_width=3.5) -> (20,)`：相对 `ego_path` 的带符号横向偏移 $d_{\text{lat}}/\text{lane\_width}$、沿/跨车道速度 $(v_{\text{lon}}/20, v_{\text{lat}}/5)$、相对航向 $(\cos\Delta\theta_{\text{lane}}, \sin\Delta\theta_{\text{lane}})$、`lane_width/3.5` 与前方 `{5, 10, 15, 20, 30, 40, 50} m` 处 7 个参考点在目标车局部系下的 `(dx/40, dy/40)`；过滤 `<= 1e-6 m` 的近重复折线顶点，并在尖锐弯道内顶点采用邻段角平分法向保持欧氏横向距离连续。
- **P-T2 方案 B 自适应宽度神经网络（`src/pbs/prediction/neural.py` & `experiments/train_waymo_neural.py`）**：
  - 严格对齐 `PhysicsAnchoredResMLP`（Stem + 2 个 ResBlock + 均值/不确定度双头；$D=80$ 时精确参数量 $4H^2 + 251H + 160$），以局部恒速外推 $\hat{p}_k^{\text{cv}}$ 为物理残差锚点，输出局部均值 $\hat{p}_k = \hat{p}_k^{\text{cv}} + \Delta\mu_k$ 与标准差 $\sigma_k = \text{clip}(\text{softplus}(z_{\sigma,k}) + 0.05, 0.05, 5.0)$；
  - `forward_resmlp_numpy(feat, weights, cv_anchor)` 与 `NeuralPredictor` 采用纯 NumPy 矩阵前向推理（运行时零 PyTorch 依赖），自动适配任意宽度 $H_{\text{dim}} \in \{128, 256, 512, 1024\}$；预测后同步更新 `self.latest` 与 `self.latest_std`（`(H, 2)`），单帧静止输入按规格退化为 CV，支持任意 `horizon` 与 `dt` 插值/外推；
  - 不读取 `scenario.agents` 或 `Trajectory.accel`，`bind(scenario)` 仅深拷贝 `ego_path`/`lane_width` 并清空旧缓存；
  - 已在 `pyproject.toml` 注册 `"pbs.prediction" = ["weights/*.npz"]`，并随库打包 `< 2 MB` 的最优权重 `waymo_nn_lane.npz`（`h256`, `1.16 MB`）与 `waymo_nn_traj.npz`（`h256`, `1.14 MB`）。
- **P-T3 不确定度感知 Swept TTC 与车身尺寸透传（`src/pbs/risk/ttc.py` & `assessors.py`）**：
  - `assess_risk` 新增可选关键字参数 `vehicle_length`, `vehicle_width`, `agent_length_pad`, `agent_width_pad`；同时兼容当前 `main` 上的 L2 `collision.py` 与 C 的 L3 参数化 `collision.py`（合入前后数学结果误差 `< 1e-12`）；
  - `ReactiveAssessor` 与 `PredictiveAssessor` 读取 `ctx.vehicle_length` / `ctx.vehicle_width`（未提供时默认 `5.0 m × 2.5 m`）；
  - `PredictiveAssessor(predictor, *, uncertainty_scale=0.0, name=None)` 在 `uncertainty_scale > 0` 且 `predictor.latest_std` 非空时，将目标车长宽分别膨胀 `2 * min(κ·σ_lon, 1.5)` 与 `2 * min(κ·σ_lat, 1.5)`，并同步对观测延迟做时间插值对齐，保证 $\text{TTC}(\kappa > 0) \le \text{TTC}(\kappa = 0)$。

---

### 2. `{128, 256, 512, 1024}` 四档容量验证结果（不阻塞合成语料）

> **数据状态说明**：A 的大规模 Waymo 训练切片 `data/waymo_train.npz` / `waymo_val.npz`（A-T2）尚未交付；按总纲“不阻塞策略”，本轮先使用按场景 80/20 严格隔离的多样化合成语料（覆盖左右切入、加减速、前车制动、横穿、邻道巡航、中止换道、防自交弯道，且按两阶段训练法：Stage 1 25 epochs 时间加权 Smooth L1 + Stage 2 35 epochs Gaussian NLL 与单调 $\sigma$ 正则，含 50% 左右镜像 + 30% 物理一致观测微噪）跑通四档容量验证与权重导出。待 A 导出 Waymo `.npz` 后，使用同一脚本直接重训替换权重文件即可。

| 变体 | 隐藏维 $H_{\text{dim}}$ | 参数量 | `.npz` 大小 (MB) | Val ADE 2s (m) | Val ADE 4s (m) | Val NLL (nats) | 合成 `exp01` ADE 2s (m) | 全通道加噪 ADE 2s (m) | 综合分 | 选优结论 |
| --- | :---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `nn_lane` | `128` | 97,824 | 0.35 | 0.2921 | 1.0153 | 0.4147 | 0.2825 | 0.7546 | 0.5072 | — |
| **`nn_lane`** | **`256`** | **326,560** | **1.16** | **0.2606** | **0.9475** | **0.1726** | **0.2324** | **0.6848** | **0.4610** | **入库 (`< 2 MB` 胜出)** |
| `nn_lane` | `512` | 1,177,248 | 4.17 | 0.2503 | 0.9558 | 0.0541 | 0.1925 | 0.6767 | 0.4504 | 体积超 2 MB |
| `nn_lane` | `1024` | 4,451,488 | 15.72 | 0.2502 | 0.9475 | 0.0335 | 0.1942 | 0.6512 | 0.4482 | 饱和 + 体积过大 |
| `nn_traj` | `128` | 95,264 | 0.34 | 0.3440 | 1.3971 | 1.2165 | 0.2541 | 1.3619 | 0.6420 | — |
| **`nn_traj`** | **`256`** | **321,440** | **1.14** | **0.3250** | **1.3659** | **1.1035** | **0.2402** | **1.3780** | **0.6203** | **入库 (`< 2 MB` 胜出)** |
| `nn_traj` | `512` | 1,167,008 | 4.13 | 0.3149 | 1.3449 | 0.9742 | 0.2239 | 1.4034 | 0.6057 | 体积超 2 MB |
| `nn_traj` | `1024` | 4,431,008 | 15.65 | 0.3173 | 1.3608 | 0.9827 | 0.2185 | 1.5235 | 0.6106 | 过拟合退化 |

**关键消融与容量结论**：
1. **车道特征消融（`nn_lane` vs `nn_traj`）**：在相同 `h256` 下，`nn_lane` 将 4 s 验证 ADE 从 `1.3659 m` 降至 `0.9475 m`（**-30.6%**），Gaussian NLL 从 `1.1035` 降至 `0.1726`，在全通道观测噪声（`pos_std=0.15 m, heading_std=0.03 rad, speed_std=0.25 m/s`）下 2 s ADE 从 `1.3780 m` 降至 **`0.6848 m`（噪声鲁棒性提升 2.0×）**。
2. **容量饱和趋势**：`h256` 相对 `h128` 带来明显增益（`nn_lane` 综合分 `0.5072 → 0.4610`）；`h512` 与 `h1024` 体积扩大至 4.17–15.72 MB 但精度趋于饱和（`nn_traj h1024` 在验证集和加噪评测上均发生反弹）。在 `< 2 MB` 契约约束下，两变体均由 `h256` 胜出。
3. **合成验证集 $1\sigma$ 覆盖率说明**：无噪合成验证集包含大量横向真值位移恒为 0 的直行路段，受 $\sigma_{\min} = 0.05\text{ m}$ 下限影响，合成验证集横向 $1\sigma$ 覆盖率为 `83.9%–86.8%`（偏保守）；而在下方真实 Waymo 冻结片段上（含真实驾驶横向微摆），`nn` 的 $1\sigma$ 覆盖率为 **`67.0%`（2 s）与 `68.1%`（3 s）**，高度贴合高斯理论值 `68.27%`。

---

### 3. `exp01` 开环预测对比（合成 14 场景 + 真实 Waymo 冻结片段）

#### 3.1 合成切片 2.0 s 时域实测均值（ADE `[m]`；Neural 附带 2 s Gaussian NLL `[nats]`）

| 场景族（窗口数） | CV | CA | CTRV | LR | `nn_traj` (ADE / NLL) | `nn` (ADE / NLL) | Oracle |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `cut_in a=+0` (92) | **0.1840** | 0.3646 | 0.3096 | 0.3195 | 0.2356 / -0.788 | 0.2908 / **-1.781** | 0 |
| `cut_in a=+2` (92) | 0.3635 | 0.4398 | 0.4611 | 0.6509 | 0.3462 / -0.536 | **0.2988** / **-1.557** | 0 |
| `cut_in a=-2` (92) | 0.3635 | 0.4398 | 0.4168 | 0.6509 | 0.2016 / -0.984 | **0.1679** / **-1.890** | 0 |
| `lead_brake` (23) | 0.9382 | **0.1959** | 0.9382 | 1.5565 | 0.2030 / -2.407 | 0.2035 / **-2.586** | 0 |
| `crossing` (23) | <1e-12 | <1e-12 | <1e-12 | <1e-12 | 0.0260 / -3.249 | 0.0201 / **-3.397** | 0 |
| **全体 14 场景均值 (322)** | 0.3273 | 0.3692 | 0.3917 | 0.5947 | 0.2402 / -1.064 | **0.2324** / **-1.921** | 0 |

#### 3.2 真实 Waymo 冻结片段零样本迁移（`tests/fixtures/`，严格区分 `primary` 与 `illustrative`）

> 按 [interfaces.md](interfaces.md) 约定，首批 5+5 主队列（`primary`, 50 窗口）与按结果挑选的说明案例（`illustrative`, 10 窗口）严格分表，不混入同一统计分母。当前权重未见过任何 Waymo 训练数据（零样本）。

| 队列 (窗口数) | 时域 | CV ADE | CA ADE | CTRV ADE | LR ADE | `nn_traj` ADE / NLL / $1\sigma$ | `nn` ADE / NLL / $1\sigma$ |
| --- | :---: | ---: | ---: | ---: | ---: | ---: | ---: |
| **`primary` (50)** | 2.0 s | 0.5979 | **0.4501** | 0.6084 | 0.8307 | 0.5591 / 2.932 / 57.8% | 0.5437 / **2.531** / **67.0%** |
| **`primary` (50)** | 3.0 s | 1.1738 | **0.9488** | 1.2154 | 1.4798 | 1.0769 / 5.726 / 59.9% | 1.0879 / **4.050** / **68.1%** |
| `illustrative` (10) | 2.0 s | 0.9255 | 0.6078 | 0.9720 | 1.5342 | **0.5130** / 2.195 / 39.0% | 0.5285 / **1.791** / 42.0% |
| `illustrative` (10) | 3.0 s | 1.9739 | 1.3352 | 2.1281 | 2.7991 | 0.9942 / 3.299 / 42.3% | **0.9575** / **3.073** / 48.0% |

---

### 4. 复现与验证门禁

```bash
conda activate aie1903_ad
# 1. 训练与 {128, 256, 512, 1024} 容量验证（有 data/waymo_{train,val}.npz 时自动读取 Waymo）
PYTHONPATH="$PWD/src" python experiments/train_waymo_neural.py \
    --train-npz data/waymo_train.npz \
    --val-npz data/waymo_val.npz \
    --hidden-dims 128 256 512 1024 \
    --epochs-warmup 25 --epochs-nll 35 --export-best-to src/pbs/prediction/weights/

# 2. exp01 七模型开环评测（保留默认 L2 与 --include-lr 原有行为）
PYTHONPATH="$PWD/src" python experiments/exp01_predictor_ade.py --include-lr --include-nn --include-waymo-clips

# 3. 专项与全仓回归门禁
PYTHONPATH="$PWD/src" pytest tests/test_neural_predictor.py tests/test_risk.py -q
PYTHONPATH="$PWD/src" pytest -q
node --test tests/test_presentation_events.cjs
ruff check src tests experiments examples
git diff --check
```

**限制与说明**：
- 当前入库权重基于合成语料零样本训练，在 Waymo `primary` 队列上已优于 CV / CTRV / LR 且 $1\sigma$ 校准度良好，但尚未在 Waymo 真实训练集上微调；待 A-T2 导出 `data/waymo_train.npz` / `waymo_val.npz` 后直接重跑上述命令即可无代码修改升级权重。
- `latest_std` $(\sigma_{\text{lon}}, \sigma_{\text{lat}})$ 在 $t_0$ 车体局部系中预测，在急弯大角度转向（如 $90^\circ$）远期步上相对瞬时航向存在小幅轴向近似。
- 开环预测误差不能直接等同于闭环安全性，碰撞率、误刹率与分级制动舒适性需由 D 在 `exp02` / `exp03` / `exp07` / `exp08` 中闭环评估。

---

## 第二部分：既有 L3 基线 —— LR 预测器验证（2026-11-18）

基于合并 PR #8 后的 main `321528a`，在 `codex/p-l3-lr` 实现文档分工中的 P L3：
`LinearRegressionPredictor` 及其 ADE/FDE 验证。按现有分工，latency 实验和可选 nuScenes
不属于本轮 P 的模块交付；LR 可由 D 的既有注册表和实验矩阵直接调用。

### 方法与接口

- 默认最近至多 10 个位置样本，按实际时间拟合二维直线 p(t)=b+v·(t−t_last)。
  相对时间和位置缩放避免大绝对时间/坐标导致病态求解。只依赖过去位置，
  至少 2 帧时不使用速度/航向元数据，也不读取真值加速度或绑定场景的未来。
- 预测用拟合截距 b，有别于强制通过最后噪声观测的 CV；单帧用原 CV 速度/航向退化。
  `m` 必须为整数 ≥ 2；其余输入校验复用已有 `prediction_inputs`。
- 原模型名 `lr`、predict/bind、`(H,2)` 输出和时间锚点保持兼容；允许零时域。
  不修改输入。现有 `PredictiveAssessor` 继续负责延迟对齐。
- 不引入新依赖，不修改 A 的场景、C 的仿真、D 的矩阵或 `CORE` 四边界集合。
  exp01 默认 L2 四模型入口保留，`--include-lr` 追加 LR，CLI 默认另存 L3 目录。

### 复现与验证

```bash
conda activate aie1903_ad
python -m pytest tests/test_l3_predictors.py -q -o cache_dir=outputs/logs/l3/.pytest_cache
python -m pytest -q -k 'not test_http_assets_errors_and_results' -o cache_dir=outputs/logs/l3/.pytest_cache
python experiments/exp01_predictor_ade.py --include-lr
```

Windows 未激活环境时可用 `conda run --no-capture-output -n aie1903_ad python ...`。
CLI 子进程验证需要 `PYTHONPATH` 指向仓库的绝对 `src` 路径。

LR 专项 32 项通过：解析直线/不规则采样/大时间坐标、窗口最小二乘结果、过去窗口范围、
真值及元数据隔离、单帧退化、静止/零时域/输入校验/不修改输入、观测噪声、延迟对齐、
三类场景仿真、既有 D 的 `run_matrix`、五模型 CLI 在相同预测起点上的 CSV/图导出。
20 个固定随机种子的匀速噪声样例中，LR 平均 ADE 小于两点 CV 的 30%；该断言只用于
验证窗口平滑的效果，不代表所有运动类型。

最终全仓回归 332 passed / 1 deselected；排除项仍是已记录的 Windows Demo 端口测试，
未修改其代码。变更 Python 文件 Ruff、Black 与 `git diff --check` 通过。
L2 默认入口也在回归中验证；L3 误差图已目视检查标签、图例和布局。

### 误差与限制

输出 `outputs/figures/l3-predictors/`：`prediction_errors.csv`、`predictor_ade_fde.csv`、
`exp01_predictor_ade.png`、`summary.json`。五模型共用 A 的固定历史/真值切片：14 个场景、
各 23 个预测起点、0.5/1/2/3 s 时域，共 6440 行误差数据。

2 s 时域实测 ADE 均值（m）：

| 场景族 | CV | LR |
| --- | ---: | ---: |
| cut-in，a=0 | 0.1840 | 0.3195 |
| cut-in，a=±2（各档） | 0.3635 | 0.6509 |
| lead-brake | 0.9382 | 1.5565 |
| 匀速 crossing | <1e-12 | <1e-12 |

在这些无观测噪声、运动变化明显的合成切片上，窗口滞后使 LR 高于 CV，符合线性趋势
模型的限制。噪声匀速样例的收益与这里的代价需同时报告，不能只挑选有利场景。

LR 只拟合线性趋势，在接近匀速且有观测噪声时能平滑误差；加速、刹车、quintic 换道
期间窗口会滞后，长时域可能过冲，不能代替 CA/CTRV 或声称一定优于 CV。
该实验只比较合成预测误差，安全边界、误刹率与真实交通效果需相应实验验证。

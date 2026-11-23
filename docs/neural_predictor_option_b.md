# L3 方案 B 神经网络预测器构建与 `{128, 256, 512, 1024}` 容量验证规范

> **维护者**：P（华华 · `pbs.prediction` & `pbs.risk`） · **协同**：A（Persistent · `pbs.data`）、C（前背痛 · `pbs.sim`）、D（yj · `pbs.eval`）
> **基准分支**：`main`（`c74a3de`）
> **关联文档**：[L3 全局总纲](L3_global_guide.md) · [接口冻结规范](interfaces.md) · [CA 与固定切片底座](CA_scenario_support.md) · [L2 闭环基线](l2-baseline.md) · [L3 LR 验证报告](l3-validation.md) · [三级滞回与双档 Jerk](staged_braking_and_jerk.md)

---

## 1. 在既有 `docs/` 文档架构中的定位与接口边界

为确保神经网络模块无缝嵌入现有仓库且不破坏任何已合入的 L1 / L2 / L3(LR) 契约，本模块严格遵循既有文档体系的 4 条边界：

| 对接文档 | 既有冻结约定 | `NeuralPredictor`（方案 B）的对齐方式 |
| --- | --- | --- |
| **[docs/interfaces.md](interfaces.md)** | `Predictor.predict(history, horizon, dt) -> (horizon, 2)`；可选 `bind(scenario)`；`horizon=0` 返回 `(0, 2)`；严禁在 `predict()` 中读取 `Trajectory.accel` 真值或修改输入 `history` | `NeuralPredictor.predict()` 签名与返回形状 `(H, 2)` 完全一致；通过 `bind(scenario)` 只读缓存 `scenario.ego_path` 与 `lane_width`；异方差不确定度 $(\sigma_{\text{lon},k}, \sigma_{\text{lat},k})$ 保存在实例属性 `self.latest_std: (H, 2)`，不破坏返回值契约 |
| **[docs/CA_scenario_support.md](CA_scenario_support.md)**<br>**[docs/l3-validation.md](l3-validation.md)** | `prediction_slices(scenario, history=1.0, horizon=..., stride=0.5)` 提供固定的 `[t0 − 1.0, t0]` 滑动历史窗口（11 个采样点 = 10 个 `0.1 s` 间隔），`CA` / `CTRV` / `LR` 均使用 `m=10` 帧 | 神经网络同样固定读取最近 `1.0 s`（10 帧）历史，在 `exp01_predictor_ade.py` 中与 `CV` / `CA` / `CTRV` / `LR` / `Oracle` 共用完全相同的预测起点 `t0` 与真值切片 |
| **[docs/l2-baseline.md](l2-baseline.md)** | `PredictiveAssessor` 负责观测延迟对齐（`age = t_now − history.t[-1]`），并在 L2/L3 矩形模式下调用 `assess_risk(..., swept=True)` 做步间连续扫掠 | 延迟对齐与步间线性扫掠逻辑保持不变；`PredictiveAssessor` 新增可选参数 `uncertainty_scale: float = 0.0`（默认 `0.0` 不膨胀；设为 $\kappa = 1.0$ 时按 `predictor.latest_std` 动态膨胀目标车碰撞包络） |
| **[docs/d-baseline.md](d-baseline.md)** | `pbs.systems.SYSTEMS` 统一注册被比较的 AEB 系统，`run_matrix` 驱动 `exp02` / `exp03` / `exp05` | 在 `systems.py` 中新增 `"nn_traj"`（60 维纯轨迹）、`"nn"`（80 维含车道，$\kappa=0$）、`"nn_ua"`（80 维含车道 + 包络膨胀，$\kappa=1.0$） |

---

## 2. 模型在整体 AEB 闭环中的三重作用

在每 `dt = 0.1 s` 的 Receding Horizon 闭环步中，方案 B 神经网络通过以下链路改善 AEB 的安全边界与驾乘舒适度：

```text
[1.0 s 历史轨迹] + [bind(scenario) 提供的参考车道 ego_path]
        │
        ▼ (1) Physics-Anchored ResMLP (NeuralPredictor)
  • 局部均值轨迹 (x̂_k, ŷ_k) = p_cv,k + Δμ_k
    ──► 学会“切入到车道中心线后自动回正、弯道顺沿曲率行驶”，消除 CV/CA 的横向过冲误报
  • 局部异方差标准差 (σ_lon,k, σ_lat,k)
    ──► 在变道初期、远期时域 (2.5~4.0 s) 或观测加噪时自动放大 σ_k，诚实量化不确定性
        │
        ▼ (2) Uncertainty-Aware Swept TTC (PredictiveAssessor, κ = 1.0)
  • 在第 k 步扫掠区间将目标车碰撞矩形半长/半宽膨胀 (+κ·σ_lon,k, +κ·σ_lat,k)
    ──► 危险切入初期提前 0.2~0.4 s 触发冲突判定；邻道平稳巡航时 σ_lat 极小，不侵入本车道
        │
        ▼ (3) 联动三级滞回 + 双档 Jerk 控制器 (StagedBrakingPolicy)
  • TTC < 3.0 s 先触发一级舒适预制动 (-2.5 m/s², jerk_soft = 5 m/s³)，平顺降速并完成“预建压”
  • 若升级为 TTC < 1.5 s 二级急刹 (-6.0 m/s², jerk_hard = 20 m/s³)，仅需 0.2 s 即可踩满！
```

---

## 3. 跨场景同构的输入特征、输出定义与样本筛选

为确保**同一个训练出来的模型既能跑 Waymo 真实弯道/切入，又能跑 A 的合成参数化场景**，所有特征均在**以目标车当前最后观测时刻 $t_0$ 的位置 $p_0 = (x_0, y_0)$ 为原点、当前航向 $\theta_0$ 为 $+x$ 轴的局部坐标系（Agent-Centric Frame）**下提取。

### 3.1 输入特征向量 $x \in \mathbb{R}^{80}$（`NN-Traj` 取前 60 维，`NN-Lane` 取全部 80 维）

| 模块 | 索引范围 | 维度 | 物理定义（均在目标车 $t_0$ 局部系下） | 归一化因子 |
| --- | :---: | :---: | --- | :---: |
| **1. 局部历史轨迹 (`hist_traj`)** | `[0 : 60]` | $10 \times 6 = 60$ | 在时间网格 $\tau_i = t_0 - (9-i)\cdot 0.1\text{ s}$（$i=0\dots 9$）上插值：<br>• $\Delta x_i, \Delta y_i$：相对 $p_0$ 的局部坐标 `[m]`<br>• $v_{x,i}, v_{y,i}$：局部纵/横向速度 `[m/s]`<br>• $\cos\Delta\theta_i, \sin\Delta\theta_i$：相对 $\theta_0$ 的航向角余弦/正弦 | 位移 `/ 20.0`<br>速度 `/ 20.0`<br>三角函数 `/ 1.0` |
| **2. 相对参考车道 (`lane_ctx`)** | `[60 : 80]` | $6 + 14 = 20$ | 将 $p_0$ 投影到 `scenario.ego_path`（弧长 $s_0$，切向 $\hat{t}_{\text{lane}}$）：<br>• `[60]`：带符号横向偏移 $d_{\text{lat}} / \text{lane\_width}$（左正右负）<br>• `[61, 62]`：沿车道切向速度 $v_{\text{lon}}/20.0$、法向切入速度 $v_{\text{lat}}/5.0$<br>• `[63, 64]`：相对车道夹角 $\cos\Delta\theta_{\text{lane}}, \sin\Delta\theta_{\text{lane}}$<br>• `[65]`：归一化车道宽 $\text{lane\_width} / 3.5$<br>• `[66 : 80]`：`ego_path` 在前方弧长 $s_0 + \{5, 10, 15, 20, 30, 40, 50\}\text{ m}$ 处 7 个采样点转到目标车局部系下的 $(dx/40.0, dy/40.0)$ | 均已归一化至 $\approx [-1, 1]$ |

> **短历史退化规则**（兼容仿真前 `0.9 s` 或单帧输入）：当 `history` 时长不足 `0.9 s` 时，对早于 `history.t[0]` 的采样点用首帧速度反向匀速外推补齐 10 帧；当仅含 1 帧且速度为 0 时，`NeuralPredictor` 直接退化为 `ConstantVelocityPredictor`（保持与 `CA`/`CTRV`/`LR` 退化约定一致）。

### 3.2 输出张量 $y \in \mathbb{R}^{40 \times 4}$（未来 `4.0 s`，步长 `0.1 s`）

对未来第 $k \in \{1, \dots, 40\}$ 步（时刻 $t_0 + k \cdot 0.1\text{ s}$）：
1. **CV 物理残差锚定的局部位置均值 $(\hat{x}_k, \hat{y}_k)$ `[m]`**：
   $$\hat{x}_k = v_{x,\text{local}}(t_0) \cdot (k \Delta t) + \Delta\hat{x}_k, \qquad \hat{y}_k = v_{y,\text{local}}(t_0) \cdot (k \Delta t) + \Delta\hat{y}_k$$
   - 经逆旋转平移 $\hat{p}_{k,\text{world}} = p_0 + R(\theta_0)^\top [\hat{x}_k, \hat{y}_k]^\top$ 转回世界坐标作为 `predict()` 返回值。
2. **正值钳位的局部位置标准差 $(\sigma_{\text{lon}, k}, \sigma_{\text{lat}, k})$ `[m]`**：
   $$\sigma_{\text{lon}, k} = \text{clip}\big(\text{softplus}(z_{\text{lon}, k}) + 0.05,\; 0.05,\; 5.0\big), \qquad \sigma_{\text{lat}, k} = \text{clip}\big(\text{softplus}(z_{\text{lat}, k}) + 0.05,\; 0.05,\; 5.0\big)$$
   - 保存在 `self.latest_std`（形状 `(H, 2)`）。当调用方请求的 `horizon != 40` 或 `dt != 0.1` 时，在 `0.1 s × 40` 网格上沿时间轴线性插值/外推。

### 3.3 训练集“机动样本（Maneuver）”重采样定量判据

在 `scripts/extract_waymo_npz.py` 中，对每个 `1.0 s 历史 + 4.0 s 未来` 切片：
1. **剔除静止车**：全 5 s 位移 $< 1.0\text{ m}$，或 $v(t_0) < 0.5\text{ m/s}$ 且未来最大车速 $< 1.0\text{ m/s}$。
2. **机动样本判定**（满足任一即为 `is_maneuver = True`）：
   - 未来 4 s 局部横向位移 $\max_k |y_k^{\text{gt}}| > 0.8\text{ m}$，或航向总变化 $\max_k |\Delta\theta_k| > 0.14\text{ rad}\;(8^\circ)$；
   - 未来 4 s 车速极差 $\max_k v_k - \min_k v_k > 2.5\text{ m/s}$；
   - 相对 2D 匀速基线的 4 s 终点误差 $\text{FDE}_{\text{CV}} = \|p_{40}^{\text{gt}} - p_{40}^{\text{cv}}\|_2 > 1.5\text{ m}$。
3. **重采样比例**：按 **机动样本 : 平稳巡航样本 $\approx 1 : 1$** 构建训练集（建议总计 `30,000 ~ 60,000` 条切片），并按 **Scenario ID（80% / 20%）** 严格隔离训练集与验证集。

---

## 4. 网络架构与 `{128, 256, 512, 1024}` 四档容量验证协议

### 4.1 统一残差网络架构（`PhysicsAnchoredResMLP`）

所有容量档位共享同一套拓扑结构，仅改变隐藏层宽度 $H_{\text{dim}} \in \{128, 256, 512, 1024\}$：

```text
Input x ∈ R^D (D = 80 for NN-Lane, 60 for NN-Traj)
  │
  ├─► Stem:       h0 = SiLU( LayerNorm( Linear(D → H_dim)(x) ) )
  ├─► ResBlock 1: h1 = h0 + Linear(H_dim → H_dim)( SiLU( LayerNorm( Linear(H_dim → H_dim)(h0) ) ) )
  ├─► ResBlock 2: h2 = h1 + Linear(H_dim → H_dim)( SiLU( LayerNorm( Linear(H_dim → H_dim)(h1) ) ) )
  │
  ├─► Mean Head:        Δμ = Linear(H_dim → 80)(h2).reshape(40, 2)  ──► μ_local = p_cv + Δμ
  └─► Uncertainty Head: z_σ = Linear(H_dim → 80)(h2).reshape(40, 2) ──► σ_local = clip(softplus(z_σ)+0.05, 0.05, 5.0)
```

> **纯 NumPy 自适应推理特性**：前向推理函数 `forward_resmlp_numpy(feat, weights, cv_anchor)` 直接根据 `.npz` 中权重矩阵 `in_w.shape[0]` 自动适配 $H_{\text{dim}}$，**切换 `128 / 256 / 512 / 1024` 时 `src/pbs/prediction/neural.py` 一行代码都不用改**！

### 4.2 四档容量规模参数与理论复杂度对照表

以 `D = 80`（`NN-Lane`）、`2` 个残差块为例，精确参数量计算公式为：
$$N_{\text{params}}(H) = \underbrace{(80 H + H)}_{\text{Stem Linear}} + \underbrace{2 H}_{\text{Stem LN}} + 2 \times \underbrace{(2 H^2 + 4 H)}_{\text{ResBlock (2 Linear + LN)}} + 2 \times \underbrace{(80 H + 80)}_{\text{Dual Heads}} = 4 H^2 + 251 H + 160$$

| 模型档位 | 隐藏维 $H_{\text{dim}}$ | 精确参数量 | Float32 `.npz` 体积 | 单次纯 NumPy CPU 推理耗时（预估） | 跑完 `exp03` 全量（~23 万次调用）耗时 |
| --- | :---: | ---: | ---: | :---: | :---: |
| **`h128`** | `128` | **97,824**（~9.8 万） | **~0.39 MB** | **~0.03 ms** | ~7 s |
| **`h256`** | `256` | **326,560**（~32.7 万） | **~1.31 MB** | **~0.08 ms** | ~18 s |
| **`h512`** | `512` | **1,177,248**（~117.7 万） | **~4.71 MB** | **~0.25 ms** | ~58 s |
| **`h1024`** | `1024` | **4,451,488**（~445.1 万） | **~17.81 MB** | **~0.95 ms** | ~3.6 min |

### 4.3 四档容量的训练与选优验证协议（Verification Protocol）

为科学回答**“在 80 维低维物理输入下，`128 / 256 / 512 / 1024` 哪一档最好？”**，训练脚本 `experiments/train_waymo_neural.py` 固定随机种子 `seed=42`，在完全相同的训练集、数据增强与两阶段超参下依次训练这 4 个模型，并在 **5 个独立验证维度** 上自动打分：

#### 两阶段统一训练策略（每档 60 Epochs）
1. **Stage 1 · 均值预热（Epoch 1–25）**：仅优化时间加权 Smooth L1 Loss（对 $0.5 \sim 2.5\text{ s}$ 核心 TTC 决策窗口赋予权重 $1.2$），快速收敛位置均值 $\hat{\mu}_k$；
2. **Stage 2 · 异方差不确定度联合校准（Epoch 26–60）**：优化 Gaussian NLL Loss + 单调平滑正则（鼓励 $\sigma_k \ge \sigma_{k-1}$）：
   $$\mathcal{L} = \frac{1}{40}\sum_{k=1}^{40}\left[\frac{(x_k - \hat{x}_k)^2}{2\sigma_{\text{lon},k}^2} + \frac{(y_k - \hat{y}_k)^2}{2\sigma_{\text{lat},k}^2} + \log\sigma_{\text{lon},k} + \log\sigma_{\text{lat},k}\right] + 0.05 \sum_{k=2}^{40} \|\max(0, \sigma_{k-1} - \sigma_k)\|_2^2$$
3. **统一正则与增强**：`AdamW(lr=1e-3, weight_decay=1e-4)` + `CosineAnnealingLR`；50% 概率左右镜像翻转（$y, v_y, \Delta\theta, d_{\text{lat}}$ 取反）；30% 概率输入历史微噪注入（$\sigma_{\text{pos}}=0.08\text{ m}, \sigma_v=0.15\text{ m/s}$）。

#### 5 维选优评价指标（输出至 `outputs/logs/l3_neural_capacity/verification_summary.csv`）

| 评价维度 | 具体指标 | 用来检验什么问题？ |
| --- | --- | --- |
| **1. Waymo 留出验证集精度** | `val_ade_2s`, `val_fde_2s`, `val_ade_4s`, `val_fde_4s` `[m]` | 随着 $H_{\text{dim}}$ 从 `128` 增至 `1024`，真实交通预测误差在哪个档位开始饱和或因过拟合反弹？ |
| **2. 不确定度校准度（关键）** | `val_nll`、**$1\sigma$ 覆盖率偏差** $\|P(|e_y| \le \sigma_{\text{lat}}) - 0.6827\|$ | `512` / `1024` 大模型是否因为背下训练集而把 $\sigma$ 压得过小（Overconfidence），导致在验证集上 $1\sigma$ 覆盖率远低于理论值 $68.3\%$？ |
| **3. 合成切片跨域泛化（Real-to-Sim）** | 在 A 的 14 场景 × 23 起点切片（`exp01`）上的 `synth_ade_2s`, `synth_fde_2s` `[m]` | 大容量模型在合成 quintic 换道上是否出现过拟合伪影，哪一档跨域最稳？ |
| **4. 观测噪声鲁棒性** | 注入 `GaussianObservationModel(pos_std=0.15)` 后的 `noisy_ade_2s` `[m]` | 面对未平滑的传感器抖动，哪一档模型的轨迹和 $\sigma$ 最稳健？ |
| **5. 闭环推理效率** | 纯 NumPy 单步延迟 `latency_ms`（10,000 次均值）与 `.npz` 大小 `size_mb` | 在精度与校准度相当时，优先选择延迟最低、体积最小的一档入库。 |

#### 综合选优判据（Selection Rule）
- 在满足 **$1\sigma$ 横向覆盖率处于 $[60\%, 78\%]$（无严重过度自信）** 且 **合成跨域 `synth_ade_2s` 优于 `CV`** 的候选中，选取 **`score = 0.5 * val_ade_2s + 0.3 * val_ade_4s + 0.2 * synth_ade_2s` 最小**的模型作为胜出者（若两档 `score` 差距 $< 1.5\%$，按奥卡姆剃刀选参数量更小的一档）；
- **产物去向**：
  - 四档权重全部保存在 `outputs/models/waymo_nn_lane_h{128,256,512,1024}.npz`，生成四档对比图 `outputs/figures/l3-predictors/neural_capacity_verification.png`；
  - 将胜出档位的权重自动复制到 **`src/pbs/prediction/weights/waymo_nn_lane.npz`**（同理导出纯轨迹版 `waymo_nn_traj.npz`）随代码入库，并在 `docs/l3-validation.md` 中完整报告四档对比表！

---

## 5. 代码模块划分与实现接口（`src/pbs/prediction/` & `src/pbs/risk/`）

### 5.1 新增/修改文件清单

| 文件路径 | 职责 | 是否入库 Git |
| --- | --- | :---: |
| `src/pbs/prediction/features.py` | `extract_traj_features` (60D)、`extract_lane_features` (20D)、局部/世界坐标正逆变换 | ✅ |
| `src/pbs/prediction/neural.py` | `NeuralPredictor(Predictor)`：纯 NumPy 自适应宽度前向推理、`(H, 2)` 时间插值、`latest_std` 记录 | ✅ |
| `src/pbs/prediction/weights/waymo_nn_lane.npz` | 验证胜出的最优档位权重（默认 `NN-Lane`） | ✅ |
| `src/pbs/prediction/weights/waymo_nn_traj.npz` | 验证胜出的最优档位权重（消融对照 `NN-Traj`） | ✅ |
| `src/pbs/risk/ttc.py` & `assessors.py` | 支持 `uncertainty_scale` ($\kappa \cdot \sigma_k$) 包络膨胀与 `ctx.vehicle_length/width` 透传 | ✅ |
| `experiments/train_waymo_neural.py` | 支持 `--hidden-dims 128 256 512 1024` 一键训练、5 维自动验证选优、导出 `.npz` 与对比图表 | ✅ |
| `tests/test_neural_predictor.py` | 覆盖 `128/256/512/1024` 自适应加载、SE(2) 旋转平移等变性、边界校验与不确定度膨胀 TTC | ✅ |

### 5.2 一键训练与四档容量验证命令

```bash
# 1. A 从 Waymo TFRecord 导出训练与验证切片 (data/waymo_train.npz, data/waymo_val.npz)
PYTHONPATH="$PWD/src" python scripts/extract_waymo_npz.py --tfrecord-dir data/waymo/ --output-dir data/

# 2. P 在 GPU 上一键训练 {128, 256, 512, 1024} 四档容量并自动运行 5 维选优验证
PYTHONPATH="$PWD/src" python experiments/train_waymo_neural.py \
    --train-npz data/waymo_train.npz \
    --val-npz data/waymo_val.npz \
    --hidden-dims 128 256 512 1024 \
    --epochs-warmup 25 \
    --epochs-nll 35 \
    --export-best-to src/pbs/prediction/weights/

# 3. 运行 exp01 生成包含最优 NeuralPredictor 的开环 ADE/FDE/NLL 对比图
PYTHONPATH="$PWD/src" python experiments/exp01_predictor_ade.py --include-lr --include-nn

# 4. 运行神经网络与不确定度 TTC 专项测试
PYTHONPATH="$PWD/src" pytest tests/test_neural_predictor.py tests/test_risk.py -q
```

---

## 6. 单元测试与质量验收清单（`tests/test_neural_predictor.py`）

- [ ] **四档宽度自适应推理测试**：构造 `h ∈ {128, 256, 512, 1024}` 的合成权重字典传入 `NeuralPredictor`，断言无需修改代码即可正常输出 `(H, 2)` 均值与 `(H, 2)` 标准差。
- [ ] **接口冻结契约测试**：
  - `horizon=0` 返回形状 `(0, 2)`，`latest_std` 形状 `(0, 2)`；
  - 任意 `horizon`（如 `15, 30, 40`）与 `dt`（如 `0.1, 0.2`）均能正确插值对齐；
  - 污染 `history.accel` 真值或外部修改返回值不影响预测结果；原 `history` 数组不被篡改。
- [ ] **SE(2) 刚体变换等变性测试**：将 `Scenario`（含 `ego_path` 与 `history`）整体平移任意向量 $(\Delta X, \Delta Y)$ 并旋转任意角度 $\phi$，断言 `NeuralPredictor` 输出的世界坐标均值轨迹严格等于原输出做相同平移旋转，且 `latest_std` 逐位不变（误差 $< 10^{-5}$）。
- [ ] **不确定度包络膨胀 TTC 单调性测试**：在相同预测中心轨迹下，断言 `PredictiveAssessor(predictor, uncertainty_scale=1.0).assess(ctx, hist).ttc <= PredictiveAssessor(predictor, uncertainty_scale=0.0).assess(ctx, hist).ttc`。

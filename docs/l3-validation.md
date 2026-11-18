# P 的 L3：LR 预测器验证（2026-11-18）

基于合并 PR #8 后的 main `321528a`，在 `codex/p-l3-lr` 实现文档分工中的 P L3：
`LinearRegressionPredictor` 及其 ADE/FDE 验证。按现有分工，latency 实验和可选 nuScenes
不属于本轮 P 的模块交付；LR 可由 D 的既有注册表和实验矩阵直接调用。

## 方法与接口

- 默认最近至多 10 个位置样本，按实际时间拟合二维直线 p(t)=b+v·(t−t_last)。
  相对时间和位置缩放避免大绝对时间/坐标导致病态求解。只依赖过去位置，
  至少 2 帧时不使用速度/航向元数据，也不读取真值加速度或绑定场景的未来。
- 预测用拟合截距 b，有别于强制通过最后噪声观测的 CV；单帧用原 CV 速度/航向退化。
  `m` 必须为整数 ≥ 2；其余输入校验复用已有 `prediction_inputs`。
- 原模型名 `lr`、predict/bind、`(H,2)` 输出和时间锚点保持兼容；允许零时域。
  不修改输入。现有 `PredictiveAssessor` 继续负责延迟对齐。
- 不引入新依赖，不修改 A 的场景、C 的仿真、D 的矩阵或 `CORE` 四边界集合。
  exp01 默认 L2 四模型入口保留，`--include-lr` 追加 LR，CLI 默认另存 L3 目录。

## 复现与验证

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

## 误差与限制

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

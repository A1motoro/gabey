# P 的 L2 验证（2026-11-18）

本轮先 pull main `3b77312`，提交前再次同步并 rebase 到 main `8072b02`，
实现 P 的 CA、CTRV、Oracle，并完成 P 所属的
`exp01_predictor_ade.py`。只修改预测器、对应测试和交付文档；沿用已有场景、风险评估、
系统注册与仿真接口。LR 留待 L3，D 的 trade-off/failure-boundary 实验仍由 D 完成。

## 模型与边界

- **CA**：默认最近 10 个位置样本，以相对时间/位置缩放作二维二次最小二乘拟合，
  预测起点固定为最后观测位置。不使用真值加速度，也不对课程数据的速度列差分。
  不足 3 帧退化为 CV；沿当前速度方向的减速积分在零速度处截断，垂直加速度仍外推。
  已静止观测及停车附近的拟合反向过冲保持原位。
- **CTRV**：默认最近 10 帧解缠航向，使用真实时间间隔拟合转率；末帧速度/航向驱动
  恒转率圆弧。用 sinc 中角形式避免小转率数值消减，零转率或单帧退化为原二维 CV。
  相邻采样之间的实际转角应小于 π，速度/航向应可信；转率变化时仍有模型误差。
- **Oracle**：bind 时按唯一非空 agent ID 复制真值；按最后观测时间加 k·dt 插值。
  未 bind、未知 ID 或重复/空 ID 报错，重新 bind 替换原映射。超出真值范围与 `state_at`
  一样保持端点；这不是未知未来的真值，解释为上界前必须保证预测时域有充分覆盖。
- 三者不修改输入，预测时间与输出形状不变。可选窗口构造参数为兼容追加。
  现有 `PredictiveAssessor` 负责观测延迟对齐，非整步时间仍使用已有网格线性插值近似。

## 复现

```bash
conda activate aie1903_ad
python -m pytest tests/test_predictors.py tests/test_l2_predictors.py -q -o cache_dir=outputs/logs/l2/.pytest_cache
python -m pytest -q -k 'not test_http_assets_errors_and_results' -o cache_dir=outputs/logs/l2/.pytest_cache
python experiments/exp01_predictor_ade.py
```

Windows 未激活环境时使用
`conda run --no-capture-output -n aie1903_ad python ...`。
从临时目录启动的测试子进程需 `PYTHONPATH` 指向仓库的绝对 `src` 路径。

L2 专项覆盖：不规则采样下的二维恒加速度、旋转坐标、刹停/停车过渡、短历史退化、
GT 加速度污染不影响预测、窗口范围、正/负/极小转率与 ±π 跨界、Oracle 多目标身份和
重绑定/插值/端点规则、零时域/输入校验/不修改输入、延迟风险对齐，三类 A 场景经
原 C 仿真和 D 的注册表运行，以及 exp01 的独立 CLI 导出。

全仓初次测试发现已有 `tests/test_demo.py::test_http_assets_errors_and_results` 超时：
该测试在本机 Windows 上启动第二个服务检查占用端口，子进程未在 10 s 内退出。
它使用 CV，未经过本轮 CA/CTRV/Oracle；按本轮边界未修改 Demo 或该测试。
其余回归全部通过，完整测试结果不应表述为全仓无失败。

最终验证（main `8072b02` 上）：L2 专项 69 项通过；排除上述既有失败后的全仓回归
286 passed、1 deselected，没有跳过项。变更 Python 文件 Ruff 与 Black 检查通过，
`git diff --check` 通过；误差图已目视检查标签、图例与布局。

## 误差证据

输出目录：`outputs/figures/l2-predictors/`，包含逐切片 `prediction_errors.csv`、聚合
`predictor_ade_fde.csv`、`exp01_predictor_ade.png` 和 `summary.json`。
使用 A 的 12 个 CA cut-in 场景，加默认 lead-brake/crossing，共 14 个场景。
各场景统一起点 t0=1.0…12.0 s（间隔 0.5 s，共 23 个），四种预测器共用同一历史与真值，
0.5/1/2/3 s 时域均用这些起点。`prediction_slices(history=1.0)` 提供固定的
`[t0−1.0, t0]` 历史（11 点）；CA/CTRV 内部仅取最近 10 帧。

2 s 时域的 ADE 均值（单位 m）：

| 场景族 | CV | CA | CTRV | Oracle |
| --- | ---: | ---: | ---: | ---: |
| cut-in，a=0（92 切片） | 0.1840 | 0.3646 | 0.3096 | 0 |
| cut-in，a=+2（92 切片） | 0.3635 | 0.4398 | 0.4611 | 0 |
| cut-in，a=−2（92 切片） | 0.3635 | 0.4398 | 0.4168 | 0 |
| lead-brake（23 切片） | 0.9382 | 0.1959 | 0.9382 | 0 |
| crossing（23 切片） | <1e-12 | <1e-12 | <1e-12 | 0 |

Oracle 所有覆盖内切片的最大坐标误差为 0。CA 在恒减速前车上明显降低误差；
quintic 换道的加速度/转率随时间改变，CA/CTRV 的长时域外推会过冲，聚合误差可高于 CV。
这说明模型具有适用条件，不能声称复杂模型恒优于 CV。上述数字只描述合成预测误差，
不代替碰撞率、误刹率或四边界安全收益。

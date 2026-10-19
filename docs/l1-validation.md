# P 的 L1 基线验证（2026-10-09）

本轮完成 P 的 L1：二维 CV、首次冲突 TTC、Reactive/Predictive assessor，
以及 CV ADE/FDE 与同一 cut-in 的刹车时间对照。范围以用户确认和 [README](../README.md)
为准。A 的场景 API、C 的仿真模块、D 的实验矩阵未作实现改动；CA、CTRV、Oracle 留待 L2，LR 留待 L3。

## 五轮测试结果

五轮验证最终均通过。第三轮因环境启动方式导致原生异常而重跑一次，实际执行六次 pytest；
下表是每轮完成时的结果，不把中断的首次第三轮计为通过。

| 轮次 | 验证范围 | 通过 | 失败 | 跳过 | pytest 用时 | 日志 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 1 | CV、单样本回退、横向速度、非均匀采样、输入不变性 | 9 | 0 | 0 | 0.43 s | [round-1.log](../outputs/logs/l1-baseline/round-1.log) |
| 2 | 首次冲突、严格半径、Reactive/CV 区别、车道投影、时间对齐 | 18 | 0 | 0 | 0.24 s | [round-2.log](../outputs/logs/l1-baseline/round-2.log) |
| 3 | 输入边界、20 组随机 CV、20 组 TTC 不变性、旋转车道 | 65 | 0 | 0 | 0.46 s | [round-3.log](../outputs/logs/l1-baseline/round-3.log) |
| 4 | 仿真集成、counterfactual、延迟、多目标、重复性、证据脚本 | 22 | 0 | 0 | 4.44 s | [round-4.log](../outputs/logs/l1-baseline/round-4.log) |
| 5 | 全仓库回归 | **114** | **0** | **2** | 4.34 s | [round-5.log](../outputs/logs/l1-baseline/round-5.log) |

前四轮覆盖不同测试集合，共 114 项；第五轮再次全量运行，不能相加当成独立用例数。
每轮同时保存 `outputs/logs/l1-baseline/round-N.xml`（JUnit），错误和失败均为 0。
第 1 轮排除两项 L2 用例；第 5 轮的两个 skip 是原有的
`test_ctrv_degenerates_to_cv`、`test_oracle_exact`。L1 测试没有跳过。

## 环境问题与处理

- 第 1 轮出现已有 `.pytest_cache` 无法写入的警告；后续指定缓存路径后消失，断言没有失败。
- 第 3 轮首次已通过 41 项，随后矩阵运算触发 Windows 原生异常 `0xc06d007f`，未生成完整 JUnit。
  [首次异常日志](../outputs/logs/l1-baseline/round-3-initial-crash.log) 已保留。
- 直接调用课程环境的 `python.exe` 可以运行部分 NumPy 操作，却不足以提供全部原生库运行环境。
  用 `conda run -n aie1903_ad` 后相同矩阵运算、全部 65 项测试及第 4、5 轮均通过。
  没有改写测试绕开矩阵运算；未进一步确认具体缺失的 DLL 名称。
- 已按 README 安装 `requirements.txt` 及可编辑项目，不再借用其他 conda 环境的包。
  验证环境：Windows、Python 3.10.21、pytest 9.1.1、NumPy 1.26.4。
- 涉及改动的 11 个 Python 文件均通过 Ruff、Black；`git diff --check` 通过。

## CV ADE/FDE 证据

P 的解析样例在 `experiments/l1_fixtures.py`，与 M1 的生产 `make_scenario` 无依赖。
每个 0.5–3.0 s 预测时域取 1.0、1.5、2.0、2.5 s 四个历史窗口，报告平均误差。
二维匀速样例最大 ADE 约 `2.86e-14 m`、最大 FDE 约 `5.34e-14 m`（浮点误差量级）。
minimum-jerk cut-in 在 3 s 时域的 ADE 为 **1.504 m**、FDE 为 **3.326 m**，CV 对变化的横向速度不精确。

![CV ADE/FDE](../outputs/figures/l1-baseline/cv_ade_fde.png)

原始数据：[cv_ade_fde.csv](../outputs/figures/l1-baseline/cv_ade_fde.csv)。

## Reactive vs CV cut-in

ego 初速 15 m/s，agent 纵向速度 10 m/s，初始 gap 20 m。换道从 0.5 s
开始、持续 3 s，横向位移 3.5 m，解析峰值横向加速度约 2.245 m/s²。
仿真时长 6 s、步长 0.1 s、预测时域 3 s、TTC 刹车阈值 2 s、`r_safe=2.5 m`。

| 系统 | 首次刹车时间 | 碰撞 | 最小中心距离 |
| --- | ---: | --- | ---: |
| Reactive | 2.0 s | 否 | 3.98 m |
| CV | 1.7 s | 否 | 3.86 m |

同一场景在永不刹车的 counterfactual 中发生碰撞。CV 首次刹车提前 **0.3 s**，
但最小距离并未更大；提前察觉与安全区扩大需要分别验证。现有阈值策略出现多段刹车，保持 C 的策略语义。
结果仅验证固定合成样例中的方法区别，不能据此得出所有场景的安全收益或 failure boundary。

![Reactive/CV timeline](../outputs/figures/l1-baseline/cut_in_timeline.png)

数据：[Reactive timeline](../outputs/figures/l1-baseline/reactive_timeline.csv)、
[CV timeline](../outputs/figures/l1-baseline/cv_timeline.csv)、
[summary.json](../outputs/figures/l1-baseline/summary.json)；
脚本日志：[evidence.log](../outputs/logs/l1-baseline/evidence.log)。

## 复现

按 README 激活课程环境并安装项目后依次执行。Windows 未激活环境时，
将每条命令的 `python` 替换为 `conda run --no-capture-output -n aie1903_ad python`。

```bash
python -m pytest tests/test_predictors.py -k "not ctrv and not oracle" -v -o cache_dir=outputs/logs/l1-baseline/.pytest_cache
python -m pytest tests/test_risk.py -v -o cache_dir=outputs/logs/l1-baseline/.pytest_cache
python -m pytest tests/test_baseline_edges.py -v -o cache_dir=outputs/logs/l1-baseline/.pytest_cache
python -m pytest tests/test_baseline_integration.py tests/test_scenarios.py tests/test_counterfactual.py tests/test_collision.py -v -o cache_dir=outputs/logs/l1-baseline/.pytest_cache
python -m pytest -q -rs -o cache_dir=outputs/logs/l1-baseline/.pytest_cache
python experiments/exp00_l1_baseline.py
```

完整日志和图表位于 Git 忽略的 `outputs/` 中，复制项目后可按上述命令重建。
第 4、5 轮的 CLI 测试在临时目录独立生成并核验产物。
下一轮推进 P 的 L2：CA、CTRV、Oracle 与对应测试；之后处理 L3 的 LR。
AI 使用记录已更新，人工复核待用户查看报告与代码。

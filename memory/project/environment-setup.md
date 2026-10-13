# 环境约束（本机 Windows + conda）

## 当前结论

- 课程解释器位于 `C:\Users\14441\anaconda3\envs\aie1903_ad\python.exe`（Python 3.10.21）。2026-10-09 已按 README 安装开发依赖并以可编辑方式安装项目；原“缺 pytest/black/ruff”已失效。
- 同日裸 `python` 仍指向系统 Python 3.13；应激活 `aie1903_ad`，或用 `C:\Users\14441\anaconda3\Scripts\conda.exe run --no-capture-output -n aie1903_ad python ...`。

## 失败方案

- 直接调用课程环境的 `python.exe`，部分 NumPy 操作可运行，但第三轮矩阵运算触发原生异常 `0xc06d007f`；改用 `conda run` 后相同运算与全部测试通过。依据：2026-10-09 实测；未确认具体缺失的 DLL 名称。
- 旧记忆曾建议用 `htn-learner` 的整个 `site-packages` 作为 `PYTHONPATH` 借用 pytest。现已在课程环境正式安装，不再需要该跨环境方案。
- 根目录已有 `.pytest_cache` 无法写入；本轮后续使用 `-o cache_dir=outputs/logs/l1-baseline/.pytest_cache` 成功。无需删除原目录。

## 日志线索

- `outputs/logs/l1-baseline/round-3-initial-crash.log` 记录直接启动时的异常；`round-3.log`、`round-4.log`、`round-5.log` 记录通过 conda 环境启动的验证。摘要与命令见 [L1 报告](../../docs/l1-validation.md)。

## 修订依据

- 2026-10-09：本轮实际安装及测试覆盖先前环境状态；没有关于网络受限的可靠记录，不能恢复此前被移除的断言。

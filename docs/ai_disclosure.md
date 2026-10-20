# AI Tool Usage Disclosure

按课程 AI Tool Usage Policy 记录。每条注明：谁、工具、用途、产出如何被检查。

| 日期 | 成员 | 工具 | 用途 | 涉及文件 | 人工检查方式 |
| --- | --- | --- | --- | --- | --- |
| 2026-09-29 | A1m | Claude | 生成项目骨架（目录、`types.py`、walking skeleton、文档模板） | 全仓库初始提交 | 阅读全部文件；smoke test 通过 |
| 2026-09-29 | A1m | Claude | 按 spec v2 重构骨架（三层 baseline、risk 包、counterfactual、实验矩阵）与文档 | `src/pbs/`、`docs/spec.md`、`docs/interfaces.md`、README | 阅读全部文件；smoke / counterfactual 测试通过 |
| 2026-10-09 | P（用户身份据既有记忆） | Codex | 实现 L1 CV、TTC、Reactive，编写验证样例和测试；排查 Windows 环境启动问题 | `src/pbs/prediction/`、`src/pbs/risk/`、L1 测试、`experiments/exp00_l1_baseline.py`、`experiments/l1_fixtures.py`、相关文档 | 五轮自动验证记录见 `docs/l1-validation.md`；人工复核待用户查看，未声称已完成人工检查 |
| 2026-10 | A | DSH（DeepSeek Harness）agent | 实现 M1 场景模块：quintic 换道、三类参数化场景、横向加速度过滤、场景网格与命名场景、`load_toy`；新增 `exp00` 验证脚本与 `tests/test_data.py` 测试 | `src/pbs/data/`、`experiments/exp00_scenarios.py`、`tests/test_data.py`、`docs/scenarios.md`、`docs/A_baseline_handoff.md` | `pytest tests -q`（38 passed / 6 skipped）、`ruff check` 通过；数值与解析公式交叉核对（峰值横向加速度、C² 端点、网格计数）。**本人仍需**逐行复核代码与产物（清单见 `docs/A_baseline_handoff.md` §9），并能在被抽问时独立解释 |

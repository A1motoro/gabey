# AI Tool Usage Disclosure

按课程 AI Tool Usage Policy 记录。每条注明：谁、工具、用途、产出如何被检查。

| 日期 | 成员 | 工具 | 用途 | 涉及文件 | 人工检查方式 |
| --- | --- | --- | --- | --- | --- |
| 2026-09-29 | A1m | Claude | 生成项目骨架（目录、`types.py`、walking skeleton、文档模板） | 全仓库初始提交 | 阅读全部文件；smoke test 通过 |
| 2026-09-29 | A1m | Claude | 按 spec v2 重构骨架（三层 baseline、risk 包、counterfactual、实验矩阵）与文档 | `src/pbs/`、`docs/spec.md`、`docs/interfaces.md`、README | 阅读全部文件；smoke / counterfactual 测试通过 |
| 2026-10-09 | P（用户身份据既有记忆） | Codex | 实现 L1 CV、TTC、Reactive，编写验证样例和测试；排查 Windows 环境启动问题 | `src/pbs/prediction/`、`src/pbs/risk/`、L1 测试、`experiments/exp00_l1_baseline.py`、`experiments/l1_fixtures.py`、相关文档 | 五轮自动验证记录见 `docs/l1-validation.md`；人工复核待用户查看，未声称已完成人工检查 |
| 2026-10-20 | C（姓名待填写） | OpenAI Codex | 实现 C 第一版：滞回、步内停车、输入/观测检查、子步碰撞、执行动作元数据及 D 统计兼容；新增测试、独立演示、接入与未启用降级策略说明 | `src/pbs/sim/`、`src/pbs/types.py`、`src/pbs/utils.py`、`src/pbs/eval/metrics.py`、`tests/test_policy.py`、`tests/test_simulator.py`、`tests/test_scenarios.py`、`examples/c_smoke.py`、`docs/interfaces.md`、`docs/C_baseline.md` | 用户确认 A/P 尚未交付并选择缺失时报错；AI 实测 38 passed、6 skipped（P TODO）、Ruff 通过、演示导出 21 行；人工代码审阅及 D 全组集成待完成，未 commit/push |
| 2026-10-20 | C（姓名待填写） | OpenAI Codex | 对照当前源码解释 C 的架构：统一入口、路径距离状态、历史/真值分离、P 接口、滞回、运动积分、步内碰撞和 SimLog | `src/pbs/sim/`、`src/pbs/types.py`、`src/pbs/risk/assessors.py`（只读）、本记录 | AI 读取实际代码并按行定位说明；用户理解与人工复核待完成，本轮未修改 Python 或重新运行测试 |
| 2026-10-20 | C | OpenAI Codex | 按用户要求创建本地钱北桐C部份分支并提交 C；合并远端 P L1，保留双方 AI 记录，更新 baseline 状态；用户随后要求审核后再推送 | C 的 13 个变更文件；远端 P 文件仅随主分支合并 | GitHub 连接报告 push 权限；合并后 142 passed、2 skipped、3 failed（两项缺少延迟前历史，一项环境缺 matplotlib）；尚未 push 或创建 PR，等待用户审核 |
| 2026-10-20 | C | OpenAI Codex | 用户审核后授权推送；修复延迟集成测试输入并准备 C 分支与 PR | tests/test_scenarios.py、tests/test_baseline_integration.py、docs/C_baseline.md、本记录 | 为测试显式提供预历史，未放宽缺失观测规则；临时环境补齐 matplotlib/pandas 并设置 PYTHONPATH；145 passed、2 skipped，变更 Python 文件 Ruff 与差异检查通过；完整场景实验仍由 D 完成 |
| 2026-10-20 | A | DSH（DeepSeek Harness）agent | 实现 M1 场景模块：quintic 换道、三类参数化场景、横向加速度过滤、场景网格与命名场景、`load_toy`；新增 `exp00` 验证脚本与 `tests/test_data.py` 测试 | `src/pbs/data/`、`experiments/exp00_scenarios.py`、`tests/test_data.py`、`docs/scenarios.md`、`docs/A_baseline_handoff.md` | `pytest tests -q`（38 passed / 6 skipped）、`ruff check` 通过；数值与解析公式交叉核对（峰值横向加速度、C² 端点、网格计数）。**本人仍需**逐行复核代码与产物（清单见 `docs/A_baseline_handoff.md` §9），并能在被抽问时独立解释 |
| 2026-10-20 | hzy | DSH（harness）agent | 编写中期计划与数据审计：DL1/DL2 交付项与时间表、`data/*.csv` 字段/时长/冲突片段审计 | `docs/midterm_plan.md`、`docs/data_audit.md` | 行数、时长、最近距离等数字由脚本对原始 CSV 复算；结论仅作参考，待 hzy 复核 |
| 2026-10-25 | hzy | DSH（harness）agent | 起草与改写 proposal 相关文档：§1–§6 骨架与页数预算、§2 相关工作中文对照、写作分工与硬性要求整理、参考文献候选清单 | `docs/proposal.md`、`docs/proposal_writing.md`、`docs/section2_zh.md`、`docs/references_candidates.md` | 每条论断标注 `path:line` 出处；文献四要素（作者/年份/题名/出处）标注为待核实，未核实条目不写入正文；**仍需 hzy 逐行复核**后提交 |
| 2026-10-25 | hzy | DSH（harness）agent | 核查 baseline 与文档的上传状态：本地 reflog 与 GitHub API 双向比对分支 tip SHA | 无代码改动 | 远端 `feat/m1-scenarios-baseline` tip 与本地 SHA 一致（`213ef43`） |

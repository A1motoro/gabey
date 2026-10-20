# AI Tool Usage Disclosure

按课程 AI Tool Usage Policy 记录。每条注明：谁、工具、用途、产出如何被检查。

| 日期 | 成员 | 工具 | 用途 | 涉及文件 | 人工检查方式 |
| --- | --- | --- | --- | --- | --- |
| 2026-09-29 | A1m | Claude | 生成项目骨架（目录、`types.py`、walking skeleton、文档模板） | 全仓库初始提交 | 阅读全部文件；smoke test 通过 |
| 2026-09-29 | A1m | Claude | 按 spec v2 重构骨架（三层 baseline、risk 包、counterfactual、实验矩阵）与文档 | `src/pbs/`、`docs/spec.md`、`docs/interfaces.md`、README | 阅读全部文件；smoke / counterfactual 测试通过 |
| 2026-10-20 | C（姓名待填写） | OpenAI Codex | 实现 C 第一版：滞回、步内停车、输入/观测检查、子步碰撞、执行动作元数据及 D 统计兼容；新增测试、独立演示、接入与未启用降级策略说明 | `src/pbs/sim/`、`src/pbs/types.py`、`src/pbs/utils.py`、`src/pbs/eval/metrics.py`、`tests/test_policy.py`、`tests/test_simulator.py`、`tests/test_scenarios.py`、`examples/c_smoke.py`、`docs/interfaces.md`、`docs/C_baseline.md` | 用户确认 A/P 尚未交付并选择缺失时报错；AI 实测 38 passed、6 skipped（P TODO）、Ruff 通过、演示导出 21 行；人工代码审阅及 D 全组集成待完成，未 commit/push |
| 2026-10-20 | C（姓名待填写） | OpenAI Codex | 对照当前源码解释 C 的架构：统一入口、路径距离状态、历史/真值分离、P 接口、滞回、运动积分、步内碰撞和 SimLog | `src/pbs/sim/`、`src/pbs/types.py`、`src/pbs/risk/assessors.py`（只读）、本记录 | AI 读取实际代码并按行定位说明；用户理解与人工复核待完成，本轮未修改 Python 或重新运行测试 |

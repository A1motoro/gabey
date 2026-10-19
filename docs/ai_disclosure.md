# AI Tool Usage Disclosure

按课程 AI Tool Usage Policy 记录。每条注明：谁、工具、用途、产出如何被检查。

| 日期 | 成员 | 工具 | 用途 | 涉及文件 | 人工检查方式 |
| --- | --- | --- | --- | --- | --- |
| 2026-09-29 | A1m | Claude | 生成项目骨架（目录、`types.py`、walking skeleton、文档模板） | 全仓库初始提交 | 阅读全部文件；smoke test 通过 |
| 2026-09-29 | A1m | Claude | 按 spec v2 重构骨架（三层 baseline、risk 包、counterfactual、实验矩阵）与文档 | `src/pbs/`、`docs/spec.md`、`docs/interfaces.md`、README | 阅读全部文件；smoke / counterfactual 测试通过 |
| 2026-10-18（补记本对话前序讨论） | C（姓名待填写） | OpenAI Codex | 解释项目目标、TTC、Scenario/Trajectory、RiskAssessor、配置与 SimLog；明确 C/P/A/D 边界；讨论灯光与神经网络扩展并将其排除在当前基础任务外 | 对话教学示例；阅读 `docs/spec.md`、`docs/interfaces.md`、`src/pbs/`，未因该讨论修改 Python 文件 | 用户通过多轮追问确认理解；尚未记录对全部示例的独立人工复算或代码测试 |
| 2026-10-18（补记本对话前序讨论） | C（姓名待填写） | OpenAI Codex | 辅助设计 run_sim pipeline 与伪代码、双阈值滞回策略、观测缺失处理、步内碰撞检查；生成并解释 11 个时刻的教学数据 | 对话伪代码与示例；`docs/C_Proposal_制作指导.md` | 用户明确选择将滞回纳入 C 设计；AI 使用独立计算脚本核算教学表格，非完整仿真实验；其余方法待人工审阅与实现测试 |
| 2026-10-18 | C（姓名待填写） | OpenAI Codex | 生成 C 部分 proposal 制作指导，整理接口、架构图、伪代码、验证计划和待确认问题 | `docs/C_Proposal_制作指导.md` | AI 对照本地方案与代码并检查 Markdown 代码围栏及本地链接；人工全文审阅待完成，未运行项目测试 |
| 2026-10-18 | C（姓名待填写） | OpenAI Codex | 按 spec/interfaces 修订指导，补充兼容性、原计划与新增设计的区别、gabey 内开发布局、Git 协作流程；按用户要求补记 AI 使用情况 | `docs/C_Proposal_制作指导.md`、`docs/ai_disclosure.md` | AI 核对当前文档、评估代码、Git 状态和差异；人工审阅待完成；未修改 Python、未提交或推送到 GitHub |
| 2026-10-18 | C（姓名待填写） | OpenAI Codex | 按用户请求验证 GitHub 推送权限：SSH 认证及 git push --dry-run --no-verify 预检 | 本地 origin 对应 `A1motoro/gabey`；本记录与指导文件 | AI 实测 SSH 身份为 qbt-tbq，目标仓库拒绝该身份推送；未上传提交或创建远程分支；人工复核待完成 |
| 2026-10-18 | C（姓名待填写） | OpenAI Codex | 阅读课程 project_topic_examples.md，区分选题示例与正式 proposal 要求，整理 Topic 9 对应的全组和 C 部分准备清单 | `AIE1903_AD_week01/projects/project_topic_examples.md`（仓库外课程材料，只读）、对话答复、本记录 | AI 阅读原文，并在同级 projects 与 Week 1 README 中检索 proposal 要求；未找到页数、截止日期或评分细则，人工确认课程正式要求待完成 |
| 2026-10-18 | C（姓名待填写） | OpenAI Codex | 阅读用户更正提供的正式 project_guidelines.md，整理 proposal 六节要求、展示/文档截止时间、MVP/扩展/备选方案、证据边界及 C 的准备清单 | `/Users/qianbt/Downloads/project_guidelines.md`（只读）、对话答复、本记录 | AI 对照指南原文，指出与旧 spec 中最终展示时长及先前资源限制表述的差异；未代替成员确认课程后续通知，人工复核待完成 |
| 2026-10-18 | C（姓名待填写） | OpenAI Codex | 按用户要求汇总本对话的项目目标、数据接口、C pipeline、滞回设计、待确认事项、正式课程要求及 Git 权限结果 | `docs/Project_讨论总结.md`、`docs/ai_disclosure.md` | AI 根据本对话与已读取材料整理，检查本地链接、代码围栏与文档差异；人工全文复核待完成，未新增 Python 实现或实验结果 |

| 2026-10-18 | C（姓名待填写） | OpenAI Codex | 按用户批准计划构建六节 proposal 协作框架、闭环架构图、evaluation 决策表、10 页展示填写卡与日期验收清单 | `docs/proposal/`、`docs/ai_disclosure.md` | AI 对照正式指南和接口检查章节、日期、本地链接、围栏与 600 秒预算；人工全文审阅及参数/文献确认待完成；未运行仿真，未生成实验结果或 PPTX |

| 2026-10-18 | C（姓名待填写） | OpenAI Codex | 梳理 C 的输入、循环、输出和证据关系；核对 SimLog 与 evaluate 实际字段；检索 MathWorks AEB/Relay 和 Box2D 碰撞文档作为 Related work 入门资料 | `src/pbs/types.py`、`src/pbs/eval/metrics.py`（只读）、对话说明、本记录 | AI 核对代码和官方检索结果；参考资料需成员阅读、判断适用范围，示例日志非项目实测结果，未修改 Python |

## 记录说明与后续维护

- 本次按上表“谁、工具、用途、涉及文件、检查方式”记录对话中的实质性 AI 辅助，不是逐字聊天转录。前序讨论按本次补记日期登记，不把补记日期当成每条消息的发生时间。
- 成员姓名未由用户确认，暂写 C；不将历史行中的 A1m 自动视为当前用户。
- 保留既有记录原文；其中历史测试通过的表述属于原记录，本次未重新验证。
- 后续在 gabey 中生成或修改 Python、测试、文档、实验解释时，应追加对应 AI 使用记录，注明实际文件和验证方式；只有取得人工核查或测试证据后，才能更新为“已检查/已通过”。
- 用户已选择滞回策略；预历史、无观测失败机制、步内检查细节、终点日志等仍属于待确认设计，不写成全组已批准或已实现。
- 不记录账号密钥、令牌等无关敏感内容。课程完整 AI 政策与具体声明要求仍需由成员核对，本文不声称已代替课程审查。

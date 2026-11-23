# 期中 proposal 指导（严格按 course guidelines 逐条）

> **一句话**：期中只交两样东西 —— **10/27 课上讲 10 分钟**（15%）和 **11/01 交 4–6 页文档**（15%）。**不需要任何实验结果。** 代码、数据、实验都是 11 月以后的事。
> 依据：[project_guidelines.md](../project_guidelines.md)。文档写作骨架见 [proposal.md](proposal.md)。

---

## 0. 先纠正两个误会

| 误会 | 事实 |
| --- | --- |
| “那条路断了” | 那指 12 月之后一个**可选**想法（用真实换道数据校准合成场景的参数）。期中 proposal 第 4 节写的是**将来时**（"Our final evidence **will be** …"），不需要现在有结果，所以那条路通不通与期中无关。**已作废的说法不再跟进**。 |
| “怎么已经在做数据分析” | 因为你更新了数据，我只做了一件事：确认它**会不会挡住期中**。结论：不挡。相关实测压缩到[附录 A](#附录-a工程执行细节11-月以后用不是期中要求)，期中阶段可以不看。 |

**期中阶段唯一要判断的是：这 4–6 页 + 10 分钟，能不能让老师相信"这件事做得出来、值得做、你们知道风险在哪"。**

---

## 1. 老师到底要什么（guideline 原文 → 我们要交什么）

### 1.1 两个交付物

| # | 交付物 | 截止 | 权重 | 老师原话里的硬要求 |
| --- | --- | --- | --- | --- |
| DL1 | **Midterm proposal presentation**（课堂） | **2026-10-27** | **15%** | 约 **10 分钟**讲 + **5 分钟** Q&A；`Each group must state its intended final output and its main data/tool/schedule risks.` |
| DL2 | **Project proposal 文档** | **2026-11-01 23:59:59** | **15%** | **4–6 页**（约每人一页），**6 个章节**；`Feedback received should be incorporated into the final submitted proposal document.` |

### 1.2 文档的 6 个章节 = 老师给的 6 个问题

| 节 | 老师原文问的问题 | 我们要给的东西 | 负责人 | 页数 |
| --- | --- | --- | --- | --- |
| 1. Introduction | `What topic have you chosen and why? Which part(s) of the autonomous-driving pipeline does it relate to?` | 选题动机 + 管线定位（我们做 prediction + decision） | C | 0.6 |
| 2. Related work | `Where do your ideas come from, and where will you obtain additional resources (references, libraries, datasets, pre-trained models, etc.)?` | 想法来源 + 资源清单（要具体到库名/数据集名/论文） | A | 0.8 |
| 3. Our work | `What will you implement in detail, e.g., the overall architecture and individual components…? What are the highlights that make your project distinctive?` 并且 `Define a minimum viable version, an optional extension, and a fallback plan.` | 架构图 + 三层 baseline + **MVP / extension / fallback 三件套** + highlights | C + P | 1.6 |
| 4. Evidence and evaluation | `What will your final demo or analysis show? Complete the sentence: "Our final evidence will be …", and state what that evidence will and will not demonstrate.` | **那一整句 + will / will not 两栏** | D | 0.8 |
| 5. Schedule | `What are the project milestones (at least bi-weekly) and their respective completion dates?` | 双周里程碑表（具体到日期） | C | 0.5 |
| 6. Team | `Who are the team members, and how will all the work be distributed among them?` | 4 人分工 + 权重（**之和 = 4**） | C | 0.5 |

> 4 人 → 正文约 4 页 + 2 张图 ≈ 5 页，正好落在 4–6 页区间。**“每人一页”是老师的暗示：每个人要有自己可指认的一节。**

### 1.3 三条会直接扣分的规则

| 规则 | 原文 | 对我们的做法 |
| --- | --- | --- |
| 7 | `Honest claims matched to evidence are valued more than broad but unsupported claims.` | 第 4 节必须写 **will not demonstrate**；不要写“首次”“novel”“提升安全性” |
| 8 | 工作量明显低于预期 → 扣分 | 4 人就要有 4 份可指认产出（第 6 节写死） |
| 9 | 个人分 = 项目分 × 个人权重（之和 = 人数） | 第 6 节现在就把 4×1.0 写下来 |

### 1.4 关于 AI 声明（别写错）

`AI-tool usage statement` 是**最终 report 第 5.6 节**的要求，**proposal 的 6 个章节里没有这一项**。所以期中不必在正文加 AI 声明；但请继续维护内部记录 [ai_disclosure.md](ai_disclosure.md)，因为 12 月的 report 需要它，而且老师随时可以要求你解释任何一段代码或文字。

---

## 2. 10 分钟汇报：讲什么（这是 DL1 的全部）

老师的要求是“presenting the proposal below”+“state intended final output + main data/tool/schedule risks”。所以 **10 分钟就是把 6 节压缩讲一遍**，不需要 demo，也不需要跑代码。

| # | 时间 | 页 | 一句话 | 谁 |
| --- | --- | --- | --- | --- |
| 1 | 0:00–0:30 | 标题 | 我们要回答：**prediction 到底给 AEB 多争取了几秒，又在哪失效？** | C |
| 2 | 0:30–1:45 | 动机 | reactive AEB 只盯“已在本车道”的目标，切入车要等切完才被看见——这中间是盲区 | P |
| 3 | 1:45–2:30 | 管线定位 | 我们做 prediction + decision 的闭环仿真，不做 perception / 控制 / 动力学 | C |
| 4 | 2:30–4:00 | **三个定义** | ①Reactive（只看本车道+纵向 TTC）vs CV（全目标 2D 外推）；②误刹用 counterfactual 定义；③lead time = 提前察觉切入的秒数 | P |
| 5 | 4:00–5:30 | 我们会做什么 | M1→M4 架构、三层 baseline（Reactive/Predictive/Oracle）、实验清单 | C |
| 6 | 5:30–6:30 | **最终产出** | 一张四边界叠加图 + trade-off 曲线 + lead time；**老师明确要求这一页** | D |
| 7 | 6:30–7:30 | 证据与边界 | “Our final evidence will be …” + 它**不能**证明什么 | D |
| 8 | 7:30–8:45 | **风险** | data / tool / schedule 三类风险 + 各自的应对 + 触发条件（**老师明确要求**） | C |
| 9 | 8:45–9:30 | 退路 | L0→L3 的分层交付：做不完砍到哪一层、结论仍成立 | C |
| 10 | 9:30–10:00 | 团队 | 4 人分工、双周里程碑、权重 | C |

**纪律**：10 分钟 = 10 页以内、每页 ≤ 1 个论点；字体 ≥ 20 pt；不要现场跑代码；带 PDF 备份。

### 5 分钟 Q&A 的预备答案

| 会问 | 答 |
| --- | --- |
| 场景是你们自己生成的，结论是不是自证？ | 是受控实验，不是数据拟合。我们不声称复现真实交通分布，只声称“在给定运动学参数范围内 prediction 改变了什么”；限制会写进第 4 节。 |
| 阈值怎么选？四个系统用一个阈值公平吗？ | 不公平，所以主图是 `threshold × 系统` 的 trade-off 曲线，并在**匹配误报率**下比碰撞率；oracle 提供上界。 |
| 收益是预测带来的，还是仅仅因为 CV 看了更多车？ | 我们设计了消融阶梯：Reactive → Reactive-all（去掉车道过滤）→ CV → Oracle，把“过滤器的贡献”和“外推的贡献”分开读（见第 3 节 highlights）。**这是最可能被问的一题，答好了是加分项。** |
| `r_safe` 怎么定的？ | 由车体包络 + 安全余量推导，并写清是“冲突阈值”还是“物理碰撞”；两者不混用。 |
| 四个人工作量够吗？ | 4 个模块 + 5 个实验 + 接口文档 + 分层退路，每人有独立可指认交付（第 6 节）。 |
| 如果做不完怎么办？ | 第 9 页的 L0–L3：最差情况下也有“同场景 Reactive vs CV 的刹车提前量”这一个结论。 |

---

## 3. 时间表（10/20 → 11/01，只排“写和讲”）

> 没有代码任务。真要写代码，是 11 月 2 日以后的事。

| 日期 | 文档（DL2） | 汇报（DL1） |
| --- | --- | --- |
| **10/20（今天）** | 分节认领：C 写 §1+§5+§6，A 写 §2，P 写 §3.1–3.3，D 写 §3.4–3.6 + §4 | 定讲者与页序（建议 4 人都出镜） |
| 10/21–10/22 | 各自写自己那节（直接改 [proposal.md](proposal.md) 里对应 TODO） | 每人交 2–3 张 slide 的草稿 |
| 10/23 | **合稿**：统一术语（TTC / lead time / benign / false brake / safe area） | C 统一排版 |
| 10/24 | 补图：架构图 1 张（必须有）、别用假数据图 | slide 定稿 |
| 10/25 | 自检 6 节是否都被回答（用 §1.2 的表逐行打勾） | 计时讲第 1 遍 |
| 10/26 | 通读、页数核对 | 计时讲第 2 遍，卡 9:30，用 Q&A 表互相拷问 |
| **10/27** | — | **课上汇报，逐条记录老师反馈** |
| 10/28–10/30 | 按反馈改（做一张 `反馈 → 改了哪节` 的表） | — |
| 10/31 | 终稿：页数 4–6、引用逐条核实 | — |
| **11/01** | **由 representative 提交一份（PDF）** | — |

---

## 4. 对照评分档位：期中我们要拿哪一档

| 档位 | 老师定义 | 期中对应 |
| --- | --- | --- |
| 21–40% | 文档可读但有缺项 | 6 节缺一两节、或第 3 节没有 fallback |
| **41–60%** | 文档信息完整 | 6 节齐全但写得很薄 |
| **61–80%（本期目标）** | 文档完整详细 + 结果与 proposal 一致 | 6 节全填、有架构图、里程碑具体到日期、evidence 有 will/will not、风险有触发条件 |
| 81–100% | 有洞见 + 有创意 | 期中能体现洞见的就是**第 3 节那句“把收益拆成过滤 vs 外推”**，写清楚就够 |

**最容易丢分的三个地方**（都在第 3、4 节）：
1. **没写 fallback**（老师第 3 节明确要求三件套，很多人只写 MVP）；
2. **`Our final evidence will be …` 没有写成完整的一句**；
3. **没写 "will not demonstrate"**（规则 7 直接对着这个）。

---

## 5. 提交注意事项

- 只由 **representative** 提交一份，重复提交会扣分（guidelines 明写）；
- PDF；页数按**排版后**数，不靠估算；
- 文件名含组号/队名；
- 反馈落地表附在文末（DL2 明确要求“incorporate feedback”，附上就是证据）；
- 参考文献**逐条人工核实**后再引（[proposal.md](proposal.md) 文末有候选清单，带勾选框）。

---

## 附录 A：工程执行细节（11 月以后用，**不是期中要求**）

> 以下内容与 DL1/DL2 无关，期中阶段可以完全不看。保留在此，是因为 11 月 2 日开工时可以直接拿来当清单。

### A.1 代码现状（截至 10-20）

11 个函数仍是 `NotImplementedError`（`quintic_lateral`、`make_scenario`、`load_toy`、5 个 predictor、`assess_risk`、`ReactiveAssessor`、`run_matrix/tradeoff/failure_boundary/safe_area`），5 个 `exp*.py` 的 `main()` 为空；`systems.py` 里 6 个系统没有一个能跑。能跑的是 `types`、`run_sim` 骨架、`ego_rollout`、`in_collision`、两个 policy、`is_benign`、`evaluate`、`ade/fde`、`plot_timeline`，以及 6 个测试。环境也未装（无 pandas/matplotlib/pytest，`pbs` 未 `pip install -e .`）。

**11 月开工顺序**：环境 → `load_toy` → `quintic_lateral` + `make_scenario("cut_in")` → `assess_risk` + `CV` + `ReactiveAssessor` → `exp05_cut_in_lead_time`。这一条链跑通就够第 3 节写的 MVP。

### A.2 数据（结论只有一句：不挡期中）

课程样例已到位（3 个 CSV = 1 个场景，ego + 2 agents，12.9 s @ 10 Hz），格式与 `load_toy` 契约一致。三点备忘：文件需归位到 `data/sample_trajectories/`（或改 `DATA_DIR`）；`load_toy` 要排除 `agent_type=="ego"`；ego 的速度列有噪声，预测器用窗口最小二乘而非两点差分。完整实测与它对“能/不能支撑什么”的判断见 [data_audit.md](data_audit.md)。

### A.3 骨架里已知的 3 个坑（11 月修，不影响期中）

1. **brake chatter**：实测 16 次刹车/巡航切换（单步点刹），因为 `ego_future` 按恒速 rollout 而 ego 在减速 → 需要滞回/锁存，否则 `brake_onset_t` 与 lead time 全是噪声。
2. **`r_safe = 2.5` 语义不清**：注释说是“半长+半宽+余量”，代码当两车圆心距阈值用，两者差约 2 倍，会同时污染 collision、`is_benign` 和边界位置。
3. **`history` 可能只有 1 个采样点**（latency>0 时），predictor 必须处理，否则 latency 实验的结论会被代码 bug 污染。

### A.4 第 3 节 highlights 里“把收益拆成两块”的写法

| 系统 | 关注对象 | 运动模型 | 增量含义 |
| --- | --- | --- | --- |
| `reactive` | 仅本车道 | 1D 视距（range/range-rate） | 基线 |
| `reactive_all` | 全目标 | 1D 视距 | **Δfilter**：只看“是否过滤”的贡献 |
| `cv` / `ca` / `ctrv` | 全目标 | 2D 外推 | **Δpred**：外推的贡献 |
| `oracle` | 全目标 | 真值 | 上界；`oracle − cv` = 预测误差的代价 |

只多一个约 5 行的 assessor 变体，却能把“我们比了四个系统”变成“我们解释收益从哪来”。**这是第 3 节唯一值得写进 highlights 的洞见。**

### A.5 11 月的实验纪律（现在知道就好，不用做）

- **相图 ≠ 率**：确定性网格叫 phase diagram（讲“边界在哪”），要讲“多常发生”必须随机采样 30–50 次并给均值±CI；
- **阈值公平性**：固定同一阈值不公平，主结论用“匹配误报率下的碰撞率”；
- **自检**：oracle 边界必须落在解析刹停距离 `v·t_react + v²/(2a)` 附近，否则说明仿真有问题。

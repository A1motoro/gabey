# Project Proposal — Predictive Braking Simulator

<!-- =====================================================================
内部使用说明（提交前删除本块）
- 期中交的是**计划书**，不是结果：全文用将来时（"will"），不需要任何实验数据。
  第 4 节的固定句式 "Our final evidence will be …" 本身就是将来时。
- 本文件是 proposal 的**骨架**：英文正文可直接沿用，标 TODO 的地方必须由对应同学填。
- 课程为英文授课，正文建议英文；内部讨论可中文。
- 页数预算（总计 4–6 页，4 人 ≈ 每人 1 页）：
    §1 Introduction        0.6 页
    §2 Related work        0.8 页（含资源来源表）
    §3 Our work            1.6 页（核心，含架构图 + MVP/extension/fallback 表）
    §4 Evidence            0.8 页（必须有 "Our final evidence will be …" 整句 + will/will not 两栏）
    §5 Schedule            0.5 页
    §6 Team                0.5 页
    References             0.3 页
  合计约 5.1 页 + 1 张架构图。
- 分节认领与时间表见 [midterm_plan.md](midterm_plan.md) §1.2 与 §3。
- 参考文献必须**逐条人工核实**后再引用（见文末候选清单，带 [ ] 勾选框）。
====================================================================== -->

**AIE1903 Exploration on Basic Autonomous Driving — Course Project Proposal**
**Team _<Team name/number>_ — 2026–2027 Fall**

> **Research question:** How much does trajectory prediction expand the safe operating area of an automatic emergency braking (AEB) system, and where does it still fail?

---

## 1. Introduction

<!-- TODO(C): 0.7 页。第一段讲"选题 + 为什么"，第二段讲"在 AD 管线的哪个位置"，最后 3 条 bullet 列 contributions。 -->

Automatic emergency braking (AEB) is among the most widely deployed functions in modern vehicles, yet its decision logic is surprisingly simple: most production systems estimate the risk of a target from longitudinal range and range-rate in the ego lane only, and brake when the resulting time-to-collision (TTC) falls below a threshold. This design is structurally blind to **cut-in** manoeuvres: a vehicle that is changing into the ego lane is not "in" the ego lane until the manoeuvre is essentially complete, so a reactive system can only react after the conflict already exists.

Trajectory prediction offers an obvious remedy — extrapolate the lateral motion of neighbouring vehicles and anticipate the conflict **before** the cut-in finishes. But prediction is never exact, and the resulting error can brake too late (safety) or too early (false braking). The literature reports prediction accuracy (ADE/FDE) and AEB performance separately; what is missing is a *controlled, closed-loop* study that asks **what the accuracy is actually worth**.

Our project sits at the interface of two stages of the autonomous-driving pipeline: **prediction** (motion forecasting of surrounding agents) and **decision making** (braking policy). We deliberately do not model perception or lateral control, so that the only factor we vary is the prediction model.

**Our research question and contributions:**

1. We build a small, fully reproducible closed-loop 2D simulator in which AEB systems are compared at three tiers — **Reactive** (no prediction), **Predictive** (kinematic predictors), and **Oracle** (true future, an upper bound) — over deterministic parameter grids and randomised scenario draws.
2. We evaluate each system not only by collision rate but by **counterfactual false-brake rate** and by the **area of its safe region** in the (initial gap, relative speed) plane, so that "how much prediction helps" becomes a measured number rather than a claim.
3. We **decompose** the benefit of prediction into a *lane-filter* component and a *2D-extrapolation* component, using an ablation ladder (§3.2). This answers the obvious objection that a predictive system merely "looks at more vehicles" rather than predicting better.

---

## 2. Related work

<!-- TODO(A): 0.8 页。4 段 + 1 张资源表，正文约 520 词。
     引用编号 [1]–[10] 对应 [section2_zh.md](section2_zh.md) §A；网址与核实状态见
     [references_candidates.md](references_candidates.md)。**References 一节尚未加入**（等核实结果）。
     交叉引用按 P 的 §3 结构：消融 → §3.5，counterfactual → §4；若结构再变，同步 P4。
     超页时先删 P4 第 2 句（"Forecasting benchmarks report ADE/FDE…"）与表最后一行。 -->

**Risk assessment and AEB.** Forward collision warning has standardised performance requirements [1]. Automatic emergency braking is usually evaluated with scenario-based test protocols [2]. In both, the decision variable is a longitudinal time-to-collision (TTC). It is computed from range and range-rate under a constant-velocity assumption. A more formal line of work replaces the threshold with a provably safe following distance, as in Responsibility-Sensitive Safety (RSS) [3].

We borrow three things from this work. First, we use the TTC rule as the baseline controller. Second, we use the last-resort stopping distance `d = v·t_react + v²/(2a)` as an analytic reference. It tells us when a simulated failure boundary is implausible. Third, we describe operating limits as a region, not as a single number. We keep the controller at one TTC threshold on purpose. Any difference we measure between systems then comes from how a conflict is perceived, not from a smarter decision rule.

**Motion prediction.** Short-horizon prediction is usually split into two families: kinematic models and learning-based models [4]. The kinematic models are constant velocity (CV), constant acceleration (CA), and constant turn-rate-and-velocity (CTRV) [5]. They are the interpretable baseline in most tracking and prediction stacks. Learning-based models, such as Social LSTM [6] and Trajectron++ [7], reduce displacement error a lot. In return they need training data, training time, and integration work in a closed loop.

Our predictive tier uses kinematic models only. This is a scoping decision, not a claim about the state of the art. A learned predictor would beat constant velocity. In our comparison it would land somewhere between CV and the oracle. That is why we keep an **oracle** tier. The oracle bounds what any predictor could contribute. The gap between oracle and CV then prices prediction error in the currency we care about: collision outcomes, not ADE. For scenario generation we use a minimum-jerk (quintic) lateral profile [8]. We bound its peak lateral acceleration, so every generated cut-in stays dynamically feasible.

**Data and benchmarking.** Public datasets give real trajectories with map information, for example nuScenes [9] and highD [10]. We use them for context and for kinematic reference values. They are not our benchmark. Prediction error is measured on analytic fixtures with exactly known ground truth. One fixture is a straight line, which checks correctness. The other is a minimum-jerk cut-in, where constant velocity is wrong by construction. ADE and FDE therefore do not depend on the simulator. The course-provided clip validates our loader. As an extension it also supports one real longitudinal-conflict case study. That clip contains a conflict. It contains no lane change into the ego lane. Our **cut-in benchmark is therefore synthetic**. We justify its kinematics with the bounds above and with a sensitivity analysis over the lateral-acceleration cap. We state this as a limitation in §4 rather than hide it. For evaluation we report **phase diagrams**: the collision / no-collision frontier over (initial gap, relative speed). One aggregate score would hide the shape of that frontier. The shape is what we study.

**Positioning.** Forecasting benchmarks report ADE/FDE. AEB studies report collisions and false braking [2,4]. These two lines are usually evaluated apart. We connect them in one closed loop that runs on an ordinary laptop. The predictor is interchangeable behind a single interface. The same scenario can be replayed with a lane-gated reactive baseline or with a predictive system. Beyond that direct comparison, an extension separates two contributions: lane-gating and 2D extrapolation (§3.5). "Prediction helps" then becomes an attributed claim, not only an observed one. On the cost side we count false braking counterfactually (§4). Benefit and cost are then measured on the same scenarios.

**Table 1.** External resources and what we take from each. No pre-trained model is used.

| Resource | What we borrow |
| --- | --- |
| Standards and test protocols: ISO 15623 [1]; Euro NCAP AEB tests [2] | The TTC decision rule; the vocabulary of test scenarios |
| Responsibility-Sensitive Safety [3] | The idea of a provably safe following distance; used only to check boundaries |
| Survey of motion models [4] | How we group predictors, and why we stop at kinematic models |
| CTRV model [5] | The turn-rate predictor |
| Social LSTM [6]; Trajectron++ [7] | Evidence that stronger predictors exist, hence the need for an oracle bound |
| Minimum-jerk profile [8] | The lane-change trajectory generator |
| nuScenes [9]; highD [10] | Dataset context and kinematic reference ranges; **no data is taken** |
| Waymo Open Motion (Ettinger et al., 2021; [arXiv:2104.10133](https://arxiv.org/abs/2104.10133)) | November L3 update: real 10 Hz validation tracks, official map/SDC metadata; selected-target replay, distinct from the synthetic benchmark |
| Course-provided clip (1 ego + 2 agents, 12.9 s @ 10 Hz) | Loader validation; extension case study of one real conflict |
| NumPy, pandas, matplotlib, pytest | Implementation, experiment matrix, figures, tests |

---

## 3. Our work

<!-- TODO(C+P): 1.6 页。这一节决定分数。3.2 的消融阶梯和 3.3 的定义是 highlight；3.5 是规则强制要求的 fallback。 -->

### 3.1 Architecture

A scenario is a set of parameterised trajectories (`Scenario`); at every simulation step `dt = 0.1 s` the simulator (i) rolls out the ego under "keep current speed", (ii) hands each neighbour's *observable* history (with optional perception latency `τ`) to a risk assessor, (iii) takes the minimum TTC over agents, (iv) applies the braking policy, and (v) integrates the ego forward. The ego is simulated (so it responds to its own braking) while neighbours are replayed from their reference trajectories (so they do not react). Assessment is **receding-horizon**: nothing is cached between steps.

```
M1 scenario  ->  M2 prediction  ->  risk (TTC)  ->  M3 closed-loop sim  ->  M4 evaluation
synthetic         CV / CA /        assess_risk     ego rollout, brake      metrics, phase
cut-in,           CTRV / LR /      per agent,      policy, collision,      diagrams, area
lead-brake,       Oracle          min over agents latency τ               decomposition
crossing
```

Interfaces between the data, prediction, risk, simulation and evaluation modules are frozen and documented (`docs/interfaces.md`); a walking skeleton already runs end-to-end and is covered by smoke tests, so the remaining work is confined to well-defined functions with a single owner each.

### 3.2 Three tiers and the ablation ladder

| Tier | System | Which agents | Motion model | What it isolates |
| --- | --- | --- | --- | --- |
| Baseline | `reactive` | **ego lane only** | 1D line-of-sight (range / range-rate) | the deployed design |
| Ablation | `reactive_all` | all agents | 1D line-of-sight | **Δ filter**: value of *looking* at the right agents |
| Predictive | `cv` / `ca` / `ctrv` / `lr` | all agents | 2D extrapolation | **Δ pred**: value of *extrapolating* them |
| Upper bound | `oracle` | all agents | ground-truth future | ceiling; `oracle − cv` = cost of prediction error |

The ablation row is deliberately cheap to implement (one flag in the assessor) and is what turns "we compared four systems" into "we explain where the improvement comes from". If Δfilter dominates Δpred, that is a legitimate — and reportable — finding.

### 3.3 Definitions that make the comparison meaningful

- **Reactive ≠ CV.** A reactive AEB inspects only agents *already inside* the ego lane and computes TTC from longitudinal range and range-rate, ignoring lateral motion. A predictive assessor extrapolates **all** agents in 2D and therefore sees a cutter while it is still mid-lane-change. The quantity of interest is the resulting **lead time** — how many seconds earlier the brake is triggered.
- **Counterfactual false braking.** A scenario is *benign* if the ego, with brakes disabled, would not collide. Any brake in a benign scenario is a false brake. The scenario set therefore contains deliberately dangerous-looking but safe cut-ins (large gap, non-closing relative speed); otherwise the false-brake rate is identically zero and the trade-off curve cannot be drawn.
- **Safe area.** For a fixed braking policy, the (initial gap `gap_d`, relative speed `rel_v`) plane is divided into "collision" and "collision-free" cells; the collision-free fraction is the system's safe area. `area(cv) − area(reactive)` is the value of prediction; `area(oracle) − area(cv)` is the price of prediction error.

### 3.4 Scenario design

Cut-ins follow a **quintic (minimum-jerk) lateral profile** `y(τ) = y0 + Δy·(10τ³ − 15τ⁴ + 6τ⁵)`, whose peak lateral acceleration is `(10/√3)·|Δy|/T²`. Parameter combinations exceeding **3.0 m/s²** are rejected, so every scenario is dynamically feasible for a passenger vehicle; the rejected fraction is reported, and the cap is varied (2.0 / 3.0 / 4.5 m/s²) as a sensitivity analysis. Three families are generated — `cut_in`, `lead_brake`, `crossing` — each with a benign and a dangerous region, and each parameterised by exactly the quantities that appear on the axes of the result figures.

| Parameter | Range (to be frozen in W8) | Appears in |
| --- | --- | --- |
| `gap_d` initial longitudinal gap | 8–40 m | phase diagram x-axis |
| `rel_v` ego minus cutter speed | −5 … +10 m/s | phase diagram y-axis |
| `cut_in_t` start of lane change | 0.5–3.0 s | onset timing |
| `lc_T` lane-change duration | 2.0–4.0 s | peak lateral acceleration filter |

### 3.5 Minimum viable version, extension, and fallback

| Level | Content | Decision trigger |
| --- | --- | --- |
| **MV (minimum viable)** | `cut_in` + `lead_brake`; `reactive` vs `cv`; ADE/FDE on real course trajectories; the **same-scenario timeline** with brake onset; lead-time table | If not achieved by **2026-11-08**, the final deliverable is reported at this level |
| **Extension** | + `ca` / `ctrv` / `oracle`; counterfactual false-brake trade-off curve; the four-system boundary overlay; the Δfilter/Δpred ablation | Target: **2026-11-22** |
| **Optional (L3)** | regression predictor; perception-latency sweep `τ ∈ [0, 300] ms`; Waymo Open Motion replay cross-check (replaces nuScenes-mini, November update) | Scope freeze **2026-11-29**; A's data/replay evidence completed 2026-11-23, see `L3_waymo_case_study.md` |
| **Fallback** | If the simulator itself proves unstable (e.g. policy oscillation, invalid collision model), we fall back to an **open-loop** study: prediction error and TTC lead time on logged trajectories, with the closed-loop boundary reported as a single hand-selected case study | Invoked at the 2026-11-08 checkpoint at the latest |

### 3.6 Highlights

1. **Decomposition, not just ranking** — the ablation ladder separates "looking at more agents" from "predicting better" (§3.2), which a naive three-system comparison cannot do.
2. **Counterfactual safety accounting** — false braking is defined against a brake-free rollout of the same scenario, making cost and benefit directly comparable in one plane.
3. **Phase diagrams instead of a single number** — results are reported over the full parameter grid, so the *boundary* and its shape (rather than one aggregate score) is the object of study.
4. **Analytic cross-checks** — every simulated boundary is compared with the last-resort stopping distance `v·t_react + v²/(2a)`; the oracle boundary must lie close to it, otherwise the simulator is wrong.

---

## 4. Evidence and evaluation

<!-- TODO(D): 0.8 页。核心是那一整句 "Our final evidence will be …" 以及"不能证明什么"。老师会逐字看这一段。 -->

**Metrics.** ADE / FDE of predicted futures against ground truth; collision rate; counterfactual false-brake rate; minimum centre distance; brake-onset time; cut-in lead time (reactive onset − predictive onset); safe area and its decomposition.

> **Our final evidence will be** a set of closed-loop experiments on parameterised cut-in, lead-braking and crossing scenarios, in which (i) prediction quality is quantified as ADE/FDE versus prediction horizon, (ii) the same scenarios are replayed with three tiers of AEB system, and (iii) the resulting difference is reported as a *quantified* change of the safe region in the (gap, relative speed) plane, together with the brake-timing lead time on individual scenarios and the false-brake cost measured counterfactually.

**What this evidence will demonstrate.**

- That, in a controlled and reproducible closed-loop setting, adding 2D prediction shifts the collision boundary of an AEB system by a measurable amount, and by how many seconds earlier the same cut-in is detected.
- That the choice of motion model matters **only for the conflict type it can represent**: on the real longitudinal conflict clip, a constant-velocity reactive TTC reports 13.9 s of clearance at `t = 0` while geometric contact occurs at 10.5 s — a case in which acceleration-aware prediction is decisive and lateral prediction is worth nothing.
- How much of that shift is attributable to *which* agents are considered versus *how* their motion is extrapolated.
- The price of prediction error: the gap between the predictive and oracle boundaries bounds how much a better predictor could possibly be worth.

**What this evidence will not demonstrate.**

- It will **not** establish real-world safety or accident-reduction rates; scenarios are synthetic, sampled from a parameter range, not from a traffic distribution.
- It will **not** be calibrated against real cut-in statistics: the cut-in family is synthetic, its plausibility is argued from literature bounds plus a sensitivity analysis, and the one course-provided clip contains no lane change into the ego lane.
- It will **not** show that any specific production AEB is unsafe, nor compare against real deployed systems.
- It will **not** cover perception, sensor noise (beyond an optional latency term), vehicle dynamics, lateral control, or non-flat road geometry.
- It will **not** claim state-of-the-art prediction accuracy: kinematic models are used as interpretable baselines, and a learned predictor would only move the result toward the oracle bound.

**Self-checks that must pass before any claim is made.** (a) the oracle boundary lies within a stated tolerance of the analytic last-resort stopping distance; (b) the "reactive sees the cutter later than CV" property holds on every recorded cut-in (unit-tested); (c) repeated runs are bit-identical when random seeds are fixed; (d) a brake-latch guarantees no brake/cruise oscillation (unit-tested), so onset times are meaningful.

---

## 5. Schedule

<!-- TODO(C): 0.5 页。规则要求"至少双周"里程碑。日期要具体；DL 日期来自 guidelines。 -->

| Date | Milestone | Owner(s) | Deliverable |
| --- | --- | --- | --- |
| **2026-10-27** | Midterm proposal presentation (DL1) | all (C presents) | slides + a first lead-time number on one cut-in |
| **2026-11-01** | Proposal document (DL2) | all | this document, feedback incorporated |
| 2026-11-08 | **MV complete**: cut_in + lead_brake, reactive vs cv, ADE on real trajectories, real-clip case study (`exp06`) | A, P, C, D | 2 figures + 1 table, one-command reproducible |
| 2026-11-15 | `ca`, `ctrv`, `oracle`; counterfactual false-brake accounting | P, D | trade-off curve per system |
| 2026-11-22 | Boundary overlay + Δfilter/Δpred ablation | D, P | headline figure + staircase figure |
| 2026-11-29 | Optional L3 (latency, regression, dataset cross-check) or **scope freeze** | C | frozen numbers; code freeze announced |
| 2026-12-06 | Final report draft + demo rehearsal | all | 8–12 page draft, recorded backup demo |
| **2026-12-08** | Final presentation and demo (DL3) | all | 10 min + 10 min Q&A |
| **2026-12-13** | Code package + final report (DL4) | all | tagged repo, README, environment file, report |

---

## 6. Team

<!-- TODO(C): 0.5 页。填真实姓名；权重之和必须 = 4.0（规则 9）。先写 1.0，W11 互评后微调。 -->

| Code | Member | Role | Modules / deliverables | Weight |
| --- | --- | --- | --- | --- |
| C | _TBD_ | Coordinator; simulation & decision | `pbs.sim` (rollout, footprint, brake policy, latency), integration, interface document | 1.0 |
| P | _TBD_ | Prediction & risk assessment | `pbs.prediction` (CV/CA/CTRV/LR/Oracle), `pbs.risk` (TTC, reactive vs predictive assessors) | 1.0 |
| A | _TBD_ | Scenario design & related work | `pbs.data` (quintic cut-in generator, scenario families, parameter bounds and sensitivity analysis); §2 | 1.0 |
| D | _TBD_ | Evaluation & experiments | `pbs.eval` (counterfactual, matrix, trade-off, boundaries, figures), demo | 1.0 |
| | | | **Total** | **4.0** |

**Collaboration rules.** Trunk-based development with one small pull request per deliverable; cross-review is rotated (D reviews C, P reviews A, C reviews P, A reviews D). Every week each member records delivered artefacts (PR / file / figure) in `docs/contributions.md`; individual weights are revisited once at the end of W11 using those records plus a single peer-review round, and are finalised before the final report.

<!-- 本版不使用参考文献表（作者决定）。若之后需要正式引用，候选清单见
     [references_candidates.md](references_candidates.md)，注意每条都需人工核实后再用。 -->

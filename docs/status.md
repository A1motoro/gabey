# 仓库状态更新 — 2026-11-24：PR #16 评审后续修复

**A 的 L3 已从“只有数据”推进到可独立复现的真实数据证据**：Waymo 原始 CRC 读取、
官方 SDC 索引、有效窗口、5+5 首批和 2 个说明案例共 12 个 NPZ；三张主图、BEV、
两组 CSV、说明案例表、CV ADE/FDE、全候选审计、测试与交接说明。

8 分片 / 2321 条记录 → 40 个切入目标窗口、435 个对照窗口；首批 5+5 都 benign，均无刹车。
完整切入审计中 1 dangerous / 39 benign（39 个场景）；真实危险说明案例 CV 2.9 s、Reactive 3.2 s，
均避免碰撞；另有两者均误刹的说明案例。说明案例不进入首批统计。
**已合入最新 main 的 CA/CTRV/LR/Oracle、D 的 L2 评测与 OBB 基线。**
L3 默认实验仍比较 Reactive/CV，并显式使用圆形兼容参数；CA 可选、Oracle 真值未来不足跳过、GRU 未注册。

复现：`python experiments/exp07_real_clip.py`；报告和限制见
[L3_waymo_case_study.md](L3_waymo_case_study.md)，给 P 的输入契约与 A 的答辩说明见
[L3_A_handoff.md](L3_A_handoff.md)。原始分片继续被 ignore；小型 `exp07` 图表已允许纳入代码包。
此前 demo 测试的 Windows 路径、端口预留与子进程编码问题已在测试侧修正。

此前合并 main 到 L3 分支时，本机具备课程 toy CSV，实测 389 passed（38.60 s）。
组员在当时干净克隆上实测 387 passed / 2 failed：两项失败来自缺少外部课程 CSV。
本次将这两项外部数据集成测试改为缺少 CSV 时明确 skip，并新增无需下载数据的 CSV 单元测试。
课程文件存在但损坏时仍然失败，不会把解析错误静默跳过。
验证计数会随 main 更新与可选依赖变化，以本次实际命令输出为准。


## 环境前提与历史归档

`pip install -r requirements.txt`、`pip install -e .` 是基础环境；
`pip install -e ".[womd]"` 另外启用原始 TFRecord/CRC 验证。NPZ 实验不需要这项额外依赖。
无课程 CSV 时只跳过两项外部数据集成测试；自包含 toy loader 单元测试和 L3 快照测试照常运行。
若没有安装可选 google-crc32c，原始读取 CRC 测试也会单独 skip。

原 2026-11-23 21:30 的旧分支快照已移至 [历史归档](archive/status_2026-11-23_2130.md)。
当前状态文件不再混入旧的模块进度或失效排期。

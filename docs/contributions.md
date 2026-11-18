# Contributions

每周更新一次（周日前）。写具体产出（PR / 文件 / 图），不写「参与讨论」。

| 周 | C | P | A | D |
| --- | --- | --- | --- | --- |
| W11（P L3） | | `LinearRegressionPredictor` 窗口位置拟合；`tests/test_l3_predictors.py`；exp01 `--include-lr` 五模型同切片 CSV/图；`docs/l3-validation.md` | | |
| W11（P L2） | | `ConstantAccelPredictor`、`CTRVPredictor`、`OraclePredictor`；`tests/test_l2_predictors.py` 与原跳过用例；`exp01_predictor_ade.py` 同切片误差 CSV/图；`docs/l2-validation.md` 与接口追加约定 | | |
| W4 | | | M1 交付：`pbs/data/{scenarios,loader}.py`（quintic 换道 + `cut_in`/`lead_brake`/`crossing` 三类场景 + 横向加速度过滤 + 96 场景网格，47% benign）；`experiments/exp00_scenarios.py`（3 图 + CSV + 汇总）；`tests/test_data.py`（31 项）；`docs/scenarios.md`、`docs/A_baseline_handoff.md` | |
| W10 | | | | `run_matrix` / trade-off / rescue rate / 四边界图；`exp02`、`exp03`、`exp05` 的 `--quick`（场景用 A 的 `make_scenario` / `scenario_set`） |
| W11 | | | CA 评测底座：`cut_in` 的 `a_cutter` 纵向加速度、`Trajectory.accel` 解析真值、`CA_GRID`/`ca_scenario_set()`（12 场景，6/6）、`prediction_slices()` 固定切片；修 exp00 的 `min_dist`/`collision` 口径（96 场景 52 dangerous / 44 benign，1 个 marginal）；`tests/test_data.py` 40 项；`docs/CA_scenario_support.md` | |

# Reproducing D's results

> Owner: D (yj). How to regenerate the figures, tables and numbers quoted in
> [results.md](results.md). `outputs/figures/` and `outputs/logs/` are not versioned
> (`.gitignore`), so they have to be generated locally.

## Setup

As in the README:

```bash
conda activate aie1903_ad
pip install -r requirements.txt
pip install -e .
pytest
```

`pytest` reports two failures in `tests/test_data.py::test_load_toy_*` when
`data/sample_trajectories/` is absent. They do not involve D's code. On Windows see the
README note on `conda run`.

## Quick mode (what results.md reports)

```bash
python experiments/exp02_threshold_sweep.py --quick
python experiments/exp03_failure_boundary.py --quick
python experiments/exp05_cut_in_lead_time.py --quick
python experiments/check_label_window.py
```

| Script | Figure | Table | Rows | Measured time |
| --- | --- | --- | ---: | ---: |
| `exp02_threshold_sweep.py --quick` | `exp02_tradeoff.png` (F3) | `exp02.csv`, `exp02_summary.json` | 880 = 110 × 4 × 2 | 143 s, 145 s |
| `exp03_failure_boundary.py --quick` | `exp03_boundaries.png` (F4) | `exp03.csv`, `exp03_summary.json` | 200 = 25 × 4 × 2 | 33 s, 36 s |
| `exp05_cut_in_lead_time.py --quick` | `exp05_lead_time.png` (F2) | `exp05.csv`, `exp05_summary.json` | 75 = 25 × 3 | 14 s, 14 s |
| `check_label_window.py` | — | `exp02_label_window.json` | — | 85 s |

The three experiments together took 190 s and 195 s in two runs (Python 3.12.3,
NumPy 2.5.5, Linux VM), under the 5 min bound of M2. `exp05.csv` holds Reactive, CV and
CTRV rows; `n_rows = 50` in its summary counts the Reactive-vs-predictive pairs.
`check_label_window.py` reads `outputs/logs/exp02.csv`, so run it after exp02. It re-runs
the never-brake case over the whole 19 s log and reports which benign labels depend on the
15 s window. It changes no label.

Each script takes `--from-csv <path>` to redraw its figure from a saved table without
simulating. The redrawn PNGs were byte-identical to the originals. Note that this
rewrites the summary JSON with `quick` and `config` set to `null`.

## Checking the numbers

```bash
jq '.rescue.fixed_2s' outputs/logs/exp03_summary.json
# reactive 0.42857…, cv 0.64285…, ca 0.64285…, oracle 0.85714…  (6/14, 9/14, 9/14, 12/14)

jq '.value_of_prediction_fixed_2s, .cost_of_prediction_error_fixed_2s' outputs/logs/exp03_summary.json
# 0.21428… twice: the fixed t_on = 2.0 s panel

jq '.value_of_prediction, .cost_of_prediction_error, .operating_point[].met_budget' outputs/logs/exp03_summary.json
# 0.21428… twice and four `true`: the false-brake ≤ 5 % panel

jq '.median_lead_time_s, .median_lead_time_when_predictive_is_earlier_s' outputs/logs/exp05_summary.json
# cv 0.39999…, ctrv 0.5; then cv 1.0, ctrv 1.2

jq '.exp02.n_late_contact, .boundary_grid' outputs/logs/exp02_label_window.json
# 6; quick grid 0 of 11 benign, full grid 6 of 101 benign
```

The summary values are unrounded floats. `value_of_prediction` (budget panel) and
`value_of_prediction_fixed_2s` (fixed panel) are different keys that coincide in quick
mode; compare a full-resolution run against the key that matches the panel.

## Determinism

The scenarios are fixed parameter grids and the simulation has no random draws.
Running the three quick experiments twice (at `a3f91cb` and at `3d34fe2`) gave CSVs that
are identical except for the `git_rev` column, and identical summaries.

## Not run for this deliverable

- **Full resolution.** Dropping `--quick` gives 110 × 4 × 6 = 2640 rows for exp02,
  255 × 4 × 6 = 6120 for exp03 and 255 × 3 = 765 for exp05 (from the grids and threshold
  lists in `experiments/matrix_setup.py` and `scenario_sets.py`). These were not run and
  no runtime was measured.
- **`--legacy-l1`.** The circle + single-threshold configuration of PR #8 is described in
  [d-baseline.md](d-baseline.md) §9 and was not rerun here.
- **exp04.** `experiments/exp04_latency.py` is a stub.

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
python experiments/check_outcome_window.py
```

| Script | Figure | Table | Rows | Measured time |
| --- | --- | --- | ---: | ---: |
| `exp02_threshold_sweep.py --quick` | `exp02_tradeoff.png` (F3) | `exp02.csv`, `exp02_summary.json` | 880 = 110 × 4 × 2 | 185 s, 186 s |
| `exp03_failure_boundary.py --quick` | `exp03_boundaries.png` (F4) | `exp03.csv`, `exp03_summary.json` | 200 = 25 × 4 × 2 | 43 s, 41 s |
| `exp05_cut_in_lead_time.py --quick` | `exp05_lead_time.png` (F2) | `exp05.csv`, `exp05_summary.json` | 75 = 25 × 3 | 18 s, 18 s |
| `check_outcome_window.py` | — | `exp02_outcome_window.json` | — | 236 s, 243 s |

The three experiments together took 246 s and 245 s in two runs (Python 3.12.3,
NumPy 2.5.5, Linux VM). That is under the 5 min bound of M2, with less margin than the
190 s and 195 s measured when every run was 15 s long: each run is now 19 s and each
scenario that is not dangerous gets a second never-brake run. The bound was measured on
this machine only. `exp05.csv` holds Reactive, CV and CTRV rows; `n_rows = 50` in its
summary counts the Reactive-vs-predictive pairs.

Every run lasts 19 s on 23 s logs. Brakes are scored in the first 15 s. The CSVs carry
the class in `label` (`dangerous`, `late_contact`, `benign`), the never-brake contact time
in `never_brake_contact_t`, and the two windows in `decision_duration` and
`outcome_duration`.

`check_outcome_window.py` reads `outputs/logs/exp02.csv`, so run it after exp02. It
reruns that matrix for 25 s instead of 19 s and lists every label, collision flag and
in-window brake that differs. The saved CSV is only read; the differences go to
`exp02_outcome_window.json`. For the boundary grid:

```bash
python experiments/check_outcome_window.py --scenarios boundary-quick --csv outputs/logs/exp03.csv
python experiments/check_outcome_window.py --scenarios boundary-quick --csv outputs/logs/exp05.csv
```

These took 57 s and 23 s.

Each script takes `--from-csv <path>` to redraw its figure from a saved table without
simulating. The redrawn PNGs were byte-identical to the originals. Note that this
rewrites the summary JSON with `quick` and `config` set to `null`. Tables saved before the
`label` column existed are read as two classes.

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

jq '.n_dangerous_scenarios, .n_late_contact_scenarios, .n_benign_scenarios, .oracle_false_brake_free' outputs/logs/exp02_summary.json
# 57, 6, 47, true

jq '.late_contact[] | select(.threshold == 2)' outputs/logs/exp02_summary.json
# each system: n_late_contact 6, braked_in_window 3, braked_later 3, collided 0

jq '.cells' outputs/logs/exp03_summary.json
# dangerous 14, late_contact 0, benign 11

jq '.label_changes, .collision_changes, .decision_changes, .false_brakes_on_changed_labels' outputs/logs/exp02_outcome_window.json
# one label change (lead_brake init_gap 80, decel 0.3: benign -> contact at 23.36 s), [], 0, 0
```

The summary values are unrounded floats. `value_of_prediction` (budget panel) and
`value_of_prediction_fixed_2s` (fixed panel) are different keys that coincide in quick
mode; compare a full-resolution run against the key that matches the panel.

## Determinism

The scenarios are fixed parameter grids and the simulation has no random draws.
Running the three quick experiments twice, before and after the code was committed
(`63ea148`), gave CSVs that are identical except for the `git_rev` column, identical
summaries and byte-identical figures.

## Against the one-window tables

The saved one-window CSVs (15 s runs, `3d34fe2`) were compared row by row with the new
ones. In all 880 + 200 + 75 rows the brake onsets and contact times inside the first
15 s are identical, and the dangerous scenarios are the same. The differences are the six
exp02 scenarios that move from benign to late contact and the four exp02 runs whose
contact comes after 15 s. [results.md](results.md) lists both.

## `--legacy-l1` regression check

`--legacy-l1` keeps one 15 s window. The three quick runs were made before and after this
change (94 s, 22 s, 10 s before; 97 s, 22 s, 10 s after). exp03 and exp05 are identical in
every pre-existing column. exp02 is identical in every `benign`, `collision` and
`false_brake` flag and in every rate, and differs in one row: `crossing` gap_d 25,
arrival_t 1.0, Reactive, threshold 1.0 s brakes at 0.6 s instead of 0.5 s and makes contact
at 1.82 s instead of 1.95 s. Its TTC at t = 0.5 s is exactly the threshold, 1.0 s. With the
19 s log the ego path is 385 m long and the TTC evaluates to 0.9999999999999998; with the
23 s log the path is 445 m long and it evaluates to 1.0, which is not below the threshold.
The scenario collides either way. The L2 tables show no such difference (previous
section).

## Not run for this deliverable

- **Full resolution.** Dropping `--quick` gives 110 × 4 × 6 = 2640 rows for exp02,
  255 × 4 × 6 = 6120 for exp03 and 255 × 3 = 765 for exp05 (from the grids and threshold
  lists in `experiments/matrix_setup.py` and `scenario_sets.py`). These were not run and
  no runtime was measured.
- **`--legacy-l1` results.** The circle + single-threshold configuration of PR #8 is
  described in [d-baseline.md](d-baseline.md) §9. It was run only for the regression
  check above; its numbers are not reported.
- **exp04.** `experiments/exp04_latency.py` is a stub.

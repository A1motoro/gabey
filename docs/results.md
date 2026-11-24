# Experimental results and limitations (D, M3 draft)

> Owner: D (yj). Draft of the report's "实验结果 / 局限" section required by
> [d-baseline.md](d-baseline.md) §7 (M3): each conclusion is written as
> claim → evidence (figure) → limitation. Figure numbers follow d-baseline §5:
> **F2** = exp05, **F3** = exp02, **F4** = exp03. F5 (exp04, latency) was not run.

## What was run

| Item | Value |
| --- | --- |
| Mode | `--quick` only. No full-resolution run and no `--legacy-l1` run is reported here |
| Configuration | L2 baseline ([l2-baseline.md](l2-baseline.md)): 5.0 m × 2.5 m oriented rectangles (`r_safe=None`), `HysteresisPolicy(t_on, t_on + 1.0, a_brake=6.0)`, 4 s horizon (40 steps), 15 s scored simulation on 19 s logs, `latency = 0.0` in every row |
| Code | Experiment code is that of `main` at `a3f91cb`. Run on 2026-11-23 at `a3f91cb`, rerun at `3d34fe2` (a docs-only commit): CSVs identical except the `git_rev` column, summaries identical |
| Environment | Python 3.12.3, NumPy 2.5.5, pandas 3.1.1, Matplotlib 3.11.2, Linux |
| Outputs | `outputs/figures/` and `outputs/logs/` are gitignored. Regenerate them with [reproducibility.md](reproducibility.md) |

Every rate below is given with its count and denominator. The scenarios are deterministic
points on parameter grids, not samples from a traffic distribution, so no confidence
intervals are reported and none should be inferred.

## Read first: `benign` is defined on a 15 s window

A scenario is benign when the never-braking ego has no contact within the 15 s scored
simulation ([scenarios.md](scenarios.md) §10 item 5; open question Q2 in d-baseline §6).
D's logs are 4 s longer so that Oracle can look past the last scored frame. Re-running the
never-brake case over the whole 19 s log (`experiments/check_label_window.py`) shows:

| Set | Scenarios | Benign at 15 s | Of those, contact between 15 s and 19 s |
| --- | ---: | ---: | ---: |
| exp02 trade-off set | 110 | 53 | **6** |
| exp03 / exp05 quick grid | 25 | 11 | 0 |
| exp03 / exp05 full grid (not run as an experiment) | 255 | 101 | 6 |

The six exp02 scenarios:

| Scenario | First contact, never-brake [s] | Systems that brake within 15 s |
| --- | ---: | --- |
| `cut_in` gap_d 20, rel_v 1, lc_T 3 | 15.00 | all four, onset 13.1 s at t_on = 2.0 s |
| `cut_in` gap_d 20, rel_v 1, lc_T 4 | 15.00 | all four, onset 13.1 s |
| `lead_brake` init_gap 35, decel 0.3 | 15.14 | all four, onset 13.2–13.3 s |
| `cut_in` gap_d 40, rel_v 2, lc_T 3 | 17.50 | none |
| `cut_in` gap_d 40, rel_v 2, lc_T 4 | 17.50 | none |
| `lead_brake` init_gap 50, decel 0.3 | 18.32 | none |

Consequence: in the first three, every system brakes about 2 s before a contact that
happens at or just after the end of the scored window, and each of those brakes is counted
as a false brake. The F3 section splits the false-brake counts accordingly. The quick
grid behind F4 and F2 has no such cell, so those numbers are unaffected. No label or
metric was changed; whether to change the window is a team decision (Q2).

---

## F3 — exp02: collisions and false brakes on the mixed scenario set

`outputs/figures/exp02_tradeoff.png`, `outputs/logs/exp02.csv` (880 rows = 110 scenarios ×
4 systems × 2 thresholds), `outputs/logs/exp02_summary.json`. 57 dangerous and 53 benign
scenarios: A's default grid (96), the CA slice with `a_cutter = ±2` (8) and six aborted
cut-ins built in `experiments/scenario_sets.py`.

### Claim

At the same threshold (t_on = 2.0 s, t_off = 3.0 s), fewer dangerous scenarios end in a
collision with prediction than without: Reactive 14, CV 7, CA 4, Oracle 2 of 57.
This is a same-threshold comparison, not a comparison at equal false-brake rates, and the
quick run does not show a trade-off curve.

### Evidence

| System | Collisions, t_on = 1.0 s | Collisions, t_on = 2.0 s | False brakes (same at both thresholds) |
| --- | ---: | ---: | ---: |
| Reactive | 27/57 (0.474) | 14/57 (0.246) | 4/53 (0.075) |
| CV | 24/57 (0.421) | 7/57 (0.123) | 7/53 (0.132) |
| CA | 23/57 (0.404) | 4/57 (0.070) | 7/53 (0.132) |
| Oracle | 21/57 (0.368) | 2/57 (0.035) | 3/53 (0.057) |

- Each system false-brakes on exactly the same benign scenarios at t_on = 1.0 s and 2.0 s.
  In the figure each system is therefore a vertical segment: raising the threshold removed
  collisions without adding a false brake. Two thresholds do not give an operating point
  at which the systems share a false-brake rate.
- At t_on = 2.0 s CV has half of Reactive's collisions (7 vs 14) and more false brakes
  (7 vs 4). CA has the same false brakes as CV and three fewer collisions.
- Oracle's two remaining collisions are both `crossing` scenarios (gap_d 15, lateral_v 1.5,
  arrival_t 1.0 and 2.5). CV and CA collide in the same two. Oracle and CV brake from
  t = 0.0 s there and contact still occurs at 0.92 s and 0.98 s.

Where the false brakes come from (t_on = 2.0 s; identical at 1.0 s):

| System | False brakes | Late contact (see "Read first") | Aborted cut-ins | Other | On the 47 benign scenarios without late contact |
| --- | ---: | ---: | ---: | ---: | ---: |
| Reactive | 4 | 3 | 0 | 1 | 1/47 (0.021) |
| CV | 7 | 3 | 4 | 0 | 4/47 (0.085) |
| CA | 7 | 3 | 4 | 0 | 4/47 (0.085) |
| Oracle | 3 | 3 | 0 | 0 | 0/47 |

- All three Oracle false brakes are the late-contact scenarios: Oracle reads the real log
  and brakes for a contact that exists, just outside the scored window.
- CV's and CA's other four false brakes are four of the six aborted cut-ins, which were
  added by D to provoke exactly this. Reactive and Oracle do not brake on any abort.
- Reactive's one other false brake is a `crossing` scenario (gap_d 50, arrival_t 1.0).
- `tradeoff_without_aborts` in the summary (Reactive 4/47, CV / CA / Oracle 3/47) still
  contains the three late-contact scenarios for every system.

By family at t_on = 2.0 s (Reactive / CV / CA / Oracle):

| Family | Dangerous | Benign | Collisions | False brakes |
| --- | ---: | ---: | --- | --- |
| `cut_in` | 28 | 32 | 7 / 4 / 2 / 0 | 2 / 2 / 2 / 2 (both late contact) |
| `lead_brake` | 16 | 4 | 1 / 1 / 0 / 0 | 1 / 1 / 1 / 1 (late contact) |
| `crossing` | 9 | 7 | 6 / 2 / 2 / 2 | 1 / 0 / 0 / 0 |
| `cut_in_ca` | 4 | 4 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |
| `cut_in_abort` | 0 | 6 | — | 0 / 4 / 4 / 0 |

### Limitations

1. Two thresholds only. The six-threshold sweep was not run, so the shape of the
   collision / false-brake curve and any equal-false-brake comparison are not established.
2. The false-brake rate is dominated by two constructed effects: the window-end brakes
   (3 scenarios) and the hand-built aborts (6 scenarios, 4 of which trigger CV and CA).
   On the remaining 41 benign scenarios the counts are Reactive 1, CV 0, CA 0, Oracle 0.
   The measured cost of prediction in false brakes therefore rests on the abort family alone.
3. Family denominators are small (4 to 32). One scenario moves a family rate by 3 to 25
   percentage points.
4. `latency = 0.0` throughout. No noise is added to the trajectories the systems observe.

---

## F4 — exp03: failure boundaries on the cut-in (gap_d, rel_v) grid

`outputs/figures/exp03_boundaries.png`, `outputs/logs/exp03.csv` (200 rows = 25 scenarios ×
4 systems × 2 thresholds), `outputs/logs/exp03_summary.json`. Grid: gap_d ∈ {8, 14, 22, 32,
48} m × rel_v ∈ {−4, 0, 4, 8, 12} m/s, `lc_T = 3.0 s`, `a_cutter = 0`.
14 cells are dangerous and 11 benign. Benign cells are the ten with rel_v ≤ 0 plus
(gap_d 8, rel_v 12); every other cell with rel_v > 0 is dangerous.

### Claim

On this 5 × 5 grid at t_on = 2.0 s, Reactive leaves 6 of the 14 dangerous cells
collision-free, CV and CA 9, Oracle 12. CV rescues three cells that Reactive does not, and
Oracle three more that CV does not.

### Evidence

| System | Rescue rate, t_on = 2.0 s | Rescue rate, t_on = 1.0 s | Colliding cells (gap_d, rel_v) at t_on = 2.0 s |
| --- | ---: | ---: | --- |
| Reactive | 6/14 (0.429) | 5/14 | (8,4) (14,4) (8,8) (14,8) (22,8) (14,12) (22,12) (32,12) |
| CV | 9/14 (0.643) | 7/14 | (8,4) (8,8) (14,8) (14,12) (22,12) |
| CA | 9/14 (0.643) | 7/14 | same cells as CV |
| Oracle | 12/14 (0.857) | 8/14 | (8,8) (14,12) |

- rescue(CV) − rescue(Reactive) = 3/14 = **0.214**: cells (14,4), (22,8), (32,12).
- rescue(Oracle) − rescue(CV) = 3/14 = **0.214**: cells (8,4), (14,8), (22,12).
- The colliding sets are nested: Oracle ⊂ CV = CA ⊂ Reactive.
- **The two panels are identical on this grid.** The left panel uses, per system, the
  threshold with the fewest collisions among those with a false-brake rate ≤ 5 %
  (`rescue.budget`, `value_of_prediction`, `cost_of_prediction_error` in the summary).
  The right panel fixes t_on = 2.0 s (`rescue.fixed_2s`, `*_fixed_2s`). No system
  false-brakes on any of the 11 benign cells at either threshold (0/11), so the left
  panel also selects 2.0 s for all four systems and both key sets hold the same values.
  A full-resolution run can separate them and must be read from the matching key.
- The dashed curve is the zero-delay 1D stopping gap d = 5.0 + rel_v² / (2 · 6.0) m.
  Two dangerous cells lie below it, (8,8) and (14,12), and every system collides in both.
  Oracle brakes from t = 0.0 s in both and still makes contact at 1.70 s and 1.56 s.
  Of the 12 dangerous cells above the curve Oracle collides in none, CV and CA in 3,
  Reactive in 6.

### Limitations

1. 14 dangerous cells: one cell is 0.071 of rescue rate, and both headline differences
   are three cells. The full 15 × 17 grid was not run.
2. The contour lines in the figure interpolate between grid points 6 to 16 m and 4 m/s
   apart. The boundary position is known only to that spacing.
3. Along gap_d the danger label barely varies on this grid: with a 15 s window the cells
   with rel_v of 4 and 8 m/s are dangerous at all five gaps, up to 48 m. The benign side
   is almost entirely rel_v ≤ 0.
4. One lane-change duration (3.0 s) and one scenario family (cut-in). `a_cutter = 0`, so
   CA and CV are not distinguished here; they differ on exp02's mixed set.
5. Rescue is binary. A contact at any speed counts the same.
6. The full grid contains 6 late-contact benign cells out of 101. False-brake rates in a
   full run will include window-end brakes, which can change which threshold meets the
   5 % budget. Check `operating_point[].met_budget` before quoting the left panel.

---

## F2 — exp05: cut-in brake lead time

`outputs/figures/exp05_lead_time.png`, `outputs/logs/exp05.csv` (75 rows = 25 scenarios ×
{reactive, cv, ctrv}), `outputs/logs/exp05_summary.json` (`n_rows` = 50 is the number of
Reactive-vs-predictive pairs). Same grid as F4, t_on = 2.0 s, t_off = 3.0 s.
Lead time = onset(Reactive) − onset(predictive). It is defined only when both brake.

### Claim

In the 9 scenarios where both systems brake, the median lead time is 0.4 s for CV and
0.5 s for CTRV. The lead is not uniformly positive: it is zero or negative in 4 of the 9.
The three scenarios with a lead of 1.0 s or more are the three cells that CV rescues and
Reactive does not in F4.

### Evidence

| | CV | CTRV |
| --- | --- | --- |
| Both brake / predictive only / neither | 9 / 5 / 11 | 9 / 5 / 11 |
| Lead times, sorted [s] | −0.1, 0, 0, 0, 0.4, 0.6, 1.0, 1.0, 1.1 | −0.2, −0.1, 0, 0, 0.5, 0.7, 1.2, 1.2, 1.2 |
| Min / median / 75th percentile / max [s] | −0.1 / 0.4 / 1.0 / 1.1 | −0.2 / 0.5 / 1.2 / 1.2 |
| Reactive earlier / tie / predictive earlier | 1 / 3 / 5 | 2 / 2 / 5 |
| Median of the leads above 0.3 s (n = 5) | 1.0 s | 1.2 s |

- **Both brake (9, all dangerous).** The predictive system avoids contact in all 9.
  Reactive collides in 3 of them, cells (14,4), (22,8), (32,12), where it brakes at 2.0 s
  and makes contact at 2.10–2.28 s. Those are the three largest leads (CV 1.0, 1.0, 1.1 s;
  CTRV 1.2 s each).
- **Predictive only (5, all dangerous).** Lead time is undefined: Reactive has no brake
  before its contact. Both Reactive and the predictive system collide in all five. The
  predictive system brakes at 0.8–1.2 s and makes contact at 1.16–1.81 s.
- **Neither (11).** These are exactly the 11 benign cells.
- CTRV's onset is 0.1–0.2 s earlier than CV's in five of the nine both-brake scenarios,
  0.1 s later in two, and equal in two.
- `median_lead_time_when_predictive_is_earlier_s` in the summary is the median over
  both-brake pairs with a lead above 0.3 s. It is not a statistic of the predictive-only
  scenarios.

### Limitations

1. Nine pairs per system, on a 0.1 s simulation step. Differences of 0.1 s between CV
   and CTRV are one step.
2. One threshold. Lead times at other thresholds were not measured.
3. An earlier brake is not a rescue: in the five predictive-only scenarios the predictive
   system brakes and still collides.
4. A smooth quintic lane change with `lc_T = 3.0 s` only.

---

## What these results do not show

Stated in the repository and not changed by this work (README "Limitations",
[scenarios.md](scenarios.md) §10, d-baseline §8):

- Scenarios are synthetic and not calibrated to real lane changes. One agent per scenario,
  replayed from its log; it does not react to the ego.
- Point-mass motion, longitudinal braking only, 2D, no steering.
- No perception: systems read trajectory-level data.

Specific to this deliverable:

- **Latency.** Every row uses `latency = 0.0`. `run_matrix` accepts a latency list, but
  `experiments/exp04_latency.py` is still a stub that raises `NotImplementedError`, so
  nothing here says anything about delay.
- **Resolution.** Quick mode only: 2 thresholds, a 5 × 5 grid.
- **Scoring window.** Benign means "no contact within 15 s". See "Read first".
- **No statistics.** Deterministic grids and small counts. Rates are descriptive.
- **No external validity.** Nothing here is evidence about real traffic, real vehicles or
  compliance with any AEB standard.

## Summary

Research question: how much does prediction enlarge the AEB safe region, and where does it
still fail? In this configuration and on these grids:

1. On the cut-in grid at t_on = 2.0 s, prediction with CV raises the rescue rate from
   6/14 to 9/14, and the true future would raise it to 12/14 (F4). The remaining two
   cells are below the 1D stopping limit and are not rescued even by braking from t = 0.
2. The three cells CV gains are those where it brakes 1.0–1.1 s before Reactive (F2).
   Elsewhere the lead is between −0.1 s and 0.6 s.
3. On the mixed set at t_on = 2.0 s, collisions fall from 14/57 (Reactive) to 7/57 (CV),
   4/57 (CA) and 2/57 (Oracle) (F3).
4. The measured false-brake cost of prediction comes from the aborted cut-ins: CV and CA
   brake on 4 of 6, Reactive and Oracle on none. The other false brakes of CV, CA and
   Oracle are window-end brakes shared by all four systems (F3).

Open items that need a team decision or more runs are listed in the PR description and in
d-baseline §6: the scoring window (Q2), the full-resolution runs, and whether exp04 is
built or dropped.

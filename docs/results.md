# Experimental results and limitations (D, M3 draft)

> Owner: D (yj). Draft of the report's "实验结果 / 局限" section required by
> [d-baseline.md](d-baseline.md) §7 (M3): each conclusion is written as
> claim → evidence (figure) → limitation. Figure numbers follow d-baseline §5:
> **F2** = exp05, **F3** = exp02, **F4** = exp03. F5 (exp04, latency) was not run.
>
> **Scoring rule in this version.** Brakes are scored in the first 15 s and collisions
> over 19 s. This is D's implementation of a fix for open question Q2 and is **not yet
> agreed by A and C**: [scenarios.md](scenarios.md) §10 item 5, [interfaces.md](interfaces.md)
> and [l2-baseline.md](l2-baseline.md) still describe one 15 s window. The numbers under
> the one-window rule are kept below in "What the two windows changed".

## What was run

| Item | Value |
| --- | --- |
| Mode | `--quick` only. No full-resolution run is reported. `--legacy-l1` was run only as a regression check ([reproducibility.md](reproducibility.md)) |
| Configuration | L2 baseline ([l2-baseline.md](l2-baseline.md)): 5.0 m × 2.5 m oriented rectangles (`r_safe=None`), `HysteresisPolicy(t_on, t_on + 1.0, a_brake=6.0)`, 4 s horizon (40 steps), every run 19 s on 23 s logs, brakes scored in the first 15 s, `latency = 0.0` in every row |
| Code | Run on 2026-11-23 at `63ea148`. Two earlier runs of the same code before it was committed gave CSVs identical except the `git_rev` column, and identical summaries. The one-window numbers quoted for comparison are from `3d34fe2` |
| Environment | Python 3.12.3, NumPy 2.5.5, pandas 3.1.1, Matplotlib 3.11.2, Linux |
| Outputs | `outputs/figures/` and `outputs/logs/` are gitignored. Regenerate them with [reproducibility.md](reproducibility.md) |

Every rate below is given with its count and denominator. The scenarios are deterministic
points on parameter grids, not samples from a traffic distribution, so no confidence
intervals are reported and none should be inferred.

## Read first: two windows and three scenario classes

With one 15 s window for everything, two kinds of run were scored wrongly. A brake at
13 s for a real contact at 15.1 s was a false brake, because the never-brake run stopped
before the contact. A collision that braking only postponed from 14.8 s to 15.1 s was a
rescue, because the system's run stopped before it. The rule used here:

- Every simulation runs 19 s, the never-brake one and each system's.
- **Dangerous**: the never-brake ego makes contact in a 15 s run. This is the one-window
  label, unchanged scenario by scenario.
- **Benign**: the never-brake ego makes no contact in 19 s.
- **Late contact**: in between. These scenarios are in neither rate and are reported
  separately.
- **False brake**: a brake commanded in the first 15 s (decision steps 0.0 to 14.9 s) of
  a benign scenario.
- **Collision**: contact within 19 s in a dangerous scenario.

Why 19 s: a system brakes when its TTC falls below t_on, and t_on is at most 3.0 s in any
run here. A brake commanded before 15 s therefore claims a contact before 18 s. The
outcome window is 15 s plus the 4 s prediction horizon, the furthest any predictive
system looks.

| Set | Scenarios | Dangerous | Late contact | Benign |
| --- | ---: | ---: | ---: | ---: |
| exp02 trade-off set | 110 | 57 | **6** | 47 |
| exp03 / exp05 quick grid | 25 | 14 | 0 | 11 |
| exp03 / exp05 full grid (labels only, not run as an experiment) | 255 | 154 | 6 | 95 |

The six late-contact scenarios of exp02. Brake onsets are the range over the four systems:

| Scenario | First contact, never-brake [s] | Brake onset, t_on = 2.0 s [s] | Brake onset, t_on = 1.0 s [s] |
| --- | ---: | ---: | ---: |
| `cut_in` gap_d 20, rel_v 1, lc_T 3 | 15.00 | 13.1 | 14.1 |
| `cut_in` gap_d 20, rel_v 1, lc_T 4 | 15.00 | 13.1 | 14.1 |
| `lead_brake` init_gap 35, decel 0.3 | 15.14 | 13.2–13.3 | 14.2 |
| `cut_in` gap_d 40, rel_v 2, lc_T 3 | 17.50 | 15.5–15.6 | 16.5–16.6 |
| `cut_in` gap_d 40, rel_v 2, lc_T 4 | 17.50 | 15.5–15.6 | 16.5–16.6 |
| `lead_brake` init_gap 50, decel 0.3 | 18.32 | 16.4–16.5 | 17.4 |

All four systems brake in all six at both thresholds, and none of them collides. In the
first three the brake falls inside the decision window; under one 15 s window each of
those was a false brake.

The two `cut_in` rows at 15.00 s need a precise reading. 15 m of clearance closes at
1 m/s, so at t = 15.0 s the two rectangles touch with zero overlap. The simulator does not
count touching as contact: the 15 s never-brake run ends with `collision = False` and
`min_dist = 5.0 m`. Overlap begins in the step that starts at 15.0 s, and its timestamp
is 15.0. These two scenarios are late contact because the 15 s run ends at the instant of
touching, not because a contact on the last frame was ignored. `lead_brake` 35 / 0.3, with
contact at 15.14 s, is the plain case of a contact after the window.

### What the two windows changed

One-window numbers are those of the previous version of this document (same code for the
first 15 s; Reactive / CV / CA / Oracle):

| Quantity | One 15 s window | 15 s decisions, 19 s outcomes |
| --- | --- | --- |
| exp02 classes | 57 dangerous, 53 benign | 57 dangerous, 6 late contact, 47 benign |
| exp02 false brakes, both thresholds | 4 / 7 / 7 / 3 of 53 | 1 / 4 / 4 / 0 of 47 |
| exp02 collisions, t_on = 1.0 s | 27 / 24 / 23 / 21 of 57 | 28 / 25 / 24 / 22 of 57 |
| exp02 collisions, t_on = 2.0 s | 14 / 7 / 4 / 2 of 57 | unchanged |
| exp03 and exp05, quick grid | | every number unchanged |

- Each system loses the same three false brakes: the first three late-contact scenarios.
- Each system gains one collision at t_on = 1.0 s, in `lead_brake` init_gap 100, decel 1.0
  (never-brake contact at 14.78 s). All four systems brake at 13.8–13.9 s and make contact
  at 15.09–15.21 s. The 15 s run ended before that contact and counted a rescue.
- Compared row by row with the saved one-window tables (880 + 200 + 75 rows), every brake
  onset and every contact time inside the first 15 s is identical.

### Checks on the rule

- **Oracle has no false brake**: 0 of 47 at both thresholds (`oracle_false_brake_free` in
  `exp02_summary.json`). Oracle reads the real future, so a false brake by Oracle means
  the label missed a contact. Under one window it had three.
- **A longer outcome window changes no score.** `experiments/check_outcome_window.py`
  reran each saved matrix for 25 s instead of 19 s. exp02: no collision flag changes, no
  brake inside the first 15 s changes, and one label changes: `lead_brake` init_gap 80,
  decel 0.3 is benign at 19 s and makes never-brake contact at 23.36 s. No system brakes
  in its first 15 s, so it adds no false brake. exp03 and exp05 quick grid: nothing changes.

What the rule does not settle: a contact after 19 s still leaves the scenario benign (the
one case above), and the six late-contact scenarios are not scored at all. The 4 s margin
is argued from the thresholds and checked on these grids; it is not proved for other
scenario sets, thresholds above 3 s or a longer horizon.

---

## F3 — exp02: collisions and false brakes on the mixed scenario set

`outputs/figures/exp02_tradeoff.png`, `outputs/logs/exp02.csv` (880 rows = 110 scenarios ×
4 systems × 2 thresholds), `outputs/logs/exp02_summary.json`. 57 dangerous, 6 late-contact
and 47 benign scenarios: A's default grid (96), the CA slice with `a_cutter = ±2` (8) and
six aborted cut-ins built in `experiments/scenario_sets.py`.

### Claim

At the same threshold (t_on = 2.0 s, t_off = 3.0 s), fewer dangerous scenarios end in a
collision with prediction than without: Reactive 14, CV 7, CA 4, Oracle 2 of 57.
This is a same-threshold comparison, not a comparison at equal false-brake rates, and the
quick run does not show a trade-off curve.

### Evidence

| System | Collisions, t_on = 1.0 s | Collisions, t_on = 2.0 s | False brakes (same at both thresholds) |
| --- | ---: | ---: | ---: |
| Reactive | 28/57 (0.491) | 14/57 (0.246) | 1/47 (0.021) |
| CV | 25/57 (0.439) | 7/57 (0.123) | 4/47 (0.085) |
| CA | 24/57 (0.421) | 4/57 (0.070) | 4/47 (0.085) |
| Oracle | 22/57 (0.386) | 2/57 (0.035) | 0/47 |

- Each system false-brakes on exactly the same benign scenarios at t_on = 1.0 s and 2.0 s.
  In the figure each system is therefore a vertical segment: raising the threshold removed
  collisions without adding a false brake. Two thresholds do not give an operating point
  at which the systems share a false-brake rate.
- At t_on = 2.0 s CV has half of Reactive's collisions (7 vs 14) and more false brakes
  (4 vs 1). CA has the same false brakes as CV and three fewer collisions.
- Oracle's two remaining collisions are both `crossing` scenarios (gap_d 15, lateral_v 1.5,
  arrival_t 1.0 and 2.5). CV and CA collide in the same two. Oracle and CV brake from
  t = 0.0 s there and contact still occurs at 0.92 s and 0.98 s.

Where the false brakes come from (identical at both thresholds):

| System | False brakes | Aborted cut-ins | Other | On the 41 benign scenarios that are not aborts |
| --- | ---: | ---: | ---: | ---: |
| Reactive | 1 | 0 | 1 | 1/41 (0.024) |
| CV | 4 | 4 | 0 | 0/41 |
| CA | 4 | 4 | 0 | 0/41 |
| Oracle | 0 | 0 | 0 | 0/41 |

- CV's and CA's four false brakes are four of the six aborted cut-ins (gap_d 0 with rel_v
  0 and −2; gap_d 1.5 and 3.0 with rel_v 0), which were added by D to provoke exactly this.
  Reactive and Oracle do not brake on any abort.
- Reactive's one false brake is a `crossing` scenario (gap_d 50, arrival_t 1.0).
- `tradeoff_without_aborts` in the summary holds the last column.

By family at t_on = 2.0 s (Reactive / CV / CA / Oracle):

| Family | Dangerous | Late contact | Benign | Collisions | False brakes |
| --- | ---: | ---: | ---: | --- | --- |
| `cut_in` | 28 | 4 | 28 | 7 / 4 / 2 / 0 | 0 / 0 / 0 / 0 |
| `lead_brake` | 16 | 2 | 2 | 1 / 1 / 0 / 0 | 0 / 0 / 0 / 0 |
| `crossing` | 9 | 0 | 7 | 6 / 2 / 2 / 2 | 1 / 0 / 0 / 0 |
| `cut_in_ca` | 4 | 0 | 4 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |
| `cut_in_abort` | 0 | 0 | 6 | — | 0 / 4 / 4 / 0 |

### Limitations

1. Two thresholds only. The six-threshold sweep was not run, so the shape of the
   collision / false-brake curve and any equal-false-brake comparison are not established.
2. The false-brake cost of prediction rests on one constructed family. CV's and CA's
   false brakes are all on the hand-built aborts (4 of 6). On the other 41 benign
   scenarios the counts are Reactive 1, CV 0, CA 0, Oracle 0.
3. Family denominators are small (2 to 28). `lead_brake` has two benign scenarios, so one
   scenario there is 50 percentage points.
4. The six late-contact scenarios are in neither rate. Every system brakes and avoids
   contact in all six; that is reported above and is not part of the comparison.
5. `latency = 0.0` throughout. No noise is added to the trajectories the systems observe.

---

## F4 — exp03: failure boundaries on the cut-in (gap_d, rel_v) grid

`outputs/figures/exp03_boundaries.png`, `outputs/logs/exp03.csv` (200 rows = 25 scenarios ×
4 systems × 2 thresholds), `outputs/logs/exp03_summary.json`. Grid: gap_d ∈ {8, 14, 22, 32,
48} m × rel_v ∈ {−4, 0, 4, 8, 12} m/s, `lc_T = 3.0 s`, `a_cutter = 0`.
14 cells are dangerous and 11 benign; none is late contact. Benign cells are the ten with rel_v ≤ 0 plus
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
3. Along gap_d the danger label barely varies on this grid: with contact looked for over
   15 s, the cells with rel_v of 4 and 8 m/s are dangerous at all five gaps, up to 48 m.
   The benign side is almost entirely rel_v ≤ 0.
4. One lane-change duration (3.0 s) and one scenario family (cut-in). `a_cutter = 0`, so
   CA and CV are not distinguished here; they differ on exp02's mixed set.
5. Rescue is binary. A contact at any speed counts the same.
6. The full grid has 154 dangerous, 6 late-contact and 95 benign cells. The late-contact
   cells (gap_d 21.4 to 60.0 m, rel_v 1 to 3 m/s, never-brake contact at 15.8–18.3 s) are
   in neither rate in a full run. Check `cells` and `operating_point[].met_budget` in the
   summary before quoting the left panel.

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
- Onsets are taken over the whole 19 s run. Every onset here lies between 0.8 s and 8.8 s
  (`brake_onsets_after_decision_window` = 0), so the 15 s decision window plays no part.
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
- **Scoring windows.** See "Read first". The two-window rule is D's and awaits A's and
  C's agreement; C's closed-loop experiment and `--legacy-l1` use one 15 s window. A
  contact after 19 s still leaves a scenario benign, and late-contact scenarios are unscored.
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
   brake on 4 of 6, Reactive and Oracle on none. The only other false brake is one by
   Reactive in a crossing scenario. Oracle has none (F3).

Open items that need a team decision or more runs are listed in the PR description and in
d-baseline §6: A's and C's agreement to the two scoring windows (Q2), the full-resolution
runs, and whether exp04 is built or dropped.

# Experimental Results — D's M3 Deliverable

> **Date**: 2026-11-23  
> **Baseline**: L2 (OBB geometry, hysteresis policy, swept TTC)  
> **Config**: Quick mode (5×5 boundary grid, 110 tradeoff scenarios, 2 thresholds)  
> **Owner**: D (yj)

---

## Overview

This document presents the experimental results for the three core evaluation experiments (exp02, exp03, exp05) that answer the research question:

> **How much does prediction expand the AEB safety working region, and where does it still fail?**

All experiments use:
- **Scenarios**: A's parametric synthetic scenarios (quintic lane change, lead brake, crossing)
- **Systems**: Reactive / CV / CA / Oracle (CORE set)
- **Simulation**: L2 baseline (OBB 5.0m×2.5m, hysteresis policy, 15s duration, 40-step horizon)
- **Fairness**: Benign/dangerous labels via counterfactual (NeverBrakePolicy)

---

## Experiment 02: Safety–Nuisance Trade-off

### Claim

**Each AEB system exhibits a trade-off between collision rate (dangerous scenarios) and false-brake rate (benign scenarios). At comparable false-brake budgets, predictive systems achieve lower collision rates than reactive.**

### Evidence

**Figure**: `outputs/figures/exp02_tradeoff.png`  
**Data**: `outputs/logs/exp02.csv` (880 runs = 110 scenarios × 4 systems × 2 thresholds)  
**Summary**: `outputs/logs/exp02_summary.json`

**Key Results** (threshold = 2.0s, hysteresis off at 3.0s):

| System | Collision Rate | False-Brake Rate | n_dangerous | n_benign |
|--------|----------------|------------------|-------------|----------|
| Reactive | 0.246 (14/57) | 0.075 (4/53) | 57 | 53 |
| CV | 0.123 (7/57) | 0.132 (7/53) | 57 | 53 |
| CA | 0.070 (4/57) | 0.132 (7/53) | 57 | 53 |
| Oracle | 0.035 (2/57) | 0.057 (3/53) | 57 | 53 |

**Interpretation**:
- **CV vs Reactive**: CV cuts collision rate by **50%** (0.123 vs 0.246) with **76% higher false-brake rate** (0.132 vs 0.075)
- **CA vs CV**: CA further reduces collision rate by **43%** (0.070 vs 0.123) at same false-brake rate (0.132)
- **Oracle upper bound**: Even perfect prediction still has 3.5% collision rate (2 unsaveable scenarios at t=2.0s threshold)

**Abort Cut-in Analysis**:
- **With aborts** (6 scenarios): CV/CA false-brake rate = 0.132
- **Without aborts**: CV/CA false-brake rate = 0.064 (drops by **51%**)
- Aborts test CV's vulnerability to cut-ins that lean toward ego lane then return

**Per-Family Breakdown** (from summary JSON):

| Family | n_dangerous | n_benign | Collision Rate @ t=2.0s (R/CV/CA/O) | False-Brake Rate @ t=2.0s (R/CV/CA/O) |
|--------|-------------|----------|-------------------------------------|--------------------------------------|
| cut_in | 28 | 32 | 0.250 / 0.143 / 0.071 / 0.000 | 0.062 / 0.062 / 0.062 / 0.062 |
| lead_brake | 16 | 4 | 0.062 / 0.062 / 0.000 / 0.000 | 0.250 / 0.250 / 0.250 / 0.250 |
| crossing | 9 | 7 | 0.667 / 0.222 / 0.222 / 0.222 | 0.143 / 0.000 / 0.000 / 0.000 |
| cut_in_ca | 4 | 4 | 0.000 / 0.000 / 0.000 / 0.000 | 0.000 / 0.000 / 0.000 / 0.000 |
| cut_in_abort | 0 | 6 | — | 0.000 / 0.667 / 0.667 / 0.000 |

**Observations**:
- **Cut-in**: CV reduces Reactive's 25% collision rate to 14.3%; CA to 7.1%; Oracle to 0%
- **Crossing**: Reactive fails 67% (6/9); predictive systems fail 22% (2/9) — lateral extrapolation helps but not enough
- **Lead brake**: Nearly all systems avoid collision; one Reactive/CV failure each at 6.2%
- **CA slice**: All systems succeed (comfortable longitudinal acceleration, no edge cases)
- **Aborts**: Only CV/CA false-brake (67% = 4/6); Reactive and Oracle ignore harmless maneuvers

### Limitations

1. **Trade-off shape depends on threshold grid**: Only 2 thresholds tested (1.0s, 2.0s); full curve needs denser sweep
2. **Benign scenarios are synthetic**: Abort cut-ins hand-crafted to test false-brake; real benign distribution unknown
3. **No perception uncertainty**: Direct trajectory input; real sensors add noise → more false brakes
4. **No computational delay**: Assumes instant prediction; real systems have 50–200ms latency
5. **Fixed ego policy**: Log replay agents don't respond to ego braking; real agents might react

---

## Experiment 03: Failure Boundary (HEADLINE FIGURE)

### Claim

**On the (gap_d, rel_v) plane, CV's safe region is larger than Reactive's, but Oracle's is larger still. The area differences quantify (1) the value of prediction and (2) the cost of prediction error.**

### Evidence

**Figure**: `outputs/figures/exp03_boundaries.png` (two panels)  
**Data**: `outputs/logs/exp03.csv` (200 runs = 25 scenarios × 4 systems × 2 thresholds)  
**Summary**: `outputs/logs/exp03_summary.json`

**Key Results** (5×5 quick grid, t_on=2.0s):

| System | Rescue Rate | Collisions (out of 14 dangerous) |
|--------|-------------|----------------------------------|
| Reactive | 0.429 (6/14 saved) | 8 |
| CV | 0.643 (9/14 saved) | 5 |
| CA | 0.643 (9/14 saved) | 5 |
| Oracle | 0.857 (12/14 saved) | 2 |

**Value of Prediction**: rescue(CV) − rescue(Reactive) = **0.214** (3 additional scenarios saved)  
**Cost of Prediction Error**: rescue(Oracle) − rescue(CV) = **0.214** (3 more saveable with perfect foresight)

**Interpretation**:
- CV's lateral extrapolation rescues **3 cut-in scenarios** (21.4% improvement) that Reactive misses
- CA matches CV on this grid (quintic profile has zero acceleration at endpoints; CA's advantage needs acceleration-rich scenarios)
- Oracle's perfect foresight rescues **3 more scenarios** that CV's constant-velocity assumption misclassifies

**Stopping Distance Overlay**:
- Figure includes 1D kinematic limit: $d_{\text{safe}} = 5.0 + v_{\text{rel}}^2 / (2 \times 6.0)$ [m]
- Physical brake limit (zero delay, full deceleration) forms the **absolute lower bound**
- All system boundaries stay above this curve (no unphysical braking)
- Oracle boundary approaches but doesn't violate the kinematic limit

**Grid Coverage**:
- **14 dangerous** (56%), **11 benign** (44%) out of 25 scenarios
- Dangerous: low gap_d / high rel_v corner
- Benign: high gap_d / negative rel_v (ego slower than cutter) corner

### Limitations

1. **Coarse grid**: 5×5 quick grid; full 15×17 grid would reveal finer boundary details
2. **Fixed lane-change duration**: lc_T = 3.0s only; faster lane changes (2.0s) might shift boundaries
3. **Single conflict type**: Cut-in only; lead_brake and crossing have different boundary shapes
4. **Rescue rate ignores severity**: A grazing contact and a high-speed collision both count as "not rescued"
5. **2D point-mass dynamics**: Real vehicles have steering, stability control, and lateral dynamics that could expand the safe region
6. **Deterministic scenarios**: No stochasticity in agent behavior; real cut-ins have intent uncertainty

---

## Experiment 05: Cut-in Lead Time

### Claim

**Predictive systems detect imminent cut-in collisions earlier than Reactive, providing additional reaction time (lead time). CV achieves ~0.4s median lead time; CTRV achieves ~0.5s.**

### Evidence

**Figure**: `outputs/figures/exp05_lead_time.png`  
**Data**: `outputs/logs/exp05.csv` (50 runs = 25 scenarios × 2 predictive systems)  
**Summary**: `outputs/logs/exp05_summary.json`

**Key Results** (threshold t_on=2.0s, hysteresis t_off=3.0s):

| System | Both Brake | Predictive Only | Neither | Median Lead Time (all) | Median Lead Time (predictive earlier) |
|--------|------------|-----------------|---------|------------------------|---------------------------------------|
| CV | 9/25 | 5/25 | 11/25 | 0.4s | 1.0s |
| CTRV | 9/25 | 5/25 | 11/25 | 0.5s | 1.2s |

**Interpretation**:
- **Both brake** (9 scenarios): Both Reactive and predictive systems trigger; predictive is earlier by 0.4–0.5s median
- **Predictive only** (5 scenarios): Reactive never brakes (cutter still in adjacent lane); predictive brakes **up to 1.0–1.2s earlier**
- **Neither** (11 scenarios): Benign cut-ins or large gaps; no braking needed
- **CTRV advantage**: Slightly longer lead time (0.1s median, 0.2s when earlier) due to turn-rate modeling

**Lead Time Distribution**:
- Minimum: ~0.1s (marginal earlier detection)
- Maximum: ~1.5s (early cut-in prediction before lateral crossing)
- 75th percentile: ~0.8s for CV, ~1.0s for CTRV

### Limitations

1. **Lead time ≠ safety margin**: Earlier braking doesn't guarantee collision avoidance; depends on deceleration and distance
2. **Fixed threshold**: t_on=2.0s; more aggressive thresholds (1.0s) would shift lead times but increase false brakes
3. **No multi-agent scenarios**: Only one cutter per scenario; real highway has multiple lane-changers
4. **Quintic profile**: Smooth, predictable lane changes; abrupt swerves would challenge CTRV's constant turn-rate assumption
5. **Lead time doesn't account for latency**: Real systems lose 50–200ms to sensing/compute; net lead time is reduced

---

## Summary of Limitations (Consolidated)

### Data & Scenario Limitations

1. **Synthetic scenarios only**: Parametric quintic lane changes, not real traffic distributions
2. **2D simplification**: No vertical dynamics, road grade, or banking
3. **Point-mass vehicles**: No yaw, roll, pitch, or tire friction models
4. **No multi-agent interaction**: Agents don't respond to ego braking (log replay)
5. **Fixed ego path**: Straight-line only; no steering evasion
6. **Deterministic**: No stochastic agent behavior or intent uncertainty

### Perception & Sensing Limitations

1. **Perfect perception**: Direct trajectory input; no sensor noise, occlusion, or failure modes
2. **Latency approximation**: Computational delay modeled as fixed lag; real pipelines have variable latency
3. **No tracking errors**: No lost tracks, ID switches, or multi-hypothesis tracking
4. **Full observability**: Complete state of all agents; real sensors have limited FOV and range

### Modeling Limitations

1. **Simplified predictors**: CV/CA/CTRV are classical baselines; modern learned models (LSTM, Transformer) may perform better
2. **Fixed TTC threshold**: Binary brake decision; real controllers use continuous pedal modulation
3. **No brake fade or ABS**: Assumes constant deceleration; real brakes heat up and lose effectiveness
4. **Rectangular collision geometry**: 5.0m × 2.5m OBB; real vehicles have complex shapes
5. **No impact dynamics**: Collision is binary (yes/no); no injury severity or crash pulse modeling

### Experimental Limitations

1. **Coarse grids**: Quick mode (5×5 boundary, 2 thresholds) for speed; full grids would give finer resolution
2. **Single lc_T**: Only 3.0s lane-change duration tested; 2.0s or 4.0s might shift boundaries
3. **No sensitivity analysis**: Lateral acceleration bound (3.0 m/s²), brake decel (6.0 m/s²) not swept
4. **Limited benign scenarios**: Abort cut-ins hand-crafted; real benign distribution unknown
5. **No real-world validation**: Results not compared to field data (nuScenes, Waymo, etc.)

### Generalization Limitations

1. **No external validity**: Conclusions apply to this specific 2D point-mass TTC-threshold setup only
2. **No safety certification**: Results do not imply compliance with ISO 26262, NHTSA, or Euro NCAP standards
3. **No deployment guidance**: Not sufficient for production AEB tuning or liability assessment

---

## Implications for Future Work

### Near-Term Improvements (Within Current Framework)

1. **Dense grid sweeps**: Run full 15×17 boundary grid and 6-threshold trade-off
2. **Sensitivity analysis**: Vary max_lat_acc (2.0–4.5 m/s²), a_brake (4.0–8.0 m/s²), lc_T (2.0–4.0s)
3. **Latency experiments**: Complete exp04 (τ ∈ [0, 300] ms) to quantify perception delay cost
4. **Per-family analysis**: Separate boundaries for cut_in, lead_brake, crossing
5. **Impact severity**: Weight collisions by relative velocity at first contact

### Medium-Term Extensions (Requires New Implementation)

1. **Learned predictors**: Add LSTM/Transformer baselines (requires training dataset)
2. **Continuous control**: Replace binary brake with PID or MPC throttle/brake modulation
3. **Steering evasion**: Add lateral control (requires path planning and stability checks)
4. **Multi-agent interaction**: Agent responds to ego braking (requires game-theoretic modeling)
5. **Sensor simulation**: Add noise, occlusion, and tracking errors (requires perception pipeline)

### Long-Term Validation (Requires Real Data)

1. **Real scenario distribution**: Fit parameters to naturalistic driving data (NGSIM, HighD, inD)
2. **Field validation**: Test on nuScenes, Waymo, or recorded vehicle data
3. **Human-in-loop**: Compare AEB timing to human drivers' brake responses
4. **Crash reconstruction**: Validate against NHTSA crash database (pre-crash kinematics)

---

## Conclusion

The experimental results demonstrate:

1. ✅ **Prediction improves safety**: CV reduces collision rate by 50% vs Reactive (exp02)
2. ✅ **Prediction expands safe region**: CV rescues 21% more dangerous scenarios (exp03)
3. ✅ **Prediction provides lead time**: CV/CTRV brake 0.4–0.5s earlier on cut-ins (exp05)
4. ⚠️ **Prediction increases false brakes**: CV's false-brake rate is 76% higher than Reactive (exp02)
5. ⚠️ **Prediction error has cost**: Oracle outperforms CV by 21%, showing room for better predictors (exp03)

**Research question answered**: In the specific setting of 2D point-mass kinematics with TTC-threshold braking on parametric synthetic scenarios, **prediction expands the AEB safety region by ~21% (measured by rescue rate)**, but **perfect prediction (Oracle) could expand it by an additional 21%**, indicating significant value from improving prediction accuracy.

**Generalization**: These conclusions apply to the tested configuration and do not establish real-world safety superiority. Field validation on naturalistic driving data is required before deployment.

---

**Generated**: 2026-11-23 13:42 UTC  
**Figures**: `outputs/figures/exp{02,03,05}*.png`  
**Data**: `outputs/logs/exp{02,03,05}.{csv,json}`  
**Tests**: 348/350 passed (2 failures: missing optional toy data)  
**Commit**: To be included in D's M3 PR on branch `cursor/d-m3-results-and-report-64a3`

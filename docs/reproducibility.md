# Reproducibility Guide

> How to regenerate all experimental results from scratch

## Quick Start (< 5 minutes)

```bash
# 1. Setup environment
conda activate aie1903_ad  # or your Python 3.12+ environment
pip install -r requirements.txt
pip install -e .

# 2. Run tests to verify setup
pytest

# 3. Generate all experimental results (quick mode)
python experiments/exp02_threshold_sweep.py --quick
python experiments/exp03_failure_boundary.py --quick
python experiments/exp05_cut_in_lead_time.py --quick

# 4. Check outputs
ls outputs/figures/  # exp02_tradeoff.png, exp03_boundaries.png, exp05_lead_time.png
ls outputs/logs/     # exp{02,03,05}.{csv,json}
```

**Expected runtime**: ~3 minutes on typical workstation

## Full Resolution (~ 30 minutes)

For publication-quality figures with dense grids:

```bash
python experiments/exp02_threshold_sweep.py  # 110 scenarios × 4 systems × 6 thresholds
python experiments/exp03_failure_boundary.py  # 255 scenarios × 4 systems × 2 thresholds
python experiments/exp05_cut_in_lead_time.py  # 255 scenarios × 2 systems
```

**Expected runtime**: ~30 minutes (depends on CPU)

## Output Files

All experimental outputs are **gitignored** (see `.gitignore:7-9`) for repository cleanliness. They are **reproducible** from code and should be regenerated locally.

### Generated Files

```
outputs/
├── figures/           # Publication-ready PNG plots
│   ├── exp02_tradeoff.png         # Collision rate vs false-brake rate
│   ├── exp03_boundaries.png       # Failure boundaries (headline figure)
│   └── exp05_lead_time.png        # Cut-in lead time distributions
└── logs/              # Raw data for post-processing
    ├── exp02.csv                  # 880 rows (quick) / 2640 rows (full)
    ├── exp02_summary.json         # Aggregated metrics + config
    ├── exp03.csv                  # 200 rows (quick) / 2040 rows (full)
    ├── exp03_summary.json         # Rescue rates + value/cost of prediction
    ├── exp05.csv                  # 50 rows (quick) / 510 rows (full)
    └── exp05_summary.json         # Lead time medians + brake status counts
```

### File Sizes (Quick Mode)

- `exp02_tradeoff.png`: ~54 KB
- `exp03_boundaries.png`: ~133 KB (two-panel figure)
- `exp05_lead_time.png`: ~69 KB
- `exp02.csv`: ~188 KB (880 runs)
- `exp03.csv`: ~43 KB (200 runs)
- `exp05.csv`: ~17 KB (50 runs)

## Redrawing Figures from Saved Data

If you have CSV files but want to regenerate figures (e.g., after tweaking plot styles):

```bash
python experiments/exp02_threshold_sweep.py --from-csv outputs/logs/exp02.csv
python experiments/exp03_failure_boundary.py --from-csv outputs/logs/exp03.csv
python experiments/exp05_cut_in_lead_time.py --from-csv outputs/logs/exp05.csv
```

This skips the simulation and only redraws the figures.

## Verifying Results Against Documentation

After running experiments, compare the output JSON summaries against `docs/results.md` and `docs/d-baseline.md:122-150`:

```bash
# Check key metrics
jq '.rescue.fixed_2s' outputs/logs/exp03_summary.json
# Expected: {"reactive": 0.429, "cv": 0.643, "ca": 0.643, "oracle": 0.857}

jq '.value_of_prediction' outputs/logs/exp03_summary.json
# Expected: 0.214

jq '.median_lead_time_s' outputs/logs/exp05_summary.json
# Expected: {"cv": ~0.4, "ctrv": ~0.5}
```

## Legacy L1 Mode

To reproduce the original PR #8 results (circular collision geometry + single-threshold policy):

```bash
python experiments/exp02_threshold_sweep.py --quick --legacy-l1
python experiments/exp03_failure_boundary.py --quick --legacy-l1
python experiments/exp05_cut_in_lead_time.py --quick --legacy-l1
```

**Differences from L2**:
- Collision geometry: circle (r_safe=2.5m) instead of OBB (5.0m × 2.5m)
- Policy: TTCThresholdPolicy (single threshold) instead of HysteresisPolicy (t_on, t_off)
- Stopping clearance: 2.5m instead of 5.0m

Collision counts and rescue rates should match; lead time may differ slightly due to brake onset truncation at first collision.

## Determinism & Random Seeds

All experiments use **fixed seeds** (default: 0) and **deterministic simulation**. Re-running should produce **bit-identical** results.

```bash
# Verify determinism
python experiments/exp02_threshold_sweep.py --quick
cp outputs/logs/exp02.csv exp02_run1.csv
python experiments/exp02_threshold_sweep.py --quick
diff exp02_run1.csv outputs/logs/exp02.csv  # Should be identical
```

If results differ, check:
1. Python version (tested on 3.12.3)
2. NumPy version (tested on 1.24+)
3. Random state leakage (all scenarios are deterministic; no stochastic elements)

## Troubleshooting

### "No module named pbs"

```bash
pip install -e .
```

### "FileNotFoundError: no *.csv trajectories"

This error occurs when running `test_data.py::test_load_toy_*`. It's **expected** if you don't have the optional course data. These tests don't affect the core experiments.

```bash
# Optional: Copy course data (if available)
cp -r /path/to/AIE1903_AD/data/sample_trajectories/ data/
```

### "outputs/figures/ is empty"

The figures are **gitignored**. Run the experiments to generate them locally:

```bash
python experiments/exp02_threshold_sweep.py --quick
ls outputs/figures/  # Should show exp02_tradeoff.png
```

### Windows Path Issues

On Windows, use:

```bash
conda run --no-capture-output -n aie1903_ad python experiments/exp02_threshold_sweep.py --quick
```

Direct `python.exe` invocation may lack NumPy's native library environment.

## Performance Notes

**Quick mode** (recommended for iterative development):
- exp02: ~140s (110 scenarios × 4 systems × 2 thresholds = 880 sims)
- exp03: ~33s (25 scenarios × 4 systems × 2 thresholds = 200 sims)
- exp05: ~14s (25 scenarios × 2 systems = 50 sims)
- **Total: ~3 minutes**

**Full mode** (publication-quality):
- exp02: ~8 minutes (110 scenarios × 4 systems × 6 thresholds = 2640 sims)
- exp03: ~12 minutes (255 scenarios × 4 systems × 2 thresholds = 2040 sims)
- exp05: ~5 minutes (255 scenarios × 2 systems = 510 sims)
- **Total: ~25 minutes**

Parallelization is **not implemented** (scenarios run serially). Future work could add `multiprocessing` over scenario batches.

## Citation

If you use these experiments in your work, cite:

```bibtex
@techreport{gabey2026,
  title={Predictive Braking Simulator: Quantifying the Safety–Nuisance Trade-off in AEB Systems},
  author={Team gabey (C, P, A, D)},
  institution={AIE1903 Exploration on Basic Autonomous Driving},
  year={2026},
  note={Topic 9, Course Project}
}
```

---

**Last Updated**: 2026-11-23  
**Maintainer**: D (yj)  
**Related Docs**: `docs/results.md`, `docs/d-baseline.md`, `README.md`

"""Audit EVERY geometric cut-in candidate; freeze illustrative danger/false-brake cases.

These outcome-selected cases illustrate behavior and never enter the primary cohort.
The full audit (including uninteresting outcomes) is saved to make selection explicit.
"""

import argparse
import csv
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pbs.data.womd import (  # noqa: E402
    DEFAULT_RAW_DIR,
    clip_evidence,
    read_scenarios,
    real_scenario,
    save_clip,
)
from pbs.eval.counterfactual import is_benign  # noqa: E402
from pbs.eval.metrics import evaluate  # noqa: E402
from pbs.sim.policy import HysteresisPolicy  # noqa: E402
from pbs.sim.simulator import run_sim  # noqa: E402
from pbs.systems import make_system  # noqa: E402


def main(args):
    with (args.fixtures / "womd_candidates.csv").open(encoding="utf-8", newline="") as f:
        candidates = [r for r in csv.DictReader(f) if r["group"] == "cut_in"]
    by_shard = {}
    for row in candidates:
        by_shard.setdefault(row["source_shard"], {}).setdefault(row["clip"], []).append(row)
    results, cases, roles = [], [], set()
    for shard, targets in sorted(by_shard.items()):
        for scene in read_scenarios(args.path / shard):
            if scene.scenario_id not in targets:
                continue
            for row in targets[scene.scenario_id]:
                sc = real_scenario(scene, int(row["track_index"]), anchor=int(row["anchor"]))
                benign = is_benign(sc, duration=6, r_safe=2.5)
                result = {
                    "clip": scene.scenario_id,
                    "track_id": sc.params["track_id"],
                    "source_shard": shard,
                    "anchor": sc.params["anchor"],
                    "benign": benign,
                }
                for system in ("reactive", "cv"):
                    metrics = evaluate(
                        run_sim(
                            sc, make_system(system), HysteresisPolicy(), duration=6, r_safe=2.5
                        ),
                        benign,
                    )
                    onset = metrics["brake_onset_t"]
                    metrics["brake_onset_t"] = None if onset is None else float(onset)
                    result[system] = metrics
                results.append(result)
                role = (
                    "dangerous"
                    if not benign
                    else ("false_brake" if result["cv"]["false_brake"] else None)
                )
                if role is None or role in roles:
                    continue
                # Selection uses outcomes ONLY for explicitly labelled illustration.
                p = sc.params
                p.update(
                    {
                        "clip": scene.scenario_id,
                        "source_shard": shard,
                        "record_index": int(row["record_index"]),
                        "group": "cut_in",
                        "cohort": "illustrative",
                        "case_role": role,
                        "t_cross_s": float(row["t_cross_s"]),
                        "lon_m": float(row["lon_m"]),
                        "lc_T_s": float(row["lc_T_s"]),
                        "rel_v": float(row["rel_v"]),
                        "gap_d": float(row["gap_d"]),
                        "selection": "outcome-selected illustration from full 40-candidate audit; "
                        "excluded from primary cohort statistics",
                    }
                )
                filename = f"cases/womd_{role}_{scene.scenario_id}_track{p['track_id']}.npz"
                path = args.fixtures / filename
                save_clip(sc, path, evidence=clip_evidence(scene, sc))
                cases.append(
                    {
                        **p,
                        "filename": filename,
                        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                        "size_bytes": path.stat().st_size,
                    }
                )
                roles.add(role)
                print(
                    f"Illustration {role}: {scene.scenario_id}; "
                    f"reactive={result['reactive']['brake_onset_t']}, "
                    f"cv={result['cv']['brake_onset_t']}",
                    flush=True,
                )
    if len(results) != len(candidates):
        raise ValueError("Audit did not cover every candidate in the frozen ledger")
    audit = {
        "candidate_count": len(results),
        "unique_scenarios": len({r["clip"] for r in results}),
        "benign_count": sum(r["benign"] for r in results),
        "dangerous_count": sum(not r["benign"] for r in results),
        "systems": {
            name: {
                "braked": sum(r[name]["brake_onset_t"] is not None for r in results),
                "false_brakes": sum(r[name]["false_brake"] for r in results),
                "collisions": sum(r[name]["collision"] for r in results),
            }
            for name in ("reactive", "cv")
        },
        "simulation": "duration=6, dt=0.1, horizon=30, HysteresisPolicy(2,3,6), "
        "r_safe=2.5, latency=0, selected target only",
        "results": results,
    }
    (args.fixtures / "womd_cut_in_audit.json").write_text(
        json.dumps(audit, indent=2, allow_nan=False), encoding="utf-8"
    )
    (args.fixtures / "womd_cases_manifest.json").write_text(
        json.dumps(
            {
                "selection": "outcome-selected illustrations; excluded from primary statistics",
                "case_counts": dict(Counter(c["case_role"] for c in cases)),
                "selected": cases,
            },
            indent=2,
            allow_nan=False,
        ),
        encoding="utf-8",
    )
    print(json.dumps({k: v for k, v in audit.items() if k != "results"}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", type=Path, default=DEFAULT_RAW_DIR)
    parser.add_argument("--fixtures", type=Path, default=ROOT / "tests/fixtures")
    main(parser.parse_args())

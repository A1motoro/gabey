"""Scan WOMD by geometry and freeze first eligible, distinct scenarios per cohort.

No prediction or brake outcomes are consulted. Full candidate ledger is preserved.
Run: python scripts/export_womd_clips.py [waymo/v1.2/validation] --per-group 5
"""

import argparse
import csv
import hashlib
import json
import re
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pbs.data.womd import (  # noqa: E402
    DEFAULT_RAW_DIR,
    clip_evidence,
    find_candidates,
    read_scenarios,
    real_scenario,
    save_clip,
    shard_paths,
)


def main(args):
    if args.per_group < 1:
        raise ValueError("per-group must be positive")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.log_dir.mkdir(parents=True, exist_ok=True)
    rows, selected, used_ids = [], [], set()
    counts, scanned, rejected, shard_counts = Counter(), 0, Counter(), {}
    for shard in shard_paths(args.path):
        n = 0
        for record_index, scene in enumerate(read_scenarios(shard)):
            n += 1
            scanned += 1
            try:
                candidates = find_candidates(scene)
            except ValueError as exc:
                rejected[str(exc)] += 1
                continue
            for candidate in sorted(candidates, key=lambda r: (r["group"], r["track_id"])):
                row = dict(
                    candidate, source_shard=shard.name, record_index=record_index, selected=False
                )
                rows.append(row)
                group = candidate["group"]
                if counts[group] >= args.per_group or scene.scenario_id in used_ids:
                    continue
                scenario = real_scenario(
                    scene, candidate["track_index"], anchor=candidate["anchor"], group=group
                )
                scenario.params.update(candidate)
                scenario.params.update(source_shard=shard.name, record_index=record_index)
                safe_id = re.sub(r"[^a-zA-Z0-9_-]", "_", scene.scenario_id)
                filename = f"womd_{group}_{safe_id}_track{candidate['track_id']}.npz"
                target = args.output_dir / filename
                save_clip(scenario, target, evidence=clip_evidence(scene, scenario))
                row["selected"] = True
                selected.append(
                    dict(
                        row,
                        filename=filename,
                        sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                        size_bytes=target.stat().st_size,
                    )
                )
                counts[group] += 1
                used_ids.add(scene.scenario_id)
                print(
                    f"selected {group}: {filename}; anchor={candidate['anchor']}, "
                    f"cross={candidate['t_cross_s']}",
                    flush=True,
                )
        shard_counts[shard.name] = n
        print(
            f"scanned {shard.name}: {n} records; total={scanned}; selected={dict(counts)}",
            flush=True,
        )
    if rows:
        with (args.log_dir / "exp07_candidates.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    manifest = {
        "dataset": "Waymo Open Motion v1.2 validation",
        "records_scanned": scanned,
        "shard_record_counts": shard_counts,
        "crc32c_verified": True,
        "eligible_candidates": len(rows),
        "eligible_by_group": dict(Counter(r["group"] for r in rows)),
        "rejected_records": dict(rejected),
        "selected_by_group": dict(counts),
        "selection": "first eligible track by group/track ID in sorted shard/record order; "
        "distinct scenario IDs; no system outcomes used",
        "config": {
            "duration_s": 6.0,
            "history_steps": 20,
            "dt_s": 0.1,
            "per_group": args.per_group,
            "ego_path": "straight; max lateral <1 m",
            "agent_scope": "selected_target_only",
        },
        "selected": selected,
    }
    (args.output_dir / "womd_manifest.json").write_text(
        json.dumps(manifest, indent=2, allow_nan=False), encoding="utf-8"
    )
    # This ledger is small enough to version and reproduces the cohort audit without raw files.
    (args.output_dir / "womd_candidates.csv").write_bytes(
        (args.log_dir / "exp07_candidates.csv").read_bytes() if rows else b"clip,group,selected\n"
    )
    print(json.dumps({k: v for k, v in manifest.items() if k != "selected"}, indent=2))
    if any(counts[g] < 3 for g in ("cut_in", "control")):
        raise RuntimeError("Fewer than 3 clips in a cohort; inspect ledger, do not relax silently")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", type=Path, default=DEFAULT_RAW_DIR)
    parser.add_argument("--per-group", type=int, default=5)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "tests/fixtures")
    parser.add_argument("--log-dir", type=Path, default=ROOT / "outputs/logs")
    main(parser.parse_args())

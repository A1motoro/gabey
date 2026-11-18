"""Coarse, reproducible cut-in comparison; no tuning or selection of favourable trials."""

import argparse
import csv
import itertools
import json
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pbs.demo.service import simulate  # noqa: E402

GRID = {
    "gap_d": [5, 10, 15, 20, 30, 40],
    "rel_v": [-2, 0, 1, 2, 4, 6, 8],
    "cut_in_t": [0.5, 1.5],
    "lc_T": [3, 4, 5],
}
LABELS = {
    "predict_safety": "Predictive 更好：只有它未碰撞",
    "reactive_safety": "Reactive 更好：只有它未碰撞",
    "predict_control_margin": "均未碰撞，Predictive 在制动/间距指标占优",
    "reactive_control_margin": "均未碰撞，Reactive 在制动/间距指标占优",
    "tie_safe": "均未碰撞，所选指标相当",
    "tradeoff": "均未碰撞，存在权衡",
    "both_collision": "均碰撞，不判定谁更好",
}


def classify(reactive, cv):
    if reactive["collision"] != cv["collision"]:
        return "predict_safety" if reactive["collision"] else "reactive_safety"
    if reactive["collision"]:
        return "both_collision"

    # Pareto comparison, not a weighted score. First-brake time is descriptive only.
    def no_worse(a, b):
        return (
            a["min_dist"] >= b["min_dist"] - 0.1
            and a["brake_count"] <= b["brake_count"]
            and a["brake_duration_s"] <= b["brake_duration_s"] + 0.05
            and a["false_brake"] <= b["false_brake"]
        )

    c, r = no_worse(cv, reactive), no_worse(reactive, cv)
    if c and r:
        return "tie_safe"
    if c:
        return "predict_control_margin"
    if r:
        return "reactive_control_margin"
    return "tradeoff"


def trial(params):
    result = simulate(params)
    summaries = {
        name: {**side["summary"], "first_collision_t": side["first_collision_t"]}
        for name, side in result["systems"].items()
    }
    row = dict(params, category=classify(summaries["reactive"], summaries["cv"]))
    for name, values in summaries.items():
        row.update({f"{name}_{k}": v for k, v in values.items()})
    return row


def format_row(row):
    p = [row[k] for k in GRID]

    def val(name):
        onset = row[f"{name}_brake_onset_t"]
        return (
            f"{'碰撞' if row[f'{name}_collision'] else '未撞'} / "
            f"{row[f'{name}_min_dist']:.2f} m / "
            f"{'—' if onset is None else f'{onset:.2f} s'} / "
            f"{row[f'{name}_brake_count']} 次 / {row[f'{name}_brake_duration_s']:.2f} s"
        )

    return (
        f"| {' | '.join(map(str, p))} | {val('reactive')} | "
        f"{val('cv')} | {LABELS[row['category']]} |"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/logs/cut-in-demo")
    args = parser.parse_args()
    rows = []
    for values in itertools.product(*GRID.values()):
        rows.append(trial(dict(zip(GRID, values, strict=True))))
        if len(rows) % 42 == 0:
            print(f"Evaluated {len(rows)}/252 scenarios", flush=True)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / "comparison.csv").open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    counts = Counter(r["category"] for r in rows)
    summary = {
        "grid": GRID,
        "n": len(rows),
        "counts": dict(counts),
        "fixed": {
            "duration_s": 10,
            "dt": 0.1,
            "prediction_horizon_s": 3,
            "ego_speed_mps": 15,
            "a_brake": 6,
            "r_safe_m": 2.5,
            "policy": "HysteresisPolicy: enter <2, release >3; equality holds",
        },
    }
    (args.output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    lines = [
        "# Cut-in 参数对比测试",
        "",
        "全网格预先固定，共 252 组，无噪声和延迟。",
        "",
        "两边固定：本车 15 m/s，10 s 仿真，dt=0.1 "
        "s，预测 3 s，减速度 6 m/s²，r_safe=2.5"
        " m；迟滞 <2 s 刹车、>3 s 释放。",
        "",
        "比较顺序：只有一方碰撞时，未碰撞者占优；两方都碰撞则都失败，不用碰后轨迹判断优劣。",
        "",
        "均未碰撞时，对最小中心距（容差 0.10 m）、刹车次数、累"
        "计 BRAKE 时间（容差 0.05 s）、误刹标记做 Pa"
        "reto 比较：一方各项不差且至少一项更好才称占优，否则标记"
        "相当或权衡。首次刹车更早本身不计作胜出。",
        "",
        "误刹定义：同一场景不刹车、在相同 10 s 内也不碰撞，则其中任何一次刹车视作误刹。",
        "",
        "网页和 CSV 均在每侧首次接触时截止制动统计；未碰撞者统计至 10 s。"
        "最小中心距在初次接触通常为 2.5 m；这不是碰后距离，也不表示车辆停止的真实动力学。"
        "首次接触时间基于 ≤0.01 s 分段线性扫掠。",
        "",
        "| 分类 | 数量 |",
        "|---|---:|",
    ]
    lines += [f"| {LABELS[k]} | {counts[k]} |" for k in LABELS]
    lines += [
        "",
        "## 代表性条件",
        "",
        "单元格依次为：碰撞 / 截至接触或10s的最小中心距 / 首次刹车 / 次数 / 累计指令时间。",
        "",
        "| gap (m) | relative speed (m/s) | start (s) | 换道时长 (s) | Re"
        "active | Predictive · CV | 判定 |",
        "|---:|---:|---:|---:|---|---|---|",
    ]
    picked = []
    for category in LABELS:
        candidate = next((r for r in rows if r["category"] == category), None)
        if candidate:
            picked.append(candidate)
    required = next(
        r
        for r in rows
        if all(r[k] == v for k, v in dict(gap_d=5, rel_v=1, cut_in_t=0.5, lc_T=3).items())
    )
    if required not in picked:
        picked.insert(0, required)
    lines += [format_row(r) for r in picked]
    absent = [LABELS[k] for k in LABELS if not counts[k]]
    lines += [
        "",
        "未观察到的分类：" + ("；".join(absent) or "无"),
        "",
        "这些结论仅限给定合成场景和简化模型，不代表真实交通总体或一般"
        "安全保证。完整条件逐行见 comparison.csv。",
    ]
    (args.output_dir / "comparison.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    print(f"Report: {args.output_dir / 'comparison.md'}", flush=True)


if __name__ == "__main__":
    main()

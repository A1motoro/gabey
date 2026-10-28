"""基线可视化 — 学习/调试工具，**不是**报告实验。

目的：把 A 的场景（M1）→ P 的 CV/Reactive（M2+Risk）→ C 的 run_sim（M3）→ D 的
evaluate（M4）这条流水线，摊开成能"看一眼就明白"的图。它不产生任何报告结论，
也不替代 exp00_l1_baseline.py / exp00_scenarios.py。

三种模式：

  report   单个场景跑 Reactive 与 CV，出 4 联图：
             ① 俯视路景（压缩 x 轴）  ② 横向偏移 vs 时间（Reactive 的盲区）
             ③ TTC vs 时间           ④ ego 速度 vs 时间（刹车区间）
  anatomy  CV 预测解剖：6 个快照上，把"已知历史 / CV 预测 / 真实未来"画在一起，
           直观解释 ADE/FDE 为什么是这个量级。
  sweep    cut_in 在 (gap_d, rel_v) 平面上的碰撞分布 —— 四边界叠加图的雏形。

用法（仓库根目录，先 conda activate aie1903_ad）：

    python experiments/viz_baseline.py --list
    python experiments/viz_baseline.py --scenario cut_in_danger
    python experiments/viz_baseline.py --scenario "cut_in:gap_d=25,rel_v=3,lc_T=3"
    python experiments/viz_baseline.py --scenario lead_brake_danger --mode report
    python experiments/viz_baseline.py --mode sweep

产物默认写到 outputs/figures/viz-baseline/（该目录按 .gitignore 不入库）。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from pbs.data import CANONICAL, canonical, make_scenario, scenario_label
from pbs.eval.counterfactual import is_benign
from pbs.eval.metrics import evaluate, fde
from pbs.prediction import ConstantVelocityPredictor
from pbs.sim.policy import HysteresisPolicy, TTCThresholdPolicy
from pbs.sim.simulator import run_sim
from pbs.systems import make_system
from pbs.utils import history_until, state_at

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = ROOT / "outputs" / "figures" / "viz-baseline"

R_SAFE = 2.5
HORIZON = 30  # 3.0 s @ dt=0.1
TTC_THRESHOLD = 2.0

COLORS = {"reactive": "#1f77b4", "cv": "#ff7f0e"}
# 两条曲线常常几乎重合，只靠颜色分不出来：用线型再区分一次。
LINESTYLES = {"reactive": dict(ls="-", lw=2.4), "cv": dict(ls="--", lw=1.9)}
LABELS = {"reactive": "Reactive（无预测）", "cv": "CV（2D 匀速预测）"}

# 中文字体：Windows 上 YaHei/SimHei 都在；缺失时退化为 DejaVu，只影响字形不影响数据。
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


# --------------------------------------------------------------------------------------
# 场景与仿真
# --------------------------------------------------------------------------------------
def build_scenario(spec: str):
    """接受 CANONICAL 名字，或 "kind:key=value,key=value" 形式的自定义参数。"""
    if spec in CANONICAL:
        return canonical(spec)
    if ":" not in spec:
        raise SystemExit(
            f"无法解析场景 {spec!r}。用 --list 看可用名字，"
            '或写成 "cut_in:gap_d=25,rel_v=3,lc_T=3"。'
        )
    kind, raw = spec.split(":", 1)
    params = {}
    for token in filter(None, (chunk.strip() for chunk in raw.split(","))):
        if "=" not in token:
            raise SystemExit(f"参数片段 {token!r} 缺少 '='，应为 key=value。")
        key, value = token.split("=", 1)
        try:
            params[key.strip()] = float(value)
        except ValueError:
            raise SystemExit(f"参数 {key!r} 的值 {value!r} 不是数字。") from None
    return make_scenario(kind.strip(), **params)


def build_policy(name: str):
    if name == "hysteresis":
        return HysteresisPolicy(t_on=TTC_THRESHOLD, t_off=3.0)
    return TTCThresholdPolicy(ttc_threshold=TTC_THRESHOLD)


def run_systems(scenario, policy_name: str = "threshold"):
    """返回 (benign, {system: (log, metrics)})。"""
    duration = float(scenario.params.get("duration", 15.0))
    benign = is_benign(scenario, duration=duration, horizon=HORIZON, r_safe=R_SAFE)
    results = {}
    for name in ("reactive", "cv"):
        # 每个系统一个独立 policy 实例：滞回是有状态的，不能跨系统复用。
        log = run_sim(
            scenario,
            make_system(name),
            build_policy(policy_name),
            duration=duration,
            horizon=HORIZON,
            r_safe=R_SAFE,
        )
        results[name] = (log, evaluate(log, benign))
    return benign, results


# --------------------------------------------------------------------------------------
# 小工具
# --------------------------------------------------------------------------------------
def brake_spans(log):
    """把 action 序列压成 (start, end) 的刹车区间，用于背景着色。"""
    t = log.t[: len(log.action)]
    if not len(t):
        return []
    step = float(log.t[1] - log.t[0])
    spans, start = [], None
    for ti, action in zip(t, log.action, strict=True):
        if action == "brake" and start is None:
            start = ti
        elif action != "brake" and start is not None:
            spans.append((start, ti))
            start = None
    if start is not None:
        spans.append((start, float(t[-1]) + step))
    return spans


def per_step_distance(log, agent):
    """每一仿真步 ego 中心到 agent 中心的欧氏距离。"""
    return np.array(
        [
            float(
                np.linalg.norm(np.array([log.ego_x[k], log.ego_y[k]]) - state_at(agent, log.t[k]))
            )
            for k in range(len(log.t))
        ]
    )


def first_lane_entry(agent, lane_width):
    """agent 第一次进入 |y| <= lane_width/2 的时刻；从未进入则 None。"""
    inside = np.flatnonzero(np.abs(agent.y) <= lane_width / 2)
    return None if not len(inside) else float(agent.t[inside[0]])


def maneuver_window(scenario):
    """返回 (start, end, 说明)：该场景里"正在做动作"的时间窗，用于背景高亮。

    这是看懂 lead time 的关键：只有当 TTC 跌破阈值的时刻**落在**这个窗口里，
    CV 的横向外推才可能比 Reactive 更早发现问题。
    """
    params = scenario.params
    if scenario.kind == "cut_in" and "cut_in_t" in params and "lc_T" in params:
        start = float(params["cut_in_t"])
        return start, start + float(params["lc_T"]), "换道进行中"
    if scenario.kind == "lead_brake" and "t_brake" in params:
        return float(params["t_brake"]), float(params.get("duration", 15.0)), "前车减速中"
    return None


def _lead_text(onsets) -> str:
    """把两次刹车时刻的差写成一句人话（含负值情况）。"""
    if onsets["reactive"] is None or onsets["cv"] is None:
        return "无提前量可测"
    delta = onsets["reactive"] - onsets["cv"]
    if abs(delta) < 1e-9:
        return "两者同时刹车"
    return f"CV 早 {delta:.2f}s" if delta > 0 else f"CV 晚 {-delta:.2f}s"


# --------------------------------------------------------------------------------------
# 模式 1：单场景 4 联图
# --------------------------------------------------------------------------------------
def plot_report(scenario, benign, results, policy_name, outdir: Path) -> Path:
    agent = scenario.agents[0]
    lane = scenario.lane_width
    entry_t = first_lane_entry(agent, lane)

    fig = plt.figure(figsize=(14.5, 12.5))
    grid = fig.add_gridspec(4, 1, height_ratios=[1.55, 1.0, 1.0, 0.85], hspace=0.55)
    ax_map = fig.add_subplot(grid[0])
    ax_lat = fig.add_subplot(grid[1])
    ax_ttc = fig.add_subplot(grid[2], sharex=ax_lat)
    ax_v = fig.add_subplot(grid[3], sharex=ax_lat)

    # ---- ① 俯视路景 ---------------------------------------------------------------
    ax_map.axhspan(-lane / 2, lane / 2, color="#e8f4e5", zorder=0)
    for edge in (-lane / 2, lane / 2):
        ax_map.axhline(edge, color="#7a9a70", lw=1.0, ls="--", zorder=1)

    ax_map.plot(agent.x, agent.y, color="black", lw=1.7, label="agent 真实轨迹", zorder=3)
    stride = max(1, len(agent.t) // 15)
    ax_map.plot(
        agent.x[::stride], agent.y[::stride], "o", color="black", ms=3.5, alpha=0.55, zorder=3
    )

    for name, (log, _) in results.items():
        ax_map.plot(
            log.ego_x,
            log.ego_y,
            color=COLORS[name],
            alpha=0.9,
            label=f"ego（{name}）",
            **LINESTYLES[name],
        )

    # agent 驶入本车道的横截面：Reactive 从这一刻起才"看得见"
    if entry_t is not None:
        k = int(np.argmin(np.abs(agent.t - entry_t)))
        ax_map.plot(
            [agent.x[k]],
            [agent.y[k]],
            marker="v",
            ms=11,
            color="#2ca02c",
            mec="white",
            mew=0.8,
            zorder=5,
            label=f"agent 进入本车道（t={entry_t:.2f}s）",
        )

    # 各系统首次刹车的位置
    for name, (log, metrics) in results.items():
        onset = metrics["brake_onset_t"]
        if onset is not None:
            k = int(np.argmin(np.abs(log.t - onset)))
            ax_map.axvline(
                log.ego_x[k],
                color=COLORS[name],
                ls=":",
                lw=1.6,
                label=f"{name} 首次刹车（t={onset:.1f}s）",
            )

    # 最小中心距：画一条水平括线
    worst = min(results.items(), key=lambda item: item[1][0].min_dist)
    worst_log = worst[1][0]
    dist = per_step_distance(worst_log, agent)
    k_md = int(np.argmin(dist))
    ax_map.annotate(
        "",
        xy=(worst_log.ego_x[k_md], 0),
        xytext=(worst_log.ego_x[k_md] + max(dist[k_md], 0.2), 0),
        arrowprops=dict(arrowstyle="<->", color="#d62728", lw=1.6),
    )
    ax_map.text(
        worst_log.ego_x[k_md] + dist[k_md] / 2,
        0.55,
        f"最小中心距 {worst_log.min_dist:.2f} m",
        color="#d62728",
        ha="center",
        fontsize=9.5,
    )

    ax_map.set(
        xlim=(0, float(max(agent.x.max(), worst_log.ego_x.max())) * 1.02),
        ylim=(-lane - 1.0, lane + 1.0),
        ylabel="横向 y [m]",
    )
    ax_map.set_title("① 俯视路景（x 轴压缩显示，非等比）", loc="left", fontsize=11)
    ax_map.legend(loc="lower left", fontsize=8.5, framealpha=0.92, ncol=1)
    ax_map.grid(alpha=0.22)

    # ---- ② 横向偏移 vs 时间 -------------------------------------------------------
    window = maneuver_window(scenario)
    ax_lat.axhspan(-lane / 2, lane / 2, color="#e8f4e5", zorder=0)
    if window is not None:
        ax_lat.axvspan(
            window[0],
            window[1],
            color="#ffe9b8",
            alpha=0.6,
            lw=0,
            zorder=0.1,
            label=f"{window[2]}（{window[0]:.1f}–{window[1]:.1f}s）",
        )
    ax_lat.axhline(lane / 2, color="#7a9a70", ls="--", lw=1.1)
    ax_lat.axhline(-lane / 2, color="#7a9a70", ls="--", lw=1.1)
    ax_lat.plot(agent.t, agent.y, color="black", lw=1.9, label="agent 横向位置")
    if entry_t is not None:
        ax_lat.axvline(entry_t, color="#2ca02c", ls="-.", lw=1.7)
        ax_lat.text(
            entry_t,
            lane * 0.78,
            f"  t={entry_t:.2f}s 起进入本车道\n  （Reactive 从此刻才看见）",
            color="#2ca02c",
            fontsize=9,
            va="top",
            bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none", alpha=0.85),
        )
    ax_lat.text(
        0.012,
        0.06,
        "Reactive 只能看见 ±1.75 m 这一带（浅绿色）",
        transform=ax_lat.transAxes,
        fontsize=9,
        color="#4d6b45",
        va="bottom",
        bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none", alpha=0.85),
    )
    ax_lat.set_ylabel("agent 横向 y [m]")
    # crossing 场景里 agent 会一直横向走下去（-20 m），不裁剪的话 ±1.75 m 的车道带会被压没。
    y_hi = min(1.15 * max(float(np.abs(agent.y).max()), lane), 1.45 * lane)
    ax_lat.set_ylim(-1.45 * lane, y_hi)
    ax_lat.set_title("② 横向偏移：Reactive 的盲区在哪", loc="left")
    ax_lat.legend(loc="upper right", fontsize=9)
    ax_lat.grid(alpha=0.22)

    # ---- ③ TTC vs 时间 ------------------------------------------------------------
    if window is not None:
        ax_ttc.axvspan(
            window[0], window[1], color="#ffe9b8", alpha=0.6, lw=0, zorder=0.1, label=f"{window[2]}"
        )
    for name, (log, metrics) in results.items():
        ttc = np.where(np.isfinite(log.ttc), log.ttc, np.nan)
        ax_ttc.plot(log.t, ttc, color=COLORS[name], label=LABELS[name], **LINESTYLES[name])
        onset = metrics["brake_onset_t"]
        if onset is not None:
            ax_ttc.plot([onset], [TTC_THRESHOLD], marker="v", ms=9, color=COLORS[name], zorder=5)
            ax_ttc.annotate(
                f"{onset:.1f}s",
                (onset, TTC_THRESHOLD),
                textcoords="offset points",
                xytext=(0, -16),
                ha="center",
                fontsize=8.5,
                color=COLORS[name],
            )
    ax_ttc.axhline(
        TTC_THRESHOLD, color="#999999", ls=":", lw=1.3, label=f"刹车阈值 {TTC_THRESHOLD:.1f}s"
    )
    # 本场景为什么没有 lead time：阈值触发时动作窗口已经结束
    if window is not None:
        first_onset = min(
            (m["brake_onset_t"] for _, m in results.values() if m["brake_onset_t"] is not None),
            default=None,
        )
        if first_onset is not None and first_onset > window[1]:
            ax_ttc.text(
                window[1] + 0.2,
                5.15,
                "TTC 跌破阈值时动作窗口已经结束\n→ 此刻 CV 的预测已精确，提前量为 0",
                fontsize=8.8,
                color="#8a6d1f",
                bbox=dict(boxstyle="round,pad=0.3", fc="#fffbe8", ec="#e8d9a8", alpha=0.95),
            )
    ax_ttc.set_ylabel("TTC [s]")
    ax_ttc.set_ylim(0, 6)
    ax_ttc.set_title("③ TTC：低于阈值就刹车（断线 = 无冲突 inf）", loc="left")
    ax_ttc.legend(loc="upper right", fontsize=9)
    ax_ttc.grid(alpha=0.22)

    # ---- ④ ego 速度 ---------------------------------------------------------------
    for name, (log, _) in results.items():
        for start, end in brake_spans(log):
            ax_v.axvspan(start, end, color=COLORS[name], alpha=0.14, lw=0)
        ax_v.plot(
            log.t, log.ego_v, color=COLORS[name], label=f"ego 速度（{name}）", **LINESTYLES[name]
        )
    ax_v.set_xlabel("时间 t [s]")
    ax_v.set_ylabel("ego 速度 [m/s]")
    ax_v.set_title("④ 决策后果：速度与刹车区间", loc="left")
    ax_v.legend(loc="upper right", fontsize=9)
    ax_v.grid(alpha=0.22)
    if any(len(brake_spans(log)) > 1 for log, _ in results.values()):
        ax_v.text(
            0.012,
            0.06,
            "注意：单阈值策略会在 TTC 回升时松开刹车 → 多段刹车。\n"
            "C 的 HysteresisPolicy(t_on=2, t_off=3) 就是为消除这种抖动设计的",
            transform=ax_v.transAxes,
            fontsize=8.5,
            color="#555555",
            va="bottom",
            bbox=dict(boxstyle="round,pad=0.3", fc="#f7f7f7", ec="#dddddd"),
        )

    onsets = {name: metrics["brake_onset_t"] for name, (_, metrics) in results.items()}
    kind_text = "benign（不刹车也不会撞）" if benign else "dangerous（不刹车会撞）"
    fig.suptitle(
        f"{scenario_label(scenario)} — {kind_text}\n"
        f"Reactive 首次刹车 {_fmt(onsets['reactive'])} · CV 首次刹车 {_fmt(onsets['cv'])} · "
        f"{_lead_text(onsets)} · 策略={policy_name}",
        fontsize=12.5,
    )
    path = outdir / f"report_{_slug(scenario)}.png"
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    return path


def _fmt(value):
    return "—" if value is None else f"{value:.2f}s"


def _slug(scenario) -> str:
    """短文件名：kind + 两个起决定作用的参数，避免一长串数字。"""
    params = scenario.params
    if scenario.kind == "cut_in":
        return (
            f"cut_in_g{params.get('gap_d', 0):g}_r{params.get('rel_v', 0):g}"
            f"_T{params.get('lc_T', 0):g}"
        )
    if scenario.kind == "lead_brake":
        return f"lead_brake_g{params.get('init_gap', 0):g}_d{params.get('decel', 0):g}"
    if scenario.kind == "crossing":
        return f"crossing_g{params.get('gap_d', 0):g}_a{params.get('arrival_t', 0):g}"
    return scenario.kind


# --------------------------------------------------------------------------------------
# 模式 2：CV 预测解剖
# --------------------------------------------------------------------------------------
def plot_anatomy(scenario, outdir: Path, snapshots=None) -> Path:
    """在若干快照上叠加：已知历史 / CV 预测 / 真实未来。

    这是理解 ADE/FDE 最直接的一张图：CV 把观测到的横向速度当常数外推，而 quintic
    换道的横向速度是从 0 升到峰值再降回 0 的，所以预测会系统性地冲过头。
    """
    agent = scenario.agents[0]
    dt = scenario.dt
    duration = float(scenario.params.get("duration", 15.0))
    predictor = ConstantVelocityPredictor()

    if snapshots is None:
        window = maneuver_window(scenario)
        lo, hi = (window[0], window[1]) if window is not None else (0.5, 4.0)
        # 覆盖整个动作窗口，两端各留一点余量，取 6 个等距快照
        snapshots = np.linspace(lo + 0.3, min(hi + 0.3, duration - 0.2), 6)
    snapshots = [float(t) for t in snapshots if 0.0 < t < duration]

    fig, axes = plt.subplots(2, 3, figsize=(15.5, 7.0))
    ego_v = float(scenario.ego_start[3])
    fde_values = []
    for ax, t_now in zip(axes.ravel(), snapshots, strict=False):
        observed = history_until(agent, t_now)
        forecast = predictor.predict(observed, HORIZON, dt)
        f_times = observed.t[-1] + np.arange(1, HORIZON + 1) * dt
        truth = np.column_stack(
            (np.interp(f_times, agent.t, agent.x), np.interp(f_times, agent.t, agent.y))
        )

        keep = observed.t >= t_now - 1.0
        ax.plot(
            observed.x[keep],
            observed.y[keep],
            "-o",
            color="#1f77b4",
            ms=3.2,
            lw=1.7,
            label="已知历史（过去 1.0s）",
        )
        ax.plot(
            forecast[:, 0],
            forecast[:, 1],
            "--",
            color="#ff7f0e",
            lw=2.0,
            label="CV 预测（未来 3s）",
        )
        ax.plot(truth[:, 0], truth[:, 1], "-", color="black", lw=2.0, label="真实未来")

        ego_xy = np.array([ego_v * t_now, 0.0])
        ax.plot([ego_xy[0]], [ego_xy[1]], "s", color="#555555", ms=9, label="ego（未刹车）")

        err_lat = abs(forecast[-1, 1] - truth[-1, 1])
        err_fde = fde(forecast, truth)
        fde_values.append(err_fde)
        ax.set_title(
            f"t = {t_now:.1f} s   |  FDE {err_fde:.2f} m  |  横向终点误差 {err_lat:.2f} m",
            fontsize=10,
        )
        ax.annotate(
            f"预测横向终点 y={forecast[-1, 1]:+.2f} m\n真实横向终点 y={truth[-1, 1]:+.2f} m",
            xy=(0.02, 0.03),
            xycoords="axes fraction",
            fontsize=8.5,
            color="#333333",
            bbox=dict(boxstyle="round,pad=0.28", fc="white", ec="#cccccc", alpha=0.9),
        )
        x_lo = min(ego_xy[0], observed.x[keep].min()) - 4.0
        x_hi = max(forecast[:, 0].max(), truth[:, 0].max()) + 4.0
        ax.set(
            xlim=(x_lo, x_hi),
            ylim=(-1.0, scenario.lane_width + 1.4),
            xlabel="纵向 x [m]",
            ylabel="横向 y [m]",
        )
        ax.grid(alpha=0.22)

    handles, labels_ = axes.ravel()[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels_,
        loc="lower center",
        ncol=4,
        fontsize=9.5,
        frameon=False,
        bbox_to_anchor=(0.5, -0.015),
    )
    worst_fde = max(fde_values) if fde_values else 0.0
    note = (
        "本场景是匀速直线运动 → CV 在数学上完全精确（FDE≈0）。CV 只在存在加速度时才出错。"
        if worst_fde < 0.05
        else "CV 把观测到的 2D 速度当常数外推；只要真实运动带加速度，就会系统性地冲过头或跟不上。"
    )
    fig.suptitle(
        f"CV 预测解剖 — {scenario_label(scenario)}\n{note}"
        "\n注意：横纵坐标比例不同（横向被放大），看的是轨迹形状而不是视觉距离",
        fontsize=12.5,
    )
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    path = outdir / f"anatomy_{_slug(scenario)}.png"
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    return path


# --------------------------------------------------------------------------------------
# 模式 3：(gap_d, rel_v) 平面扫描
# --------------------------------------------------------------------------------------
def plot_sweep(outdir: Path, gap_ds, rel_vs, lc_T=3.0, cut_in_t=0.5):
    """cut_in 在 (gap_d, rel_v) 平面上的碰撞分布 —— exp03 四边界图的雏形。"""
    rows = []
    for gap_d in gap_ds:
        for rel_v in rel_vs:
            try:
                scenario = make_scenario(
                    "cut_in", gap_d=gap_d, rel_v=rel_v, lc_T=lc_T, cut_in_t=cut_in_t
                )
            except ValueError as err:
                rows.append(dict(gap_d=gap_d, rel_v=rel_v, status="rejected", reason=str(err)))
                continue
            benign, results = run_systems(scenario)
            rows.append(
                dict(
                    gap_d=gap_d,
                    rel_v=rel_v,
                    status="ok",
                    benign=benign,
                    reactive_collision=results["reactive"][0].collision,
                    cv_collision=results["cv"][0].collision,
                )
            )

    ok = [row for row in rows if row["status"] == "ok"]
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.0), sharex=True, sharey=True)
    for ax, key, title in zip(
        axes,
        ("reactive_collision", "cv_collision", None),
        ("Reactive", "CV", "对照：prediction 的收益"),
        strict=True,
    ):
        for row in ok:
            x, y = row["gap_d"], row["rel_v"]
            color = "#9e9e9e" if row["benign"] else "#c62828"
            if key is None:
                crashed = row["reactive_collision"] and row["cv_collision"]
                only_cv = row["cv_collision"] and not row["reactive_collision"]
                only_re = row["reactive_collision"] and not row["cv_collision"]
                marker = "X" if (crashed or only_cv or only_re) else "o"
                if crashed:
                    color = "#6a1b9a"
                elif only_cv:
                    color = "#ef6c00"
                elif only_re:
                    color = "#1565c0"
                elif not (crashed or only_cv or only_re):
                    color = "#9e9e9e"  # 都不撞：不区分 benign，第三张图只回答"谁撞了"
            else:
                marker = "X" if row[key] else "o"
            ax.scatter(
                [x],
                [y],
                marker=marker,
                s=140 if marker == "X" else 90,
                color=color,
                alpha=0.9,
                edgecolors="white",
                linewidths=0.8,
            )
        ax.set_title(f"{title}", loc="left", fontsize=11)
        ax.set_xlabel("初始纵向间距 gap_d [m]")
        ax.grid(alpha=0.25)
    axes[0].set_ylabel("rel_v = ego 速度 − 切入车速度 [m/s]")

    handles_system = [
        plt.Line2D([], [], marker="X", ls="", color="#c62828", ms=11, label="碰撞"),
        plt.Line2D(
            [], [], marker="o", ls="", color="#c62828", ms=9, label="不碰撞（该场景 dangerous）"
        ),
        plt.Line2D(
            [], [], marker="o", ls="", color="#9e9e9e", ms=9, label="benign（本来就不该刹）"
        ),
    ]
    handles_gain = [
        plt.Line2D([], [], marker="X", ls="", color="#6a1b9a", ms=11, label="两者都撞"),
        plt.Line2D([], [], marker="X", ls="", color="#ef6c00", ms=11, label="只有 CV 撞"),
        plt.Line2D([], [], marker="X", ls="", color="#1565c0", ms=11, label="只有 Reactive 撞"),
        plt.Line2D([], [], marker="o", ls="", color="#9e9e9e", ms=9, label="都不撞"),
    ]
    fig.legend(
        handles=handles_system,
        loc="lower left",
        bbox_to_anchor=(0.03, 0.0),
        ncol=3,
        fontsize=8.8,
        frameon=False,
        title="左图 / 中图：单系统结果",
        title_fontsize=9,
    )
    fig.legend(
        handles=handles_gain,
        loc="lower right",
        bbox_to_anchor=(0.97, 0.0),
        ncol=2,
        fontsize=8.8,
        frameon=False,
        title="右图：两系统对比",
        title_fontsize=9,
    )

    n_re = sum(row["reactive_collision"] for row in ok)
    n_cv = sum(row["cv_collision"] for row in ok)
    fig.suptitle(
        f"cut_in 参数扫描：{len(ok)} 组有效参数（lc_T={lc_T:g}s, cut_in_t={cut_in_t:g}s, "
        f"阈值 {TTC_THRESHOLD:g}s）\n"
        f"碰撞数：Reactive {n_re} / {len(ok)}，CV {n_cv} / {len(ok)}",
        fontsize=12,
    )
    fig.tight_layout(rect=(0, 0.14, 1, 1))
    path = outdir / "sweep_cut_in.png"
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    return path, ok


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------
def print_list():
    print("可用命名场景（docs/spec.md §1.5 要求每类都有 danger 与 benign）：\n")
    for name, spec in CANONICAL.items():
        params = ", ".join(f"{k}={v}" for k, v in spec.items() if k != "kind")
        print(f"  {name:30s} {spec['kind']:11s} {params}")
    print('\n也可以用自定义参数：--scenario "cut_in:gap_d=25,rel_v=3,lc_T=3"')


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--scenario", default="cut_in_danger", help="CANONICAL 名字或 kind:key=value,..."
    )
    parser.add_argument("--mode", default="all", choices=("report", "anatomy", "sweep", "all"))
    parser.add_argument("--policy", default="threshold", choices=("threshold", "hysteresis"))
    parser.add_argument("--outdir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--list", action="store_true", help="列出命名场景后退出")
    parser.add_argument("--zoom", type=float, default=1.0, help="保留参数；当前版本未使用")
    args = parser.parse_args()

    if args.list:
        print_list()
        return

    args.outdir.mkdir(parents=True, exist_ok=True)

    if args.mode in ("report", "anatomy", "all"):
        scenario = build_scenario(args.scenario)
        benign, results = run_systems(scenario, args.policy)
        duration = float(scenario.params.get("duration", 15.0))

        print(f"场景      : {scenario_label(scenario)}")
        print(f"判定      : {'benign（不刹车也不撞）' if benign else 'dangerous（不刹车会撞）'}")
        print(f"时长/步长 : {duration:g}s / {scenario.dt:g}s   预测时域 {HORIZON * scenario.dt:g}s")
        print(f"lane_width: {scenario.lane_width:g} m   r_safe: {R_SAFE:g} m\n")
        print(f"{'系统':<10}{'首次刹车':>10}{'碰撞':>8}{'最小距离':>10}{'误刹':>8}")
        for name, (log, metrics) in results.items():
            onset = metrics["brake_onset_t"]
            print(
                f"{name:<10}{('—' if onset is None else f'{onset:.1f}s'):>10}"
                f"{str(log.collision):>8}{log.min_dist:>9.2f}m{str(metrics['false_brake']):>8}"
            )
        onset_re = results["reactive"][1]["brake_onset_t"]
        onset_cv = results["cv"][1]["brake_onset_t"]
        if onset_re is not None and onset_cv is not None:
            delta = onset_re - onset_cv
            if delta > 0:
                print(f"\nlead time : CV 比 Reactive 早 {delta:.2f}s 察觉")
            elif delta < 0:
                print(f"\nlead time : CV 比 Reactive 晚 {-delta:.2f}s（本场景预测没带来提前量）")
            else:
                print("\nlead time : 两者同时刹车")
        window = maneuver_window(scenario)
        if window is not None:
            print(f"动作窗口  : {window[2]} {window[0]:.1f}–{window[1]:.1f}s")
        print()

        produced = []
        if args.mode in ("report", "all"):
            produced.append(plot_report(scenario, benign, results, args.policy, args.outdir))
        if args.mode in ("anatomy", "all"):
            produced.append(plot_anatomy(scenario, args.outdir))

        summary = {
            "scenario": scenario_label(scenario),
            "params": scenario.params,
            "benign": benign,
            "policy": args.policy,
            "horizon_s": HORIZON * scenario.dt,
            "ttc_threshold_s": TTC_THRESHOLD,
            "r_safe_m": R_SAFE,
            "systems": {name: metrics for name, (_, metrics) in results.items()},
        }
        (args.outdir / f"summary_{_slug(scenario)}.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        for path in produced:
            print(f"图: {path}")

    if args.mode in ("sweep", "all"):
        print("扫描 cut_in 参数平面（约 10-20 秒）…")
        path, ok = plot_sweep(
            args.outdir,
            gap_ds=[10.0, 15.0, 20.0, 25.0, 30.0, 40.0, 50.0, 60.0],
            rel_vs=[0.0, 1.0, 2.0, 3.0, 4.0, 6.0],
        )
        print(f"图: {path}  （{len(ok)} 组有效组合）")


if __name__ == "__main__":
    main()

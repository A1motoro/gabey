"""Run C's 15 s closed-loop comparison on A's CA cut-ins using P's risk assessors."""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pbs.data.scenarios import DT, DURATION, ca_scenario_set  # noqa: E402
from pbs.eval.metrics import brake_activity  # noqa: E402
from pbs.risk import NullAssessor  # noqa: E402
from pbs.sim.collision import VEHICLE_LENGTH, VEHICLE_WIDTH  # noqa: E402
from pbs.sim.policy import HysteresisPolicy, NeverBrakePolicy  # noqa: E402
from pbs.sim.simulator import run_sim  # noqa: E402
from pbs.systems import make_system  # noqa: E402

HORIZON_S = 4.0
SYSTEM_NAMES = ("never_brake", "reactive", "cv", "ca", "ctrv", "oracle")
EXAMPLE = (20.0, 2.0, -2.0)


def write_rows(path, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def plot_example(path, logs, agent):
    fig, (speed, risk) = plt.subplots(2, 1, sharex=True, figsize=(9, 6), layout="constrained")
    speed.plot(agent.t, agent.v, color="0.6", linewidth=3, label="Cut-in vehicle (truth)")
    for name, log in logs.items():
        (line,) = speed.plot(log.t, log.ego_v, label=name.upper())
        if name == "never_brake":
            if log.first_collision_t is not None:
                t = log.first_collision_t
                speed.plot(t, np.interp(t, log.t, log.ego_v), "x", color=line.get_color(), ms=10)
                line.set_data(log.t[log.t < t], log.ego_v[log.t < t])
            continue
        k = np.flatnonzero(np.asarray(log.action[: log.executed_steps]) == "brake")
        if len(k):
            speed.plot(log.t[k[0]], log.ego_v[k[0]], "o", color=line.get_color(), ms=5)
        risk.step(
            log.t,
            np.where(np.isfinite(log.ttc), log.ttc, np.nan),
            where="post",
            color=line.get_color(),
        )
    risk.axhline(2, color="black", linestyle=":", linewidth=1, label="Brake threshold")
    risk.axhline(3, color="black", linestyle="--", linewidth=1, label="Release threshold")
    speed.set(
        title="Decelerating cut-in  |  initial gap 20 m, closing speed 2 m/s", ylabel="Speed (m/s)"
    )
    speed.legend(fontsize=8, ncol=4)
    speed.set_xlabel("Dot = first brake command     × = first contact", fontsize=9)
    risk.set(xlabel="Time (s)", ylabel="Estimated TTC (s)", ylim=(0, 6), xlim=(0, DURATION))
    risk.legend(fontsize=8)
    for ax in (speed, risk):
        ax.grid(alpha=0.2)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def run(output_dir):
    summaries, frames = [], []
    # A's generator supplies real future samples for every Oracle query, including at t=15.
    scenarios = ca_scenario_set(duration=DURATION + HORIZON_S).scenarios
    for scene in scenarios:
        params = scene.params
        gap, closing, acceleration = (params[k] for k in ("gap_d", "rel_v", "a_cutter"))
        case_id = f"gap{gap:g}_rel{closing:g}_acc{acceleration:+g}"
        logs = {}
        agent = scene.agents[0]
        for name in SYSTEM_NAMES:
            reference = name == "never_brake"
            assessor = NullAssessor() if reference else make_system(name)
            policy = NeverBrakePolicy() if reference else HysteresisPolicy(2, 3, a_brake=6)
            log = run_sim(
                scene,
                assessor,
                policy,
                duration=DURATION,
                horizon=round(HORIZON_S / DT),
                latency=0.0,
                collision_dt=0.01,
            )
            logs[name] = log
            benign = not logs["never_brake"].collision
            stop = DURATION if log.first_collision_t is None else log.first_collision_t
            episodes = [e for e in brake_activity(log)["episodes"] if e["start_t"] < stop]
            summaries.append(
                dict(
                    case_id=case_id,
                    system=name,
                    gap_d_m=gap,
                    rel_v_mps=closing,
                    a_cutter_mps2=acceleration,
                    cut_in_t_s=params["cut_in_t"],
                    lc_T_s=params["lc_T"],
                    accel_T_s=params["accel_T"],
                    ego_start_mps=params["v_ego"],
                    duration_s=DURATION,
                    dt_s=DT,
                    horizon_s=HORIZON_S,
                    latency_s=0,
                    collision_dt_s=0.01,
                    vehicle_length_m=VEHICLE_LENGTH,
                    vehicle_width_m=VEHICLE_WIDTH,
                    t_on_s=None if reference else policy.t_on,
                    t_off_s=None if reference else policy.t_off,
                    a_brake_mps2=None if reference else policy.a_brake,
                    benign=benign,
                    collision=log.collision,
                    first_contact_s=log.first_collision_t,
                    first_brake_s=episodes[0]["start_t"] if episodes else None,
                    brake_count=len(episodes),
                    brake_duration_s=sum(min(e["end_t"], stop) - e["start_t"] for e in episodes),
                    false_brake=benign and bool(episodes),
                    min_center_dist_m=log.min_dist_until_contact,
                )
            )
            for i, t in enumerate(log.t):
                frames.append(
                    dict(
                        case_id=case_id,
                        system=name,
                        time_s=t,
                        ego_x_m=log.ego_x[i],
                        ego_y_m=log.ego_y[i],
                        ego_v_mps=log.ego_v[i],
                        agent_x_m=agent.x[i],
                        agent_y_m=agent.y[i],
                        ttc_s=log.ttc[i],
                        action=log.action[i] if i < log.executed_steps else "",
                        after_contact=t > stop,
                    )
                )
        if (gap, closing, acceleration) == EXAMPLE:
            example_logs, example_agent = logs, agent
    output_dir.mkdir(parents=True, exist_ok=True)
    write_rows(output_dir / "runs.csv", summaries)
    write_rows(output_dir / "timeline.csv", frames)
    plot_example(output_dir / "decelerating_cut_in.png", example_logs, example_agent)
    print(
        f"{len(scenarios)} scenarios, {len(summaries)} runs, "
        f"{len(frames)} state rows in {output_dir}"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/logs/c_l2_closed_loop")
    run(parser.parse_args().output_dir)

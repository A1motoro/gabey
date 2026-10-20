"""C-only runnable demonstration, NOT an A/P integration or safety experiment.

    PYTHONPATH=src python examples/c_smoke.py

The scripted assessor intentionally supplies risk independent of the distant
vehicle. Replace the Scenario and assessor with A/P's implementations for real
experiments. The CSV contains state rows; the last row has executed=False.
"""

import argparse
import csv
from pathlib import Path

import numpy as np

from pbs.risk.assessors import RiskAssessor
from pbs.sim.policy import HysteresisPolicy
from pbs.sim.simulator import run_sim
from pbs.types import RiskEstimate, Scenario, Trajectory


class ScriptedRiskForDemo(RiskAssessor):
    """Test stimulus: brake for a second, then release; not a TTC algorithm."""

    def assess(self, ctx, history):
        return RiskEstimate(ttc=1.0 if ctx.t_now < 1.0 else 4.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/logs/c_smoke.csv"))
    args = parser.parse_args()
    times = np.linspace(-1, 4, 51)
    agent = Trajectory(
        t=times,
        x=np.full_like(times, 100),
        y=np.full_like(times, 20),
        v=np.zeros_like(times),
        heading=np.zeros_like(times),
        agent_id="demo_target",
    )
    scenario = Scenario(
        ego_start=(0, 0, 0, 15),
        ego_path=np.array([[0.0, 0.0], [500.0, 0.0]]),
        agents=[agent],
        kind="c_smoke",
    )
    log = run_sim(
        scenario,
        ScriptedRiskForDemo(),
        HysteresisPolicy(2, 3),
        duration=2,
        horizon=40,
        latency=0.2,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t", "ego_x", "ego_y", "ego_v", "ego_a", "ttc", "action", "executed"])
        for k in range(len(log.t)):
            writer.writerow(
                [
                    log.t[k],
                    log.ego_x[k],
                    log.ego_y[k],
                    log.ego_v[k],
                    log.ego_a[k],
                    log.ttc[k],
                    log.action[k],
                    k < log.executed_steps,
                ]
            )
    print("C smoke demo only; scripted risk is NOT a P prediction result.")
    print(f"rows={len(log.t)}, executed_steps={log.executed_steps}")
    print(f"final x={log.ego_x[-1]:.3f} m, final v={log.ego_v[-1]:.3f} m/s")
    print(f"collision={log.collision}, min_dist={log.min_dist:.3f} m")
    print(f"CSV: {args.output.resolve()}")


if __name__ == "__main__":
    main()

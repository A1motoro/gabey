"""Small, separate two-vehicle fixtures for LiDAR visibility studies.

The existing scenario kinds/grids are unchanged. The blocker stays in the
adjacent lane behind the cutter; ego's own lane starts empty.
"""

import numpy as np

from pbs.data.scenarios import make_scenario
from pbs.types import Scenario, Trajectory


def occluded_cut_in(*, duration: float = 8.0, dt: float = 0.1) -> Scenario:
    scenario = make_scenario(
        "cut_in",
        gap_d=24.0,
        rel_v=3.0,
        cut_in_t=0.5,
        lc_T=3.0,
        duration=duration,
        dt=dt,
    )
    times = scenario.agents[0].t.copy()
    blocker = Trajectory(
        t=times,
        x=18.0 + 12.0 * times,
        y=np.full_like(times, 2.65),
        v=np.full_like(times, 12.0),
        heading=np.zeros_like(times),
        agent_id="blocker",
    )
    scenario.agents.append(blocker)
    scenario.kind = "occluded_cut_in"
    # The blocker is offset toward the inner side of the adjacent lane. Its
    # inner edge is y=1.40 m, outside ego's y=1.25 m footprint: no front blocker.
    scenario.params = {**scenario.params, "blocker_gap": 18.0, "blocker_y": 2.65}
    return scenario


def adjacent_partial_occlusion(*, duration: float = 8.0, dt: float = 0.1) -> Scenario:
    """A straight target (true vy=0) with changing partial occlusion.

    This fixture examines measured centre drift. Whether it causes a false brake
    depends on the sensor/tracker/predictor configuration, not the fixture name.
    """
    scenario = occluded_cut_in(duration=duration, dt=dt)
    times = scenario.agents[0].t.copy()
    scenario.agents[0] = Trajectory(
        t=times,
        x=24.0 + 12.0 * times,
        y=np.full_like(times, 3.5),
        v=np.full_like(times, 12.0),
        heading=np.zeros_like(times),
        agent_id="adjacent",
    )
    # A slower outer-side blocker sweeps its angular shadow across the target.
    scenario.agents[1].x = 12.0 + 10.0 * times
    scenario.agents[1].y = np.full_like(times, 4.0)
    scenario.agents[1].v = np.full_like(times, 10.0)
    scenario.kind = "adjacent_partial_occlusion"
    scenario.params = {"duration": duration, "dt": dt, "target_vy": 0.0}
    return scenario

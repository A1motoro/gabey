import numpy as np

from pbs.sim.collision import in_collision
from pbs.sim.simulator import ego_rollout


def test_in_collision():
    assert in_collision([0, 0], [1, 0], r_safe=2.5)
    assert not in_collision([0, 0], [3, 0], r_safe=2.5)


def test_ego_rollout_constant_speed():
    path = np.array([[0.0, 0.0], [100.0, 0.0]])
    fut = ego_rollout(path, s=0.0, v=10.0, horizon=5, dt=0.1)
    assert fut.shape == (5, 2)
    assert np.allclose(fut[:, 0], [1, 2, 3, 4, 5])

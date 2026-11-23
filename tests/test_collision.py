import numpy as np
import pytest

from pbs.risk.ttc import assess_risk
from pbs.sim.collision import in_collision, swept_contact_fraction
from pbs.sim.simulator import ego_rollout


def test_in_collision():
    assert in_collision([0, 0], [1, 0], r_safe=2.5)
    assert not in_collision([0, 0], [3, 0], r_safe=2.5)


def test_ego_rollout_constant_speed():
    path = np.array([[0.0, 0.0], [100.0, 0.0]])
    fut = ego_rollout(path, s=0.0, v=10.0, horizon=5, dt=0.1)
    assert fut.shape == (5, 2)
    assert np.allclose(fut[:, 0], [1, 2, 3, 4, 5])


@pytest.mark.parametrize(
    "offset,heading,expected",
    [
        ((0, 3.5), 0, False),
        ((0, 2.5), 0, False),
        ((0, 2.49), 0, True),
        ((5, 0), 0, False),
        ((4.99, 0), 0, True),
        ((4.9, 2.4), 0, True),
        ((4, 0), np.pi / 2, False),
        ((0, 3.5), np.pi / 2, True),
    ],
)
def test_vehicle_footprints_match_prediction(offset, heading, expected):
    assert in_collision([0, 0], offset, agent_heading=heading) == expected
    risk = assess_risk([[0, 0]], [offset], 0.1, ego_heading=0, agent_heading=heading)
    assert np.isfinite(risk.ttc) == expected


def test_swept_rectangle_crosses_between_frames_and_excludes_grazing():
    assert swept_contact_fraction([0, 0], [20, 0], [10, 2.4], [10, 2.4]) == pytest.approx(0.25)
    assert swept_contact_fraction([0, 0], [20, 0], [10, 2.5], [10, 2.5]) is None
    assert swept_contact_fraction(
        [0, 0], [20, 0], [10, 0], [10, 0], agent_heading=np.pi / 2
    ) == pytest.approx((10 - 3.75) / 20)


def test_pedestrian_uses_rounded_corners_around_vehicle():
    assert not in_collision([0, 0], [3.6, 2.4], pedestrian=True)
    assert in_collision([0, 0], [3.1, 2.0], pedestrian=True)
    assert swept_contact_fraction(
        [0, 0], [10, 0], [6, 2.0], [6, 2.0], pedestrian=True
    ) == pytest.approx(0.25)


@pytest.mark.parametrize("width,expected", [(2.0, False), (2.5, True)])
def test_width_matches_static_swept_and_predicted_contact(width, expected):
    dims = dict(vehicle_width=width)
    assert in_collision([0, 0], [0, 2.25], **dims) == expected
    entry = swept_contact_fraction([-10, 0], [10, 0], [0, 2.25], [0, 2.25], **dims)
    assert entry == (0.25 if expected else None)
    risk = assess_risk(
        [[10, 0]],
        [[0, 2.25]],
        1.0,
        ego_origin=[-10, 0],
        agent_origin=[0, 2.25],
        swept=True,
        **dims,
    )
    assert risk.ttc == (0.25 if expected else np.inf)


def test_unequal_rotated_footprints_use_both_bodies_axes():
    dims = dict(vehicle_length=8, vehicle_width=2, agent_length=4, agent_width=1)
    # At a right angle the combined half-extents are 4.5 along x and 3 along y.
    for offset, expected in [((4.49, 2.99), True), ((4.5, 0), False), ((0, 3), False)]:
        assert in_collision([0, 0], offset, agent_heading=np.pi / 2, **dims) == expected
    assert swept_contact_fraction(
        [0, 0], [10, 0], [8, 2.99], [8, 2.99], agent_heading=np.pi / 2, **dims
    ) == pytest.approx(0.35)

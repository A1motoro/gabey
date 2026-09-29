import numpy as np
import pytest

from pbs.prediction import StaticPredictor
from pbs.types import Trajectory


def test_static_predictor_shape():
    t = np.arange(5) * 0.1
    h = Trajectory(t, t, np.zeros(5), np.ones(5), np.zeros(5))
    out = StaticPredictor().predict(h, horizon=20, dt=0.1)
    assert out.shape == (20, 2)


@pytest.mark.skip(reason="TODO(P): CV straight line is exact")
def test_cv_predictor_straight_line(): ...


@pytest.mark.skip(reason="TODO(P): CTRV with yaw_rate≈0 equals CV")
def test_ctrv_degenerates_to_cv(): ...


@pytest.mark.skip(reason="TODO(P): Oracle returns true future")
def test_oracle_exact(): ...

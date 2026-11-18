"""Episode statistics distinguish repeated braking from prolonged braking."""

import numpy as np
import pytest

from pbs.eval.metrics import brake_activity
from pbs.types import SimLog


def make_log(actions, terminal="brake"):
    n = len(actions) + 1
    return SimLog(
        t=np.arange(n, dtype=float) * 0.1,
        ego_x=np.zeros(n),
        ego_y=np.zeros(n),
        ego_v=np.zeros(n),
        ego_a=np.zeros(n),
        action=[*actions, terminal],
        ttc=np.full(n, np.inf),
        min_dist=10.0,
        collision=False,
        executed_steps=len(actions),
    )


def test_continuous_brake_is_one_event_not_one_per_frame():
    stats = brake_activity(make_log(["cruise", "brake", "brake", "brake", "cruise"]))
    assert stats["total_count"] == 1
    assert stats["total_duration_s"] == pytest.approx(0.3)
    assert stats["count"] == [0, 1, 1, 1, 1, 1]
    assert stats["duration_s"] == pytest.approx([0, 0, 0.1, 0.2, 0.3, 0.3])
    assert stats["episodes"][0]["release_t"] == pytest.approx(0.4)


def test_release_then_rebrake_counts_another_event():
    stats = brake_activity(make_log(["brake", "cruise", "brake", "cruise"]))
    assert stats["total_count"] == 2
    assert stats["total_duration_s"] == pytest.approx(0.2)
    assert [e["start_t"] for e in stats["episodes"]] == pytest.approx([0, 0.2])
    assert [e["release_t"] for e in stats["episodes"]] == pytest.approx([0.1, 0.3])


def test_end_is_not_a_release_and_copied_terminal_action_is_not_executed():
    stats = brake_activity(make_log(["cruise", "brake", "brake"]))
    assert stats["total_count"] == 1
    assert stats["episodes"][0]["release_t"] is None
    assert stats["episodes"][0]["end_t"] == pytest.approx(0.3)
    assert stats["total_duration_s"] == pytest.approx(0.2)
    assert stats["duration_s"][-1] == pytest.approx(0.2)


@pytest.mark.parametrize("actions", [[], ["cruise"] * 5])
def test_never_brakes_despite_unexecuted_terminal_brake(actions):
    stats = brake_activity(make_log(actions))
    assert stats["total_count"] == 0
    assert stats["total_duration_s"] == 0
    assert stats["episodes"] == []


def test_longer_hold_changes_duration_not_count():
    shorter = brake_activity(make_log(["brake", "cruise", "cruise"]))
    longer = brake_activity(make_log(["brake", "brake", "cruise"]))
    assert shorter["total_count"] == longer["total_count"] == 1
    assert longer["total_duration_s"] == pytest.approx(shorter["total_duration_s"] + 0.1)
    assert longer["episodes"][0]["release_t"] == pytest.approx(0.2)


def test_legacy_logs_count_all_rows_as_executed():
    log = make_log(["brake", "cruise"], terminal="brake")
    log.executed_steps = None
    stats = brake_activity(log)
    assert stats["total_count"] == 2
    assert stats["total_duration_s"] == pytest.approx(0.2)
    assert stats["episodes"][-1]["release_t"] is None

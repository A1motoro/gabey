"""The two tests below guard the core claim of the project (docs/spec.md §1.4)."""

import pytest


@pytest.mark.skip(reason="TODO(P): assess_risk head-on → ttc = gap / closing speed")
def test_assess_risk_head_on(): ...


@pytest.mark.skip(reason="TODO(P): Reactive ignores an agent still in the adjacent lane")
def test_reactive_ignores_adjacent_lane(): ...


@pytest.mark.skip(reason="TODO(P): mid-lane-change cutter → CV ttc < inf while Reactive = inf")
def test_cv_sees_cut_in_before_reactive(): ...

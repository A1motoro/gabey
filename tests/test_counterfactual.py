from pbs.eval.counterfactual import is_benign
from tests.test_scenarios import straight_scenario


def test_slower_lead_is_dangerous():
    assert not is_benign(straight_scenario(lead_v=10.0))


def test_faster_lead_is_benign():
    assert is_benign(straight_scenario(lead_v=20.0))

"""M3 — braking policies. Owner: C."""

CRUISE, BRAKE = "cruise", "brake"


class NeverBrakePolicy:
    """Counterfactual reference (defines benign scenarios) and walking-skeleton policy."""

    def decide(self, ttc: float) -> str:
        return CRUISE


class TTCThresholdPolicy:
    """TTC < ttc_threshold → brake, else cruise."""

    def __init__(self, ttc_threshold: float = 2.0, a_brake: float = 6.0):
        self.ttc_threshold = ttc_threshold
        self.a_brake = a_brake  # m/s², used by simulator when action == brake

    def decide(self, ttc: float) -> str:
        return BRAKE if ttc < self.ttc_threshold else CRUISE


AlwaysCruisePolicy = NeverBrakePolicy  # backward-compatible alias

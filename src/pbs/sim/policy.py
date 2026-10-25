"""M3 — braking policies. Owner: C."""

import math

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


class HysteresisPolicy:
    """Stateful TTC control; cruise holds speed, it does not accelerate.

    Enter braking below t_on, release above t_off; equality retains state.
    run_sim resets this policy before every independent run.
    """

    def __init__(self, t_on: float = 2.0, t_off: float = 3.0, a_brake: float = 6.0):
        if not all(math.isfinite(x) for x in (t_on, t_off, a_brake)):
            raise ValueError("Policy parameters must be finite.")
        if not 0 < t_on < t_off or a_brake <= 0:
            raise ValueError("Require 0 < t_on < t_off and a_brake > 0.")
        self.t_on, self.t_off, self.a_brake = t_on, t_off, a_brake
        self.reset()

    def reset(self) -> None:
        self.state = CRUISE

    def decide(self, ttc: float) -> str:
        if math.isnan(ttc) or ttc < 0:
            raise ValueError("TTC must be nonnegative or positive infinity.")
        if self.state == CRUISE and ttc < self.t_on:
            self.state = BRAKE
        elif self.state == BRAKE and ttc > self.t_off:
            self.state = CRUISE
        return self.state

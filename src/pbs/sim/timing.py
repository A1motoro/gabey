"""Reserved computation timing seam, separate from observation age and hysteresis.

A model describes total predictor/assessor/policy work for one decision tick.
Nonzero delay needs a future execution queue and vehicle evolution while waiting;
run_sim explicitly rejects it until those semantics are implemented. No wall-clock
sleep or animation-speed change is used as a substitute.
"""

from typing import Protocol


class ComputationDelayModel(Protocol):
    def delay_seconds(self, *, t_now: float) -> float: ...


class NoComputationDelay:
    def delay_seconds(self, *, t_now: float) -> float:
        return 0.0

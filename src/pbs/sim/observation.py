"""Observation boundary: transforms may change measurements, never reference tracks."""

from typing import Protocol

from pbs.types import Trajectory


class ObservationModel(Protocol):
    """Return a nonempty, causal trajectory with the same agent identity.

    Input arrays are independent of scenario truth. Implementations must preserve
    timestamps and cache stochastic errors per (agent_id, timestamp), so rereading
    a sample does not redraw its noise. Construct a fresh model per simulation.
    """

    def observe(self, history: Trajectory, *, t_now: float) -> Trajectory: ...


class IdentityObservation:
    """Explicit no-noise adapter; run_sim already supplies isolated history arrays."""

    def observe(self, history: Trajectory, *, t_now: float) -> Trajectory:
        return history

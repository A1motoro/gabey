"""Opt-in frame watchdog and missing-observation arbitration for staged braking."""

import math
import time
from collections import Counter
from dataclasses import dataclass
from numbers import Integral

import numpy as np

from pbs.sim._failsafe_channels import AsyncChannels, VirtualChannels
from pbs.sim.policy import COAST_ZERO, CRUISE, HARD_BRAKE, SOFT_BRAKE, StagedBrakingPolicy
from pbs.types import FrameDecision

_PRIORITY = {"normal": 0, COAST_ZERO: 1, SOFT_BRAKE: 2, HARD_BRAKE: 3}


@dataclass(frozen=True)
class FailsafeConfig:
    frame_budget_s: float = 0.05
    max_obs_age_s: float = 0.05
    missing_grace_s: float = 0.20
    recovery_confirm_frames: int = 2
    latency_margin: float = 0.0
    base_latency: float = 0.0

    def __post_init__(self):
        durations = (
            self.max_obs_age_s,
            self.missing_grace_s,
            self.latency_margin,
            self.base_latency,
            self.frame_budget_s,
        )
        if not all(math.isfinite(d) and d >= 0 for d in durations) or self.frame_budget_s == 0:
            raise ValueError("Expected nonnegative finite durations and a positive frame budget.")
        n = self.recovery_confirm_frames
        if isinstance(n, bool) or not isinstance(n, Integral) or n < 1:
            raise ValueError("recovery_confirm_frames must be a positive integer.")


@dataclass
class _Track:
    sample: float = -math.inf
    demand: str = CRUISE
    confirming: int | None = None

    def observe(self, agent, history, cutoff, max_age):
        if history is None or history.agent_id != agent or not len(history.t):
            return False
        sample = float(history.t[-1])
        if sample <= self.sample or not -1e-9 <= cutoff - sample <= max_age + 1e-9:
            return False
        measurements = (history.t, history.x, history.y, history.v, history.heading)
        if not all(
            np.shape(a) == (len(history.t),) and np.isfinite(a).all() for a in measurements
        ) or not np.all(np.diff(history.t) > 0):
            return False
        self.sample = sample
        return True

    def update_demand(self, demand, confirm_frames):
        if self.confirming is not None:
            self.confirming += 1
            if self.confirming >= confirm_frames:
                self.confirming = None
        if self.confirming is None or demand == HARD_BRAKE or self.demand == CRUISE:
            self.demand = demand


class FrameFailsafeCoordinator:
    """Use bind/step_frame/close directly, or pass to run_sim, which owns this lifecycle.

    Deterministic timings are simulated seconds since frame start, independent of
    actual CPU time. Supply clock() for a virtual frame-start clock (default ctx.t_now).
    Async mode uses a monotonic wall clock and spawn-compatible assessors.
    """

    def __init__(
        self, primary, *, reactive=None, config=None, executor_mode="deterministic", clock=None
    ):
        from pbs.risk.assessors import ReactiveAssessor

        if executor_mode not in ("deterministic", "async"):
            raise ValueError("Unknown executor_mode.")
        if executor_mode == "async" and clock is not None:
            raise ValueError("Async deadlines use time.monotonic.")
        self.primary = primary
        self.reactive = ReactiveAssessor() if reactive is None else reactive
        self.config = config or FailsafeConfig()
        self.executor_mode = executor_mode
        self.clock = time.monotonic if executor_mode == "async" else clock
        self.channels = None

    def bind(self, scenario, *, policy=None, base_latency=None):
        self.close()
        self.policy = policy if policy is not None else StagedBrakingPolicy()
        self.latency = self.config.base_latency if base_latency is None else base_latency
        self.tracks = {a.agent_id: _Track() for a in scenario.agents}
        self.last_frame = None
        self.last_valid_sample = None
        self.global_hold = "normal"
        self.global_confirm = 0
        self.fallback_reasons = Counter()
        self.dropped_stale_results = 0
        self.last_decision = None
        backend = AsyncChannels if self.executor_mode == "async" else VirtualChannels
        self.channels = backend({"reactive": self.reactive, "primary": self.primary}, scenario)

    def close(self):
        if self.channels is not None:
            self.channels.close()
            self.channels = None

    def step_frame(
        self, frame_id, ctx, agent_histories, *, current_policy_mode, channel_timings=None
    ):
        if self.channels is None or (self.last_frame is not None and frame_id <= self.last_frame):
            raise ValueError("Bind the coordinator and provide strictly increasing frame IDs.")
        start = float(ctx.t_now if self.clock is None else self.clock())
        cutoff = ctx.t_now - self.latency
        fresh = {}
        for agent, history in agent_histories.items():
            track = self.tracks.setdefault(agent, _Track())
            if track.observe(agent, history, cutoff, self.config.max_obs_age_s):
                fresh[agent] = history

        values, reasons, dropped, elapsed = self.channels.collect(
            frame_id, ctx, fresh, start, start + self.config.frame_budget_s, channel_timings or {}
        )
        self.last_frame = frame_id
        self.fallback_reasons.update(reasons)
        self.dropped_stale_results += dropped
        sources, missing = {}, []
        effective = math.inf
        floor = "normal"
        for agent, track in self.tracks.items():
            primary = values.get(("primary", agent))
            reactive = values.get(("reactive", agent))
            if primary is None and reactive is None:
                missing.append(agent)
                track.confirming = 0
            else:
                if primary is None:
                    sources[agent], ttc = "reactive_fallback", reactive
                elif reactive is None:
                    sources[agent], ttc = "primary_only", primary
                else:
                    sources[agent], ttc = "fused", min(primary, reactive)
                ttc = max(0.0, ttc - self.config.latency_margin)
                effective = min(effective, ttc)
                self.last_valid_sample = (
                    track.sample
                    if self.last_valid_sample is None
                    else max(self.last_valid_sample, track.sample)
                )
                demand = self.policy.brake_mode(ttc, track.demand)
                track.update_demand(demand, self.config.recovery_confirm_frames)
            if track.demand in (HARD_BRAKE, SOFT_BRAKE) and track.confirming is not None:
                floor = max(floor, track.demand, key=_PRIORITY.get)

        if missing and not sources:
            if self.last_valid_sample is None:
                self.last_valid_sample = cutoff  # No sample since the simulation started.
            effective = math.nan
            hold = COAST_ZERO
            if cutoff - self.last_valid_sample + 1e-9 >= self.config.missing_grace_s:
                hold = SOFT_BRAKE
            if current_policy_mode in (SOFT_BRAKE, HARD_BRAKE):
                hold = max(hold, current_policy_mode, key=_PRIORITY.get)
            self.global_hold = max(self.global_hold, hold, key=_PRIORITY.get)
            self.global_confirm = 0
        elif self.global_hold != "normal":
            self.global_confirm += 1
            if self.global_confirm >= self.config.recovery_confirm_frames:
                self.global_hold = "normal"
        floor = max(floor, self.global_hold, key=_PRIORITY.get)
        normal = self.policy.brake_mode(effective, current_policy_mode)
        if floor != "normal" and normal in (SOFT_BRAKE, HARD_BRAKE):
            floor = max(floor, normal, key=_PRIORITY.get)
        distinct = set(sources.values())
        source = "mixed" if distinct else "clear"
        if len(distinct) == 1:
            source = distinct.pop()
        self.last_decision = FrameDecision(
            frame_id,
            effective,
            floor,
            "failsafe_hold" if floor != "normal" else source,
            tuple(missing),
            dropped,
            elapsed,
            sources,
        )
        return self.last_decision

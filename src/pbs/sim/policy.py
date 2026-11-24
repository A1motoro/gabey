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


SOFT_BRAKE, HARD_BRAKE, RECOVER = "soft_brake", "hard_brake", "recover"


COAST_ZERO = "coast_zero"


def is_braking_action(action: str) -> bool:
    return action in (BRAKE, SOFT_BRAKE, HARD_BRAKE)


def _speed_limit_accel(speed_margin: float, dt: float, jerk_step: float) -> float:
    """Largest acceleration magnitude that can ramp to zero within the speed margin.

    With n positive steps including this one, releasing at maximum jerk uses
    dt * (n * accel - jerk_step * n * (n - 1) / 2) of the remaining speed.
    """
    n = max(1, math.ceil((math.sqrt(1 + 8 * speed_margin / (dt * jerk_step)) - 1) / 2))
    return (speed_margin / dt) / n + (n - 1) * jerk_step / 2


class StagedBrakingPolicy:
    """TTC hysteresis with independent comfort and emergency acceleration slew rates."""

    def __init__(
        self,
        *,
        t_on_soft=3.0,
        t_off_soft=3.5,
        t_on_hard=1.5,
        t_off_hard=2.0,
        a_soft=2.5,
        a_hard=6.0,
        a_recover=1.5,
        jerk_soft: float = 5.0,
        jerk_hard: float = 20.0,
        v_target: float | None = None,
        max_jerk: float | None = None,
    ):
        # Older experiments explicitly selected one rate for both stages.
        if max_jerk is not None:
            jerk_soft = jerk_hard = max_jerk
        parameters = (
            t_on_soft,
            t_off_soft,
            t_on_hard,
            t_off_hard,
            a_soft,
            a_hard,
            a_recover,
            jerk_soft,
            jerk_hard,
        )
        if not all(math.isfinite(x) for x in parameters):
            raise ValueError("Staged braking parameters must be finite.")
        if not 0 < t_on_hard < t_off_hard <= t_on_soft < t_off_soft:
            raise ValueError("Invalid staged TTC thresholds.")
        if not 0 < a_soft <= a_hard or a_recover < 0 or not 0 < jerk_soft <= jerk_hard:
            raise ValueError("Invalid staged acceleration or jerk bounds.")
        self.t_on_soft, self.t_off_soft = t_on_soft, t_off_soft
        self.t_on_hard, self.t_off_hard = t_on_hard, t_off_hard
        self.a_soft, self.a_hard = a_soft, a_hard
        self.a_recover, self.a_brake = a_recover, a_hard
        self.jerk_soft, self.jerk_hard = jerk_soft, jerk_hard
        self._default_v_target = v_target
        self.reset()

    def reset(self, *, v_target: float | None = None) -> None:
        if v_target is None:
            v_target = self._default_v_target
        if v_target is not None and (not math.isfinite(v_target) or v_target < 0):
            raise ValueError("Target speed must be finite and nonnegative.")
        self.v_target = v_target
        self.state = CRUISE
        self.prev_a = 0.0

    def brake_mode(self, ttc: float, previous: str) -> str:
        """Brake demand with hysteresis, without advancing the actuator."""
        if ttc < self.t_on_hard or (previous == HARD_BRAKE and ttc <= self.t_off_hard):
            return HARD_BRAKE
        if ttc < self.t_on_soft or (is_braking_action(previous) and ttc <= self.t_off_soft):
            return SOFT_BRAKE
        return CRUISE

    def decide_accel(
        self, ttc: float, *, v: float, dt: float, mode_override: str = "normal"
    ) -> tuple[str, float]:
        if mode_override == "normal" and (math.isnan(ttc) or ttc < 0):
            raise ValueError("TTC must be nonnegative or positive infinity.")
        if not math.isfinite(v) or v < 0 or not math.isfinite(dt) or dt <= 0:
            raise ValueError("Require nonnegative finite speed and positive finite dt.")
        if self.v_target is None:
            self.v_target = v
        if mode_override == "normal":
            mode = self.brake_mode(ttc, self.state)
            if mode == CRUISE and self.a_recover > 0 and v < self.v_target - 1e-6:
                mode = RECOVER
        elif mode_override in (COAST_ZERO, SOFT_BRAKE, HARD_BRAKE):
            mode = mode_override
        else:
            raise ValueError(f"Invalid failsafe mode: {mode_override!r}")
        self.state = mode
        return self._apply_mode_accel(mode, v=v, dt=dt)

    def _apply_mode_accel(self, mode: str, *, v: float, dt: float) -> tuple[str, float]:
        target = {
            HARD_BRAKE: -self.a_hard,
            SOFT_BRAKE: -self.a_soft,
            RECOVER: self.a_recover,
            CRUISE: 0.0,
            COAST_ZERO: 0.0,
        }[mode]
        step = (self.jerk_hard if mode == HARD_BRAKE else self.jerk_soft) * dt
        a = max(self.prev_a - step, min(target, self.prev_a + step))
        # Begin releasing before either speed boundary. Clamping only after
        # reaching rest or cruise speed would create a one-frame jerk spike.
        # Even an emergency command may release into the comfort stage next tick.
        release_step = self.jerk_soft * dt
        if a < 0:
            a = max(a, -_speed_limit_accel(v, dt, release_step))
        elif a > 0:
            a = min(a, _speed_limit_accel(max(0.0, self.v_target - v), dt, release_step))
        self.prev_a = a
        return self.state, a

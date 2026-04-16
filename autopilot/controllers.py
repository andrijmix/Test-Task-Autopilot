"""Stage 3 flight controllers.

AltitudeController — PI controller that outputs throttle PWM to hold a target
altitude in ArduCopter Stabilize mode.  The plant model is:

    throttle_pwm = hover_pwm + Kp * alt_error + Ki * integral(alt_error)

  alt_error > 0  → below target → increase throttle → climb
  alt_error < 0  → above target → decrease throttle → descend

Tuning notes (SITL defaults):
  Kp = 5.0   →  10 m error  →  50 PWM delta from hover
  Ki = 0.05  →  mild steady-state correction, anti-windup at ±200 PWM
  hover_pwm  ≈  1500 (neutral throttle ≈ hover in default SITL)
"""

import time


class AltitudeController:
    """PI altitude controller outputting throttle PWM.

    Parameters
    ----------
    kp            P gain (PWM per metre error)
    ki            I gain (PWM per metre·second)
    hover_pwm     Base throttle PWM that gives level flight in Stabilize mode
    min_pwm       Hard lower bound for throttle output
    max_pwm       Hard upper bound for throttle output
    max_delta_pwm Maximum absolute deviation from hover_pwm
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        hover_pwm: int,
        min_pwm: int,
        max_pwm: int,
        max_delta_pwm: int,
    ) -> None:
        self._kp = kp
        self._ki = ki
        self._hover_pwm = hover_pwm
        self._min_pwm = min_pwm
        self._max_pwm = max_pwm
        self._max_delta = max_delta_pwm
        self._integral: float = 0.0
        self._last_t: float | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def compute(self, target_alt: float, current_alt: float, t: float) -> int:
        """Return throttle PWM for the given altitude error at time *t*."""
        error = target_alt - current_alt

        dt = 0.0
        if self._last_t is not None:
            dt = max(0.0, t - self._last_t)
        self._last_t = t

        self._integral += error * dt
        # Anti-windup: clamp integral so its contribution stays within max_delta
        max_i = self._max_delta / max(self._ki, 1e-9)
        self._integral = max(-max_i, min(max_i, self._integral))

        delta = int(self._kp * error + self._ki * self._integral)
        delta = max(-self._max_delta, min(self._max_delta, delta))

        pwm = self._hover_pwm + delta
        return max(self._min_pwm, min(self._max_pwm, pwm))

    def reset(self) -> None:
        """Clear integrator state (call when entering a new phase)."""
        self._integral = 0.0
        self._last_t = None


class LateralController:
    """PI controller for lateral drift correction via the roll channel.

    Measures lateral speed perpendicular to the desired track bearing and
    outputs a roll PWM delta that counteracts the drift.

    Sign convention
    ---------------
    lateral_speed > 0  →  drone drifting rightward of track  →  roll_delta < 0 (bank left)
    lateral_speed < 0  →  drone drifting leftward of track   →  roll_delta > 0 (bank right)

    roll_delta = -(Kp * lateral_speed + Ki * ∫lateral_speed dt)

    The I term naturally accumulates steady-state wind offset and provides a
    feedforward compensation once the wind pattern is learned (typically within
    the first 5–10 seconds of cruise flight).
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        max_roll_delta_pwm: int,
    ) -> None:
        self._kp = kp
        self._ki = ki
        self._max_delta = max_roll_delta_pwm
        self._integral: float = 0.0
        self._last_t: float | None = None

    def compute(self, lateral_speed: float, t: float) -> int:
        """Return roll PWM delta to counteract lateral drift.

        Parameters
        ----------
        lateral_speed   m/s perpendicular to track, positive = rightward
        t               monotonic timestamp in seconds

        Returns
        -------
        int  PWM delta: negative = bank left, positive = bank right
        """
        dt = 0.0
        if self._last_t is not None:
            dt = max(0.0, min(1.0, t - self._last_t))
        self._last_t = t

        self._integral += lateral_speed * dt
        if self._ki > 1e-9:
            max_i = self._max_delta / self._ki
            self._integral = max(-max_i, min(max_i, self._integral))

        delta = -(self._kp * lateral_speed + self._ki * self._integral)
        return max(-self._max_delta, min(self._max_delta, int(round(delta))))

    def reset(self) -> None:
        """Clear integrator state."""
        self._integral = 0.0
        self._last_t = None

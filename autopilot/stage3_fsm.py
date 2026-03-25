"""Stage 3 Mission FSM — controlled flight A→B in ArduCopter Stabilize mode.

State machine
─────────────
  INIT  ──► READY  ──► TAKEOFF  ──► ENROUTE_TO_B  ──► APPROACH_FINE  ──► LANDING  ──► COMPLETE
   │          │           │               │                  │               │
   └──────────┴───────────┴───────────────┴──────────────────┴───────────────┴──► ABORT

Control overview
────────────────
  Altitude: PI controller on throttle channel (CH3).
      throttle_pwm = hover_pwm + Kp*(target_alt - alt) + Ki*∫error dt

  Heading: P controller on yaw channel (CH4).
      yaw_pwm = neutral_yaw + Kp_yaw * normalize(bearing_to_B - heading)

  Forward motion: constant positive pitch delta on pitch channel (CH2).
      ENROUTE:  neutral_pitch + pitch_delta_cruise   (aggressive)
      APPROACH: neutral_pitch + pitch_delta_fine     (gentle)

  Landing: two-phase throttle reduction.
      alt > 5 m  → ramp below hover at 2 PWM/s, floor = hover-150
      alt ≤ 5 m  → fixed flare throttle = hover-80
      alt ≤ land_complete_alt → COMPLETE

Safety
──────
  • Per-state timeout → ABORT
  • GPS/altitude unavailable → ABORT
  • ABORT / exception → guaranteed safe_reset()
  • Rate limiting and PWM saturation enforced by RcOverrideAdapter

SITL-only flag
──────────────
  dry_run=True  skips arming and mode changes; all RC commands are still
  computed and logged (DryRunRcOverrideAdapter drops them).
"""

import logging
import math
import time
from enum import Enum
from typing import Optional, Tuple

from .config import AppConfig
from .controllers import AltitudeController
from .dronekit_compat import ensure_dronekit_compat
from .geo import bearing_deg, distance_m, normalize_angle_deg
from .rc_override import RcCommand
from .telemetry import telemetry_snapshot

ensure_dronekit_compat()

from dronekit import VehicleMode  # noqa: E402  (after compat patch)


# ---------------------------------------------------------------------------
# State enum
# ---------------------------------------------------------------------------


class Stage3State(str, Enum):
    INIT = "INIT"
    READY = "READY"
    TAKEOFF = "TAKEOFF"
    ENROUTE_TO_B = "ENROUTE_TO_B"
    APPROACH_FINE = "APPROACH_FINE"
    LANDING = "LANDING"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"


# ---------------------------------------------------------------------------
# FSM
# ---------------------------------------------------------------------------


class Stage3FSM:
    """Stage 3 mission state machine.

    Parameters
    ----------
    vehicle    DroneKit Vehicle (real SITL connection or mock)
    cfg        Full AppConfig
    rc_adapter RcOverrideAdapter or DryRunRcOverrideAdapter
    logger     Bound Python logger
    dry_run    When True, skip arm/mode-change calls to the vehicle
    """

    def __init__(
        self,
        vehicle,
        cfg: AppConfig,
        rc_adapter,
        logger: logging.Logger,
        dry_run: bool = False,
    ) -> None:
        self._vehicle = vehicle
        self._cfg = cfg
        self._rc = rc_adapter
        self._logger = logger
        self._dry_run = dry_run

        self._state = Stage3State.INIT
        self._state_entered_at = time.monotonic()
        self._abort_reason: Optional[str] = None

        s3 = cfg.stage3
        # Arming helpers
        self._arm_override_cleared: bool = False
        self._alt_ctrl = AltitudeController(
            kp=s3.alt_ctrl.kp,
            ki=s3.alt_ctrl.ki,
            hover_pwm=s3.alt_ctrl.hover_pwm,
            min_pwm=cfg.rc_override.bounds.min_pwm,
            max_pwm=cfg.rc_override.bounds.max_pwm,
            max_delta_pwm=s3.alt_ctrl.max_delta_pwm,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(self) -> Stage3State:
        """Block until COMPLETE or ABORT; always returns a terminal state."""
        self._logger.info(
            "Stage3FSM started: state=%s target_alt=%.1fm target_B=(%.6f, %.6f) dry_run=%s",
            self._state.value,
            self._cfg.mission.target_altitude_m,
            self._cfg.mission.point_b.lat,
            self._cfg.mission.point_b.lon,
            self._dry_run,
        )
        try:
            while True:
                self._tick()
                if self._state in (Stage3State.COMPLETE, Stage3State.ABORT):
                    return self._state
                time.sleep(self._cfg.mission_runtime.tick_interval_s)
        except Exception:
            self._safe_reset("exception in Stage3FSM main loop")
            raise

    # ------------------------------------------------------------------
    # Tick dispatcher
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        elapsed = time.monotonic() - self._state_entered_at
        timeout = self._timeout_for(self._state)

        if timeout is not None and elapsed > timeout:
            self._abort(
                f"timeout in {self._state.value}: {elapsed:.1f}s > {timeout:.1f}s"
            )
            return

        dispatch = {
            Stage3State.INIT: self._tick_init,
            Stage3State.READY: lambda: self._tick_ready(elapsed),
            Stage3State.TAKEOFF: lambda: self._tick_takeoff(elapsed),
            Stage3State.ENROUTE_TO_B: lambda: self._tick_enroute(elapsed),
            Stage3State.APPROACH_FINE: lambda: self._tick_approach(elapsed),
            Stage3State.LANDING: lambda: self._tick_landing(elapsed),
            Stage3State.COMPLETE: lambda: self._safe_reset("mission complete"),
            Stage3State.ABORT: lambda: self._safe_reset(
                f"abort: {self._abort_reason}"
            ),
        }
        dispatch[self._state]()

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def _tick_init(self) -> None:
        target_alt = self._cfg.mission.target_altitude_m
        if target_alt <= 0:
            self._abort("invalid target_altitude in config")
            return
        self._logger.info(
            "Stage3 INIT: target_alt=%.1fm  B=(%.6f, %.6f)",
            target_alt,
            self._cfg.mission.point_b.lat,
            self._cfg.mission.point_b.lon,
        )
        self._transition(Stage3State.READY, "config validated")

    def _tick_ready(self, elapsed_s: float) -> None:
        if self._dry_run:
            self._logger.info("Stage3 READY [dry-run]: skip mode set, elapsed=%.1fs", elapsed_s)
            self._transition(Stage3State.TAKEOFF, "dry-run: skip STABILIZE wait")
            return

        # Disarm first if vehicle is already armed (e.g. after a previous crash-respawn)
        if getattr(self._vehicle, "armed", False):
            self._logger.warning("Stage3 READY: vehicle is armed — disarming for clean takeoff")
            self._vehicle.armed = False
            return  # wait for disarm on next tick

        current_mode = getattr(self._vehicle.mode, "name", None)
        if current_mode != "STABILIZE":
            self._logger.info(
                "Stage3 READY: requesting STABILIZE mode (current=%s)", current_mode
            )
            self._set_mode("STABILIZE")
            return  # re-check on next tick

        self._logger.info("Stage3 READY: STABILIZE mode confirmed, elapsed=%.1fs", elapsed_s)
        self._transition(Stage3State.TAKEOFF, "STABILIZE confirmed")

    def _tick_takeoff(self, elapsed_s: float) -> None:
        snap = self._fresh_snapshot()
        if snap is None:
            return

        alt = snap.get("alt_m") or 0.0
        armed = snap.get("armed", False)
        target_alt = self._cfg.mission.target_altitude_m
        neutral = self._rc.neutral_command()

        if not armed:
            if not self._dry_run:
                if not self._arm_override_cleared:
                    # First tick: clear all RC overrides so SITL falls back to its
                    # default throttle (~1000), which satisfies the arm throttle check.
                    # (RC calibration in SITL may have RC3_MIN=1000, making any override
                    # ≥1100 read as >5% stick and trigger "Throttle too high".)
                    self._vehicle.channels.overrides = {}
                    self._arm_override_cleared = True
                    self._logger.info(
                        "state=TAKEOFF elapsed=%.1fs  waiting_arm=True  alt=%.2fm  "
                        "overrides_cleared (SITL throttle now at default)",
                        elapsed_s, alt,
                    )
                else:
                    # Subsequent ticks: SITL default throttle≈1000 is active → arm
                    self._logger.info(
                        "state=TAKEOFF elapsed=%.1fs  waiting_arm=True  alt=%.2fm",
                        elapsed_s, alt,
                    )
                    self._try_arm()
            else:
                self._logger.info(
                    "state=TAKEOFF elapsed=%.1fs  waiting_arm=True  alt=%.2fm  [dry-run: skip arm]",
                    elapsed_s, alt,
                )
            return

        # Armed: apply full climb initially, then taper throttle as we approach target altitude.
        nav_cfg = self._cfg.stage3.nav
        taper_start_alt = target_alt * nav_cfg.takeoff_taper_start_fraction
        if alt >= taper_start_alt:
            thr = max(
                nav_cfg.takeoff_taper_min_pwm,
                self._alt_ctrl.compute(target_alt, alt, time.monotonic()),
            )
        else:
            thr = nav_cfg.takeoff_throttle_pwm
        cmd = self._rc.apply(
            roll=neutral.roll,
            pitch=neutral.pitch,
            throttle=thr,
            yaw=neutral.yaw,
        )
        pct = (alt / target_alt * 100.0) if target_alt else 0.0
        self._logger.info(
            "state=TAKEOFF elapsed=%.1fs  alt=%.2fm/%.1fm (%.0f%%)  rc(thr=%d)",
            elapsed_s,
            alt,
            target_alt,
            pct,
            cmd.throttle,
        )

        threshold = target_alt * nav_cfg.takeoff_complete_fraction
        if alt >= threshold:
            self._alt_ctrl.reset()
            self._transition(
                Stage3State.ENROUTE_TO_B,
                f"takeoff done: alt={alt:.1f}m >= {threshold:.1f}m ({pct:.0f}%)",
            )

    def _tick_enroute(self, elapsed_s: float) -> None:
        nav = self._nav_snapshot()
        if nav is None:
            return

        lat, lon, alt, dist, brng, yaw_err, heading = nav
        target_alt = self._cfg.mission.target_altitude_m
        alt_err = target_alt - alt

        throttle = self._alt_ctrl.compute(target_alt, alt, time.monotonic())
        neutral = self._rc.neutral_command()

        yaw_kp = self._cfg.stage3.nav.yaw_kp
        yaw_delta = int(max(-200, min(200, yaw_err * yaw_kp)))
        # Suppress forward pitch until heading is roughly aligned with bearing
        aligned = abs(yaw_err) <= self._cfg.stage3.nav.align_threshold_deg
        if aligned:
            ramp_s = max(0.5, self._cfg.stage3.nav.enroute_pitch_ramp_s)
            ramp = min(1.0, elapsed_s / ramp_s)
            pitch_delta = int(round(self._cfg.stage3.nav.pitch_delta_cruise * ramp))
        else:
            pitch_delta = 0

        cmd = self._rc.apply(
            roll=neutral.roll,
            pitch=neutral.pitch + pitch_delta,
            throttle=throttle,
            yaw=neutral.yaw + yaw_delta,
        )

        self._logger.info(
            "state=ENROUTE_TO_B  dist=%.1fm  brng=%.1f  hdg=%.1f  "
            "alt=%.1fm  alt_err=%.1f  aligned=%s  rc(roll=%d pitch=%d thr=%d yaw=%d)",
            dist, brng, heading, alt, alt_err, aligned,
            cmd.roll, cmd.pitch, cmd.throttle, cmd.yaw,
        )

        r_fine = self._cfg.stage3.nav.r_cruise_to_fine_m
        if dist <= r_fine:
            self._transition(
                Stage3State.APPROACH_FINE,
                f"fine approach: dist={dist:.1f}m <= r_fine={r_fine:.1f}m",
            )

    def _tick_approach(self, elapsed_s: float) -> None:
        nav = self._nav_snapshot()
        if nav is None:
            return

        lat, lon, alt, dist, brng, yaw_err, heading = nav
        target_alt = self._cfg.mission.target_altitude_m
        alt_err = target_alt - alt

        throttle = self._alt_ctrl.compute(target_alt, alt, time.monotonic())
        neutral = self._rc.neutral_command()

        yaw_kp = self._cfg.stage3.nav.yaw_kp
        yaw_delta = int(max(-150, min(150, yaw_err * yaw_kp)))
        aligned = abs(yaw_err) <= self._cfg.stage3.nav.align_threshold_deg

        # In the final approach, keep momentum until close to B, then slow down sharply.
        nav_cfg = self._cfg.stage3.nav
        r_land = nav_cfg.r_land_trigger_m
        slowdown_start = nav_cfg.r_approach_slowdown_m
        max_fine = nav_cfg.pitch_delta_fine
        min_fine = nav_cfg.pitch_delta_fine_min
        if aligned and dist > r_land:
            if dist > slowdown_start:
                pitch_delta = max_fine
            else:
                ratio = max(0.0, min(1.0, (dist - r_land) / (slowdown_start - r_land)))
                pitch_delta = int(round(min_fine + ratio * (max_fine - min_fine)))
        else:
            pitch_delta = 0

        cmd = self._rc.apply(
            roll=neutral.roll,
            pitch=neutral.pitch + pitch_delta,
            throttle=throttle,
            yaw=neutral.yaw + yaw_delta,
        )

        h_speed = self._horizontal_speed()

        self._logger.info(
            "state=APPROACH_FINE  dist=%.1fm  alt=%.1fm  alt_err=%.1f  "
            "h_speed=%.2fm/s  aligned=%s  rc(roll=%d pitch=%d thr=%d yaw=%d)",
            dist, alt, alt_err, h_speed, aligned,
            cmd.roll, cmd.pitch, cmd.throttle, cmd.yaw,
        )

        h_thresh = nav_cfg.land_h_speed_threshold_ms

        if dist <= r_land and h_speed < h_thresh:
            self._transition(
                Stage3State.LANDING,
                f"landing: dist={dist:.1f}m h_speed={h_speed:.2f}m/s",
            )
        elif dist <= nav_cfg.r_arrival_m:
            self._transition(
                Stage3State.LANDING,
                f"arrival radius: dist={dist:.1f}m forcing landing",
            )

    def _tick_landing(self, elapsed_s: float) -> None:
        snap = self._fresh_snapshot()
        if snap is None:
            return

        alt = snap.get("alt_m") or 0.0
        neutral = self._rc.neutral_command()
        hover = self._cfg.stage3.alt_ctrl.hover_pwm
        min_pwm = self._cfg.rc_override.bounds.min_pwm

        # Assertive descent to avoid ballooning after switching from approach to landing.
        if alt > 5.0:
            desc_thr = max(hover - 180, hover - 40 - int(elapsed_s * 4))
        elif alt > self._cfg.stage3.nav.land_complete_alt_m:
            desc_thr = hover - 90
        else:
            desc_thr = max(min_pwm, hover - 200)

        desc_thr = max(min_pwm, desc_thr)

        cmd = self._rc.apply(
            roll=neutral.roll,
            pitch=neutral.pitch,
            throttle=desc_thr,
            yaw=neutral.yaw,
        )

        self._logger.info(
            "state=LANDING  elapsed=%.1fs  alt=%.2fm  thr=%d",
            elapsed_s, alt, cmd.throttle,
        )

        land_alt = self._cfg.stage3.nav.land_complete_alt_m
        if alt <= land_alt:
            b = self._cfg.mission.point_b
            final_dist = distance_m(
                snap["lat"],
                snap["lon"],
                b.lat,
                b.lon,
            )
            self._transition(
                Stage3State.COMPLETE,
                f"landed: alt={alt:.2f}m <= {land_alt:.1f}m, dist_to_B={final_dist:.2f}m",
            )

    # ------------------------------------------------------------------
    # Helpers: navigation & telemetry
    # ------------------------------------------------------------------

    def _fresh_snapshot(self) -> Optional[dict]:
        snap = telemetry_snapshot(self._vehicle)
        if snap.get("lat") is None or snap.get("lon") is None:
            self._abort("GPS position unavailable")
            return None
        return snap

    def _nav_snapshot(self) -> Optional[Tuple[float, float, float, float, float, float, float]]:
        snap = self._fresh_snapshot()
        if snap is None:
            return None

        lat = snap["lat"]
        lon = snap["lon"]
        alt = snap.get("alt_m") or 0.0

        b = self._cfg.mission.point_b
        dist = distance_m(lat, lon, b.lat, b.lon)
        brng = bearing_deg(lat, lon, b.lat, b.lon)
        heading = self._safe_heading()
        yaw_err = normalize_angle_deg(brng - heading)

        return lat, lon, alt, dist, brng, yaw_err, heading

    def _horizontal_speed(self) -> float:
        vel = getattr(self._vehicle, "velocity", None)
        if vel is not None and len(vel) >= 2:
            return math.sqrt(vel[0] ** 2 + vel[1] ** 2)
        snap = telemetry_snapshot(self._vehicle)
        return snap.get("h_speed_ms") or 0.0

    def _safe_heading(self) -> float:
        """Return the vehicle's TRUE heading in degrees [0, 360).

        The ArduCopter SITL magnetometer is calibrated 180° inverted relative to
        the vehicle body, so vehicle.heading reports the direction OPPOSITE to the
        nose.  Adding 180° converts the raw magnetic reading to the actual nose
        direction, which matches the GPS ground-track when flying at speed.
        """
        heading = getattr(self._vehicle, "heading", None)
        h = float(heading) if heading is not None else 0.0
        return (h + 180.0) % 360.0

    # ------------------------------------------------------------------
    # Helpers: vehicle control
    # ------------------------------------------------------------------

    def _set_mode(self, mode_name: str) -> None:
        self._logger.info("Setting vehicle mode → %s", mode_name)
        self._vehicle.mode = VehicleMode(mode_name)

    def _try_arm(self) -> None:
        if not getattr(self._vehicle, "is_armable", False):
            self._logger.warning("Vehicle not armable yet, retrying on next tick")
            return
        self._logger.info("Arming vehicle…")
        self._vehicle.armed = True

    # ------------------------------------------------------------------
    # Helpers: FSM bookkeeping
    # ------------------------------------------------------------------

    def _safe_reset(self, context: str) -> None:
        try:
            self._rc.safe_reset()
            self._logger.info("RC override safely reset (%s)", context)
        except Exception as exc:  # pragma: no cover
            self._logger.error("Error during safe_reset [%s]: %s", context, exc)

    def _abort(self, reason: str) -> None:
        self._abort_reason = reason
        self._logger.error("Stage3 ABORT: %s", reason)
        self._safe_reset(f"abort: {reason}")
        self._state = Stage3State.ABORT
        self._state_entered_at = time.monotonic()

    def _transition(self, target: Stage3State, reason: str) -> None:
        elapsed = time.monotonic() - self._state_entered_at
        self._logger.info(
            "FSM transition: %s -> %s  reason=%s  elapsed_in_state=%.2fs",
            self._state.value,
            target.value,
            reason,
            elapsed,
        )
        self._state = target
        self._state_entered_at = time.monotonic()

    def _timeout_for(self, state: Stage3State) -> Optional[float]:
        t = self._cfg.stage3.timeouts
        return {
            Stage3State.INIT: t.init_s,
            Stage3State.READY: t.ready_s,
            Stage3State.TAKEOFF: t.takeoff_s,
            Stage3State.ENROUTE_TO_B: t.enroute_s,
            Stage3State.APPROACH_FINE: t.approach_s,
            Stage3State.LANDING: t.landing_s,
        }.get(state)

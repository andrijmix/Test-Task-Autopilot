import logging
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from autopilot.config import build_default_config
from autopilot.navigation import NavigationSnapshot
from autopilot.rc_override import RcCommand
from autopilot.stage3_fsm import Stage3FSM


def _make_snap(
    alt_m: float = 0.0,
    heading_deg: float = 180.0,
    armed: bool = True,
) -> NavigationSnapshot:
    return NavigationSnapshot(
        x=0.0, y=0.0, alt_m=alt_m, heading_deg=heading_deg,
        vx=0.0, vy=0.0, armed=armed, mode="STABILIZE",
    )


def _nav_mock(alt: float, dist: float, brng: float, yaw_err: float, heading: float):
    return _make_snap(alt_m=alt, heading_deg=heading), dist, brng, yaw_err


class DummyRcAdapter:
    def __init__(self) -> None:
        self._neutral = RcCommand(roll=1500, pitch=1500, throttle=1500, yaw=1500)

    def neutral_command(self) -> RcCommand:
        return self._neutral

    def apply(self, roll: int, pitch: int, throttle: int, yaw: int) -> RcCommand:
        return RcCommand(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw)

    def safe_reset(self) -> None:
        return None


class DummyVehicle:
    def __init__(self) -> None:
        self.velocity = [0.0, 0.0, 0.0]
        self.heading = 0.0
        self.location = SimpleNamespace(
            global_relative_frame=SimpleNamespace(lat=50.450739, lon=30.461242, alt=300.0),
            local_frame=SimpleNamespace(north=0.0, east=0.0, down=0.0),
        )


def _build_fsm() -> Stage3FSM:
    cfg = build_default_config()
    cfg = replace(
        cfg,
        stage3=replace(
            cfg.stage3,
            nav=replace(
                cfg.stage3.nav,
                pitch_delta_cruise=200,
                enroute_pitch_ramp_s=1.0,
                enroute_unaligned_pitch_base=120,
                enroute_unaligned_target_speed_ms=0.2,
                enroute_unaligned_speed_kp=60.0,
                approach_target_speed_max_ms=2.0,
                approach_hover_target_speed_ms=0.0,
                approach_pitch_speed_kp=80.0,
                approach_pitch_delta_max=260,
                align_threshold_deg=20.0,
            ),
        ),
    )
    return Stage3FSM(
        vehicle=DummyVehicle(),
        cfg=cfg,
        rc_adapter=DummyRcAdapter(),
        logger=logging.getLogger("tests.stage3_fsm"),
        dry_run=True,
    )


def test_enroute_heading_correction_keeps_forward_pitch_near_zero_speed() -> None:
    fsm = _build_fsm()

    pitch_delta = fsm._enroute_pitch_delta(elapsed_s=1.0, yaw_err=35.0, h_speed=0.0)

    assert pitch_delta > 0


def test_enroute_heading_correction_can_fully_bleed_large_forward_speed() -> None:
    fsm = _build_fsm()

    pitch_delta = fsm._enroute_pitch_delta(elapsed_s=1.0, yaw_err=35.0, h_speed=3.0)

    assert pitch_delta == 0


def test_enroute_aligned_uses_cruise_pitch() -> None:
    fsm = _build_fsm()

    pitch_delta = fsm._enroute_pitch_delta(elapsed_s=1.0, yaw_err=5.0, h_speed=0.0)

    assert pitch_delta == 200


def test_takeoff_pitch_correction_pushes_forward_when_wind_blows_away_from_b() -> None:
    fsm = _build_fsm()

    pitch_corr = fsm._takeoff_pitch_correction(track_h_speed=-0.6)

    assert pitch_corr > 0


def test_takeoff_pitch_correction_does_not_push_backward_when_already_moving_to_b() -> None:
    fsm = _build_fsm()
    fsm._takeoff_pitch_integral = 0.0

    pitch_corr = fsm._takeoff_pitch_correction(track_h_speed=0.4)

    assert pitch_corr == 0


def test_takeoff_applies_pitch_correction_to_hold_against_backward_drift() -> None:
    fsm = _build_fsm()
    fsm._fresh_snapshot = lambda: _make_snap(alt_m=40.0, armed=True)
    fsm._horizontal_speed = lambda target_bearing_deg=None: -0.5 if target_bearing_deg is not None else 0.5

    recorded = {}

    def capture_apply(roll: int, pitch: int, throttle: int, yaw: int) -> RcCommand:
        recorded.update({"roll": roll, "pitch": pitch, "throttle": throttle, "yaw": yaw})
        return RcCommand(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw)

    fsm._rc.apply = capture_apply

    fsm._tick_takeoff(8.0)

    assert recorded["pitch"] > fsm._rc.neutral_command().pitch


def test_approach_does_not_land_outside_land_radius() -> None:
    fsm = _build_fsm()
    # dist=12m is outside r_land_trigger_m=10.0m → should NOT trigger landing
    fsm._nav_snapshot = lambda: _nav_mock(300.0, 12.0, 180.0, 35.0, 180.0)
    fsm._horizontal_speed = lambda target_bearing_deg=None: -4.45 if target_bearing_deg is not None else 4.45
    fsm._transition = Mock()

    fsm._tick_approach(0.5)

    fsm._transition.assert_not_called()


def test_approach_lands_once_inside_land_radius_even_with_horizontal_motion() -> None:
    fsm = _build_fsm()
    # dist=8.0m is inside r_land_trigger_m=10.0m → should trigger landing
    fsm._nav_snapshot = lambda: _nav_mock(300.0, 8.0, 180.0, 5.0, 180.0)
    fsm._horizontal_speed = lambda target_bearing_deg=None: 1.30 if target_bearing_deg is None else -0.35
    fsm._transition = Mock()

    fsm._tick_approach(0.5)

    fsm._transition.assert_called_once()
    state, reason = fsm._transition.call_args.args
    assert state.value == "LANDING"
    assert "track_h_speed=-0.35m/s" in reason
    assert "h_speed=1.30m/s" in reason


def test_approach_hover_target_speed_is_zero_inside_land_radius() -> None:
    fsm = _build_fsm()

    target_speed = fsm._approach_target_track_speed(2.0)

    assert target_speed == pytest.approx(0.0)


def test_approach_brakes_aggressively_when_closing_fast_over_point() -> None:
    fsm = _build_fsm()

    pitch_delta = fsm._approach_pitch_delta(dist=2.0, yaw_err=5.0, track_h_speed=2.0)

    assert pitch_delta == 0


def test_approach_adds_recovery_pitch_when_wind_pushes_away_from_point() -> None:
    fsm = _build_fsm()

    pitch_delta = fsm._approach_pitch_delta(dist=6.4, yaw_err=5.0, track_h_speed=-2.11)

    assert pitch_delta > fsm._cfg.stage3.nav.pitch_delta_fine


def test_landing_keeps_approach_horizontal_hold_while_descending() -> None:
    fsm = _build_fsm()
    fsm._nav_snapshot = lambda: _nav_mock(30.0, 2.0, 180.0, 5.0, 180.0)
    fsm._transition = Mock()
    fsm._horizontal_speed = lambda target_bearing_deg=None: 1.20 if target_bearing_deg is None else -0.40

    recorded = {}

    def capture_apply(roll: int, pitch: int, throttle: int, yaw: int) -> RcCommand:
        recorded.update({"roll": roll, "pitch": pitch, "throttle": throttle, "yaw": yaw})
        return RcCommand(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw)

    fsm._rc.apply = capture_apply

    fsm._tick_landing(1.5)

    assert recorded["pitch"] > fsm._rc.neutral_command().pitch
    assert recorded["throttle"] < fsm._cfg.stage3.alt_ctrl.hover_pwm
    fsm._transition.assert_not_called()


def test_fine_approach_entry_radius_uses_slowdown_radius_when_configured_lower() -> None:
    fsm = _build_fsm()

    assert fsm._fine_approach_entry_radius() == pytest.approx(18.0)


def test_enroute_enters_approach_before_crossing_the_point() -> None:
    fsm = _build_fsm()
    fsm._nav_snapshot = lambda: _nav_mock(300.0, 12.0, 180.0, 5.0, 180.0)
    fsm._horizontal_speed = lambda target_bearing_deg=None: 1.0
    fsm._transition = Mock()

    fsm._tick_enroute(1.0)

    fsm._transition.assert_called_once()
    state, reason = fsm._transition.call_args.args
    assert state.value == "APPROACH_FINE"
    assert "dist=12.0m <= r_fine=18.0m" in reason
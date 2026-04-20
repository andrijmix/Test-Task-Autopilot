import logging
import time
from enum import Enum
from typing import Optional, Protocol

from .config import AppConfig
from .geo import bearing_2d, distance_2d, normalize_angle_deg
from .navigation import GpsPositionProvider, PositionProvider
from .rc_override import RcCommand


class MissionState(str, Enum):
    INIT = "INIT"
    READY = "READY"
    ARM_TAKEOFF_STUB = "ARM_TAKEOFF_STUB"
    NAVIGATE_STUB = "NAVIGATE_STUB"
    LAND_STUB = "LAND_STUB"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"


class RcAdapterProtocol(Protocol):
    def apply(self, roll: int, pitch: int, throttle: int, yaw: int) -> RcCommand:
        ...

    def neutral_command(self) -> RcCommand:
        ...

    def safe_reset(self) -> None:
        ...


class MissionFSM:
    def __init__(
        self,
        vehicle,
        cfg: AppConfig,
        rc_adapter: RcAdapterProtocol,
        logger: logging.Logger,
        pos_provider: Optional[PositionProvider] = None,
    ):
        self._vehicle = vehicle
        self._cfg = cfg
        self._rc = rc_adapter
        self._logger = logger
        self._pos_provider: PositionProvider = (
            pos_provider if pos_provider is not None else GpsPositionProvider(vehicle, cfg)
        )
        self._state = MissionState.INIT
        self._state_entered_at = time.time()

    def run(self) -> MissionState:
        self._logger.info("FSM started: state=%s", self._state.value)

        try:
            while True:
                timeout = self._get_timeout_s(self._state)
                elapsed = time.time() - self._state_entered_at
                if timeout is not None and elapsed > timeout:
                    self._transition(MissionState.ABORT, f"timeout in {self._state.value}: {elapsed:.1f}s > {timeout:.1f}s")

                if self._state == MissionState.INIT:
                    self._transition(MissionState.READY, "initialization complete")
                elif self._state == MissionState.READY:
                    self._transition(MissionState.ARM_TAKEOFF_STUB, "preflight passed, mission ready")
                elif self._state == MissionState.ARM_TAKEOFF_STUB:
                    self._tick_arm_takeoff_stub(elapsed)
                elif self._state == MissionState.NAVIGATE_STUB:
                    self._tick_navigate_stub()
                elif self._state == MissionState.LAND_STUB:
                    self._tick_land_stub(elapsed)
                elif self._state == MissionState.COMPLETE:
                    self._rc.safe_reset()
                    self._logger.info("FSM complete: RC override safely reset")
                    return MissionState.COMPLETE
                elif self._state == MissionState.ABORT:
                    self._rc.safe_reset()
                    self._logger.error("FSM abort: RC override safely reset")
                    return MissionState.ABORT

                time.sleep(self._cfg.mission_runtime.tick_interval_s)
        except Exception:
            self._rc.safe_reset()
            raise

    def _tick_arm_takeoff_stub(self, elapsed_s: float) -> None:
        neutral = self._rc.neutral_command()
        cmd = self._rc.apply(
            roll=neutral.roll,
            pitch=neutral.pitch,
            throttle=neutral.throttle,
            yaw=neutral.yaw,
        )
        self._logger.info(
            "state=%s stub_hold_s=%.1f rc(roll=%d pitch=%d throttle=%d yaw=%d)",
            self._state.value,
            elapsed_s,
            cmd.roll,
            cmd.pitch,
            cmd.throttle,
            cmd.yaw,
        )
        if elapsed_s >= self._cfg.mission_runtime.arm_takeoff_stub_hold_s:
            self._transition(MissionState.NAVIGATE_STUB, "takeoff stub hold complete")

    def _tick_navigate_stub(self) -> None:
        snap = self._pos_provider.snapshot()
        if snap is None:
            self._transition(MissionState.ABORT, "position unavailable during navigation")
            return

        dist = distance_2d(snap.x, snap.y, self._pos_provider.target_x, self._pos_provider.target_y)
        bearing_to_target = bearing_2d(snap.x, snap.y, self._pos_provider.target_x, self._pos_provider.target_y)
        yaw_error = normalize_angle_deg(bearing_to_target - snap.heading_deg)

        cmd = self._build_navigation_command(dist, yaw_error)
        applied = self._rc.apply(
            roll=cmd.roll,
            pitch=cmd.pitch,
            throttle=cmd.throttle,
            yaw=cmd.yaw,
        )

        self._logger.info(
            "state=%s dist_m=%.1f bearing_deg=%.1f heading_deg=%.1f yaw_error_deg=%.1f rc(roll=%d pitch=%d throttle=%d yaw=%d)",
            self._state.value,
            dist,
            bearing_to_target,
            snap.heading_deg,
            yaw_error,
            applied.roll,
            applied.pitch,
            applied.throttle,
            applied.yaw,
        )

        if dist <= self._cfg.geo_thresholds.navigate_arrival_radius_m:
            self._transition(MissionState.LAND_STUB, f"arrived in target radius: {dist:.1f}m")

    def _tick_land_stub(self, elapsed_s: float) -> None:
        neutral = self._rc.neutral_command()
        cmd = self._rc.apply(
            roll=neutral.roll,
            pitch=neutral.pitch,
            throttle=neutral.throttle,
            yaw=neutral.yaw,
        )
        self._logger.info(
            "state=%s stub_hold_s=%.1f rc(roll=%d pitch=%d throttle=%d yaw=%d)",
            self._state.value,
            elapsed_s,
            cmd.roll,
            cmd.pitch,
            cmd.throttle,
            cmd.yaw,
        )
        if elapsed_s >= self._cfg.mission_runtime.land_stub_hold_s:
            self._transition(MissionState.COMPLETE, "landing stub hold complete")

    def _build_navigation_command(self, distance_to_target_m: float, yaw_error_deg: float) -> RcCommand:
        neutral = self._rc.neutral_command()

        yaw_gain = 2.0
        yaw_delta = int(max(-200, min(200, yaw_error_deg * yaw_gain)))

        if distance_to_target_m > self._cfg.geo_thresholds.navigate_slowdown_radius_m:
            pitch_delta = 120
        elif distance_to_target_m > self._cfg.geo_thresholds.navigate_arrival_radius_m:
            pitch_delta = 70
        else:
            pitch_delta = 0

        return RcCommand(
            roll=neutral.roll,
            pitch=neutral.pitch + pitch_delta,
            throttle=neutral.throttle,
            yaw=neutral.yaw + yaw_delta,
        )

    def _get_timeout_s(self, state: MissionState) -> Optional[float]:
        cfg = self._cfg.mission_timeouts
        if state == MissionState.INIT:
            return cfg.init_s
        if state == MissionState.READY:
            return cfg.ready_s
        if state == MissionState.ARM_TAKEOFF_STUB:
            return cfg.arm_takeoff_stub_s
        if state == MissionState.NAVIGATE_STUB:
            return cfg.navigate_stub_s
        if state == MissionState.LAND_STUB:
            return cfg.land_stub_s
        return None

    def _transition(self, target: MissionState, reason: str) -> None:
        previous = self._state
        self._logger.info(
            "FSM transition: %s -> %s reason=%s elapsed_in_state=%.2fs",
            previous.value,
            target.value,
            reason,
            time.time() - self._state_entered_at,
        )
        self._state = target
        self._state_entered_at = time.time()

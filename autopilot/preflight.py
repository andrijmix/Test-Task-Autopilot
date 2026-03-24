from dataclasses import dataclass
from typing import List

from .dronekit_compat import ensure_dronekit_compat

ensure_dronekit_compat()

from dronekit import Vehicle

from .config import PreflightConfig


@dataclass(frozen=True)
class CheckResult:
    name: str
    ok: bool
    details: str


@dataclass(frozen=True)
class PreflightReport:
    checks: List[CheckResult]

    @property
    def ok(self) -> bool:
        return all(check.ok for check in self.checks)


def run_preflight_checks(vehicle: Vehicle, cfg: PreflightConfig) -> PreflightReport:
    checks = [
        _check_heartbeat(vehicle, cfg),
        _check_armable(vehicle, cfg),
        _check_mode_readable(vehicle),
        _check_gps_status(vehicle, cfg),
    ]
    return PreflightReport(checks=checks)


def _check_heartbeat(vehicle: Vehicle, cfg: PreflightConfig) -> CheckResult:
    age = vehicle.last_heartbeat
    if age is None:
        return CheckResult("heartbeat", False, "No heartbeat received yet")
    if age > cfg.heartbeat_max_age_s:
        return CheckResult(
            "heartbeat",
            False,
            f"Heartbeat too old: {age:.2f}s > {cfg.heartbeat_max_age_s:.2f}s",
        )
    return CheckResult("heartbeat", True, f"Heartbeat age: {age:.2f}s")


def _check_armable(vehicle: Vehicle, cfg: PreflightConfig) -> CheckResult:
    armable = bool(vehicle.is_armable)
    if cfg.require_armable and not armable:
        return CheckResult("armable", False, "Vehicle is not armable")
    return CheckResult("armable", True, f"is_armable={armable}")


def _check_mode_readable(vehicle: Vehicle) -> CheckResult:
    mode_name = getattr(vehicle.mode, "name", None)
    if not mode_name:
        return CheckResult("mode", False, "Vehicle mode is not readable")
    return CheckResult("mode", True, f"mode={mode_name}")


def _check_gps_status(vehicle: Vehicle, cfg: PreflightConfig) -> CheckResult:
    gps = vehicle.gps_0
    fix_type = getattr(gps, "fix_type", None)
    eph = getattr(gps, "eph", None)
    satellites_visible = getattr(gps, "satellites_visible", None)

    if fix_type is None:
        return CheckResult("gps", False, "GPS fix type is unavailable")

    if fix_type < cfg.min_gps_fix_type:
        return CheckResult(
            "gps",
            False,
            f"Insufficient GPS fix: {fix_type} < {cfg.min_gps_fix_type}",
        )

    return CheckResult(
        "gps",
        True,
        f"fix_type={fix_type}, eph={eph}, satellites={satellites_visible}",
    )

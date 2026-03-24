import logging
import time
from typing import Optional

from .dronekit_compat import ensure_dronekit_compat

ensure_dronekit_compat()

from dronekit import Vehicle

from .config import TelemetryConfig


def telemetry_snapshot(vehicle: Vehicle) -> dict:
    location = vehicle.location.global_relative_frame
    gps = vehicle.gps_0
    mode_name: Optional[str] = getattr(vehicle.mode, "name", None)

    return {
        "lat": location.lat,
        "lon": location.lon,
        "alt_m": location.alt,
        "mode": mode_name,
        "armed": bool(vehicle.armed),
        "gps_fix_type": getattr(gps, "fix_type", None),
        "gps_eph": getattr(gps, "eph", None),
        "gps_satellites": getattr(gps, "satellites_visible", None),
    }


def log_telemetry_session(vehicle: Vehicle, cfg: TelemetryConfig, logger: logging.Logger) -> None:
    end_time = time.time() + cfg.session_duration_s

    while time.time() < end_time:
        snap = telemetry_snapshot(vehicle)
        logger.info(
            "telemetry time=%s lat=%s lon=%s alt_m=%s mode=%s armed=%s gps_fix=%s gps_eph=%s gps_sat=%s",
            time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime()),
            _fmt_float(snap["lat"]),
            _fmt_float(snap["lon"]),
            _fmt_float(snap["alt_m"]),
            snap["mode"],
            snap["armed"],
            snap["gps_fix_type"],
            snap["gps_eph"],
            snap["gps_satellites"],
        )
        time.sleep(cfg.interval_s)


def _fmt_float(value: Optional[float]) -> str:
    if value is None:
        return "None"
    return f"{value:.6f}"

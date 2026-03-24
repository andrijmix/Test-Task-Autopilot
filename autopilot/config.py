from dataclasses import dataclass


@dataclass(frozen=True)
class GeoPoint:
    lat: float
    lon: float


@dataclass(frozen=True)
class MissionConfig:
    point_a: GeoPoint
    point_b: GeoPoint
    target_altitude_m: float


@dataclass(frozen=True)
class PreflightConfig:
    heartbeat_max_age_s: float = 5.0
    require_armable: bool = True
    min_gps_fix_type: int = 2


@dataclass(frozen=True)
class TelemetryConfig:
    interval_s: float = 1.0
    session_duration_s: float = 15.0


@dataclass(frozen=True)
class AppConfig:
    mission: MissionConfig
    preflight: PreflightConfig
    telemetry: TelemetryConfig


def build_default_config() -> AppConfig:
    return AppConfig(
        mission=MissionConfig(
            point_a=GeoPoint(lat=50.450739, lon=30.461242),
            point_b=GeoPoint(lat=50.443326, lon=30.448078),
            target_altitude_m=100.0,
        ),
        preflight=PreflightConfig(),
        telemetry=TelemetryConfig(),
    )

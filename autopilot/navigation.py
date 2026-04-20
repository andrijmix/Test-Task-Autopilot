"""Navigation abstraction layer.

Decouples FSMs from the position source (GPS vs. odometry) by providing a
common NavigationSnapshot and PositionProvider interface.  FSMs work entirely
in local NE coordinates (x=North, y=East, metres from origin) and never touch
lat/lon or MAVLink-specific fields directly.

Coordinate convention
─────────────────────
  x     North offset from origin [m]
  y     East  offset from origin [m]
  vx    North velocity [m/s]  (matches DroneKit vehicle.velocity[0])
  vy    East  velocity [m/s]  (matches DroneKit vehicle.velocity[1])
  bearing_2d(x1,y1, x2,y2) → 0°=North, 90°=East  (standard nav bearing)

Providers
─────────
  GpsPositionProvider      — DroneKit GPS, converts lat/lon to local NE frame
                             using point_a as origin.
  OdometryPositionProvider — DroneKit LOCAL_POSITION_NED (EKF odometry output);
                             target_x/target_y are supplied directly as local
                             NE coordinates.
"""

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple

from .dronekit_compat import ensure_dronekit_compat
from .geo import bearing_deg, distance_m

ensure_dronekit_compat()


# ---------------------------------------------------------------------------
# Snapshot
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class NavigationSnapshot:
    x: float            # North offset from origin [m]
    y: float            # East  offset from origin [m]
    alt_m: float        # altitude AGL [m]
    heading_deg: float  # true heading [0, 360)
    vx: float           # North velocity [m/s]
    vy: float           # East  velocity [m/s]
    armed: bool
    mode: Optional[str]


# ---------------------------------------------------------------------------
# Abstract interface
# ---------------------------------------------------------------------------


class PositionProvider(ABC):
    """Source-agnostic navigation data provider."""

    @abstractmethod
    def snapshot(self) -> Optional[NavigationSnapshot]:
        """Current navigation state, or None when position is unavailable."""
        ...

    @property
    @abstractmethod
    def target_x(self) -> float:
        """North coordinate of target B [m] in local frame."""
        ...

    @property
    @abstractmethod
    def target_y(self) -> float:
        """East coordinate of target B [m] in local frame."""
        ...


# ---------------------------------------------------------------------------
# GPS provider
# ---------------------------------------------------------------------------


class GpsPositionProvider(PositionProvider):
    """Converts DroneKit GPS data to local NE frame using point_a as origin.

    Heading correction (+180°) compensates for the inverted SITL magnetometer
    calibration so that heading_deg matches the actual nose direction.
    """

    _HEADING_OFFSET_DEG: float = 180.0

    def __init__(self, vehicle, cfg) -> None:
        self._vehicle = vehicle
        self._cfg = cfg
        self._tx, self._ty = self._to_local(cfg.mission.point_b.lat, cfg.mission.point_b.lon)

    # ------------------------------------------------------------------
    # PositionProvider interface
    # ------------------------------------------------------------------

    @property
    def target_x(self) -> float:
        return self._tx

    @property
    def target_y(self) -> float:
        return self._ty

    def snapshot(self) -> Optional[NavigationSnapshot]:
        location = self._vehicle.location.global_relative_frame
        lat = getattr(location, "lat", None)
        lon = getattr(location, "lon", None)
        if lat is None or lon is None:
            return None

        x, y = self._to_local(lat, lon)
        alt = getattr(location, "alt", None) or 0.0

        heading = getattr(self._vehicle, "heading", None)
        heading_deg = (float(heading) + self._HEADING_OFFSET_DEG) % 360.0 if heading is not None else 0.0

        vel = getattr(self._vehicle, "velocity", None)
        vx = float(vel[0]) if vel and len(vel) >= 1 else 0.0
        vy = float(vel[1]) if vel and len(vel) >= 2 else 0.0

        armed = bool(getattr(self._vehicle, "armed", False))
        mode = getattr(getattr(self._vehicle, "mode", None), "name", None)

        return NavigationSnapshot(
            x=x, y=y, alt_m=alt, heading_deg=heading_deg,
            vx=vx, vy=vy, armed=armed, mode=mode,
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        """Convert lat/lon to local NE offset (x=North, y=East) from point_a."""
        a = self._cfg.mission.point_a
        dist = distance_m(a.lat, a.lon, lat, lon)
        brng_rad = math.radians(bearing_deg(a.lat, a.lon, lat, lon))
        return dist * math.cos(brng_rad), dist * math.sin(brng_rad)


# ---------------------------------------------------------------------------
# Odometry provider
# ---------------------------------------------------------------------------


class OdometryPositionProvider(PositionProvider):
    """Reads position from MAVLink LOCAL_POSITION_NED (EKF odometry output).

    Target coordinates must be provided in the same local NE frame as the
    EKF origin (typically the arming location / home position).

    Heading correction (+180°) is applied for the same SITL compass reason
    as in GpsPositionProvider.
    """

    _HEADING_OFFSET_DEG: float = 180.0

    def __init__(self, vehicle, target_x: float, target_y: float) -> None:
        self._vehicle = vehicle
        self._tx = target_x
        self._ty = target_y

    # ------------------------------------------------------------------
    # PositionProvider interface
    # ------------------------------------------------------------------

    @property
    def target_x(self) -> float:
        return self._tx

    @property
    def target_y(self) -> float:
        return self._ty

    def snapshot(self) -> Optional[NavigationSnapshot]:
        local = getattr(getattr(self._vehicle, "location", None), "local_frame", None)
        if local is None:
            return None

        x = getattr(local, "north", None)
        y = getattr(local, "east", None)
        down = getattr(local, "down", None)
        if x is None or y is None:
            return None

        alt = -float(down) if down is not None else 0.0

        heading = getattr(self._vehicle, "heading", None)
        heading_deg = (float(heading) + self._HEADING_OFFSET_DEG) % 360.0 if heading is not None else 0.0

        vel = getattr(self._vehicle, "velocity", None)
        vx = float(vel[0]) if vel and len(vel) >= 1 else 0.0
        vy = float(vel[1]) if vel and len(vel) >= 2 else 0.0

        armed = bool(getattr(self._vehicle, "armed", False))
        mode = getattr(getattr(self._vehicle, "mode", None), "name", None)

        return NavigationSnapshot(
            x=float(x), y=float(y), alt_m=alt, heading_deg=heading_deg,
            vx=vx, vy=vy, armed=armed, mode=mode,
        )

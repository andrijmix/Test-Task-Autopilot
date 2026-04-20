import math


EARTH_RADIUS_M = 6371000.0


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in meters using the haversine formula."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)

    a = (
        math.sin(d_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2.0) ** 2
    )
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS_M * c


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing from point 1 to point 2 in degrees [0, 360)."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_lambda = math.radians(lon2 - lon1)

    y = math.sin(d_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(d_lambda)

    brng = math.degrees(math.atan2(y, x))
    return (brng + 360.0) % 360.0


def normalize_angle_deg(angle_deg: float) -> float:
    """Normalize angle to [-180, 180]."""
    normalized = (angle_deg + 180.0) % 360.0 - 180.0
    if normalized == -180.0:
        return 180.0
    return normalized


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """Euclidean distance between two points in local NE frame [m]."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def bearing_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """Bearing from (x1,y1) to (x2,y2) in local NE frame [0, 360). x=North, y=East."""
    angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
    return (angle + 360.0) % 360.0

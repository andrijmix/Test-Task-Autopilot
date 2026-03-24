from .dronekit_compat import ensure_dronekit_compat

ensure_dronekit_compat()

from dronekit import Vehicle, connect


def connect_vehicle(connect_string: str, baud: int = 57600, timeout: int = 60) -> Vehicle:
    """Connect to an autopilot and return a live DroneKit vehicle instance."""
    return connect(connect_string, wait_ready=True, timeout=timeout, baud=baud)

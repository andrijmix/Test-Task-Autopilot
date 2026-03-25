"""
Запуск SITL через dronekit-sitl (без зовнішнього ArduCopter.exe)
і виконання повної місії Stage 3 A→B.

Використання:
    python run_mission.py
"""
import os
import sys
import time
import subprocess
import dronekit_sitl

POINT_A_LAT = 50.450739
POINT_A_LON = 30.461242

def main() -> int:
    print("=== Starting SITL at Point A ===", flush=True)
    sitl = dronekit_sitl.start_default(
        lat=POINT_A_LAT,
        lon=POINT_A_LON,
    )
    conn = sitl.connection_string()
    print(f"SITL ready: {conn}", flush=True)

    # Wait for simulator to stabilize
    time.sleep(5)

    print(f"=== Running autopilot --sim-only -> {conn} ===", flush=True)
    try:
        result = subprocess.run(
            [sys.executable, "-m", "autopilot.main",
             "--connect", conn,
             "--sim-only"],
            env=os.environ.copy(),
        )
        return result.returncode
    finally:
        sitl.stop()
        print("=== SITL stopped ===", flush=True)


if __name__ == "__main__":
    sys.exit(main())

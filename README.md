# Copter Autopilot - Stage 1 (Mission Planner + DroneKit)

This stage provides a minimal and clean project skeleton for:
- reliable vehicle connection
- pre-flight checks
- short telemetry logging session

No takeoff, landing, RC control loops, or FSM are implemented at this stage.

## Mission Parameters

- Point A: `50.450739, 30.461242`
- Point B: `50.443326, 30.448078`
- Target altitude: `100 m`

These values are defined in `autopilot/config.py`.

## Project Structure

- `autopilot/config.py` - mission and runtime configuration
- `autopilot/connection.py` - DroneKit vehicle connection
- `autopilot/preflight.py` - pre-flight checks
- `autopilot/telemetry.py` - stable telemetry logger
- `autopilot/main.py` - CLI entry point for stage 1 run

## Setup

1. Install Python 3.10+.
2. Install dependencies:

```bash
pip install -r requirements.txt
```

## Run (SITL or real vehicle)

```bash
python -m autopilot.main --connect 127.0.0.1:14550
```

Or run the file directly:

```bash
python autopilot/main.py --connect 127.0.0.1:14550
```

Optional flags:

- `--baud 57600`
- `--telemetry-duration 20`
- `--telemetry-interval 1`

## Typical SITL start examples

If you already have ArduPilot SITL running and MAVProxy forwarding to `14550`, run only the Python command above.

Example with `sim_vehicle.py`:

```bash
sim_vehicle.py -v ArduCopter --console --map
```

Then run:

```bash
python -m autopilot.main --connect 127.0.0.1:14550
```

## Expected log format

```text
2026-03-24 12:00:00,000 | INFO | Mission config: A=(50.450739, 30.461242), B=(50.443326, 30.448078), target_alt_m=100.0
2026-03-24 12:00:00,200 | INFO | Connecting to vehicle: 127.0.0.1:14550
2026-03-24 12:00:01,100 | INFO | Connection established
2026-03-24 12:00:01,101 | INFO | preflight check=heartbeat ok=True details=Heartbeat age: 0.42s
2026-03-24 12:00:01,101 | INFO | preflight check=armable ok=True details=is_armable=True
2026-03-24 12:00:01,101 | INFO | preflight check=mode ok=True details=mode=STABILIZE
2026-03-24 12:00:01,101 | INFO | preflight check=gps ok=True details=fix_type=3, eph=120, satellites=10
2026-03-24 12:00:01,102 | INFO | Starting telemetry session: duration=15.0s interval=1.0s
2026-03-24 12:00:01,102 | INFO | telemetry time=2026-03-24T09:00:01 lat=50.450739 lon=30.461242 alt_m=0.000000 mode=STABILIZE armed=False gps_fix=3 gps_eph=120 gps_sat=10
2026-03-24 12:00:16,110 | INFO | Telemetry session finished
```

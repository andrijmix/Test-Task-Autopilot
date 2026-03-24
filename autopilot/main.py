import argparse
import logging
import sys
from dataclasses import replace
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from autopilot.config import build_default_config
    from autopilot.connection import connect_vehicle
    from autopilot.preflight import run_preflight_checks
    from autopilot.telemetry import log_telemetry_session
else:
    from .config import build_default_config
    from .connection import connect_vehicle
    from .preflight import run_preflight_checks
    from .telemetry import log_telemetry_session


def configure_logging() -> logging.Logger:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )
    return logging.getLogger("autopilot")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Autopilot Stage 1: connect, preflight checks, telemetry logging"
    )
    parser.add_argument("--connect", required=True, help="Vehicle connection string")
    parser.add_argument("--baud", type=int, default=57600, help="Connection baud rate")
    parser.add_argument(
        "--telemetry-duration",
        type=float,
        default=None,
        help="Override telemetry session duration in seconds",
    )
    parser.add_argument(
        "--telemetry-interval",
        type=float,
        default=None,
        help="Override telemetry log interval in seconds",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logger = configure_logging()
    cfg = build_default_config()

    if args.telemetry_duration is not None:
        cfg = replace(
            cfg,
            telemetry=replace(cfg.telemetry, session_duration_s=args.telemetry_duration),
        )

    if args.telemetry_interval is not None:
        cfg = replace(cfg, telemetry=replace(cfg.telemetry, interval_s=args.telemetry_interval))

    vehicle = None
    try:
        logger.info(
            "Mission config: A=(%.6f, %.6f), B=(%.6f, %.6f), target_alt_m=%.1f",
            cfg.mission.point_a.lat,
            cfg.mission.point_a.lon,
            cfg.mission.point_b.lat,
            cfg.mission.point_b.lon,
            cfg.mission.target_altitude_m,
        )
        logger.info("Connecting to vehicle: %s", args.connect)
        vehicle = connect_vehicle(args.connect, baud=args.baud)
        logger.info("Connection established")

        report = run_preflight_checks(vehicle, cfg.preflight)
        for check in report.checks:
            level = logging.INFO if check.ok else logging.ERROR
            logger.log(level, "preflight check=%s ok=%s details=%s", check.name, check.ok, check.details)

        if not report.ok:
            logger.error("Pre-flight checks failed. Telemetry session aborted.")
            return 2

        logger.info(
            "Starting telemetry session: duration=%.1fs interval=%.1fs",
            cfg.telemetry.session_duration_s,
            cfg.telemetry.interval_s,
        )
        log_telemetry_session(vehicle, cfg.telemetry, logger)
        logger.info("Telemetry session finished")
        return 0
    except Exception as exc:
        logger.exception("Fatal error: %s", exc)
        return 1
    finally:
        if vehicle is not None:
            vehicle.close()
            logger.info("Vehicle connection closed")


if __name__ == "__main__":
    sys.exit(main())

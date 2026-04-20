import argparse
import logging
import math
import sys
from dataclasses import replace
from datetime import datetime
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from autopilot.config import Stage3TimeoutsConfig, build_default_config
    from autopilot.connection import connect_vehicle
    from autopilot.geo import bearing_deg, distance_m
    from autopilot.mission_fsm import MissionFSM, MissionState
    from autopilot.navigation import GpsPositionProvider, OdometryPositionProvider, PositionProvider
    from autopilot.preflight import run_odometry_preflight_checks, run_preflight_checks
    from autopilot.rc_override import DryRunRcOverrideAdapter, RcOverrideAdapter
    from autopilot.stage3_fsm import Stage3FSM, Stage3State
else:
    from .config import Stage3TimeoutsConfig, build_default_config
    from .connection import connect_vehicle
    from .geo import bearing_deg, distance_m
    from .mission_fsm import MissionFSM, MissionState
    from .navigation import GpsPositionProvider, OdometryPositionProvider, PositionProvider
    from .preflight import run_odometry_preflight_checks, run_preflight_checks
    from .rc_override import DryRunRcOverrideAdapter, RcOverrideAdapter
    from .stage3_fsm import Stage3FSM, Stage3State


def configure_logging() -> logging.Logger:
    log_dir = Path.cwd() / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"autopilot-{datetime.now().strftime('%Y%m%d-%H%M%S')}.log"

    formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    root_logger.handlers.clear()

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)

    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(formatter)

    root_logger.addHandler(stream_handler)
    root_logger.addHandler(file_handler)

    logger = logging.getLogger("autopilot")
    logger.info("Logging to file: %s", log_path)
    return logger


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Autopilot: preflight + FSM mission in Stabilize (Stage 2 stub / Stage 3 SITL)"
    )
    parser.add_argument("--connect", required=True, help="Vehicle connection string")
    parser.add_argument("--baud", type=int, default=57600, help="Connection baud rate")
    parser.add_argument(
        "--nav-source",
        choices=["gps", "odometry"],
        default="gps",
        help="Navigation source: 'gps' (default) or 'odometry' (LOCAL_POSITION_NED from EKF). "
             "Use 'odometry' when GPS is jammed.",
    )
    parser.add_argument(
        "--target-lat",
        type=float,
        default=None,
        metavar="DEG",
        help="[odometry mode] GPS latitude of target B. Converted to local NE offset using point_a as origin.",
    )
    parser.add_argument(
        "--target-lon",
        type=float,
        default=None,
        metavar="DEG",
        help="[odometry mode] GPS longitude of target B. Converted to local NE offset using point_a as origin.",
    )
    parser.add_argument(
        "--target-x",
        type=float,
        default=None,
        metavar="METRES",
        help="[odometry mode] North offset of target B from EKF origin [m]. Alternative to --target-lat/lon.",
    )
    parser.add_argument(
        "--target-y",
        type=float,
        default=None,
        metavar="METRES",
        help="[odometry mode] East offset of target B from EKF origin [m]. Alternative to --target-lat/lon.",
    )
    parser.add_argument(
        "--sim-only",
        action="store_true",
        help="[REQUIRED for Stage 3] Confirm this is a SITL simulation run. "
             "Enables Stage 3 FSM: real takeoff, altitude hold, A→B flight, landing.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute and log RC commands but do NOT send them to the vehicle.",
    )
    # Stage 2 stub overrides (backward-compatible)
    parser.add_argument(
        "--navigate-stub-timeout",
        type=float,
        default=None,
        metavar="SECONDS",
        help="Override Stage 2 FSM NAVIGATE_STUB timeout (useful for tests)",
    )
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
    # Stage 3 per-state timeout overrides (useful for tests)
    parser.add_argument(
        "--stage3-takeoff-timeout",
        type=float,
        default=None,
        metavar="SECONDS",
        help="Override Stage 3 TAKEOFF state timeout",
    )
    parser.add_argument(
        "--stage3-enroute-timeout",
        type=float,
        default=None,
        metavar="SECONDS",
        help="Override Stage 3 ENROUTE_TO_B state timeout",
    )
    parser.add_argument(
        "--stage3-approach-timeout",
        type=float,
        default=None,
        metavar="SECONDS",
        help="Override Stage 3 APPROACH_FINE state timeout",
    )
    parser.add_argument(
        "--stage3-landing-timeout",
        type=float,
        default=None,
        metavar="SECONDS",
        help="Override Stage 3 LANDING state timeout",
    )
    return parser.parse_args()


def _apply_overrides(cfg, args):
    """Return a new AppConfig with CLI overrides applied."""
    if args.telemetry_duration is not None:
        cfg = replace(cfg, telemetry=replace(cfg.telemetry, session_duration_s=args.telemetry_duration))

    if args.telemetry_interval is not None:
        cfg = replace(cfg, telemetry=replace(cfg.telemetry, interval_s=args.telemetry_interval))

    if args.navigate_stub_timeout is not None:
        cfg = replace(
            cfg,
            mission_timeouts=replace(cfg.mission_timeouts, navigate_stub_s=args.navigate_stub_timeout),
        )

    # Stage 3 timeout overrides
    s3_timeout_changes = {}
    if args.stage3_takeoff_timeout is not None:
        s3_timeout_changes["takeoff_s"] = args.stage3_takeoff_timeout
    if args.stage3_enroute_timeout is not None:
        s3_timeout_changes["enroute_s"] = args.stage3_enroute_timeout
    if args.stage3_approach_timeout is not None:
        s3_timeout_changes["approach_s"] = args.stage3_approach_timeout
    if args.stage3_landing_timeout is not None:
        s3_timeout_changes["landing_s"] = args.stage3_landing_timeout

    if s3_timeout_changes:
        cfg = replace(
            cfg,
            stage3=replace(
                cfg.stage3,
                timeouts=replace(cfg.stage3.timeouts, **s3_timeout_changes),
            ),
        )

    return cfg


def _build_position_provider(args, vehicle, cfg) -> PositionProvider:
    if args.nav_source == "odometry":
        if args.target_lat is not None and args.target_lon is not None:
            # Convert GPS coords to local NE offset using point_a as EKF origin
            a = cfg.mission.point_a
            dist = distance_m(a.lat, a.lon, args.target_lat, args.target_lon)
            brng_rad = math.radians(bearing_deg(a.lat, a.lon, args.target_lat, args.target_lon))
            target_x = dist * math.cos(brng_rad)
            target_y = dist * math.sin(brng_rad)
        elif args.target_x is not None and args.target_y is not None:
            target_x, target_y = args.target_x, args.target_y
        else:
            raise SystemExit(
                "ERROR: --nav-source odometry requires either:\n"
                "  --target-lat LAT --target-lon LON  (GPS coords of target B)\n"
                "  --target-x M --target-y M           (local NE offset in metres)"
            )
        return OdometryPositionProvider(vehicle, target_x=target_x, target_y=target_y)
    return GpsPositionProvider(vehicle, cfg)


def main() -> int:
    args = parse_args()
    logger = configure_logging()
    cfg = build_default_config()
    cfg = _apply_overrides(cfg, args)

    logger.info(
        "Mission config: A=(%.6f, %.6f), B=(%.6f, %.6f), target_alt_m=%.1f",
        cfg.mission.point_a.lat,
        cfg.mission.point_a.lon,
        cfg.mission.point_b.lat,
        cfg.mission.point_b.lon,
        cfg.mission.target_altitude_m,
    )

    # ---------------------------------------------------------------
    # Stage 3 SITL guard
    # ---------------------------------------------------------------
    if not args.sim_only:
        logger.info("Running Stage 2 stub FSM (no --sim-only flag). "
                    "Pass --sim-only to enable Stage 3 controlled flight.")
    else:
        logger.info(
            "Stage 3 SIMULATION-ONLY mode enabled. "
            "Controlled takeoff/flight/landing in SITL. dry_run=%s",
            args.dry_run,
        )

    vehicle = None
    try:
        logger.info("Connecting to vehicle: %s", args.connect)
        vehicle = connect_vehicle(args.connect, baud=args.baud)
        logger.info("Connection established")

        nav_source = args.nav_source
        logger.info("Navigation source: %s", nav_source)

        if nav_source == "odometry":
            report = run_odometry_preflight_checks(vehicle, cfg.preflight)
        else:
            report = run_preflight_checks(vehicle, cfg.preflight)
        for check in report.checks:
            level = logging.INFO if check.ok else logging.ERROR
            logger.log(level, "preflight check=%s ok=%s details=%s", check.name, check.ok, check.details)

        if not report.ok:
            logger.error("Pre-flight checks failed. Mission aborted before FSM start.")
            return 2

        pos_provider = _build_position_provider(args, vehicle, cfg)
        logger.info("PositionProvider: %s  target=(%.2f, %.2f) m",
                    type(pos_provider).__name__, pos_provider.target_x, pos_provider.target_y)

        if args.dry_run:
            logger.info("DRY-RUN mode: RC override commands will be logged only")
            rc_adapter = DryRunRcOverrideAdapter(cfg.rc_override)
        else:
            logger.info("NORMAL mode: RC override commands will be sent to vehicle")
            rc_adapter = RcOverrideAdapter(vehicle, cfg.rc_override)

        # ---------------------------------------------------------------
        # Stage 3 FSM (SITL controlled flight)
        # ---------------------------------------------------------------
        if args.sim_only:
            fsm3 = Stage3FSM(
                vehicle=vehicle,
                cfg=cfg,
                rc_adapter=rc_adapter,
                logger=logger,
                dry_run=args.dry_run,
                pos_provider=pos_provider,
            )
            result3 = fsm3.run()
            if result3 == Stage3State.COMPLETE:
                logger.info("Stage3 mission finished: COMPLETE")
                return 0
            logger.error("Stage3 mission finished: ABORT")
            return 3

        # ---------------------------------------------------------------
        # Stage 2 stub FSM (no --sim-only)
        # ---------------------------------------------------------------
        fsm = MissionFSM(vehicle=vehicle, cfg=cfg, rc_adapter=rc_adapter, logger=logger,
                         pos_provider=pos_provider)
        result = fsm.run()

        if result == MissionState.COMPLETE:
            logger.info("Mission finished with COMPLETE state")
            return 0

        logger.error("Mission finished with ABORT state")
        return 3

    except Exception as exc:
        logger.exception("Fatal error: %s", exc)
        return 1
    finally:
        if vehicle is not None:
            vehicle.close()
            logger.info("Vehicle connection closed")


if __name__ == "__main__":
    sys.exit(main())


"""Command-line entry point for ascent-only and full-mission runs."""

import argparse
import logging
import os
import sys
from dataclasses import replace

from rlv_sim._run_full_mission import run_full_mission
from rlv_sim.config_factory import create_default_config, create_demo_config
from rlv_sim.config_io import load_config_json
from rlv_sim.main import run_simulation

from . import constants as C

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def parse_args(argv=None):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="RLV full-mission 6-DOF physics simulation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--profile",
        choices=("research", "demo"),
        default="research",
        help=(
            "Run profile: 'research' is the validated default; 'demo' uses a coarser "
            "timestep and coast time-warp for faster live demonstrations."
        ),
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Shortcut for --profile demo (faster web-style demonstration run).",
    )
    parser.add_argument(
        "--config-json",
        type=str,
        default=None,
        help="Path to a JSON file containing SimulationConfig overrides",
    )
    parser.add_argument(
        "--output-dir", "-o",
        type=str,
        default="plots",
        help="Directory to save output plots, telemetry, and run manifest"
    )
    parser.add_argument(
        "--quiet", "-q",
        action="store_true",
        help="Suppress verbose output"
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Skip plot generation"
    )
    parser.add_argument(
        "--no-telemetry",
        action="store_true",
        help="Skip CSV telemetry export"
    )
    parser.add_argument(
        "--mission",
        choices=("full", "ascent"),
        default="full",
        help="Simulation mode to run"
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=None,
        help="Simulation timestep in seconds (profile value when omitted)"
    )
    parser.add_argument(
        "--max-time",
        type=float,
        default=None,
        help="Override the maximum simulation time in seconds"
    )
    parser.add_argument(
        "--nav-mode",
        choices=("truth", "onboard"),
        default="truth",
        help=(
            "Guidance navigation source: truth uses exact simulator state; "
            "onboard enables GPS, IMU propagation, landing altimeter, and "
            "guidance from the estimated state"
        ),
    )
    parser.add_argument(
        "--reuse-stage2",
        action="store_true",
        help=(
            "Recover Stage 2 too: after reaching orbit it holds, deorbits, "
            "re-enters, and makes a propulsive drone-ship landing. Extends the "
            "max time automatically if not set explicitly."
        ),
    )
    parser.add_argument(
        "--s2-orbit-hold",
        type=float,
        default=None,
        help=(
            "Seconds Stage 2 coasts in orbit before deorbiting (with "
            "--reuse-stage2). One orbital period is ~5534 s, so use ~5534 for "
            "one full revolution. Max time is extended automatically to fit."
        ),
    )
    parser.add_argument(
        "--enable-abort-modes",
        action="store_true",
        help="Arm in-flight abort monitoring",
    )
    parser.add_argument(
        "--disable-abort-modes",
        action="store_true",
        help="Explicitly disable abort monitoring",
    )
    parser.add_argument(
        "--validation-report",
        type=str,
        default=None,
        help=(
            "Path to a reference validation_report.json file. "
            "If provided, the CLI will compare the current run against it."
        ),
    )
    parser.add_argument(
        "--attitude-controller",
        choices=("pd", "pid"),
        default=None,
        help="Attitude controller for ascent/orbit/recovery (default: pd)",
    )
    parser.add_argument(
        "--landing-guidance",
        choices=["heuristic", "gfold"],
        default=None,
        help="Booster landing-burn guidance: heuristic suicide burn (default) "
             "or G-FOLD convex minimum-fuel descent (needs cvxpy).",
    )
    parser.add_argument(
        "--recovery-attitude-controller",
        choices=("pd", "pid", "auto"),
        default=None,
        help=(
            "Booster terminal-landing controller (default: auto = pid during "
            "BOOSTER_LANDING only; ascent stays pd)"
        ),
    )
    return parser.parse_args(argv)


def build_config_from_args(args) -> object:
    """Build a validated simulation config from CLI arguments.
    Only the 'research' profile (best current profile) is supported.
    """
    use_demo = getattr(args, "demo", False) or getattr(args, "profile", "research") == "demo"
    if getattr(args, "config_json", None):
        config = load_config_json(args.config_json)
    elif use_demo:
        config = create_demo_config()
    else:
        config = create_default_config(dt=0.05)

    if use_demo and not getattr(args, "config_json", None):
        config = replace(
            config,
            enable_demo_mode=True,
            dt=0.25 if args.dt is None else config.dt,
        )

    if args.max_time is not None:
        config = replace(config, max_time=args.max_time)
    if args.dt is not None:
        config = replace(config, dt=args.dt)
    if args.nav_mode == "onboard":
        config = replace(
            config,
            enable_gps=True,
            enable_imu=True,
            enable_landing_altimeter=True,
            use_navigation_estimate_for_guidance=True,
        )
    if getattr(args, "disable_abort_modes", False):
        config = replace(config, enable_abort_modes=False)
    elif getattr(args, "enable_abort_modes", False):
        config = replace(config, enable_abort_modes=True)
    if getattr(args, "attitude_controller", None) is not None:
        config = replace(config, attitude_controller=args.attitude_controller)
    if getattr(args, "recovery_attitude_controller", None) is not None:
        config = replace(
            config,
            recovery_attitude_controller=args.recovery_attitude_controller,
        )
    if getattr(args, "landing_guidance", None) is not None:
        config = replace(config, booster_landing_guidance=args.landing_guidance)
    if getattr(args, "reuse_stage2", False):
        config = replace(config, enable_s2_recovery=True)
        # NOTE: a stage2_prop_mass increase + s2_landing_propellant_reserve_kg
        # floor was tried here and reverted -- it reproduced the exact
        # failure config_definition.py's s2_landing_propellant_reserve_kg
        # docstring already documents: this stage's sub-1.0 T/W means added
        # propellant mass increases gravity losses during insertion, eating
        # the added headroom, so insertion itself aborted at the reserve
        # floor (reentered: perigee=-233.5km) instead of reaching orbit.
        # See that docstring before changing this again -- a real fix needs
        # a thrust increase or more efficient guidance, not just more fuel.
        hold_s = getattr(args, "s2_orbit_hold", None)
        if hold_s is not None:
            config = replace(config, s2_orbit_hold_time_s=float(hold_s))
        if args.max_time is None:
            needed = (
                C.S2_COAST_TO_INSERTION_S
                + float(config.s2_orbit_hold_time_s)
                + C.S2_RECOVERY_DESCENT_S
            )
            config = replace(
                config,
                max_time=max(
                    config.max_time,
                    C.S2_RECOVERY_MIN_MAX_TIME_S,
                    needed,
                ),
            )
    config.validate()
    return config


def _resolve_output_dir(args) -> str:
    if os.path.isabs(args.output_dir):
        return args.output_dir
    return os.path.join(os.getcwd(), args.output_dir)


def main():
    """Main execution flow."""
    args = parse_args()

    if args.quiet:
        logger.setLevel(logging.WARNING)

    print(f"\n{'='*70}\nBOOSTBACK — FULL-MISSION SIMULATION RUN\n{'='*70}\n")

    exit_code = 0
    try:
        config = build_config_from_args(args)
        plot_dir = _resolve_output_dir(args)
        print(f">> Navigation mode: {args.nav_mode}")
        print(f">> Attitude controller: {config.attitude_controller}")
        print(f">> Recovery attitude controller: {config.recovery_attitude_controller}")

        assessment = None
        mission = None
        termination = {}

        if args.mission == "full":
            from rlv_sim.mission_summary import assess_full_mission

            logger.info("Starting full mission simulation...")
            print(">> Running Full Mission 6-DOF Physics Engine...")
            mission = run_full_mission(
                config=config,
                verbose=not args.quiet,
            )
            assessment = assess_full_mission(mission, config)
            termination = {
                "ascent": mission.ascent_reason,
                "booster": mission.booster_reason,
                "orbiter": mission.orbiter_reason,
            }
            separation_text = (
                f"{mission.separation_time:.2f} s"
                if mission.separation_time is not None
                else "not reached"
            )
            duration_text = (
                f"{assessment.mission_duration_s:.1f} s "
                f"({int(assessment.mission_duration_s // 60)}m"
                f"{int(assessment.mission_duration_s % 60):02d}s)"
                if assessment.mission_duration_s is not None
                else "n/a"
            )

            print("\n" + "=" * 60)
            print("MISSION SUMMARY")
            print("=" * 60)
            print(f"  Stage separation : {separation_text}")
            print(f"  Mission duration : {duration_text}")
            print(f"  Ascent           : {mission.ascent_reason}")
            print(f"  Stage 1 (booster): {mission.booster_reason}")
            print(f"  Stage 2 (orbiter): {mission.orbiter_reason}")
            if assessment.s2_recovery is not None:
                s2 = assessment.s2_recovery
                print(
                    f"     orbit reached : {s2['orbit_achieved']} "
                    f"({s2['achieved_perigee_km']}x{s2['achieved_apogee_km']} km), "
                    f"touchdown {s2['touchdown_speed_mps']} m/s, "
                    f"{s2['downrange_km']:.0f} km downrange, "
                    f"{s2['fuel_remaining_kg']:.0f} kg fuel left"
                )
            print(f"  OVERALL STATUS   : {assessment.status.upper()}")
            if assessment.failed_criteria:
                print(f"  Failed gates     : {', '.join(assessment.failed_criteria)}")
            print("=" * 60 + "\n")

            if not assessment.mission_success:
                exit_code = 1
        else:
            logger.info("Starting ascent simulation...")
            print(">> Running Ascent Simulation Physics Engine...")
            final_state, _log, reason = run_simulation(
                config=config,
                verbose=not args.quiet,
            )
            termination = {"ascent": reason}
            print("\n" + "=" * 60)
            print("SIMULATION SUMMARY")
            print("=" * 60)
            print(f"Termination reason: {reason}")
            print(f"Final time: {final_state.t:.2f} s")
            print(f"Final altitude: {final_state.altitude/1000:.2f} km")
            print(f"Final velocity: {final_state.speed:.2f} m/s")
            print("=" * 60 + "\n")

        artifacts: dict[str, str] = {}
        if args.mission == "full" and mission is not None:
            if not args.no_telemetry:
                from datetime import datetime
                from pathlib import Path

                folder = Path(plot_dir) / f"mission_{datetime.now().astimezone().strftime('%Y%m%d_%H%M%S')}"
                folder.mkdir(parents=True, exist_ok=True)
                if mission.ascent_log.get_series("time"):
                    mission.ascent_log.to_csv(folder / "ascent_telemetry.csv")
                if mission.booster_log.get_series("time"):
                    mission.booster_log.to_csv(folder / "booster_recovery_telemetry.csv")
                if mission.orbiter_log.get_series("time"):
                    mission.orbiter_log.to_csv(folder / "orbiter_telemetry.csv")
                artifacts["telemetry_dir"] = str(folder)

            if not args.no_plots:
                from rlv_sim.mission_summary import write_mission_summary

                from ._plotting_mission import (
                    generate_all_plots,
                    generate_mission_segment_plots,
                    write_plot_manifest,
                )

                logger.info(f"Generating plots in {plot_dir}")
                print(f">> Generating Plots in: {plot_dir}")
                saved_files = []
                saved_files.extend(generate_all_plots(mission.ascent_log, plot_dir))
                if mission.separation_time is not None:
                    saved_files.extend(
                        generate_mission_segment_plots(
                            mission.ascent_log,
                            mission.orbiter_log,
                            mission.booster_log,
                            mission.separation_time,
                            plot_dir,
                            config=config,
                        )
                    )
                manifest_path = write_plot_manifest(saved_files, plot_dir)
                artifacts["plot_manifest"] = manifest_path
                summary_files = write_mission_summary(mission, config, plot_dir)
                artifacts["mission_summary"] = ", ".join(summary_files)
                print("\n" + "=" * 70)
                print("Artifacts generated successfully.")
                print(f"Generated {len(saved_files)} plot files.")
                print(f"Plot manifest: {manifest_path}")
                if summary_files:
                    print(f"Mission summary: {', '.join(summary_files)}")
                print(f"Check outputs in: {plot_dir}")
                print("=" * 70)
        if config.enable_run_manifest and args.mission == "full" and mission is not None:
            from dataclasses import asdict

            from rlv_sim.run_manifest import build_run_manifest, write_run_manifest

            run_manifest = build_run_manifest(
                config,
                cli_argv=sys.argv,
                output_dir=plot_dir,
                artifacts=artifacts,
                assessment=asdict(assessment) if assessment is not None else None,
                termination=termination,
            )
            manifest_path = write_run_manifest(run_manifest, plot_dir)
            print(f">> Run manifest: {manifest_path}")

            if args.validation_report is not None:
                from rlv_sim.report_io import compare_assessments, load_run_report

                print(f">> Comparing current mission against reference report: {args.validation_report}")
                reference = load_run_report(args.validation_report)
                current = load_run_report(manifest_path)
                comparison = compare_assessments(reference, current)
                print(f"Validation comparison overall pass: {comparison['overall_pass']}")
                for key, diff in comparison["comparison"].items():
                    status = "PASS" if diff["passes"] else "FAIL"
                    print(
                        f"  {key}: {status} "
                        f"(ref={diff['reference']} cand={diff['candidate']} "
                        f"diff={diff['difference']:.2f} tol={diff['threshold']:.2f})"
                    )
                for key, diff in comparison["status_comparison"].items():
                    status = "PASS" if diff["passes"] else "FAIL"
                    print(
                        f"  {key}: {status} "
                        f"(ref={diff['reference']} cand={diff['candidate']})"
                    )
                if comparison["missing_metrics"]:
                    print(
                        "  Missing required metrics: "
                        + ", ".join(comparison["missing_metrics"])
                    )

        if exit_code != 0:
            print("\n[FAILED] Mission did not pass configured success gates.")
            sys.exit(exit_code)

    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")
        sys.exit(130)
    except (ValueError, RuntimeError, OSError) as e:
        logger.exception("Simulation failed")
        print(f"\n[ERROR] Simulation failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

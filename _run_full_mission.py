"""Full-mission orchestrator: stacked ascent plus dual-vehicle lockstep."""

from __future__ import annotations

import dataclasses
import logging
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Literal

import numpy as np

from . import _gfold
from . import constants as C
from ._guidance_common import compute_local_vertical, create_guidance_state
from ._main_models import FullMissionResult, SimulationLog
from ._mission_state import (
    _compute_separation_adjustments,
    _configured_max_mass,
    _create_configured_initial_state,
    _inherit_separation_state,
    _remaining_step_dt,
)
from ._run_simulation import _print_status, _update_energy_validation_tracker
from ._simulation_step import (
    _booster_landing_success,
    _check_runtime_safety_limits,
    _EnergyValidationTracker,
    _orbiter_outcome_success,
    check_termination,
    simulation_step,
)
from .abort import AbortMonitor
from .actuator import ActuatorState
from .config_definition import SimulationConfig
from .config_factory import create_default_config
from .mission_manager import MissionManager, MissionPhase
from .mission_summary import format_orbit_failure_reason
from .physics_checks import ValidationError, validate_state
from .state import State
from .utils import surface_relative_speed

ControlSignal = Literal["run", "paused", "stop"]
ControlCallback = Callable[[], ControlSignal]


@dataclass(frozen=True)
class MissionProgress:
    """One observable full-mission runtime update."""

    vehicle: Literal["ascent", "dual"]
    primary_state: State
    primary_guidance: dict
    phase: str
    booster_state: State | None = None
    booster_guidance: dict | None = None


ProgressCallback = Callable[[MissionProgress], None]

logger = logging.getLogger(__name__)


def _step_vehicle(state, actuator, mgr, gs, log, abort_monitor, config, max_time,
                  *, dry_mass, stage, vehicle_model, request_dt, wind_cfg,
                  max_time_reason):
    """Advance one vehicle one step; return (state, actuator, gs, done, reason, step_dt, guidance)."""
    term, reason = check_termination(state, max_time, mgr)
    if term:
        return state, actuator, gs, True, reason, None, None
    try:
        validate_state(
            state,
            dry_mass=dry_mass,
            quaternion_norm_tolerance=config.quaternion_norm_tolerance,
        )
    except ValidationError as e:
        logger.error(f"{vehicle_model.capitalize()} validation failed: {e}")
        return state, actuator, gs, True, f"Validation failure: {e}", None, None
    step_dt = _remaining_step_dt(state, request_dt, max_time)
    if step_dt <= 0.0:
        return state, actuator, gs, True, max_time_reason, None, None

    if vehicle_model == "booster":
        step_dt = min(step_dt, mgr.recovery_max_step_dt())
    mgr.update(state, step_dt)
    new_state, guid, ctrl, new_actuator, new_gs = simulation_step(
        state, actuator, mgr, step_dt,
        dry_mass=dry_mass, stage=stage,
        vehicle_model=vehicle_model, gs=gs,
        config=wind_cfg,
    )
    log.append(new_state, guid, ctrl)
    safety_reason = _check_runtime_safety_limits(
        new_state, guid, ctrl, wind_cfg,
        abort_monitor, current_phase=mgr.get_phase(),
    )
    if safety_reason is not None:
        return new_state, new_actuator, new_gs, True, safety_reason, step_dt, guid
    return new_state, new_actuator, new_gs, False, "", step_dt, guid


def _make_wind_applier(config: SimulationConfig) -> Callable[[SimulationConfig], SimulationConfig]:
    """Return a per-step function that adds seeded turbulence/gusts to a config."""
    if not config.enable_stochastic_wind:
        return lambda cfg: cfg
    if config.wind_seed is not None:
        wind_seed = int(config.wind_seed)
    elif config.gps_seed is not None:
        # Offset from gps_seed by 3, matching the existing IMU (+1) /
        # landing altimeter (+2) derived-seed pattern, so wind noise is
        # not simply identical to the raw GPS noise seed by coincidence.
        wind_seed = int(config.gps_seed) + 3
    else:
        wind_seed = 0
    wind_rng = np.random.default_rng(wind_seed)

    def apply_wind(cfg: SimulationConfig) -> SimulationConfig:
        turbulence = float(wind_rng.normal(0.0, cfg.wind_turbulence_intensity))
        gust = float(cfg.wind_gust_magnitude) * float(wind_rng.binomial(1, 0.01))
        return dataclasses.replace(
            cfg,
            runtime_wind_offset_mps=cfg.runtime_wind_offset_mps + turbulence + gust,
        )

    return apply_wind


def _make_abort_monitor(config: SimulationConfig) -> AbortMonitor | None:
    if not config.enable_abort_modes:
        return None
    return AbortMonitor(
        q_alpha_threshold=config.abort_q_alpha_threshold,
        attitude_threshold_deg=config.abort_attitude_threshold,
    )


def _separation_time(mission_mgr: MissionManager, state) -> float:
    if mission_mgr.separation_time is not None:
        return mission_mgr.separation_time
    return state.t


def _print_dual_status(orbiter_state, orbiter_done, orbiter_reason, orb_guid,
                       booster_state, booster_done, booster_reason, bst_guid) -> None:
    orb_alt = orbiter_state.altitude / 1000 if not orbiter_done else -1
    orb_vel = orbiter_state.speed if not orbiter_done else -1
    bst_alt = booster_state.altitude / 1000 if not booster_done else -1
    bst_vel = booster_state.speed if not booster_done else -1
    orb_phase = orb_guid.get('phase', '?') if not orbiter_done else orbiter_reason[:18]
    bst_phase = bst_guid.get('phase', '?') if not booster_done else booster_reason[:18]
    t_now = max(
        orbiter_state.t if not orbiter_done else 0,
        booster_state.t if not booster_done else 0
    )
    print(f"{t_now:10.1f} | {orb_alt:10.1f} | {orb_vel:10.1f} | "
          f"{bst_alt:10.1f} | {bst_vel:10.1f} | {orb_phase:^18} | {bst_phase:^18}")


def _print_mission_summary(separation_time, orbiter_state, orbiter_reason,
                           booster_state, booster_reason, config, step_count, elapsed) -> None:
    print("\n" + "=" * 90)
    print("FULL MISSION SUMMARY")
    print("=" * 90)
    print(f"Separation:    t={separation_time:.2f}s")
    print(f"Orbiter:       {orbiter_reason}")
    print(f"  Final Alt:   {orbiter_state.altitude/1000:.1f} km | V={orbiter_state.speed:.0f} m/s")
    print(f"Booster:       {booster_reason}")
    booster_v_display = surface_relative_speed(booster_state, config)
    print(f"  Final Alt:   {booster_state.altitude/1000:.1f} km | V={booster_v_display:.0f} m/s (surface-relative)")
    print(f"Total Steps:   {step_count:,} | Wall Time: {elapsed:.2f}s")
    print("=" * 90)


# Orbiter phases in which unpowered arcs may use orbit_coast_max_dt.
_ORBIT_COAST_PHASES = (
    MissionPhase.ORBIT_INSERTION,
    MissionPhase.ORBIT_ACHIEVED,
    MissionPhase.S2_ORBIT_HOLD,
)


@dataclasses.dataclass(frozen=True)
class _MissionRuntime:
    """Settings and callbacks shared by the ascent and dual-vehicle phases."""

    config: SimulationConfig
    dt: float
    max_time: float
    verbose: bool
    apply_wind: Callable[[SimulationConfig], SimulationConfig]
    wait_for_control: Callable[[], str]
    pace: Callable[[float], None]
    emit: Callable[[MissionProgress], None]


@dataclasses.dataclass
class _AscentOutcome:
    state: object
    log: SimulationLog
    reason: str
    separation_time: float | None
    actuator: ActuatorState
    guidance_state: object
    step_count: int


def _run_ascent(rt: _MissionRuntime) -> _AscentOutcome:
    """Phase A: fly the stacked vehicle until stage separation (or failure)."""
    config, dt, max_time, verbose = rt.config, rt.dt, rt.max_time, rt.verbose
    _apply_wind, _wait_for_control, _pace, _emit = rt.apply_wind, rt.wait_for_control, rt.pace, rt.emit

    if verbose:
        print("\n" + "=" * 90)
        print("FULL MISSION SIMULATION  (Ascent + Orbiter + Booster)")
        print("=" * 90)
        print("--- PHASE A: Stacked Ascent ---")

    state = _create_configured_initial_state(config)
    max_mass = _configured_max_mass(config)
    gs_ascent = create_guidance_state()

    ascent_log = SimulationLog()
    actuator = ActuatorState(thrust_dir=compute_local_vertical(state.r))
    mission_mgr = MissionManager(vehicle_type="ascent", initial_mass=state.m, config=config)
    ascent_abort_monitor = _make_abort_monitor(config)
    ascent_energy_tracker = _EnergyValidationTracker()

    current_stage = 1
    current_dry_mass = config.meco_mass_kg
    step_count = 0
    last_print_time = 0
    separation_time = None
    ascent_reason = "Stage separation"

    while True:
        if _wait_for_control() == "stop":
            ascent_reason = "User stopped"
            break
        try:
            validate_state(
                state,
                dry_mass=current_dry_mass,
                max_mass=max_mass,
                quaternion_norm_tolerance=config.quaternion_norm_tolerance,
            )
        except ValidationError as e:
            logger.error(f"Ascent validation failed: {e}")
            ascent_reason = f"Validation failure during ascent: {e}"
            break

        phase_before = mission_mgr.get_phase()
        step_dt = _remaining_step_dt(state, dt, max_time)

        step_dt = min(step_dt, C.STACKED_ASCENT_MAX_DT)
        if step_dt <= 0.0:
            ascent_reason = "Maximum simulation time reached (no separation)"
            break

        mission_mgr.update(state, step_dt)
        phase_after = mission_mgr.get_phase()

        s2_phases = {MissionPhase.S2_COAST_TO_APOGEE, MissionPhase.ORBIT_INSERTION,
                     MissionPhase.ORBIT_ACHIEVED, MissionPhase.ORBIT_FAILED}

        if (
            separation_time is None
            and phase_after == MissionPhase.STAGE_SEPARATION
            and phase_before == MissionPhase.COAST
        ):
            separation_time = _separation_time(mission_mgr, state)
            if verbose:
                print(f"\n  *** STAGE SEPARATION at t={separation_time:.2f}s "
                      f"| Alt={state.altitude/1000:.1f}km "
                      f"| V={state.speed:.0f}m/s ***\n")

        if phase_after in s2_phases:
            if separation_time is None:
                separation_time = _separation_time(mission_mgr, state)
            break

        if state.t >= max_time:
            ascent_reason = "Maximum simulation time reached (no separation)"
            break
        if state.altitude < C.CRASH_ALTITUDE_TOLERANCE:
            ascent_reason = "CRASH during ascent"
            break

        step_config = _apply_wind(config)
        state, guid_out, ctrl_out, actuator, gs_ascent = simulation_step(
            state, actuator, mission_mgr, step_dt,
            dry_mass=current_dry_mass, stage=current_stage,
            vehicle_model="stacked", gs=gs_ascent,
            config=step_config,
        )
        ascent_log.append(state, guid_out, ctrl_out)
        _emit(MissionProgress(
            vehicle="ascent",
            primary_state=state,
            primary_guidance=guid_out,
            phase=mission_mgr.get_phase().name,
        ))
        _pace(step_dt)
        safety_reason = _check_runtime_safety_limits(
            state,
            guid_out,
            ctrl_out,
            step_config,
            ascent_abort_monitor,
            current_phase=mission_mgr.get_phase(),
        )
        if safety_reason is not None:
            ascent_reason = safety_reason
            logger.error(ascent_reason)
            break

        step_count += 1
        _update_energy_validation_tracker(
            ascent_energy_tracker,
            state,
            guid_out,
            context="ascent",
        )
        if verbose and state.t - last_print_time >= 10.0:
            _print_status(state, guid_out.get('phase', '?') if isinstance(guid_out, dict) else '?')
            last_print_time = state.t

    return _AscentOutcome(
        state=state, log=ascent_log, reason=ascent_reason,
        separation_time=separation_time, actuator=actuator,
        guidance_state=gs_ascent, step_count=step_count,
    )


@dataclasses.dataclass
class _VehicleOutcome:
    state: object
    log: SimulationLog
    reason: str
    mission_manager: MissionManager


def _run_dual_vehicles(rt: _MissionRuntime, ascent: _AscentOutcome):
    """Phase B: fork into orbiter and booster and fly both to termination.

    Returns (orbiter outcome, booster outcome, total step count).
    """
    config, dt, max_time, verbose = rt.config, rt.dt, rt.max_time, rt.verbose
    _apply_wind, _wait_for_control, _pace, _emit = rt.apply_wind, rt.wait_for_control, rt.pace, rt.emit

    state = ascent.state
    actuator = ascent.actuator
    gs_ascent = ascent.guidance_state
    step_count = ascent.step_count

    s2_wet_mass = float(config.stage2_dry_mass) + float(config.stage2_prop_mass) + float(config.payload_mass)
    s2_dry_mass = float(config.stage2_dry_mass) + float(config.payload_mass)
    s1_dry_mass = float(config.stage1_dry_mass)
    booster_mass = s1_dry_mass + float(config.stage1_landing_fuel_reserve_kg)

    orbiter_dv, booster_dv, orbiter_domega, booster_domega = _compute_separation_adjustments(
        state,
        s2_wet_mass,
        booster_mass,
        config=config,
    )

    orbiter_state = _inherit_separation_state(
        state,
        s2_wet_mass,
        delta_v=orbiter_dv,
        delta_omega=orbiter_domega,
        dry_mass_kg=s2_dry_mass,
    )
    gs_orbiter = create_guidance_state()

    gs_orbiter.last_ascent_direction = gs_ascent.last_ascent_direction
    orbiter_actuator = ActuatorState(thrust_dir=actuator.thrust_dir.copy())
    orbiter_mgr = MissionManager(vehicle_type="orbiter", initial_mass=s2_wet_mass, config=config)

    orbiter_mgr.update(orbiter_state, min(dt, max(0.0, max_time - orbiter_state.t)))

    booster_state = _inherit_separation_state(
        state,
        booster_mass,
        delta_v=booster_dv,
        delta_omega=booster_domega,
        dry_mass_kg=s1_dry_mass,
    )
    gs_booster = create_guidance_state()
    booster_actuator = ActuatorState(thrust_dir=actuator.thrust_dir.copy())
    booster_mgr = MissionManager(vehicle_type="booster", initial_mass=booster_mass, config=config)

    booster_mgr.set_phase_entry_time(booster_state.t)

    booster_mgr.update(booster_state, min(dt, max(0.0, max_time - booster_state.t)))

    orbiter_log = SimulationLog()
    booster_log = SimulationLog()
    orbiter_abort_monitor = _make_abort_monitor(config)
    booster_abort_monitor = _make_abort_monitor(config)

    orbiter_done = False
    booster_done = False
    orbiter_reason = "Running"
    booster_reason = "Running"
    orb_guid = {'thrust_on': False, 'phase': orbiter_mgr.get_phase().name}
    bst_guid = {'thrust_on': False, 'phase': booster_mgr.get_phase().name}
    orbiter_energy_tracker = _EnergyValidationTracker()
    booster_energy_tracker = _EnergyValidationTracker()

    if verbose:
        print("--- PHASE B: Dual Vehicle Tracking (Orbiter + Booster) ---")
        print(f"{'Time (s)':^10} | {'Orb Alt km':^10} | {'Orb V m/s':^10} | "
              f"{'Bst Alt km':^10} | {'Bst V m/s':^10} | {'Phase O':^18} | {'Phase B':^18}")
        print("-" * 110)

    dual_step = 0

    while not (orbiter_done and booster_done):
        if _wait_for_control() == "stop":
            orbiter_done = True
            booster_done = True
            orbiter_reason = "User stopped"
            booster_reason = "User stopped"
            break

        if not orbiter_done:
            orbiter_request_dt = dt
            if (
                config.enable_demo_mode
                and booster_done
                and orbiter_mgr.get_phase() == MissionPhase.S2_COAST_TO_APOGEE
            ):
                orbiter_request_dt = float(config.demo_coast_max_dt)
            elif (
                booster_done
                and orbiter_mgr.get_phase() in _ORBIT_COAST_PHASES
                and not orb_guid.get('thrust_on', False)
                and orbiter_state.altitude > C.AERO_DISABLE_ALTITUDE
            ):
                orbiter_request_dt = max(dt, float(config.orbit_coast_max_dt))
            orbiter_state, orbiter_actuator, gs_orbiter, orbiter_done, orbiter_reason, _, new_orb_guid = _step_vehicle(
                orbiter_state, orbiter_actuator, orbiter_mgr, gs_orbiter, orbiter_log,
                orbiter_abort_monitor, config, max_time,
                dry_mass=s2_dry_mass, stage=2, vehicle_model="orbiter",
                request_dt=orbiter_request_dt, wind_cfg=_apply_wind(config),
                max_time_reason=format_orbit_failure_reason(orbiter_state, "maximum time reached"),
            )
            if new_orb_guid is not None:
                orb_guid = new_orb_guid

        if not booster_done:
            booster_state, booster_actuator, gs_booster, booster_done, booster_reason, _, new_bst_guid = _step_vehicle(
                booster_state, booster_actuator, booster_mgr, gs_booster, booster_log,
                booster_abort_monitor, config, max_time,
                dry_mass=s1_dry_mass, stage=1, vehicle_model="booster",
                request_dt=dt, wind_cfg=_apply_wind(config),
                max_time_reason="Maximum simulation time reached",
            )
            if new_bst_guid is not None:
                bst_guid = new_bst_guid

        _emit(MissionProgress(
            vehicle="dual",
            primary_state=orbiter_state,
            primary_guidance=orb_guid,
            phase=orbiter_mgr.get_phase().name,
            booster_state=booster_state,
            booster_guidance=bst_guid,
        ))
        _pace(dt)

        dual_step += 1
        step_count += 1
        if not orbiter_done:
            _update_energy_validation_tracker(
                orbiter_energy_tracker,
                orbiter_state,
                orb_guid,
                context="orbiter",
            )
        else:
            orbiter_energy_tracker.reset()
        if not booster_done:
            _update_energy_validation_tracker(
                booster_energy_tracker,
                booster_state,
                bst_guid,
                context="booster",
            )
        else:
            booster_energy_tracker.reset()

        if verbose and dual_step % 2000 == 0:
            _print_dual_status(
                orbiter_state, orbiter_done, orbiter_reason, orb_guid,
                booster_state, booster_done, booster_reason, bst_guid,
            )

    return (
        _VehicleOutcome(orbiter_state, orbiter_log, orbiter_reason, orbiter_mgr),
        _VehicleOutcome(booster_state, booster_log, booster_reason, booster_mgr),
        step_count,
    )


def run_full_mission(dt: float | None = None, max_time: float | None = None,
                     verbose: bool = True,
                     config: SimulationConfig = None,
                     *,
                     progress_callback: ProgressCallback | None = None,
                     control_callback: ControlCallback | None = None,
                     realtime_factor: float | None = None) -> FullMissionResult:
    """
    Run a full integrated mission: S1 ascent → separation → dual tracking.

    After stage separation the simulation forks into two vehicles
    (orbiter and booster) running in lockstep on the same clock.
    Each vehicle has its own GuidanceState, MissionManager and telemetry log.

    Args:
        dt: Timestep (overrides config.dt if given)
        max_time: Maximum mission time (overrides config.max_time if given)
        verbose: Print status updates
        config: SimulationConfig (default created if None)
        progress_callback: Optional observer called after each mission update
        control_callback: Optional source of ``run``, ``paused``, or ``stop``
        realtime_factor: Optional pacing factor; disabled for batch runs

    Returns:
        FullMissionResult with telemetry for ascent, orbiter and booster.
    """
    if config is None:
        config = create_default_config()
    if dt is None:
        dt = config.dt
    if max_time is None:
        max_time = config.max_time
    config.validate()
    _gfold.require_available(config)
    if dt <= 0:
        raise ValueError(f"dt must be positive, got {dt}")
    if max_time <= 0:
        raise ValueError(f"max_time must be positive, got {max_time}")
    if realtime_factor is not None and realtime_factor <= 0:
        raise ValueError(f"realtime_factor must be positive when set, got {realtime_factor}")

    def _wait_for_control() -> str:
        if control_callback is None:
            return "run"
        signal = control_callback()
        while signal == "paused":
            time.sleep(0.05)
            signal = control_callback()
        return signal

    def _pace(step_size: float) -> None:
        if realtime_factor is not None:
            time.sleep(max(float(step_size), 0.0) / realtime_factor)

    def _emit(progress: MissionProgress) -> None:
        if progress_callback is not None:
            progress_callback(progress)

    rt = _MissionRuntime(
        config=config, dt=dt, max_time=max_time, verbose=verbose,
        apply_wind=_make_wind_applier(config),
        wait_for_control=_wait_for_control, pace=_pace, emit=_emit,
    )
    start_wall = time.time()

    ascent = _run_ascent(rt)
    ascent_log, ascent_reason = ascent.log, ascent.reason
    separation_time = ascent.separation_time
    ascent_final = ascent.state.copy()

    if separation_time is None:

        empty_log = SimulationLog()
        return FullMissionResult(
            ascent_log=ascent_log, ascent_final_state=ascent_final,
            ascent_reason=ascent_reason, separation_time=None,
            orbiter_log=empty_log, orbiter_final_state=ascent_final,
            orbiter_reason="No separation",
            booster_log=empty_log, booster_final_state=ascent_final,
            booster_reason="No separation",
            orbiter_success=False,
            booster_landing_success=False,
        )

    orbiter, booster, step_count = _run_dual_vehicles(rt, ascent)
    orbiter_state, orbiter_log, orbiter_reason = orbiter.state, orbiter.log, orbiter.reason
    booster_state, booster_log, booster_reason = booster.state, booster.log, booster.reason
    orbiter_mgr = orbiter.mission_manager

    elapsed = time.time() - start_wall

    if verbose:
        _print_mission_summary(
            separation_time, orbiter_state, orbiter_reason,
            booster_state, booster_reason, config, step_count, elapsed,
        )

    return FullMissionResult(
        ascent_log=ascent_log,
        ascent_final_state=ascent_final,
        ascent_reason=ascent_reason,
        separation_time=separation_time,
        orbiter_log=orbiter_log,
        orbiter_final_state=orbiter_state,
        orbiter_reason=orbiter_reason,
        booster_log=booster_log,
        booster_final_state=booster_state,
        booster_reason=booster_reason,
        orbiter_success=_orbiter_outcome_success(orbiter_state, orbiter_mgr, config),
        booster_landing_success=_booster_landing_success(
            booster_log,
            booster_reason,
        ),
        booster_touchdown_speed_mps=booster.mission_manager.touchdown_speed_mps,
        booster_site_error_m=booster.mission_manager.touchdown_site_error_m,
    )


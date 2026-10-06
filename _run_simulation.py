"""Single-vehicle simulation loop."""

import logging
import time

import numpy as np

from . import constants as C
from ._guidance_common import compute_local_vertical, create_guidance_state
from ._main_models import SimulationLog
from ._mission_state import (
    _compute_separation_adjustments,
    _configured_max_mass,
    _create_configured_initial_state,
    _inherit_separation_state,
    _remaining_step_dt,
)
from ._simulation_step import (
    _check_runtime_safety_limits,
    _EnergyValidationTracker,
    check_termination,
    simulation_step,
)
from .abort import AbortMonitor
from .actuator import ActuatorState
from .config_definition import SimulationConfig
from .config_factory import create_default_config
from .forces import compute_atmosphere_properties
from .mass import is_propellant_exhausted
from .mission_manager import MissionManager, MissionPhase
from .mission_summary import format_orbit_failure_reason
from .physics_checks import (
    ValidationError,
    compute_total_energy,
    validate_energy_conservation,
    validate_state,
)
from .state import State
from .utils import vec_norm

logger = logging.getLogger(__name__)

def run_simulation(initial_state: State | None = None, dt: float = None, max_time: float = None,
                   verbose: bool = True, vehicle_type: str = "ascent",
                   config: SimulationConfig = None) -> tuple:
    """
    Run a single-vehicle simulation path.

    Supports three vehicle types:
    - "ascent": stacked ascent path
    - "orbiter": post-separation S2 orbit insertion path
    - "booster": post-separation S1 recovery path

    Args:
        initial_state: Optional starting state. If None, starts from liftoff default.
        dt: Time step (default from config/constants). Overrides config.dt if given.
        max_time: Maximum simulation time. Overrides config.max_time if given.
        verbose: Print progress updates. Overrides config.verbose if explicitly passed.
        vehicle_type: "ascent", "orbiter", or "booster" to determine mission logic
        config: SimulationConfig instance. If None a default is created.

    Returns:
        (final_state, log, termination_reason) tuple
    """
    if config is None:
        config = create_default_config()

                                                                     
    if dt is None:
        dt = config.dt
    if max_time is None:
        max_time = config.max_time
    config.validate()
    if dt <= 0:
        raise ValueError(f"dt must be positive, got {dt}")
    if max_time <= 0:
        raise ValueError(f"max_time must be positive, got {max_time}")

                      
    if initial_state is not None:
        state = initial_state.copy()
    else:
        state = _create_configured_initial_state(config)
    max_mass = _configured_max_mass(config)

                                                                          
    gs = create_guidance_state()

    log = SimulationLog()
    actuator = ActuatorState(thrust_dir=compute_local_vertical(state.r))
    mission_mgr = MissionManager(vehicle_type=vehicle_type, initial_mass=state.m, config=config)
    abort_monitor = (
        AbortMonitor(
            q_alpha_threshold=config.abort_q_alpha_threshold,
            attitude_threshold_deg=config.abort_attitude_threshold,
        )
        if config.enable_abort_modes
        else None
    )

                                                         
    energy_tracker = _EnergyValidationTracker()

                      
    logger.info(f"Starting simulation: dt={dt}s, max_time={max_time}s, vehicle={vehicle_type}")
    logger.debug(f"Initial state: {state}")

    if verbose:
        print("\n" + "=" * 80)
        print(f"RLV MISSION SIMULATION    | dt={dt}s | T_max={max_time}s | vehicle={vehicle_type}")
        print("=" * 80)
        print(f"{'Time (s)':^10} | {'Alt (km)':^10} | {'Vel (m/s)':^10} | {'Mass (kg)':^12} | {'Phase':<20}")
        print("-" * 80)

    start_time = time.time()
    step_count = 0
    last_print_time = 0
    meco_time = None
    separation_mass_applied = False                                     
                                                                             
                                                                        
                                                                                 
                                                                                
    cleared_pad = False

                                       
                                                  
                                                 
    current_stage = 1
    current_vehicle_model = "stacked"

                                                      
    if vehicle_type == "booster":
        current_dry_mass = float(config.stage1_dry_mass) if config is not None else C.STAGE1_DRY_MASS
        current_vehicle_model = "booster"
    elif vehicle_type == "orbiter":
                                                                    
        current_dry_mass = (float(config.stage2_dry_mass) + float(config.payload_mass)) if config is not None else C.STAGE2_DRY_MASS
        current_stage = 2
        current_vehicle_model = "orbiter"
    else:
                                                         
                                                                                           
        if config is not None:
            stack_dry_mass = (
                float(config.stage1_dry_mass)
                + float(config.stage2_dry_mass)
                + float(config.stage2_prop_mass)
                + float(config.payload_mass)
            )
        else:
            stack_dry_mass = C.DRY_MASS
        current_dry_mass = stack_dry_mass + config.stage1_landing_fuel_reserve_kg

                          
    while True:
                                                                  
        if meco_time is None and is_propellant_exhausted(state.m, current_dry_mass):
            meco_time = state.t

                           
        if not cleared_pad and state.altitude > C.LAUNCH_PAD_CLEAR_ALTITUDE:
            cleared_pad = True
        should_terminate, reason = check_termination(
            state, max_time, mission_mgr, cleared_pad=cleared_pad
        )
        if should_terminate:
            logger.info(f"Simulation terminated: {reason}")
            if verbose:
                print(f"\nTermination: {reason}")
            return state, log, reason

                        
        try:
            validate_state(
                state,
                dry_mass=current_dry_mass,
                max_mass=max_mass,
                quaternion_norm_tolerance=config.quaternion_norm_tolerance,
            )
        except ValidationError as e:
            logger.error(f"Validation failed: {e}")
            if verbose:
                print(f"\nValidation Error: {e}")
            reason = f"Validation failure: {e}"
            break

                                
        phase_before = mission_mgr.get_phase()
        step_dt = _remaining_step_dt(state, dt, max_time)
        if step_dt <= 0.0:
            if (
                current_vehicle_model == "orbiter"
                or phase_before in (MissionPhase.S2_COAST_TO_APOGEE, MissionPhase.ORBIT_INSERTION)
            ):
                reason = format_orbit_failure_reason(state, "maximum time reached")
            else:
                reason = "Maximum simulation time reached"
            logger.info(f"Simulation terminated: {reason}")
            if verbose:
                print(f"\nTermination: {reason}")
            return state, log, reason

        mission_mgr.update(state, step_dt)
        phase_after = mission_mgr.get_phase()

                                                 
                                                                                   
                                                                                       
                                               
                                                                                          
                                                                               
        s2_phases = {MissionPhase.S2_COAST_TO_APOGEE, MissionPhase.ORBIT_INSERTION,
                     MissionPhase.ORBIT_ACHIEVED, MissionPhase.ORBIT_FAILED}
        pre_sep_phases = {MissionPhase.ASCENT, MissionPhase.COAST, MissionPhase.STAGE_SEPARATION}
        if (not separation_mass_applied and
                phase_after in s2_phases and
                phase_before in pre_sep_phases):

            mass_before = state.m
            booster_mass = float(config.stage1_dry_mass) + config.stage1_landing_fuel_reserve_kg
                                                                                  
                                                                               
            s2_mass = float(config.stage2_dry_mass) + float(config.stage2_prop_mass) + float(config.payload_mass)
            orbiter_dv, _, orbiter_domega, _ = _compute_separation_adjustments(
                state,
                s2_mass,
                booster_mass,
                config=config,
            )
            state = _inherit_separation_state(
                state,
                s2_mass,
                delta_v=orbiter_dv,
                delta_omega=orbiter_domega,
                dry_mass_kg=float(config.stage2_dry_mass) + float(config.payload_mass),
            )

                                            
            current_stage = 2
            current_dry_mass = float(config.stage2_dry_mass) + float(config.payload_mass)
            current_vehicle_model = "orbiter"
            separation_mass_applied = True
            energy_tracker.reset()

            logger.info(f"SEPARATION MASS EVENT at t={state.t:.2f}s: "
                       f"mass {mass_before:.0f} -> {state.m:.0f} kg "
                       f"(dropped S1 dry={C.STAGE1_DRY_MASS:.0f}kg + "
                       f"landing fuel={config.stage1_landing_fuel_reserve_kg:.0f}kg)")
            if config.enable_separation_dynamics:
                logger.info(
                    f"Applied separation dynamics: "
                    f"|dv|={vec_norm(orbiter_dv):.3f} m/s, "
                    f"|domega|={vec_norm(orbiter_domega):.4f} rad/s"
                )
            if verbose:
                print(f"  *** STAGE SEPARATION: {mass_before:.0f} -> {state.m:.0f} kg | "
                      f"Switched to S2 engine ({C.STAGE2_THRUST/1e3:.0f} kN, Isp={C.STAGE2_ISP_VAC}s)")

                          
        state, guidance, control, actuator, gs = simulation_step(
            state, actuator, mission_mgr, step_dt,
            dry_mass=current_dry_mass, stage=current_stage,
            vehicle_model=current_vehicle_model, gs=gs,
            config=config
        )

                                                                     
        if mission_mgr.get_phase() == MissionPhase.ASCENT:
            gamma_err = abs(guidance.get('gamma_command_deg', 0.0) - guidance.get('gamma_measured_deg', 0.0))
            if state.t > 60.0 and gamma_err > 70.0:
                reason = f"Guidance divergence: |gamma_error|={gamma_err:.1f} deg"
                logger.error(reason)
                break

                  
        log.append(state, guidance, control)
        safety_reason = _check_runtime_safety_limits(
            state,
            guidance,
            control,
            config,
            abort_monitor,
        )
        if safety_reason is not None:
            reason = safety_reason
            logger.error(reason)
            break

                                                    
        step_count += 1
        _update_energy_validation_tracker(energy_tracker, state, guidance)

        if step_count % 100 == 0:
            if verbose and state.t - last_print_time >= 10.0:
                _print_status(state, guidance['phase'])
                last_print_time = state.t

    elapsed = time.time() - start_time

                       
    _log_completion(state, step_count, elapsed, verbose)

    return state, log, reason


def _validate_energy(
    E_current: float,
    E_prev: float,
    time_delta: float,
    t: float,
    context: str | None = None,
):
    """Helper to validate and log energy conservation."""
    energy_result = validate_energy_conservation(E_current, E_prev, time_delta)
    if not energy_result['valid']:
        prefix = f"{context} " if context else ""
        logger.warning(f"{prefix}Energy drift at t={t:.1f}s: "
                       f"relative_error={energy_result['relative_error']:.2e}, "
                       f"dE={energy_result['dE']:.2e} J")


def _update_energy_validation_tracker(
    tracker: _EnergyValidationTracker,
    state: State,
    guidance: dict,
    context: str | None = None,
) -> None:
    """Validate energy only across one uninterrupted ballistic interval."""
    if not _should_validate_energy(state, guidance):
        tracker.reset()
        return

    E_current = compute_total_energy(state.r, state.v, state.m)
    if tracker.reference_energy is None:
        tracker.reference_energy = E_current
        tracker.reference_time = state.t
        tracker.eligible_steps = 0
        return

    tracker.eligible_steps += 1
    if tracker.eligible_steps < tracker.interval_steps:
        return

    reference_time = state.t if tracker.reference_time is None else tracker.reference_time
    _validate_energy(
        E_current,
        tracker.reference_energy,
        max(float(state.t - reference_time), 0.0),
        state.t,
        context=context,
    )
    tracker.reference_energy = E_current
    tracker.reference_time = state.t
    tracker.eligible_steps = 0


def _should_validate_energy(state: State, guidance: dict) -> bool:
    """Only apply the two-body invariant within its stated validity domain."""
    if guidance.get('thrust_on', False):
        return False

    config = state.sim_config
    if config is not None:
        gravity_model = str(getattr(config, "gravity_model", "central")).lower()
        enable_j2 = bool(getattr(config, "enable_j2", False))
        if gravity_model != "central" or enable_j2:
            return False

    altitude = float(state.altitude)
    v_rel = np.asarray(guidance.get('v_rel', state.v), dtype=float)
    _, _, rho, _ = compute_atmosphere_properties(altitude)
    q_dyn = 0.5 * rho * float(np.dot(v_rel, v_rel))
    return q_dyn <= 1.0


def _print_status(state: State, phase: str):
    """Print a formatted status row."""
                                             
    msg = (f"{state.t:10.1f} | {state.altitude/1000:10.1f} | "
           f"{state.speed:10.1f} | {state.m:12.1f} | {phase:<15}")
    print(msg)
    logger.info(msg)


def _log_completion(state: State, steps: int, elapsed: float, verbose: bool):
    """Log and print comparison statistics."""
    logger.info(f"Simulation complete: {steps} steps in {elapsed:.2f}s")
    logger.info(f"Final state: alt={state.altitude/1000:.2f}km, v={state.speed:.1f}m/s")
    
    if verbose:
        print("-" * 80)
        print("SIMULATION COMPLETED")
        print("-" * 80)
        print(f"Final Time:     {state.t:.2f} s")
        print(f"Final Altitude: {state.altitude/1000:.2f} km")
        print(f"Final Velocity: {state.speed:.2f} m/s")
        print(f"Final Mass:     {state.m:.1f} kg")
        print("-" * 80)
        print(f"Steps:       {steps:,}")
        print(f"Wall Time:   {elapsed:.2f} s")
        print(f"Performance: {steps/elapsed:.0f} steps/s" if elapsed > 0 else "Performance: N/A")
        print("=" * 80)

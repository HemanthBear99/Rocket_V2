"""Per-timestep simulation pipeline extracted from main."""

import logging
from dataclasses import dataclass

import numpy as np

from . import constants as C
from ._guidance_ascent import compute_guidance_output
from ._guidance_booster import compute_booster_guidance
from ._guidance_common import (
    GuidanceState,
    compute_local_vertical,
    create_guidance_state,
)
from ._guidance_orbit import (
    compute_coast_guidance,
    compute_deorbit_guidance,
    compute_orbit_insertion_guidance,
    compute_s2_entry_landing_guidance,
)
from ._main_models import SimulationLog
from .abort import AbortMonitor
from .actuator import ActuatorState, update_actuator
from .config_definition import SimulationConfig
from .config_factory import create_default_config
from .control import (
    compute_control_output,
    prepare_control_state,
    resolve_attitude_controller,
)
from .dynamics import DynamicsContext
from .forces import (
    _altitude_from_r,
    _tvc_lever_arm,
    apply_engine_transient,
    compute_atmosphere_properties,
    compute_specific_forces,
)
from .frames import rotate_vector_by_quaternion
from .integrators import integrate
from .mass import compute_inertia_tensor
from .mission_manager import MissionManager, MissionPhase
from .mission_summary import (
    classify_landing,
    format_orbit_failure_reason,
    hard_landing_speed_limit,
)
from .navigation import update_navigation_estimate
from .rcs import RCSState, rcs_available_torque, update_rcs_propellant
from .recovery import (
    great_circle_distance_m,
    predicted_landing_site_eci,
    rotating_launch_site_eci,
    target_landing_site_eci,
)
from .recovery_hardware import (
    assess_touchdown_contact,
    command_grid_fins_to_target,
    compute_grid_fin_force,
    update_landing_leg_state,
)
from .state import State
from .utils import compute_relative_velocity, surface_relative_speed

logger = logging.getLogger(__name__)


def _is_s2_touchdown_success(state: State, config: SimulationConfig) -> bool:
    return surface_relative_speed(state, config) < float(config.s2_touchdown_speed_limit_mps)


def _orbiter_outcome_success(state: State, mission_mgr: MissionManager, config: SimulationConfig) -> bool:
    phase = mission_mgr.get_phase()
    if bool(getattr(config, "enable_s2_recovery", False)):
        return phase == MissionPhase.S2_LANDING and state.altitude <= 0.1 and _is_s2_touchdown_success(state, config)
    return phase == MissionPhase.ORBIT_ACHIEVED


def _booster_landing_success(log: SimulationLog, reason: str = "") -> bool:
    """Return the terminal booster landing outcome.

    Touchdown is detected before another telemetry row is appended, so the
    final contact status never lands in the log at the moment of touchdown.
    The explicit termination reason is the sole source of truth.
    """
    del log
    return str(reason).startswith("LANDING SUCCESS")


def _run_guidance(
    state: State,
    gs: GuidanceState,
    mission_mgr: MissionManager,
    config: SimulationConfig,
    dt: float,
) -> tuple[dict, GuidanceState]:
    """Execute guidance phase dispatch and navigation update."""
    phase = mission_mgr.get_phase()
    gs.navigation_state = update_navigation_estimate(
        gs.navigation_state,
        state,
        config,
        dt,
    )
    nav_estimate = gs.navigation_state.estimate
    use_nav_for_guidance = (
        bool(config.use_navigation_estimate_for_guidance)
        and nav_estimate is not None
    )
    guidance_r = nav_estimate.position_eci_m if use_nav_for_guidance else state.r
    guidance_v = nav_estimate.velocity_eci_mps if use_nav_for_guidance else state.v
    guidance_t = nav_estimate.time_s if use_nav_for_guidance else state.t

    stage1_landing_reserve_kg = config.stage1_landing_fuel_reserve_kg

    if phase == MissionPhase.ASCENT:
        meco_mass = C.DRY_MASS + stage1_landing_reserve_kg
        guidance, gs = compute_guidance_output(
            guidance_r, guidance_v, guidance_t, state.m,
            meco_mass=meco_mass, gs=gs, dt=dt, config=config,
        )
    elif phase in [MissionPhase.COAST, MissionPhase.STAGE_SEPARATION,
                   MissionPhase.S2_COAST_TO_APOGEE]:
        guidance, gs = compute_coast_guidance(
            guidance_r, guidance_v, guidance_t, state.m, gs=gs, config=config,
        )
    elif phase == MissionPhase.ORBIT_INSERTION:
        guidance, gs = compute_orbit_insertion_guidance(
            guidance_r, guidance_v, guidance_t, state.m, gs=gs, config=config,
        )
    elif phase == MissionPhase.S2_DEORBIT:
        guidance, gs = compute_deorbit_guidance(
            guidance_r, guidance_v, guidance_t, state.m, gs=gs, config=config,
        )
    elif phase in [MissionPhase.S2_ENTRY, MissionPhase.S2_LANDING]:
        guidance, gs = compute_s2_entry_landing_guidance(
            guidance_r, guidance_v, guidance_t, state.m, phase.name, gs=gs, config=config,
        )
    elif phase in [MissionPhase.BOOSTER_FLIP, MissionPhase.BOOSTER_BOOSTBACK,
                   MissionPhase.BOOSTER_COAST, MissionPhase.BOOSTER_ENTRY,
                   MissionPhase.BOOSTER_LANDING]:
        guidance, gs = compute_booster_guidance(
            guidance_r, guidance_v, guidance_t, state.m, phase.name, gs=gs, config=config,
        )
    else:
        guidance, gs = compute_coast_guidance(
            guidance_r, guidance_v, guidance_t, state.m, gs=gs, config=config,
        )
        guidance['phase'] = phase.name

    if nav_estimate is not None:
        guidance['guidance_uses_navigation_estimate'] = use_nav_for_guidance
        guidance['gps_available'] = nav_estimate.gps_available
        guidance['imu_available'] = nav_estimate.imu_available
        guidance['altimeter_available'] = nav_estimate.altimeter_available
        guidance['nav_position_error_m'] = nav_estimate.position_error_m
        guidance['nav_velocity_error_mps'] = nav_estimate.velocity_error_mps

    guidance['v_rel'] = compute_relative_velocity(
        state.r, state.v,
        wind_offset_mps=config.runtime_wind_offset_mps,
    )
    guidance['phase'] = phase.name

    return guidance, gs


def _run_booster_hardware(
    state: State,
    guidance: dict,
    gs: GuidanceState,
    mission_mgr: MissionManager,
    config: SimulationConfig,
    vehicle_model: str,
    dt: float,
) -> tuple[dict, GuidanceState, object | None]:
    """Compute grid fin commands and update landing leg state for booster recovery."""
    grid_fin_command = None
    if vehicle_model == "booster":
        landing_target_lead_time = max(
            float(guidance.get('landing_target_lead_time_s', 0.0)),
            0.0,
        )
        if landing_target_lead_time > 0.0:
            target_site = predicted_landing_site_eci(
                state.t, landing_target_lead_time,
                config.booster_landing_target_downrange_km, config=config,
            )
        else:
            target_site = target_landing_site_eci(
                state.t, config.booster_landing_target_downrange_km, config=config,
            )
        grid_fin_command = command_grid_fins_to_target(
            state.r,
            state.v,
            target_site,
            config,
            mass=state.m,
            time_to_go_s=float(
                guidance.get('landing_guidance_time_to_go_s', 0.0)
            ),
            zero_effort_miss=guidance.get('grid_fin_zero_effort_miss'),
        )
        grid_fin_force = compute_grid_fin_force(
            state.r, state.v, state.q, grid_fin_command, config,
            wind_offset_mps=config.runtime_wind_offset_mps,
        )
        gs.landing_leg_state = update_landing_leg_state(
            gs.landing_leg_state, state, config, dt,
        )
        mission_mgr.booster_landing_leg_state = gs.landing_leg_state
        guidance['grid_fin_deployed_fraction'] = grid_fin_command.deployed_fraction
        guidance['grid_fin_pitch_cmd_deg'] = grid_fin_command.pitch_cmd_deg
        guidance['grid_fin_yaw_cmd_deg'] = grid_fin_command.yaw_cmd_deg
        guidance['grid_fin_force_n'] = float(np.linalg.norm(grid_fin_force))
        guidance['grid_fin_saturated'] = grid_fin_command.saturated
        guidance['landing_leg_status'] = gs.landing_leg_state.status.value
        guidance['landing_leg_deployed_fraction'] = gs.landing_leg_state.deployed_fraction
        guidance_predicted = float(guidance.get('predicted_landing_error_m', 0.0))
        if guidance_predicted <= 0.0:
            guidance['predicted_landing_error_m'] = great_circle_distance_m(state.r, target_site)
        guidance['touchdown_contact_status'] = ''

    return guidance, gs, grid_fin_command


def _available_attitude_torque_limit(
    thrust_newton: float,
    throttle: float,
    thrust_active: bool,
    stage: int,
    vehicle_model: str,
    config: SimulationConfig,
    structural_limit: float,
    rcs_state: RCSState | None = None,
) -> float:
    """Bound commanded torque by actual TVC/RCS authority."""
    thrust_torque = 0.0
    if thrust_active and throttle > 0.0 and thrust_newton > 0.0:
        lateral_force = thrust_newton * float(throttle) * np.sin(C.MAX_GIMBAL_ANGLE)
        thrust_torque = lateral_force * _tvc_lever_arm(stage, vehicle_model, config=config)

    rcs_torque = 0.0
    if config is not None and config.enable_rcs and rcs_state is not None:
        rcs_torque = rcs_available_torque(rcs_state, config)

    return float(max(0.0, min(structural_limit, thrust_torque + rcs_torque)))


def _allocate_attitude_torque(
    torque: np.ndarray,
    thrust_newton: float,
    throttle: float,
    thrust_active: bool,
    stage: int,
    vehicle_model: str,
    config: SimulationConfig,
    structural_limit: float,
    rcs_state: RCSState | None = None,
) -> np.ndarray:
    """Apply simple TVC/RCS control allocation in body axes.

    Body z is the vehicle longitudinal axis, so TVC supplies pitch/yaw torque
    in body x/y. Roll torque about z is limited to RCS authority.
    """
    torque = np.asarray(torque, dtype=float).copy()

    tvc_limit = 0.0
    if thrust_active and throttle > 0.0 and thrust_newton > 0.0:
        tvc_limit = (
            thrust_newton
            * float(throttle)
            * np.sin(C.MAX_GIMBAL_ANGLE)
            * _tvc_lever_arm(stage, vehicle_model, config=config)
        )
    tvc_limit = min(float(tvc_limit), float(structural_limit))

    rcs_limit = 0.0
    if config is not None and config.enable_rcs and rcs_state is not None:
        rcs_limit = rcs_available_torque(rcs_state, config)

    xy_limit = min(float(structural_limit), tvc_limit + rcs_limit)
    xy = torque[:2]
    xy_norm = float(np.linalg.norm(xy))
    if xy_norm > xy_limit and xy_norm > 1e-9:
        torque[:2] = xy * (xy_limit / xy_norm)
    torque[2] = float(np.clip(torque[2], -rcs_limit, rcs_limit))

    total_norm = float(np.linalg.norm(torque))
    if total_norm > structural_limit and total_norm > 1e-9:
        torque *= float(structural_limit) / total_norm
    return torque


def _pad_radius_m(state: State, config: SimulationConfig | None = None) -> float:
    """Spherical-Earth radius of the configured launch pad surface (m)."""
    launch_alt = 0.0
    cfg = config if config is not None else state.sim_config
    if cfg is not None:
        launch_alt = float(getattr(cfg, "launch_site_altitude_m", 0.0) or 0.0)
    return C.R_EARTH + launch_alt


def apply_launch_pad_constraint(
    state: State,
    config: SimulationConfig | None = None,
) -> State:
    """Keep the vehicle on the pad while engine spool-up has TWR < 1.

    Without a pad normal force, free-body integration sinks the stack below
    the spherical surface for ~1 s of spool-up (negative altitudes at t≈0).
    Project onto the pad sphere and remove inward radial velocity while
    contact is active. Once altitude is positive, this is a no-op.
    """
    pad_r = _pad_radius_m(state, config)
    r = np.asarray(state.r, dtype=float).copy()
    v = np.asarray(state.v, dtype=float).copy()
    r_norm = float(np.linalg.norm(r))
    if r_norm < C.ZERO_TOLERANCE:
        return state

    r_hat = r / r_norm
    altitude = r_norm - pad_r
                            
    if altitude > 0.0:
        return state

    if altitude < 0.0:
        r = r_hat * pad_r
        r_hat = r / pad_r

    v_rad = float(np.dot(v, r_hat))
    if v_rad < 0.0:
                                                                                        
        v = v - v_rad * r_hat

    state.r = r
    state.v = v
    return state


def _interpolate_ground_crossing(old_state: State, new_state: State) -> State:
    """Return a linearly interpolated state at the Earth-surface crossing."""
    old_alt = float(old_state.altitude)
    new_alt = float(new_state.altitude)
    if old_alt <= 0.0 or new_alt >= 0.0:
        return new_state

    frac = old_alt / max(old_alt - new_alt, 1e-12)
    frac = float(np.clip(frac, 0.0, 1.0))
    from .frames import quaternion_normalize
                                                                                
                                                                 
    q_crossed = quaternion_normalize(old_state.q + frac * (new_state.q - old_state.q))
    crossed = State(
        r=old_state.r + frac * (new_state.r - old_state.r),
        v=old_state.v + frac * (new_state.v - old_state.v),
        q=q_crossed,
        omega=old_state.omega + frac * (new_state.omega - old_state.omega),
        m=old_state.m + frac * (new_state.m - old_state.m),
        t=old_state.t + frac * (new_state.t - old_state.t),
        sim_config=new_state.sim_config,
        dry_mass_kg=new_state.dry_mass_kg,
    )
    pad_r = _pad_radius_m(crossed)
    r_norm = float(np.linalg.norm(crossed.r))
    if r_norm > C.ZERO_TOLERANCE:
        crossed.r = crossed.r * (pad_r / r_norm)
    return crossed


@dataclass
class _EnergyValidationTracker:
    """Track one continuous unpowered, low-drag interval for energy checks."""

    interval_steps: int = 100
    reference_energy: float | None = None
    reference_time: float | None = None
    eligible_steps: int = 0

    def reset(self) -> None:
        self.reference_energy = None
        self.reference_time = None
        self.eligible_steps = 0


def check_termination(state: State, max_time: float, mission_mgr: MissionManager = None,
                      cleared_pad: bool = True) -> tuple:
    """
    Check if simulation should terminate.

    Termination criteria depend on vehicle type and mission phase:
    - ascent: Stops at apogee (legacy single-stage sim)
    - orbiter: Stops when orbit is achieved or S2 propellant exhausted
    - booster: Stops at touchdown

    Args:
        state: Current state
        max_time: Maximum allowed simulation time (s)
        mission_mgr: Mission Manager instance
        cleared_pad: Whether the vehicle has cleared the launch pad/tower. While
            False, the strict ground-impact check is held off during early ascent
            (pad contact is enforced separately). Defaults to True so airborne
            callers (orbiter, booster) are unaffected; the -1000 m hard floor
            always applies.

    Returns:
        (should_terminate, reason) tuple
    """
    phase = mission_mgr.get_phase()
    cfg = getattr(mission_mgr, 'config', None)
    s2_recovery = bool(cfg.enable_s2_recovery) if cfg is not None else False

                                                  
    if phase == MissionPhase.ORBIT_ACHIEVED:
                                                                              
                                                                             
        if not s2_recovery:
            return True, "ORBIT ACHIEVED - Mission Complete"
                                                                
    if phase == MissionPhase.S2_LANDING and state.altitude <= 0.1:
        if cfg is None:
            raise RuntimeError("S2 landing termination requires SimulationConfig")
        v_touchdown = float(np.linalg.norm(
            compute_relative_velocity(
                state.r, state.v,
                wind_offset_mps=cfg.runtime_wind_offset_mps if cfg is not None else 0.0,
            )
        ))
        downrange_km = great_circle_distance_m(
            state.r, rotating_launch_site_eci(state.t, config=cfg)
        ) / 1000.0
        if _is_s2_touchdown_success(state, cfg):
            return True, (
                f"S2 DRONE-SHIP LANDING SUCCESS - Touchdown at {v_touchdown:.2f} m/s "
                f"({downrange_km:.0f} km downrange)"
            )
        return True, (
            f"S2 DRONE-SHIP CRASH - Impact at {v_touchdown:.2f} m/s "
            f"({downrange_km:.0f} km downrange)"
        )
    if phase == MissionPhase.ORBIT_FAILED:
        reason = getattr(mission_mgr, 'orbit_failure_reason', None) or "Target orbit not achieved"
        return True, f"ORBIT INSERTION FAILED - {reason}"

                                 
    if mission_mgr.vehicle_type == "booster":
        touchdown_tolerance_m = 0.1
        if phase == MissionPhase.BOOSTER_LANDING and state.altitude <= touchdown_tolerance_m:
            cfg = mission_mgr.config
            v_touchdown_rel = float(np.linalg.norm(
                compute_relative_velocity(
                    state.r,
                    state.v,
                    wind_offset_mps=cfg.runtime_wind_offset_mps,
                )
            ))
            target_site = target_landing_site_eci(
                state.t,
                cfg.booster_landing_target_downrange_km,
                config=cfg,
            )
            site_error_m = great_circle_distance_m(state.r, target_site)
            propellant_remaining = max(0.0, state.m - cfg.stage1_dry_mass)
            body_z = rotate_vector_by_quaternion(C.BODY_Z_AXIS, state.q)
            terminal_attitude_error = float(
                np.degrees(
                    np.arccos(
                        np.clip(np.dot(body_z, compute_local_vertical(state.r)), -1.0, 1.0)
                    )
                )
            )
            contact = assess_touchdown_contact(
                state,
                cfg,
                site_error_m=site_error_m,
                leg_state=getattr(mission_mgr, "booster_landing_leg_state", None),
                terminal_attitude_error_deg=terminal_attitude_error,
            )
            if contact.status == "landing_success":
                landing = classify_landing(
                    v_touchdown_rel,
                    site_error_m,
                    cfg,
                    propellant_remaining_kg=propellant_remaining,
                )
                return True, landing.reason
                                                                         
                                                                         
            if v_touchdown_rel > hard_landing_speed_limit(cfg):
                site_error_km = site_error_m / 1000.0
                if site_error_m <= cfg.booster_pad_tolerance_m:
                    return True, (
                        f"CRASH - Impact at {v_touchdown_rel:.2f} m/s "
                        f"(site error {site_error_m:.1f} m)"
                    )
                return True, (
                    f"OFFSITE CRASH - Missed pad by {site_error_km:.2f} km "
                    f"at {v_touchdown_rel:.2f} m/s"
                )
                                                                       
                                                                                 
                              
            if (
                contact.status == "offsite_touchdown"
                and cfg.booster_enforce_pad_landing
            ):
                site_error_km = site_error_m / 1000.0
                return True, (
                    f"OFFSITE LANDING - Missed pad by {site_error_km:.2f} km "
                    f"at {v_touchdown_rel:.2f} m/s"
                )
            return True, contact.reason

                                                                   
                                                              
                                                           
    if mission_mgr.vehicle_type == "ascent_only":
        vertical = compute_local_vertical(state.r)
        v_vert = float(np.dot(state.v, vertical))
                                                           
        if state.t > 100.0 and v_vert <= 0.0:
            return True, "Apogee Reached (v_vert <= 0)"

                          
    if state.t >= max_time:
        if (
            mission_mgr.vehicle_type == "orbiter"
            or phase in (MissionPhase.S2_COAST_TO_APOGEE, MissionPhase.ORBIT_INSERTION)
        ):
            return True, format_orbit_failure_reason(state, "maximum time reached")
        return True, "Maximum simulation time reached"

                       
                                                                      
    if cleared_pad and state.t > 1.0 and state.altitude <= 0.0:
        return True, "CRASH - Ground impact"
    if state.altitude < C.CRASH_ALTITUDE_TOLERANCE:
        return True, "CRASH - Below Earth's surface"

    return False, None





def _check_runtime_safety_limits(
    state: State,
    guidance: dict,
    control: dict,
    config: SimulationConfig,
    abort_monitor: AbortMonitor | None = None,
    current_phase=None,
) -> str | None:
    """Return a termination reason when enabled structural/abort limits trip."""
    if config is None:
        return None

    v_rel = np.asarray(guidance.get('v_rel', state.v), dtype=float)
    v_rel_mag = float(np.linalg.norm(v_rel))
    effective_alt = _altitude_from_r(state.r, config)
    enable_upper = bool(getattr(config, 'enable_upper_atmosphere', False))
    _, _, rho_atm, _ = compute_atmosphere_properties(effective_alt, enable_upper_atm=enable_upper)

    body_z = rotate_vector_by_quaternion(C.BODY_Z_AXIS, state.q)
    if v_rel_mag > 10.0:
        cos_aoa = float(np.clip(np.dot(body_z, v_rel / v_rel_mag), -1.0, 1.0))
        aoa_rad = float(np.arccos(cos_aoa))
    else:
        aoa_rad = 0.0
    q_alpha = 0.5 * rho_atm * v_rel_mag ** 2 * aoa_rad

    if config.abort_on_q_alpha_limit and q_alpha > config.q_alpha_max:
        return (
            f"Q-alpha limit exceeded: {q_alpha:.3g} > "
            f"{config.q_alpha_max:.3g} Pa*rad"
        )

    if config.enable_abort_modes and abort_monitor is not None:
                                                                      
                                                                            
                                                                                  
                                                                                
                                                              
                                                                           
                                                                            
                                                                   
        _booster_recovery_phases = {
            MissionPhase.BOOSTER_FLIP,
            MissionPhase.BOOSTER_BOOSTBACK,
            MissionPhase.BOOSTER_COAST,
            MissionPhase.BOOSTER_ENTRY,
            MissionPhase.BOOSTER_LANDING,
        }
        if current_phase not in _booster_recovery_phases:
            abort_result = abort_monitor.check(
                state.r,
                state.v,
                state.q,
                state.omega,
                state.t,
                attitude_error_rad=float(control.get('error_angle', 0.0)),
                wind_offset_mps=config.runtime_wind_offset_mps,
                enable_upper_atmosphere=bool(getattr(config, 'enable_upper_atmosphere', False)),
            )
            if abort_result.get('abort'):
                return (
                    f"ABORT - {abort_result.get('mode')}: "
                    f"{abort_result.get('reason')}"
                )

    return None


def simulation_step(state: State, actuator: ActuatorState, mission_mgr: MissionManager,
                    dt: float, dry_mass: float = C.DRY_MASS, stage: int = 1,
                    vehicle_model: str = "stacked",
                    gs: GuidanceState | None = None,
                    config: SimulationConfig | None = None) -> tuple:
    """
    Execute one complete simulation timestep.

    Execution order (as per spec):
    1. Guidance -> thrust direction
    2. Attitude command -> quaternion
    3. Control torque
    4-8. Integration (dynamics propagate in integrator)

    Args:
        state: Current state
        actuator: Gimbal actuator state
        mission_mgr: Mission phase manager
        dt: Time step
        dry_mass: Current dry mass limit
        stage: Engine stage (1 = S1, 2 = S2)
        gs: Per-vehicle guidance state (created if None)
        config: Simulation configuration (uses default if None)

    Returns:
        (new_state, guidance_output, control_output, actuator, guidance_state) tuple
    """
    if gs is None:
        gs = create_guidance_state()
    if config is None:
        config = create_default_config()

    phase = mission_mgr.get_phase()
    stage1_landing_reserve_kg = config.stage1_landing_fuel_reserve_kg

                                                                            
    guidance, gs = _run_guidance(state, gs, mission_mgr, config, dt)

                                                                             
    guidance, gs, grid_fin_command = _run_booster_hardware(
        state, guidance, gs, mission_mgr, config, vehicle_model, dt,
    )

                                                                             
    desired_dir = guidance['thrust_direction']
    s2_recovery_phase = vehicle_model == "orbiter" and phase in (
        MissionPhase.S2_DEORBIT, MissionPhase.S2_ENTRY, MissionPhase.S2_LANDING
    )
    if vehicle_model == "booster" or s2_recovery_phase:
        n_des = np.linalg.norm(desired_dir)
        thrust_dir_cmd = desired_dir / n_des if n_des > 1e-9 else compute_local_vertical(state.r)
        actuator = ActuatorState(thrust_dir=thrust_dir_cmd, throttle=actuator.throttle)
    else:
        actuator = update_actuator(actuator, desired_dir, dt)
        thrust_dir_cmd = actuator.thrust_dir

    requested_thrust_on = bool(guidance.get('thrust_on', False))
    throttle_cmd = float(guidance.get('throttle', 1.0))
    propellant_available = state.m > (dry_mass + 1e-6)
    if not propellant_available:
        throttle_cmd = 0.0
    actual_throttle = throttle_cmd
    if config.enable_engine_transients:
        actual_throttle = apply_engine_transient(
            throttle_cmd, actuator.throttle, dt,
            spool_up_time=config.engine_spool_up_time,
            spool_down_time=config.engine_spool_down_time,
        )
    actuator = ActuatorState(thrust_dir=actuator.thrust_dir, throttle=actual_throttle)
    thrust_active = requested_thrust_on and propellant_available and actual_throttle > 0.0
    thrust_magnitude_override = None
    if mission_mgr.vehicle_type == "booster" and stage == 1 and vehicle_model == "booster":
        if phase == MissionPhase.BOOSTER_BOOSTBACK:
            thrust_magnitude_override = C.BOOSTBACK_THRUST
        elif phase == MissionPhase.BOOSTER_ENTRY:
            thrust_magnitude_override = C.ENTRY_THRUST
        elif phase == MissionPhase.BOOSTER_LANDING:
            thrust_magnitude_override = C.LANDING_THRUST
    thrust_magnitude_n = (
        float(thrust_magnitude_override)
        if thrust_magnitude_override is not None
        else float(
            config.stage2_thrust_vac if stage == 2 else C.THRUST_MAGNITUDE
        )
    ) * float(config.runtime_thrust_scale)

                                                                            
    I_tensor = compute_inertia_tensor(
        state.m, vehicle_model=vehicle_model,
        stage1_landing_reserve_kg=stage1_landing_reserve_kg,
    )
    inertia_diag = np.array([I_tensor[0, 0], I_tensor[1, 1], I_tensor[2, 2]])

    if gs.rcs_state is None and config.enable_rcs:
        gs.rcs_state = RCSState(propellant_remaining_kg=config.rcs_propellant_mass)

    s2_recovery_attitude = (
        vehicle_model == "orbiter"
        and phase in (MissionPhase.S2_DEORBIT, MissionPhase.S2_ENTRY, MissionPhase.S2_LANDING)
    )
    if s2_recovery_attitude:
        stage_max_torque = config.max_torque * (C.BOOSTER_MAX_TORQUE_NM / C.MAX_TORQUE)
    elif stage == 2:
        stage_max_torque = config.max_torque * (2.0e6 / C.MAX_TORQUE)
    elif vehicle_model == "booster":
        stage_max_torque = config.max_torque * (C.BOOSTER_MAX_TORQUE_NM / C.MAX_TORQUE)
    else:
        stage_max_torque = config.max_torque

    if s2_recovery_attitude:
        available_torque = stage_max_torque
    else:
        available_torque = _available_attitude_torque_limit(
            thrust_magnitude_n, actual_throttle, thrust_active,
            stage, vehicle_model, config, stage_max_torque,
            rcs_state=gs.rcs_state,
        )

    control_inertia = inertia_diag
    if vehicle_model == "booster":
        # Gain-scheduling input only (see constants.BOOSTER_CONTROL_INERTIA_SCALE)
        # - does not change the physical inertia_diag used in the EOM above.
        control_inertia = C.BOOSTER_CONTROL_INERTIA_SCALE * inertia_diag

    attitude_controller = resolve_attitude_controller(phase, vehicle_model, config)
    gs.control_state = prepare_control_state(gs.control_state, attitude_controller, phase.name)

    rcs_limit = 0.0
    if config is not None and config.enable_rcs and gs.rcs_state is not None:
        rcs_limit = rcs_available_torque(gs.rcs_state, config)

    # RCS-only means the attitude actuators can produce torque but the TVC
    # cannot -- i.e. no gimbal authority at all. Derive it from the real TVC
    # contribution rather than from a torque-magnitude literal inside
    # control.py, so the attitude deadband tracks the actual actuator state
    # (unpowered ascent/coast/landing) instead of an unreachable threshold.
    tvc_authority = 0.0
    if thrust_active and actual_throttle > 0.0 and thrust_magnitude_n > 0.0:
        tvc_authority = (
            float(thrust_magnitude_n) * float(actual_throttle)
            * float(np.sin(C.MAX_GIMBAL_ANGLE))
            * _tvc_lever_arm(stage, vehicle_model, config=config)
        )
    rcs_only = bool(tvc_authority <= 0.0 and available_torque > 0.0)

    control = compute_control_output(
        state.q, state.omega, thrust_dir_cmd,
        inertia=control_inertia, max_torque=available_torque,
        kp_attitude=config.kp_attitude, kd_attitude=config.kd_attitude,
        ki_attitude=config.ki_attitude, control_state=gs.control_state,
        dt=dt, controller=attitude_controller,
        integral_windup_fraction=config.attitude_integral_windup_fraction,
        max_roll_torque=rcs_limit,
        rcs_only=rcs_only,
    )
    if control.get('control_state') is not None:
        gs.control_state = control['control_state']
    control.pop('control_state', None)

    if s2_recovery_attitude:
        tau = np.asarray(control['torque'], dtype=float)
        tau_norm = float(np.linalg.norm(tau))
        if tau_norm > stage_max_torque and tau_norm > 1e-9:
            tau = tau * (stage_max_torque / tau_norm)
        control['torque'] = tau
    else:
        control['torque'] = _allocate_attitude_torque(
            control['torque'], thrust_magnitude_n, actual_throttle,
            thrust_active, stage, vehicle_model, config, stage_max_torque,
            rcs_state=gs.rcs_state,
        )
    control['torque_magnitude'] = float(np.linalg.norm(control['torque']))
    control['saturated'] = control['torque_magnitude'] >= max(available_torque, 1e-9) * 0.999

                                                                           
    if config.enable_rcs and gs.rcs_state is not None:
        tvc_capacity = 0.0
        if thrust_active and actual_throttle > 0.0 and thrust_magnitude_n > 0.0:
            tvc_capacity = (
                thrust_magnitude_n * float(actual_throttle)
                * np.sin(C.MAX_GIMBAL_ANGLE) * _tvc_lever_arm(stage, vehicle_model, config=config)
            )
        xy_torque = float(np.linalg.norm(control['torque'][:2]))
        rcs_xy_torque = max(0.0, xy_torque - tvc_capacity)
        roll_torque = float(abs(control['torque'][2])) if len(control['torque']) > 2 else 0.0
        gs.rcs_state = update_rcs_propellant(
            gs.rcs_state, float(np.hypot(rcs_xy_torque, roll_torque)), dt, config,
        )

                                                                                        
    booster_aero_mode = phase.name if (
        vehicle_model == "booster"
        or (vehicle_model == "orbiter" and phase.name in ("S2_ENTRY", "S2_LANDING"))
    ) else None

                                                      
    guidance['thrust_on'] = thrust_active
    guidance['throttle_commanded'] = throttle_cmd
    guidance['throttle'] = actual_throttle
    guidance['thrust_magnitude_n'] = thrust_magnitude_n
    guidance['attitude_torque_limit_n_m'] = available_torque
    guidance['vehicle_model'] = vehicle_model
    guidance['dry_mass_kg'] = dry_mass
    guidance['stage'] = stage
    if gs.rcs_state is not None:
        guidance['rcs_propellant_remaining_kg'] = gs.rcs_state.propellant_remaining_kg
        guidance['rcs_exhausted'] = gs.rcs_state.exhausted

                                                                           
                                                                   
                                                                            
                                                
    dx = DynamicsContext(
        thrust_on=thrust_active, throttle=actual_throttle, dry_mass=dry_mass,
        stage=stage, vehicle_model=vehicle_model,
        booster_aero_mode=booster_aero_mode,
        thrust_magnitude_override=thrust_magnitude_override,
        grid_fin_command=grid_fin_command,
        config=config,
    )
    new_state = integrate(state, control['torque'], dt, ctx=dx, method='rk4')
                                                                             
                                                                               
                                                                                      
                                                                               
    pad_r = _pad_radius_m(state, config)
    r_now = float(np.linalg.norm(state.r))
    alt_above_pad = r_now - pad_r
    contact_tolerance = max(C.ZERO_TOLERANCE, np.finfo(float).eps * pad_r)
    if (
        vehicle_model == "stacked"
        and phase == MissionPhase.ASCENT
        and alt_above_pad <= contact_tolerance
    ):
        new_state = apply_launch_pad_constraint(new_state, config)
    elif state.altitude > 0.0 and new_state.altitude < 0.0:
        new_state = _interpolate_ground_crossing(state, new_state)

                                                                            
    force_breakdown = compute_specific_forces(
        new_state.r, new_state.v, new_state.q, new_state.m,
        thrust_on=thrust_active, stage=stage, throttle=actual_throttle,
        vehicle_model=vehicle_model, booster_aero_mode=booster_aero_mode,
        thrust_magnitude_override=thrust_magnitude_override,
        enable_j2=config.enable_j2, j2_coefficient=config.j2_coefficient,
        gravity_model=config.gravity_model,
        thrust_scale=config.runtime_thrust_scale,
        wind_offset_mps=config.runtime_wind_offset_mps,
        grid_fin_command=grid_fin_command, config=config,
        control_torque_xy=np.asarray(control['torque'], dtype=float)[:2],
    )
    guidance['force_gravity_n'] = float(force_breakdown['gravity_magnitude'])
    guidance['force_thrust_n'] = float(force_breakdown['thrust_magnitude'])
    guidance['force_drag_n'] = float(force_breakdown['drag_magnitude'])
    guidance['force_lift_n'] = float(force_breakdown['lift_magnitude'])
    guidance['grid_fin_force_n'] = float(force_breakdown.get('grid_fin_magnitude', guidance.get('grid_fin_force_n', 0.0)))
    guidance['force_total_n'] = float(np.linalg.norm(force_breakdown['total']))
    guidance['attitude_torque_used_fraction'] = (
        float(control['torque_magnitude']) / max(float(available_torque), 1e-9)
        if available_torque > 0.0 else 0.0
    )

    return new_state, guidance, control, actuator, gs

"""Orbit-insertion and coast guidance functions.

Launch-guidance reference context:
    docs/MATHEMATICAL_REFERENCES.md#5-aerodynamic-forces-and-ascent-guidance

These are project steering and terminal-correction laws, not a verified IGM,
PEG, or trajectory-optimization implementation.
"""


import numpy as np

from . import constants as C
from ._guidance_common import (
    GuidanceState,
    _limit_aoa,
    _resolve_guidance_state,
    compute_local_frame,
    compute_local_horizontal,
    compute_local_vertical,
)
from ._mission_manager_helpers import compute_orbit_metrics
from ._types import GuidanceOutput
from .config_definition import SimulationConfig
from .recovery import estimate_suicide_burn
from .state import State
from .utils import compute_relative_velocity, vec_norm


def _circularization_command(
    r_mag, v_horiz, v_circ_here, target_alt, apogee_alt, apo_burn_limit,
    s2_prop_available, perigee_needs_raise, circularization_settling,
) -> dict:
    """Tangential apsidal burn toward the vis-viva speed for the target orbit."""
    desired_radius = C.R_EARTH + float(target_alt)
    vis_viva_term = 2.0 / max(r_mag, 1.0) - 1.0 / max(desired_radius, 1.0)
    target_tangential_speed = (
        float(np.sqrt(C.MU_EARTH * vis_viva_term))
        if vis_viva_term > 0.0
        else v_circ_here
    )
    dv_to_target_orbit = target_tangential_speed - v_horiz
    thrust_on = (
        s2_prop_available > 0.0
        and perigee_needs_raise
        and dv_to_target_orbit > 0.10
        and apogee_alt <= apo_burn_limit
    )
    if thrust_on and circularization_settling:
        throttle = 0.40
    elif thrust_on:
        throttle = float(np.clip(dv_to_target_orbit / 25.0, 0.50, 1.0))
    else:
        throttle = 0.0
    return {
        "thrust_on": thrust_on,
        "throttle": throttle,
        "target_tangential_speed": float(target_tangential_speed),
        "dv_to_target": float(dv_to_target_orbit),
    }


def _raise_apogee_command(
    r_mag, v_horiz, v_radial, altitude, m, target_alt, alt_tol,
    apogee_alt, raise_apogee_cutoff, s2_prop_available, config,
) -> tuple[float, bool, float]:
    """Pitch/throttle that track a radial-velocity target while raising apogee."""
    g_eff = (C.MU_EARTH / (r_mag ** 2)) - (v_horiz ** 2 / r_mag)
    v_radial_target = 0.0005 * max(0.0, target_alt - apogee_alt)
    if altitude < 120000.0:
        v_radial_target = max(v_radial_target, 25.0 * (1.0 - altitude / 120000.0))
    a_req_vert = 0.12 * (v_radial_target - v_radial)
    s2_thrust_vac = (
        float(config.stage2_thrust_vac) if config is not None else C.STAGE2_THRUST
    )
    a_thrust = s2_thrust_vac / max(m, 1.0)
    sin_pitch = (a_req_vert + g_eff) / max(a_thrust, 0.1)
    pitch_target_rad = np.arcsin(np.clip(sin_pitch, -0.99, 0.99))
    pitch_target_deg = float(np.clip(np.degrees(pitch_target_rad), -5.0, 75.0))
    thrust_on = s2_prop_available > 0.0 and apogee_alt < raise_apogee_cutoff
    apo_room = max(0.0, raise_apogee_cutoff - apogee_alt)
    throttle = float(np.clip(apo_room / max(8000.0, 0.3 * alt_tol), 0.40, 1.0))
    return pitch_target_deg, thrust_on, throttle


def _blend_from_start_direction(gs: GuidanceState, t: float, target_dir: np.ndarray) -> np.ndarray:
    """Smoothstep from the insertion start direction to the target over 20 s."""
    dt_since_start = t - gs.oi_start_time
    ramp_duration = 20.0
    if dt_since_start < ramp_duration and gs.oi_start_direction is not None:
        x = dt_since_start / ramp_duration
        blend = x * x * (3.0 - 2.0 * x)
        blended = (1.0 - blend) * gs.oi_start_direction + blend * target_dir
        blended_norm = vec_norm(blended)
        return blended / blended_norm if blended_norm > 1e-9 else target_dir
    return target_dir


def _select_insertion_mode(
    gs: GuidanceState, t: float, altitude: float, v_radial: float, metrics: dict,
    target_alt: float, alt_tol: float, ecc_max: float, s2_prop_available: float,
) -> dict:
    """Pick the orbit-insertion guidance mode and update the circularisation latch."""
    apogee_alt = metrics["apogee_alt"]
    perigee_alt = metrics["perigee_alt"]
    ecc = metrics["ecc"]

    lower_alt = target_alt - alt_tol
    upper_alt = target_alt + alt_tol
    # First place apogee at the requested orbital radius.  Circularisation is
    # then an apsidal, tangential burn at that radius, as assumed by the
    # vis-viva target below.
    raise_apogee_cutoff = target_alt
                                                                           
                                                                             
                                                             
    # Circularisation is governed by the configured orbital-element acceptance
    # band.  Do not introduce a second, tighter apogee ceiling here: vis-viva
    # computes the tangential speed for the requested semi-major axis, and the
    # mission manager independently verifies both apsides and eccentricity.
    apo_burn_limit = upper_alt
    orbit_ok = (
        metrics["energy"] < 0.0
        and perigee_alt > 100000.0  # Ensure perigee is outside atmosphere
        and abs(perigee_alt - target_alt) <= alt_tol
        and abs(apogee_alt - target_alt) <= alt_tol
        and ecc <= ecc_max
    )
    if apogee_alt >= raise_apogee_cutoff:
        gs.apogee_raise_complete = True
    apogee_below_band = (not gs.apogee_raise_complete) and (apogee_alt < raise_apogee_cutoff)
    apogee_usable = gs.apogee_raise_complete or apogee_alt >= raise_apogee_cutoff
    apogee_too_high = apogee_alt > upper_alt
    perigee_needs_raise = perigee_alt < lower_alt
    distance_to_apogee = apogee_alt - altitude
                                                                          
    at_apogee_peak = altitude >= apogee_alt - 30000.0 and abs(v_radial) <= 60.0
    ready_to_start_circularization = at_apogee_peak
    can_circularize_this_pass = (
        perigee_needs_raise
        and apogee_usable
        and apogee_alt <= apo_burn_limit
        and s2_prop_available > 0.0
    )
    if orbit_ok or not can_circularize_this_pass:
        gs.orbit_circularization_active = False
        gs.orbit_circularization_start_time = None
    elif ready_to_start_circularization or (
        gs.orbit_circularization_active
        and abs(v_radial) <= 60.0
        and altitude >= apogee_alt - 30000.0
        and apogee_alt <= apo_burn_limit
    ):
        if not gs.orbit_circularization_active or gs.orbit_circularization_start_time is None:
            gs.orbit_circularization_start_time = t
        gs.orbit_circularization_active = True
    near_or_approaching_apogee = (
        ready_to_start_circularization
        or bool(gs.orbit_circularization_active)
    )
    circularization_settle_time_s = 1.5
    circularization_settling = (
        gs.orbit_circularization_active
        and gs.orbit_circularization_start_time is not None
        and (t - gs.orbit_circularization_start_time) < circularization_settle_time_s
    )
    circularization_pending = (
        perigee_needs_raise
        and apogee_usable
        and not near_or_approaching_apogee
    )

    if orbit_ok:
        orbit_guidance_mode = "HOLD_ORBIT"
    elif s2_prop_available <= 0.0:
        orbit_guidance_mode = "FAILED_UNREACHABLE"
    elif apogee_below_band:
        orbit_guidance_mode = "RAISE_APOGEE"
    elif perigee_needs_raise and apogee_alt <= apo_burn_limit and near_or_approaching_apogee:
        orbit_guidance_mode = "CIRCULARIZE"
    elif perigee_needs_raise:
        orbit_guidance_mode = "COAST_TO_CIRCULARIZATION"
    else:
                                                                               
                                                                            
        orbit_guidance_mode = "HOLD_ORBIT" if not apogee_too_high else "COAST_TO_CIRCULARIZATION"
    return {
        "orbit_guidance_mode": orbit_guidance_mode,
        "apo_burn_limit": apo_burn_limit,
        "raise_apogee_cutoff": raise_apogee_cutoff,
        "perigee_needs_raise": perigee_needs_raise,
        "circularization_settling": circularization_settling,
        "circularization_pending": circularization_pending,
        "ready_to_start_circularization": ready_to_start_circularization,
        "distance_to_apogee": distance_to_apogee,
    }


def compute_orbit_insertion_guidance(
    r: np.ndarray,
    v: np.ndarray,
    t: float,
    m: float,
    gs: GuidanceState | None = None,
    config: SimulationConfig | None = None,
) -> tuple[GuidanceOutput, GuidanceState]:
    """Guidance logic for Stage 2 orbit insertion."""
    gs = _resolve_guidance_state(gs)

    altitude = float(vec_norm(r) - C.R_EARTH)
    v_inertial = float(vec_norm(v))
    r_mag = float(vec_norm(r))
    vertical = compute_local_vertical(r)

    target_alt = (
        float(config.orbit_target_altitude_m)
        if config is not None
        else float(C.TARGET_ORBIT_ALTITUDE)
    )
    alt_tol = (
        float(config.orbit_altitude_tolerance_m)
        if config is not None
        else 25000.0
    )
    ecc_max = float(config.orbit_ecc_max) if config is not None else 0.01
    v_circ_here = float(np.sqrt(C.MU_EARTH / r_mag))

    v_radial = float(np.dot(v, vertical))
    v_horiz_vec = v - v_radial * vertical
    v_horiz = float(vec_norm(v_horiz_vec))
    if v_horiz > 10.0:
        horiz_hat = v_horiz_vec / v_horiz
    else:
        _, east, _ = compute_local_frame(r)
        horiz_hat = east

    if gs.oi_start_time is None:
        gs.oi_start_time = t
        if gs.last_ascent_direction is not None:
            gs.oi_start_direction = gs.last_ascent_direction.copy()
        elif v_inertial > 10.0:
            gs.oi_start_direction = v / v_inertial
        else:
            gs.oi_start_direction = vertical.copy()

    s2_dry = float(config.stage2_dry_mass) + float(config.payload_mass) if config is not None else C.STAGE2_DRY_MASS
    orbit_state = State(
        r=np.asarray(r, dtype=float),
        v=np.asarray(v, dtype=float),
        q=np.array([1.0, 0.0, 0.0, 0.0]),
        omega=np.zeros(3),
        m=float(m),
        t=float(t),
        dry_mass_kg=s2_dry,
    )
    metrics = compute_orbit_metrics(orbit_state)
    apogee_alt = metrics["apogee_alt"]
    perigee_alt = metrics["perigee_alt"]
    ecc = metrics["ecc"]
    s2_prop_remaining = metrics["s2_prop_remaining"]

    # Reserve propellant for the S2_LANDING drone-ship burn when S2
    # recovery is enabled -- previously orbit-insertion burned propellant
    # with no awareness of what the landing burn would need afterward; see
    # s2_landing_propellant_reserve_kg's docstring in config_definition.py
    # for the failing-run evidence this fixes. Only applied when recovery
    # is actually enabled, so a non-recovery mission's orbit-insertion
    # capability is unaffected.
    landing_reserve_kg = (
        float(config.s2_landing_propellant_reserve_kg)
        if config is not None and bool(getattr(config, "enable_s2_recovery", False))
        else 0.0
    )
    s2_prop_available = s2_prop_remaining - landing_reserve_kg

    mode = _select_insertion_mode(
        gs, t, altitude, v_radial, metrics, target_alt, alt_tol, ecc_max, s2_prop_available,
    )
    orbit_guidance_mode = mode["orbit_guidance_mode"]
    apo_burn_limit = mode["apo_burn_limit"]
    raise_apogee_cutoff = mode["raise_apogee_cutoff"]
    perigee_needs_raise = mode["perigee_needs_raise"]
    circularization_settling = mode["circularization_settling"]
    circularization_pending = mode["circularization_pending"]
    ready_to_start_circularization = mode["ready_to_start_circularization"]
    distance_to_apogee = mode["distance_to_apogee"]

    circularization = None
    if orbit_guidance_mode == "CIRCULARIZE":
        circularization = _circularization_command(
            r_mag, v_horiz, v_circ_here, target_alt, apogee_alt, apo_burn_limit,
            s2_prop_available, perigee_needs_raise, circularization_settling,
        )
        pitch_target_deg = 0.0
        thrust_on = circularization["thrust_on"]
        throttle = circularization["throttle"]
    elif orbit_guidance_mode == "RAISE_APOGEE":
        pitch_target_deg, thrust_on, throttle = _raise_apogee_command(
            r_mag, v_horiz, v_radial, altitude, m, target_alt, alt_tol,
            apogee_alt, raise_apogee_cutoff, s2_prop_available, config,
        )
    else:
        pitch_target_deg = 0.0
        thrust_on = False
        throttle = 0.0

    pitch_target_rad = np.radians(pitch_target_deg)
    target_dir = (
        np.cos(pitch_target_rad) * horiz_hat
        + np.sin(pitch_target_rad) * vertical
    )
    target_dir = target_dir / max(vec_norm(target_dir), 1e-12)

    desired_dir = _blend_from_start_direction(gs, t, target_dir)

    if altitude < C.AERO_DISABLE_ALTITUDE:
        desired_dir = _limit_aoa(desired_dir, v, np.radians(15.0))

    cos_pitch = np.clip(np.dot(vertical, desired_dir), -1.0, 1.0)
    pitch_angle = float(np.arccos(cos_pitch))

    wind_offset = float(config.runtime_wind_offset_mps) if config is not None else 0.0
    v_rel = compute_relative_velocity(r, v, wind_offset_mps=wind_offset)
    v_rel_norm = float(vec_norm(v_rel))
    v_vert = float(np.dot(v_rel, vertical))
    v_horiz_vec_rel = v_rel - v_vert * vertical
    v_horiz_rel = float(vec_norm(v_horiz_vec_rel))
    gamma_actual = float(np.degrees(np.arctan2(v_vert, max(v_horiz_rel, 1e-6))))

    if v_inertial > 1.0:
        prograde = v / v_inertial
    else:
        prograde = vertical

    output = {
        'thrust_direction': desired_dir,
        'phase': "ORBIT_INSERTION",
        'thrust_on': thrust_on,
        'pitch_angle': pitch_angle,
        'gamma_angle': np.radians(gamma_actual),
        'gamma_command_deg': gamma_actual,
        'gamma_measured_deg': gamma_actual,
        'velocity_tilt_deg': gamma_actual,
        'blend_alpha': 1.0,
        'altitude': altitude,
        'velocity': v_inertial,
        'local_vertical': vertical,
        'local_horizontal': compute_local_horizontal(r, v),
        'prograde': prograde,
        'throttle': throttle,
        'v_rel': v_rel,
        'v_rel_mag': v_rel_norm,
        'orbit_guidance_mode': orbit_guidance_mode,
        'orbit_perigee_altitude_m': float(perigee_alt),
        'orbit_apogee_altitude_m': float(apogee_alt),
        'orbit_eccentricity': float(ecc),
        'orbit_radial_velocity_mps': float(v_radial),
        'propellant_remaining_kg': float(max(0.0, s2_prop_remaining)),
        'circularization_pending': bool(circularization_pending),
        'circularization_active': bool(gs.orbit_circularization_active),
        'circularization_start_ready': bool(ready_to_start_circularization),
        'circularization_settling': bool(circularization_settling),
        'distance_to_apogee_m': float(distance_to_apogee),
    }
    if circularization is not None:
        output['orbit_target_tangential_velocity_mps'] = circularization["target_tangential_speed"]
        output['orbit_tangential_velocity_mps'] = float(v_horiz)
        output['orbit_dv_to_target_mps'] = circularization["dv_to_target"]
    if orbit_guidance_mode == "FAILED_UNREACHABLE":
        # This mode is only selected when s2_prop_available <= 0.
        output['orbit_guidance_failure_reason'] = "Stage 2 propellant exhausted before target orbit"
    return output, gs


def compute_deorbit_guidance(
    r: np.ndarray,
    v: np.ndarray,
    t: float,
    m: float,
    gs: GuidanceState | None = None,
    config: SimulationConfig | None = None,
) -> tuple[GuidanceOutput, GuidanceState]:
    """Stage 2 deorbit burn: one short, hard retrograde burn that drops perigee
    deep below the surface for a steep, clean atmospheric entry.

    A single full-throttle burst (only ~100-150 m/s) is near-impulsive, so it
    cleanly lowers the opposite apsis without warping the orbit. Driving perigee
    well below the surface (negative target) makes the descent steep instead of
    a shallow skip that skims thousands of km downrange. The burn cuts the
    instant the target perigee is reached.
    """
    gs = _resolve_guidance_state(gs)

    altitude = float(vec_norm(r) - C.R_EARTH)
    v_inertial = float(vec_norm(v))
    vertical = compute_local_vertical(r)
    retrograde = -v / v_inertial if v_inertial > 1.0 else -vertical

    target_perigee_m = (
        float(config.s2_deorbit_target_perigee_m) if config is not None else -80000.0
    )
    s2_dry = float(config.stage2_dry_mass) + float(config.payload_mass) if config is not None else C.STAGE2_DRY_MASS
    metrics = compute_orbit_metrics(
        State(r=np.asarray(r, float), v=np.asarray(v, float),
              q=np.array([1.0, 0.0, 0.0, 0.0]), omega=np.zeros(3), m=float(m), t=float(t),
              dry_mass_kg=s2_dry)
    )
    perigee_alt = metrics["perigee_alt"]
    s2_prop_remaining = metrics["s2_prop_remaining"]
    # Same landing-propellant reserve as compute_orbit_insertion_guidance
    # (see s2_landing_propellant_reserve_kg's docstring in
    # config_definition.py); the deorbit burn is capped at only ~300 m/s
    # so it is a much smaller consumer than orbit insertion, but should
    # still respect the same reserve rather than potentially tipping an
    # already-tight remaining margin below what the landing burn needs.
    landing_reserve_kg = (
        float(config.s2_landing_propellant_reserve_kg)
        if config is not None and bool(getattr(config, "enable_s2_recovery", False))
        else 0.0
    )
    s2_prop_available = s2_prop_remaining - landing_reserve_kg

                                                                              
                                                                         
                                                                          
                                                                              
                                                                           
                                                   
    if gs.deorbit_start_mass is None:
        gs.deorbit_start_mass = float(m)
                                                                                
                                                                                
                                                                           
                                                                                
                                         
    dv_used = max(
        0.0,
        float(C.STAGE2_ISP_VAC * C.G0 * np.log(max(gs.deorbit_start_mass, 1.0) / max(m, 1.0))),
    )
    dv_cap_mps = 300.0

    if perigee_alt <= target_perigee_m or s2_prop_available <= 0.0 or dv_used >= dv_cap_mps:
        thrust_on = False
        throttle = 0.0
    else:
        thrust_on = True
        throttle = 1.0

    wind_offset = float(config.runtime_wind_offset_mps) if config is not None else 0.0
    v_rel = compute_relative_velocity(r, v, wind_offset_mps=wind_offset)
    v_rel_norm = float(vec_norm(v_rel))
    cos_pitch = float(np.clip(np.dot(vertical, retrograde), -1.0, 1.0))

    output = {
        'thrust_direction': retrograde,
        'phase': "S2_DEORBIT",
        'thrust_on': thrust_on,
        'pitch_angle': float(np.arccos(cos_pitch)),
        'gamma_angle': 0.0,
        'gamma_command_deg': 0.0,
        'gamma_measured_deg': 0.0,
        'velocity_tilt_deg': 0.0,
        'blend_alpha': 1.0,
        'altitude': altitude,
        'velocity': v_inertial,
        'local_vertical': vertical,
        'local_horizontal': compute_local_horizontal(r, v),
        'prograde': v / v_inertial if v_inertial > 1.0 else vertical,
        'throttle': throttle,
        'v_rel': v_rel,
        'v_rel_mag': v_rel_norm,
        'propellant_remaining_kg': float(max(0.0, s2_prop_remaining)),
        'orbit_perigee_altitude_m': float(perigee_alt),
        'orbit_apogee_altitude_m': float(metrics["apogee_alt"]),
    }
    return output, gs


def compute_s2_entry_landing_guidance(
    r: np.ndarray,
    v: np.ndarray,
    t: float,
    m: float,
    phase: str,
    gs: GuidanceState | None = None,
    config: SimulationConfig | None = None,
) -> tuple[GuidanceOutput, GuidanceState]:
    """Stage 2 atmospheric entry + propulsive drone-ship landing.

    S2_ENTRY:   unpowered aero-braking. Hold retrograde (engine/heat-shield into
                the airflow) while the atmosphere bleeds off orbital velocity.
    S2_LANDING: vertical suicide-burn that nulls the descent rate for a soft
                touchdown on the drone ship positioned under the descent point.
    """
    gs = _resolve_guidance_state(gs)

    altitude = float(vec_norm(r) - C.R_EARTH)
    wind_offset = float(config.runtime_wind_offset_mps) if config is not None else 0.0
    v_rel = compute_relative_velocity(r, v, wind_offset_mps=wind_offset)
    v_rel_norm = float(vec_norm(v_rel))
    vertical = compute_local_vertical(r)
    prograde = v_rel / v_rel_norm if v_rel_norm > 1.0 else vertical
    retrograde = -prograde
    s2_dry = float(config.stage2_dry_mass) + float(config.payload_mass) if config is not None else C.STAGE2_DRY_MASS
    s2_thrust = float(config.stage2_thrust_vac) if config is not None else C.STAGE2_THRUST
    s2_isp = float(config.stage2_isp_vac) if config is not None else C.STAGE2_ISP_VAC
    s2_prop_remaining = max(0.0, m - s2_dry)
    v_descent = max(-float(np.dot(v_rel, vertical)), 0.0)

    thrust_on = False
    throttle = 0.0
    desired_dir = retrograde

                                                                                
                                                                            
                                                                        
                                                                                
                                                         
    if phase == "S2_ENTRY" and altitude < 20000.0:
        desired_dir = vertical

    if phase == "S2_LANDING":
                                                             
                                                                               
                                                                                
                                                                                
                                                                               
                                                                             
                                                        
                                                                               
                                                         
         
                                                                      
                                                                               
                                                                             
                                                                                
        ignite_sf = (
            float(config.booster_landing_ignition_safety_factor)
            if config is not None else 1.35
        )
                                                                                
                                                                               
                                                  
        burn = estimate_suicide_burn(
            r,
            v,
            m,
            s2_thrust,
            safety_factor=ignite_sf,
            isp=s2_isp,
            dry_mass_kg=s2_dry,
        )
        ignite_altitude = max(float(burn["burn_altitude"]) * ignite_sf, 1200.0)
        should_burn = (
            s2_prop_remaining > 0.0
            and v_rel_norm > 0.5
            and (altitude <= ignite_altitude or gs.booster_landing_burn_started)
        )
        if should_burn:
            gs.booster_landing_burn_started = True
            thrust_on = True
            g_loc = float(C.MU_EARTH / max(vec_norm(r) ** 2, 1.0))
            t_accel = float(s2_thrust / max(m, 1.0))
            h_stop = max(altitude - 40.0, 1.0)

            v_vert_signed = float(np.dot(v_rel, vertical))
            v_descent = max(-v_vert_signed, 0.0)
            v_horiz_vec = v_rel - v_vert_signed * vertical
            v_horiz_mag = float(vec_norm(v_horiz_vec))

            a_vert = v_descent ** 2 / (2.0 * h_stop) + g_loc
            tau_h = 8.0
            a_horiz = v_horiz_mag / tau_h
            a_cmd = a_vert * vertical
            if v_horiz_mag > 1e-6:
                a_cmd = a_cmd - a_horiz * (v_horiz_vec / v_horiz_mag)
            a_cmd_mag = float(vec_norm(a_cmd))
            desired_dir = a_cmd / max(a_cmd_mag, 1e-6)
            throttle = float(np.clip(a_cmd_mag / max(t_accel, 1e-6), 0.0, 1.0))
            if altitude < 3.0 and v_rel_norm < 1.5:
                thrust_on = False
                throttle = 0.0

    cos_pitch = float(np.clip(np.dot(vertical, desired_dir), -1.0, 1.0))
    v_vert = float(np.dot(v_rel, vertical))
    v_horiz = float(vec_norm(v_rel - v_vert * vertical))
    gamma_deg = float(np.degrees(np.arctan2(v_vert, max(v_horiz, 1e-6)))) if v_rel_norm > 1.0 else -90.0

    output = {
        'thrust_direction': desired_dir,
        'phase': phase,
        'thrust_on': thrust_on,
        'pitch_angle': float(np.arccos(cos_pitch)),
        'gamma_angle': np.radians(gamma_deg),
        'gamma_command_deg': gamma_deg,
        'gamma_measured_deg': gamma_deg,
        'velocity_tilt_deg': gamma_deg,
        'blend_alpha': 1.0,
        'altitude': altitude,
        'velocity': float(vec_norm(v)),
        'local_vertical': vertical,
        'local_horizontal': compute_local_horizontal(r, v),
        'prograde': prograde,
        'throttle': throttle,
        'v_rel': v_rel,
        'v_rel_mag': v_rel_norm,
        'propellant_remaining_kg': float(s2_prop_remaining),
    }
    return output, gs


def compute_coast_guidance(
    r: np.ndarray, v: np.ndarray, t: float, m: float,
    gs: GuidanceState | None = None,
    config: SimulationConfig | None = None,
) -> tuple[GuidanceOutput, GuidanceState]:
    """Guidance logic for unpowered coast."""
    gs = _resolve_guidance_state(gs)

    altitude = float(vec_norm(r) - C.R_EARTH)
    s2_dry = float(config.stage2_dry_mass) + float(config.payload_mass) if config is not None else C.STAGE2_DRY_MASS
    wind_offset = float(config.runtime_wind_offset_mps) if config is not None else 0.0
    v_rel = compute_relative_velocity(r, v, wind_offset_mps=wind_offset)
    v_rel_norm = float(vec_norm(v_rel))
    v_inertial = float(vec_norm(v))

    if v_inertial > 1.0:
        prograde = v / v_inertial
    else:
        prograde = compute_local_vertical(r)

    if gs.last_ascent_direction is not None:
        desired_dir = gs.last_ascent_direction.copy()
        if v_rel_norm > 10.0:
            desired_dir = _limit_aoa(desired_dir, v_rel, np.radians(15.0))
    else:
        desired_dir = prograde

    vertical = compute_local_vertical(r)
    cos_pitch = np.clip(np.dot(vertical, desired_dir), -1.0, 1.0)
    pitch_angle = float(np.arccos(cos_pitch))

    v_vert = float(np.dot(v_rel, vertical))
    v_horiz_vec = v_rel - v_vert * vertical
    v_horiz = float(vec_norm(v_horiz_vec))
    gamma_actual = float(np.degrees(np.arctan2(v_vert, v_horiz)))

    output = {
        'thrust_direction': desired_dir,
        'phase': "COAST_PHASE",
        'thrust_on': False,
        'pitch_angle': pitch_angle,
        'gamma_angle': np.radians(gamma_actual),
        'gamma_command_deg': gamma_actual,
        'gamma_measured_deg': gamma_actual,
        'velocity_tilt_deg': gamma_actual,
        'blend_alpha': 1.0,
        'altitude': altitude,
        'velocity': float(vec_norm(v)),
        'local_vertical': vertical,
        'local_horizontal': compute_local_horizontal(r, v),
        'prograde': desired_dir,
        'throttle': 0.0,
        'v_rel': v_rel,
        'v_rel_mag': v_rel_norm,
        'propellant_remaining_kg': float(max(0.0, m - s2_dry)),
    }
    return output, gs

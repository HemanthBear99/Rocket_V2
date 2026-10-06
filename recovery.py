"""
Shared recovery-physics helpers for booster guidance and mission transitions.

Powered-descent references and heuristic limitations:
    docs/MATHEMATICAL_REFERENCES.md#11-powered-descent-and-recovery-guidance

The module uses standard two-body, work-energy, and rocket-equation building
blocks, but its ignition margins, time-to-go approximations, capture gates, and
throttle commands are not a complete constrained optimal-guidance derivation.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import constants as C
from .config_definition import SimulationConfig
from .utils import compute_ground_relative_velocity

NEAR_PAD_ENTRY_SPEED_GATE_MPS = 250.0

                                                                          
ENTRY_BURN_EXIT_SPEED_MPS = 150.0                                                 
ENTRY_MASS_FRACTION = 0.55                                                                   
                                                                                             
ENTRY_BURN_THROTTLE_FRACTION = 0.45                                                        
                                                                                                  
ENTRY_RETROGRADE_EFFICIENCY = 0.89                                                             
                                                                                         
TC_AND_LANDING_TIME_S = 40.0                                                                 


@dataclass(frozen=True)
class RecoveryTargetEstimate:
    """Shared RTLS target estimate for boostback guidance and cutoff logic."""

    coast_time_s: float
    touchdown_time_s: float
    landing_site_eci: np.ndarray
    toward_site: np.ndarray
    site_distance_m: float
    v_return_needed_mps: float
    v_toward_site_mps: float
    predicted_landing_error_m: float


@dataclass(frozen=True)
class BallisticImpactEstimate:
    """3DOF no-thrust impact estimate used as a guidance predictor."""

    time_to_impact_s: float
    impact_site_eci: np.ndarray
    target_site_eci: np.ndarray
    miss_vector_m: np.ndarray
    miss_distance_m: float


def _fixed_surface_site_eci(
    t: float,
    lat_deg: float,
    lon_deg: float,
    altitude_m: float = 0.0,
) -> np.ndarray:
    """Rotate a fixed spherical-Earth site from the Earth-fixed frame into ECI."""
    site_ecef = C.surface_position_from_lat_lon_deg(
        lat_deg,
        lon_deg,
        altitude_m=altitude_m,
    )
    theta = C.EARTH_ROTATION_RATE * float(t)
    c_t = float(np.cos(theta))
    s_t = float(np.sin(theta))
    x0, y0, z0 = site_ecef
    return np.array([
        c_t * x0 - s_t * y0,
        s_t * x0 + c_t * y0,
        z0,
    ], dtype=float)


def rotating_launch_site_eci(
    t: float,
    config: SimulationConfig | None = None,
) -> np.ndarray:
    """Launch-site position in ECI at time t (Earth rotation applied)."""
    if config is None:
        return _fixed_surface_site_eci(
            t,
            C.DEFAULT_LAUNCH_SITE_LAT_DEG,
            C.DEFAULT_LAUNCH_SITE_LON_DEG,
            altitude_m=0.0,
        )
    return _fixed_surface_site_eci(
        t,
        float(config.launch_site_lat_deg),
        float(config.launch_site_lon_deg),
        altitude_m=float(config.launch_site_altitude_m),
    )


def target_landing_site_eci(
    t: float,
    downrange_km: float,
    config: SimulationConfig | None = None,
) -> np.ndarray:
    """
    RTLS landing-site position in ECI.

    `downrange_km` is a surface-arc offset from the launch site along local east.

    When `config.enable_movable_landing_target` is set, the offset is advanced
    by `movable_target_downrange_km()` so the target sits where wind carries
    the vehicle (drone-ship style). Every consumer -- guidance, impact
    prediction, grid-fin ZEM/ZEV, and the terminal pad gates -- routes through
    this function, so the whole recovery stack sees one consistent target.
    """
    downrange_km = movable_target_downrange_km(downrange_km, config=config)
    if (
        config is not None and
        config.booster_landing_site_lat_deg is not None and
        config.booster_landing_site_lon_deg is not None
    ):
        return _fixed_surface_site_eci(
            t,
            config.booster_landing_site_lat_deg,
            config.booster_landing_site_lon_deg,
            altitude_m=config.booster_landing_site_altitude_m,
        )

    launch = rotating_launch_site_eci(t, config=config)
    launch_hat = launch / max(np.linalg.norm(launch), 1e-9)
    k_axis = np.array([0.0, 0.0, 1.0], dtype=float)
    east = np.cross(k_axis, launch_hat)
    east_norm = float(np.linalg.norm(east))
    if east_norm < 1e-9:
        east = np.array([0.0, 1.0, 0.0], dtype=float)
        east_norm = 1.0
    east = east / east_norm

    theta = float(downrange_km) * 1000.0 / C.R_EARTH
    target_hat = np.cos(theta) * launch_hat + np.sin(theta) * east
    target_hat = target_hat / max(np.linalg.norm(target_hat), 1e-9)
    return C.R_EARTH * target_hat


def movable_target_downrange_km(
    base_downrange_km: float,
    config: SimulationConfig | None = None,
) -> float:
    """Effective target downrange including wind-dispersed target placement.

    Falcon-class recovery meets a MOVABLE target: the drone ship is towed
    downrange so the vehicle's wind-drifted trajectory intersects it, rather
    than the vehicle being forced onto a fixed pad. This reproduces that
    dispatch layer analytically.

    The vehicle is exposed to the wind for roughly
    `landing_target_wind_exposure_s` of its recovery flight, so the wind
    advects it by `v_wind * t_exposure` along the local east axis. That
    along-track displacement is the downrange offset applied to the target,
    scaled by `landing_target_wind_gain` and clamped to
    `landing_target_max_downrange_offset_km`.

    Returns `base_downrange_km` unchanged when
    `enable_movable_landing_target` is False (the default), so existing
    fixed-pad RTLS behaviour is bit-for-bit preserved.

    Note this models target placement only. It deliberately does not change
    the vehicle's physics or its lateral control authority -- it represents
    the operational decision of where to put the recovery asset, which is
    what actually buys crosswind tolerance on the real vehicle.
    """
    base = float(base_downrange_km)
    if config is None:
        return base
    if not bool(getattr(config, "enable_movable_landing_target", False)):
        return base

    from .utils import _wind_vector

    wind_offset = float(getattr(config, "runtime_wind_offset_mps", 0.0))
    if abs(wind_offset) <= 1e-9:
        return base

    # Sample the wind at a representative mid-descent altitude and project it
    # onto the local east axis (the axis the downrange offset is measured on).
    t_sample = 0.0
    r_sample = rotating_launch_site_eci(t_sample, config=config)
    altitude_m = 20000.0
    probe = r_sample + altitude_m * (r_sample / max(float(np.linalg.norm(r_sample)), 1e-9))
    wind = _wind_vector(probe, wind_offset_mps=wind_offset)

    r_hat = r_sample / max(float(np.linalg.norm(r_sample)), 1e-9)
    k_axis = np.array([0.0, 0.0, 1.0], dtype=float)
    east = np.cross(k_axis, r_hat)
    east_norm = float(np.linalg.norm(east))
    if east_norm < 1e-9:
        return base
    east = east / east_norm

    # The wind model already has a substantial BASE profile (~34 m/s along-track
    # at the sample altitude with zero offset), which the nominal mission is
    # already trimmed against. Only the DELTA attributable to
    # runtime_wind_offset_mps represents new, un-compensated dispersion.
    # Using the total wind here over-shifted the target by ~4x (a +12 m/s
    # offset shifted the target as if it were +46 m/s) and made the landing
    # error worse rather than better. Subtract the zero-offset baseline.
    wind_baseline = _wind_vector(probe, wind_offset_mps=0.0)
    along_track_mps = float(np.dot(wind, east) - np.dot(wind_baseline, east))

    exposure_s = float(getattr(config, "landing_target_wind_exposure_s", 150.0))
    gain = float(getattr(config, "landing_target_wind_gain", 1.0))
    max_offset_km = float(getattr(config, "landing_target_max_downrange_offset_km", 40.0))

    offset_m = along_track_mps * exposure_s * gain
    offset_km = float(np.clip(offset_m / 1000.0, -max_offset_km, max_offset_km))
    return base + offset_km


def predicted_landing_site_eci(
    t: float,
    time_to_go_s: float,
    target_downrange_km: float,
    config: SimulationConfig,
) -> np.ndarray:
    """Landing-site ECI position at the predicted touchdown time."""
    lead_time = max(float(time_to_go_s), 0.0)
    return target_landing_site_eci(
        float(t) + lead_time,
        target_downrange_km,
        config=config,
    )


def landing_site_offset_m(config: SimulationConfig | None = None) -> float:
    """
    Surface distance between the launch site and the configured landing target.

    This resolves the actual landing-site geometry first and avoids inferring
    "same-pad RTLS" directly from raw config fields that may not be the active
    source of truth.
    """
    if config is None:
        return 0.0

    launch_site = rotating_launch_site_eci(0.0, config=config)
    target_site = target_landing_site_eci(
        0.0,
        config.booster_landing_target_downrange_km,
        config=config,
    )
    return great_circle_distance_m(launch_site, target_site)


def is_near_pad_target(
    config: SimulationConfig | None = None,
    threshold_m: float = 15000.0,
) -> bool:
    """True when the resolved landing site is within the launch-site complex."""
    return landing_site_offset_m(config) <= float(threshold_m)


def great_circle_distance_m(r_a: np.ndarray, r_b: np.ndarray) -> float:
    """Great-circle surface distance between two geocentric position vectors."""
    a_hat = np.asarray(r_a, dtype=float)
    b_hat = np.asarray(r_b, dtype=float)
    a_hat = a_hat / max(np.linalg.norm(a_hat), 1e-9)
    b_hat = b_hat / max(np.linalg.norm(b_hat), 1e-9)
    angle = float(np.arccos(np.clip(np.dot(a_hat, b_hat), -1.0, 1.0)))
    return C.R_EARTH * angle


def booster_propellant_remaining(mass_kg: float, config: SimulationConfig = None) -> float:
    """Return booster propellant remaining above Stage-1 dry mass."""
    dry_mass = float(config.stage1_dry_mass) if config is not None else C.STAGE1_DRY_MASS
    return max(0.0, float(mass_kg - dry_mass))


def booster_min_propellant_after_boostback(cfg: SimulationConfig) -> float:
    """Absolute reserve required after boostback to protect entry + landing."""
    return float(
        cfg.booster_entry_budget_kg
        + cfg.booster_landing_reserve_kg
        + cfg.booster_landing_propellant_margin_kg
    )


def compute_entry_energy_speed_gate(config: SimulationConfig) -> float:
    """Near-pad RTLS hands off entry braking early enough to preserve divert time."""
    configured_gate = float(config.booster_entry_burn_min_speed_mps)
    if is_near_pad_target(config):
        return max(configured_gate, NEAR_PAD_ENTRY_SPEED_GATE_MPS)
    return configured_gate


def _propagate_2body_to_surface(
    r: np.ndarray,
    v: np.ndarray,
    max_steps: int = 300,
    dt: float = 2.0,
    mass_kg: float | None = None,
    config: SimulationConfig | None = None,
    aero_mode: str | None = None,
) -> tuple[np.ndarray, float] | None:
    """Numerically integrate a no-thrust trajectory to surface intersection.

    Central gravity is always modeled. When mass/config are supplied, use the
    same Mach-dependent drag and deployed-hardware drag scaling as the 6DOF
    booster dynamics. This is essential below entry interface, where a
    gravity-only predictor overestimates horizontal range by tens of kilometres.
    """
    from .forces import _booster_recovery_aero_scale, compute_drag_force

    mu = C.MU_EARTH
    R = C.R_EARTH
    r_cur = np.asarray(r, dtype=float)
    v_cur = np.asarray(v, dtype=float)
    elapsed = 0.0
    include_drag = mass_kg is not None and float(mass_kg) > 0.0
    drag_scale = 1.0
    if include_drag:
        drag_scale, _ = _booster_recovery_aero_scale(
            aero_mode,
            config=config,
        )

    for _ in range(max_steps):
        def accel(pos: np.ndarray, velocity: np.ndarray) -> np.ndarray:
            rn = max(float(np.linalg.norm(pos)), 1.0)
            acceleration = -mu / (rn ** 3) * pos
            if include_drag:
                wind_offset = float(
                    getattr(config, "runtime_wind_offset_mps", 0.0)
                )
                drag = compute_drag_force(
                    pos,
                    velocity,
                    wind_offset_mps=wind_offset,
                    config=config,
                )
                acceleration = (
                    acceleration
                    + drag_scale * drag / max(float(mass_kg), 1.0)
                )
            return acceleration

        a1 = accel(r_cur, v_cur)
        r2 = r_cur + 0.5 * dt * v_cur
        v2 = v_cur + 0.5 * dt * a1
        a2 = accel(r2, v2)
        r3 = r_cur + 0.5 * dt * v2
        v3 = v_cur + 0.5 * dt * a2
        a3 = accel(r3, v3)
        r4 = r_cur + dt * v3
        v4 = v_cur + dt * a3
        a4 = accel(r4, v4)

        r_new = r_cur + (dt / 6.0) * (v_cur + 2.0 * v2 + 2.0 * v3 + v4)
        v_new = v_cur + (dt / 6.0) * (a1 + 2.0 * a2 + 2.0 * a3 + a4)

        r_prev_norm = float(np.linalg.norm(r_cur))
        r_new_norm = float(np.linalg.norm(r_new))
        elapsed += dt

        if r_new_norm <= R:
                                                                 
                                                      
            f = (R - r_new_norm) / max(r_prev_norm - r_new_norm, 1e-12)
            f = float(np.clip(f, 0.0, 1.0))
            r_surface = r_new + f * (r_cur - r_new)
            r_surface = R * r_surface / max(float(np.linalg.norm(r_surface)), 1.0)
            return r_surface, elapsed - dt * (1.0 - f)

        r_cur, v_cur = r_new, v_new

    return None                                          


def estimate_ballistic_impact_to_pad(
    r: np.ndarray,
    v: np.ndarray,
    t: float,
    target_downrange_km: float,
    config: SimulationConfig,
    lead_time_s: float = 0.0,
    mass_kg: float | None = None,
    aero_mode: str | None = None,
) -> BallisticImpactEstimate:
    """
    Predict the no-thrust touchdown miss with numerical propagation.

    Uses RK4 integration under central gravity and, when mass/config are
    supplied, the same passive recovery drag model as the main dynamics.
    This replaces the old straight-line projection ``r + v_horiz * t``.

    The simulator still advances the vehicle with the full 6DOF dynamics,
    attitude, aero, thrust, and actuator model — this is a guidance predictor
    only.
    """
    r_arr = np.asarray(r, dtype=float)
    v_arr = np.asarray(v, dtype=float)
    r_norm = max(float(np.linalg.norm(r_arr)), 1.0)
    vertical = r_arr / r_norm
    altitude = max(r_norm - C.R_EARTH, 0.0)
    g_local = C.MU_EARTH / max(r_norm ** 2, 1.0)

                                                           
    v_vert = float(np.dot(v_arr, vertical))
    discriminant = max(v_vert ** 2 + 2.0 * g_local * altitude, 0.0)
    time_to_impact_est = max((v_vert + float(np.sqrt(discriminant))) / max(g_local, 1e-9), 0.0)

                                                                           
                                                               
    result = _propagate_2body_to_surface(
        r_arr,
        v_arr,
        mass_kg=mass_kg,
        config=config,
        aero_mode=aero_mode,
    )
    if result is not None:
        impact_site, time_to_impact = result
    else:
                                                                              
        v_horiz = v_arr - v_vert * vertical
        impact_vec = r_arr + v_horiz * time_to_impact_est
        impact_hat = impact_vec / max(float(np.linalg.norm(impact_vec)), 1.0)
        impact_site = C.R_EARTH * impact_hat
        time_to_impact = time_to_impact_est

    target_site = target_landing_site_eci(
        float(t) + time_to_impact + float(lead_time_s),
        target_downrange_km,
        config=config,
    )
    target_vertical = target_site / max(float(np.linalg.norm(target_site)), 1.0)
    miss_vec = target_site - impact_site
    miss_horiz = miss_vec - float(np.dot(miss_vec, target_vertical)) * target_vertical

    return BallisticImpactEstimate(
        time_to_impact_s=float(time_to_impact),
        impact_site_eci=impact_site,
        target_site_eci=target_site,
        miss_vector_m=miss_horiz,
        miss_distance_m=float(np.linalg.norm(miss_horiz)),
    )


def estimate_ballistic_apogee(altitude_m: float, radial_velocity_mps: float, g_local: float) -> float:
    """Estimate no-thrust ballistic apogee from current altitude and radial speed."""
    if radial_velocity_mps <= 0.0:
        return float(altitude_m)
    return float(altitude_m + (radial_velocity_mps ** 2) / (2.0 * max(g_local, 1e-6)))


def estimate_recovery_targeting(
    r: np.ndarray,
    v: np.ndarray,
    t: float,
    target_downrange_km: float,
    config: SimulationConfig,
    lead_time_s: float | None = None,
) -> RecoveryTargetEstimate:
    """Estimate the future rotating landing-site intercept for booster RTLS."""
    r = np.asarray(r, dtype=float)
    v = np.asarray(v, dtype=float)
    vertical = r / max(float(np.linalg.norm(r)), 1.0)
    altitude = float(np.linalg.norm(r) - C.R_EARTH)
    radial_velocity = float(np.dot(v, vertical))
    g_local = C.MU_EARTH / max(float(np.linalg.norm(r)) ** 2, 1.0)
    h_apogee_pred = estimate_ballistic_apogee(altitude, radial_velocity, g_local)
    t_to_apogee = max(radial_velocity, 0.0) / max(g_local, 1e-6)
                                                                         
                                                                            
                                                                            
                                                                            
                                                 
    h_entry_iface = float(config.booster_entry_interface_altitude_m)
    t_fall_height = max(h_apogee_pred - h_entry_iface, 0.0)
    t_fall = float(np.sqrt(2.0 * t_fall_height / max(g_local, 1e-6)))
    coast_time = max(t_to_apogee + t_fall, 60.0)
    lead = (
        float(lead_time_s)
        if lead_time_s is not None
        else float(config.booster_powered_descent_lead_time_s)
    )
    touchdown_time = float(t + coast_time + lead)
    landing_site = target_landing_site_eci(
        touchdown_time,
        target_downrange_km,
        config=config,
    )

    r_to_site = landing_site - r
    r_to_site_horiz = r_to_site - float(np.dot(r_to_site, vertical)) * vertical
    site_dist = float(np.linalg.norm(r_to_site_horiz))
    if site_dist > 1e-6:
        toward_site = r_to_site_horiz / site_dist
    else:
        toward_site = np.zeros(3)

    v_horiz = v - float(np.dot(v, vertical)) * vertical
    v_toward_site = float(np.dot(v_horiz, toward_site)) if site_dist > 1e-6 else 0.0
    v_return_needed = float(np.clip(site_dist / (coast_time + lead), 5.0, 450.0))
    predicted_landing_error = max(0.0, abs(v_return_needed - max(v_toward_site, 0.0)) * coast_time)

    return RecoveryTargetEstimate(
        coast_time_s=float(coast_time),
        touchdown_time_s=touchdown_time,
        landing_site_eci=landing_site,
        toward_site=toward_site,
        site_distance_m=site_dist,
        v_return_needed_mps=v_return_needed,
        v_toward_site_mps=v_toward_site,
        predicted_landing_error_m=float(predicted_landing_error),
    )


def compute_powered_descent_lead_time(
    r: np.ndarray,
    v: np.ndarray,
    m: float,
    config: SimulationConfig,
) -> float:
    """Estimate duration of powered descent phases (entry burn + TC + landing)."""
    r_arr = np.asarray(r, dtype=float)
    v_arr = np.asarray(v, dtype=float)
    r_norm = float(np.linalg.norm(r_arr))
    if r_norm < 1.0:
        return float(config.booster_powered_descent_lead_time_s)

    g_local = float(C.MU_EARTH / max(r_norm ** 2, 1.0))
    vertical = r_arr / r_norm
    radial_vel = float(np.dot(v_arr, vertical))
    altitude = r_norm - C.R_EARTH

    h_apogee = altitude + max(radial_vel, 0.0) ** 2 / (2.0 * max(g_local, 1e-9))

    h_entry = float(config.booster_entry_interface_altitude_m)
    h_above_entry = max(h_apogee - h_entry, 0.0)
    v_at_entry = float(np.sqrt(2.0 * g_local * h_above_entry))

    entry_gate = ENTRY_BURN_EXIT_SPEED_MPS
    m_at_entry = max(
        float(m) * ENTRY_MASS_FRACTION,
        C.STAGE1_DRY_MASS + float(config.booster_landing_reserve_kg),
    )
    entry_net_decel = max(
        C.ENTRY_THRUST * ENTRY_BURN_THROTTLE_FRACTION * ENTRY_RETROGRADE_EFFICIENCY
        / max(m_at_entry, 1.0) - g_local,
        1.0,
    )
    entry_time = max(v_at_entry - entry_gate, 0.0) / entry_net_decel

    tc_and_landing_time = TC_AND_LANDING_TIME_S

    powered = entry_time + tc_and_landing_time

    return max(powered, float(config.booster_powered_descent_lead_time_s))


def estimate_suicide_burn(
    r: np.ndarray,
    v: np.ndarray,
    mass_kg: float,
    thrust_newton: float,
    safety_factor: float = 1.5,
    min_throttle: float = 0.3,
    max_throttle: float = 1.0,
    horizontal_weight: float = 1.0,
    isp: float = None,
    dry_mass_kg: float | None = None,
) -> dict[str, float]:
    """
    Variable-mass landing ignition estimator.

    Uses the rocket equation to estimate the propellant needed for the
    current effective speed, then uses the variable-mass mean net
    deceleration to estimate braking altitude.  This replaces the earlier
    constant-acceleration heuristic (v^2 / 2a * safety_factor).

        dv_ideal = Isp * g0 * ln(m0 / mf)
        a_mean = F * ln(m0 / mf) / (m0 - mf) - g
        h_ignite = v_eff^2 / (2 * a_mean)

    The safety factor is applied to ``h_ignite``.  This is a braking-distance
    approximation, not an energy-balance calculation.

    Returns dict with keys: ignite, throttle, burn_altitude, v_descent,
    v_horizontal, v_effective, a_brake.
    """
    r_norm = float(np.linalg.norm(r))
    if r_norm < 1.0:
        return {
            'ignite': False,
            'throttle': 0.0,
            'burn_altitude': 0.0,
            'v_descent': 0.0,
            'a_brake': 0.0,
        }

    altitude = r_norm - C.R_EARTH
    if isp is None:
        from .forces import compute_atmosphere_properties
        _, P_amb, _, _ = compute_atmosphere_properties(altitude)
        pressure_ratio = float(np.clip(P_amb / C.ATM_P0, 0.0, 1.2))
        isp = C.ISP_VAC - (C.ISP_VAC - C.ISP) * pressure_ratio
    dry_mass = C.STAGE1_DRY_MASS if dry_mass_kg is None else float(dry_mass_kg)

    vertical = r / r_norm
    v_ground = compute_ground_relative_velocity(r, v)
    v_descent = -float(np.dot(v_ground, vertical))
    v_horiz_vec = v_ground - np.dot(v_ground, vertical) * vertical
    v_horiz = float(np.linalg.norm(v_horiz_vec))
    v_effective = float(np.sqrt(
        v_descent ** 2 + horizontal_weight * (v_horiz ** 2)
    ))

    if v_descent <= 0.0 and v_effective <= 0.0:
        return {
            'ignite': False,
            'throttle': 0.0,
            'burn_altitude': 0.0,
            'v_descent': v_descent,
            'v_horizontal': v_horiz,
            'v_effective': v_effective,
            'a_brake': 0.0,
        }

    g_local = C.MU_EARTH / (r_norm ** 2)
    thrust_accel = float(thrust_newton / max(mass_kg, 1.0))
    a_brake = thrust_accel - g_local

    if a_brake <= 0.0:
        return {
            'ignite': True,
            'throttle': max_throttle,
            'burn_altitude': max(0.0, altitude),
            'v_descent': v_descent,
            'v_horizontal': v_horiz,
            'v_effective': v_effective,
            'a_brake': a_brake,
        }

                                            
                                                                          
                                                                         
                                                         
     
                        
     
                                                               
                                           
     
                                                                            
                                                                   
                          
                                           
                                                                 
                                                  
     
                                                                        
                                                                   

    ve = isp * C.G0
    propellant = max(mass_kg - dry_mass, 0.0)

                                                                              
                                 
    mass_ratio_needed = np.exp(v_effective / ve)
    m_final_ideal = mass_kg / mass_ratio_needed
    fuel_needed = mass_kg - m_final_ideal

    if fuel_needed > propellant:
                                                           
        m_final_actual = dry_mass
    else:
        m_final_actual = m_final_ideal

                                                                        
    dm = mass_kg - m_final_actual
    if dm > 1.0 and m_final_actual > 0.0:
                                                                        
        a_mean = thrust_newton * np.log(mass_kg / m_final_actual) / dm - g_local
    else:
        a_mean = a_brake

                                                                
                                                                       
                                                                       
                                                                        
                                                                      
    a_mean = max(a_mean, g_local * 0.15)                                  

                                               
    h_ignite = (v_effective ** 2) / (2.0 * a_mean)

                                                      
    ignite = altitude <= h_ignite * safety_factor

    if ignite:
                                                                               
        if altitude > 10.0:
            a_required = (v_effective ** 2) / (2.0 * altitude) + g_local
        else:
            a_required = thrust_accel
        throttle = float(np.clip(
            a_required / max(thrust_accel, 1e-6), min_throttle, max_throttle
        ))
    else:
        throttle = 0.0

    return {
        'ignite': bool(ignite),
        'throttle': float(throttle),
        'burn_altitude': float(h_ignite),
        'v_descent': float(v_descent),
        'v_horizontal': float(v_horiz),
        'v_effective': float(v_effective),
        'a_brake': float(a_brake),
    }


def estimate_booster_touchdown_time(
    r: np.ndarray,
    v: np.ndarray,
    mass_kg: float,
    thrust_newton: float,
    safety_factor: float = 1.5,
) -> float:
    """
    Estimate remaining booster time-to-touchdown for pad-target prediction.

    The old guidance path used `2h / v_descent`, which implicitly assumes the
    current descent profile continues unchanged. That underestimates the real
    remaining time badly once the booster starts braking early with a large
    landing-ignition safety margin, causing entry/landing guidance to target a
    pad position that is behind the actual future landing site.

    This helper uses a two-part approximation instead:
    1. Ballistic coast from current altitude down to the landing ignition band.
    2. A safety-factor-scaled powered-descent horizon inside that ignition band.

    It is still a heuristic, but it tracks the live return timeline much better
    than the raw `2h / v_descent` shortcut and is stable enough for guidance.
    """
    r = np.asarray(r, dtype=float)
    v = np.asarray(v, dtype=float)
    r_norm = float(np.linalg.norm(r))
    if r_norm < 1.0:
        return 0.0

    altitude = max(r_norm - C.R_EARTH, 0.0)
    if altitude <= 0.0:
        return 0.0

    vertical = r / r_norm
    v_rel = compute_ground_relative_velocity(r, v)
                                                                           
                                                                                
                                                                                
                                                                              
                                                                          
                                                                            
    v_descent = max(-float(np.dot(v_rel, vertical)), 1.0)
    g_local = float(C.MU_EARTH / max(r_norm ** 2, 1.0))

    burn = estimate_suicide_burn(
        r,
        v,
        mass_kg,
        thrust_newton,
        safety_factor=safety_factor,
    )
    ignition_altitude = max(float(burn['burn_altitude']) * safety_factor, 0.0)

                                                                            
                                          
     
                                                                           
                                                                           
                                                                         
                                                                            
                                                                            
                                                                                
     
                                                                               
                                                                               
                                                                             
                                                                          
                                                                            
                                                     
    powered_scale = 1.2

    if altitude <= ignition_altitude:
        return powered_scale * 2.0 * altitude / v_descent

    delta_h = altitude - ignition_altitude
    a = 0.5 * g_local
    b = v_descent
    c = -delta_h
    disc = max(b * b - 4.0 * a * c, 0.0)
    t_coast = (-b + float(np.sqrt(disc))) / max(2.0 * a, 1e-9)
    v_ignite = v_descent + g_local * t_coast
    t_power = powered_scale * 2.0 * ignition_altitude / max(v_ignite, 1.0)
    return float(t_coast + t_power)

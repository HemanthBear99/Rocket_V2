"""Ascent guidance functions.

Reference context and provenance classification:
    docs/MATHEMATICAL_REFERENCES.md#5-aerodynamic-forces-and-ascent-guidance

RocketV uses an altitude-scheduled pitch program, PID correction, Max-Q
shaping, and heuristic mode blending. It is not an implementation of a proven
optimal ascent law such as IGM, PEG, or a collocation solution.
"""


import numpy as np

from . import constants as C
from ._guidance_common import (
    GuidanceState,
    _limit_aoa,
    _resolve_guidance_state,
    compute_blend_parameter,
    compute_local_frame,
    compute_local_horizontal,
    compute_local_vertical,
    gamma_profile_from_altitude,
)
from ._types import GuidanceOutput
from .config_definition import SimulationConfig
from .forces import compute_configured_atmosphere_properties
from .utils import compute_relative_velocity, vec_norm


def compute_max_q_throttle(
    dynamic_pressure_pa: float,
    dynamic_pressure_rate_pa_s: float,
    config: SimulationConfig | None = None,
) -> tuple[float, float]:
    """Return predictive Max-Q throttle command and predicted pressure.

    The pressure-rate look-ahead accounts for engine spool-down delay. A
    pressure-ratio trim begins when predicted pressure crosses the target,
    avoiding the deep throttle bucket caused by a hard limiter.
    """
    target = float(
        config.ascent_max_q_target_pa
        if config is not None
        else 30000.0
    )
    horizon = float(
        config.ascent_max_q_prediction_horizon_s
        if config is not None
        else 1.0
    )
    min_throttle = float(
        config.min_engine_throttle_fraction
        if config is not None
        else SimulationConfig.min_engine_throttle_fraction
    )
    max_throttle = float(
        config.max_engine_throttle_fraction
        if config is not None
        else SimulationConfig.max_engine_throttle_fraction
    )

    q_now = max(float(dynamic_pressure_pa), 0.0)
    q_rate_rising = max(float(dynamic_pressure_rate_pa_s), 0.0)
    q_predicted = max(q_now, q_now + q_rate_rising * max(horizon, 0.0))
    if q_predicted <= target:
        throttle = max_throttle
    else:
        pressure_ratio_command = np.sqrt(
            target / max(q_predicted, target)
        )
        throttle = max_throttle * pressure_ratio_command
    return float(np.clip(throttle, min_throttle, max_throttle)), q_predicted


def compute_desired_thrust_direction(
    r: np.ndarray,
    v: np.ndarray,
    t: float,
    gs: GuidanceState | None = None,
    dt: float = C.DT,
    config: SimulationConfig | None = None,
) -> tuple[np.ndarray, float, float, GuidanceState]:
    """Compute desired thrust direction for ascent guidance.

    Args:
        r, v, t: Vehicle state
        gs: Guidance state (uses module default if None)
        dt: Time step

    Returns:
        (thrust_dir, gamma_cmd_deg, gamma_meas_deg, updated_gs)
    """
    gs = _resolve_guidance_state(gs)

    altitude = float(vec_norm(r) - C.R_EARTH)
    vertical, east, north = compute_local_frame(r)
    pitchover_start_alt = (
        float(config.pitchover_start_altitude)
        if config is not None
        else C.PITCHOVER_START_ALTITUDE
    )
    pitchover_end_alt = (
        float(config.pitchover_end_altitude)
        if config is not None
        else C.PITCHOVER_END_ALTITUDE
    )
    pitchover_end_alt = max(pitchover_end_alt, pitchover_start_alt)
    pitchover_angle_cfg = (
        float(config.pitchover_angle)
        if config is not None
        else float(C.PITCHOVER_ANGLE)
    )
    pitchover_angle_cfg = float(np.clip(pitchover_angle_cfg, 0.0, np.radians(30.0)))
    min_velocity_for_turn = (
        float(config.min_velocity_for_turn)
        if config is not None
        else C.MIN_VELOCITY_FOR_TURN
    )
    wind_offset = float(config.runtime_wind_offset_mps) if config is not None else 0.0

                                               
    gamma_target = gamma_profile_from_altitude(altitude, config=config)
    gamma_target_deg = float(np.degrees(gamma_target))

                                     
    v_rel = compute_relative_velocity(r, v, wind_offset_mps=wind_offset)
    v_rel_norm = float(vec_norm(v_rel))

    v_vert = float(np.dot(v_rel, vertical))
    v_horiz_vec = v_rel - v_vert * vertical
    v_horiz = float(vec_norm(v_horiz_vec))

    if v_rel_norm < 1e-3:
        gamma_meas_deg = 90.0
    else:
        gamma_meas_deg = float(np.degrees(np.arctan2(v_vert, v_horiz)))

    error = gamma_target_deg - gamma_meas_deg
    gamma_rate = (gamma_meas_deg - gs.prev_gamma_meas) / max(dt, 1e-3)

               
    kp, ki, kd = 0.85, 0.05, 0.12
    gs.gamma_int += error * dt
    gs.gamma_int = float(np.clip(gs.gamma_int, -20.0, 20.0))
    gamma_raw = gamma_target_deg + kp * error + ki * gs.gamma_int - kd * gamma_rate

                                                              
    _, _, rho, _ = compute_configured_atmosphere_properties(altitude, config)
    q_dyn = 0.5 * rho * v_rel_norm**2

    if v_rel_norm > 1.0:
        prograde = v_rel / v_rel_norm
    else:
        prograde = vertical

    gamma_cmd_clamped = float(np.clip(gamma_raw, 10.0, 90.0))

    pitchover_active = pitchover_start_alt <= altitude <= pitchover_end_alt
    if pitchover_active:
        ramp_span = max(
            min(C.PITCHOVER_RAMP_DISTANCE, pitchover_end_alt - pitchover_start_alt),
            1.0,
        )
        x = float(np.clip(
            (altitude - pitchover_start_alt) / ramp_span,
            0.0,
            1.0,
        ))
        ramp = x * x * (3.0 - 2.0 * x)
        min_pitch_deg = np.degrees(pitchover_angle_cfg) * ramp
        gamma_cmd_clamped = min(gamma_cmd_clamped, 90.0 - min_pitch_deg)

    pitch_cmd_rad = np.radians(90.0 - gamma_cmd_clamped)

    if v_horiz > 1.0:
        horiz_axis = v_horiz_vec / v_horiz
    else:
        horiz_axis = east

    guidance_dir = np.cos(pitch_cmd_rad) * vertical + np.sin(pitch_cmd_rad) * horiz_axis
    guidance_dir /= vec_norm(guidance_dir)

                                                                              
                                                                          
                                        
    if pitchover_active:
        pitchover_axis = (
            np.cos(C.PITCHOVER_AZIMUTH) * north
            + np.sin(C.PITCHOVER_AZIMUTH) * east
        )
        pitchover_axis /= vec_norm(pitchover_axis)
                                                                             
                                                                              
                                                                               
                                                                        
        kick_peak_alt = min(pitchover_start_alt + 500.0, pitchover_end_alt)
        kick_end_alt = min(pitchover_end_alt, pitchover_start_alt + 1000.0)
        kick_peak_angle = pitchover_angle_cfg

        if altitude <= kick_peak_alt:
            pitchover_scale = (
                (altitude - pitchover_start_alt)
                / max(kick_peak_alt - pitchover_start_alt, 1.0)
            )
        else:
            pitchover_scale = 1.0 - (
                (altitude - kick_peak_alt)
                / max(kick_end_alt - kick_peak_alt, 1.0)
            )
        pitchover_scale = float(np.clip(pitchover_scale, 0.0, 1.0))

        if pitchover_scale > 0.0:
            pitchover_angle = kick_peak_angle * pitchover_scale
            pitchover_dir = (
                np.cos(pitchover_angle) * vertical
                + np.sin(pitchover_angle) * pitchover_axis
            )
            pitchover_dir /= vec_norm(pitchover_dir)
            guidance_dir = pitchover_dir

                                                                       
                                                                         
                                                                          
                                                                          
                                                                        
                                                     
    blend_alpha = compute_blend_parameter(
        altitude,
        velocity_mag=v_rel_norm,
        config=config,
    )
    if altitude < pitchover_end_alt:
        w_prograde = 0.0
    else:
        w_prograde = blend_alpha

    if v_rel_norm < min_velocity_for_turn:
        w_prograde = 0.0

    if q_dyn > 20000.0 and altitude >= pitchover_end_alt and v_vert > 0.0:
         w_prograde = 1.0

    thrust_dir_mixed = (1.0 - w_prograde) * guidance_dir + w_prograde * prograde

    norm_mixed = vec_norm(thrust_dir_mixed)
    if norm_mixed > 1e-6:
        thrust_dir_mixed /= norm_mixed
    else:
        thrust_dir_mixed = guidance_dir

    thrust_dir = thrust_dir_mixed

                     
    if q_dyn > 5000.0:
        max_aoa = np.radians(3.0)
    elif altitude < 50000.0:
        max_aoa = np.radians(5.0)
    else:
        max_aoa = np.radians(10.0)

    if altitude >= pitchover_end_alt and v_vert > 0.0:
        thrust_dir = _limit_aoa(thrust_dir, v_rel, max_aoa)

    cos_p = np.dot(thrust_dir, vertical)
    gamma_cmd_clamped = np.degrees(np.arcsin(np.clip(cos_p, -1.0, 1.0)))

    gs.prev_gamma_meas = gamma_meas_deg

    return thrust_dir, gamma_cmd_clamped, gamma_meas_deg, gs


def compute_guidance_output(
    r: np.ndarray, v: np.ndarray, t: float, m: float,
    meco_mass: float = None,
    gs: GuidanceState | None = None,
    dt: float = C.DT,
    config: SimulationConfig | None = None,
) -> tuple[GuidanceOutput, GuidanceState]:
    """
    Compute ascent guidance output for Stage 1 powered flight.

    Args:
        r: Position vector (ECI, m)
        v: Velocity vector (ECI, m/s)
        t: Current time (s)
        m: Current vehicle mass (kg)
        meco_mass: Mass at which MECO occurs.
        gs: Per-vehicle guidance state. Uses module default if None.

    Returns:
        (guidance_output_dict, updated_guidance_state)
    """
    gs = _resolve_guidance_state(gs)
    if meco_mass is None:
        meco_mass = (
            config.meco_mass_kg
            if config is not None
            else C.DRY_MASS + C.STAGE1_LANDING_FUEL_RESERVE
        )

    altitude = float(vec_norm(r) - C.R_EARTH)
    pitchover_start_alt = (
        float(config.pitchover_start_altitude)
        if config is not None
        else C.PITCHOVER_START_ALTITUDE
    )
    pitchover_end_alt = (
        float(config.pitchover_end_altitude)
        if config is not None
        else C.PITCHOVER_END_ALTITUDE
    )
    wind_offset = float(config.runtime_wind_offset_mps) if config is not None else 0.0
    v_rel = compute_relative_velocity(r, v, wind_offset_mps=wind_offset)
    v_rel_norm = float(vec_norm(v_rel))
    thrust_dir, gamma_cmd, gamma_meas, gs = compute_desired_thrust_direction(
        r,
        v,
        t,
        gs,
        dt=dt,
        config=config,
    )

    alpha = compute_blend_parameter(altitude, velocity_mag=v_rel_norm, config=config)
    if alpha < 0.01:
        if pitchover_start_alt <= altitude <= pitchover_end_alt:
            phase = "PITCHOVER"
        else:
            phase = "VERTICAL_ASCENT"
    elif alpha < 0.99:
        phase = "GRAVITY_TURN"
    else:
        phase = "PROGRADE"

    thrust_on = (m > meco_mass)

                                                                              
                                                                                
    _, _, rho, _ = compute_configured_atmosphere_properties(altitude, config)
    q_dyn = 0.5 * rho * (v_rel_norm ** 2)
    q_rate = (
        (q_dyn - gs.prev_dynamic_pressure_pa) / max(float(dt), 1e-3)
        if gs.prev_dynamic_pressure_pa > 0.0
        else 0.0
    )
    throttle, predicted_q = compute_max_q_throttle(q_dyn, q_rate, config)
    gs.prev_dynamic_pressure_pa = q_dyn

    if v_rel_norm < C.ZERO_TOLERANCE:
        prograde = compute_local_vertical(r)
    else:
        prograde = v_rel / v_rel_norm

                                                                             
    gs.last_ascent_direction = thrust_dir.copy()

    vertical = compute_local_vertical(r)
    cos_pitch = np.clip(np.dot(vertical, thrust_dir), -1.0, 1.0)
    pitch_angle = float(np.arccos(cos_pitch))
    gamma_angle = np.radians(gamma_cmd)

    output = {
        'thrust_direction': thrust_dir,
        'phase': phase,
        'thrust_on': thrust_on,
        'pitch_angle': pitch_angle,
        'gamma_angle': gamma_angle,
        'gamma_command_deg': gamma_cmd,
        'gamma_measured_deg': gamma_meas,
        'velocity_tilt_deg': gamma_meas,
        'blend_alpha': alpha,
        'altitude': altitude,
        'velocity': float(vec_norm(v)),
        'local_vertical': vertical,
        'local_horizontal': compute_local_horizontal(r, v),
        'prograde': prograde,
        'throttle': throttle,
        'max_q_predicted_pa': predicted_q,
        'dynamic_pressure_rate_pa_s': q_rate,
        'v_rel': v_rel,
        'v_rel_mag': v_rel_norm
    }
    return output, gs

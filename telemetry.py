from typing import Any

import numpy as np

from rlv_sim import constants as C
from rlv_sim._guidance_common import compute_local_vertical
from rlv_sim._mission_manager_helpers import compute_orbit_metrics
from rlv_sim._run_full_mission import MissionProgress
from rlv_sim.config_definition import SimulationConfig
from rlv_sim.forces import (
    compute_aerodynamic_heating,
    compute_configured_atmosphere_properties,
    compute_dynamic_pressure,
)
from rlv_sim.recovery import (
    great_circle_distance_m,
    rotating_launch_site_eci,
    target_landing_site_eci,
)
from rlv_sim.utils import surface_relative_speed

from .utils import vec_norm


def extract_telemetry_point(state: Any, guidance: dict, phase_name: str, config: SimulationConfig) -> dict:
    """
    Convert simulation state and guidance outputs into a formatted telemetry dictionary
    for transmission to the frontend or logging.
    """
    vertical = compute_local_vertical(state.r)
    v_rel = np.asarray(guidance.get('v_rel', state.v))
    v_vert = float(np.dot(v_rel, vertical))
    
    alt_m = state.altitude
    if config.enable_atmosphere:
        _, _, rho_atm, _ = compute_configured_atmosphere_properties(alt_m, config)
    else:
        rho_atm = 0.0
    v_rel_mag = float(vec_norm(v_rel))
    
    q_dyn = compute_dynamic_pressure(rho_atm, v_rel_mag)
    r_nose = C.REFERENCE_DIAMETER / 2.0
    q_heat = compute_aerodynamic_heating(rho_atm, v_rel_mag, r_nose)
    
    pitch_angle_rad = guidance.get('pitch_angle', 0.0)
    orbit = compute_orbit_metrics(state)
    launch_site = rotating_launch_site_eci(state.t, config=config)
    
    return {
        "time": float(state.t),
        "altitude": float(state.altitude),
        "velocity": float(surface_relative_speed(state, config)),
        "mass": float(state.m),
        "downrange": float(great_circle_distance_m(state.r, launch_site)),
        "heat_flux": float(q_heat / 1000.0),                                        
        "dynamic_pressure": float(q_dyn),
        "throttle": float(guidance.get('throttle', 0.0)),
        "pitch_angle": float(np.degrees(pitch_angle_rad)),
        "phase": phase_name,
        "perigee_altitude": float(orbit["perigee_alt"]),
        "apogee_altitude": float(orbit["apogee_alt"]),
        "eccentricity": float(orbit["ecc"]),
        "orbital_velocity_deficit": float(orbit["v_deficit"]),
                                          
        "actual_quat_w": float(state.q[0]),
        "actual_quat_x": float(state.q[1]),
        "actual_quat_y": float(state.q[2]),
        "actual_quat_z": float(state.q[3]),
    }


def extract_mission_progress(progress: MissionProgress, config: SimulationConfig) -> dict:
    """Convert a canonical runtime progress update into web telemetry."""

    payload = extract_telemetry_point(
        progress.primary_state,
        progress.primary_guidance,
        progress.phase,
        config,
    )
    if progress.vehicle != "dual" or progress.booster_state is None:
        return payload

    booster_state = progress.booster_state
    booster_guidance = progress.booster_guidance or {}
    landing_site = target_landing_site_eci(
        booster_state.t,
        config.booster_landing_target_downrange_km,
        config=config,
    )
    booster_hat = booster_state.r / float(vec_norm(booster_state.r))
    landing_hat = landing_site / float(vec_norm(landing_site))
    central_angle = float(np.arccos(np.clip(np.dot(booster_hat, landing_hat), -1.0, 1.0)))
    landing_error_m = float(C.R_EARTH * central_angle)
    fuel_remaining = max(0.0, booster_state.m - float(config.stage1_dry_mass))
    reserve = float(config.stage1_landing_fuel_reserve_kg)
    booster_phase = str(booster_guidance.get("phase", "UNKNOWN"))

                                                                           
    launch_site = rotating_launch_site_eci(booster_state.t, config=config)
    booster_downrange_m = float(great_circle_distance_m(booster_state.r, launch_site))

    payload.update({
        "time": float(max(progress.primary_state.t, booster_state.t)),
        "booster_altitude": float(booster_state.altitude),
        "booster_velocity": float(surface_relative_speed(booster_state, config)),
        "booster_mass": float(booster_state.m),
        "booster_downrange": booster_downrange_m,
        "booster_phase": booster_phase,
        "booster_throttle": float(booster_guidance.get("throttle", 0.0)),
        "booster_pitch_angle": float(np.degrees(booster_guidance.get("pitch_angle", 0.0))),
        "booster_fuel_remaining": float((fuel_remaining / reserve) * 100.0 if reserve > 0 else 0.0),
        "booster_grid_fins": (
            "DEPLOYED"
            if config.enable_grid_fins and booster_phase in ("BOOSTER_ENTRY", "BOOSTER_LANDING")
            else "STOWED"
        ),
        "booster_landing_legs": (
            "DEPLOYED"
            if config.enable_landing_legs and booster_phase == "BOOSTER_LANDING"
            else "STOWED"
        ),
        "booster_landing_error": landing_error_m,
    })
    return payload


__all__ = ["extract_mission_progress", "extract_telemetry_point"]

"""
RLV Phase-I Ascent Simulation - Utility Functions

This module contains shared utility functions used across multiple modules
to eliminate code duplication (DRY principle).
"""

import numpy as np

from . import constants as C


def _wind_vector(r: np.ndarray, wind_offset_mps: float = 0.0) -> np.ndarray:
    """Simple altitude-dependent wind in inertial frame (East/West)."""
    alt = np.linalg.norm(r) - C.R_EARTH
    if alt <= 0.0:
        return np.zeros(3)
                                                             
                                                               
    speed = C.WIND_REF_SPEED * (alt / C.WIND_REF_ALT) ** C.WIND_EXPONENT
    speed += float(wind_offset_mps)
    if alt < 5000.0:
                                                                         
        x = alt / 5000.0
        speed *= x * x * (3.0 - 2.0 * x)
                                                                         
    up = r / max(np.linalg.norm(r), 1e-9)
    k_axis = np.array([0.0, 0.0, 1.0])
    east = np.cross(k_axis, up)
    east_norm = np.linalg.norm(east)
    if east_norm < C.ZERO_TOLERANCE:
        east = np.array([0.0, 1.0, 0.0])
        east_norm = 1.0
    east = east / east_norm
    north = np.cross(up, east)
    north = north / max(np.linalg.norm(north), 1e-9)
    dir_vec = np.cos(C.WIND_DIRECTION_AZIMUTH) * north + np.sin(C.WIND_DIRECTION_AZIMUTH) * east
    return speed * dir_vec


def compute_relative_velocity(
    r: np.ndarray,
    v: np.ndarray,
    wind_offset_mps: float = 0.0,
) -> np.ndarray:
    """
    Compute air-relative velocity removing Earth rotation and winds.
    v_rel = v_inertial - (omega_earth × r) - v_wind
    """
    omega_earth = np.array([0.0, 0.0, C.EARTH_ROTATION_RATE])
    wind = _wind_vector(r, wind_offset_mps=wind_offset_mps)
    return v - np.cross(omega_earth, r) - wind


def compute_ground_relative_velocity(
    r: np.ndarray,
    v: np.ndarray,
) -> np.ndarray:
    """Compute velocity relative to the rotating ground, excluding wind.

    Ground-relative velocity is the correct navigation/contact quantity for
    pad targeting and touchdown. Air-relative velocity remains the correct
    quantity for aerodynamic force and attitude-to-flow calculations.
    """
    omega_earth = np.array([0.0, 0.0, C.EARTH_ROTATION_RATE])
    return np.asarray(v, dtype=float) - np.cross(
        omega_earth,
        np.asarray(r, dtype=float),
    )


def surface_relative_speed(state, config) -> float:
    """Magnitude of wind+Earth-rotation-corrected (air-relative) speed."""
    return float(np.linalg.norm(
        compute_relative_velocity(
            state.r, state.v,
            wind_offset_mps=getattr(config, "runtime_wind_offset_mps", 0.0),
        )
    ))


def axisymmetric_angle_of_attack(body_axis: np.ndarray, flow_vector: np.ndarray) -> float:
    """
    Return lateral incidence for an axisymmetric vehicle.

    Nose-first and tail-first axial flow both have zero lateral angle of
    attack; broadside flow remains 90 degrees.
    """
    axis = np.asarray(body_axis, dtype=float)
    flow = np.asarray(flow_vector, dtype=float)
    axis_norm = float(np.linalg.norm(axis))
    flow_norm = float(np.linalg.norm(flow))
    if axis_norm < C.ZERO_TOLERANCE or flow_norm < C.ZERO_TOLERANCE:
        return 0.0
    cos_axis = float(np.clip(np.dot(axis, flow) / (axis_norm * flow_norm), -1.0, 1.0))
    return float(np.arccos(abs(cos_axis)))

"""
RLV Phase-I Ascent Simulation - Utility Functions

This module contains shared utility functions used across multiple modules
to eliminate code duplication (DRY principle).
"""

import math

import numpy as np

from . import constants as C


def cross3(a, b) -> np.ndarray:
    """Cross product of two 3-vectors.

    Same arithmetic as ``np.cross`` (bit-identical results) without its
    generic N-d axis handling, which dominated simulation run time.
    """
    a0, a1, a2 = float(a[0]), float(a[1]), float(a[2])
    b0, b1, b2 = float(b[0]), float(b[1]), float(b[2])
    return np.array([a1 * b2 - a2 * b1, a2 * b0 - a0 * b2, a0 * b1 - a1 * b0])


def vec_norm(x) -> np.float64:
    """Euclidean norm of a 1-D vector; same result as ``np.linalg.norm(x)``."""
    try:
        return np.float64(math.sqrt(x.dot(x)))
    except (AttributeError, TypeError, ValueError):
        return np.linalg.norm(x)


_K_AXIS = np.array([0.0, 0.0, 1.0])
_OMEGA_EARTH = np.array([0.0, 0.0, C.EARTH_ROTATION_RATE])
_WIND_COS_AZ = np.cos(C.WIND_DIRECTION_AZIMUTH)
_WIND_SIN_AZ = np.sin(C.WIND_DIRECTION_AZIMUTH)


def _wind_vector(r: np.ndarray, wind_offset_mps: float = 0.0) -> np.ndarray:
    """Simple altitude-dependent wind in inertial frame (East/West)."""
    r_norm = vec_norm(r)
    alt = r_norm - C.R_EARTH
    if alt <= 0.0:
        return np.zeros(3)

    speed = C.WIND_REF_SPEED * (alt / C.WIND_REF_ALT) ** C.WIND_EXPONENT
    speed += float(wind_offset_mps)
    if alt < 5000.0:

        x = alt / 5000.0
        speed *= x * x * (3.0 - 2.0 * x)

    up = r / max(r_norm, 1e-9)
    east = cross3(_K_AXIS, up)
    east_norm = vec_norm(east)
    if east_norm < C.ZERO_TOLERANCE:
        east = np.array([0.0, 1.0, 0.0])
        east_norm = 1.0
    east = east / east_norm
    north = cross3(up, east)
    north = north / max(vec_norm(north), 1e-9)
    dir_vec = _WIND_COS_AZ * north + _WIND_SIN_AZ * east
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
    wind = _wind_vector(r, wind_offset_mps=wind_offset_mps)
    return v - cross3(_OMEGA_EARTH, r) - wind


def compute_ground_relative_velocity(
    r: np.ndarray,
    v: np.ndarray,
) -> np.ndarray:
    """Compute velocity relative to the rotating ground, excluding wind.

    Ground-relative velocity is the correct navigation/contact quantity for
    pad targeting and touchdown. Air-relative velocity remains the correct
    quantity for aerodynamic force and attitude-to-flow calculations.
    """
    return np.asarray(v, dtype=float) - cross3(_OMEGA_EARTH, r)


def surface_relative_speed(state, config) -> float:
    """Magnitude of wind+Earth-rotation-corrected (air-relative) speed."""
    return float(vec_norm(
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
    axis_norm = float(vec_norm(axis))
    flow_norm = float(vec_norm(flow))
    if axis_norm < C.ZERO_TOLERANCE or flow_norm < C.ZERO_TOLERANCE:
        return 0.0
    cos_axis = float(np.clip(np.dot(axis, flow) / (axis_norm * flow_norm), -1.0, 1.0))
    return float(np.arccos(abs(cos_axis)))

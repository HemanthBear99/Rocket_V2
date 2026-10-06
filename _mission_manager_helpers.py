"""Pure helper functions used by the mission manager state machine."""

from __future__ import annotations

import numpy as np

from . import constants as C
from .frames import quaternion_to_rotation_matrix
from .recovery import estimate_suicide_burn
from .state import State


def compute_radial_velocity(state: State) -> float:
    """Compute radial velocity: r_dot = (r . v) / |r|."""
    r_norm = np.linalg.norm(state.r)
    if r_norm > 1.0:
        return float(np.dot(state.r, state.v) / r_norm)
    return 0.0


def compute_horizontal_velocity(state: State) -> float:
    """Compute horizontal speed perpendicular to the local radial direction."""
    r_hat = state.r / max(np.linalg.norm(state.r), 1.0)
    v_radial = np.dot(state.v, r_hat) * r_hat
    v_horiz = state.v - v_radial
    return float(np.linalg.norm(v_horiz))


def check_attitude_aligned(
    state: State,
    target_dir: np.ndarray,
    threshold_deg: float = 15.0,
) -> bool:
    """Check whether body +Z is aligned with the target direction."""
    rotation = quaternion_to_rotation_matrix(state.q)
    body_z_inertial = rotation[:, 2]
    target_norm = np.linalg.norm(target_dir)
    if target_norm < 1e-6:
        return False
    target_hat = target_dir / target_norm
    cos_angle = np.clip(np.dot(body_z_inertial, target_hat), -1.0, 1.0)
    angle_deg = np.degrees(np.arccos(cos_angle))
    return angle_deg < threshold_deg


def compute_downrange_distance(state: State, launch_site: np.ndarray) -> float:
    """Compute great-circle distance from the launch site."""
    r_hat = state.r / max(np.linalg.norm(state.r), 1.0)
    r0_hat = launch_site / max(np.linalg.norm(launch_site), 1.0)
    cos_angle = np.clip(np.dot(r_hat, r0_hat), -1.0, 1.0)
    return float(C.R_EARTH * np.arccos(cos_angle))


def compute_suicide_burn_altitude(state: State, safety_factor: float) -> float:
    """Compute the shared recovery-based suicide burn ignition altitude."""
    burn = estimate_suicide_burn(
        state.r,
        state.v,
        state.m,
        C.LANDING_THRUST,
        safety_factor=safety_factor,
    )
    return float(burn["burn_altitude"])


def compute_orbit_metrics(state: State) -> dict:
    """Return derived orbital metrics for phase transition checks."""
    r_mag = np.linalg.norm(state.r)
    v_mag = np.linalg.norm(state.v)
    energy = 0.5 * v_mag ** 2 - C.MU_EARTH / r_mag

    if abs(energy) > 1.0:
        a_sma = -C.MU_EARTH / (2.0 * energy)
    else:
        a_sma = r_mag

    h_vec = np.cross(state.r, state.v)
    h_mag = np.linalg.norm(h_vec)
    if a_sma > 0 and C.MU_EARTH * a_sma > 0:
        ecc_sq = max(0.0, 1.0 - h_mag ** 2 / (C.MU_EARTH * a_sma))
        ecc = float(np.sqrt(ecc_sq))
    else:
        ecc = 1.0

    perigee_alt = a_sma * (1.0 - ecc) - C.R_EARTH if a_sma > 0 else -C.R_EARTH
    apogee_alt = a_sma * (1.0 + ecc) - C.R_EARTH if a_sma > 0 else -C.R_EARTH
    v_circular = np.sqrt(C.MU_EARTH / r_mag)
    v_deficit = v_circular - v_mag
    s2_prop_remaining = state.m - state.dry_mass_kg

    return {
        "r_mag": float(r_mag),
        "v_mag": float(v_mag),
        "energy": float(energy),
        "a_sma": float(a_sma),
        "ecc": float(ecc),
        "perigee_alt": float(perigee_alt),
        "apogee_alt": float(apogee_alt),
        "v_circular": float(v_circular),
        "v_deficit": float(v_deficit),
        "s2_prop_remaining": float(s2_prop_remaining),
        "s2_prop_exhausted": bool(s2_prop_remaining <= 0.0),
    }

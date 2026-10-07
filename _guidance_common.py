"""
Guidance module implementing altitude-based gamma profile with PID tracking
and thrust-direction generation for gravity turn.

All mutable guidance state is encapsulated in GuidanceState to support
concurrent multi-vehicle simulations.
"""

from dataclasses import dataclass

import numpy as np

from . import constants as C
from .config_definition import SimulationConfig
from .utils import compute_relative_velocity, cross3, vec_norm


@dataclass
class GuidanceState:
    """Per-vehicle guidance state for PID tracking and orbit insertion.

    Each vehicle (stacked, orbiter, booster) gets its own instance so that
    guidance can run independently in parallel simulations.
    """

    prev_gamma_meas: float = 90.0
    gamma_int: float = 0.0
    prev_dynamic_pressure_pa: float = 0.0


    oi_start_time: float | None = None
    oi_start_direction: np.ndarray | None = None
    last_ascent_direction: np.ndarray | None = None
    prev_cmd_direction: np.ndarray | None = None
    coast_start_time: float | None = None
    orbit_circularization_active: bool = False
    orbit_circularization_start_time: float | None = None


    apogee_raise_complete: bool = False
    booster_landing_burn_started: bool = False
    deorbit_start_mass: float | None = None
    landing_leg_state: object | None = None
    navigation_state: object | None = None


    rcs_state: object | None = None
    control_state: object | None = None


def create_guidance_state() -> GuidanceState:
    """Create a fresh GuidanceState. Replaces the old reset_guidance()."""
    return GuidanceState()


def _resolve_guidance_state(gs: GuidanceState | None) -> GuidanceState:
    """Return caller state or a fresh one-shot state for stateless use.

    The live simulation always passes an explicit GuidanceState. When helper
    functions are called ad hoc without one, using a fresh state avoids hidden
    cross-call coupling through a module-global default.
    """
    if gs is None:
        return create_guidance_state()
    return gs


def reset_guidance() -> GuidanceState:
    """Return a fresh run-local guidance state for compatibility."""
    return create_guidance_state()


def _limit_aoa(thrust_dir: np.ndarray, velocity: np.ndarray,
               max_aoa_rad: float) -> np.ndarray:
    """
    Limit angle-of-attack: cap the angle between thrust direction and velocity.

    If the angle between thrust_dir and velocity exceeds max_aoa_rad,
    rotate thrust_dir toward velocity until the angle equals max_aoa_rad.

    This prevents structural loads (high Q·alpha) during atmospheric flight
    and keeps attitude error bounded during phase transitions.

    Args:
        thrust_dir: Desired thrust direction (unit vector)
        velocity: Velocity vector (does not need to be unit)
        max_aoa_rad: Maximum allowed angle-of-attack (radians)

    Returns:
        AoA-limited thrust direction (unit vector)
    """
    v_norm = vec_norm(velocity)
    if v_norm < 50.0:
        return thrust_dir

    v_hat = velocity / v_norm
    cos_aoa = np.clip(np.dot(thrust_dir, v_hat), -1.0, 1.0)
    aoa = np.arccos(cos_aoa)

    if aoa <= max_aoa_rad:
        return thrust_dir


    axis = cross3(v_hat, thrust_dir)
    axis_norm = vec_norm(axis)
    if axis_norm < 1e-9:
        return thrust_dir

    axis = axis / axis_norm

    limited = (v_hat * np.cos(max_aoa_rad)
               + cross3(axis, v_hat) * np.sin(max_aoa_rad)
               + axis * np.dot(axis, v_hat) * (1.0 - np.cos(max_aoa_rad)))
    limited_norm = vec_norm(limited)
    if limited_norm > 1e-9:
        limited /= limited_norm
    return limited


def compute_local_vertical(r: np.ndarray) -> np.ndarray:
    r_norm = vec_norm(r)
    if r_norm < C.ZERO_TOLERANCE:
        return np.array([1.0, 0.0, 0.0])
    return r / r_norm


def compute_local_frame(r: np.ndarray):
    vertical = compute_local_vertical(r)
    k_axis = np.array([0.0, 0.0, 1.0])
    east = cross3(k_axis, vertical)
    if vec_norm(east) < C.ZERO_TOLERANCE:
        east = np.array([0.0, 1.0, 0.0])
    east = east / vec_norm(east)
    north = cross3(vertical, east)
    north = north / vec_norm(north)
    return vertical, east, north


def compute_local_horizontal(r: np.ndarray, v: np.ndarray) -> np.ndarray:
    vertical = compute_local_vertical(r)
    v_rel = compute_relative_velocity(r, v)
    v_rel_norm = float(vec_norm(v_rel))
    if v_rel_norm < C.ZERO_TOLERANCE:
        return vertical
    return v_rel / v_rel_norm


def gamma_profile_from_altitude(
    altitude: float,
    config: SimulationConfig | None = None,
) -> float:
    """
    Continuous, smooth gamma profile (gravity turn) based on altitude.

    Uses a tanh-based shaping function to ensure C1 continuity (smooth derivatives)
    which helps the Attitude Control System track the command without saturating.

    Gamma starts at 90 deg (Vertical) and smoothly decays toward the target
    pitch angle at MECO.  The scale height controls how aggressively the
    vehicle pitches over during the ascent.  A faster turn (lower scale)
    produces a higher horizontal velocity at MECO, which is needed to reach
    the typical 45-60 deg pitch-from-vertical at engine cutoff.
    """
    gamma_start = 90.0


    gamma_final = 45.0

    turn_start_alt = (
        float(config.gravity_turn_start_altitude)
        if config is not None
        else 500.0
    )
    transition_range = (
        float(config.gravity_turn_transition_range)
        if config is not None
        else C.GRAVITY_TURN_TRANSITION_RANGE
    )


    turn_scale = max(1.4 * transition_range, 1.0)

    if altitude < turn_start_alt:
        return np.radians(gamma_start)

    h_norm = (altitude - turn_start_alt) / turn_scale
    progress = np.tanh(h_norm)
    gamma_deg = gamma_start + (gamma_final - gamma_start) * progress

    return np.radians(gamma_deg)


def compute_blend_parameter(
    altitude: float,
    velocity_mag: float | None = None,
    config: SimulationConfig | None = None,
) -> float:
    """
    Blend factor (0 → vertical ascent, 1 → prograde) based on altitude.

    A smooth ramp is used so guidance phase labels and thrust blending
    remain continuous.
    """
    start = (
        float(config.gravity_turn_start_altitude)
        if config is not None
        else C.GRAVITY_TURN_START_ALTITUDE
    )
    width = (
        float(config.gravity_turn_transition_range)
        if config is not None
        else C.GRAVITY_TURN_TRANSITION_RANGE
    )
    min_velocity_for_turn = (
        float(config.min_velocity_for_turn)
        if config is not None
        else C.MIN_VELOCITY_FOR_TURN
    )

    if velocity_mag is not None and velocity_mag < min_velocity_for_turn:
        return 0.0

    if altitude <= start:
        return 0.0
    if altitude >= start + width:
        return 1.0

    x = (altitude - start) / max(width, 1.0)
    return x * x * (3.0 - 2.0 * x)



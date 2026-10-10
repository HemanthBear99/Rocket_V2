"""
RLV Phase-I Ascent Simulation - Validation Checks

This module implements physics validation checks:
- Finite-state check (no NaN/inf in r, v, q, omega, m, t)
- Quaternion norm check
- Position/mass/velocity/angular-velocity bounds checks
- Energy conservation (approximate, diagnostic only -- non-fatal by default)

Previously this docstring also claimed "zero force -> constant velocity"
and "zero torque -> constant omega" checks, but no such functions were ever
implemented here; those specific invariants are instead exercised as
integration tests against the RK4 integrator directly (see
tests/test_validation_tier1.py's Keplerian energy/angular-momentum
conservation cases), not as a standalone check in this module.

Abort on violation (validate_state's default abort_on_error=True).
"""


import numpy as np

from . import constants as C
from .state import State
from .utils import vec_norm


class ValidationError(Exception):
    """Raised when a state validation check fails."""


def check_finite_state(state: State) -> bool:
    """Reject NaN/inf state values before norm and bounds checks run."""
    fields = {
        "r": state.r,
        "v": state.v,
        "q": state.q,
        "omega": state.omega,
        "m": state.m,
        "t": state.t,
    }
    for name, value in fields.items():
        if not np.all(np.isfinite(value)):
            raise ValidationError(f"State contains non-finite {name}: {value}")
    return True


def check_quaternion_norm(q: np.ndarray, tolerance: float | None = None) -> bool:
    """
    Verify quaternion is unit-normalized.

    Args:
        q: Quaternion [w, x, y, z]
        tolerance: Allowable deviation from 1.0

    Returns:
        True if valid, raises ValidationError otherwise
    """
    if tolerance is None:
        tolerance = C.QUATERNION_NORM_TOL

    norm = vec_norm(q)
    if abs(norm - 1.0) > tolerance:
        raise ValidationError(
            f"Quaternion norm violation: |q| = {norm:.10f}, "
            f"deviation = {abs(norm - 1.0):.2e}, tolerance = {tolerance:.2e}"
        )
    return True


def check_position_valid(r: np.ndarray) -> bool:
    """
    Check that position is physically valid (above Earth's center).

    Args:
        r: Position vector (m)

    Returns:
        True if valid, raises ValidationError otherwise
    """
    r_norm = vec_norm(r)

    if r_norm < C.R_EARTH * 0.5:
        raise ValidationError(
            f"Position inside Earth: |r| = {r_norm/1000:.2f} km, "
            f"R_Earth = {C.R_EARTH/1000:.2f} km"
        )
    return True


def check_mass_valid(
    m: float,
    dry_mass: float = C.DRY_MASS,
    max_mass: float | None = None,
) -> bool:
    """
    Check that mass is physically valid.

    Args:
        m: Vehicle mass (kg)
        dry_mass: Dry mass limit (kg)

    Returns:
        True if valid, raises ValidationError otherwise
    """
    if m < dry_mass * 0.999:
        raise ValidationError(
            f"Mass below dry mass: m = {m:.2f} kg, "
            f"dry mass = {dry_mass:.2f} kg"
        )

    if max_mass is None:
        max_mass = C.INITIAL_MASS * 1.01

    if m > max_mass:
        raise ValidationError(
            f"Mass exceeds initial mass: m = {m:.2f} kg, "
            f"max = {max_mass:.2f} kg"
        )
    return True


def check_velocity_reasonable(v: np.ndarray) -> bool:
    """
    Check that velocity is within reasonable bounds.

    Args:
        v: Velocity vector (m/s)

    Returns:
        True if valid, raises ValidationError otherwise
    """
    v_mag = vec_norm(v)

    max_reasonable_v = 15000.0

    if v_mag > max_reasonable_v:
        raise ValidationError(
            f"Velocity exceeds reasonable bounds: |v| = {v_mag:.2f} m/s, "
            f"max = {max_reasonable_v:.2f} m/s"
        )
    return True


def check_angular_velocity_reasonable(omega: np.ndarray) -> bool:
    """
    Check that angular velocity is within reasonable bounds.

    Args:
        omega: Angular velocity (rad/s)

    Returns:
        True if valid, raises ValidationError otherwise
    """
    omega_mag = vec_norm(omega)

    max_reasonable_omega = 10.0

    if omega_mag > max_reasonable_omega:
        raise ValidationError(
            f"Angular velocity exceeds reasonable bounds: |ω| = {omega_mag:.4f} rad/s "
            f"({np.degrees(omega_mag):.2f} deg/s), max = {max_reasonable_omega:.2f} rad/s"
        )
    return True


def validate_state(
    state: State,
    abort_on_error: bool = True,
    dry_mass: float = C.DRY_MASS,
    max_mass: float | None = None,
    quaternion_norm_tolerance: float | None = None,
) -> tuple[bool, str | None]:
    """
    Perform all validation checks on a state.

    Args:
        state: State to validate
        abort_on_error: If True, raise exception on first error
        dry_mass: Dry mass lower bound
    """
    try:
        check_finite_state(state)
        check_quaternion_norm(state.q, tolerance=quaternion_norm_tolerance)
        check_position_valid(state.r)
        check_mass_valid(state.m, dry_mass, max_mass=max_mass)
        check_velocity_reasonable(state.v)
        check_angular_velocity_reasonable(state.omega)
        return True, None
    except ValidationError as e:
        if abort_on_error:
            raise
        return False, str(e)


def compute_total_energy(r: np.ndarray, v: np.ndarray, m: float) -> float:
    """
    Compute total mechanical energy (kinetic + potential).

    E_total = (1/2) * m * |v|² - (μ * m) / |r|

    Used to monitor integration accuracy during unpowered, low-drag coasts.

    Args:
        r: Position in ECI (m)
        v: Velocity in ECI (m/s)
        m: Vehicle mass (kg)

    Returns:
        Total mechanical energy (J)
    """
    r_norm = vec_norm(r)
    v_norm = vec_norm(v)

    if r_norm < C.ZERO_TOLERANCE:
        return 0.0
    if m < 0:
        return 0.0

    kinetic = 0.5 * m * (v_norm ** 2)
    potential = -C.MU_EARTH * m / r_norm

    return float(kinetic + potential)


def validate_energy_conservation(E_current: float, E_previous: float,
                                 dt: float) -> dict:
    """
    Validate energy conservation across a time step.

    Checks that the relative energy change between steps stays small, as a
    measure of integration quality during ballistic coasts.

    Args:
        E_current: Current total energy (J)
        E_previous: Previous total energy (J)
        dt: Time step (s)

    Returns:
        Dictionary with validation results
    """
    if abs(E_previous) < C.ZERO_TOLERANCE:
        return {'valid': True, 'error': 0.0, 'message': 'Energy too small to validate'}

    dE = E_current - E_previous
    relative_error = abs(dE / E_previous) if abs(E_previous) > C.ZERO_TOLERANCE else 0.0

    return {
        'valid': relative_error < C.ENERGY_TOLERANCE,
        'dE': float(dE),
        'relative_error': float(relative_error),
        'message': f"Energy error: {relative_error:.2e} (tol={C.ENERGY_TOLERANCE:.2e})"
    }

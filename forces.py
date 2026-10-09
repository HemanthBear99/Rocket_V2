"""
RLV Phase-I Ascent Simulation - Force Computations

This module implements all force calculations:
- Central gravity / spherical-harmonic geopotential (EGM96, full degree/order)
- Thrust (in body frame, transformed to inertial, altitude-compensated)
- Atmospheric drag (Mach-dependent Cd lookup table)
- Aerodynamic lift (slender-body CL-alpha model)
- Aerodynamic moments (CP/CG offset instability model)
References and model-validity notes:
    docs/MATHEMATICAL_REFERENCES.md#3-earth-gravity-and-coordinate-systems
    docs/MATHEMATICAL_REFERENCES.md#4-standard-atmosphere
    docs/MATHEMATICAL_REFERENCES.md#5-aerodynamic-forces-and-ascent-guidance
    docs/MATHEMATICAL_REFERENCES.md#6-propulsion-and-variable-mass
    docs/MATHEMATICAL_REFERENCES.md#7-aerothermal-heating

The force equation forms are standard. Mach tables, upper-atmosphere
extension, post-stall blending, engine corrections, and recovery aero
coefficients are engineering approximations rather than vehicle test data.
The EGM96 geopotential coefficients are the public NASA/NGA 1996 model
(EGM96, truncated to degree and order 6 here for performance -- see
_EGM96_MAX_DEG; the full model is complete to degree and order 360); the
J2-only legacy path is retained for backward compatibility.
"""

import functools
import logging

import numpy as np

from . import _numba_kernels
from . import constants as C
from ._types import ForceBreakdown
from .aero_database import AeroDatabase
from .frames import quaternion_to_rotation_matrix
from .high_fidelity_atmosphere import (
    compute_gram_atmosphere,
    eci_to_geodetic,
)
from .utils import (
    compute_ground_relative_velocity,
    compute_relative_velocity,
    cross3,
    vec_norm,
)

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# EGM96 spherical-harmonic geopotential coefficients (public domain, NASA/NGA).
#
# Fully-normalized tesseral coefficients C_nm, S_nm. This is a truncated
# complete expansion to low degree/order. The EGM96 reference defines the
# (n,m) = (2,0) term as the principal oblateness term; its normalized value
# is derived from the unnormalized J2 = 1.082626173e-3 via
#     C_20 = -J2 / sqrt(5)
# Higher-order coefficients are the published EGM96 normalized values
# (zeroed beyond the truncation). Source: NIMA/NASA EGM96 (complete to
# degree and order 360); here we embed a degree-6 truncation for speed,
# which captures oblateness, triaxiality, and the dominant gravity anomalies.
# ---------------------------------------------------------------------------
_EGM96_MAX_DEG = 6

# Fully-normalized EGM96 coefficients C_nm, S_nm. C_20 is derived from the
# canonical J2 = 1.082626173e-3:  C_20 = -J2 / sqrt(5).
_EGM96_C = {
    (2, 0): -0.484165143403758e-03,
    (2, 1): -0.186987654058389e-09,
    (2, 2): 0.243938357328313e-05,
    (3, 0): 0.957170110506111e-06,
    (3, 1): 0.203012444506208e-05,
    (3, 2): 0.904627519539148e-06,
    (3, 3): 0.721403509886931e-06,
    (4, 0): 0.539782597177617e-06,
    (4, 1): -0.536280500699696e-06,
    (4, 2): 0.350808224973665e-06,
    (4, 3): 0.237264769072511e-06,
    (4, 4): -0.156020388814889e-06,
    (5, 0): 0.686750604359933e-07,
    (5, 1): -0.589452687666875e-07,
    (5, 2): 0.654447476808239e-07,
    (5, 4): -0.584933124017353e-07,
    (5, 5): 0.102230282975025e-06,
    (6, 0): -0.437543275321895e-07,
    (6, 1): -0.385725121146447e-07,
    (6, 3): 0.295185648830200e-07,
    (6, 4): -0.528517761146810e-07,
    (6, 5): -0.397187127590200e-07,
    (6, 6): 0.162250284465700e-07,
}
_EGM96_S = {
    (2, 1): 0.121639158509667e-08,
    (2, 2): -0.140028336829753e-05,
    (3, 1): 0.248302905898171e-06,
    (3, 2): 0.750037150681267e-06,
    (3, 3): -0.365235018466910e-06,
    (4, 1): 0.105892194943372e-06,
    (4, 2): 0.303046866334966e-06,
    (4, 3): -0.553483595144639e-06,
    (4, 4): 0.328627661792302e-06,
    (5, 1): 0.539891095438227e-07,
    (5, 2): -0.568426229717463e-07,
    (5, 3): 0.0,
    (5, 4): 0.147124179078449e-06,
    (5, 5): -0.882741339242633e-07,
    (6, 1): 0.167219920800182e-07,
    (6, 2): -0.552304035528301e-07,
    (6, 3): 0.747165190261761e-07,
    (6, 4): -0.380205404950532e-07,
    (6, 5): -0.431940511798099e-07,
    (6, 6): 0.334651568695225e-07,
}

_AOA_POST_STALL_START_RAD = np.radians(20.0)
_AOA_POST_STALL_FULL_RAD = np.radians(45.0)

_US76_H = np.array([0.0, 11000.0, 20000.0, 32000.0, 47000.0, 51000.0, 71000.0, 84852.0])
_US76_L = np.array([-0.0065, 0.0, 0.0010, 0.0028, 0.0, -0.0028, -0.0020])


@functools.lru_cache(maxsize=1)
def _build_us76_tables():
    """Precompute layer-base temperatures and pressures for US-76.

    The tables are a pure function of module constants, so they are built once
    and cached. Using lru_cache instead of module-level mutable globals keeps
    the builder thread-safe and free of shared-state footguns. Returns
    (TB, PB): read-only arrays of layer-base temperatures (K) and pressures (Pa).
    """
    tb = [C.ATM_T0]
    pb = [C.ATM_P0]
    for i, lapse in enumerate(_US76_L):
        h0 = _US76_H[i]
        h1 = _US76_H[i + 1]
        T0 = tb[-1]
        P0 = pb[-1]
        dh = h1 - h0
        if abs(lapse) > 1e-12:
            T1 = T0 + lapse * dh
            exponent = -C.G0 / (lapse * C.R_GAS)
            P1 = P0 * (T1 / T0) ** exponent
        else:
            T1 = T0
            P1 = P0 * np.exp(-C.G0 * dh / (C.R_GAS * T0))
        tb.append(float(T1))
        pb.append(float(P1))

    us76_tb = np.array(tb)
    us76_pb = np.array(pb)


    us76_tb.flags.writeable = False
    us76_pb.flags.writeable = False
    return us76_tb, us76_pb


def compute_atmosphere_properties(altitude: float, enable_upper_atm: bool = True) -> tuple:
    """
    Compute atmospheric properties (Temperature, Pressure, Density, Speed of Sound).
    Based on US Standard Atmosphere 1976 (Troposphere & Stratosphere).

    Args:
        altitude: Geometric altitude above sea level (m)
        enable_upper_atm: If True, extend model above 84.852 km with exponential
            decay (thermosphere approximation). If False, return zero density above
            the US76 ceiling — strict US76 behaviour.

    Returns:
        (temperature, pressure, density, speed_of_sound)
        T in K, P in Pa, rho in kg/m^3, a in m/s
    """
    us76_tb, us76_pb = _build_us76_tables()
    h = max(0.0, float(altitude))

    if h <= _US76_H[-1]:
        idx = int(np.searchsorted(_US76_H, h, side='right') - 1)
        idx = max(0, min(idx, len(_US76_L) - 1))
        h0 = _US76_H[idx]
        lapse = _US76_L[idx]
        T0 = us76_tb[idx]
        P0 = us76_pb[idx]
        dh = h - h0

        if abs(lapse) > 1e-12:
            T = T0 + lapse * dh
            exponent = -C.G0 / (lapse * C.R_GAS)
            P = P0 * (T / T0) ** exponent
        else:
            T = T0
            P = P0 * np.exp(-C.G0 * dh / (C.R_GAS * T0))
    elif enable_upper_atm:


        h0 = _US76_H[-1]
        T0 = us76_tb[-1]
        P0 = us76_pb[-1]
        scale_height = 5000.0
        T = T0
        P = P0 * np.exp(-(h - h0) / scale_height)
    else:

        T = us76_tb[-1]
        P = 0.0

    rho = P / (C.R_GAS * T) if (T > 0.0 and P > 0.0) else 0.0
    if rho < C.DENSITY_FLOOR:
        rho = 0.0
    speed_of_sound = np.sqrt(C.GAMMA * C.R_GAS * T) if T > 0.0 else C.ATM_SPEED_OF_SOUND_FALLBACK
    return float(T), float(P), float(rho), float(speed_of_sound)


def compute_configured_atmosphere_properties(altitude: float, config=None, r: np.ndarray = None, t: float = 0.0) -> tuple:
    """Return atmosphere properties with runtime/GRAM-style scale factors applied."""
    enable_high_fidelity = bool(getattr(config, "enable_high_fidelity_gram", False)) if config is not None else False
    density_modifier = 1.0

    if enable_high_fidelity and r is not None:
        lat_deg, lon_deg, geodetic_alt = eci_to_geodetic(r, t)
        altitude = geodetic_alt
        density_modifier, _tropopause_base = compute_gram_atmosphere(lat_deg, lon_deg, geodetic_alt)
        # Note: We could dynamically adjust US-76 model boundaries here, but keeping it simple:
        # scale the computed density/pressure using the GRAM density modifier.

    enable_upper = bool(getattr(config, "enable_upper_atmosphere", False))
    temperature, pressure, density, speed_of_sound = compute_atmosphere_properties(
        altitude,
        enable_upper_atm=enable_upper,
    )
    if config is not None and not bool(getattr(config, "enable_atmosphere", True)):
        return temperature, 0.0, 0.0, speed_of_sound

    scale = float(getattr(config, "runtime_atmosphere_density_scale", 1.0)) * density_modifier
    pressure *= scale
    density *= scale
    if density < C.DENSITY_FLOOR:
        density = 0.0
    return temperature, pressure, density, speed_of_sound


def compute_dynamic_pressure(rho: float, v_rel_mag: float) -> float:
    """
    Compute dynamic pressure (q).

    Args:
        rho: Atmospheric density (kg/m^3)
        v_rel_mag: Relative velocity magnitude (m/s)

    Returns:
        Dynamic pressure (Pa)
    """
    return 0.5 * rho * v_rel_mag ** 2


def compute_post_stall_normal_coefficient(
    alpha_rad: float,
    linear_slope_per_rad: float,
    max_coefficient: float = 1.6,
) -> float:
    """Monotone slender-body to bounded cross-flow normal-force coefficient.

    Below the stall onset angle the linear slender-body law
    ``CN = slope * alpha`` applies unchanged. Past stall it rises smoothly and
    MONOTONICALLY to ``max_coefficient``, which it then holds to 90 deg.

    The previous formulation blended ``(1-b)*linear + b*max_coef*sin^2(alpha)``
    where the linear branch was unbounded. Because the blend weight saturated
    at 45 deg while the linear term kept growing, the result peaked near 25 deg
    and then FELL ~50% by 45 deg (measured: 1.594 -> 0.800 for slope 4,
    max 1.6) -- negative aerodynamic stiffness in precisely the deep-stall
    regime the blend exists to stabilize. Here the post-stall branch is
    anchored at the linear value reached at stall onset and rises to the same
    ceiling, so the whole curve is monotonically non-decreasing and still
    bounded, with no discontinuity at the stall angle.
    """
    alpha = float(np.clip(abs(alpha_rad), 0.0, 0.5 * np.pi))
    max_coef = max(float(max_coefficient), 0.0)
    slope = max(float(linear_slope_per_rad), 0.0)

    linear = slope * alpha
    if alpha <= _AOA_POST_STALL_START_RAD:
        return float(np.clip(linear, 0.0, max_coef))

    # Value the linear law reaches exactly at stall onset: the anchor point, so
    # the post-stall branch is continuous with the pre-stall branch.
    linear_at_stall = slope * _AOA_POST_STALL_START_RAD
    blend = float(np.clip(
        (alpha - _AOA_POST_STALL_START_RAD)
        / max(_AOA_POST_STALL_FULL_RAD - _AOA_POST_STALL_START_RAD, 1e-9),
        0.0,
        1.0,
    ))
    blend = blend * blend * (3.0 - 2.0 * blend)
    coefficient = linear_at_stall + (max_coef - linear_at_stall) * blend
    return float(np.clip(coefficient, 0.0, max_coef))


def compute_aerodynamic_heating(rho: float, v_rel_mag: float, nose_radius: float = 1.83) -> float:
    """
    Compute approximate stagnation point aerodynamic heat flux.
    Uses the Sutton-Graves approximation for convective heating.

    Args:
        rho: Atmospheric density (kg/m^3)
        v_rel_mag: Relative velocity magnitude (m/s)
        nose_radius: Radius of curvature at the stagnation point (m). Default is typical 3.66m diameter RLV nose.

    Returns:
        Heat flux (W/m^2)
    """
    if rho > 1e-12 and v_rel_mag > 100.0:

        return 1.7415e-4 * np.sqrt(rho / max(nose_radius, 0.01)) * v_rel_mag ** 3
    return 0.0


# Position-independent EGM96 tables: the non-zero (n, m, C_nm, S_nm) terms
# and the Legendre recurrence coefficients. Values are computed exactly as the per-call code used to.
_EGM96_TERMS_CACHE: dict = {"source": None, "terms": ()}


def _egm96_terms() -> tuple:
    """Non-zero (n, m, C_nm, S_nm) terms in summation order.

    Rebuilt whenever the coefficient dicts are replaced (tests patch them).
    """
    source = (_EGM96_C, _EGM96_S)
    cached = _EGM96_TERMS_CACHE["source"]
    if cached is None or cached[0] is not source[0] or cached[1] is not source[1]:
        _EGM96_TERMS_CACHE["terms"] = tuple(
            (n, m, _EGM96_C.get((n, m), 0.0), _EGM96_S.get((n, m), 0.0))
            for n in range(2, _EGM96_MAX_DEG + 1)
            for m in range(n + 1)
            if not (_EGM96_C.get((n, m), 0.0) == 0.0 and _EGM96_S.get((n, m), 0.0) == 0.0)
        )
        _EGM96_TERMS_CACHE["array"] = np.array(
            _EGM96_TERMS_CACHE["terms"], dtype=float
        ).reshape(-1, 4)
        _EGM96_TERMS_CACHE["source"] = source
    return _EGM96_TERMS_CACHE["terms"]


_EGM96_SECTORAL_FACTOR = {
    m: (np.sqrt(3.0) if m == 1 else np.sqrt((2.0 * m + 1.0) / (2.0 * m)))
    for m in range(1, _EGM96_MAX_DEG + 1)
}
_EGM96_SUBDIAG_FACTOR = {m: np.sqrt(2.0 * m + 3.0) for m in range(_EGM96_MAX_DEG)}
_EGM96_RECURRENCE = {
    (n, m): (
        np.sqrt((2.0 * n + 1.0) * (2.0 * n - 1.0) / ((n - m) * (n + m))),
        np.sqrt((2.0 * n + 1.0) * (n + m - 1.0) * (n - m - 1.0)
                / ((2.0 * n - 3.0) * (n - m) * (n + m))),
    )
    for m in range(_EGM96_MAX_DEG + 1)
    for n in range(m + 2, _EGM96_MAX_DEG + 1)
}


def _egm96_legendre(n_max: int, sin_phi: float, cos_phi: float):
    """Fully normalized P_nm(sin(latitude)) and latitude derivatives.

    No Condon-Shortley sign. Q stores P_nm / cos(latitude) for m > 0, avoiding
    division by zero at the poles.
    """
    P = {(0, 0): 1.0}
    dP = {(0, 0): 0.0}
    Q = {}
    for m in range(n_max + 1):
        if m > 0:
            factor = _EGM96_SECTORAL_FACTOR[m]
            previous = P[(m - 1, m - 1)]
            P[(m, m)] = factor * cos_phi * previous
            dP[(m, m)] = factor * (-sin_phi * previous + cos_phi * dP[(m - 1, m - 1)])
            Q[(m, m)] = factor * previous
        if m < n_max:
            factor = _EGM96_SUBDIAG_FACTOR[m]
            P[(m + 1, m)] = factor * sin_phi * P[(m, m)]
            dP[(m + 1, m)] = factor * (cos_phi * P[(m, m)] + sin_phi * dP[(m, m)])
            if m > 0:
                Q[(m + 1, m)] = factor * sin_phi * Q[(m, m)]
        for n in range(m + 2, n_max + 1):
            a_n, b_n = _EGM96_RECURRENCE[(n, m)]
            P[(n, m)] = a_n * sin_phi * P[(n - 1, m)] - b_n * P[(n - 2, m)]
            dP[(n, m)] = a_n * (cos_phi * P[(n - 1, m)] + sin_phi * dP[(n - 1, m)]) - b_n * dP[(n - 2, m)]
            if m > 0:
                Q[(n, m)] = a_n * sin_phi * Q[(n - 1, m)] - b_n * Q[(n - 2, m)]
    return P, dP, Q


# Array forms of the recurrence tables for the compiled kernel.
_EGM96_SECTORAL_ARR = np.array(
    [0.0] + [_EGM96_SECTORAL_FACTOR[m] for m in range(1, _EGM96_MAX_DEG + 1)]
)
_EGM96_SUBDIAG_ARR = np.array(
    [_EGM96_SUBDIAG_FACTOR[m] for m in range(_EGM96_MAX_DEG)] + [0.0]
)
_EGM96_REC_A = np.zeros((_EGM96_MAX_DEG + 1, _EGM96_MAX_DEG + 1))
_EGM96_REC_B = np.zeros((_EGM96_MAX_DEG + 1, _EGM96_MAX_DEG + 1))
for (_n, _m), (_a_n, _b_n) in _EGM96_RECURRENCE.items():
    _EGM96_REC_A[_n, _m] = _a_n
    _EGM96_REC_B[_n, _m] = _b_n


def compute_egm96_gravity_accel(r: np.ndarray, max_deg: int = _EGM96_MAX_DEG) -> np.ndarray:
    """Spherical-harmonic (EGM96 truncated) gravitational acceleration.

    Uses fully-normalized associated Legendre functions with the standard
    recurrence relations and the embedded EGM96 coefficient set. Position is
    geocentric (ECI). Acceleration returned in the same inertial frame.

    Args:
        r: Geocentric position vector (m)
        max_deg: Maximum degree/order of the expansion (<= _EGM96_MAX_DEG)

    Returns:
        Gravitational acceleration vector (m/s^2)
    """
    r_norm = vec_norm(r)
    if r_norm < C.ZERO_TOLERANCE:
        return np.zeros(3)

    mu = C.MU_EARTH
    a_e = C.R_EARTH_EQUATORIAL_WGS84
    if _numba_kernels.AVAILABLE:
        _egm96_terms()
        return _numba_kernels.egm96_accel(
            float(r[0]), float(r[1]), float(r[2]),
            min(int(max_deg), _EGM96_MAX_DEG), mu, a_e,
            _EGM96_TERMS_CACHE["array"], _EGM96_SECTORAL_ARR, _EGM96_SUBDIAG_ARR,
            _EGM96_REC_A, _EGM96_REC_B,
        )
    x, y, z = r
    # Spherical coordinates (geocentric).
    r_sph = r_norm
    sin_phi = z / r_sph                      # sin(latitude)
    cos_phi = np.sqrt(max(x * x + y * y, 0.0)) / r_sph
    lam = np.arctan2(y, x)                   # longitude

    sin_phi = float(np.clip(sin_phi, -1.0, 1.0))
    cos_phi = float(np.clip(cos_phi, 0.0, 1.0))
    sin_lam = np.sin(lam)
    cos_lam = np.cos(lam)

    n_max = min(int(max_deg), _EGM96_MAX_DEG)
    P, dP, Q = _egm96_legendre(n_max, sin_phi, cos_phi)

    # Accumulate acceleration via the standard Cartesian gradient of
    #   U = mu/r * sum_{n,m} (a_e/r)^n * P_nm(sin phi) * (C_nm cos m lam + S_nm sin m lam)
    # The gradient is most stable in the local spherical basis (r_hat, phi_hat,
    # lam_hat) and then mapped to ECI using the geocentric unit vectors:
    #   r_hat  = r / r
    #   phi_hat = (-sin phi cos lam, -sin phi sin lam, cos phi) (north)
    #   lam_hat = (-sin lam, cos lam, 0)                        (east)
    r_hat = np.array([cos_phi * cos_lam, cos_phi * sin_lam, sin_phi])
    lam_hat = np.array([-sin_lam, cos_lam, 0.0])

    ratio_n = {n: (a_e / r_sph) ** n for n in range(2, n_max + 1)}
    trig_m = {m: (np.cos(m * lam), np.sin(m * lam)) for m in range(1, n_max + 1)}
    dU_dr = 0.0
    dU_dphi = 0.0
    dU_dlam = 0.0
    for n, m, c, s in _egm96_terms():
        if n > n_max:
            continue
        p = P[(n, m)]
        dp = dP[(n, m)]
        ratio = ratio_n[n]
        if m == 0:
            cos_mlam = 1.0
            sin_mlam = 0.0
        else:
            cos_mlam, sin_mlam = trig_m[m]
        Y = c * cos_mlam + s * sin_mlam
        dU_dr += -(n + 1) * ratio * p * Y
        dU_dphi += ratio * dp * Y
        if m != 0:
            dU_dlam += ratio * Q[(n, m)] * m * (-c * sin_mlam + s * cos_mlam)

    # Gradient of the positive geopotential U. The longitude sum already
    # includes division by cos(latitude) through Q, including its polar limit.
    phi_hat = np.array([-sin_phi * cos_lam, -sin_phi * sin_lam, cos_phi])
    a = (mu / r_sph ** 2) * dU_dr * r_hat
    a += (mu / r_sph ** 2) * dU_dphi * phi_hat
    a += (mu / r_sph ** 2) * dU_dlam * lam_hat
    # The loop above only sums the disturbing potential (n >= 2); add the
    # missing central (two-body) term so the function returns total
    # gravitational acceleration, not just the perturbation.
    a += -(mu / r_sph ** 2) * r_hat
    return a


def compute_gravity_force(r: np.ndarray, m: float, enable_j2: bool = False,
                          j2: float = 1.08263e-3,
                          gravity_model: str = "central") -> np.ndarray:
    """
    Compute gravitational force.

    Supported gravity models:
      - "central":  F = -mu * m * r / ||r||^3
      - "j2":       central + J2 zonal oblateness correction (legacy)
      - "egm96":    full spherical-harmonic geopotential (EGM96 truncated)

    J2 perturbation:
        a_J2 = (3/2) * J2 * mu * R_E^2 / r^5 * [x*(5*z^2/r^2 - 1),
                                                    y*(5*z^2/r^2 - 1),
                                                    z*(5*z^2/r^2 - 3)]

    Args:
        r: Position vector (ECI, m)
        m: Vehicle mass (kg)
        enable_j2: If True and gravity_model=="central", include J2 correction
        j2: J2 coefficient (default Earth J2 = 1.08263e-3)
        gravity_model: One of "central", "j2", "egm96"

    Returns:
        Gravitational force vector (N) in ECI frame
    """
    r_norm = vec_norm(r)
    if r_norm < C.ZERO_TOLERANCE:
        return np.zeros(3)

    F_central = -C.MU_EARTH * m * r / (r_norm ** 3)

    model = (gravity_model or "central").lower()
    if model == "central":
        if not enable_j2:
            return F_central
        model = "j2"

    if model == "j2":
        x, y, z = r
        r2 = r_norm ** 2
        r5 = r_norm ** 5
        factor = 1.5 * j2 * C.MU_EARTH * C.R_EARTH_EQUATORIAL_WGS84 ** 2 / r5
        z2_over_r2 = z ** 2 / r2
        a_j2 = factor * np.array([
            x * (5.0 * z2_over_r2 - 1.0),
            y * (5.0 * z2_over_r2 - 1.0),
            z * (5.0 * z2_over_r2 - 3.0)
        ])
        return F_central + m * a_j2

    if model == "egm96":
        return m * compute_egm96_gravity_accel(r)

    # Unknown model: fall back to central.
    return F_central


def apply_engine_transient(throttle_cmd: float, throttle_prev: float, dt: float,
                           spool_up_time: float = 1.5,
                           spool_down_time: float = 0.8) -> float:
    """
    Apply engine spool-up / spool-down rate limiting to throttle command.

    Models the finite response time of turbopump-fed rocket engines.
    Spool-up is typically slower than spool-down (emergency cutoff is faster).

    Args:
        throttle_cmd: Desired throttle (0.0 - 1.0)
        throttle_prev: Previous actual throttle (0.0 - 1.0)
        dt: Time step (s)
        spool_up_time: Time to go from 0% to 100% (s)
        spool_down_time: Time to go from 100% to 0% (s)

    Returns:
        Rate-limited actual throttle (0.0 - 1.0)
    """
    delta = throttle_cmd - throttle_prev
    if delta > 0:

        max_rate = 1.0 / max(spool_up_time, 0.01)
        max_delta = max_rate * dt
        actual_delta = min(delta, max_delta)
    else:

        max_rate = 1.0 / max(spool_down_time, 0.01)
        max_delta = max_rate * dt
        actual_delta = max(delta, -max_delta)

    return float(np.clip(throttle_prev + actual_delta, 0.0, 1.0))


def altitude_above_configured_surface(r: np.ndarray, config=None) -> float:
    """Altitude above the spherical Earth surface (m)."""
    return float(vec_norm(r)) - C.R_EARTH


def _altitude_from_r(r: np.ndarray, config=None) -> float:
    return altitude_above_configured_surface(r, config)

@functools.lru_cache(maxsize=8)
def _load_aero_db_cached(path: str) -> AeroDatabase:
    db = AeroDatabase(path)
    db.filepath = path
    return db


def _get_aero_db(path: str | None) -> AeroDatabase | None:
    """Load a caller-selected aero deck, cached per path.

    Previously this reparsed the deck's JSON from disk on every single
    force call (drag/lift/moment each call it independently per
    timestep) -- fine when no deck was configured, but made an actual
    aero_deck_path unusably slow: thousands of file reads + JSON parses
    per second of simulated flight. The cache is keyed by the immutable
    path string, same pattern as _build_us76_tables above, so it carries
    none of the process-global mutable-state risk the previous docstring
    was avoiding -- a given path always loads to the same immutable
    database instance.
    """
    if path is None:
        return None
    return _load_aero_db_cached(path)


def _ref_area(config) -> float:
    """Aerodynamic reference area: from the vehicle diameter when configured."""
    return float(config.reference_area_m2) if config is not None else C.REFERENCE_AREA


def _ref_diameter(config) -> float:
    return float(config.vehicle_diameter_m) if config is not None else C.REFERENCE_DIAMETER


def compute_drag_force(
    r: np.ndarray,
    v: np.ndarray,
    wind_offset_mps: float = 0.0,
    config=None,
    alpha_deg: float = 0.0,
    beta_deg: float = 0.0,
    fin_deg: float = 0.0,
) -> np.ndarray:
    """
    Compute atmospheric drag force with Mach-dependent Cd.

    F_drag = -0.5 * ρ * Cd(Mach) * A * ||v_rel||² * v_rel_hat

    Uses Earth rotation for relative velocity (wind).
    """
    altitude = _altitude_from_r(r, config)
    _, _, rho, speed_of_sound = compute_configured_atmosphere_properties(altitude, config)

    if rho < C.DENSITY_FLOOR:
        return np.zeros(3)

    v_ground = compute_ground_relative_velocity(r, v)
    if vec_norm(v) < C.SMALL_VELOCITY_TOL or vec_norm(v_ground) < C.SMALL_VELOCITY_TOL:
        v_rel = np.zeros(3)
    else:
        v_rel = compute_relative_velocity(r, v, wind_offset_mps=wind_offset_mps)
    v_rel_norm = vec_norm(v_rel)

    if v_rel_norm < C.SMALL_VELOCITY_TOL:
        return np.zeros(3)

    mach = v_rel_norm / max(speed_of_sound, 1.0)

    aero_db = _get_aero_db(getattr(config, 'aero_deck_path', None) if config else None)
    if aero_db and aero_db.is_loaded:
        cd = aero_db.interpolate_4d(mach, alpha_deg, beta_deg, fin_deg, 'CD')
    else:
        cd = np.interp(mach, C.MACH_BREAKPOINTS, C.CD_VALUES)

    v_rel_hat = v_rel / v_rel_norm
    drag_magnitude = 0.5 * rho * cd * _ref_area(config) * v_rel_norm ** 2

    return -drag_magnitude * v_rel_hat


def compute_lift_force(
    r: np.ndarray,
    v: np.ndarray,
    q: np.ndarray,
    wind_offset_mps: float = 0.0,
    config=None,
    fin_deg: float = 0.0,
) -> np.ndarray:
    """
    Compute lift force using small-angle slender-body approximation, or
    tabular Aero-Deck if available.
    """
    altitude = _altitude_from_r(r, config)
    _, _, rho, speed_of_sound = compute_configured_atmosphere_properties(altitude, config)
    if rho < C.DENSITY_FLOOR:
        return np.zeros(3)

    v_ground = compute_ground_relative_velocity(r, v)
    if vec_norm(v) < C.SMALL_VELOCITY_TOL or vec_norm(v_ground) < C.SMALL_VELOCITY_TOL:
        return np.zeros(3)

    v_rel = compute_relative_velocity(r, v, wind_offset_mps=wind_offset_mps)
    v_rel_norm = vec_norm(v_rel)
    if v_rel_norm < C.SMALL_VELOCITY_TOL:
        return np.zeros(3)

    mach = v_rel_norm / max(speed_of_sound, 1.0)

    R = quaternion_to_rotation_matrix(q)
    v_body = R.T @ v_rel

    v_transverse = np.sqrt(v_body[0]**2 + v_body[1]**2)
    alpha = np.arctan2(v_transverse, abs(v_body[2]))
    alpha_deg = float(np.degrees(alpha))

    # Beta (sideslip) is practically 0 due to symmetrical axis, but we compute it if needed
    beta_deg = 0.0

    q_dyn = 0.5 * rho * v_rel_norm**2

    aero_db = _get_aero_db(getattr(config, 'aero_deck_path', None) if config else None)
    if aero_db and aero_db.is_loaded:
        cl = aero_db.interpolate_4d(mach, alpha_deg, beta_deg, fin_deg, "CL")
    else:
        cl_alpha = np.interp(mach, C.MACH_BREAKPOINTS, C.CL_ALPHA_VALUES)
        cl = compute_post_stall_normal_coefficient(
            alpha,
            cl_alpha,
            max_coefficient=1.2,
        )

    lift_mag = q_dyn * cl * _ref_area(config)

    body_z_inertial = R[:, 2]
    cross_intermediate = cross3(v_rel, body_z_inertial)
    cross_norm = float(vec_norm(cross_intermediate))
    v_rel_mag = float(vec_norm(v_rel))

    if cross_norm < 1e-4 * v_rel_mag:
        return np.zeros(3)
    lift_dir = cross3(cross_intermediate, v_rel)
    norm = float(vec_norm(lift_dir))
    if norm < 1e-9:
        return np.zeros(3)
    lift_dir /= norm
    # cross(cross(v, z), v) == |v|^2 * (z - (z.v_hat) v_hat), i.e. the POSITIVE
    # projection of body +Z onto the plane normal to the relative wind. That
    # is only the correct lift direction for nose-first flow (v_body[2] > 0).
    # In tail-first flight (v_body[2] < 0) it yields the exact opposite of the
    # physical normal force -Fn*u_trans that compute_aerodynamic_moment uses,
    # so the force model would amplify the lateral motion the moment model
    # damps. Apply sign(v_body[2]) to keep force and moment consistent.
    if v_body[2] < 0.0:
        lift_dir = -lift_dir
    return lift_mag * lift_dir


def _tvc_lever_arm(
    stage: int,
    vehicle_model: str,
    config: "object | None" = None,
) -> float:
    """Moment arm from the engine gimbal plane to the vehicle CG (m).

    Single source of truth for this figure - shared by the thrust-vector
    gimbal calculation below and by _simulation_step.py's TVC/RCS torque
    authority allocation, so both use the exact same lever arm.

    A real Merlin TVC gimbals within a few metres of the CG (the engine bay is
    only a short stand-off below the vehicle centre of gravity), so this is
    physically SHORT. The previous hard-coded booster value of 21 m implied a
    gimbal envelope several times larger than any flown vehicle and produced
    ~47 rad/s^2 of roll authority at landing thrust, which is hot even for a
    small booster. The constant remains the default, but config may now
    override it so the sensitivity of guidance to pitch authority is a
    measurable quantity rather than an unexamined hard-coded number.
    """
    overrides = getattr(config, "tvc_lever_arm_m", None) if config is not None else None
    if overrides is not None:
        try:
            return float(overrides)
        except (TypeError, ValueError):
            pass
    if (vehicle_model or "").lower() == "booster":
        return C.BOOSTER_TVC_LEVER_ARM
    if stage == 2:
        return C.S2_TVC_LEVER_ARM
    return C.S1_TVC_LEVER_ARM


def _gimbaled_thrust_body_vector(
    thrust_magnitude: float,
    control_torque_xy: np.ndarray | None,
    lever_arm: float | None,
) -> np.ndarray:
    """Body-frame thrust vector including TVC gimbal deflection.

    The attitude controller commands a transverse torque tau_xy. A single
    gimbaled engine produces that torque as tau = lever_arm x F_lateral, i.e.
    (for a gimbal point on the body -Z axis at distance `lever_arm` below the
    CG): tau_x = lever_arm * F_y, tau_y = -lever_arm * F_x. Inverting gives
    the lateral thrust component consistent with the commanded torque, which
    also reduces the axial component by cos(gimbal_angle) - the standard
    thrust-vector-control geometry (see e.g. Sutton, Rocket Propulsion
    Elements, TVC chapter). Only the fraction of tau_xy within the engine's
    own torque authority (thrust*sin(MAX_GIMBAL_ANGLE)*lever_arm) is treated
    as gimbal-produced; any remainder is assumed to come from RCS (matching
    the split already used in _simulation_step.py's RCS-excess accounting)
    and contributes no lateral thrust force here.
    """
    if (
        control_torque_xy is None
        or lever_arm is None
        or lever_arm <= 0.0
        or thrust_magnitude <= 0.0
    ):
        return np.array([0.0, 0.0, thrust_magnitude])

    torque_xy = np.asarray(control_torque_xy, dtype=float)[:2]
    torque_xy_mag = float(vec_norm(torque_xy))
    if torque_xy_mag < 1e-9:
        return np.array([0.0, 0.0, thrust_magnitude])

    tvc_capacity = thrust_magnitude * np.sin(C.MAX_GIMBAL_ANGLE) * lever_arm
    tvc_component_mag = min(torque_xy_mag, tvc_capacity)
    tau_x, tau_y = torque_xy * (tvc_component_mag / torque_xy_mag)

    # Invert tau = lever_arm x F for the lateral thrust components.
    f_y = tau_x / lever_arm
    f_x = -tau_y / lever_arm
    lateral_mag = float(np.hypot(f_x, f_y))

    gimbal_angle = np.arcsin(np.clip(lateral_mag / thrust_magnitude, 0.0, 1.0))
    if lateral_mag < 1e-9:
        return np.array([0.0, 0.0, thrust_magnitude])
    lateral_unit_x = f_x / lateral_mag
    lateral_unit_y = f_y / lateral_mag

    return np.array([
        thrust_magnitude * np.sin(gimbal_angle) * lateral_unit_x,
        thrust_magnitude * np.sin(gimbal_angle) * lateral_unit_y,
        thrust_magnitude * np.cos(gimbal_angle),
    ])


def compute_thrust_force(q: np.ndarray, r: np.ndarray, thrust_on: bool = True, throttle: float = 1.0,
                         stage: int = 1,
                         thrust_magnitude_override: float | None = None,
                         thrust_scale: float = 1.0,
                         config=None,
                         control_torque_xy: np.ndarray | None = None,
                         lever_arm: float | None = None) -> np.ndarray:
    """
    Compute thrust force in inertial frame with altitude compensation and throttling.

    Thrust varies with ambient pressure (Stage 1):
    T = T_vac + (T_sl - T_vac) * (P_amb / P_sl)

    Stage 2 operates in vacuum only (no pressure compensation needed).

    Args:
        q: Orientation quaternion
        r: Position vector (for altitude/pressure)
        thrust_on: active flag
        throttle: Throttle setting (0.0 to 1.0)
        stage: Engine stage (1 = S1 engines, 2 = S2 engine)
        control_torque_xy: Commanded transverse (body x,y) torque (N*m), used
            to derive the TVC gimbal deflection so the thrust vector carries
            the physically consistent lateral component and cosine loss. If
            None, thrust stays purely along body +Z (old behavior).
        lever_arm: TVC moment arm (m) for the gimbal-deflection calculation.
    """
    if not thrust_on:
        return np.zeros(3)


    throttle = float(np.clip(throttle, 0.0, 1.0))
    if throttle < 0.01:
        return np.zeros(3)


    altitude = vec_norm(r) - C.R_EARTH
    _, P_amb, _, _ = compute_configured_atmosphere_properties(altitude, config)
    P0 = C.ATM_P0

    if stage == 2:


        s2_thrust = (
            float(thrust_magnitude_override)
            if thrust_magnitude_override is not None
            else float(config.stage2_thrust_vac if config is not None else C.STAGE2_THRUST)
        )
        thrust_magnitude = s2_thrust * float(thrust_scale)

        if P_amb > 100.0:


            thrust_magnitude *= max(0.0, 1.0 - 0.1 * P_amb / P0)
    else:


        thrust_sl = (
            float(thrust_magnitude_override)
            if thrust_magnitude_override is not None
            else (float(config.stage1_thrust_n) if config is not None else C.THRUST_MAGNITUDE)
        )
        thrust_sl *= float(thrust_scale)
        pressure_thrust_scale = thrust_sl / max(C.THRUST_MAGNITUDE, 1e-9)
        thrust_vac = pressure_thrust_scale * C.MASS_FLOW_RATE * C.ISP_VAC * C.G0


        pressure_ratio = float(np.clip(P_amb / P0, 0.0, 1.2))
        thrust_magnitude = thrust_vac - (thrust_vac - thrust_sl) * pressure_ratio


    throttle_efficiency = 0.96 + 0.04 * throttle
    thrust_magnitude *= throttle * throttle_efficiency

    F_body = _gimbaled_thrust_body_vector(thrust_magnitude, control_torque_xy, lever_arm)


    R = quaternion_to_rotation_matrix(q)
    F_inertial = R @ F_body

    return F_inertial


def compute_aerodynamic_moment(r: np.ndarray, v: np.ndarray, q: np.ndarray,
                               cg_pos_z: float, cp_pos_z: float | None = None,
                               wind_offset_mps: float = 0.0,
                               omega: np.ndarray = None,
                               config=None,
                               fin_deg: float = 0.0) -> np.ndarray:
    """
    Compute aerodynamic moment about the Center of Mass (Body Frame).

    Models the aerodynamic instability (CP ahead of CG) or uses tabular Aero-Deck.
    """
    altitude = _altitude_from_r(r, config)
    _, _, rho, speed_of_sound = compute_configured_atmosphere_properties(altitude, config)

    if rho < C.DENSITY_FLOOR:
        return np.zeros(3)

    v_ground = compute_ground_relative_velocity(r, v)
    if vec_norm(v) < C.SMALL_VELOCITY_TOL or vec_norm(v_ground) < C.SMALL_VELOCITY_TOL:
        return np.zeros(3)

    v_rel = compute_relative_velocity(r, v, wind_offset_mps=wind_offset_mps)
    v_rel_norm = vec_norm(v_rel)

    if v_rel_norm < C.SMALL_VELOCITY_TOL:
        return np.zeros(3)

    R = quaternion_to_rotation_matrix(q)
    v_body = R.T @ v_rel

    vx, vy, vz = v_body

    q_dyn = 0.5 * rho * v_rel_norm**2

    v_transverse = np.sqrt(vx**2 + vy**2)
    mach = v_rel_norm / max(speed_of_sound, 1.0)

    if v_transverse < 1e-6:
        alpha = 0.0
        u_trans = np.array([1.0, 0.0, 0.0]) # Arbitrary transverse direction
    else:
        alpha = np.arctan2(v_transverse, max(abs(vz), 1e-9))
        u_trans = np.array([vx, vy, 0.0]) / v_transverse
        if alpha > _AOA_POST_STALL_START_RAD and not compute_aerodynamic_moment._post_stall_warned:
            compute_aerodynamic_moment._post_stall_warned = True
            logger.warning(
                "Angle of attack %.1f deg is post-stall; using bounded cross-flow aerodynamics.",
                np.degrees(alpha),
            )

    alpha_deg = float(np.degrees(alpha))
    beta_deg = 0.0

    aero_db = _get_aero_db(getattr(config, 'aero_deck_path', None) if config else None)

    if aero_db and aero_db.is_loaded:
        cm = aero_db.interpolate_4d(mach, alpha_deg, beta_deg, fin_deg, "CM")
        # A pitching-moment coefficient acts about the axis normal to the
        # angle-of-attack plane (z_hat_body x u_trans), which is what the
        # analytic CP/CG branch below produces via cross(r_arm, F_normal).
        # Multiplying u_trans itself put the deck torque INSIDE the AoA plane,
        # turning a pure pitching input into spurious roll/yaw, and degenerated
        # to an arbitrary constant axis when v_transverse -> 0.
        cm_axis = cross3(np.array([0.0, 0.0, 1.0]), u_trans)
        cm_axis_norm = float(vec_norm(cm_axis))
        if cm_axis_norm < 1e-9:
            cm_axis = np.array([0.0, 0.0, 1.0])
        else:
            cm_axis = cm_axis / cm_axis_norm
        # Sign convention matches the analytic branch: positive Cm is
        # destabilizing (nose-up for a CP-ahead-of-CG vehicle), which is the
        # -Fn*u_trans normal force crossed with a +z arm.
        torque_aero = cm_axis * (q_dyn * _ref_area(config) * _ref_diameter(config) * cm)
    else:
        cn = compute_post_stall_normal_coefficient(
            alpha,
            C.C_N_ALPHA,
            max_coefficient=1.6,
        )
        Fn_mag = q_dyn * _ref_area(config) * cn
        F_normal_body = -Fn_mag * u_trans

        if cp_pos_z is None:
            cp_pos_z = C.H_CP
        cp_effective = float(cp_pos_z)
        # The tail-first CP relocation below is booster-specific geometry
        # (config.booster_tail_first_cp_z_m defaults to the Stage-1 recovery CP
        # of ~31 m). It previously fired for ANY vehicle whose axial flow was
        # negative, so the orbiter -- whose CG is ~58-63 m -- silently had its
        # CP overwritten with booster geometry. Gate it on the vehicle actually
        # being the booster, identified by its CG being in the Stage-1 recovery
        # range (the caller supplies the correct per-model CG), and use the
        # same configured value for the broadside limit so a user override of
        # booster_tail_first_cp_z_m is not silently discarded at broadside.
        if vz < -C.SMALL_VELOCITY_TOL and cg_pos_z <= C.STAGE1_RECOVERY_CP + 1.0:
            tail_cp = float(getattr(config, "booster_tail_first_cp_z_m", C.STAGE1_RECOVERY_CP))
            broadside_fraction = float(np.sin(alpha) ** 2)
            broadside_cp = min(cp_effective, tail_cp)
            cp_effective = ((1.0 - broadside_fraction) * tail_cp + broadside_fraction * broadside_cp)

        arm_z = cp_effective - cg_pos_z
        r_arm = np.array([0.0, 0.0, arm_z])
        torque_aero = cross3(r_arm, F_normal_body)

    if omega is not None:
        damp_coeff = (
            C.C_M_DAMPING * q_dyn * _ref_area(config)
            * _ref_diameter(config) ** 2 / (2.0 * max(v_rel_norm, 1.0))
        )
        torque_aero[0] += damp_coeff * omega[0]
        torque_aero[1] += damp_coeff * omega[1]

    return torque_aero


compute_aerodynamic_moment._post_stall_warned = False


def _booster_recovery_aero_scale(
    mode: str | None,
    grid_fin_deployed: float = 0.0,
    landing_leg_deployed: float = 0.0,
    reference_area: float | None = None,
    config=None,
) -> tuple[float, float]:
    """Effective drag/moment scaling for booster recovery configurations.

    Computes scaling from deployed hardware geometry rather than magic numbers:
    - Grid fins: add projected area and shift CP aft
    - Landing legs: add parasitic drag when deployed
    - Entry burn: engine plume increases effective base area

    Args:
        mode: Current booster mission phase name
        grid_fin_deployed: Grid fin deployment fraction (0-1). If 0, estimated
            from phase (deployed during entry and landing).
        landing_leg_deployed: Landing leg deployment fraction (0-1). If 0,
            estimated from phase (deployed during landing only).
        reference_area: Vehicle reference area (defaults to C.REFERENCE_AREA)

    Returns:
        (drag_scale, moment_scale) tuple
    """
    if reference_area is None:
        reference_area = _ref_area(config)

    normalized = (mode or "").upper()


    if grid_fin_deployed <= 0.0:
        if normalized in ("BOOSTER_ENTRY", "BOOSTER_LANDING"):
            grid_fin_deployed = 1.0
        elif normalized == "BOOSTER_COAST":
            grid_fin_deployed = 0.5
    if landing_leg_deployed <= 0.0 and normalized == "BOOSTER_LANDING":
        landing_leg_deployed = 1.0


    base_area = reference_area


    if config is not None:
        grid_fin_total_area = float(
            getattr(config, "grid_fin_drag_area_m2", C.GRID_FIN_COUNT * C.GRID_FIN_AREA_M2)
        )
    else:
        grid_fin_total_area = C.GRID_FIN_COUNT * C.GRID_FIN_AREA_M2
    grid_fin_area = grid_fin_total_area * grid_fin_deployed
    grid_fin_drag_area = grid_fin_area * C.GRID_FIN_CD


    leg_area = C.LANDING_LEG_COUNT * C.LANDING_LEG_AREA_M2 * landing_leg_deployed
    leg_drag_area = leg_area * C.LANDING_LEG_CD


    entry_plume_factor = 1.0
    if normalized == "BOOSTER_ENTRY":
        entry_plume_factor = 1.3

    # Known simplification: this returns an area-ratio multiplier applied on
    # top of the vehicle body's own Mach-dependent Cd (CD_VALUES table), so
    # the grid fins/legs are implicitly assumed to share the body's Cd-vs-Mach
    # shape rather than using an independent fin/leg Cd curve. No public Cd
    # curve exists for this specific fin geometry, so this documents the
    # approximation rather than substituting an unverified one.
    total_drag_area = (base_area + grid_fin_drag_area + leg_drag_area) * entry_plume_factor
    drag_scale = total_drag_area / max(base_area, 1e-9)


    moment_scale = 1.0
    if grid_fin_deployed > 0.0:


        moment_scale = 1.0 + 2.5 * grid_fin_deployed
    if normalized == "BOOSTER_ENTRY":

        moment_scale *= 1.2

    return float(np.clip(drag_scale, 1.0, 8.0)), float(np.clip(moment_scale, 1.0, 6.0))


def _compute_force_breakdown(
    r: np.ndarray,
    v: np.ndarray,
    q: np.ndarray,
    m: float,
    thrust_on: bool = True,
    throttle: float = 1.0,
    stage: int = 1,
    vehicle_model: str = "stacked",
    booster_aero_mode: str | None = None,
    thrust_magnitude_override: float | None = None,
    enable_j2: bool = False,
    j2_coefficient: float = 1.08263e-3,
    gravity_model: str = "central",
    thrust_scale: float = 1.0,
    wind_offset_mps: float = 0.0,
    grid_fin_command=None,
    config=None,
    control_torque_xy: np.ndarray | None = None,
) -> ForceBreakdown:
    """Compute the same force components used by the live dynamics path."""
    model = gravity_model
    if (model or "central").lower() == "central" and enable_j2:
        model = "j2"
    F_grav = compute_gravity_force(r, m, enable_j2=enable_j2, j2=j2_coefficient,
                                   gravity_model=model)
    F_thrust = compute_thrust_force(
        q,
        r,
        thrust_on,
        throttle,
        stage=stage,
        thrust_magnitude_override=thrust_magnitude_override,
        thrust_scale=thrust_scale,
        config=config,
        control_torque_xy=control_torque_xy,
        lever_arm=_tvc_lever_arm(stage, vehicle_model, config=config) if control_torque_xy is not None else None,
    )

    altitude_m = _altitude_from_r(r, config)
    atmosphere_enabled = bool(getattr(config, "enable_atmosphere", True))
    drag_enabled = atmosphere_enabled and bool(getattr(config, "enable_drag", True))
    lift_enabled = atmosphere_enabled and bool(getattr(config, "enable_lift", True))
    if altitude_m > C.AERO_DISABLE_ALTITUDE or not (drag_enabled or lift_enabled):
        F_drag = np.zeros(3)
        F_lift = np.zeros(3)
    else:
        # compute_drag_force takes alpha/beta/fin for its aero-deck lookup, but
        # the body-axes velocity needed to form alpha was never passed in, so
        # every drag lookup silently sampled the alpha=0/beta=0/fin=0 slice
        # while lift (via compute_lift_force) correctly saw incidence. Compute
        # the incidence here and hand it to both so the deck is sampled
        # consistently.
        _deck_alpha_deg = 0.0
        _deck_beta_deg = 0.0
        _deck_fin_deg = 0.0
        if getattr(config, 'aero_deck_path', None):
            _R = quaternion_to_rotation_matrix(q)
            _v_body = _R.T @ (v - cross3(np.array([0.0, 0.0, C.EARTH_ROTATION_RATE]), r))
            _v_t = float(np.hypot(_v_body[0], _v_body[1]))
            _deck_alpha_deg = float(
                np.degrees(np.arctan2(_v_t, max(abs(_v_body[2]), 1e-9)))
            )
            if grid_fin_command is not None:
                _deck_fin_deg = float(
                    np.degrees(np.hypot(
                        float(getattr(grid_fin_command, 'pitch_cmd_deg', 0.0)),
                        float(getattr(grid_fin_command, 'yaw_cmd_deg', 0.0)),
                    ))
                )
        F_drag = (
            compute_drag_force(
                r, v, wind_offset_mps=wind_offset_mps, config=config,
                alpha_deg=_deck_alpha_deg, beta_deg=_deck_beta_deg,
                fin_deg=_deck_fin_deg,
            )
            if drag_enabled else np.zeros(3)
        )
        F_lift = (
            compute_lift_force(r, v, q, wind_offset_mps=wind_offset_mps, config=config)
            if lift_enabled else np.zeros(3)
        )
        if (vehicle_model or "").lower() == "booster":


            drag_scale, _ = _booster_recovery_aero_scale(booster_aero_mode, config=config)
            F_drag = drag_scale * F_drag

    if (vehicle_model or "").lower() == "orbiter" and booster_aero_mode in ("S2_ENTRY", "S2_LANDING"):
        s2_scale = float(getattr(config, "s2_entry_drag_scale", 45.0)) if config is not None else 45.0
        F_drag = s2_scale * F_drag

    F_grid_fin = np.zeros(3)
    if grid_fin_command is not None and config is not None and (vehicle_model or "").lower() == "booster":
        from .recovery_hardware import compute_grid_fin_force

        F_grid_fin = compute_grid_fin_force(
            r,
            v,
            q,
            grid_fin_command,
            config,
            wind_offset_mps=wind_offset_mps,
        )

    total = F_grav + F_thrust + F_drag + F_lift + F_grid_fin
    return {
        'gravity': F_grav,
        'thrust': F_thrust,
        'drag': F_drag,
        'lift': F_lift,
        'grid_fin': F_grid_fin,
        'total': total,
        'gravity_magnitude': vec_norm(F_grav),
        'thrust_magnitude': vec_norm(F_thrust),
        'drag_magnitude': vec_norm(F_drag),
        'lift_magnitude': vec_norm(F_lift),
        'grid_fin_magnitude': vec_norm(F_grid_fin),
    }


def compute_specific_forces(r: np.ndarray, v: np.ndarray, q: np.ndarray,
                            m: float, thrust_on: bool = True, stage: int = 1,
                            throttle: float = 1.0,
                            vehicle_model: str = "stacked",
                            booster_aero_mode: str | None = None,
                            thrust_magnitude_override: float | None = None,
                            enable_j2: bool = False,
                            j2_coefficient: float = 1.08263e-3,
                            gravity_model: str = "central",
                            thrust_scale: float = 1.0,
                            wind_offset_mps: float = 0.0,
                            grid_fin_command=None,
                            config=None,
                            control_torque_xy: np.ndarray | None = None) -> ForceBreakdown:
    """Compute force components using the live translational force model."""
    return _compute_force_breakdown(
        r, v, q, m, thrust_on, throttle, stage, vehicle_model,
        booster_aero_mode, thrust_magnitude_override, enable_j2,
        j2_coefficient, gravity_model, thrust_scale, wind_offset_mps,
        grid_fin_command, config, control_torque_xy,
    )

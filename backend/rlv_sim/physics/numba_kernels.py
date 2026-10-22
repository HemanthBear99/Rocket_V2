"""Optional Numba-compiled kernels for the two hottest numeric paths.

* ``egm96_accel`` -- spherical-harmonic gravity (called 4x per RK4 step).
* ``propagate_to_surface`` -- the ballistic impact predictor used by booster
  guidance (hundreds of gravity+drag evaluations per call).

Both mirror the pure-Python implementations in ``forces.py`` / ``recovery.py``
operation for operation. Results agree to floating-point round-off (compiled
transcendental functions may differ from numpy's in the last bit).

Numba is an optional dependency: when it is missing, ``AVAILABLE`` is False and
callers keep using the pure-Python code paths.
"""

import math
import sys

import numpy as np

try:
    from numba import njit

    AVAILABLE = True
except ImportError:  # pragma: no cover - exercised only without numba
    AVAILABLE = False

    def njit(*args, **kwargs):
        if args and callable(args[0]):
            return args[0]
        return lambda fn: fn


# On-disk compile cache speeds up later runs; frozen (PyInstaller) builds have
# no writable source tree to cache into.
_CACHE = not getattr(sys, "frozen", False)


@njit(cache=_CACHE)
def _norm3(x):
    return math.sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2])


@njit(cache=_CACHE)
def egm96_accel(x, y, z, n_max, mu, a_e, terms, sectoral, subdiag, rec_a, rec_b):
    """EGM96 acceleration; see forces.compute_egm96_gravity_accel."""
    r_sph = math.sqrt(x * x + y * y + z * z)
    sin_phi = z / r_sph
    cos_phi = math.sqrt(max(x * x + y * y, 0.0)) / r_sph
    lam = math.atan2(y, x)
    sin_phi = min(max(sin_phi, -1.0), 1.0)
    cos_phi = min(max(cos_phi, 0.0), 1.0)
    sin_lam = math.sin(lam)
    cos_lam = math.cos(lam)

    size = n_max + 1
    P = np.zeros((size, size))
    dP = np.zeros((size, size))
    Q = np.zeros((size, size))
    P[0, 0] = 1.0
    for m in range(n_max + 1):
        if m > 0:
            factor = sectoral[m]
            previous = P[m - 1, m - 1]
            P[m, m] = factor * cos_phi * previous
            dP[m, m] = factor * (-sin_phi * previous + cos_phi * dP[m - 1, m - 1])
            Q[m, m] = factor * previous
        if m < n_max:
            factor = subdiag[m]
            P[m + 1, m] = factor * sin_phi * P[m, m]
            dP[m + 1, m] = factor * (cos_phi * P[m, m] + sin_phi * dP[m, m])
            if m > 0:
                Q[m + 1, m] = factor * sin_phi * Q[m, m]
        for n in range(m + 2, n_max + 1):
            a_n = rec_a[n, m]
            b_n = rec_b[n, m]
            P[n, m] = a_n * sin_phi * P[n - 1, m] - b_n * P[n - 2, m]
            dP[n, m] = a_n * (cos_phi * P[n - 1, m] + sin_phi * dP[n - 1, m]) - b_n * dP[n - 2, m]
            if m > 0:
                Q[n, m] = a_n * sin_phi * Q[n - 1, m] - b_n * Q[n - 2, m]

    dU_dr = 0.0
    dU_dphi = 0.0
    dU_dlam = 0.0
    for k in range(terms.shape[0]):
        n = int(terms[k, 0])
        m = int(terms[k, 1])
        if n > n_max:
            continue
        c = terms[k, 2]
        s = terms[k, 3]
        ratio = (a_e / r_sph) ** n
        if m == 0:
            cos_mlam = 1.0
            sin_mlam = 0.0
        else:
            cos_mlam = math.cos(m * lam)
            sin_mlam = math.sin(m * lam)
        Y = c * cos_mlam + s * sin_mlam
        dU_dr += -(n + 1) * ratio * P[n, m] * Y
        dU_dphi += ratio * dP[n, m] * Y
        if m != 0:
            dU_dlam += ratio * Q[n, m] * m * (-c * sin_mlam + s * cos_mlam)

    g = mu / r_sph ** 2
    out = np.empty(3)
    r_hat0 = cos_phi * cos_lam
    r_hat1 = cos_phi * sin_lam
    r_hat2 = sin_phi
    out[0] = g * dU_dr * r_hat0 + g * dU_dphi * (-sin_phi * cos_lam) + g * dU_dlam * (-sin_lam) - g * r_hat0
    out[1] = g * dU_dr * r_hat1 + g * dU_dphi * (-sin_phi * sin_lam) + g * dU_dlam * cos_lam - g * r_hat1
    out[2] = g * dU_dr * r_hat2 + g * dU_dphi * cos_phi - g * r_hat2
    return out


@njit(cache=_CACHE)
def _us76(h_in, enable_upper, H, L, TB, PB, g0, r_gas, gamma, density_floor, sos_fallback):
    """US-76 (T, rho, speed of sound); see forces.compute_atmosphere_properties."""
    h = max(0.0, h_in)
    if h <= H[H.shape[0] - 1]:
        idx = np.searchsorted(H, h, side='right') - 1
        idx = max(0, min(idx, L.shape[0] - 1))
        lapse = L[idx]
        T0 = TB[idx]
        P0 = PB[idx]
        dh = h - H[idx]
        if abs(lapse) > 1e-12:
            T = T0 + lapse * dh
            P = P0 * (T / T0) ** (-g0 / (lapse * r_gas))
        else:
            T = T0
            P = P0 * math.exp(-g0 * dh / (r_gas * T0))
    elif enable_upper:
        T = TB[TB.shape[0] - 1]
        P = PB[PB.shape[0] - 1] * math.exp(-(h - H[H.shape[0] - 1]) / 5000.0)
    else:
        T = TB[TB.shape[0] - 1]
        P = 0.0
    rho = P / (r_gas * T) if (T > 0.0 and P > 0.0) else 0.0
    if rho < density_floor:
        rho = 0.0
    sos = math.sqrt(gamma * r_gas * T) if T > 0.0 else sos_fallback
    return rho, sos


@njit(cache=_CACHE)
def propagate_to_surface(r0, v0, max_steps, dt, include_drag, drag_scale, mass_kg,
                         mu, p, H, L, TB, PB, mach_bp, cd_vals):
    """Ballistic RK4 to the surface; see recovery._propagate_2body_to_surface.

    Returns (hit, r_surface, time_of_flight).
    """
    (r_earth, omega_e, wind_offset, wind_ref_speed, wind_ref_alt, wind_exp,
     cos_az, sin_az, zero_tol, small_v, density_floor, rho_scale, enable_upper,
     enable_atm, g0, r_gas, gamma, sos_fallback, ref_area) = (
        p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10],
        p[11], p[12], p[13] > 0.5, p[14], p[15], p[16], p[17], p[18])
    upper = enable_upper > 0.5
    mass_div = max(mass_kg, 1.0)

    def accel(pos, vel):
        rn = max(_norm3(pos), 1.0)
        k = -mu / (rn ** 3)
        a = np.empty(3)
        a[0] = k * pos[0]
        a[1] = k * pos[1]
        a[2] = k * pos[2]
        if not include_drag:
            return a
        r_norm = _norm3(pos)
        altitude = r_norm - r_earth
        rho, sos = _us76(altitude, upper, H, L, TB, PB, g0, r_gas, gamma,
                         density_floor, sos_fallback)
        if not enable_atm:
            rho = 0.0
        rho = rho * rho_scale
        if rho < density_floor:
            rho = 0.0
        if rho < density_floor:
            return a
        # Ground-relative velocity: v - omega x r (omega along +Z).
        vg0 = vel[0] + omega_e * pos[1]
        vg1 = vel[1] - omega_e * pos[0]
        vg2 = vel[2]
        if _norm3(vel) < small_v or math.sqrt(vg0 * vg0 + vg1 * vg1 + vg2 * vg2) < small_v:
            return a
        # Wind (see utils._wind_vector).
        w0 = 0.0
        w1 = 0.0
        w2 = 0.0
        if altitude > 0.0:
            speed = wind_ref_speed * (altitude / wind_ref_alt) ** wind_exp + wind_offset
            if altitude < 5000.0:
                xs = altitude / 5000.0
                speed *= xs * xs * (3.0 - 2.0 * xs)
            inv = 1.0 / max(r_norm, 1e-9)
            u0 = pos[0] * inv
            u1 = pos[1] * inv
            u2 = pos[2] * inv
            e0 = -u1
            e1 = u0
            e2 = 0.0
            en = math.sqrt(e0 * e0 + e1 * e1)
            if en < zero_tol:
                e0 = 0.0
                e1 = 1.0
                en = 1.0
            e0 /= en
            e1 /= en
            n0 = u1 * e2 - u2 * e1
            n1 = u2 * e0 - u0 * e2
            n2 = u0 * e1 - u1 * e0
            nn = max(math.sqrt(n0 * n0 + n1 * n1 + n2 * n2), 1e-9)
            n0 /= nn
            n1 /= nn
            n2 /= nn
            w0 = speed * (cos_az * n0 + sin_az * e0)
            w1 = speed * (cos_az * n1 + sin_az * e1)
            w2 = speed * (cos_az * n2 + sin_az * e2)
        vr0 = vg0 - w0
        vr1 = vg1 - w1
        vr2 = vg2 - w2
        vn = math.sqrt(vr0 * vr0 + vr1 * vr1 + vr2 * vr2)
        if vn < small_v:
            return a
        mach = vn / max(sos, 1.0)
        cd = np.interp(mach, mach_bp, cd_vals)
        drag_mag = 0.5 * rho * cd * ref_area * vn ** 2
        f = -drag_mag / vn * drag_scale / mass_div
        a[0] += f * vr0
        a[1] += f * vr1
        a[2] += f * vr2
        return a

    r_cur = r0.copy()
    v_cur = v0.copy()
    elapsed = 0.0
    for _ in range(max_steps):
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
        r_prev_norm = _norm3(r_cur)
        r_new_norm = _norm3(r_new)
        elapsed += dt
        if r_new_norm <= r_earth:
            f = (r_earth - r_new_norm) / max(r_prev_norm - r_new_norm, 1e-12)
            f = min(max(f, 0.0), 1.0)
            r_surface = r_new + f * (r_cur - r_new)
            r_surface = r_earth * r_surface / max(_norm3(r_surface), 1.0)
            return True, r_surface, elapsed - dt * (1.0 - f)
        r_cur = r_new
        v_cur = v_new
    return False, r_cur, elapsed

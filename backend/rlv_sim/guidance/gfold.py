"""G-FOLD-style convex powered-descent guidance for the booster landing burn.

Implements the lossless-convexification minimum-fuel powered-descent problem
of Acikmese & Ploen (2007) and Blackmore, Acikmese & Scharf (2010)
("G-FOLD"): a point-mass descent in a local East-North-Up frame at the pad,
with

* upper and lower thrust bounds (the engine cannot throttle below its minimum),
  made convex with the slack ``sigma`` and the log-mass ``z = ln m``,
* a thrust-pointing (tilt) cone about local vertical,
* a glide-slope cone that keeps the vehicle above the ground near the pad,
* mass depletion through the engine's mass-flow coefficient,
* arrival at a terminal gate above the pad (a few tens of metres up,
  descending slowly), maximising final mass. Below the gate a simple
  vertical-descent law flies the last metres, as in flight practice: a
  minimum-fuel plan that ends at exactly zero velocity on the ground leaves
  no margin for the 6-DOF tracking lag.

This is GUIDANCE only. The plan is a 3-DOF point-mass trajectory; the 6-DOF
simulation still flies it with full attitude dynamics, gimbal limits, drag and
wind. The plan is re-solved periodically (receding horizon) from the current
6-DOF state, and the commanded thrust direction/throttle are interpolated
from the latest plan between solves.

Requires the optional ``cvxpy`` package (``gfold`` extra). Without it
``AVAILABLE`` is False and callers fall back to the heuristic guidance.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass

import numpy as np

try:
    import cvxpy as cp

    AVAILABLE = True
except Exception:  # noqa: BLE001  # pragma: no cover - cvxpy missing or broken (frozen build)
    # Any import failure disables G-FOLD instead of crashing the app; the
    # option is hidden in the UI and selecting it raises a clear error.
    cp = None
    AVAILABLE = False


@dataclass
class GfoldPlan:
    """A solved descent: thrust accelerations (ENU, m/s^2) at uniform nodes."""

    t_start: float
    dt: float
    accel_enu: np.ndarray        # (N+1, 3) thrust acceleration T/m
    time_of_flight: float
    final_mass_kg: float
    frame: np.ndarray            # 3x3: rows are east, north, up unit vectors (ECI)

    def accel_at(self, t: float) -> np.ndarray | None:
        """Linearly interpolated thrust acceleration at time t (ENU)."""
        x = (t - self.t_start) / self.dt
        n = self.accel_enu.shape[0] - 1
        if x < 0.0 or x > n:
            return None
        k = min(int(x), n - 1)
        f = x - k
        return (1.0 - f) * self.accel_enu[k] + f * self.accel_enu[k + 1]


@dataclass(frozen=True)
class DescentLimits:
    """Vehicle limits for one solve."""

    thrust_min_n: float
    thrust_max_n: float
    mass_flow_per_newton: float   # alpha = mdot / T  (kg/s per N)
    dry_mass_kg: float
    max_tilt_deg: float
    glideslope_deg: float
    gravity_mps2: float


class _DescentProblem:
    """Parametrised SOCP for a fixed number of nodes (compiled once, re-solved)."""

    def __init__(self, nodes: int, max_tilt_deg: float, glideslope_deg: float):
        n = nodes
        self.nodes = n
        self.r0 = cp.Parameter(3)
        self.v0 = cp.Parameter(3)
        self.z_init = cp.Parameter()
        self.z_dry = cp.Parameter()
        self.r_final = cp.Parameter(3)
        self.v_final = cp.Parameter(3)
        self.dt = cp.Parameter(nonneg=True)
        self.half_dt = cp.Parameter(nonneg=True)
        self.dt2_12 = cp.Parameter(nonneg=True)
        self.alpha_half_dt = cp.Parameter(nonneg=True)
        self.g_dt = cp.Parameter(3)
        self.z0 = cp.Parameter(n + 1)
        self.mu1 = cp.Parameter(n + 1, nonneg=True)
        self.mu2 = cp.Parameter(n + 1, nonneg=True)
        self.z_min = cp.Parameter(n + 1)
        self.z_max = cp.Parameter(n + 1)

        r = cp.Variable((n + 1, 3))
        v = cp.Variable((n + 1, 3))
        u = cp.Variable((n + 1, 3))
        z = cp.Variable(n + 1)
        s = cp.Variable(n + 1)
        w = cp.Variable(n + 1)
        self.r, self.v, self.u, self.z = r, v, u, z

        cos_tilt = math.cos(math.radians(max_tilt_deg))
        cot_glideslope = 1.0 / math.tan(math.radians(glideslope_deg))
        cons = [
            r[0] == self.r0,
            v[0] == self.v0,
            z[0] == self.z_init,
            r[n] == self.r_final,
            v[n] == self.v_final,
            z[n] >= self.z_dry,
            # Trapezoidal dynamics with constant gravity.
            v[1:] == v[:-1] + self.half_dt * (u[:-1] + u[1:]) + np.ones((n, 1)) @ cp.reshape(self.g_dt, (1, 3), order="C"),
            r[1:] == r[:-1] + self.half_dt * (v[:-1] + v[1:]) + self.dt2_12 * (u[:-1] - u[1:]),
            z[1:] == z[:-1] - self.alpha_half_dt * (s[:-1] + s[1:]),
            # Lossless convexification of the thrust bounds.
            cp.norm(u, 2, axis=1) <= s,
            w == z - self.z0,
            s >= cp.multiply(self.mu1, 1.0 - w + 0.5 * cp.square(w)),
            s <= cp.multiply(self.mu2, 1.0 - w),
            z >= self.z_min,
            z <= self.z_max,
            # Thrust pointing: within max_tilt of local vertical.
            u[:, 2] >= cos_tilt * s,
            # Glide slope: stay inside an upward cone about the pad.
            cp.norm(r[:, :2], 2, axis=1) <= cot_glideslope * r[:, 2],
        ]
        self.problem = cp.Problem(cp.Maximize(z[n]), cons)

    def solve(self, r0, v0, m0, tf, lim: DescentLimits, r_final, v_final):
        n = self.nodes
        dt = tf / n
        t = np.arange(n + 1) * dt
        alpha = lim.mass_flow_per_newton
        m_low = np.maximum(m0 - alpha * lim.thrust_max_n * t, lim.dry_mass_kg)
        m_high = np.maximum(m0 - alpha * lim.thrust_min_n * t, lim.dry_mass_kg)
        z0 = np.log(m_low)
        self.r0.value = r0
        self.r_final.value = r_final
        self.v_final.value = v_final
        self.v0.value = v0
        self.z_init.value = math.log(m0)
        self.z_dry.value = math.log(lim.dry_mass_kg)
        self.dt.value = dt
        self.half_dt.value = 0.5 * dt
        self.dt2_12.value = dt * dt / 12.0
        self.alpha_half_dt.value = 0.5 * alpha * dt
        self.g_dt.value = np.array([0.0, 0.0, -lim.gravity_mps2 * dt])
        self.z0.value = z0
        self.mu1.value = lim.thrust_min_n * np.exp(-z0)
        self.mu2.value = lim.thrust_max_n * np.exp(-z0)
        self.z_min.value = z0
        self.z_max.value = np.log(m_high)
        try:
            # Cold start every solve: warm starts made results depend on the
            # previous solve, so identical missions differed run to run.
            self.problem.solve(solver=cp.CLARABEL, warm_start=False)
        except cp.SolverError:
            return None
        if self.problem.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
            return None
        return np.asarray(self.u.value), float(math.exp(self.z.value[n])), dt


# Compiled problems are cached per thread: solve() writes cvxpy Parameter
# values before solving, so sharing one instance across the server's
# concurrent mission threads would let runs overwrite each other's inputs.
_THREAD_LOCAL = threading.local()


def _problem(nodes: int, max_tilt_deg: float, glideslope_deg: float) -> _DescentProblem:
    key = (nodes, round(max_tilt_deg, 6), round(glideslope_deg, 6))
    problems = getattr(_THREAD_LOCAL, "problems", None)
    if problems is None:
        problems = _THREAD_LOCAL.problems = {}
    if key not in problems:
        problems[key] = _DescentProblem(nodes, max_tilt_deg, glideslope_deg)
    return problems[key]


def require_available(config) -> None:
    """Raise if G-FOLD guidance is selected but cvxpy is not installed."""
    if getattr(config, "booster_landing_guidance", "heuristic") == "gfold" and not AVAILABLE:
        raise ValueError(
            "booster_landing_guidance='gfold' needs the optional 'cvxpy' package "
            "(install the 'gfold' extra)"
        )


def local_frame(site_eci: np.ndarray) -> np.ndarray:
    """Rows: east, north, up unit vectors at the landing site (ECI)."""
    up = site_eci / np.linalg.norm(site_eci)
    east = np.cross([0.0, 0.0, 1.0], up)
    east /= np.linalg.norm(east)
    north = np.cross(up, east)
    return np.vstack([east, north, up])


def plan_descent(
    r_eci: np.ndarray,
    v_ground_eci: np.ndarray,
    mass_kg: float,
    t: float,
    site_eci: np.ndarray,
    limits: DescentLimits,
    tf_guess_s: float,
    nodes: int = 25,
    gate_altitude_m: float = 0.0,
    gate_descent_rate_mps: float = 0.0,
) -> GfoldPlan | None:
    """Solve for the minimum-fuel descent; search the time of flight around a guess.

    Returns None when no time of flight in the search window is feasible.
    """
    if not AVAILABLE:
        return None
    frame = local_frame(site_eci)
    r0 = frame @ (np.asarray(r_eci, dtype=float) - site_eci)
    v0 = frame @ np.asarray(v_ground_eci, dtype=float)
    if r0[2] <= gate_altitude_m:
        return None
    r_final = np.array([0.0, 0.0, float(gate_altitude_m)])
    v_final = np.array([0.0, 0.0, -float(gate_descent_rate_mps)])
    problem = _problem(nodes, limits.max_tilt_deg, limits.glideslope_deg)
    best = None
    for factor in (0.8, 1.0, 1.25, 1.6, 2.0, 2.6):
        tf = max(1.0, factor * tf_guess_s)
        solved = problem.solve(r0, v0, mass_kg, tf, limits, r_final, v_final)
        if solved is None:
            continue
        accel, final_mass, dt = solved
        if best is None or final_mass > best.final_mass_kg:
            best = GfoldPlan(t, dt, accel, tf, final_mass, frame)
        elif best is not None:
            # Final mass falls once past the optimum time of flight.
            break
    return best


def command_from_plan(plan: GfoldPlan, t: float, mass_kg: float, thrust_max_n: float):
    """Thrust direction (ECI unit vector) and throttle fraction at time t."""
    accel_enu = plan.accel_at(t)
    if accel_enu is None:
        return None
    magnitude = float(np.linalg.norm(accel_enu))
    if magnitude < 1e-9:
        return None
    direction_eci = plan.frame.T @ (accel_enu / magnitude)
    throttle = magnitude * mass_kg / max(thrust_max_n, 1e-9)
    return direction_eci, float(np.clip(throttle, 0.0, 1.0))


__all__ = ["AVAILABLE", "DescentLimits", "GfoldPlan", "command_from_plan", "local_frame", "plan_descent"]

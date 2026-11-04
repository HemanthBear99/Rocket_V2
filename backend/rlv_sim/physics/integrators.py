"""
RLV Phase-I Ascent Simulation - Numerical Integration

This module implements the RK4 integrator for all state variables
with quaternion normalization after each step.

References and convergence requirements:
    docs/MATHEMATICAL_REFERENCES.md#2-numerical-integration
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from rlv_sim.core import constants as C
from rlv_sim.core.frames import quaternion_normalize
from rlv_sim.core.state import State
from rlv_sim.physics.dynamics import DynamicsContext, state_derivative_vector
from rlv_sim.physics.mass import compute_mass_derivative, default_dry_mass


def rk4_step(state: State, torque: np.ndarray, dt: float,
             ctx: DynamicsContext) -> State:
    """
    Perform a single RK4 integration step with performance optimizations.

    The RK4 method computes:
    k1 = f(t, y)
    k2 = f(t + dt/2, y + dt/2 * k1)
    k3 = f(t + dt/2, y + dt/2 * k2)
    k4 = f(t + dt, y + dt * k3)
    y_new = y + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

    Performance optimizations:
    - Conditional quaternion normalization (only at final step)
    - Pre-allocated arrays for reduced memory allocation
    - Optimized parameter passing through DynamicsContext

    Args:
        state: Current state
        torque: Control torque in body frame (N*m)
        dt: Time step (s)
        ctx: DynamicsContext with engine, aero, and physics configuration

    Returns:
        New state after integration

    Raises:
        ValueError: If dt <= 0 or torque has wrong shape
    """

    dry_mass = ctx.dry_mass
    if dry_mass is None:
        dry_mass = default_dry_mass(ctx.stage, ctx.config)

    if dt <= 0:
        raise ValueError(f"Time step dt must be positive, got {dt}")
    if torque.shape != (3,):
        raise ValueError(f"Torque must have shape (3,), got {torque.shape}")
    if np.any(np.isnan(torque)):
        raise ValueError("Torque contains NaN values")

    _cfg = ctx.config
    if ctx.thrust_on and ctx.throttle > 0.0 and state.m > dry_mass:
        mdot_now = compute_mass_derivative(
            state.m,
            thrust_on=True,
            throttle=ctx.throttle,
            dry_mass=dry_mass,
            stage=ctx.stage,
            thrust_magnitude_override=ctx.thrust_magnitude_override,
            thrust_scale=_cfg.runtime_thrust_scale if _cfg is not None else 1.0,
            isp_scale=_cfg.runtime_isp_scale if _cfg is not None else 1.0,
            stage2_thrust_vac=_cfg.stage2_thrust_vac if _cfg is not None else None,
            stage2_isp_vac=_cfg.stage2_isp_vac if _cfg is not None else None,
            stage1_thrust_n=_cfg.stage1_thrust_n if _cfg is not None else None,
            stage1_isp_sl=_cfg.stage1_isp_sl if _cfg is not None else None,
        )
        if mdot_now < 0.0 and state.m + mdot_now * dt < dry_mass:
            t_to_dry = (state.m - dry_mass) / (-mdot_now)
            # Both sub-steps use the plain RK4 core: re-entering rk4_step let
            # rounding re-trigger this split with identical arguments until
            # Python's recursion limit (seen at S2 m=6675.133319624401, dt=20).
            coast_ctx = replace(ctx, thrust_on=False, throttle=0.0)
            if t_to_dry <= C.ZERO_TOLERANCE:
                return _rk4_core(state, torque, dt, coast_ctx, dry_mass)
            powered_dt = min(float(t_to_dry), float(dt))
            coast_dt = float(dt) - powered_dt
            powered_state = _rk4_core(state, torque, powered_dt, ctx, dry_mass)
            powered_state.m = dry_mass
            if coast_dt <= C.ZERO_TOLERANCE:
                return powered_state
            return _rk4_core(powered_state, torque, coast_dt, coast_ctx, dry_mass)

    return _rk4_core(state, torque, dt, ctx, dry_mass)


def _rk4_core(state: State, torque: np.ndarray, dt: float, ctx, dry_mass: float) -> State:
    """One classical RK4 step with quaternion renormalisation (no exhaustion split)."""
    t = state.t
    y = state.to_vector()

    k1 = np.zeros(14, dtype=np.float64)
    k2 = np.zeros(14, dtype=np.float64)
    k3 = np.zeros(14, dtype=np.float64)
    k4 = np.zeros(14, dtype=np.float64)

    k1[:] = state_derivative_vector(y, t, torque, ctx)

    y2 = y + 0.5 * dt * k1
    y2[6:10] = quaternion_normalize(y2[6:10])
    k2[:] = state_derivative_vector(y2, t + 0.5*dt, torque, ctx)

    y3 = y + 0.5 * dt * k2
    y3[6:10] = quaternion_normalize(y3[6:10])
    k3[:] = state_derivative_vector(y3, t + 0.5*dt, torque, ctx)

    y4 = y + dt * k3
    y4[6:10] = quaternion_normalize(y4[6:10])
    k4[:] = state_derivative_vector(y4, t + dt, torque, ctx)

    y_new = y + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

    y_new[6:10] = quaternion_normalize(y_new[6:10])

    y_new[13] = max(y_new[13], dry_mass)

    return State.from_vector(
        y_new,
        t + dt,
        sim_config=state.sim_config if state.sim_config is not None else ctx.config,
        dry_mass_kg=state.dry_mass_kg if state.dry_mass_kg is not None else dry_mass,
    )


def integrate(state: State, torque: np.ndarray, dt: float,
              ctx: DynamicsContext, method: str = 'rk4') -> State:
    """
    Integrate the state forward by one timestep.

    Args:
        state: Current state
        torque: Control torque in body frame (N*m)
        dt: Time step (s)
        ctx: DynamicsContext with engine, aero, and physics configuration
        method: Integration method (only 'rk4' is supported)

    Returns:
        New state after integration
    """
    if method != 'rk4':
        raise ValueError(f"Unknown integration method: {method}. Only 'rk4' is supported.")
    return rk4_step(state, torque, dt, ctx=ctx)

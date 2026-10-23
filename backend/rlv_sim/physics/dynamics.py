"""
RLV Phase-I Ascent Simulation - Dynamics Equations

This module implements the equations of motion:
- Rotational dynamics: ω̇ = I⁻¹ (τ − ω × (Iω))
- Translational dynamics: r̈ = F_total / m
- Quaternion kinematics: q̇ = 0.5 * Ω(ω) * q
References and validity limits:
    docs/MATHEMATICAL_REFERENCES.md#1-six-degree-of-freedom-rigid-body-dynamics

The equation form is standard rigid-body mechanics. The current changing-
inertia implementation is not a complete variable-mass angular-momentum
derivation; see the limitation recorded in the reference document.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, NamedTuple

import numpy as np

from rlv_sim.core import constants as C
from rlv_sim.core.frames import quaternion_derivative, quaternion_normalize
from rlv_sim.core.utils import cross3
from rlv_sim.physics.forces import (
    _booster_recovery_aero_scale,
    _compute_force_breakdown,
    compute_aerodynamic_moment,
)
from rlv_sim.physics.mass import (
    compute_center_of_mass,
    compute_inertia_tensor,
    compute_mass_derivative,
)

if TYPE_CHECKING:
    from rlv_sim.config.definition import SimulationConfig


@dataclass(frozen=True)
class DynamicsContext:
    """Configuration/parameters for dynamics computation that remain constant
    across RK4 sub-steps.

    Encapsulates engine, aero, physics toggle, and configuration parameters
    that were previously threaded individually through the integration chain
    (rk4_step -> state_derivative_vector -> compute_state_derivative ->
    compute_linear_acceleration). Reduces the parameter list from ~15
    individual kwargs to a single dataclass, making the API surface cleaner
    and eliminating repetition across the four RK4 sub-step calls.

    ponytail: Fields duplicated in ctx.config (enable_j2, j2_coefficient,
    thrust_scale, isp_scale, wind_offset_mps, stage1_landing_reserve_kg) are
    read from ctx.config instead of being threaded as separate fields.

    Fields:
        thrust_on: Whether thrust is active
        throttle: Throttle setting (0.0 to 1.0)
        dry_mass: Dry mass limit for propellant exhaustion
        stage: Engine stage (1 = S1 engines, 2 = S2 engine)
        vehicle_model: Vehicle model identifier ("stacked", "orbiter", "booster")
        booster_aero_mode: Booster recovery aero phase name or None
        thrust_magnitude_override: Override thrust magnitude for recovery burns
        grid_fin_command: Grid fin command state for booster recovery
        config: Full SimulationConfig reference
    """
    thrust_on: bool = True
    throttle: float = 1.0
    dry_mass: float | None = None
    stage: int = 1
    vehicle_model: str = "stacked"
    booster_aero_mode: str | None = None
    thrust_magnitude_override: float | None = None
    grid_fin_command: object | None = None
    config: SimulationConfig | None = None


class StateDerivative(NamedTuple):
    """Container for state derivatives."""
    r_dot: np.ndarray
    v_dot: np.ndarray
    q_dot: np.ndarray
    omega_dot: np.ndarray
    m_dot: float


def compute_angular_acceleration(omega: np.ndarray, torque: np.ndarray,
                                 I_tensor: np.ndarray, I_inv: np.ndarray,
                                 I_dot: np.ndarray | None = None,
                                 jet_damping_torque: np.ndarray | None = None) -> np.ndarray:
    """
    Compute angular acceleration from Euler's equation with VARIABLE INERTIA.

    ω̇ = I⁻¹ (τ − İω − ω × (Iω) − τ_damping)

    Args:
        omega: Angular velocity in body frame (rad/s)
        torque: Total torque (Control + Aero) in body frame (N*m)
        I_tensor: Current Inertia Tensor (kg*m^2)
        I_inv: Inverse Inertia Tensor
        I_dot: Time derivative of Inertia Tensor (kg*m^2/s)
        jet_damping_torque: Jet damping Coriolis torque (N*m)

    Returns:
        Angular acceleration in body frame (rad/s²)
    """

    I_omega = I_tensor @ omega
    gyroscopic = cross3(omega, I_omega)

    total_torque = torque - gyroscopic

    if I_dot is not None:
        total_torque -= I_dot @ omega

    if jet_damping_torque is not None:
        total_torque -= jet_damping_torque

    return I_inv @ total_torque


def _active_engine_count(ctx: DynamicsContext) -> int:
    """Engines firing in the current burn (centre engine is always one of them)."""
    cfg = ctx.config
    if ctx.vehicle_model == "orbiter" or ctx.stage == 2:
        return 1
    if ctx.vehicle_model == "booster":
        mode = (ctx.booster_aero_mode or "").upper()
        if mode == "BOOSTBACK":
            return int(cfg.boostback_engine_count if cfg is not None else C.BOOSTBACK_ENGINE_COUNT)
        if mode == "ENTRY":
            return int(cfg.entry_engine_count if cfg is not None else C.ENTRY_ENGINE_COUNT)
        if mode == "LANDING":
            return int(cfg.landing_engine_count if cfg is not None else C.LANDING_ENGINE_COUNT)
    return int(cfg.stage1_engine_count if cfg is not None else C.STAGE1_ENGINE_COUNT)


def compute_jet_damping_torque(m_dot: float, cg_pos_z: float, omega: np.ndarray,
                               ctx: DynamicsContext) -> np.ndarray:
    """Per-axis jet damping torque J*omega (subtracted in the Euler equation).

    The exhaust leaving each nozzle exit p_i (about the CG) carries angular
    momentum mdot_i [(p_i.p_i)E - p_i p_i^T] omega, plus the exit disk's own
    spread. For a symmetric cluster the off-diagonal terms cancel, leaving
      pitch/yaw: mdot (l^2 + f R^2/2 + r_e^2/4),   l = CG height above exit
      roll:      mdot (f R^2 + r_e^2/2),
    where f is the fraction of flow from ring engines at radius R. Roll
    damping therefore depends on the engines' radial offsets, not on l; using
    l^2 for roll overstated it ~100-750x (docs/reports, jet damping section).
    Pairs with the explicit -Idot*omega term; do not also use mdot(l^2-k^2).
    """
    flow = -float(m_dot)
    if flow <= 0.0:
        return np.zeros(3)
    cfg = ctx.config
    if ctx.vehicle_model == "orbiter" or ctx.stage == 2:
        ring_fraction, ring_radius = 0.0, 0.0
        exit_radius = (cfg.stage2_nozzle_exit_radius_m if cfg is not None
                       else C.STAGE2_NOZZLE_EXIT_RADIUS_M)
    else:
        n = max(_active_engine_count(ctx), 1)
        ring_fraction = (n - 1) / n
        if cfg is not None:
            ring_radius = cfg.stage1_engine_ring_radius_m
            exit_radius = cfg.stage1_nozzle_exit_radius_m
        else:
            ring_radius = C.STAGE1_ENGINE_RING_RADIUS_M
            exit_radius = C.STAGE1_NOZZLE_EXIT_RADIUS_M
    ring_sq = ring_fraction * ring_radius ** 2
    c_transverse = flow * (cg_pos_z ** 2 + 0.5 * ring_sq + 0.25 * exit_radius ** 2)
    c_roll = flow * (ring_sq + 0.5 * exit_radius ** 2)
    return np.array([c_transverse * omega[0], c_transverse * omega[1], c_roll * omega[2]])


def compute_linear_acceleration(r: np.ndarray, v: np.ndarray, q: np.ndarray,
                                m: float,
                                ctx: DynamicsContext,
                                torque: np.ndarray | None = None) -> np.ndarray:
    """
    Compute linear acceleration from Newton's second law (ECI frame).

    r̈ = (F_grav + F_thrust + F_drag + F_lift) / m

    No Coriolis force: ECI is an inertial frame. Earth rotation effects on
    aerodynamic forces are captured via air-relative velocity (v - omega_E x r).

    Delegates force computation to _compute_force_breakdown so the same force
    model is used for both integration and post-integration force logging,
    eliminating the previously duplicated force computation path.

    Args:
        r: Position in ECI frame (m)
        v: Velocity in ECI frame (m/s)
        q: Orientation quaternion [w, x, y, z]
        m: Vehicle mass (kg)
        ctx: DynamicsContext with engine, aero, and physics configuration
        torque: Commanded body torque (N*m), same value used for the
            rotational EOM below. Its transverse (x,y) components are used to
            derive the TVC gimbal deflection so the thrust force carries the
            physically consistent lateral component and cosine loss instead
            of always pointing along body +Z.
    """

    _cfg = ctx.config
    force_breakdown = _compute_force_breakdown(
        r, v, q, m,
        thrust_on=ctx.thrust_on, throttle=ctx.throttle, stage=ctx.stage,
        vehicle_model=ctx.vehicle_model,
        booster_aero_mode=ctx.booster_aero_mode,
        thrust_magnitude_override=ctx.thrust_magnitude_override,
        enable_j2=_cfg.enable_j2 if _cfg is not None else False,
        j2_coefficient=_cfg.j2_coefficient if _cfg is not None else 1.08263e-3,
        gravity_model=_cfg.gravity_model if _cfg is not None else "central",
        thrust_scale=_cfg.runtime_thrust_scale if _cfg is not None else 1.0,
        wind_offset_mps=_cfg.runtime_wind_offset_mps if _cfg is not None else 0.0,
        grid_fin_command=ctx.grid_fin_command,
        config=_cfg,
        control_torque_xy=torque[:2] if torque is not None else None,
    )

    if m < C.ZERO_TOLERANCE:
        return np.zeros(3)

    return force_breakdown['total'] / m


def compute_state_derivative(r: np.ndarray, v: np.ndarray, q: np.ndarray,
                             omega: np.ndarray, m: float, torque: np.ndarray,
                             ctx: DynamicsContext) -> StateDerivative:
    """
    Compute all state derivatives for the full dynamics.

    Includes:
    - Variable Inertia
    - Aerodynamic Moments (Instability)
    - Throttling

    Args:
        r: Position vector (ECI, m)
        v: Velocity vector (ECI, m/s)
        q: Orientation quaternion [w, x, y, z]
        omega: Angular velocity (body frame, rad/s)
        m: Total mass (kg)
        torque: Total body torque (N*m)
        ctx: DynamicsContext with engine, aero, and physics configuration
    """

    dry_mass = ctx.dry_mass
    if dry_mass is None:
        dry_mass = C.STAGE2_DRY_MASS if ctx.stage == 2 else C.DRY_MASS

    _cfg = ctx.config
    _stage1_reserve = _cfg.stage1_landing_fuel_reserve_kg if _cfg is not None else None
    I_tensor = compute_inertia_tensor(
        m,
        vehicle_model=ctx.vehicle_model,
        stage1_landing_reserve_kg=_stage1_reserve,
        config=_cfg,
    )

    try:
        I_inv = np.linalg.inv(I_tensor)
    except np.linalg.LinAlgError:

        diag = np.diag(I_tensor)
        I_inv = np.diag(1.0 / np.where(diag != 0.0, diag, 1.0))

    cg_pos_z = compute_center_of_mass(
        m,
        vehicle_model=ctx.vehicle_model,
        stage1_landing_reserve_kg=_stage1_reserve,
        config=_cfg,
    )
    from rlv_sim.physics.forces import _altitude_from_r
    altitude_m = _altitude_from_r(r, ctx.config)
    aero_enabled = (
        bool(getattr(_cfg, "enable_atmosphere", True))
        and (
            bool(getattr(_cfg, "enable_drag", True))
            or bool(getattr(_cfg, "enable_lift", True))
        )
    )
    if aero_enabled and altitude_m <= C.AERO_DISABLE_ALTITUDE:
        tau_aero = compute_aerodynamic_moment(
            r, v, q,
            cg_pos_z,
            cp_pos_z=C.H_CP,
            wind_offset_mps=_cfg.runtime_wind_offset_mps if _cfg is not None else 0.0,
            omega=omega,
            config=ctx.config,
        )
        if ctx.vehicle_model == "booster":
            _, moment_scale = _booster_recovery_aero_scale(ctx.booster_aero_mode, config=ctx.config)
            tau_aero = moment_scale * tau_aero
    else:
        tau_aero = np.zeros(3)

    m_dot = compute_mass_derivative(
        m,
        ctx.thrust_on, ctx.throttle,
        dry_mass=dry_mass,
        stage=ctx.stage,
        thrust_magnitude_override=ctx.thrust_magnitude_override,
        thrust_scale=_cfg.runtime_thrust_scale if _cfg is not None else 1.0,
        isp_scale=_cfg.runtime_isp_scale if _cfg is not None else 1.0,
        r=r,
        stage2_thrust_vac=_cfg.stage2_thrust_vac if _cfg is not None else None,
        stage2_isp_vac=_cfg.stage2_isp_vac if _cfg is not None else None,
    )

    from rlv_sim.physics.mass import compute_inertia_derivative
    I_dot = compute_inertia_derivative(
        m,
        m_dot,
        vehicle_model=ctx.vehicle_model,
        stage1_landing_reserve_kg=_stage1_reserve,
        config=_cfg,
    )

    jet_damping_torque = compute_jet_damping_torque(m_dot, cg_pos_z, omega, ctx)

    omega_dot = compute_angular_acceleration(omega, torque + tau_aero, I_tensor, I_inv, I_dot, jet_damping_torque)
    r_dot = v
    v_dot = compute_linear_acceleration(r, v, q, m, ctx, torque)
    q_dot = quaternion_derivative(q, omega)

    return StateDerivative(r_dot, v_dot, q_dot, omega_dot, m_dot)


def state_derivative_vector(state_vec: np.ndarray, t: float,
                           torque: np.ndarray,
                           ctx: DynamicsContext) -> np.ndarray:
    """
    Compute state derivative as a flat vector for numerical integration.

    Args:
        state_vec: Flat state vector [r(3), v(3), q(4), omega(3), m(1)]
        t: Current simulation time
        torque: Control torque in body frame (N*m)
        ctx: DynamicsContext with engine, aero, and physics configuration
    """

    r = state_vec[0:3]
    v = state_vec[3:6]
    q = state_vec[6:10]
    omega = state_vec[10:13]
    m = state_vec[13]

    q = quaternion_normalize(q)

    derivs = compute_state_derivative(r, v, q, omega, m, torque, ctx)

    return np.concatenate([
        derivs.r_dot,
        derivs.v_dot,
        derivs.q_dot,
        derivs.omega_dot,
        [derivs.m_dot]
    ])

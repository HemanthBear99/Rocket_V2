"""
RLV Phase-I Ascent Simulation - Mass and inertia computations.

References and approximation notes:
    docs/MATHEMATICAL_REFERENCES.md#6-propulsion-and-variable-mass

The mass-flow equation and elementary component inertias are standard. Vehicle
component locations and geometric idealizations are project assumptions unless
separately identified as measured or published data.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from rlv_sim.core import constants as C

if TYPE_CHECKING:
    from rlv_sim.config.definition import SimulationConfig

S1_DRY_CG_Z = 24.0
S1_PROP_CG_Z = 19.0
S2_DRY_CG_Z = 59.0
S2_PROP_CG_Z = 56.0
# Payload sits above the stage-2 tank (project estimate, stack-base frame).
PAYLOAD_CG_Z = 65.0


def _vehicle_masses(config) -> tuple[float, float, float, float, float]:
    """(S1 dry, S1 propellant, S2 dry, S2 propellant, payload) in kg.

    From the configured vehicle when a config is given, else the reference
    constants (which carry no payload).
    """
    if config is None:
        return (C.STAGE1_DRY_MASS, C.STAGE1_PROPELLANT_MASS,
                C.STAGE2_DRY_MASS, C.STAGE2_PROPELLANT_MASS, 0.0)
    return (float(config.stage1_dry_mass), float(config.stage1_prop_mass),
            float(config.stage2_dry_mass), float(config.stage2_prop_mass),
            float(config.payload_mass))


def _stacked_component_masses(m: float, config=None) -> tuple[float, float, float, float, float]:
    """S1 dry, S1 prop, S2 dry, S2 prop, payload for the stacked vehicle.

    Only stage-1 propellant burns on the stack; payload is its own component
    (it was once counted as stage-1 propellant).
    """
    s1_dry, s1_prop_full, s2_dry, s2_prop, payload = _vehicle_masses(config)
    fixed = s1_dry + s2_dry + s2_prop + payload
    s1_prop = float(np.clip(m - fixed, 0.0, s1_prop_full))
    return s1_dry, s1_prop, s2_dry, s2_prop, payload


def _orbiter_propellant_fraction(m: float, config=None) -> float:
    _, _, s2_dry, s2_prop, payload = _vehicle_masses(config)
    return max(0.0, min(1.0, (m - s2_dry - payload) / max(s2_prop, 1.0)))


def _booster_reserve_fraction(m: float, stage1_landing_reserve_kg, config=None) -> tuple[float, float]:
    """(fraction of the recovery reserve left, reserve kg) for the booster."""
    if stage1_landing_reserve_kg is not None:
        reserve_source = stage1_landing_reserve_kg
    elif config is not None:
        reserve_source = config.stage1_landing_fuel_reserve_kg
    else:
        reserve_source = C.STAGE1_LANDING_FUEL_RESERVE
    reserve = max(float(reserve_source), 1.0)
    s1_dry = _vehicle_masses(config)[0]
    return max(0.0, min(1.0, (m - s1_dry) / reserve)), reserve


def compute_mass_flow_rate(
    thrust_on: bool = True,
    throttle: float = 1.0,
    stage: int = 1,
    thrust_magnitude_override: float | None = None,
    thrust_scale: float = 1.0,
    isp_scale: float = 1.0,
    r: np.ndarray = None,
    stage2_thrust_vac: float | None = None,
    stage2_isp_vac: float | None = None,
    stage1_thrust_n: float | None = None,
) -> float:
    """
    Compute mass flow rate (negative while propellant is consumed).

    Stage-2 mdot is always T/(Isp*g0) from the same thrust/Isp the force model
    uses, so effective Isp cannot drift from the configured vacuum Isp.
    """
    if not thrust_on or throttle <= 0.0:
        return 0.0

    if stage == 2:
        thrust = (
            float(thrust_magnitude_override)
            if thrust_magnitude_override is not None
            else float(stage2_thrust_vac if stage2_thrust_vac is not None else C.STAGE2_THRUST)
        )
        isp = float(stage2_isp_vac if stage2_isp_vac is not None else C.STAGE2_ISP_VAC)
        mdot = thrust / max(isp * C.G0, 1e-9)
    elif thrust_magnitude_override is not None:
        override_scale = float(thrust_magnitude_override) / max(C.THRUST_MAGNITUDE, 1e-9)
        mdot = C.MASS_FLOW_RATE * override_scale
    elif stage1_thrust_n is not None:
        # Same Isp as the reference flow, so configured thrust costs propellant.
        mdot = float(stage1_thrust_n) / (C.ISP * C.G0)
    else:
        mdot = C.MASS_FLOW_RATE

    mdot *= float(thrust_scale) / max(float(isp_scale), 1e-9)
    return -mdot * float(np.clip(throttle, 0.0, 1.0))


def compute_mass_derivative(
    m: float,
    thrust_on: bool = True,
    throttle: float = 1.0,
    dry_mass: float | None = None,
    stage: int = 1,
    thrust_magnitude_override: float | None = None,
    thrust_scale: float = 1.0,
    isp_scale: float = 1.0,
    r: np.ndarray = None,
    stage2_thrust_vac: float | None = None,
    stage2_isp_vac: float | None = None,
    stage1_thrust_n: float | None = None,
    config=None,
) -> float:
    """
    Compute dm/dt with dry-mass floor enforcement.
    """
    if dry_mass is None:
        dry_mass = default_dry_mass(stage, config)

    propellant_remaining = m - dry_mass
    if propellant_remaining <= 0.0 or not thrust_on or throttle <= 0.0:
        return 0.0
    return compute_mass_flow_rate(
        thrust_on=True,
        throttle=throttle,
        stage=stage,
        thrust_magnitude_override=thrust_magnitude_override,
        thrust_scale=thrust_scale,
        isp_scale=isp_scale,
        r=r,
        stage2_thrust_vac=stage2_thrust_vac,
        stage2_isp_vac=stage2_isp_vac,
        stage1_thrust_n=stage1_thrust_n,
    )


def default_dry_mass(stage: int, config=None) -> float:
    """Mass floor when a caller gives none: stage-1 burnout (stage 2 still
    full) for stage 1, stage 2 empty for stage 2; payload stays aboard."""
    s1_dry, _, s2_dry, s2_prop, payload = _vehicle_masses(config)
    if stage == 2:
        return s2_dry + payload
    return s1_dry + s2_dry + s2_prop + payload


def is_propellant_exhausted(m: float, dry_mass: float = C.DRY_MASS) -> bool:
    """True if current mass is at/below dry mass."""
    return m <= dry_mass


class _ReferenceMassProperties:
    """Reference-vehicle mass properties for callers without a config."""

    stack_ixx_full_kgm2 = C.IXX_FULL
    stack_ixx_empty_kgm2 = C.IXX_EMPTY
    stack_izz_full_kgm2 = C.IZZ_FULL
    stack_izz_empty_kgm2 = C.IZZ_EMPTY
    booster_cg_full_m = C.STAGE1_RECOVERY_CG_FULL
    booster_cg_empty_m = C.STAGE1_RECOVERY_CG_EMPTY
    booster_ixx_full_kgm2 = C.STAGE1_RECOVERY_IXX_FULL
    booster_ixx_empty_kgm2 = C.STAGE1_RECOVERY_IXX_EMPTY
    booster_izz_full_kgm2 = C.STAGE1_RECOVERY_IZZ_FULL
    booster_izz_empty_kgm2 = C.STAGE1_RECOVERY_IZZ_EMPTY


def _mass_properties(config):
    """The config's geometry fields, or the reference vehicle's."""
    return config if config is not None else _ReferenceMassProperties


def compute_center_of_mass(
    m: float,
    vehicle_model: str = "stacked",
    stage1_landing_reserve_kg: float | None = None,
    config: SimulationConfig | None = None,
) -> float:
    """
    Compute center-of-mass location (m from vehicle base).

    vehicle_model:
      - "stacked": S1+S2 ascent model
      - "orbiter": post-separation S2 model
      - "booster": post-separation S1 recovery model
    """
    model = (vehicle_model or "stacked").lower()

    if model == "orbiter":
        # Same component stations as the stack, so the CG is continuous at
        # separation and moves the right way as stage-2 propellant burns.
        _, _, s2_dry, s2_prop_full, payload = _vehicle_masses(config)
        s2_prop = _orbiter_propellant_fraction(m, config) * s2_prop_full
        masses = np.array([s2_dry, s2_prop, payload], dtype=float)
        stations = np.array([S2_DRY_CG_Z, S2_PROP_CG_Z, PAYLOAD_CG_Z], dtype=float)
        return float(np.dot(masses, stations) / max(float(np.sum(masses)), 1.0))

    if model == "booster":
        s1_frac, _ = _booster_reserve_fraction(m, stage1_landing_reserve_kg, config)
        p = _mass_properties(config)
        return p.booster_cg_empty_m + s1_frac * (p.booster_cg_full_m - p.booster_cg_empty_m)

    masses = np.array(_stacked_component_masses(m, config), dtype=float)
    stations = np.array([S1_DRY_CG_Z, S1_PROP_CG_Z, S2_DRY_CG_Z, S2_PROP_CG_Z, PAYLOAD_CG_Z], dtype=float)
    total = max(float(np.sum(masses)), 1.0)
    return float(np.dot(masses, stations) / total)


def compute_inertia_tensor(
    m: float,
    vehicle_model: str = "stacked",
    stage1_landing_reserve_kg: float | None = None,
    config: SimulationConfig | None = None,
) -> np.ndarray:
    """
    Compute principal inertia tensor diag([Ixx, Iyy, Izz]) in body frame.

    vehicle_model:
      - "stacked": S1+S2 ascent interpolation between full and empty reference
      - "orbiter": post-separation S2 interpolation
      - "booster": post-separation S1 recovery interpolation
    """
    model = (vehicle_model or "stacked").lower()

    if model == "orbiter":
        s2_frac = _orbiter_propellant_fraction(m, config)
        ixx = 1.67e5 + s2_frac * (2.5e6 - 1.67e5)
        izz = 1.0e5 + s2_frac * (4.0e5 - 1.0e5)
        return np.diag([ixx, ixx, izz])

    if model == "booster":
        s1_frac, _ = _booster_reserve_fraction(m, stage1_landing_reserve_kg, config)
        p = _mass_properties(config)
        ixx = p.booster_ixx_empty_kgm2 + s1_frac * (p.booster_ixx_full_kgm2 - p.booster_ixx_empty_kgm2)
        izz = p.booster_izz_empty_kgm2 + s1_frac * (p.booster_izz_full_kgm2 - p.booster_izz_empty_kgm2)
        return np.diag([ixx, ixx, izz])

    # Default / "stacked" (S1+S2 ascent) branch, reached via guard-clause
    # early returns above for any vehicle_model other than
    # "orbiter"/"booster" (normally "stacked"). Previously this code sat
    # here with several stray blank lines and no comment marking it as a
    # deliberate fallthrough target, making it easy to misread as dead or
    # orphaned code, or to carelessly insert a new `if model == ...:`
    # branch above it that returns early and unintentionally shadows this
    # default case. Comment made explicit; behavior is unchanged.
    s1_prop = _stacked_component_masses(m, config)[1]
    prop_frac = max(0.0, min(1.0, s1_prop / max(_vehicle_masses(config)[1], 1.0)))

    p = _mass_properties(config)
    izz = p.stack_izz_empty_kgm2 + prop_frac * (p.stack_izz_full_kgm2 - p.stack_izz_empty_kgm2)

    # Linear in propellant mass fraction, same convention as izz above and as
    # the orbiter/booster branches of this function - the standard first-order
    # approximation given no per-component shape/radius data to derive a
    # rigorous parallel-axis result instead.
    ixx = p.stack_ixx_empty_kgm2 + prop_frac * (p.stack_ixx_full_kgm2 - p.stack_ixx_empty_kgm2)
    return np.diag([ixx, ixx, izz])


def compute_inertia_derivative(
    m: float,
    mdot: float,
    vehicle_model: str = "stacked",
    stage1_landing_reserve_kg: float | None = None,
    config: SimulationConfig | None = None,
) -> np.ndarray:
    """
    Compute time derivative of the principal inertia tensor diag([dIxx/dt, dIyy/dt, dIzz/dt]).

    This is directly proportional to mass flow rate (mdot). If mass is not changing,
    the derivative is zero.
    """
    if abs(mdot) < 1e-9:
        return np.zeros((3, 3))

    model = (vehicle_model or "stacked").lower()

    s1_dry, s1_prop_full, s2_dry, s2_prop, payload = _vehicle_masses(config)
    if model == "orbiter":
        if m <= s2_dry + payload or m >= s2_dry + payload + s2_prop:
            return np.zeros((3, 3))
        d_ixx_dm = (2.5e6 - 1.67e5) / s2_prop
        d_izz_dm = (4.0e5 - 1.0e5) / s2_prop
        return np.diag([d_ixx_dm * mdot, d_ixx_dm * mdot, d_izz_dm * mdot])

    if model == "booster":
        _, reserve = _booster_reserve_fraction(m, stage1_landing_reserve_kg, config)
        if m <= s1_dry or m >= s1_dry + reserve:
            return np.zeros((3, 3))
        p = _mass_properties(config)
        d_ixx_dm = (p.booster_ixx_full_kgm2 - p.booster_ixx_empty_kgm2) / reserve
        d_izz_dm = (p.booster_izz_full_kgm2 - p.booster_izz_empty_kgm2) / reserve
        return np.diag([d_ixx_dm * mdot, d_ixx_dm * mdot, d_izz_dm * mdot])

    # Stacked ascent model
    stack_floor = s1_dry + s2_dry + s2_prop + payload
    if m <= stack_floor or m >= stack_floor + s1_prop_full:
        return np.zeros((3, 3))

    p = _mass_properties(config)
    d_ixx_dm = (p.stack_ixx_full_kgm2 - p.stack_ixx_empty_kgm2) / s1_prop_full
    d_izz_dm = (p.stack_izz_full_kgm2 - p.stack_izz_empty_kgm2) / s1_prop_full
    return np.diag([d_ixx_dm * mdot, d_ixx_dm * mdot, d_izz_dm * mdot])

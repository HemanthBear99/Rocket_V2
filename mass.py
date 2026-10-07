"""
RLV Phase-I Ascent Simulation - Mass and inertia computations.

References and approximation notes:
    docs/MATHEMATICAL_REFERENCES.md#6-propulsion-and-variable-mass

The mass-flow equation and elementary component inertias are standard. Vehicle
component locations and geometric idealizations are project assumptions unless
separately identified as measured or published data.
"""

import numpy as np

from . import constants as C

S1_DRY_CG_Z = 24.0
S1_PROP_CG_Z = 19.0
S2_DRY_CG_Z = 59.0
S2_PROP_CG_Z = 56.0


def _stacked_component_masses(m: float) -> tuple[float, float, float, float]:
    """Return S1 dry, S1 prop, S2 dry, S2 prop masses for the stacked vehicle."""
    s1_prop = float(np.clip(m - C.DRY_MASS, 0.0, C.STAGE1_PROPELLANT_MASS))
    return (
        C.STAGE1_DRY_MASS,
        s1_prop,
        C.STAGE2_DRY_MASS,
        C.STAGE2_PROPELLANT_MASS,
    )


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
) -> float:
    """
    Compute dm/dt with dry-mass floor enforcement.
    """
    if dry_mass is None:
        dry_mass = C.STAGE2_DRY_MASS if stage == 2 else C.DRY_MASS

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
    )


def is_propellant_exhausted(m: float, dry_mass: float = C.DRY_MASS) -> bool:
    """True if current mass is at/below dry mass."""
    return m <= dry_mass


def compute_center_of_mass(
    m: float,
    vehicle_model: str = "stacked",
    stage1_landing_reserve_kg: float | None = None,
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
        s2_frac = max(0.0, min(1.0, (m - C.STAGE2_DRY_MASS) / C.STAGE2_PROPELLANT_MASS))
        return 58.0 + s2_frac * 5.0

    if model == "booster":
        reserve_source = (
            C.STAGE1_LANDING_FUEL_RESERVE
            if stage1_landing_reserve_kg is None
            else stage1_landing_reserve_kg
        )
        reserve = max(float(reserve_source), 1.0)
        s1_frac = max(0.0, min(1.0, (m - C.STAGE1_DRY_MASS) / reserve))
        return (
            C.STAGE1_RECOVERY_CG_EMPTY
            + s1_frac * (C.STAGE1_RECOVERY_CG_FULL - C.STAGE1_RECOVERY_CG_EMPTY)
        )

    s1_dry, s1_prop, s2_dry, s2_prop = _stacked_component_masses(m)
    masses = np.array([s1_dry, s1_prop, s2_dry, s2_prop], dtype=float)
    stations = np.array([S1_DRY_CG_Z, S1_PROP_CG_Z, S2_DRY_CG_Z, S2_PROP_CG_Z], dtype=float)
    total = max(float(np.sum(masses)), 1.0)
    return float(np.dot(masses, stations) / total)


def compute_inertia_tensor(
    m: float,
    vehicle_model: str = "stacked",
    stage1_landing_reserve_kg: float | None = None,
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
        s2_frac = max(0.0, min(1.0, (m - C.STAGE2_DRY_MASS) / C.STAGE2_PROPELLANT_MASS))
        ixx = 1.67e5 + s2_frac * (2.5e6 - 1.67e5)
        izz = 1.0e5 + s2_frac * (4.0e5 - 1.0e5)
        return np.diag([ixx, ixx, izz])

    if model == "booster":
        reserve_source = (
            C.STAGE1_LANDING_FUEL_RESERVE
            if stage1_landing_reserve_kg is None
            else stage1_landing_reserve_kg
        )
        reserve = max(float(reserve_source), 1.0)
        s1_frac = max(0.0, min(1.0, (m - C.STAGE1_DRY_MASS) / reserve))
        ixx = (
            C.STAGE1_RECOVERY_IXX_EMPTY
            + s1_frac * (C.STAGE1_RECOVERY_IXX_FULL - C.STAGE1_RECOVERY_IXX_EMPTY)
        )
        izz = (
            C.STAGE1_RECOVERY_IZZ_EMPTY
            + s1_frac * (C.STAGE1_RECOVERY_IZZ_FULL - C.STAGE1_RECOVERY_IZZ_EMPTY)
        )
        return np.diag([ixx, ixx, izz])

    # Default / "stacked" (S1+S2 ascent) branch, reached via guard-clause
    # early returns above for any vehicle_model other than
    # "orbiter"/"booster" (normally "stacked"). Previously this code sat
    # here with several stray blank lines and no comment marking it as a
    # deliberate fallthrough target, making it easy to misread as dead or
    # orphaned code, or to carelessly insert a new `if model == ...:`
    # branch above it that returns early and unintentionally shadows this
    # default case. Comment made explicit; behavior is unchanged.
    s1_prop = float(np.clip(m - C.DRY_MASS, 0.0, C.STAGE1_PROPELLANT_MASS))
    prop_frac = max(0.0, min(1.0, s1_prop / C.STAGE1_PROPELLANT_MASS))

    izz = C.IZZ_EMPTY + prop_frac * (C.IZZ_FULL - C.IZZ_EMPTY)

    # Linear in propellant mass fraction, same convention as izz above and as
    # the orbiter/booster branches of this function - the standard first-order
    # approximation given no per-component shape/radius data to derive a
    # rigorous parallel-axis result instead.
    ixx = C.IXX_EMPTY + prop_frac * (C.IXX_FULL - C.IXX_EMPTY)
    return np.diag([ixx, ixx, izz])


def compute_inertia_derivative(
    m: float,
    mdot: float,
    vehicle_model: str = "stacked",
    stage1_landing_reserve_kg: float | None = None,
) -> np.ndarray:
    """
    Compute time derivative of the principal inertia tensor diag([dIxx/dt, dIyy/dt, dIzz/dt]).

    This is directly proportional to mass flow rate (mdot). If mass is not changing,
    the derivative is zero.
    """
    if abs(mdot) < 1e-9:
        return np.zeros((3, 3))

    model = (vehicle_model or "stacked").lower()

    if model == "orbiter":
        if m <= C.STAGE2_DRY_MASS or m >= C.STAGE2_DRY_MASS + C.STAGE2_PROPELLANT_MASS:
            return np.zeros((3, 3))
        d_ixx_dm = (2.5e6 - 1.67e5) / C.STAGE2_PROPELLANT_MASS
        d_izz_dm = (4.0e5 - 1.0e5) / C.STAGE2_PROPELLANT_MASS
        return np.diag([d_ixx_dm * mdot, d_ixx_dm * mdot, d_izz_dm * mdot])

    if model == "booster":
        reserve_source = (
            C.STAGE1_LANDING_FUEL_RESERVE
            if stage1_landing_reserve_kg is None
            else stage1_landing_reserve_kg
        )
        reserve = max(float(reserve_source), 1.0)
        if m <= C.STAGE1_DRY_MASS or m >= C.STAGE1_DRY_MASS + reserve:
            return np.zeros((3, 3))
        d_ixx_dm = (C.STAGE1_RECOVERY_IXX_FULL - C.STAGE1_RECOVERY_IXX_EMPTY) / reserve
        d_izz_dm = (C.STAGE1_RECOVERY_IZZ_FULL - C.STAGE1_RECOVERY_IZZ_EMPTY) / reserve
        return np.diag([d_ixx_dm * mdot, d_ixx_dm * mdot, d_izz_dm * mdot])

    # Stacked ascent model
    if m <= C.DRY_MASS or m >= C.DRY_MASS + C.STAGE1_PROPELLANT_MASS:
        return np.zeros((3, 3))

    d_ixx_dm = (C.IXX_FULL - C.IXX_EMPTY) / C.STAGE1_PROPELLANT_MASS
    d_izz_dm = (C.IZZ_FULL - C.IZZ_EMPTY) / C.STAGE1_PROPELLANT_MASS
    return np.diag([d_ixx_dm * mdot, d_ixx_dm * mdot, d_izz_dm * mdot])

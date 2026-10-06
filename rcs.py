"""Reaction Control System (RCS) model with propellant tracking.

RCS provides attitude control authority when TVC is unavailable (coast,
zero-thrust phases) or supplements TVC during powered flight.
"""

from __future__ import annotations

from dataclasses import dataclass

from . import constants as C
from .config_definition import SimulationConfig


@dataclass
class RCSState:
    """Mutable RCS propellant and usage state."""

    propellant_remaining_kg: float = C.RCS_PROPELLANT_MASS
    total_impulse_used_ns: float = 0.0
    total_firing_time_s: float = 0.0
    exhausted: bool = False

    @property
    def propellant_fraction(self) -> float:
        """Fraction of RCS propellant remaining."""
        return max(0.0, self.propellant_remaining_kg / C.RCS_PROPELLANT_MASS)


def compute_rcs_mass_flow_rate(
    rcs_torque_n_m: float,
    lever_arm_m: float = C.RCS_LEVER_ARM,
    isp_s: float = C.RCS_ISP,
    num_thrusters: int = 8,
    thrust_per_thruster: float = C.RCS_THRUST_PER_THRUSTER,
) -> float:
    """Compute RCS propellant mass flow rate from RCS-demanded torque.

    The RCS produces torque via thruster pairs:
        torque = F_thruster * lever_arm * num_active_pairs

    Mass flow for each thruster:
        mdot = F_thruster / (Isp * g0)

    Total mass flow scales with the fraction of max RCS torque demanded.

    Sign convention: returns a positive consumption rate (kg/s), unlike
    mass.compute_mass_derivative's signed dm/dt. This is intentional, not an
    inconsistency - the two are used differently: this value is subtracted
    directly from an RCS propellant budget each step (update_rcs_propellant
    below), while mass.py's dm/dt is a true ODE derivative integrated by the
    RK4 state integrator.

    Args:
        rcs_torque_n_m: Torque that RCS must provide (not total vehicle torque).
            For TVC-controlled vehicles, this is typically just the roll torque.
    """
    if rcs_torque_n_m <= 0.0 or lever_arm_m <= 0.0 or isp_s <= 0.0:
        return 0.0

    max_torque = float(thrust_per_thruster) * int(num_thrusters) * lever_arm_m
    if max_torque <= 0.0:
        return 0.0

                                          
    throttle = min(rcs_torque_n_m / max_torque, 1.0)

    # Normalize full-authority flow against the SAME thrust value used for
    # max_torque above. Previously this used the hard-coded
    # C.RCS_THRUST_PER_THRUSTER while rcs_available_torque() used
    # config.rcs_thrust, so retuning rcs_thrust desynchronized the throttle
    # fraction from real authority (rcs_thrust=1000 gave throttle 0.75 of the
    # full 8-thruster flow for a 60 kN.m demand instead of 0.375 -- 2x
    # over-consumption), and demands above the hard-coded authority were
    # silently clamped and under-consumed.

                                      
    mdot_full = (
        float(thrust_per_thruster)
        * int(num_thrusters)
        / (isp_s * C.G0)
    )

    return mdot_full * throttle


def update_rcs_propellant(
    rcs_state: RCSState,
    rcs_torque_n_m: float,
    dt: float,
    config: SimulationConfig,
) -> RCSState:
    """Advance RCS propellant state based on RCS torque demand."""
    if rcs_state.exhausted or rcs_state.propellant_remaining_kg <= 0.0:
        rcs_state.exhausted = True
        rcs_state.propellant_remaining_kg = 0.0
        return rcs_state

    mdot = compute_rcs_mass_flow_rate(
        rcs_torque_n_m,
        lever_arm_m=C.RCS_LEVER_ARM,
        isp_s=float(config.rcs_isp),
        num_thrusters=int(config.rcs_num_thrusters),
        thrust_per_thruster=float(config.rcs_thrust),
    )

    consumed = mdot * max(float(dt), 0.0)
    new_remaining = max(0.0, rcs_state.propellant_remaining_kg - consumed)

    if consumed > 0.0:
        # Telemetry impulse must reflect the torque the thrusters ACTUALLY
        # produced (already clamped to authority inside the flow-rate
        # calculation) over the same non-negative dt used for propellant, not
        # the raw unclamped demand -- otherwise total_impulse_used_ns exceeds
        # Isp*g0*(propellant actually used) whenever demand exceeds authority,
        # and the two accounting paths disagree for negative dt.
        max_authority = (
            float(config.rcs_thrust)
            * max(int(config.rcs_num_thrusters), 0)
            * C.RCS_LEVER_ARM
        )
        applied_torque = min(max(float(rcs_torque_n_m), 0.0), max_authority)
        step_dt = max(float(dt), 0.0)
        impulse = applied_torque * step_dt / C.RCS_LEVER_ARM
        rcs_state.total_impulse_used_ns += float(impulse)
        rcs_state.total_firing_time_s += float(step_dt)

    rcs_state.propellant_remaining_kg = new_remaining
    rcs_state.exhausted = new_remaining <= 0.0
    return rcs_state


def rcs_available_torque(
    rcs_state: RCSState,
    config: SimulationConfig,
) -> float:
    """Maximum torque RCS can produce given remaining propellant."""
    if rcs_state.exhausted or rcs_state.propellant_remaining_kg <= 0.0:
        return 0.0
    return (
        float(config.rcs_thrust)
        * max(int(config.rcs_num_thrusters), 0)
        * C.RCS_LEVER_ARM
    )

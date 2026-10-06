"""
RLV Phase-I Ascent Simulation - Attitude Control

This module implements the attitude control system:
- Direction to quaternion conversion
- Quaternion error computation
- Selectable PD / PID control laws
- Torque saturation and integral anti-windup
References and tuning notes:
    docs/MATHEMATICAL_REFERENCES.md#1-six-degree-of-freedom-rigid-body-dynamics
    docs/MATHEMATICAL_REFERENCES.md#5-aerodynamic-forces-and-ascent-guidance

Quaternion feedback is standard; gains, deadbands, saturation limits, and
phase scheduling are project-specific controller tuning.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal

import numpy as np

from . import constants as C
from ._types import ControlOutput
from .frames import (
    direction_to_quaternion,
    quaternion_normalize,
    quaternion_to_rotation_matrix,
)
from .utils import cross3, vec_norm

if TYPE_CHECKING:
    from .config_definition import SimulationConfig
    from .mission_manager import MissionPhase

AttitudeControllerName = Literal["pd", "pid"]


def compute_commanded_quaternion(desired_direction: np.ndarray) -> np.ndarray:
    """
    Convert desired thrust direction to commanded orientation quaternion.
    
    The body +Z axis should align with the desired thrust direction.
    
    Args:
        desired_direction: Desired thrust direction (inertial frame)
        
    Returns:
        Commanded quaternion [w, x, y, z]
    """
                                                  
                                                                               
    return direction_to_quaternion(desired_direction, C.BODY_Z_AXIS)


def compute_thrust_axis_error(q_current: np.ndarray, desired_direction: np.ndarray) -> tuple:
    """
    Compute attitude error for thrust-vector alignment only.

    Guidance commands a desired body +Z direction, not a roll attitude. A full
    quaternion error invents an arbitrary roll target and can command large
    nonphysical roll torques. This error drives only the shortest rotation that
    aligns current body +Z with the desired inertial direction.
    """
    q_current = quaternion_normalize(q_current)
    desired = np.asarray(desired_direction, dtype=float)
    desired_norm = float(vec_norm(desired))
    if desired_norm < C.ZERO_TOLERANCE:
        return np.zeros(3), 0.0
    desired = desired / desired_norm

    R = quaternion_to_rotation_matrix(q_current)
    body_z_inertial = R @ C.BODY_Z_AXIS
    dot = float(np.clip(np.dot(body_z_inertial, desired), -1.0, 1.0))
    angle = float(np.arccos(dot))
    if angle < C.ZERO_TOLERANCE:
        return np.zeros(3), 0.0

    axis_inertial = cross3(body_z_inertial, desired)
    axis_norm = float(vec_norm(axis_inertial))
    if axis_norm < C.ZERO_TOLERANCE:
                                                              
        axis_body = np.array([1.0, 0.0, 0.0])
    else:
        axis_inertial /= axis_norm
        axis_body = R.T @ axis_inertial

    return axis_body * np.sin(0.5 * angle), angle



@dataclass
class ControlState:
    """Per-vehicle attitude-controller state (integral accumulator, phase latch)."""

    integral_error: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    active_controller: AttitudeControllerName = "pd"
    last_phase: str | None = None


def create_control_state() -> ControlState:
    """Create a fresh attitude-control state for a new vehicle path."""
    return ControlState()


def _schedule_gains(
    inertia,
    kp_attitude: float | None,
    kd_attitude: float | None,
    ki_attitude: float | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return per-axis (kp, kd, ki) with inertia-based gain scheduling."""
    inertia_array = np.asarray(inertia, dtype=float)
    use_per_axis = inertia_array.ndim > 0 and len(inertia_array) == 3

    kp_base = C.KP_ATTITUDE if kp_attitude is None else kp_attitude
    kd_base = C.KD_ATTITUDE if kd_attitude is None else kd_attitude
    ki_base = C.KI_ATTITUDE if ki_attitude is None else ki_attitude

    if use_per_axis:
        gain_ratios = np.minimum(inertia_array / C.IXX_FULL, 1.0)
        return kp_base * gain_ratios, kd_base * gain_ratios, ki_base * gain_ratios
    if inertia is not None and float(inertia) > 0.0:
        gain_ratio = min(float(inertia) / C.IXX_FULL, 1.0)
        ratio3 = np.full(3, gain_ratio)
        return kp_base * ratio3, kd_base * ratio3, ki_base * ratio3
    ones = np.ones(3, dtype=float)
    return kp_base * ones, kd_base * ones, ki_base * ones


def resolve_attitude_controller(
    phase: MissionPhase,
    vehicle_model: str,
    config: SimulationConfig,
) -> AttitudeControllerName:
    """Pick the active attitude controller for the current mission segment."""
    from .mission_manager import MissionPhase as MP

    if vehicle_model == "booster" and phase == MP.BOOSTER_LANDING:
        recovery = config.recovery_attitude_controller
        if recovery == "auto":
            return "pid"
        return recovery
    if vehicle_model == "orbiter" and phase == MP.S2_LANDING and config.enable_s2_recovery:
        recovery = config.recovery_attitude_controller
        if recovery == "auto":
            return "pid"
        return recovery
    return config.attitude_controller


def prepare_control_state(
    control_state: ControlState | None,
    controller: AttitudeControllerName,
    phase_name: str,
) -> ControlState:
    """Reset the integral when the controller or landing phase changes."""
    if control_state is None:
        control_state = create_control_state()
    if (
        controller == "pid"
        and (
            control_state.active_controller != "pid"
            or control_state.last_phase != phase_name
        )
    ):
        control_state.integral_error = np.zeros(3, dtype=float)
    control_state.active_controller = controller
    control_state.last_phase = phase_name
    return control_state


def _apply_integral_anti_windup(
    integral_error: np.ndarray,
    ki: np.ndarray,
    max_torque: float,
    windup_fraction: float,
    max_roll_torque: float | None = None,
) -> np.ndarray:
    """Clamp the integral state so Ki·integral stays within the torque budget."""
    limit_transverse = max(float(windup_fraction) * float(max_torque), 0.0)
    if max_roll_torque is not None:
        limit_roll = max(float(windup_fraction) * float(max_roll_torque), 0.0)
    else:
        limit_roll = limit_transverse

                                    
    integral_error = integral_error.copy()
    tau_i = ki * integral_error
    
                                     
    tau_i_trans = tau_i[0:2]
    mag_trans = float(vec_norm(tau_i_trans))
    if limit_transverse > 0.0 and mag_trans > limit_transverse:
        integral_error[0:2] = integral_error[0:2] * (limit_transverse / mag_trans)
    elif limit_transverse <= 0.0:
        integral_error[0:2] = 0.0
        
                   
    tau_i_roll = tau_i[2]
    mag_roll = abs(tau_i_roll)
    if limit_roll > 0.0 and mag_roll > limit_roll:
        integral_error[2] = integral_error[2] * (limit_roll / mag_roll)
    elif limit_roll <= 0.0:
        integral_error[2] = 0.0

    return integral_error



def pd_control_law(q_error_vector: np.ndarray, error_angle: float,
                   omega: np.ndarray, inertia=None,
                   max_torque: float = None,
                   kp_attitude: float = None,
                   kd_attitude: float = None,
                   ki_attitude: float = None,
                   control_state: ControlState | None = None,
                   dt: float = 0.0,
                   controller: AttitudeControllerName = "pd",
                   integral_windup_fraction: float | None = None,
                   max_roll_torque: float | None = None,
                   rcs_only: bool | None = None) -> tuple[np.ndarray, ControlState | None]:
    """
    Implement PD attitude control law.

    τ_cmd = +Kp · q_ev - Kd · ω_e

    where q_ev is the vector part of the thrust-axis error quaternion
    (compute_thrust_axis_error's sign convention - positive q_ev drives torque
    toward closing the error, so the proportional term is +Kp, not -Kp) and
    ω_e is the angular velocity error (= ω since ω_cmd = 0 in Phase I).

    Gain scheduling: When the vehicle inertia is provided, gains are
    scaled to maintain the designed natural frequency and damping ratio
    regardless of vehicle mass:
        Kp_eff = Kp_ref * (I / I_ref)
        Kd_eff = Kd_ref * (I / I_ref)
    With the reference gains in constants.py (KP_ATTITUDE, KD_ATTITUDE) and
    I_ref = IXX_FULL, this gives ωn = sqrt(Kp_ref/I_ref) ≈ 1.49 rad/s and
    ζ = Kd_ref / (2*sqrt(Kp_ref*I_ref)) ≈ 0.50 at full inertia.

    Per-axis scaling: When `inertia` is a 3-element array [Ixx, Iyy, Izz],
    gains are scaled independently per axis. This prevents roll torque
    saturation caused by applying pitch-axis gains to the roll axis
    (which has 23x lower inertia).

    This ensures consistent control response across S1 (I=5.36e7 kg·m²)
    and S2 (I=0.6e6–4.0e6 kg·m²) without over-torquing or sluggishness.

    Args:
        q_error_vector: Vector part of error quaternion [q_e1, q_e2, q_e3]
        error_angle: Magnitude of rotation error (rad) — for logging only
        omega: Current angular velocity in body frame (rad/s)
        inertia: Moment of inertia for gain scheduling.
                 - Scalar (float): Representative I for all axes (legacy)
                 - Array [Ixx, Iyy, Izz]: Per-axis inertia (recommended)
                 - None: Uses reference gains directly
        max_torque: Torque saturation limit (N·m). If None, uses C.MAX_TORQUE.

    Returns:
        (control torque in body frame (N*m), updated control_state)
    """
    kp, kd, ki = _schedule_gains(inertia, kp_attitude, kd_attitude, ki_attitude)

    if max_torque is None:
        max_torque = C.MAX_TORQUE
    if integral_windup_fraction is None:
        integral_windup_fraction = C.ATTITUDE_INTEGRAL_WINDUP_FRACTION

    q_error_vector = np.asarray(q_error_vector, dtype=float)

                                                                              
                                                                                                    
    # RCS-only detection. The deadband exists to stop the RCS thrusters
    # chasing sub-deadband attitude errors it cannot null, wasting propellant.
    # It previously keyed on a hard-coded max_torque < 50000.0 literal, but
    # real RCS authority is rcs_thrust(500) * rcs_num_thrusters(8) *
    # RCS_LEVER_ARM(20) = 80,000 N.m, so vailable_torque in the RCS-only
    # regime was always ABOVE the threshold and the deadband never engaged --
    # it only switched on after RCS propellant dropped below ~62% of budget.
    # The caller now passes the real determination explicitly (rcs_only=...);
    # the literal threshold is retained only as a fallback for direct callers.
    is_rcs_only = (max_torque < C.RCS_FULL_AUTHORITY_NM) if rcs_only is None else bool(rcs_only)
    if is_rcs_only:
        deadband_rad = np.radians(3.5)
        if error_angle < deadband_rad:
            q_error_vector = np.zeros_like(q_error_vector)

                                  
    tau_p = kp * q_error_vector

                                    
                                                                              
                                                                                  
                                                                                           
    tau_d = -kd * omega

    tau_i = np.zeros(3, dtype=float)
    if controller == "pid" and control_state is not None and dt > 0.0:
        integrate = True
        pre_torque = tau_p + tau_d + ki * control_state.integral_error
        pre_mag = float(vec_norm(pre_torque))
        if pre_mag >= max_torque * 0.999:
            integrate = False
        if integrate:
            control_state.integral_error = control_state.integral_error + q_error_vector * dt
            control_state.integral_error = _apply_integral_anti_windup(
                control_state.integral_error,
                ki,
                max_torque,
                integral_windup_fraction,
                max_roll_torque=max_roll_torque,
            )
        tau_i = ki * control_state.integral_error

    torque = tau_p + tau_d + tau_i

                     
    torque_magnitude = vec_norm(torque)
    if torque_magnitude > max_torque:
        torque = torque * (max_torque / torque_magnitude)

    return torque, control_state



def compute_control_output(q_current: np.ndarray, omega: np.ndarray,
                          desired_direction: np.ndarray,
                          inertia: float = None,
                          max_torque: float = None,
                          kp_attitude: float = None,
                          kd_attitude: float = None,
                          ki_attitude: float = None,
                          control_state: ControlState | None = None,
                          dt: float = 0.0,
                          controller: AttitudeControllerName = "pd",
                          integral_windup_fraction: float | None = None,
                          max_roll_torque: float | None = None,
                          rcs_only: bool | None = None) -> ControlOutput:

    """
    Compute full control output for logging and analysis.

    Args:
        q_current: Current orientation quaternion [w, x, y, z]
        omega: Current angular velocity in body frame (rad/s)
        desired_direction: Desired thrust direction (inertial frame)
        inertia: Representative moment of inertia for gain scheduling (kg*m^2)
        max_torque: Torque saturation limit (N*m)

    Returns:
        Dictionary containing control state and commands
    """
    if max_torque is None:
        max_torque = C.MAX_TORQUE

    q_commanded = compute_commanded_quaternion(desired_direction)
    q_error_vector, error_angle = compute_thrust_axis_error(q_current, desired_direction)
    torque, control_state = pd_control_law(
        q_error_vector,
        error_angle,
        omega,
        inertia=inertia,
        max_torque=max_torque,
        kp_attitude=kp_attitude,
        kd_attitude=kd_attitude,
        ki_attitude=ki_attitude,
        control_state=control_state,
        dt=dt,
        controller=controller,
        integral_windup_fraction=integral_windup_fraction,
        max_roll_torque=max_roll_torque,
        rcs_only=rcs_only,
    )
    integral_torque = np.zeros(3, dtype=float)
    if controller == "pid" and control_state is not None:
        _, _, ki = _schedule_gains(inertia, kp_attitude, kd_attitude, ki_attitude)
        integral_torque = ki * control_state.integral_error

    return {
        'q_commanded': q_commanded,
        'error_axis': q_error_vector,
        'error_angle': error_angle,
        'error_degrees': np.degrees(error_angle),
        'torque': torque,
        'torque_magnitude': vec_norm(torque),
        'saturated': vec_norm(torque) >= max_torque * 0.999,
        'attitude_controller': controller,
        'integral_torque': integral_torque,
        'integral_torque_magnitude': float(vec_norm(integral_torque)),
        'control_state': control_state,
    }

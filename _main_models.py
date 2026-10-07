"""Internal data models used by the runtime entry points."""

from __future__ import annotations

import csv
import os
from dataclasses import dataclass
from typing import Any

import numpy as np

from . import constants as C
from ._guidance_common import compute_local_vertical
from .forces import compute_configured_atmosphere_properties
from .frames import rotate_vector_by_quaternion
from .state import State
from .utils import axisymmetric_angle_of_attack, vec_norm


class SimulationLog:
    """Container for logged simulation data (dict-of-lists).

    ``append()`` builds a flat dict from state/guidance/control and appends
    each value to the corresponding list.  ``to_csv()`` auto-generates the
    header from the dict keys.  Fields are accessed as ``log.time`` via
    ``__getattr__`` — no 100+ hardcoded ``List[float]`` fields.
    """

    def __init__(self):
        self._data: dict[str, list] = {}

    def __getattr__(self, name: str) -> Any:

        try:
            return self._data[name]
        except KeyError:
            cls = type(self).__name__
            raise AttributeError(f"'{cls}' object has no attribute '{name}'")

    def get_series(self, name: str) -> list:
        """Return a logged series, or an empty list when it was never emitted."""
        return self._data.get(name, [])


    def append(self, state: State, guidance: dict, control: dict):

        v_rel = np.asarray(guidance.get('v_rel', state.v))
        vertical = compute_local_vertical(state.r)
        v_vert = float(np.dot(v_rel, vertical))
        v_horiz_vec = v_rel - v_vert * vertical
        v_horiz = float(vec_norm(v_horiz_vec))
        v_radial = float(np.dot(state.v, vertical))
        v_horiz_inertial_vec = state.v - v_radial * vertical
        torque_vec = np.asarray(control.get('torque', np.zeros(3)), dtype=float)
        cmd_thrust_dir = guidance.get('thrust_direction', np.array([0.0, 0.0, 1.0]))
        r_norm = float(vec_norm(state.r))

        r0_hat = C.INITIAL_POSITION / vec_norm(C.INITIAL_POSITION)
        r_hat = state.r / r_norm
        central_angle = float(np.arccos(np.clip(np.dot(r_hat, r0_hat), -1.0, 1.0)))

        theta = C.EARTH_ROTATION_RATE * state.t
        x_ecef = state.r[0] * np.cos(theta) + state.r[1] * np.sin(theta)
        y_ecef = -state.r[0] * np.sin(theta) + state.r[1] * np.cos(theta)
        z_ecef = state.r[2]
        r_ecef = np.array([x_ecef, y_ecef, z_ecef])
        r_ecef_norm = vec_norm(r_ecef)
        if r_ecef_norm > C.ZERO_TOLERANCE:
            r0_ecef_hat = C.INITIAL_POSITION / vec_norm(C.INITIAL_POSITION)
            central_angle_ground = float(np.arccos(
                np.clip(np.dot(r_ecef / r_ecef_norm, r0_ecef_hat), -1.0, 1.0)
            ))
        else:
            central_angle_ground = 0.0

        body_z_inertial = rotate_vector_by_quaternion(C.BODY_Z_AXIS, state.q)
        cos_pitch = np.clip(np.dot(vertical, body_z_inertial), -1.0, 1.0)

        gamma_cmd_deg = np.degrees(float(guidance.get('gamma_angle', 0.0)))
        gamma_actual = np.degrees(np.arctan2(v_vert, max(v_horiz, C.ZERO_TOLERANCE)))

        cmd_quat = control.get('q_commanded', np.array([1.0, 0.0, 0.0, 0.0]))

        throttle_val = float(guidance.get('throttle', 1.0))
        thrust_on_val = bool(guidance.get('thrust_on', True))
        phase_str = str(guidance.get('phase', ''))
        is_s2_phase = phase_str in ('ORBIT_INSERTION', 'COAST_PHASE') and state.m < C.STAGE2_WET_MASS * 1.1
        thrust_mag_nominal = float(guidance.get(
            'thrust_magnitude_n',
            C.STAGE2_THRUST if is_s2_phase else C.THRUST_MAGNITUDE,
        ))
        thrust_mag = thrust_mag_nominal if thrust_on_val else 0.0
        actual_thrust_dir = rotate_vector_by_quaternion(C.BODY_Z_AXIS, state.q)
        thrust_vec = actual_thrust_dir * (throttle_val * thrust_mag)

        prograde = guidance.get('local_horizontal', np.array([0.0, 0.0, 1.0]))
        gamma = guidance.get('gamma_angle', np.radians(gamma_actual))

        alt_m = state.altitude
        _, _, rho_atm, a_atm = compute_configured_atmosphere_properties(
            alt_m,
            state.sim_config,
        )
        v_rel_mag = float(vec_norm(v_rel))
        mach = v_rel_mag / a_atm if a_atm > 1.0 else 0.0
        q_dyn = 0.5 * rho_atm * v_rel_mag ** 2

        if v_rel_mag > 10.0:
            aoa_rad = axisymmetric_angle_of_attack(body_z_inertial, v_rel)
        else:
            aoa_rad = 0.0

        r_nose = C.REFERENCE_DIAMETER / 2.0
        if rho_atm > 1e-12 and v_rel_mag > 100.0:
            q_heat = 1.7415e-4 * np.sqrt(rho_atm / max(r_nose, 0.01)) * v_rel_mag ** 3
        else:
            q_heat = 0.0

        lat = float(np.degrees(np.arcsin(np.clip(state.r[2] / r_norm, -1.0, 1.0)))) if r_norm > 1.0 else 0.0
        lon_raw = float(np.degrees(np.arctan2(state.r[1], state.r[0]))) if r_norm > 1.0 else 0.0
        lon_geo = ((lon_raw - np.degrees(C.EARTH_ROTATION_RATE * state.t) + 180.0) % 360.0) - 180.0

        propellant = guidance.get('propellant_remaining_kg')
        if propellant is None:
            dry = guidance.get('dry_mass_kg')
            if dry is None:
                vm = str(guidance.get('vehicle_model', '')).lower()
                pn = phase_str.upper()
                cfg = state.sim_config
                if cfg is not None:
                    s2_dry = float(cfg.stage2_dry_mass) + float(cfg.payload_mass)
                    s1_dry = float(cfg.stage1_dry_mass)
                    stack_dry = s1_dry + s2_dry
                else:
                    s2_dry = C.STAGE2_DRY_MASS
                    s1_dry = C.STAGE1_DRY_MASS
                    stack_dry = C.DRY_MASS
                dry = s2_dry if (vm == 'orbiter' or pn in ('ORBIT_INSERTION', 'S2_COAST_TO_APOGEE')) else \
                      s1_dry if vm == 'booster' else stack_dry
            propellant = max(0.0, state.m - float(dry))
        propellant = float(max(0.0, propellant))

        entry = {
            'time': state.t,
            'altitude': state.altitude / 1000,
            'downrange': C.R_EARTH * central_angle / 1000,
            'downrange_ground': C.R_EARTH * central_angle_ground / 1000.0 if central_angle_ground else 0.0,
            'velocity': state.speed,
            'velocity_rel': v_rel_mag,
            'velocity_horizontal': v_horiz,
            'velocity_vertical': v_vert,
            'radial_velocity': v_radial,
            'horizontal_velocity_inertial': float(vec_norm(v_horiz_inertial_vec)),
            'velocity_x': float(state.v[0]),
            'velocity_y': float(state.v[1]),
            'velocity_z': float(state.v[2]),
            'velocity_rel_x': float(v_rel[0]),
            'velocity_rel_y': float(v_rel[1]),
            'velocity_rel_z': float(v_rel[2]),
            'mass': state.m,
            'pitch_angle': np.degrees(guidance['pitch_angle']),
            'attitude_error': float(control.get('error_degrees', 0.0)),
            'torque_magnitude': float(control.get('torque_magnitude', 0.0)),
            'torque_x': float(torque_vec[0]),
            'torque_y': float(torque_vec[1]),
            'torque_z': float(torque_vec[2]),
            'attitude_torque_limit': float(guidance.get('attitude_torque_limit_n_m', 0.0)),
            'attitude_torque_used_fraction': float(guidance.get('attitude_torque_used_fraction', 0.0)),
            'force_gravity_n': float(guidance.get('force_gravity_n', 0.0)),
            'force_thrust_n': float(guidance.get('force_thrust_n', 0.0)),
            'force_drag_n': float(guidance.get('force_drag_n', 0.0)),
            'force_lift_n': float(guidance.get('force_lift_n', 0.0)),
            'force_total_n': float(guidance.get('force_total_n', 0.0)),
            'position_x': float(state.r[0]),
            'position_y': float(state.r[1]),
            'position_z': float(state.r[2]),
            'omega_x': float(state.omega[0]),
            'omega_y': float(state.omega[1]),
            'omega_z': float(state.omega[2]),
            'quaternion_norm': float(vec_norm(state.q)),
            'actual_pitch_angle': np.degrees(np.arccos(cos_pitch)),
            'gamma_command_deg': gamma_cmd_deg,
            'gamma_actual_deg': gamma_actual,
            'velocity_tilt_deg': np.degrees(np.arctan2(v_horiz, abs(v_vert))),
            'commanded_thrust_x': float(cmd_thrust_dir[0]),
            'commanded_thrust_y': float(cmd_thrust_dir[1]),
            'commanded_thrust_z': float(cmd_thrust_dir[2]),
            'commanded_quat_w': float(cmd_quat[0]),
            'commanded_quat_x': float(cmd_quat[1]),
            'commanded_quat_y': float(cmd_quat[2]),
            'commanded_quat_z': float(cmd_quat[3]),
            'actual_quat_w': float(state.q[0]),
            'actual_quat_x': float(state.q[1]),
            'actual_quat_y': float(state.q[2]),
            'actual_quat_z': float(state.q[3]),
            'inertial_thrust_x': float(thrust_vec[0]),
            'inertial_thrust_y': float(thrust_vec[1]),
            'inertial_thrust_z': float(thrust_vec[2]),
            'prograde_x': float(prograde[0]),
            'prograde_y': float(prograde[1]),
            'prograde_z': float(prograde[2]),
            'flight_path_angle_deg': np.degrees(float(gamma)),
            'throttle': throttle_val,
            'thrust_on': 1.0 if thrust_on_val else 0.0,
            'mach_number': mach,
            'dynamic_pressure': q_dyn,
            'angle_of_attack_deg': np.degrees(aoa_rad),
            'q_alpha': q_dyn * aoa_rad,
            'heating_rate': q_heat,
            'latitude_deg': lat,
            'longitude_deg': lon_geo,
            'ignition_altitude_prediction_m': float(guidance.get('ignition_altitude_prediction_m', 0.0)),
            'propellant_remaining_kg': propellant,
            'grid_fin_deployed_fraction': float(guidance.get('grid_fin_deployed_fraction', 0.0)),
            'grid_fin_pitch_cmd_deg': float(guidance.get('grid_fin_pitch_cmd_deg', 0.0)),
            'grid_fin_yaw_cmd_deg': float(guidance.get('grid_fin_yaw_cmd_deg', 0.0)),
            'grid_fin_force_n': float(guidance.get('grid_fin_force_n', 0.0)),
            'grid_fin_saturated': 1.0 if bool(guidance.get('grid_fin_saturated', False)) else 0.0,
            'landing_leg_status': str(guidance.get('landing_leg_status', '')),
            'landing_leg_deployed_fraction': float(guidance.get('landing_leg_deployed_fraction', 0.0)),
            'predicted_landing_error_m': float(guidance.get('predicted_landing_error_m', 0.0)),
            'landing_target_lead_time_s': float(guidance.get('landing_target_lead_time_s', 0.0)),
            'landing_a_vert_needed_mps2': float(guidance.get('landing_a_vert_needed_mps2', 0.0)),
            'landing_a_horiz_budget_mps2': float(guidance.get('landing_a_horiz_budget_mps2', 0.0)),
            'planner_score': float(guidance.get('planner_score', 0.0)),
            'planner_best_miss_m': float(guidance.get('planner_best_miss_m', 0.0)),
            'planner_best_speed_mps': float(guidance.get('planner_best_speed_mps', 0.0)),
            'planner_best_propellant_kg': float(guidance.get('planner_best_propellant_kg', 0.0)),
            'planner_best_max_q_pa': float(guidance.get('planner_best_max_q_pa', 0.0)),
            'planner_reachable': 1.0 if bool(guidance.get('planner_reachable', False)) else 0.0,
            'planner_required_lateral_accel_mps2': float(guidance.get('planner_required_lateral_accel_mps2', 0.0)),
            'planner_available_lateral_accel_mps2': float(guidance.get('planner_available_lateral_accel_mps2', 0.0)),
            'planner_time_to_go_s': float(guidance.get('planner_time_to_go_s', 0.0)),
            'landing_guidance_mode': str(guidance.get('landing_guidance_mode', '')),
            'gfold_time_of_flight_s': float(guidance.get('gfold_time_of_flight_s', 0.0)),
            'touchdown_contact_status': str(guidance.get('touchdown_contact_status', '')),
            'nav_position_error_m': float(guidance.get('nav_position_error_m', 0.0)),
            'nav_velocity_error_mps': float(guidance.get('nav_velocity_error_mps', 0.0)),
            'gps_available': 1.0 if bool(guidance.get('gps_available', False)) else 0.0,
            'imu_available': 1.0 if bool(guidance.get('imu_available', False)) else 0.0,
            'altimeter_available': 1.0 if bool(guidance.get('altimeter_available', False)) else 0.0,
            'guidance_uses_navigation_estimate': 1.0 if bool(guidance.get('guidance_uses_navigation_estimate', False)) else 0.0,
            'phase_name': str(guidance.get('phase', 'UNKNOWN')),
            'rcs_propellant_remaining_kg': float(guidance.get('rcs_propellant_remaining_kg', 0.0)),
            'rcs_exhausted': 1.0 if bool(guidance.get('rcs_exhausted', False)) else 0.0,
            'thermal_q_dot_w_m2': float(guidance.get('thermal_q_dot_w_m2', 0.0)),
            'thermal_wall_temp_k': float(guidance.get('thermal_wall_temp_k', 300.0)),
            'thermal_heat_load_j_m2': float(guidance.get('thermal_heat_load_j_m2', 0.0)),
            'slosh_displacement_m': float(guidance.get('slosh_displacement_m', 0.0)),
            'slosh_cg_offset_m': float(guidance.get('slosh_cg_offset_m', 0.0)),
            'slosh_torque_n_m': float(guidance.get('slosh_torque_n_m', 0.0)),
            'flex_eta_m': float(guidance.get('flex_eta_m', 0.0)),
            'flex_eta_dot_mps': float(guidance.get('flex_eta_dot_mps', 0.0)),
            'flex_parasitic_angle_rad': float(guidance.get('flex_parasitic_angle_rad', 0.0)),
        }
        for k, v in entry.items():
            self._data.setdefault(k, []).append(v)


    def to_csv(self, filename: str):
        os.makedirs(os.path.dirname(filename) or '.', exist_ok=True)
        with open(filename, 'w', newline='') as fh:
            writer = csv.writer(fh)
            writer.writerow(list(self._data.keys()))
            writer.writerows(zip(*self._data.values()))


@dataclass
class FullMissionResult:
    """Result of a full integrated mission with dual-vehicle tracking."""

    ascent_log: SimulationLog
    ascent_final_state: State
    ascent_reason: str
    separation_time: float | None
    orbiter_log: SimulationLog
    orbiter_final_state: State
    orbiter_reason: str
    booster_log: SimulationLog
    booster_final_state: State
    booster_reason: str
    orbiter_success: bool = False
    booster_landing_success: bool = False
    # Measured at booster touchdown (None if the booster never touched down).
    booster_touchdown_speed_mps: float | None = None
    booster_site_error_m: float | None = None

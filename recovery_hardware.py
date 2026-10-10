"""Falcon-style booster recovery hardware and touchdown assessment helpers."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from . import constants as C
from .config_definition import SimulationConfig
from .forces import compute_configured_atmosphere_properties
from .utils import (
    compute_ground_relative_velocity,
    compute_relative_velocity,
    cross3,
    vec_norm,
)


class LandingLegStatus(str, Enum):
    """Discrete deployment state for Falcon-style booster landing legs."""

    STOWED = "stowed"
    DEPLOYING = "deploying"
    LOCKED = "locked"
    FAILED = "failed"


@dataclass(frozen=True)
class GridFinState:
    """Grid-fin command and deployment state for one guidance step."""

    deployed_fraction: float = 0.0
    pitch_cmd_deg: float = 0.0
    yaw_cmd_deg: float = 0.0
    saturated: bool = False
    force_direction: np.ndarray | None = None


@dataclass(frozen=True)
class LandingLegState:
    """Landing-leg deployment progress."""

    status: LandingLegStatus = LandingLegStatus.STOWED
    deployed_fraction: float = 0.0
    deploy_elapsed_s: float = 0.0


@dataclass(frozen=True)
class TouchdownAssessment:
    """Contact classification for booster touchdown."""

    success: bool
    status: str
    reason: str
    touchdown_speed_mps: float
    vertical_speed_mps: float
    horizontal_speed_mps: float
    site_error_m: float
    landing_leg_status: str
    terminal_attitude_error_deg: float
    failed_criteria: list[str]


def _local_vertical(r: np.ndarray) -> np.ndarray:
    norm = float(vec_norm(r))
    if norm < 1e-9:
        return np.array([1.0, 0.0, 0.0])
    return np.asarray(r, dtype=float) / norm


def _horizontal_component(vector: np.ndarray, vertical: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    return vector - float(np.dot(vector, vertical)) * vertical


def _unit_or_zero(vector: np.ndarray) -> np.ndarray:
    norm = float(vec_norm(vector))
    if norm < 1e-9:
        return np.zeros(3)
    return np.asarray(vector, dtype=float) / norm


def update_landing_leg_state(
    leg_state: LandingLegState | None,
    state,
    config: SimulationConfig,
    dt: float,
) -> LandingLegState:
    """Advance the landing-leg deployment state for the current booster state."""
    current = leg_state or LandingLegState()
    altitude = float(state.altitude)

    if not bool(config.enable_landing_legs):
        return LandingLegState()

    if current.status in (LandingLegStatus.LOCKED, LandingLegStatus.FAILED):
        return current

    if altitude > float(config.landing_leg_deploy_altitude_m):
        return current

    elapsed = float(current.deploy_elapsed_s) + max(float(dt), 0.0)
    deploy_time = max(float(config.landing_leg_deploy_time_s), 1e-6)
    fraction = float(np.clip(elapsed / deploy_time, 0.0, 1.0))
    status = LandingLegStatus.LOCKED if fraction >= 1.0 else LandingLegStatus.DEPLOYING
    return LandingLegState(
        status=status,
        deployed_fraction=fraction,
        deploy_elapsed_s=elapsed,
    )


def command_grid_fins_to_target(
    r: np.ndarray,
    v: np.ndarray,
    target_site_eci: np.ndarray,
    config: SimulationConfig,
    mass: float | None = None,
    wind_offset_mps: float | None = None,
    time_to_go_s: float | None = None,
    zero_effort_miss: np.ndarray | None = None,
) -> GridFinState:
    """ZEM/ZEV grid fin guidance for lateral control during descent.

    Converts the Zero-Effort Miss / Zero-Effort Velocity into a fin-pitch/yaw
    command, scaled by dynamic pressure so the fins provide the acceleration
    needed to reach the pad with zero horizontal velocity.
    """
    if not bool(config.enable_grid_fins):
        return GridFinState()

    altitude = float(vec_norm(r) - C.R_EARTH)
    if altitude > float(config.grid_fin_deploy_altitude_m):
        return GridFinState()

    deployed_fraction = 1.0
    vertical = _local_vertical(r)
    site_vec = _horizontal_component(np.asarray(target_site_eci, dtype=float) - r, vertical)
    site_dist = float(vec_norm(site_vec))
    toward_site = _unit_or_zero(site_vec)

    omega_earth = np.array([0.0, 0.0, C.EARTH_ROTATION_RATE])
    v_ground = v - cross3(omega_earth, r)
    v_ground_horiz = _horizontal_component(v_ground, vertical)
    v_vert = float(np.dot(v_ground, vertical))

    if time_to_go_s is not None and float(time_to_go_s) > 0.0:
        t_go = float(np.clip(time_to_go_s, 2.0, 180.0))
    else:
        descent_rate = max(-v_vert, 1.0)
        t_go = min(altitude / descent_rate, 60.0)
        t_go = max(t_go, 2.0)

    if zero_effort_miss is not None:
        zem = _horizontal_component(
            np.asarray(zero_effort_miss, dtype=float),
            vertical,
        )
    else:
        zem = site_vec - v_ground_horiz * t_go

    a_desired = (6.0 / (t_go ** 2)) * zem + (2.0 / t_go) * v_ground_horiz
    a_mag = float(vec_norm(a_desired))

    if a_mag < 1e-9:
        return GridFinState(deployed_fraction=deployed_fraction)

    force_direction = np.asarray(a_desired, dtype=float) / a_mag

    v_rel = compute_relative_velocity(
        r, v,
        wind_offset_mps=config.runtime_wind_offset_mps if wind_offset_mps is None else wind_offset_mps,
    )
    speed = float(vec_norm(v_rel))
    _, _, rho, _ = compute_configured_atmosphere_properties(altitude, config)

    if rho < C.DENSITY_FLOOR or speed < C.SMALL_VELOCITY_TOL:
        deflection_fraction = 1.0
    else:
        q_dyn = 0.5 * rho * speed ** 2
        area = float(config.grid_fin_area_m2)
        cl_max = float(config.grid_fin_cl_max)
        m = mass if mass is not None else 40000.0
        max_aero_accel = q_dyn * area * cl_max / m
        if max_aero_accel < 0.5:
            deflection_fraction = 1.0
        else:
            deflection_fraction = min(a_mag / max_aero_accel, 1.0)

    max_deflection = float(config.grid_fin_max_deflection_deg)
    east_like = toward_site if site_dist > 100.0 else _unit_or_zero(v_ground_horiz)
    if vec_norm(east_like) <= 0.0:
        east_like = np.array([0.0, 1.0, 0.0])
    north_like = _unit_or_zero(cross3(vertical, east_like))

    raw_pitch = deflection_fraction * max_deflection * float(np.dot(force_direction, east_like))
    raw_yaw = deflection_fraction * max_deflection * float(np.dot(force_direction, north_like))
    pitch = float(np.clip(raw_pitch, -max_deflection, max_deflection))
    yaw = float(np.clip(raw_yaw, -max_deflection, max_deflection))
    saturated = (
        abs(raw_pitch) > max_deflection
        or abs(raw_yaw) > max_deflection
        or deflection_fraction >= 1.0 - 1e-6
    )

    return GridFinState(
        deployed_fraction=deployed_fraction,
        pitch_cmd_deg=pitch,
        yaw_cmd_deg=yaw,
        saturated=saturated,
        force_direction=force_direction,
    )


def compute_grid_fin_force(
    r: np.ndarray,
    v: np.ndarray,
    q: np.ndarray,
    command: GridFinState,
    config: SimulationConfig,
    wind_offset_mps: float | None = None,
) -> np.ndarray:
    """Compute explicit grid-fin aerodynamic force in ECI coordinates."""
    del q
    if not bool(config.enable_grid_fins):
        return np.zeros(3)
    deployed = float(np.clip(command.deployed_fraction, 0.0, 1.0))
    if deployed <= 0.0:
        return np.zeros(3)

    from .forces import _altitude_from_r

    altitude = float(_altitude_from_r(r, config))
    _, _, rho, _ = compute_configured_atmosphere_properties(altitude, config)
    if rho < C.DENSITY_FLOOR:
        return np.zeros(3)

    v_rel = compute_relative_velocity(
        r,
        v,
        wind_offset_mps=config.runtime_wind_offset_mps if wind_offset_mps is None else wind_offset_mps,
    )
    speed = float(vec_norm(v_rel))
    if speed < C.SMALL_VELOCITY_TOL:
        return np.zeros(3)

    max_deflection = max(float(config.grid_fin_max_deflection_deg), 1e-6)
    deflection = min(
        1.0,
        float(np.hypot(command.pitch_cmd_deg, command.yaw_cmd_deg)) / max_deflection,
    )
    if deflection <= 0.0:
        return np.zeros(3)

    q_dyn = 0.5 * rho * speed ** 2
    area = float(config.grid_fin_area_m2)
    lift_mag = q_dyn * area * float(config.grid_fin_cl_max) * deflection * deployed
    force_direction = (
        _unit_or_zero(command.force_direction)
        if command.force_direction is not None
        else np.zeros(3)
    )
    if vec_norm(force_direction) <= 0.0:
        return np.zeros(3)

    drag_mag = q_dyn * area * float(config.grid_fin_cd_increment) * deflection * deployed
    drag = -drag_mag * (v_rel / speed)
    return lift_mag * force_direction + drag


def assess_touchdown_contact(
    state,
    config: SimulationConfig,
    site_error_m: float,
    leg_state: LandingLegState | None,
    terminal_attitude_error_deg: float,
) -> TouchdownAssessment:
    """Classify final booster contact using speed, pad, leg, and tilt gates."""
    vertical = _local_vertical(state.r)
    v_ground = compute_ground_relative_velocity(state.r, state.v)
    vertical_speed = float(np.dot(v_ground, vertical))
    horizontal_speed = float(vec_norm(_horizontal_component(v_ground, vertical)))
    touchdown_speed = float(vec_norm(v_ground))
    site_error = float(site_error_m)
    leg_status = (leg_state or LandingLegState()).status

    failed: list[str] = []
    pad_hit = site_error <= float(config.booster_pad_tolerance_m)
    site_km = site_error / 1000.0

    if bool(config.enable_landing_legs) and leg_status != LandingLegStatus.LOCKED:
        failed.append("landing_legs")
        return TouchdownAssessment(
            success=False,
            status="leg_deployment_failure",
            reason="LEG DEPLOYMENT FAILURE - landing legs not locked before touchdown",
            touchdown_speed_mps=touchdown_speed,
            vertical_speed_mps=vertical_speed,
            horizontal_speed_mps=horizontal_speed,
            site_error_m=site_error,
            landing_leg_status=leg_status.value,
            terminal_attitude_error_deg=float(terminal_attitude_error_deg),
            failed_criteria=failed,
        )

    if abs(float(terminal_attitude_error_deg)) > float(config.landing_leg_max_tilt_deg):
        failed.append("terminal_attitude_error")
        return TouchdownAssessment(
            success=False,
            status="tipover",
            reason=(
                "TIPOVER - touchdown attitude error "
                f"{float(terminal_attitude_error_deg):.1f} deg exceeds leg stability limit"
            ),
            touchdown_speed_mps=touchdown_speed,
            vertical_speed_mps=vertical_speed,
            horizontal_speed_mps=horizontal_speed,
            site_error_m=site_error,
            landing_leg_status=leg_status.value,
            terminal_attitude_error_deg=float(terminal_attitude_error_deg),
            failed_criteria=failed,
        )

    if touchdown_speed <= float(config.landing_leg_max_touchdown_speed_mps):
        if pad_hit:
            status = "landing_success"
            reason = f"LANDING SUCCESS - Touchdown at {touchdown_speed:.2f} m/s (site error {site_error:.0f} m)"
        else:
            status = "offsite_touchdown"
            reason = f"OFFSITE TOUCHDOWN - Landed {site_km:.2f} km from target at {touchdown_speed:.2f} m/s"
    elif touchdown_speed < max(
        10.0,
        float(config.landing_leg_max_touchdown_speed_mps) * 2.0,
    ):
        # Same config-derived boundary as mission_summary.hard_landing_speed_limit.
        # A bare 10.0 literal here made the hard/crash bands unreachable for any
        # configured landing_leg_max_touchdown_speed_mps above 10 m/s. Kept
        # local (rather than imported) to avoid a mission_summary <-> recovery_hardware
        # import cycle.
        if pad_hit:
            status = "hard_landing"
            reason = f"HARD LANDING - Touchdown at {touchdown_speed:.1f} m/s (site error {site_error:.0f} m)"
        else:
            status = "hard_offsite_landing"
            reason = f"HARD OFFSITE LANDING - Touchdown at {touchdown_speed:.1f} m/s ({site_km:.2f} km from target)"
    else:
        if pad_hit:
            status = "crash_landing"
            reason = f"CRASH LANDING - Impact at {touchdown_speed:.0f} m/s (site error {site_error:.0f} m)"
        else:
            status = "offsite_crash_landing"
            reason = f"OFFSITE CRASH LANDING - Impact at {touchdown_speed:.0f} m/s ({site_km:.2f} km from target)"

    if touchdown_speed > float(config.landing_leg_max_touchdown_speed_mps):
        failed.append("touchdown_speed")
    if not pad_hit:
        failed.append("pad_error")

    return TouchdownAssessment(
        success=status == "landing_success" and not failed,
        status=status,
        reason=reason,
        touchdown_speed_mps=touchdown_speed,
        vertical_speed_mps=vertical_speed,
        horizontal_speed_mps=horizontal_speed,
        site_error_m=site_error,
        landing_leg_status=leg_status.value,
        terminal_attitude_error_deg=float(terminal_attitude_error_deg),
        failed_criteria=failed,
    )

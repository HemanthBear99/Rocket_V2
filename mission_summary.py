"""Structured mission truth and report generation."""

from __future__ import annotations

import json
import math
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from . import constants as C
from ._main_models import FullMissionResult, SimulationLog
from ._mission_manager_helpers import compute_orbit_metrics
from .config_definition import SimulationConfig
from .frames import rotate_vector_by_quaternion
from .recovery import (
    great_circle_distance_m,
    rotating_launch_site_eci,
    target_landing_site_eci,
)
from .recovery_hardware import LandingLegStatus
from .state import State
from .utils import compute_ground_relative_velocity, vec_norm


@dataclass
class OrbitAssessment:
    """Reusable pass/fail assessment for the configured orbit target."""

    success: bool
    status: str
    reason: str
    perigee_altitude_km: float | None
    apogee_altitude_km: float | None
    eccentricity: float | None
    propellant_remaining_kg: float | None
    velocity_deficit_mps: float | None
    failed_criteria: list[str]

    target_altitude_km: float | None = None
    altitude_tolerance_km: float | None = None
    apogee_error_km: float | None = None
    perigee_error_km: float | None = None
    timeout_used: bool | None = None


@dataclass
class LandingAssessment:
    """Reusable pass/fail assessment for booster touchdown."""

    success: bool
    status: str
    reason: str
    touchdown_speed_mps: float | None
    site_error_m: float | None
    propellant_remaining_kg: float | None
    failed_criteria: list[str]
    vertical_speed_mps: float | None = None
    horizontal_speed_mps: float | None = None
    landing_leg_status: str | None = None
    touchdown_contact_status: str | None = None
    terminal_attitude_error_deg: float | None = None


@dataclass
class MissionAssessment:
    """Top-level mission result with strict reusable success criteria."""

    mission_success: bool
    status: str
    reason: str
    separation_occurred: bool
    orbit: OrbitAssessment
    landing: LandingAssessment
    max_ascent_q_pa: float | None
    max_recovery_q_pa: float | None
    max_recovery_q_alpha_pa_rad: float | None
    terminal_attitude_error_deg: float | None
    failed_criteria: list[str]
    separation_time_s: float | None = None
    mission_duration_s: float | None = None
    s2_recovery: dict[str, Any] | None = None


def _finite_or_none(value: float | None) -> float | None:
    if value is None:
        return None
    value = float(value)
    if not math.isfinite(value):
        return None
    return value


def _fmt_optional_float(value: float | None, precision: int = 1) -> str:
    """Format a diagnostic value without crashing on non-finite metrics."""
    finite = _finite_or_none(value)
    return f"{finite:.{precision}f}" if finite is not None else "unavailable"


def _failure_status_from_reason(termination_reason: str | None) -> bool:
    if not termination_reason:
        return False
    text = termination_reason.upper()
    return any(
        token in text
        for token in (
            "FAILED",
            "FAILURE",
            "CRASH",
            "MAXIMUM SIMULATION TIME",
            "MAX TIME",
            "VALIDATION",
            "ABORT",
        )
    )


def assess_orbit(
    state: State,
    config: SimulationConfig,
    termination_reason: str | None = None,
) -> OrbitAssessment:
    """Assess whether an orbiter state satisfies the configured target orbit."""
    metrics = compute_orbit_metrics(state)
    target = float(config.orbit_target_altitude_m)
    tolerance = float(config.orbit_altitude_tolerance_m)

    perigee_alt = metrics["perigee_alt"]
    apogee_alt = metrics["apogee_alt"]
    ecc = metrics["ecc"]
    propellant = max(0.0, metrics["s2_prop_remaining"])
    v_deficit = metrics["v_deficit"]

    failed: list[str] = []
    if not math.isfinite(perigee_alt) or abs(perigee_alt - target) > tolerance:
        failed.append("perigee_altitude")
    if not math.isfinite(apogee_alt) or abs(apogee_alt - target) > tolerance:
        failed.append("apogee_altitude")
    if not math.isfinite(ecc) or ecc > float(config.orbit_ecc_max):
        failed.append("eccentricity")
    if metrics["energy"] >= 0.0:
        failed.append("bound_orbit")

    success = not failed
    target_km = target / 1000.0
    tol_km = tolerance / 1000.0
    apo_err_km = _finite_or_none(abs(apogee_alt - target) / 1000.0) if math.isfinite(apogee_alt) else None
    peri_err_km = _finite_or_none(abs(perigee_alt - target) / 1000.0) if math.isfinite(perigee_alt) else None
    timeout_hint = bool(termination_reason and any(k in termination_reason.lower() for k in ("timeout", "stall", "maximum time", "max time")))

    if success:
        status = "orbit_achieved"
        reason = "ORBIT ACHIEVED - target orbit within configured tolerance"
    elif _failure_status_from_reason(termination_reason):
        status = "orbit_failed"
        base = termination_reason or "ORBIT INSERTION FAILED - target orbit not achieved"
        reason = (
            f"{base} "
            f"(target {target_km:.1f} km \u00b1{tol_km:.1f} km; "
            f"achieved apo={_fmt_optional_float(apogee_alt / 1000.0)} "
            f"peri={_fmt_optional_float(perigee_alt / 1000.0)}; "
            f"err_apo={apo_err_km}km err_peri={peri_err_km}km; "
            f"prop={propellant:.0f} kg)"
        )
    else:
        status = "orbit_incomplete"
        reason = termination_reason or "Orbit target not yet achieved"

    return OrbitAssessment(
        success=success,
        status=status,
        reason=reason,
        perigee_altitude_km=_finite_or_none(perigee_alt / 1000.0),
        apogee_altitude_km=_finite_or_none(apogee_alt / 1000.0),
        eccentricity=_finite_or_none(ecc),
        propellant_remaining_kg=_finite_or_none(propellant),
        velocity_deficit_mps=_finite_or_none(v_deficit),
        failed_criteria=failed,
        target_altitude_km=_finite_or_none(target_km),
        altitude_tolerance_km=_finite_or_none(tol_km),
        apogee_error_km=apo_err_km,
        perigee_error_km=peri_err_km,
        timeout_used=timeout_hint if not success else None,
    )


def hard_landing_speed_limit(config: SimulationConfig) -> float:
    """Upper edge of the survivable-but-hard touchdown band, in m/s.

    The hard/crash boundary used to be a bare 10.0 m/s literal while the
    pass/fail gate above it used config.landing_leg_max_touchdown_speed_mps.
    Because that field is only validated as > 0, any configured limit above
    10 m/s made the hard_landing / hard_offsite_landing / crash_landing /
    offsite_crash_landing bands UNREACHABLE -- a 12 m/s impact was reported
    as landing_success (verified by execution).

    Derive the boundary from the configured success limit instead, keeping
    the original 10 m/s value at the 6 m/s default so existing
    classification behaviour is unchanged, and always leaving a non-empty
    band above the success gate.
    """
    success_limit = float(config.landing_leg_max_touchdown_speed_mps)
    # Keep the legacy 10 m/s boundary EXACTLY for any success limit at or below
    # it (so the default 6 m/s profile reports identical statuses to before this
    # fix), and only scale upward once a user configures a limit above 10 m/s,
    # which is the case the old hardcoded literal made unreachable.
    if success_limit <= 10.0:
        return 10.0
    return success_limit * 2.0


def classify_landing(
    speed_mps: float,
    site_error_m: float,
    config: SimulationConfig,
    propellant_remaining_kg: float | None = None,
) -> LandingAssessment:
    """Classify touchdown by speed first, then pad error and propellant margin."""
    speed = float(speed_mps)
    site_error = float(site_error_m)
    pad_hit = site_error <= float(config.booster_pad_tolerance_m)
    site_km = site_error / 1000.0

    if speed <= float(config.landing_leg_max_touchdown_speed_mps):
        if pad_hit:
            status = "landing_success"
            reason = f"LANDING SUCCESS - Touchdown at {speed:.2f} m/s (site error {site_error:.0f} m)"
        else:
            status = "offsite_touchdown"
            reason = f"OFFSITE TOUCHDOWN - Landed {site_km:.2f} km from target at {speed:.2f} m/s"
    elif speed < hard_landing_speed_limit(config):
        if pad_hit:
            status = "hard_landing"
            reason = f"HARD LANDING - Touchdown at {speed:.1f} m/s (site error {site_error:.0f} m)"
        else:
            status = "hard_offsite_landing"
            reason = f"HARD OFFSITE LANDING - Touchdown at {speed:.1f} m/s ({site_km:.2f} km from target)"
    else:
        if pad_hit:
            status = "crash_landing"
            reason = f"CRASH LANDING - Impact at {speed:.0f} m/s (site error {site_error:.0f} m)"
        else:
            status = "offsite_crash_landing"
            reason = f"OFFSITE CRASH LANDING - Impact at {speed:.0f} m/s ({site_km:.2f} km from target)"

    failed: list[str] = []
    if speed > float(config.landing_leg_max_touchdown_speed_mps):
        failed.append("touchdown_speed")
    if not pad_hit:
        failed.append("pad_error")
    if propellant_remaining_kg is not None:
        margin = float(config.booster_landing_propellant_margin_kg)
        if float(propellant_remaining_kg) < margin:
            failed.append("propellant_margin")

    success = status == "landing_success" and not failed
    if status == "landing_success" and not success:
        reason += f" - but propellant margin below {config.booster_landing_propellant_margin_kg:.0f} kg reserve"

    return LandingAssessment(
        success=success,
        status=status,
        reason=reason,
        touchdown_speed_mps=_finite_or_none(speed),
        site_error_m=_finite_or_none(site_error),
        propellant_remaining_kg=_finite_or_none(propellant_remaining_kg),
        failed_criteria=failed,
        touchdown_contact_status=status,
    )


def extract_landing_metrics(reason: str) -> tuple[float | None, float | None]:
    """Extract touchdown speed and pad error from legacy or current reason text."""
    if not reason:
        return None, None

    # Last "at N m/s" wins: the touchdown speed is the final speed quoted.
    speed_matches = re.findall(r"\bat\s+([0-9]+(?:\.[0-9]+)?)\s*m/s", reason)
    speed = float(speed_matches[-1]) if speed_matches else None

    site_error_m = None
    patterns = [
        (r"site error\s+([0-9]+(?:\.[0-9]+)?)\s*m", 1.0),
        (r"Missed pad by\s+([0-9]+(?:\.[0-9]+)?)\s*km", 1000.0),
        (r"Landed\s+([0-9]+(?:\.[0-9]+)?)\s*km\s+from target", 1000.0),
        (r"\(([0-9]+(?:\.[0-9]+)?)\s*km\s+from target\)", 1000.0),
    ]
    for pattern, scale in patterns:
        match = re.search(pattern, reason)
        if match:
            site_error_m = float(match.group(1)) * scale
            break
    return speed, site_error_m


def _last_or_none(values: list[Any]) -> Any | None:
    return values[-1] if values else None


def _max_or_none(values: list[float]) -> float | None:
    if not values:
        return None
    return float(max(values))


def _booster_propellant_remaining(
    result: FullMissionResult,
    config: SimulationConfig,
) -> float | None:
    if result.separation_time is None:
        return None
    from_log = _last_or_none(result.booster_log.get_series("propellant_remaining_kg"))
    if from_log is not None:
        return max(0.0, float(from_log))
    return max(0.0, float(result.booster_final_state.m - config.stage1_dry_mass))


def _landing_metrics_from_result(
    result: FullMissionResult,
    config: SimulationConfig,
) -> tuple[float, float]:
    speed = getattr(result, "booster_touchdown_speed_mps", None)
    site_error = getattr(result, "booster_site_error_m", None)
    if speed is None or site_error is None:
        # Results built without touchdown measurements (older callers):
        # fall back to the values quoted in the reason text.
        parsed_speed, parsed_error = extract_landing_metrics(result.booster_reason)
        speed = parsed_speed if speed is None else speed
        site_error = parsed_error if site_error is None else site_error
    if speed is None:
        speed = float(
            vec_norm(
                compute_ground_relative_velocity(
                    result.booster_final_state.r,
                    result.booster_final_state.v,
                )
            )
        )
    if site_error is None:
        target_site = target_landing_site_eci(
            result.booster_final_state.t,
            config.booster_landing_target_downrange_km,
            config=config,
        )
        site_error = great_circle_distance_m(result.booster_final_state.r, target_site)
    return speed, site_error


def _landing_velocity_components(
    result: FullMissionResult,
    config: SimulationConfig,
) -> tuple[float | None, float | None]:
    state = result.booster_final_state
    r_norm = float(vec_norm(state.r))
    if r_norm < 1e-9:
        return None, None
    vertical = state.r / r_norm
    v_ground = compute_ground_relative_velocity(state.r, state.v)
    vertical_speed = float(np.dot(v_ground, vertical))
    horizontal_speed = float(vec_norm(v_ground - vertical_speed * vertical))
    return vertical_speed, horizontal_speed


def _terminal_touchdown_tilt_deg(result: FullMissionResult) -> float | None:
    state = result.booster_final_state
    r_norm = float(vec_norm(state.r))
    if r_norm < 1e-9:
        return None
    body_z = rotate_vector_by_quaternion(C.BODY_Z_AXIS, state.q)
    vertical = state.r / r_norm
    return float(
        np.degrees(
            np.arccos(np.clip(float(np.dot(body_z, vertical)), -1.0, 1.0))
        )
    )


def _attitude_gate_failed(result: FullMissionResult, config: SimulationConfig) -> bool:
    terminal_error = _terminal_touchdown_tilt_deg(result)
    if terminal_error is None:
        return False
    return float(terminal_error) > float(config.booster_terminal_attitude_error_max_deg)


def _contact_status_from_log(result: FullMissionResult) -> str | None:
    status = _last_or_none(result.booster_log.get_series("touchdown_contact_status"))
    if status:
        return str(status)
    return None


def _leg_status_from_log(result: FullMissionResult) -> str | None:
    status = _last_or_none(result.booster_log.get_series("landing_leg_status"))
    if status:
        return str(status)
    return None


def _landing_with_contact_gates(
    landing: LandingAssessment,
    result: FullMissionResult,
    terminal_attitude_error: float | None,
) -> LandingAssessment:
    """Overlay explicit contact/leg gates on the speed-and-pad landing result."""
    contact_status = _contact_status_from_log(result)
    leg_status = _leg_status_from_log(result)
    failed = list(landing.failed_criteria)
    status = contact_status or landing.status
    reason = landing.reason
    success = landing.success

    if leg_status is not None and leg_status != LandingLegStatus.LOCKED.value:
        if "landing_legs" not in failed:
            failed.append("landing_legs")
        status = "leg_deployment_failure"
        reason = "LEG DEPLOYMENT FAILURE - landing legs not locked before touchdown"
        success = False

    if contact_status and contact_status != "landing_success":
        success = False
        status = contact_status
        if contact_status == "tipover" and "terminal_attitude_error" not in failed:
            failed.append("terminal_attitude_error")
        elif contact_status == "leg_deployment_failure" and "landing_legs" not in failed:
            failed.append("landing_legs")
        elif "landing_contact" not in failed and contact_status != landing.status:
            failed.append("landing_contact")

    return LandingAssessment(
        success=success and not failed,
        status=status,
        reason=reason,
        touchdown_speed_mps=landing.touchdown_speed_mps,
        site_error_m=landing.site_error_m,
        propellant_remaining_kg=landing.propellant_remaining_kg,
        failed_criteria=failed,
        vertical_speed_mps=landing.vertical_speed_mps,
        horizontal_speed_mps=landing.horizontal_speed_mps,
        landing_leg_status=leg_status,
        touchdown_contact_status=(
            contact_status or landing.touchdown_contact_status or landing.status
        ),
        terminal_attitude_error_deg=_finite_or_none(terminal_attitude_error),
    )


def _fmt_time(t: float | None) -> str:
    """Format a mission time as seconds plus mm:ss for readability."""
    if t is None or not math.isfinite(float(t)):
        return "n/a"
    t = float(t)
    return f"{t:.1f} s ({int(t // 60)}m{int(t % 60):02d}s)"


def _first_phase_time(log: SimulationLog, phase: str) -> float | None:
    for i, name in enumerate(log.get_series("phase_name")):
        if name == phase:
            return float(log.time[i])
    return None


def _state_from_log(log: SimulationLog, i: int, dry_mass_kg: float) -> State:
    return State(
        r=np.array([log.position_x[i], log.position_y[i], log.position_z[i]], dtype=float),
        v=np.array([log.velocity_x[i], log.velocity_y[i], log.velocity_z[i]], dtype=float),
        q=np.array([1.0, 0.0, 0.0, 0.0]),
        omega=np.zeros(3),
        m=float(log.mass[i]),
        t=float(log.time[i]),
        dry_mass_kg=float(dry_mass_kg),
    )


def _extract_s2_recovery(
    result: FullMissionResult,
    config: SimulationConfig,
) -> dict[str, Any] | None:
    """Summarise the Stage-2 reusable-recovery run: orbit, deorbit, landing."""
    log = result.orbiter_log
    if not log.get_series("time"):
        return None

    orbit_idx = _s2_initial_orbit_index(log)
    s2_dry_mass = float(config.stage2_dry_mass) + float(config.payload_mass)
    orbit = (
        compute_orbit_metrics(_state_from_log(log, orbit_idx, s2_dry_mass))
        if orbit_idx is not None
        else None
    )

    fs = result.orbiter_final_state
    v_touchdown = float(vec_norm(
        compute_ground_relative_velocity(fs.r, fs.v)
    ))
    downrange_km = great_circle_distance_m(
        fs.r, rotating_launch_site_eci(fs.t, config=config)
    ) / 1000.0
    fuel_remaining = max(0.0, float(fs.m - s2_dry_mass))

    return {
        "landing_success": bool(result.orbiter_success),
        "reason": result.orbiter_reason,
        "orbit_achieved": bool(orbit is not None and _assess_s2_initial_orbit(result, config).success),
        "achieved_perigee_km": round(orbit["perigee_alt"] / 1000.0, 1) if orbit else None,
        "achieved_apogee_km": round(orbit["apogee_alt"] / 1000.0, 1) if orbit else None,
        "achieved_eccentricity": round(orbit["ecc"], 4) if orbit else None,
        "touchdown_speed_mps": round(v_touchdown, 2),
        "downrange_km": round(downrange_km, 0),
        "fuel_remaining_kg": round(fuel_remaining, 0),
        "orbit_hold_time_s": _first_phase_time(log, "S2_ORBIT_HOLD"),
        "deorbit_burn_time_s": _first_phase_time(log, "S2_DEORBIT"),
        "entry_interface_time_s": _first_phase_time(log, "S2_ENTRY"),
        "landing_burn_time_s": _first_phase_time(log, "S2_LANDING"),
        "touchdown_time_s": round(float(fs.t), 1),
    }


def _s2_initial_orbit_index(log: SimulationLog) -> int | None:
    """Use the first achieved orbit, or the last insertion sample if it failed."""
    last_insertion = None
    for i, phase in enumerate(log.get_series("phase_name")):
        if phase in ("ORBIT_ACHIEVED", "S2_ORBIT_HOLD"):
            return i
        if phase in ("S2_DEORBIT", "S2_ENTRY", "S2_LANDING"):
            break
        if phase == "ORBIT_INSERTION":
            last_insertion = i
    return last_insertion


def _assess_s2_initial_orbit(result: FullMissionResult, config: SimulationConfig) -> OrbitAssessment:
    """Return orbit assessment based on the insertion state before any deorbit (for s2 recovery runs).
    Prevents final reentry/crash state from overwriting initial orbit success.
    """
    log = result.orbiter_log
    if not log.get_series("time"):
        return assess_orbit(result.orbiter_final_state, config, result.orbiter_reason)
    orbit_idx = _s2_initial_orbit_index(log)
    if orbit_idx is None:
        return assess_orbit(result.orbiter_final_state, config, result.orbiter_reason)
    s2_dry_mass = float(config.stage2_dry_mass) + float(config.payload_mass)
    init_state = _state_from_log(log, orbit_idx, s2_dry_mass)

    return assess_orbit(init_state, config)


def assess_full_mission(
    result: FullMissionResult,
    config: SimulationConfig,
) -> MissionAssessment:
    """Assess a full mission against the strict success gates."""
    orbit = assess_orbit(result.orbiter_final_state, config, result.orbiter_reason)
    landing_speed, site_error = _landing_metrics_from_result(result, config)
    landing = classify_landing(
        landing_speed,
        site_error,
        config,
        propellant_remaining_kg=_booster_propellant_remaining(result, config),
    )
    vertical_speed, horizontal_speed = _landing_velocity_components(result, config)
    landing.vertical_speed_mps = _finite_or_none(vertical_speed)
    landing.horizontal_speed_mps = _finite_or_none(horizontal_speed)

    max_ascent_q = _max_or_none(result.ascent_log.get_series("dynamic_pressure"))
    max_recovery_q = _max_or_none(result.booster_log.get_series("dynamic_pressure"))
    max_recovery_q_alpha = _max_or_none(result.booster_log.get_series("q_alpha"))
    terminal_attitude_error = _terminal_touchdown_tilt_deg(result)
    landing = _landing_with_contact_gates(landing, result, terminal_attitude_error)

    _bst_alt = float(result.booster_final_state.altitude)
    _bst_reason_u = (result.booster_reason or "").upper()
    if result.separation_time is None or _bst_alt > 100.0 or "Q-ALPHA" in _bst_reason_u or ("ABORT" in _bst_reason_u and "TOUCHDOWN" not in _bst_reason_u and "LANDING" not in _bst_reason_u):
        _bst_failed = list(landing.failed_criteria)
        is_q = "Q-ALPHA" in _bst_reason_u
        if is_q and "q_alpha" not in _bst_failed:
            _bst_failed.append("q_alpha")
        elif not is_q and "pre_touchdown_abort" not in _bst_failed:
            _bst_failed.append("pre_touchdown_abort")
        landing = LandingAssessment(
            success=False,
            status="q_alpha_abort" if is_q else "pre_touchdown_failure",
            reason=f"{'Q-ALPHA ABORT' if is_q else 'PRE-TOUCHDOWN FAILURE'} at {_bst_alt/1000.0:.2f} km - {result.booster_reason}",
            touchdown_speed_mps=None,
            site_error_m=None,
            propellant_remaining_kg=landing.propellant_remaining_kg,
            failed_criteria=_bst_failed,
            vertical_speed_mps=None,
            horizontal_speed_mps=None,
            landing_leg_status=landing.landing_leg_status,
            touchdown_contact_status="no_contact",
            terminal_attitude_error_deg=landing.terminal_attitude_error_deg,
        )

    failed: list[str] = []
    separation_occurred = result.separation_time is not None
    if not separation_occurred:
        failed.append("separation")

    s2_recovery = bool(getattr(config, "enable_s2_recovery", False))
    if s2_recovery:

        orbit = _assess_s2_initial_orbit(result, config)
        if not result.orbiter_success:
            failed.append("s2_recovery")
    if not orbit.success:
        failed.append("orbit")
    if not landing.success:
        failed.append("landing")
    if max_ascent_q is not None and max_ascent_q > C.MAX_DYNAMIC_PRESSURE:
        failed.append("ascent_dynamic_pressure")
    if max_recovery_q is not None and max_recovery_q > config.booster_recovery_max_q_pa:
        failed.append("recovery_dynamic_pressure")
    if max_recovery_q_alpha is not None and max_recovery_q_alpha > config.booster_recovery_max_q_alpha_pa_rad:
        failed.append("recovery_q_alpha")
    if _attitude_gate_failed(result, config):
        failed.append("terminal_attitude_error")

    mission_success = not failed
    status = "mission_success" if mission_success else "mission_failed"
    if mission_success:
        reason = "Mission success: orbit insertion and booster recovery passed all configured gates"
    else:
        reason = "Mission failed: " + ", ".join(failed)

    mission_duration_s = max(
        float(result.ascent_final_state.t),
        float(result.orbiter_final_state.t),
        float(result.booster_final_state.t),
    )
    s2_recovery = _extract_s2_recovery(result, config) if s2_recovery else None

    return MissionAssessment(
        mission_success=mission_success,
        status=status,
        reason=reason,
        separation_occurred=separation_occurred,
        orbit=orbit,
        landing=landing,
        max_ascent_q_pa=_finite_or_none(max_ascent_q),
        max_recovery_q_pa=_finite_or_none(max_recovery_q),
        max_recovery_q_alpha_pa_rad=_finite_or_none(max_recovery_q_alpha),
        terminal_attitude_error_deg=_finite_or_none(terminal_attitude_error),
        failed_criteria=failed,
        separation_time_s=_finite_or_none(result.separation_time),
        mission_duration_s=_finite_or_none(mission_duration_s),
        s2_recovery=s2_recovery,
    )


def format_orbit_failure_reason(state: State, reason: str = "maximum time reached") -> str:
    """Return an explicit orbit-insertion failure reason with useful metrics."""
    metrics = compute_orbit_metrics(state)
    propellant = max(0.0, metrics["s2_prop_remaining"])
    return (
        f"ORBIT INSERTION FAILED - {reason}: "
        f"perigee={metrics['perigee_alt'] / 1000.0:.1f} km, "
        f"apogee={metrics['apogee_alt'] / 1000.0:.1f} km, "
        f"e={metrics['ecc']:.4f}, "
        f"propellant={propellant:.0f} kg"
    )


def _assessment_to_dict(assessment: MissionAssessment) -> dict[str, Any]:
    return asdict(assessment)


def _fmt_km(value: float | None) -> str:
    return f"{value:.2f} km" if value is not None else "unavailable"


def _write_markdown_summary(assessment: MissionAssessment, path: Path) -> None:
    a = assessment
    lines = [
        "# Mission Summary",
        "",
        f"Overall status: **{a.status}**",
        f"Reason: {a.reason}",
        "",
        "## Timeline",
        "",
        f"- Stage separation: {_fmt_time(a.separation_time_s)}",
        f"- Mission duration: {_fmt_time(a.mission_duration_s)}",
        "",
    ]

    if a.s2_recovery is not None:
        s2 = a.s2_recovery
        lines += [
            "## Stage 2 — Reusable Upper Stage",
            "",
            (
                f"- Orbit achieved: {s2['orbit_achieved']}  "
                f"(perigee {s2['achieved_perigee_km']} km x apogee {s2['achieved_apogee_km']} km, "
                f"e={s2['achieved_eccentricity']})"
            ),
            f"- Orbit hold begins: {_fmt_time(s2['orbit_hold_time_s'])}",
            f"- Deorbit burn: {_fmt_time(s2['deorbit_burn_time_s'])}",
            f"- Entry interface (70 km): {_fmt_time(s2['entry_interface_time_s'])}",
            f"- Landing burn: {_fmt_time(s2['landing_burn_time_s'])}",
            f"- Drone-ship touchdown: {_fmt_time(s2['touchdown_time_s'])}",
            f"- Touchdown speed: {s2['touchdown_speed_mps']} m/s",
            f"- Downrange: {s2['downrange_km']:.0f} km",
            f"- Fuel remaining: {s2['fuel_remaining_kg']:.0f} kg",
            f"- Landing: {'SUCCESS' if s2['landing_success'] else 'FAILED'}",
            "",
        ]
    else:
        lines += [
            "## Orbit (Stage 2)",
            "",
            f"- Status: {a.orbit.status}",
            f"- Reason: {a.orbit.reason}",
            f"- Perigee: {_fmt_km(a.orbit.perigee_altitude_km)}",
            f"- Apogee: {_fmt_km(a.orbit.apogee_altitude_km)}",
            "- Eccentricity: "
            + (f"{a.orbit.eccentricity:.5f}" if a.orbit.eccentricity is not None else "unavailable"),
            "- Target: " + (f"{a.orbit.target_altitude_km:.1f} km \u00b1 {a.orbit.altitude_tolerance_km:.1f} km" if a.orbit.target_altitude_km is not None else "unavailable"),
            f"- Errors: apo {a.orbit.apogee_error_km} km, peri {a.orbit.perigee_error_km} km" if (a.orbit.apogee_error_km is not None or a.orbit.perigee_error_km is not None) else "",
            "",
        ]

    lines += [
        "## Booster Landing (Stage 1)",
        "",
        f"- Status: {a.landing.status}",
        f"- Reason: {a.landing.reason}",
        "- Touchdown speed: "
        + (f"{a.landing.touchdown_speed_mps:.2f} m/s" if a.landing.touchdown_speed_mps is not None else "unavailable"),
        "- Pad error: "
        + (f"{a.landing.site_error_m:.0f} m" if a.landing.site_error_m is not None else "unavailable"),
        "- Fuel remaining: "
        + (f"{a.landing.propellant_remaining_kg:.0f} kg" if a.landing.propellant_remaining_kg is not None else "unavailable"),
        f"- Landing legs: {a.landing.landing_leg_status or 'unavailable'}",
        f"- Contact status: {a.landing.touchdown_contact_status or a.landing.status}",
        "",
        "## Gates",
        "",
        f"- Separation occurred: {a.separation_occurred}",
        "- Max ascent dynamic pressure: "
        + (f"{a.max_ascent_q_pa:.0f} Pa" if a.max_ascent_q_pa is not None else "unavailable"),
        "- Max recovery dynamic pressure: "
        + (f"{a.max_recovery_q_pa:.0f} Pa" if a.max_recovery_q_pa is not None else "unavailable"),
        "- Max recovery q-alpha: "
        + (f"{a.max_recovery_q_alpha_pa_rad:.0f} Pa-rad" if a.max_recovery_q_alpha_pa_rad is not None else "unavailable"),
        "- Terminal attitude error: "
        + (f"{a.terminal_attitude_error_deg:.2f} deg" if a.terminal_attitude_error_deg is not None else "unavailable"),
        "",
        "Failed criteria: "
        + (", ".join(a.failed_criteria) if a.failed_criteria else "none"),
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def write_mission_summary(
    result: FullMissionResult,
    config: SimulationConfig,
    output_dir: str | Path,
) -> list[str]:
    """Write JSON and Markdown mission summaries beside generated plots."""
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    assessment = assess_full_mission(result, config)

    json_path = output_path / "mission_summary.json"
    md_path = output_path / "mission_summary.md"

    json_path.write_text(
        json.dumps(_assessment_to_dict(assessment), indent=2, sort_keys=True),
        encoding="utf-8",
    )
    _write_markdown_summary(assessment, md_path)
    return [str(json_path), str(md_path)]

"""Booster recovery guidance functions.

Powered-descent references and provenance warning:
    docs/MATHEMATICAL_REFERENCES.md#11-powered-descent-and-recovery-guidance

The implementation is inspired by ZEM/ZEV-style feedback ideas but combines
them with project-specific acceleration caps, bias distances, altitude bands,
capture gates, and throttle limits. It does not reproduce a cited paper's
complete constrained guidance law.
"""

from dataclasses import dataclass

import numpy as np

from . import constants as C
from ._guidance_common import (
    GuidanceState,
    _resolve_guidance_state,
    compute_local_horizontal,
    compute_local_vertical,
)
from ._types import GuidanceOutput
from .config_definition import SimulationConfig
from .forces import (
    compute_configured_atmosphere_properties,
    compute_drag_force,
    compute_gravity_force,
)
from .mass import compute_mass_flow_rate
from .recovery import (
    booster_propellant_remaining,
    compute_entry_energy_speed_gate,
    compute_powered_descent_lead_time,
    estimate_ballistic_apogee,
    estimate_ballistic_impact_to_pad,
    estimate_booster_touchdown_time,
    estimate_recovery_targeting,
    estimate_suicide_burn,
    great_circle_distance_m,
    is_near_pad_target,
    target_landing_site_eci,
)
from .utils import compute_ground_relative_velocity, compute_relative_velocity, vec_norm


@dataclass(frozen=True)
class RecoveryPlanDiagnostics:
    score: float
    miss_m: float
    speed_mps: float
    propellant_kg: float
    max_q_pa: float
    reachable: bool
    required_lateral_accel_mps2: float
    available_lateral_accel_mps2: float
    throttle: float
    time_to_go_s: float


def _planner_unit(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    norm = float(vec_norm(vector))
    if norm < 1e-9:
        return fallback
    return vector / norm


def _planner_horizontal(vector: np.ndarray, normal: np.ndarray) -> np.ndarray:
    return vector - float(np.dot(vector, normal)) * normal


def _planner_mass_flow(
    thrust_n: float,
    throttle: float,
    r: np.ndarray | None = None,
    config: SimulationConfig | None = None,
) -> float:
    scale = float(thrust_n) / max(float(C.THRUST_MAGNITUDE), 1e-9)
    mdot = C.MASS_FLOW_RATE * scale * float(np.clip(throttle, 0.0, 1.0))
    if r is not None:
        altitude = float(vec_norm(r)) - C.R_EARTH
        _, P_amb, _, _ = compute_configured_atmosphere_properties(altitude, config)
        pressure_ratio = float(np.clip(P_amb / C.ATM_P0, 0.0, 1.2))
        dynamic_isp = C.ISP_VAC - (C.ISP_VAC - C.ISP) * pressure_ratio
        mdot *= C.ISP / max(dynamic_isp, 1e-9)
    return mdot


def _candidate_directions(
    r: np.ndarray,
    v: np.ndarray,
    t: float,
    phase: str,
    config: SimulationConfig,
    current_direction: np.ndarray | None,
) -> list[np.ndarray]:
    vertical = _planner_unit(r, np.array([1.0, 0.0, 0.0]))
    v_rel = compute_relative_velocity(
        r,
        v,
        wind_offset_mps=float(getattr(config, "runtime_wind_offset_mps", 0.0)),
    )
    retrograde = -_planner_unit(v_rel, -vertical)
    target = target_landing_site_eci(
        t + (160.0 if phase == "BOOSTER_BOOSTBACK" else 80.0),
        float(config.booster_landing_target_downrange_km),
        config=config,
    )
    target_vertical = _planner_unit(target, vertical)
    toward_site = _planner_unit(_planner_horizontal(target - r, target_vertical), retrograde)
    v_h = _planner_horizontal(v_rel, target_vertical)
    anti_drift = -_planner_unit(v_h, toward_site)
    candidates = [
        retrograde,
        _planner_unit(0.90 * retrograde + 0.10 * toward_site, retrograde),
        _planner_unit(0.80 * retrograde + 0.20 * toward_site, retrograde),
        _planner_unit(0.75 * retrograde + 0.25 * anti_drift, retrograde),
        _planner_unit(0.60 * vertical + 0.65 * toward_site + 0.35 * anti_drift, vertical),
        _planner_unit(0.70 * vertical + 0.70 * anti_drift, vertical),
    ]
    if current_direction is not None:
        candidates.insert(0, _planner_unit(np.asarray(current_direction, dtype=float), retrograde))
    return candidates


def _candidate_throttles(phase: str, current_throttle: float) -> tuple[float, ...]:
    current = float(np.clip(current_throttle, 0.0, 1.0))
    values = (current, 0.0, 0.45, 0.70, 0.90) if phase == "BOOSTER_BOOSTBACK" else (current, 0.0, 0.18, 0.35, 0.55)
    deduped: list[float] = []
    for value in values:
        if all(abs(value - existing) > 1e-6 for existing in deduped):
            deduped.append(value)
    return tuple(deduped)


def _propagate_candidate(
    r0: np.ndarray,
    v0: np.ndarray,
    t0: float,
    m0: float,
    direction: np.ndarray,
    throttle: float,
    thrust_n: float,
    config: SimulationConfig,
    horizon_s: float,
    step_s: float,
) -> tuple[np.ndarray, np.ndarray, float, float, float, bool]:
    r = np.asarray(r0, dtype=float).copy()
    v = np.asarray(v0, dtype=float).copy()
    m = float(m0)
    t = float(t0)
    dry_mass = float(config.stage1_dry_mass)
    direction = _planner_unit(np.asarray(direction, dtype=float), -_planner_unit(v0, np.array([1.0, 0.0, 0.0])))
    throttle = float(np.clip(throttle, 0.0, 1.0))
    max_q = 0.0
    hit_ground = False
    elapsed = 0.0
    while elapsed < horizon_s:
        altitude = float(vec_norm(r) - C.R_EARTH)
        if altitude <= 0.0:
            hit_ground = True
            break
        dt = min(step_s, horizon_s - elapsed)
        v_rel = compute_relative_velocity(
            r,
            v,
            wind_offset_mps=float(getattr(config, "runtime_wind_offset_mps", 0.0)),
        )
        _, _, rho, _ = compute_configured_atmosphere_properties(altitude, config)
        max_q = max(max_q, 0.5 * rho * float(np.dot(v_rel, v_rel)))
        thrust_on = throttle > 0.01 and m > dry_mass + 1e-6
        thrust = direction * (thrust_n * throttle) if thrust_on else np.zeros(3)
        gravity = compute_gravity_force(
            r,
            m,
            enable_j2=bool(getattr(config, "enable_j2", False)),
            j2=float(getattr(config, "j2_coefficient", 1.08263e-3)),
        )
        drag = compute_drag_force(
            r,
            v,
            wind_offset_mps=float(getattr(config, "runtime_wind_offset_mps", 0.0)),
            config=config,
        )
        a = (gravity + drag + thrust) / max(m, 1.0)
        v = v + a * dt
        r = r + v * dt
        if thrust_on:
            m = max(dry_mass, m - _planner_mass_flow(thrust_n, throttle, r, config) * dt)
        t += dt
        elapsed += dt
    return r, v, m, t, max_q, hit_ground


def score_recovery_candidates(
    r: np.ndarray,
    v: np.ndarray,
    t: float,
    m: float,
    phase: str,
    config: SimulationConfig,
    *,
    current_direction: np.ndarray | None = None,
    current_throttle: float = 0.0,
) -> RecoveryPlanDiagnostics:
    phase_name = str(phase).upper()
    thrust_n = C.BOOSTBACK_THRUST if phase_name == "BOOSTER_BOOSTBACK" else C.ENTRY_THRUST
    horizon = 180.0 if phase_name == "BOOSTER_BOOSTBACK" else 100.0
    step = 6.0 if phase_name == "BOOSTER_BOOSTBACK" else 4.0
    reserve = float(config.booster_landing_reserve_kg + config.booster_landing_propellant_margin_kg)
    best: RecoveryPlanDiagnostics | None = None
    for direction in _candidate_directions(r, v, t, phase_name, config, current_direction):
        for throttle in _candidate_throttles(phase_name, current_throttle):
            rf, vf, mf, tf, max_q, hit_ground = _propagate_candidate(
                r, v, t, m, direction, throttle, thrust_n, config, horizon, step,
            )
            target = target_landing_site_eci(tf, float(config.booster_landing_target_downrange_km), config=config)
            miss = great_circle_distance_m(rf, target)
            v_rel_f = compute_relative_velocity(
                rf,
                vf,
                wind_offset_mps=float(getattr(config, "runtime_wind_offset_mps", 0.0)),
            )
            speed = float(vec_norm(v_rel_f))
            prop = max(0.0, float(mf) - float(config.stage1_dry_mass))
            time_to_go = max(float(tf - t), 1.0)
            required_lat = 2.0 * miss / (time_to_go ** 2)
            accel_total = thrust_n / max(mf, 1.0)
            available_lat = max(0.0, 0.45 * max(accel_total - C.G0, 0.0))
            reachable = required_lat <= max(available_lat, 0.5)
            fuel_shortfall = max(0.0, reserve - prop)
            max_q_excess = max(0.0, max_q - float(config.booster_recovery_max_q_pa))
            speed_penalty = max(0.0, speed - (8.0 if hit_ground else 140.0))
            unreachable_penalty = 0.0 if reachable else miss
            score = (
                miss
                + 70.0 * speed_penalty
                + 10.0 * fuel_shortfall
                + 0.8 * max_q_excess
                + 250.0 * max(0.0, required_lat - available_lat)
                + unreachable_penalty
                + 900.0 * throttle
            )
            candidate = RecoveryPlanDiagnostics(
                score=float(score),
                miss_m=float(miss),
                speed_mps=float(speed),
                propellant_kg=float(prop),
                max_q_pa=float(max_q),
                reachable=bool(reachable),
                required_lateral_accel_mps2=float(required_lat),
                available_lateral_accel_mps2=float(available_lat),
                throttle=float(throttle),
                time_to_go_s=float(time_to_go),
            )
            if best is None or candidate.score < best.score:
                best = candidate
    assert best is not None
    return best


def divert_fraction_by_altitude(h: float, config: SimulationConfig) -> float:
    """Altitude-banded base divert fraction.

    Returns the share of the vertical-acceleration budget that may be spent on
    horizontal pad-targeting correction at altitude ``h`` (m AGL). Shared by the
    ZEM/ZEV and ZEV-only branches so the two never drift apart.
    """
    band_high = float(config.booster_divert_band_high_alt_m)
    band_low = float(config.booster_divert_band_low_alt_m)
    frac_high = float(config.booster_divert_fraction_high)
    frac_mid = float(config.booster_divert_fraction_mid)
    frac_low = float(config.booster_divert_fraction_low)

    if h > band_high:
        return frac_high
    if h > band_low:
        return frac_mid
    return frac_low


def compute_landing_reserve_authority(
    propellant_remaining_kg: float,
    full_throttle_mass_flow_kg_s: float,
    vertical_throttle_fraction: float,
    time_to_go_s: float,
    landing_reserve_kg: float,
    landing_margin_kg: float,
    safety_factor: float,
) -> tuple[float, float]:
    """Return lateral authority fraction after reserving vertical-arrest fuel."""
    predicted_vertical_propellant = (
        max(float(full_throttle_mass_flow_kg_s), 0.0)
        * float(np.clip(vertical_throttle_fraction, 0.0, 1.0))
        * max(float(time_to_go_s), 0.0)
        * max(float(safety_factor), 1.0)
    )
    protected_requirement = predicted_vertical_propellant + max(
        float(landing_margin_kg),
        0.0,
    )
    surplus = float(propellant_remaining_kg) - protected_requirement
    authority_band = max(0.25 * float(landing_reserve_kg), 250.0)
    authority = float(np.clip(surplus / authority_band, 0.0, 1.0))
    return authority, protected_requirement


def _limit_direction_cone(
    direction: np.ndarray,
    reference: np.ndarray,
    max_angle_rad: float,
) -> np.ndarray:
    """Limit a unit direction to a cone about a unit reference direction."""
    direction = direction / max(float(vec_norm(direction)), 1e-9)
    reference = reference / max(float(vec_norm(reference)), 1e-9)
    cosine = float(np.clip(np.dot(direction, reference), -1.0, 1.0))
    angle = float(np.arccos(cosine))
    if angle <= max_angle_rad:
        return direction

    transverse = direction - cosine * reference
    transverse_norm = float(vec_norm(transverse))
    if transverse_norm <= 1e-9:
        return reference
    transverse /= transverse_norm
    limited = np.cos(max_angle_rad) * reference + np.sin(max_angle_rad) * transverse
    return limited / max(float(vec_norm(limited)), 1e-9)


def _compute_boostback_direction(
    r: np.ndarray,
    v: np.ndarray,
    t: float,
    target_downrange_km: float,
    m: float,
    apogee_target_m: float = 160000.0,
    config: SimulationConfig | None = None,
) -> np.ndarray:
    """
    Compute boostback thrust direction for RTLS site-targeting.

    Physics background
    ------------------
    After stage separation the booster carries large downrange (eastward)
    horizontal velocity from the ascent.  Simply opposing that velocity
    (anti-v_horiz) leaves the booster stranded far downrange when the burn
    ends, because it was still moving eastward throughout the deceleration
    and has built up tens of kilometres of additional downrange distance.

    Correct RTLS guidance requires the burn to *overshoot* through zero
    horizontal speed and impart a small return velocity so the subsequent
    ballistic coast carries the booster back over the landing pad.

    Algorithm: 2-D velocity-targeting law
    --------------------------------------
    The desired horizontal velocity at burn cut-off is a full 2-D vector:

        v_desired_h  =  v_return  *  toward_site_future

    where toward_site_future points from the booster to the FUTURE (rotated)
    pad position at predicted touchdown, and v_return is the scalar magnitude
    needed to cover the horizontal distance in the remaining coast time.

    At each guidance call the thrust direction is aligned with the
    *horizontal velocity error*:

        thrust_dir  =  normalise( v_desired_h  −  v_horiz_current )

    Since v_desired_h has zero cross-track component, this law simultaneously
    closes the toward-site velocity error AND zeros any cross-track velocity
    that accumulated from launch-azimuth geometry.  Both components converge
    to zero simultaneously under the proportional control.

    The cutoff is driven by the ballistic impact predictor (< 1.5 km miss),
    not the velocity criterion.  This ensures the burn continues the extra
    fraction of a second needed to zero cross-track after toward-site
    velocity converges — the velocity criterion could fire prematurely
    while 50+ m/s of cross-track remained, causing a 15–17 km lateral miss.

    A secondary downward component is blended in proportionally when the
    predicted ballistic apogee significantly exceeds the cap.
    """
    vertical = compute_local_vertical(r)
    r_norm = float(vec_norm(r))
    altitude = r_norm - C.R_EARTH
    g_local = C.MU_EARTH / max(r_norm ** 2, 1.0)
    v_norm = float(vec_norm(v))
    v_radial = float(np.dot(v, vertical))
    v_horiz_vec = v - v_radial * vertical
    v_horiz_mag = float(vec_norm(v_horiz_vec))


    h_apogee_pred = estimate_ballistic_apogee(altitude, v_radial, g_local)
    _cfg = config or SimulationConfig()
    _lead = compute_powered_descent_lead_time(r, v, m, _cfg) + 22.4
    targeting = estimate_recovery_targeting(
        r,
        v,
        t,
        target_downrange_km,
        config=_cfg,
        lead_time_s=_lead,
    )
    site_dist = targeting.site_distance_m
    toward_site = targeting.toward_site
    v_return = targeting.v_return_needed_mps


    v_desired_h = v_return * toward_site


    v_guidance_h = v_horiz_vec
    v_error = v_desired_h - v_guidance_h
    v_error_mag = float(vec_norm(v_error))

    if v_error_mag > 1.0:
        thrust_dir = v_error / v_error_mag
        thrust_dir = thrust_dir - float(np.dot(thrust_dir, vertical)) * vertical
        t_dir_norm = float(vec_norm(thrust_dir))
        if t_dir_norm > 1e-9:
            thrust_dir = thrust_dir / t_dir_norm
        else:
            thrust_dir = -v / max(v_norm, 1.0)
    elif v_horiz_mag > 5.0:
        if site_dist > 100.0:
            thrust_dir = toward_site
        else:
            thrust_dir = -v_guidance_h / max(v_horiz_mag, 1e-9)
    else:
        thrust_dir = -v / max(v_norm, 1.0)


    apogee_excess = max(0.0, h_apogee_pred - apogee_target_m)
    if v_radial > 0.0 and apogee_excess > 12000.0:

        down_weight = float(np.clip(apogee_excess / 180000.0, 0.0, 0.42))
        thrust_dir = (1.0 - down_weight) * thrust_dir + down_weight * (-vertical)
        thrust_dir /= max(float(vec_norm(thrust_dir)), 1e-6)

    return thrust_dir


@dataclass(frozen=True)
class _BoosterContext:
    """Per-call quantities shared by the booster phase guidance helpers."""

    r: np.ndarray
    v: np.ndarray
    t: float
    m: float
    phase: str
    cfg: SimulationConfig
    altitude: float
    vertical: np.ndarray
    retrograde: np.ndarray
    v_ground: np.ndarray
    v_rel_norm: float
    anti_wind: np.ndarray
    propellant_remaining: float
    landing_reserve: float
    landing_margin: float
    protected_landing_propellant: float
    burn_prediction: dict
    target_downrange_km: float
    near_pad_rtls: bool
    landing_site_now: np.ndarray


def _horizontal_component(vec: np.ndarray, axis: np.ndarray) -> np.ndarray:
    return vec - float(np.dot(vec, axis)) * axis


def _ballistic_impact(ctx: _BoosterContext):
    return estimate_ballistic_impact_to_pad(
        ctx.r,
        ctx.v,
        ctx.t,
        ctx.target_downrange_km,
        ctx.cfg,
        lead_time_s=0.0,
        mass_kg=ctx.m,
        aero_mode=ctx.phase,
    )


def _entry_guidance(ctx: _BoosterContext) -> dict:
    """BOOSTER_ENTRY: retrograde entry burn plus terminal pad capture."""
    cfg = ctx.cfg
    altitude = ctx.altitude
    vertical = ctx.vertical
    retrograde = ctx.retrograde
    near_pad_rtls = ctx.near_pad_rtls
    entry_min_alt = cfg.booster_entry_burn_min_altitude_m
    entry_interface = cfg.booster_entry_interface_altitude_m

    v_vert_rel = float(np.dot(ctx.v_ground, vertical))
    landing_target_lead_time = 0.0
    grid_fin_zero_effort_miss = None
    capture_accel_cmd_mag = None
    thrust_on = False
    throttle = 0.0

    predicted_site = ctx.landing_site_now
    site_vertical = predicted_site / max(float(vec_norm(predicted_site)), 1.0)
    r_to_site_horiz = _horizontal_component(predicted_site - ctx.r, site_vertical)
    site_dist = float(vec_norm(r_to_site_horiz))
    predicted_landing_error = site_dist
    v_horiz_site = _horizontal_component(ctx.v_ground, site_vertical)
    v_horiz_site_mag = float(vec_norm(v_horiz_site))
    target_entry_descent_rate = 150.0
    # Time-to-go for the entry-descent lateral guidance must reflect the
    # vehicle's ACTUAL remaining flight time, not altitude/assumed-descent-rate.
    # The previous estimate (1.2*altitude/150 m/s, clipped to 180 s) gave
    # ~57 s at entry interface while ~170 s of flight remained, so the ZEM/ZEV
    # gain 6/t_go^2 was 5-9x too high: the grid fins demanded far more lateral
    # acceleration than the trajectory could absorb, saturated at full
    # deflection for most of the descent, and mis-steer -- leaving a residual
    # ~1.1 km pad miss after the corrected lift model. Use the ballistic
    # time-to-impact from the impact predictor (which integrates the real
    # drag-affected descent) and fall back to the old estimate only if that
    # predictor is unavailable. The estimate is floored so t_go can never
    # collapse to the 2 s clip in command_grid_fins_to_target.
    landing_guidance_time_to_go_s = float(np.clip(
        1.2 * altitude / target_entry_descent_rate,
        15.0,
        180.0,
    ))
    impact_miss_for_capture = site_dist
    impact = None  # ballistic prediction; computed at most once per call
    if site_dist > 500.0:
        toward_site = r_to_site_horiz / site_dist
        anti_drift = -v_horiz_site / max(v_horiz_site_mag, 1e-9)
        if near_pad_rtls:
            impact = _ballistic_impact(ctx)
            impact_miss_for_capture = float(impact.miss_distance_m)
            predicted_landing_error = impact_miss_for_capture
            grid_fin_zero_effort_miss = impact.miss_vector_m.copy()
            # Real ballistic time-to-impact drives the lateral guidance
            # time-to-go for the whole entry descent (see note above).
            landing_guidance_time_to_go_s = float(np.clip(
                impact.time_to_impact_s,
                15.0,
                180.0,
            ))
            if impact.miss_distance_m > 500.0:
                impact_toward = impact.miss_vector_m / max(impact.miss_distance_m, 1e-9)
                impact_toward = _horizontal_component(impact_toward, site_vertical)
                impact_toward_norm = float(vec_norm(impact_toward))
                if impact_toward_norm > 1e-9:
                    impact_toward /= impact_toward_norm
                    predictor_weight = float(np.clip(impact.miss_distance_m / 10000.0, 0.15, 0.55))
                    toward_site = (1.0 - predictor_weight) * toward_site + predictor_weight * impact_toward
                    toward_site /= max(float(vec_norm(toward_site)), 1e-9)
                    landing_target_lead_time = impact.time_to_impact_s

        alt_factor = float(np.clip(
            (altitude - entry_min_alt) / max(entry_interface - entry_min_alt, 1.0),
            0.0, 1.0
        ))
        max_site_bias = 0.55
        if near_pad_rtls:
            max_site_bias = 0.90
            site_bias = float(np.clip(
                float(site_dist) / 7000.0,
                0.0, 0.95,
            )) * (0.7 + 0.3 * alt_factor)
        else:
            site_bias = float(np.clip(site_dist / 30000.0, 0.0, max_site_bias)) * (0.6 + 0.4 * alt_factor)
        site_bias = float(np.clip(site_bias, 0.0, max_site_bias))
        approach_v = float(np.dot(v_horiz_site, toward_site)) if v_horiz_site_mag > 1e-9 else 0.0
        a_brake_entry = 24.0
        stop_dist_total = v_horiz_site_mag ** 2 / (2.0 * a_brake_entry)
        usable_site_dist = max(site_dist - float(cfg.booster_pad_tolerance_m), 0.0)
        needs_horizontal_brake = (
            approach_v > 0.0 and
            stop_dist_total >= (0.85 if near_pad_rtls else 0.40) * usable_site_dist
        )
        lateral_dir = anti_drift if needs_horizontal_brake else toward_site
        desired_dir = (1.0 - site_bias) * retrograde + site_bias * lateral_dir
        desired_dir /= max(float(vec_norm(desired_dir)), 1e-9)
        cone_reference = (
            vertical
            if near_pad_rtls and altitude < float(cfg.booster_late_entry_bias_ref_altitude_m)
            else retrograde
        )
        desired_dir = _limit_direction_cone(
            desired_dir,
            cone_reference,
            np.radians(30.0),
        )
    else:
        desired_dir = retrograde

    entry_burn_on = (
        altitude > entry_min_alt and
        ctx.v_rel_norm > compute_entry_energy_speed_gate(cfg) and
        ctx.propellant_remaining > ctx.protected_landing_propellant
    )

    if near_pad_rtls:
        terminal_capture_floor = max(
            float(cfg.booster_terminal_capture_floor_kg),
            ctx.landing_margin
            + float(cfg.booster_terminal_capture_landing_reserve_fraction)
            * ctx.landing_reserve,
        )
    else:
        terminal_capture_floor = max(
            ctx.protected_landing_propellant, 0.52 * ctx.landing_reserve
        )
    terminal_capture_on = (
        altitude < 70000.0 and
        site_dist < (60000.0 if near_pad_rtls else 25000.0) and
        (
            impact_miss_for_capture < 80000.0
            if near_pad_rtls
            else True
        ) and
        (
            (v_horiz_site_mag > 1.0 or site_dist > 150.0)
            if near_pad_rtls
            else (v_horiz_site_mag > 5.0)
        ) and
        ctx.propellant_remaining > terminal_capture_floor
    )

    if terminal_capture_on:
        thrust_on = True
        terminal_capture_coast = False
        toward_site = r_to_site_horiz / max(site_dist, 1e-9)
        anti_drift = -v_horiz_site / max(v_horiz_site_mag, 1e-9)
        site_dist_for_throttle = site_dist
        if near_pad_rtls:
            g_local = C.MU_EARTH / max(float(vec_norm(ctx.r)) ** 2, 1.0)
            if impact is None:
                impact = _ballistic_impact(ctx)
            impact_miss = impact.miss_vector_m
            impact_miss_mag = max(float(vec_norm(impact_miss)), 0.0)
            predicted_landing_error = impact_miss_mag
            grid_fin_zero_effort_miss = impact_miss.copy()
            landing_target_lead_time = impact.time_to_impact_s
            site_dist_for_throttle = min(site_dist, impact_miss_mag)
            t_go_capture = float(np.clip(
                max(
                    impact.time_to_impact_s,
                    landing_guidance_time_to_go_s,
                ),
                8.0,
                180.0,
            ))
            a_divert = (
                (6.0 / (t_go_capture ** 2)) * impact_miss
                + (2.0 / t_go_capture) * v_horiz_site
            )
            a_divert_mag = float(vec_norm(a_divert))
            if a_divert_mag > 1e-9:
                max_divert_accel = (
                    float(cfg.booster_max_divert_accel_far_mps2)
                    if impact_miss_mag > float(cfg.booster_divert_accel_miss_threshold_m)
                    else float(cfg.booster_max_divert_accel_near_mps2)
                )
                net_vertical_braking = (
                    max(-v_vert_rel, 0.0) ** 2
                    - target_entry_descent_rate ** 2
                ) / (2.0 * max(altitude, 1.0))
                a_vertical_capture = max(
                    g_local + net_vertical_braking,
                    0.0,
                )
                max_divert_accel = min(
                    max_divert_accel,
                    a_vertical_capture * np.tan(np.radians(30.0)),
                )
                if a_divert_mag > max_divert_accel:
                    a_divert *= max_divert_accel / a_divert_mag
                a_cmd_capture = a_vertical_capture * vertical + a_divert
                capture_accel_cmd_mag = float(vec_norm(a_cmd_capture))
                desired_dir = a_cmd_capture / max(capture_accel_cmd_mag, 1e-9)
            else:
                desired_dir = vertical
                terminal_capture_coast = True
        else:
            # Anti-wind term trims the commanded lateral direction against the
            # wind drift that the low-altitude lateral authority cannot fully
            # null. Weight is modest: this supplements the ZEM/ZEV miss
            # correction rather than replacing it, and is a no-op at zero wind.
            desired_dir = (
                0.50 * vertical
                + 0.62 * toward_site
                + 0.65 * anti_drift
                + 0.25 * ctx.anti_wind
            )
        desired_dir /= max(float(vec_norm(desired_dir)), 1e-9)
        if near_pad_rtls:
            if terminal_capture_coast:
                max_capture_throttle = 0.10
            elif altitude > 500.0 and ctx.propellant_remaining >= terminal_capture_floor + 1200.0:
                max_capture_throttle = 0.85
            elif ctx.propellant_remaining < terminal_capture_floor + 1200.0:
                max_capture_throttle = 0.55
            else:
                max_capture_throttle = 0.65
            if capture_accel_cmd_mag is not None:
                throttle_request = capture_accel_cmd_mag / max(
                    C.ENTRY_THRUST / max(ctx.m, 1.0),
                    1e-9,
                )
            else:
                throttle_request = max(
                    v_horiz_site_mag / 120.0,
                    site_dist_for_throttle / 8000.0,
                )
            throttle = float(np.clip(
                throttle_request,
                0.05,
                max_capture_throttle,
            ))
        else:
            throttle = float(np.clip(
                max(v_horiz_site_mag / 300.0, site_dist / 80000.0),
                0.04,
                0.24,
            ))
    elif entry_burn_on:
        thrust_on = True
        throttle = 0.45

    return {
        'desired_dir': desired_dir,
        'thrust_on': thrust_on,
        'throttle': throttle,
        'landing_target_lead_time': landing_target_lead_time,
        'landing_guidance_time_to_go_s': landing_guidance_time_to_go_s,
        'predicted_landing_error': predicted_landing_error,
        'grid_fin_zero_effort_miss': grid_fin_zero_effort_miss,
    }


def _landing_guidance(ctx: _BoosterContext, gs: GuidanceState) -> dict:
    """BOOSTER_LANDING: latched suicide burn with budgeted ZEM/ZEV divert."""
    cfg = ctx.cfg
    vertical = ctx.vertical
    burn_params = ctx.burn_prediction
    near_pad_rtls = ctx.near_pad_rtls
    landing_reserve = ctx.landing_reserve
    landing_margin = ctx.landing_margin

    ignition_corridor_top = min(
        cfg.booster_landing_ignition_ceiling_m,
        float(burn_params.get('burn_altitude', 0.0)) * cfg.booster_landing_ignition_safety_factor,
    )
    if ctx.altitude <= ignition_corridor_top:
        gs.booster_landing_burn_started = True
    if not gs.booster_landing_burn_started:
        return {'desired_dir': ctx.retrograde, 'thrust_on': False, 'throttle': 0.0}

    v_vert_rel = float(np.dot(ctx.v_ground, vertical))
    v_descent = max(-v_vert_rel, 0.0)
    v_horiz_vec = _horizontal_component(ctx.v_ground, vertical)
    v_horiz_mag = float(vec_norm(v_horiz_vec))

    g_loc = float(C.MU_EARTH / (float(vec_norm(ctx.r)) ** 2))
    t_accel = float(C.LANDING_THRUST / max(ctx.m, 1.0))
    h = max(ctx.altitude, 0.5)

    target_descent_ceiling = 4.2 if near_pad_rtls else 3.0
    landing_target_descent_rate = min(
        target_descent_ceiling,
        0.85 * float(cfg.landing_leg_max_touchdown_speed_mps),
    )

    if h < 10.0 and v_descent < landing_target_descent_rate:
        a_vert_raw = 0.0
    else:
        a_vert_raw = (
            g_loc
            + (v_descent ** 2 - landing_target_descent_rate ** 2)
            / (2.0 * h)
        )
    a_vert_needed = float(np.clip(a_vert_raw, 0.0, t_accel))

    t_go = max(
        estimate_booster_touchdown_time(
            ctx.r,
            ctx.v,
            ctx.m,
            C.LANDING_THRUST,
            safety_factor=1.0,
        ),
        1.0,
    )

    touchdown_site = ctx.landing_site_now
    site_vertical = touchdown_site / max(float(vec_norm(touchdown_site)), 1.0)
    r_err_horiz = _horizontal_component(touchdown_site - ctx.r, site_vertical)
    v_horiz_site = _horizontal_component(v_horiz_vec, site_vertical)
    v_horiz_site_mag = float(vec_norm(v_horiz_site))

    a_vert_for_budget = min(a_vert_needed, t_accel)
    a_avail_horiz = float(np.sqrt(
        max(t_accel ** 2 - a_vert_for_budget ** 2, 0.0)
    ))
    zem = r_err_horiz - v_horiz_site * t_go
    pad_error_m = float(vec_norm(r_err_horiz))
    pad_tolerance = float(cfg.booster_pad_tolerance_m)
    terminal_inside_pad_capture = pad_error_m <= pad_tolerance and h < 150.0
    terminal_horizontal_capture = (
        near_pad_rtls
        and terminal_inside_pad_capture
        and v_horiz_site_mag > 3.0
    )
    a_divert_zev = -(2.0 / t_go) * v_horiz_site
    a_divert_zem_zev = (6.0 / (t_go ** 2)) * zem + (2.0 / t_go) * v_horiz_site
    base_divert_fraction = divert_fraction_by_altitude(h, cfg)

    fuel_margin_ratio = max(ctx.propellant_remaining - landing_margin, 0.0) / max(landing_reserve, 1.0)
    vertical_load_ratio = a_vert_needed / max(t_accel, 1e-6)
    terminal_vertical_risk = (
        vertical_load_ratio > float(cfg.booster_vertical_load_risk_ratio)
        or (
            h < 15.0
            and v_descent > float(cfg.booster_terminal_descent_overspeed_factor) * landing_target_descent_rate
        )
    )
    altitude_margin_low = (
        h < max(250.0, 0.55 * burn_params['burn_altitude'])
        and terminal_vertical_risk
    )
    fuel_critical_for_vertical = (
        fuel_margin_ratio < float(cfg.booster_fuel_critical_ratio)
        and (h > 30.0 or terminal_vertical_risk)
    )
    landing_vertical_priority = (
        fuel_critical_for_vertical
        or vertical_load_ratio > 0.85
        or altitude_margin_low
    )
    if terminal_inside_pad_capture and not landing_vertical_priority:
        base_divert_fraction = max(
            base_divert_fraction,
            float(cfg.booster_divert_fraction_high),
        )
    if landing_vertical_priority:
        base_divert_fraction *= 0.25 if (fuel_critical_for_vertical or h < 500.0) else 0.60

    if vertical_load_ratio > 0.90 and h < 100.0:
        a_horiz_budget = 0.0
    else:
        a_horiz_budget = min(a_avail_horiz, max(a_vert_for_budget, g_loc) * base_divert_fraction)

    full_throttle_mdot = -compute_mass_flow_rate(
        thrust_on=True,
        throttle=1.0,
        stage=1,
        thrust_magnitude_override=C.LANDING_THRUST,
        thrust_scale=cfg.runtime_thrust_scale,
        isp_scale=cfg.runtime_isp_scale,
        r=ctx.r,
    )
    (
        landing_reserve_authority,
        landing_vertical_propellant_requirement,
    ) = compute_landing_reserve_authority(
        ctx.propellant_remaining,
        full_throttle_mdot,
        vertical_load_ratio,
        t_go,
        landing_reserve,
        landing_margin,
        cfg.booster_landing_vertical_propellant_safety_factor,
    )
    a_horiz_budget *= landing_reserve_authority
    if landing_reserve_authority <= 0.0:
        landing_vertical_priority = True

    if terminal_horizontal_capture and not terminal_vertical_risk:
        a_horiz_budget = min(
            a_avail_horiz,
            max(a_horiz_budget, min(35.0, 0.75 * t_accel)),
        )
        a_horiz_budget *= landing_reserve_authority
    reachable_distance = v_horiz_mag * t_go + 0.5 * a_horiz_budget * (t_go ** 2)
    pad_reachable = pad_error_m <= max(350.0, reachable_distance)

    if not pad_reachable and not terminal_inside_pad_capture:
        divert_fraction = divert_fraction_by_altitude(h, cfg)
        if landing_vertical_priority:
            divert_fraction *= 0.25
        a_horiz_budget = min(a_avail_horiz, max(a_vert_for_budget, g_loc) * divert_fraction)

    t_stop = float(np.clip(
        t_go,
        float(cfg.booster_terminal_hcapture_tstop_min_s),
        float(cfg.booster_terminal_hcapture_tstop_max_s),
    ))
    if terminal_horizontal_capture:
        a_divert = -v_horiz_site / max(t_stop, 1e-6)
    elif terminal_inside_pad_capture:
        a_divert = a_divert_zev
    else:
        a_divert = a_divert_zem_zev if pad_reachable else a_divert_zev

    a_divert_mag = float(vec_norm(a_divert))
    if a_divert_mag > a_horiz_budget and a_divert_mag > 1e-6:
        a_divert = a_divert * (a_horiz_budget / a_divert_mag)

    if v_descent < 1.0 and v_horiz_mag < 1.0:
        throttle = 0.0
        desired_dir = vertical
    else:
        if terminal_horizontal_capture:
            v_terminal = v_vert_rel * vertical + v_horiz_site
            a_cmd = g_loc * vertical - v_terminal / max(t_stop, 1e-6)
        else:
            a_cmd = a_vert_needed * vertical + a_divert
        a_cmd_mag = float(vec_norm(a_cmd))
        desired_dir = a_cmd / max(a_cmd_mag, 1e-6)
        throttle = float(np.clip(a_cmd_mag / max(t_accel, 1e-6), 0.0, 1.0))
    if h < 10.0 and v_descent < landing_target_descent_rate:
        desired_dir = vertical
        throttle = 0.0

    # Landing-leg stability is a hard vehicle constraint.  Keep the
    # commanded thrust axis inside the configured touchdown cone while
    # retaining the ZEM/ZEV lateral direction within that cone.
    touchdown_cone_deg = min(
        float(cfg.booster_terminal_attitude_error_max_deg),
        float(cfg.landing_leg_max_tilt_deg),
    )
    desired_dir = _limit_direction_cone(
        desired_dir,
        vertical,
        np.radians(touchdown_cone_deg),
    )

    return {
        'desired_dir': desired_dir,
        'thrust_on': True,
        'throttle': throttle,
        'landing_vertical_priority': landing_vertical_priority,
        'landing_a_vert_needed': a_vert_needed,
        'landing_a_horiz_budget': a_horiz_budget,
        'landing_target_descent_rate': landing_target_descent_rate,
        'predicted_landing_error': pad_error_m,
        'landing_reserve_authority': landing_reserve_authority,
        'landing_vertical_propellant_requirement': landing_vertical_propellant_requirement,
    }


def compute_booster_guidance(
    r: np.ndarray, v: np.ndarray, t: float, m: float, phase: str,
    gs: GuidanceState | None = None,
    config: SimulationConfig | None = None,
) -> tuple[GuidanceOutput, GuidanceState]:
    """
    Guidance logic for Booster Recovery (Phase III-B).

    Physics-based guidance for each sub-phase:
    FLIP, BOOSTBACK, COAST, ENTRY, LANDING.
    """
    gs = _resolve_guidance_state(gs)
    if config is None:
        from .config_factory import create_default_config
        config = create_default_config()
    cfg = config

    altitude = float(vec_norm(r) - C.R_EARTH)
    wind_offset = float(cfg.runtime_wind_offset_mps)
    v_rel = compute_relative_velocity(r, v, wind_offset_mps=wind_offset)
    v_rel_norm = float(vec_norm(v_rel))
    vertical = compute_local_vertical(r)
    prograde = v_rel / v_rel_norm if v_rel_norm > 1e-6 else vertical
    retrograde = -prograde

    # Wind feed-forward for lateral guidance.
    # The zero-effort-miss predictor already sees the wind (it propagates with
    # compute_drag_force(wind_offset_mps=...)), so ZEM/ZEV already compensates in
    # principle. But the *steering* decision compares ground-relative geometry
    # against a target, and during entry the residual lateral velocity is
    # dominated by wind drift that the thruster/fin authority can only partly
    # null at low dynamic pressure. Exposing the wind-drift velocity explicitly
    # lets the entry/landing blend trim against it instead of reacting late.
    anti_wind = np.zeros(3, dtype=float)
    if abs(wind_offset) > 1e-9:
        from .utils import _wind_vector
        wind_drift_velocity = _wind_vector(r, wind_offset_mps=wind_offset)
        wind_drift_norm = float(vec_norm(wind_drift_velocity))
        if wind_drift_norm > 1e-9:
            anti_wind = -wind_drift_velocity / wind_drift_norm

    landing_reserve = cfg.booster_landing_reserve_kg
    landing_margin = cfg.booster_landing_propellant_margin_kg
    target_downrange_km = cfg.booster_landing_target_downrange_km
    burn_prediction = estimate_suicide_burn(
        r, v, m, C.LANDING_THRUST,
        safety_factor=cfg.booster_landing_ignition_safety_factor,
    )
    ctx = _BoosterContext(
        r=r,
        v=v,
        t=t,
        m=m,
        phase=phase,
        cfg=cfg,
        altitude=altitude,
        vertical=vertical,
        retrograde=retrograde,
        v_ground=compute_ground_relative_velocity(r, v),
        v_rel_norm=v_rel_norm,
        anti_wind=anti_wind,
        propellant_remaining=booster_propellant_remaining(m, cfg),
        landing_reserve=landing_reserve,
        landing_margin=landing_margin,
        protected_landing_propellant=landing_reserve + landing_margin,
        burn_prediction=burn_prediction,
        target_downrange_km=target_downrange_km,
        near_pad_rtls=is_near_pad_target(cfg),
        landing_site_now=target_landing_site_eci(t, target_downrange_km, config=cfg),
    )

    cmd = {
        'desired_dir': vertical,
        'thrust_on': False,
        'throttle': 0.0,
        'landing_vertical_priority': False,
        'landing_a_vert_needed': 0.0,
        'landing_a_horiz_budget': 0.0,
        'landing_target_descent_rate': 0.0,
        'landing_target_lead_time': 0.0,
        'landing_guidance_time_to_go_s': 0.0,
        'predicted_landing_error': 0.0,
        'landing_reserve_authority': 1.0,
        'landing_vertical_propellant_requirement': 0.0,
        'grid_fin_zero_effort_miss': None,
    }
    if phase in ("BOOSTER_FLIP", "BOOSTER_COAST"):
        cmd['desired_dir'] = retrograde
    elif phase == "BOOSTER_BOOSTBACK":
        cmd['desired_dir'] = _compute_boostback_direction(
            r,
            v,
            t,
            target_downrange_km,
            m,
            apogee_target_m=cfg.booster_apogee_target_km * 1000.0,
            config=cfg,
        )
        cmd['thrust_on'] = True
        cmd['throttle'] = 0.90
    elif phase == "BOOSTER_ENTRY":
        cmd.update(_entry_guidance(ctx))
    elif phase == "BOOSTER_LANDING":
        cmd.update(_landing_guidance(ctx, gs))
    desired_dir = cmd['desired_dir']
    throttle = cmd['throttle']

    plan = None
    if (
        phase in ("BOOSTER_BOOSTBACK", "BOOSTER_ENTRY")
        and abs(t - round(t)) < 0.51 * float(cfg.dt)
    ):
        plan = score_recovery_candidates(
            r,
            v,
            t,
            m,
            phase,
            cfg,
            current_direction=desired_dir,
            current_throttle=throttle,
        )

    v_vert = float(np.dot(v_rel, vertical))
    v_horiz = float(vec_norm(v_rel - v_vert * vertical))

    if v_rel_norm > 1.0:
        gamma_deg = float(np.degrees(np.arctan2(v_vert, max(v_horiz, 1e-6))))
    else:
        gamma_deg = 90.0 if np.dot(v, vertical) >= 0 else -90.0

    cos_pitch = np.clip(np.dot(vertical, desired_dir), -1.0, 1.0)
    pitch_angle = float(np.arccos(cos_pitch))

    output = {
        'thrust_direction': desired_dir,
        'phase': phase,
        'thrust_on': cmd['thrust_on'],
        'pitch_angle': pitch_angle,
        'gamma_angle': np.radians(gamma_deg),
        'gamma_command_deg': gamma_deg,
        'gamma_measured_deg': gamma_deg,
        'velocity_tilt_deg': float(np.degrees(np.arctan2(v_horiz, abs(v_vert)))) if v_rel_norm > 1.0 else 0.0,
        'blend_alpha': 1.0,
        'altitude': altitude,
        'velocity': float(vec_norm(v)),
        'local_vertical': vertical,
        'local_horizontal': compute_local_horizontal(r, v),
        'prograde': prograde,
        'throttle': throttle,
        'v_rel': v_rel,
        'v_rel_mag': v_rel_norm,
        'propellant_remaining_kg': ctx.propellant_remaining,
        'ignition_altitude_prediction_m': burn_prediction['burn_altitude'],
        'landing_vertical_priority': cmd['landing_vertical_priority'],
        'landing_a_vert_needed_mps2': float(cmd['landing_a_vert_needed']),
        'landing_a_horiz_budget_mps2': float(cmd['landing_a_horiz_budget']),
        'landing_target_descent_rate_mps': float(cmd['landing_target_descent_rate']),
        'landing_target_lead_time_s': float(cmd['landing_target_lead_time']),
        'landing_guidance_time_to_go_s': float(cmd['landing_guidance_time_to_go_s']),
        'landing_reserve_authority': float(cmd['landing_reserve_authority']),
        'landing_vertical_propellant_requirement_kg': float(
            cmd['landing_vertical_propellant_requirement']
        ),
        'grid_fin_zero_effort_miss': cmd['grid_fin_zero_effort_miss'],
        'predicted_landing_error_m': float(cmd['predicted_landing_error']),
        'planner_score': float(plan.score) if plan else 0.0,
        'planner_best_miss_m': float(plan.miss_m) if plan else 0.0,
        'planner_best_speed_mps': float(plan.speed_mps) if plan else 0.0,
        'planner_best_propellant_kg': float(plan.propellant_kg) if plan else 0.0,
        'planner_best_max_q_pa': float(plan.max_q_pa) if plan else 0.0,
        'planner_reachable': bool(plan.reachable) if plan else False,
        'planner_required_lateral_accel_mps2': float(plan.required_lateral_accel_mps2) if plan else 0.0,
        'planner_available_lateral_accel_mps2': float(plan.available_lateral_accel_mps2) if plan else 0.0,
        'planner_time_to_go_s': float(plan.time_to_go_s) if plan else 0.0,
    }
    return output, gs

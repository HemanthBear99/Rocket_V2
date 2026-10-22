"""Seeded sensitivity and Monte Carlo campaigns for the full RLV mission."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time
from collections import Counter
from collections.abc import Iterable
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from rlv_sim.config.definition import SimulationConfig
from rlv_sim.config.factory import create_default_config
from rlv_sim.core import constants as C
from rlv_sim.core.utils import cross3, vec_norm
from rlv_sim.guidance.recovery import target_landing_site_eci
from rlv_sim.mission.full_mission import run_full_mission
from rlv_sim.outputs.summary import assess_full_mission

_MC_THRUST_DISPERSION = 0.02
_MC_ISP_DISPERSION = 0.01
_MC_MASS_DISPERSION = 0.005
_MC_WIND_DISPERSION = 5.0
_MC_ATMOSPHERE_DENSITY_DISPERSION = 0.05
_MC_PITCHOVER_ANGLE_DISPERSION_DEG = 0.25
_DEFAULT_MC_SEED = 42
_DEFAULT_MC_RUNS = 100

PARAMETERS = (
    "thrust",
    "isp",
    "mass",
    "wind",
    "atmosphere",
    "pitchover",
)

# Opt-in extended dispersions (--dispersions extended). Drawn from a separate
# random stream so the legacy six columns of a seed stay identical.
_MC_DESCENT_DRAG_DISPERSION = 0.10
_MC_SEPARATION_DV_DISPERSION = 0.20
_MC_GPS_NOISE_DISPERSION = 0.25
_MC_WIND_MODE_AMPLITUDE_MPS = 2.0
_WIND_MODE_WAVELENGTHS_M = (2_000.0, 5_000.0, 12_000.0, 25_000.0)
_EXTENDED_STREAM_OFFSET = 1_000_003

EXTENDED_PARAMETERS = (
    "descent_drag",
    "separation_dv",
    "gps_noise",
    *(f"wind_mode_{i + 1}" for i in range(len(_WIND_MODE_WAVELENGTHS_M))),
)
ALL_PARAMETERS = PARAMETERS + EXTENDED_PARAMETERS
DISPERSION_SETS = ("legacy", "extended")

# NASA-STD-7009-style input pedigree for each dispersion (0 = none, 4 = flight
# data). All current values are engineering estimates: no sourced 1-sigma
# tables were found (docs/reports/Reusable booster sim improvements.md).
_ESTIMATE = "engineering estimate"
DISPERSION_PEDIGREE: dict[str, dict[str, Any]] = {
    "thrust": {"sigma": _MC_THRUST_DISPERSION, "units": "fraction of cluster thrust",
               "pedigree": 1, "source": _ESTIMATE},
    "isp": {"sigma": _MC_ISP_DISPERSION, "units": "fraction", "pedigree": 1, "source": _ESTIMATE},
    "mass": {"sigma": _MC_MASS_DISPERSION, "units": "fraction of liftoff mass",
             "pedigree": 1, "source": _ESTIMATE},
    "wind": {"sigma": _MC_WIND_DISPERSION, "units": "m/s mean-wind offset",
             "pedigree": 1, "source": _ESTIMATE},
    "atmosphere": {"sigma": _MC_ATMOSPHERE_DENSITY_DISPERSION, "units": "fraction of density",
                   "pedigree": 1, "source": _ESTIMATE},
    "pitchover": {"sigma": _MC_PITCHOVER_ANGLE_DISPERSION_DEG, "units": "deg",
                  "pedigree": 1, "source": _ESTIMATE},
    "descent_drag": {"sigma": _MC_DESCENT_DRAG_DISPERSION, "units": "fraction of booster descent drag",
                     "pedigree": 1, "source": _ESTIMATE + "; MER and MPG disperse per-body CD"},
    "separation_dv": {"sigma": _MC_SEPARATION_DV_DISPERSION, "units": "fraction of separation delta-v",
                      "pedigree": 1, "source": _ESTIMATE},
    "gps_noise": {"sigma": _MC_GPS_NOISE_DISPERSION, "units": "fraction of GPS sigmas",
                  "pedigree": 1, "source": _ESTIMATE},
    **{
        f"wind_mode_{i + 1}": {
            "sigma": _MC_WIND_MODE_AMPLITUDE_MPS,
            "units": f"m/s amplitude, {int(lam)} m wavelength",
            "pedigree": 1,
            "source": "GRAM-like layered perturbation; amplitudes not site-calibrated",
        }
        for i, lam in enumerate(_WIND_MODE_WAVELENGTHS_M)
    },
}

METRICS = (
    "perigee_altitude_km",
    "apogee_altitude_km",
    "eccentricity",
    "touchdown_speed_mps",
    "site_error_m",
    "max_ascent_q_pa",
    "max_recovery_q_pa",
    "max_recovery_q_alpha_pa_rad",
    "max_angle_of_attack_deg",
    "max_angle_of_attack_above_5kpa_deg",
    "orbiter_propellant_remaining_kg",
    "booster_propellant_remaining_kg",
    "mission_duration_s",
    "perigee_lower_bound_margin_km",
    "ascent_q_margin_pa",
    "touchdown_speed_margin_mps",
    "pad_error_margin_m",
)


def build_sensitivity_cases(
    parameters: Iterable[str] = PARAMETERS,
    sigma: float = 1.0,
) -> list[dict[str, Any]]:
    """Create a deterministic baseline plus ±sigma cases for each parameter.

    sigma=1 gives the classic sensitivity study; sigma=3 gives the
    one-at-a-time 3-sigma sweep used to rank dominant drivers.
    """
    selected = tuple(parameters)
    unknown = sorted(set(selected) - set(ALL_PARAMETERS))
    if unknown:
        raise ValueError(f"Unknown campaign parameter(s): {', '.join(unknown)}")
    cases = [{"case_id": "baseline", "mode": "sensitivity", "z_scores": {}}]
    for parameter in selected:
        tag = f"{sigma:g}sigma"
        cases.append(
            {
                "case_id": f"{parameter}_minus_{tag}",
                "mode": "sensitivity",
                "z_scores": {parameter: -float(sigma)},
            }
        )
        cases.append(
            {
                "case_id": f"{parameter}_plus_{tag}",
                "mode": "sensitivity",
                "z_scores": {parameter: float(sigma)},
            }
        )
    return cases


def sample_monte_carlo_cases(
    runs: int,
    seed: int,
    dispersions: str = "legacy",
) -> list[dict[str, Any]]:
    """Draw reproducible independent normal dispersions, clipped at ±3σ.

    The legacy six parameters always come from ``default_rng(seed)`` so a
    seed reproduces earlier campaigns exactly; extended parameters and
    wind-mode phases come from a separate stream. Each case records its
    seed so it can be replayed alone (``--case-ids``).
    """
    if runs <= 0:
        raise ValueError(f"runs must be positive, got {runs}")
    if dispersions not in DISPERSION_SETS:
        raise ValueError(f"dispersions must be one of {DISPERSION_SETS}, got {dispersions!r}")
    rng = np.random.default_rng(seed)
    draws = np.clip(rng.normal(size=(runs, len(PARAMETERS))), -3.0, 3.0)
    extended = dispersions == "extended"
    if extended:
        ext_rng = np.random.default_rng(seed + _EXTENDED_STREAM_OFFSET)
        ext_draws = np.clip(ext_rng.normal(size=(runs, len(EXTENDED_PARAMETERS))), -3.0, 3.0)
        phases = ext_rng.uniform(0.0, 2.0 * math.pi, size=(runs, len(_WIND_MODE_WAVELENGTHS_M)))
    cases = []
    for index in range(runs):
        z_scores = {
            parameter: float(draws[index, column])
            for column, parameter in enumerate(PARAMETERS)
        }
        case: dict[str, Any] = {
            "case_id": f"mc_{index + 1:04d}",
            "mode": "monte_carlo",
            "seed": int(seed),
            "dispersions": dispersions,
            "z_scores": z_scores,
        }
        if extended:
            z_scores.update({
                parameter: float(ext_draws[index, column])
                for column, parameter in enumerate(EXTENDED_PARAMETERS)
            })
            case["wind_mode_phases"] = [float(x) for x in phases[index]]
        cases.append(case)
    return cases


def apply_case(base: SimulationConfig, case: dict[str, Any]) -> SimulationConfig:
    """Convert dimensionless case z-scores into physical runtime overrides."""
    z = case.get("z_scores", {})
    thrust_scale = base.runtime_thrust_scale * (
        1.0 + float(z.get("thrust", 0.0)) * _MC_THRUST_DISPERSION
    )
    isp_scale = base.runtime_isp_scale * (
        1.0 + float(z.get("isp", 0.0)) * _MC_ISP_DISPERSION
    )
    mass_offset = base.runtime_initial_mass_offset_kg + (
        float(z.get("mass", 0.0)) * C.INITIAL_MASS * _MC_MASS_DISPERSION
    )
    wind_offset = base.runtime_wind_offset_mps + (
        float(z.get("wind", 0.0)) * _MC_WIND_DISPERSION
    )
    density_scale = base.runtime_atmosphere_density_scale * (
        1.0
        + float(z.get("atmosphere", 0.0))
        * _MC_ATMOSPHERE_DENSITY_DISPERSION
    )
    pitchover_angle = base.pitchover_angle + math.radians(
        float(z.get("pitchover", 0.0))
        * _MC_PITCHOVER_ANGLE_DISPERSION_DEG
    )
    descent_drag_scale = base.booster_descent_drag_scale * (
        1.0 + float(z.get("descent_drag", 0.0)) * _MC_DESCENT_DRAG_DISPERSION
    )
    separation_dv = base.separation_delta_v * (
        1.0 + float(z.get("separation_dv", 0.0)) * _MC_SEPARATION_DV_DISPERSION
    )
    gps_scale = max(1.0 + float(z.get("gps_noise", 0.0)) * _MC_GPS_NOISE_DISPERSION, 0.05)
    phases = case.get("wind_mode_phases") or [0.0] * len(_WIND_MODE_WAVELENGTHS_M)
    wind_modes = tuple(base.runtime_wind_profile_modes) + tuple(
        (float(z[f"wind_mode_{i + 1}"]) * _MC_WIND_MODE_AMPLITUDE_MPS, lam, float(phases[i]))
        for i, lam in enumerate(_WIND_MODE_WAVELENGTHS_M)
        if f"wind_mode_{i + 1}" in z
    )
    # Wind-aware reserve: swap the base config's adjustment for this case's.
    from rlv_sim.mission.planner import wind_reserve_adjustment_kg

    landing_reserve = (
        base.stage1_landing_fuel_reserve_kg
        - wind_reserve_adjustment_kg(base)
        + wind_reserve_adjustment_kg(base, wind_offset)
    )
    config = replace(
        base,
        verbose=False,
        enable_run_manifest=False,
        stage1_landing_fuel_reserve_kg=landing_reserve,
        booster_descent_drag_scale=max(descent_drag_scale, 0.1),
        separation_delta_v=max(separation_dv, 0.0),
        gps_position_sigma_m=base.gps_position_sigma_m * gps_scale,
        gps_velocity_sigma_mps=base.gps_velocity_sigma_mps * gps_scale,
        runtime_wind_profile_modes=wind_modes,
        runtime_thrust_scale=max(thrust_scale, 0.5),
        runtime_isp_scale=max(isp_scale, 0.5),
        runtime_initial_mass_offset_kg=mass_offset,
        runtime_wind_offset_mps=wind_offset,
        runtime_atmosphere_density_scale=max(density_scale, 0.1),
        pitchover_angle=float(np.clip(pitchover_angle, 0.0, math.radians(30.0))),
    )
    config.validate()
    return config


def _max_log_value(*logs: Any, field: str) -> float | None:
    values: list[float] = []
    for log in logs:
        values.extend(
            float(value)
            for value in getattr(log, field, [])
            if math.isfinite(float(value))
        )
    return max(values) if values else None


def _max_aoa_above_dynamic_pressure(
    *logs: Any,
    threshold_pa: float,
) -> float | None:
    values: list[float] = []
    for log in logs:
        angles = getattr(log, "angle_of_attack_deg", [])
        pressures = getattr(log, "dynamic_pressure", [])
        values.extend(
            float(angle)
            for angle, pressure in zip(angles, pressures)
            if float(pressure) >= threshold_pa and math.isfinite(float(angle))
        )
    return max(values) if values else None


def _run_case(base: SimulationConfig, case: dict[str, Any]) -> dict[str, Any]:
    started = time.perf_counter()
    config = apply_case(base, case)
    try:
        result = run_full_mission(config=config, verbose=False)
        assessment = assess_full_mission(result, config)
        touchdown_site = target_landing_site_eci(
            result.booster_final_state.t,
            config.booster_landing_target_downrange_km,
            config=config,
        )
        site_vertical = touchdown_site / max(
            float(vec_norm(touchdown_site)),
            1.0,
        )
        east = cross3(np.array([0.0, 0.0, 1.0]), site_vertical)
        east /= max(float(vec_norm(east)), 1e-9)
        north = cross3(site_vertical, east)
        miss_vector = touchdown_site - result.booster_final_state.r
        miss_vector -= float(np.dot(miss_vector, site_vertical)) * site_vertical
        row = {
            "case_id": case["case_id"],
            "mode": case["mode"],
            "status": assessment.status,
            "mission_success": assessment.mission_success,
            "orbit_success": assessment.orbit.success,
            "landing_success": assessment.landing.success,
            "failed_criteria": list(assessment.failed_criteria),
            "wall_time_s": time.perf_counter() - started,
            "perigee_altitude_km": assessment.orbit.perigee_altitude_km,
            "apogee_altitude_km": assessment.orbit.apogee_altitude_km,
            "eccentricity": assessment.orbit.eccentricity,
            "touchdown_speed_mps": assessment.landing.touchdown_speed_mps,
            "site_error_m": assessment.landing.site_error_m,
            "site_error_east_m": float(np.dot(miss_vector, east)),
            "site_error_north_m": float(np.dot(miss_vector, north)),
            "max_ascent_q_pa": assessment.max_ascent_q_pa,
            "max_recovery_q_pa": assessment.max_recovery_q_pa,
            "max_recovery_q_alpha_pa_rad": assessment.max_recovery_q_alpha_pa_rad,
            "max_angle_of_attack_deg": _max_log_value(
                result.ascent_log,
                result.orbiter_log,
                result.booster_log,
                field="angle_of_attack_deg",
            ),
            "max_angle_of_attack_above_5kpa_deg": _max_aoa_above_dynamic_pressure(
                result.ascent_log,
                result.orbiter_log,
                result.booster_log,
                threshold_pa=5000.0,
            ),
            "orbiter_propellant_remaining_kg": assessment.orbit.propellant_remaining_kg,
            "booster_propellant_remaining_kg": assessment.landing.propellant_remaining_kg,
            "mission_duration_s": assessment.mission_duration_s,
            "separation_time_s": assessment.separation_time_s,
            "perigee_lower_bound_margin_km": (
                assessment.orbit.perigee_altitude_km
                - (
                    assessment.orbit.target_altitude_km
                    - assessment.orbit.altitude_tolerance_km
                )
                if assessment.orbit.perigee_altitude_km is not None
                and assessment.orbit.target_altitude_km is not None
                and assessment.orbit.altitude_tolerance_km is not None
                else None
            ),
            "ascent_q_margin_pa": (
                C.MAX_DYNAMIC_PRESSURE - assessment.max_ascent_q_pa
                if assessment.max_ascent_q_pa is not None
                else None
            ),
            "touchdown_speed_margin_mps": (
                config.landing_leg_max_touchdown_speed_mps
                - assessment.landing.touchdown_speed_mps
                if assessment.landing.touchdown_speed_mps is not None
                else None
            ),
            "pad_error_margin_m": (
                config.booster_pad_tolerance_m - assessment.landing.site_error_m
                if assessment.landing.site_error_m is not None
                else None
            ),
            "orbiter_reason": result.orbiter_reason,
            "booster_reason": result.booster_reason,
        }
    except Exception as exc:  # noqa: BLE001 - a crashed case is recorded as an error row
        row = {
            "case_id": case["case_id"],
            "mode": case["mode"],
            "status": "error",
            "mission_success": False,
            "orbit_success": False,
            "landing_success": False,
            "failed_criteria": ["simulation_error"],
            "error": f"{type(exc).__name__}: {exc}",
            "wall_time_s": time.perf_counter() - started,
        }

    row["z_scores"] = dict(case.get("z_scores", {}))
    if "seed" in case:
        row["seed"] = case["seed"]
        row["dispersions"] = case.get("dispersions", "legacy")
    row["inputs"] = {
        "thrust_scale": config.runtime_thrust_scale,
        "isp_scale": config.runtime_isp_scale,
        "initial_mass_offset_kg": config.runtime_initial_mass_offset_kg,
        "wind_offset_mps": config.runtime_wind_offset_mps,
        "atmosphere_density_scale": config.runtime_atmosphere_density_scale,
        "pitchover_angle_deg": math.degrees(config.pitchover_angle),
        "booster_descent_drag_scale": config.booster_descent_drag_scale,
        "separation_delta_v": config.separation_delta_v,
        "gps_position_sigma_m": config.gps_position_sigma_m,
        "wind_profile_modes": [list(mode) for mode in config.runtime_wind_profile_modes],
    }
    return row


def run_campaign(
    base: SimulationConfig,
    cases: list[dict[str, Any]],
    workers: int = 1,
) -> list[dict[str, Any]]:
    """Execute campaign cases, optionally in isolated worker processes."""
    if workers <= 1:
        return [_run_case(base, case) for case in cases]

    ordered: dict[str, dict[str, Any]] = {}
    with ProcessPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(_run_case, base, case): case["case_id"]
            for case in cases
        }
        for future in as_completed(futures):
            ordered[futures[future]] = future.result()
    return [ordered[case["case_id"]] for case in cases]


def _numeric_summary(values: list[float]) -> dict[str, float] | None:
    finite = np.asarray([value for value in values if math.isfinite(value)], dtype=float)
    if finite.size == 0:
        return None
    return {
        "min": float(np.min(finite)),
        "p00135": float(np.percentile(finite, 0.135)),
        "p05": float(np.percentile(finite, 5)),
        "median": float(np.median(finite)),
        "p95": float(np.percentile(finite, 95)),
        "p99865": float(np.percentile(finite, 99.865)),
        "max": float(np.max(finite)),
        "mean": float(np.mean(finite)),
        "std": float(np.std(finite, ddof=1)) if finite.size > 1 else 0.0,
    }


def _wilson_interval(successes: int, total: int) -> tuple[float, float]:
    if total <= 0:
        return 0.0, 0.0
    z = 1.959963984540054
    p = successes / total
    denominator = 1.0 + z * z / total
    center = (p + z * z / (2.0 * total)) / denominator
    half = z * math.sqrt((p * (1.0 - p) + z * z / (4.0 * total)) / total) / denominator
    return max(0.0, center - half), min(1.0, center + half)


def _binomial_sf(successes: int, total: int, p: float) -> float:
    """P(X >= successes) for X ~ Binomial(total, p), summed in log space."""
    if successes <= 0:
        return 1.0
    if p <= 0.0:
        return 0.0
    if p >= 1.0:
        return 1.0
    log_p, log_q = math.log(p), math.log1p(-p)
    log_terms = [
        math.lgamma(total + 1) - math.lgamma(k + 1) - math.lgamma(total - k + 1)
        + k * log_p + (total - k) * log_q
        for k in range(successes, total + 1)
    ]
    peak = max(log_terms)
    return min(1.0, math.exp(peak) * sum(math.exp(t - peak) for t in log_terms))


def clopper_pearson_lower(successes: int, total: int, confidence: float) -> float:
    """One-sided exact (Clopper-Pearson) lower confidence bound on success rate.

    This is the form NASA verification statements use: "requirement met in at
    least p of cases with (1 - confidence) consumer risk".
    """
    if total <= 0 or successes <= 0:
        return 0.0
    alpha = 1.0 - confidence
    lo, hi = 0.0, successes / total
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        if _binomial_sf(successes, total, mid) < alpha:
            lo = mid
        else:
            hi = mid
    return lo


def zero_failure_runs_required(reliability: float, confidence: float) -> int:
    """Failure-free runs needed to show ``reliability`` at ``confidence``."""
    return math.ceil(math.log(1.0 - confidence) / math.log(reliability))


def summarize_results(results: list[dict[str, Any]]) -> dict[str, Any]:
    total = len(results)
    successful = sum(bool(row.get("mission_success")) for row in results)
    orbit_successes = sum(bool(row.get("orbit_success")) for row in results)
    landing_successes = sum(bool(row.get("landing_success")) for row in results)
    lower, upper = _wilson_interval(successful, total)
    failure_counts = Counter(
        criterion
        for row in results
        for criterion in row.get("failed_criteria", [])
    )

    metrics: dict[str, Any] = {}
    for metric in METRICS:
        values = [
            float(row[metric])
            for row in results
            if row.get(metric) is not None
        ]
        metrics[metric] = _numeric_summary(values)

    summary: dict[str, Any] = {
        "runs": total,
        "mission_successes": successful,
        "mission_success_rate": successful / total if total else 0.0,
        "mission_success_rate_95pct_ci": [lower, upper],
        "mission_success_lower_bound_90pct": clopper_pearson_lower(successful, total, 0.90),
        "mission_success_lower_bound_95pct": clopper_pearson_lower(successful, total, 0.95),
        "zero_failure_runs_for_99_73pct_at_90pct": zero_failure_runs_required(0.9973, 0.90),
        "orbit_success_rate": orbit_successes / total if total else 0.0,
        "landing_success_rate": landing_successes / total if total else 0.0,
        "failure_criteria_counts": dict(sorted(failure_counts.items())),
        "metrics": metrics,
    }

    by_id = {row["case_id"]: row for row in results}
    if "baseline" in by_id:
        effects: dict[str, Any] = {}
        for parameter in ALL_PARAMETERS:
            minus = by_id.get(f"{parameter}_minus_1sigma")
            plus = by_id.get(f"{parameter}_plus_1sigma")
            scale = 1.0
            if minus is None or plus is None:
                minus = by_id.get(f"{parameter}_minus_3sigma")
                plus = by_id.get(f"{parameter}_plus_3sigma")
                scale = 3.0
            if minus is None or plus is None:
                continue
            effects[parameter] = {}
            for metric in METRICS:
                minus_value = minus.get(metric)
                plus_value = plus.get(metric)
                if minus_value is None or plus_value is None:
                    continue
                effects[parameter][metric] = (
                    float(plus_value) - float(minus_value)
                ) / (2.0 * scale)
        summary["one_sigma_effects"] = effects

    if total >= 3 and any(row.get("mode") == "monte_carlo" for row in results):
        correlations: dict[str, dict[str, float]] = {}
        present = [
            parameter for parameter in ALL_PARAMETERS
            if any(parameter in row.get("z_scores", {}) for row in results)
        ]
        for parameter in present:
            correlations[parameter] = {}
            x = np.asarray(
                [float(row.get("z_scores", {}).get(parameter, 0.0)) for row in results],
                dtype=float,
            )
            for metric in METRICS:
                pairs = [
                    (x[index], float(row[metric]))
                    for index, row in enumerate(results)
                    if row.get(metric) is not None
                ]
                if len(pairs) < 3:
                    continue
                xp = np.asarray([pair[0] for pair in pairs], dtype=float)
                yp = np.asarray([pair[1] for pair in pairs], dtype=float)
                if np.std(xp) <= 0.0 or np.std(yp) <= 0.0:
                    continue
                correlations[parameter][metric] = float(np.corrcoef(xp, yp)[0, 1])
        summary["pearson_correlations"] = correlations
    return summary


def write_campaign_artifacts(
    output_dir: str | Path,
    base: SimulationConfig,
    results: list[dict[str, Any]],
    summary: dict[str, Any],
) -> dict[str, str]:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)

    json_path = output / "campaign_report.json"
    json_path.write_text(
        json.dumps(
            {
                "generated_at": datetime.now(timezone.utc).isoformat(),
                "base_config": asdict(base),
                "summary": summary,
                "dispersion_pedigree": DISPERSION_PEDIGREE,
                "results": results,
            },
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )

    csv_path = output / "campaign_results.csv"
    fieldnames = [
        "case_id",
        "mode",
        "status",
        "mission_success",
        "orbit_success",
        "landing_success",
        "failed_criteria",
        "error",
        *METRICS,
        "separation_time_s",
        "wall_time_s",
        "seed",
        *[f"z_{parameter}" for parameter in ALL_PARAMETERS],
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for result in results:
            row = {key: result.get(key) for key in fieldnames}
            row["failed_criteria"] = ",".join(result.get("failed_criteria", []))
            for parameter in ALL_PARAMETERS:
                row[f"z_{parameter}"] = result.get("z_scores", {}).get(parameter, 0.0)
            writer.writerow(row)

    markdown_path = output / "campaign_summary.md"
    ci = summary["mission_success_rate_95pct_ci"]
    lines = [
        "# RLV Sensitivity / Monte Carlo Campaign",
        "",
        f"- Runs: {summary['runs']}",
        (
            f"- Mission success: {summary['mission_successes']}/{summary['runs']} "
            f"({100.0 * summary['mission_success_rate']:.1f}%)"
        ),
        (
            "- 95% confidence interval: "
            f"{100.0 * ci[0]:.1f}%–{100.0 * ci[1]:.1f}%"
        ),
        (
            "- One-sided lower bound (Clopper-Pearson): "
            f"{100.0 * summary['mission_success_lower_bound_90pct']:.1f}% at 90% confidence, "
            f"{100.0 * summary['mission_success_lower_bound_95pct']:.1f}% at 95%"
        ),
        (
            "- Zero-failure runs needed to show 99.73% at 90% confidence: "
            f"{summary['zero_failure_runs_for_99_73pct_at_90pct']}"
        ),
        f"- Orbit success rate: {100.0 * summary['orbit_success_rate']:.1f}%",
        f"- Landing success rate: {100.0 * summary['landing_success_rate']:.1f}%",
        "",
        "## Failure criteria",
        "",
    ]
    failures = summary["failure_criteria_counts"]
    if failures:
        lines.extend(f"- {key}: {value}" for key, value in failures.items())
    else:
        lines.append("- None")
    lines.extend(["", "## Metric distributions", ""])
    for metric, stats in summary["metrics"].items():
        if stats is None:
            continue
        lines.append(
            f"- {metric}: median={stats['median']:.4g}, "
            f"p05={stats['p05']:.4g}, p95={stats['p95']:.4g}, "
            f"one-sided 3σ: p00135={stats['p00135']:.4g}, p99865={stats['p99865']:.4g}"
        )
    used = sorted({p for row in results for p in row.get("z_scores", {})})
    if used:
        lines.extend(["", "## Dispersion pedigree (NASA-STD-7009 style, 0-4)", ""])
        for parameter in used:
            info = DISPERSION_PEDIGREE.get(parameter)
            if info:
                lines.append(
                    f"- {parameter}: 1σ = {info['sigma']:g} {info['units']}; "
                    f"pedigree {info['pedigree']} ({info['source']})"
                )
    markdown_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {
        "json": str(json_path),
        "csv": str(csv_path),
        "markdown": str(markdown_path),
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("sensitivity", "sweep-3sigma", "monte-carlo"), default="sensitivity")
    parser.add_argument(
        "--dispersions", choices=DISPERSION_SETS, default="legacy",
        help="legacy = the original 6 parameters; extended adds descent drag, separation dv, "
             "GPS noise and altitude-correlated wind modes",
    )
    parser.add_argument(
        "--case-ids", default=None,
        help="Comma-separated case IDs to run alone (replay failures from a campaign)",
    )
    parser.add_argument("--runs", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--workers", type=int, default=max(1, min(4, (os.cpu_count() or 2) - 1)))
    parser.add_argument("--dt", type=float, default=None)
    parser.add_argument("--max-time", type=float, default=None)
    parser.add_argument(
        "--parameters",
        default=",".join(PARAMETERS),
        help="Comma-separated sensitivity parameters",
    )
    parser.add_argument(
        "--set", action="append", default=[], metavar="FIELD=VALUE",
        help="Override a config field for every case (repeatable), e.g. --set booster_entry_drag_credit=true",
    )
    parser.add_argument(
        "--landing-guidance",
        choices=("heuristic", "gfold"),
        default="heuristic",
        help="Booster landing-burn guidance for every case",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Campaign output directory (default: plots/campaign_<timestamp>)",
    )
    return parser.parse_args(argv)


def _parse_overrides(items: list[str]) -> dict:
    """FIELD=VALUE strings -> typed overrides (JSON literals, else string)."""
    overrides = {}
    for item in items:
        key, sep, raw = item.partition("=")
        if not sep:
            raise SystemExit(f"--set expects FIELD=VALUE, got {item!r}")
        try:
            overrides[key.strip()] = json.loads(raw)
        except json.JSONDecodeError:
            overrides[key.strip()] = raw
    return overrides


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    base = create_default_config(
        dt=args.dt if args.dt is not None else C.DT,
        max_time=args.max_time if args.max_time is not None else C.MAX_TIME,
        booster_landing_guidance=args.landing_guidance,
        **_parse_overrides(args.set),
    )
    seed = args.seed if args.seed is not None else _DEFAULT_MC_SEED
    if args.mode in ("sensitivity", "sweep-3sigma"):
        parameters = tuple(
            part.strip() for part in args.parameters.split(",") if part.strip()
        )
        if args.dispersions == "extended" and args.parameters == ",".join(PARAMETERS):
            parameters = ALL_PARAMETERS
        cases = build_sensitivity_cases(parameters, sigma=3.0 if args.mode == "sweep-3sigma" else 1.0)
    else:
        runs = args.runs if args.runs is not None else _DEFAULT_MC_RUNS
        cases = sample_monte_carlo_cases(runs, seed, dispersions=args.dispersions)
    if args.case_ids:
        wanted = {part.strip() for part in args.case_ids.split(",") if part.strip()}
        cases = [case for case in cases if case["case_id"] in wanted]
        missing = wanted - {case["case_id"] for case in cases}
        if missing:
            raise SystemExit(f"Unknown case id(s): {', '.join(sorted(missing))}")

    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir or f"plots/campaign_{timestamp}"
    print(
        f"Running {len(cases)} {args.mode} cases with seed={seed}, "
        f"workers={args.workers}, dt={base.dt}"
    )
    started = time.perf_counter()
    results = run_campaign(base, cases, workers=args.workers)
    summary = summarize_results(results)
    artifacts = write_campaign_artifacts(output_dir, base, results, summary)
    print(
        f"Success: {summary['mission_successes']}/{summary['runs']} "
        f"({100.0 * summary['mission_success_rate']:.1f}%)"
    )
    print(f"Elapsed: {time.perf_counter() - started:.1f} s")
    for name, path in artifacts.items():
        print(f"{name}: {path}")
    return 0 if summary["mission_successes"] == summary["runs"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

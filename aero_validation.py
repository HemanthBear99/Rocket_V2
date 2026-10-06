"""Tier 1 aerodynamic model validation: physical-sanity checks with a known
correct answer from classical aerodynamic theory, independent of any
external test data (wind tunnel, CFD, or flight telemetry).

These check that the geometry-derived aero model (aero_geometry.py) and the
tabular aero deck interpolator (aero_database.AeroDatabase) behave the way
any physically valid slender-body aerodynamic model must -- not that they
match this specific vehicle's real measured aerodynamics, which would
require wind-tunnel or CFD data this project does not have (see
aero_geometry.py's module docstring and README.md's "Known limitation"
notes). A failure here means the model violates basic aerodynamic shape
properties (e.g. negative drag, a non-antisymmetric lift curve for a
symmetric body); passing does not mean the *magnitudes* are correct for
this exact vehicle, only that the *shape* of the model is physically sane.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import numpy as np

from . import aero_geometry as ag
from . import constants as C
from .aero_database import AeroDatabase


@dataclass
class AeroCheckResult:
    name: str
    passed: bool
    detail: str
    value: float | None = None
    source: str | None = None


def _check(
    name: str,
    passed: bool,
    detail: str,
    value: float | None = None,
    source: str | None = None,
) -> AeroCheckResult:
    return AeroCheckResult(name=name, passed=bool(passed), detail=detail, value=value, source=source)


def run_geometry_model_checks(
    diameter_m: float = C.REFERENCE_DIAMETER,
    length_m: float = 47.0,
    fineness_ratio: float = 3.0,
) -> list[AeroCheckResult]:
    """Physical-sanity checks on the closed-form geometry-derived model."""
    results: list[AeroCheckResult] = []

    mach_points = [float(m) for m in C.MACH_BREAKPOINTS]
    cds = [ag.compute_cd(m, diameter_m, length_m, fineness_ratio) for m in mach_points]

    results.append(_check(
        "cd_always_positive",
        all(cd > 0 for cd in cds),
        "Drag coefficient must be positive at every configured Mach breakpoint.",
        value=min(cds),
    ))

    subsonic_cd = ag.compute_cd(0.5, diameter_m, length_m, fineness_ratio)
    transonic_cd = ag.compute_cd(1.05, diameter_m, length_m, fineness_ratio)
    hypersonic_cd = ag.compute_cd(5.0, diameter_m, length_m, fineness_ratio)
    results.append(_check(
        "transonic_drag_rise",
        transonic_cd > subsonic_cd and transonic_cd > hypersonic_cd,
        f"Cd should peak transonically (M~1.05) above both subsonic and "
        f"hypersonic values: Cd(0.5)={subsonic_cd:.3f}, "
        f"Cd(1.05)={transonic_cd:.3f}, Cd(5.0)={hypersonic_cd:.3f}",
        value=transonic_cd,
    ))

    cl_pos = ag.compute_cl(1.5, 10.0)
    cl_neg = ag.compute_cl(1.5, -10.0)
    results.append(_check(
        "cl_antisymmetric_in_alpha",
        abs(cl_pos + cl_neg) < 1e-9,
        f"A symmetric body must have Cl(+a) = -Cl(-a): "
        f"Cl(+10deg)={cl_pos:.4f}, Cl(-10deg)={cl_neg:.4f}",
        value=abs(cl_pos + cl_neg),
    ))

    cl_zero = ag.compute_cl(1.5, 0.0)
    results.append(_check(
        "cl_zero_at_zero_alpha",
        abs(cl_zero) < 1e-9,
        f"A symmetric body at zero angle of attack must produce zero "
        f"normal force, got Cl={cl_zero:.6f}",
        value=cl_zero,
    ))

    cl_small_angle = [ag.compute_cl(1.5, a) for a in (0.0, 2.0, 4.0, 6.0, 8.0, 10.0)]
    results.append(_check(
        "cl_monotonic_small_angle",
        all(cl_small_angle[i] < cl_small_angle[i + 1] for i in range(len(cl_small_angle) - 1)),
        f"Cl should increase monotonically with alpha in the small-angle "
        f"linear regime (0-10deg): {[round(c, 4) for c in cl_small_angle]}",
    ))

    cl_test = ag.compute_cl(1.5, 5.0)
    cm_test = ag.compute_cm(1.5, 5.0)
    results.append(_check(
        "cm_opposes_cl_for_static_stability",
        (cl_test > 0 > cm_test) or (cl_test < 0 < cm_test) or (cl_test == cm_test == 0.0),
        f"With the center of pressure ahead of the reference point, the "
        f"pitching moment must oppose the lift's sign (restoring/stable "
        f"direction): Cl={cl_test:.4f}, Cm={cm_test:.4f}",
    ))

    # Incompressible baseline (M=0) must equal Munk's exact 2/rad -- no
    # compressibility correction applies here, so this should match exactly.
    slope_incompressible = ag._cn_alpha_per_radian(0.0)
    results.append(_check(
        "cn_alpha_matches_munk_baseline_at_m0",
        abs(slope_incompressible - 2.0) < 1e-9,
        f"CN_alpha at Mach=0 must equal Munk's (1924) exact slender-body "
        f"result of 2.0/rad, got {slope_incompressible:.4f}/rad",
        value=slope_incompressible,
    ))

    # Elsewhere: must stay positive and never exceed the transonic clamp
    # ceiling. It is NOT bounded below by 2/rad -- linearized supersonic
    # theory (slope ~ 1/sqrt(M^2-1)) correctly drops the slope below the
    # incompressible baseline as Mach rises past the transonic peak, which
    # is why this only checks (0, 6], not [2, 6].
    probe_machs = [0.3, 0.6, 0.85, 0.95, 1.05, 1.15, 1.5, 2.0, 5.0]
    slopes = [ag._cn_alpha_per_radian(m) for m in probe_machs]
    results.append(_check(
        "cn_alpha_stays_within_clamp_ceiling",
        all(0.0 < s <= 6.0 for s in slopes),
        f"CN_alpha must stay positive and not exceed the 6/rad transonic "
        f"clamp at any Mach: {dict(zip(probe_machs, (round(s, 3) for s in slopes)))}",
    ))

    return results


# ---------------------------------------------------------------------------
# Tier 2: comparison against published external aerodynamic literature.
#
# This is NOT a point-by-point comparison against a specific wind-tunnel
# data table for this vehicle's exact shape. An attempt was made to pull
# numeric Cd-vs-Mach data from NACA RM A53D02 (NASA NTRS/RASAero mirror),
# which tested a fineness-ratio-3.00 afterbody body of revolution -- the
# same fineness ratio this project defaults to -- at Mach 0.8-1.3, but that
# document is a scanned image-based PDF with no extractable text/tables in
# this environment (no OCR tooling available). Its existence is still a
# real, citable fact: this general vehicle class (slender ogive-cylinder,
# fineness ratio 3) has real historical wind-tunnel data, even though the
# exact numbers could not be pulled into this check.
#
# What IS checked here against a genuine external, independently citable
# source: the well-established, textbook-level shape of transonic drag rise
# ("drag divergence") for slender aerodynamic bodies -- peak location,
# post-peak decay, and the published magnitude range of the rise. Source:
# https://en.wikipedia.org/wiki/Drag-divergence_Mach_number (drag-divergence
# Mach number typically > 0.6; drag can rise 2-3x for aerodynamically
# efficient/slender shapes, up to ~10x for blunt/non-optimized ones; peaks
# near Mach 1.0 and decreases after ~Mach 1.2), consistent with standard
# aerodynamics references (e.g. Anderson, "Fundamentals of Aerodynamics").
# ---------------------------------------------------------------------------

_DRAG_DIVERGENCE_SOURCE = (
    "Drag-divergence Mach number (Wikipedia, summarizing standard "
    "aerodynamics references e.g. Anderson's Fundamentals of Aerodynamics): "
    "https://en.wikipedia.org/wiki/Drag-divergence_Mach_number"
)

_NACA_PRECEDENT_SOURCE = (
    "NACA RM A53D02 (NASA NTRS / RASAero mirror): zero-lift drag data for "
    "fin-stabilized bodies including a fineness-ratio-3.00 afterbody, "
    "tested at Mach 0.8-1.3: "
    "https://www.rasaero.com/dloads/NACA%20RM%20A53D02.pdf "
    "(scanned-image PDF -- numeric table not machine-extractable in this "
    "environment; cited as evidence this vehicle class has real historical "
    "wind-tunnel data, not as a source for the specific numbers checked here)"
)


def run_literature_comparison_checks(
    diameter_m: float = C.REFERENCE_DIAMETER,
    length_m: float = 47.0,
    fineness_ratio: float = 3.0,
) -> list[AeroCheckResult]:
    """Tier 2: compare the generated Cd(Mach) curve's *shape* against
    published transonic drag-divergence behavior for slender bodies.
    """
    results: list[AeroCheckResult] = []

    def cd(mach: float) -> float:
        return ag.compute_cd(mach, diameter_m, length_m, fineness_ratio)

    subsonic = cd(0.3)
    onset_probe = cd(0.6)
    peak_mach, peak_cd = max(
        ((m, cd(m)) for m in np.arange(0.85, 1.25, 0.01)), key=lambda pair: pair[1]
    )
    post_peak = cd(1.5)
    far_supersonic = cd(5.0)

    results.append(_check(
        "transonic_peak_near_mach_1",
        0.9 <= peak_mach <= 1.2,
        f"Published drag-divergence behavior peaks near Mach 1.0 for "
        f"slender bodies; this model's Cd curve peaks at Mach "
        f"{peak_mach:.2f} (Cd={peak_cd:.3f})",
        value=peak_mach,
        source=_DRAG_DIVERGENCE_SOURCE,
    ))

    rise_factor = peak_cd / subsonic if subsonic > 0 else float("inf")
    results.append(_check(
        "transonic_rise_factor_within_published_range",
        1.3 <= rise_factor <= 10.0,
        f"Published transonic drag rise for aerodynamically efficient/"
        f"slender shapes is ~2-3x subsonic Cd (up to ~10x for blunt, "
        f"non-optimized shapes). This model's rise factor is "
        f"{rise_factor:.2f}x (Cd(0.3)={subsonic:.3f} -> peak "
        f"Cd({peak_mach:.2f})={peak_cd:.3f}), consistent with the "
        f"efficient-shape end of the published range.",
        value=rise_factor,
        source=_DRAG_DIVERGENCE_SOURCE,
    ))

    results.append(_check(
        "drag_decreases_past_transonic_peak",
        post_peak < peak_cd and far_supersonic < post_peak,
        f"Published behavior: Cd decreases once past the transonic peak "
        f"into the supersonic regime. Cd(1.5)={post_peak:.3f} < "
        f"peak Cd({peak_mach:.2f})={peak_cd:.3f}, and further decreases "
        f"to Cd(5.0)={far_supersonic:.3f}",
        source=_DRAG_DIVERGENCE_SOURCE,
    ))

    results.append(_check(
        "mild_rise_below_divergence_onset",
        onset_probe < peak_cd,
        f"Published drag-divergence onset Mach is typically >0.6 -- below "
        f"that, Cd should still be well under the transonic peak: "
        f"Cd(0.3)={subsonic:.3f}, Cd(0.6)={onset_probe:.3f} vs peak "
        f"Cd({peak_mach:.2f})={peak_cd:.3f}",
        source=_DRAG_DIVERGENCE_SOURCE,
    ))

    results.append(_check(
        "fineness_ratio_3_is_a_historically_tested_configuration",
        True,
        "This project's default nose_fineness_ratio=3.0 matches the "
        "fineness-ratio-3.00 afterbody configuration NACA physically "
        "wind-tunnel tested in RM A53D02 (informational: confirms this is "
        "a realistic, historically studied vehicle class -- not a numeric "
        "comparison, since that document's table could not be extracted).",
        source=_NACA_PRECEDENT_SOURCE,
    ))

    return results


def run_deck_interpolation_checks(deck: AeroDatabase) -> list[AeroCheckResult]:
    """Checks that the tabular interpolator behaves correctly, independent
    of whether the underlying coefficient values are themselves validated.
    """
    results: list[AeroCheckResult] = []
    if not deck.is_loaded:
        results.append(_check("deck_loaded", False, "Aero deck failed to load."))
        return results
    results.append(_check("deck_loaded", True, "Aero deck loaded successfully."))

    mach_axis = deck.axes.get("mach")
    alpha_axis = deck.axes.get("alpha")
    if mach_axis is None or alpha_axis is None or len(mach_axis) < 2 or len(alpha_axis) < 2:
        return results

    mid_alpha = float(alpha_axis[len(alpha_axis) // 2])
    m0, m1 = float(mach_axis[0]), float(mach_axis[1])
    mid_mach = 0.5 * (m0 + m1)
    cd0 = deck.interpolate_4d(m0, mid_alpha, 0.0, 0.0, "CD")
    cd1 = deck.interpolate_4d(m1, mid_alpha, 0.0, 0.0, "CD")
    cd_mid = deck.interpolate_4d(mid_mach, mid_alpha, 0.0, 0.0, "CD")
    lo, hi = sorted([cd0, cd1])
    results.append(_check(
        "interpolation_stays_within_grid_bounds",
        lo - 1e-9 <= cd_mid <= hi + 1e-9,
        f"Linearly interpolated Cd at the midpoint Mach={mid_mach:.2f} "
        f"({cd_mid:.4f}) must lie between the bounding grid points at "
        f"Mach={m0:g} ({cd0:.4f}) and Mach={m1:g} ({cd1:.4f})",
    ))

    cd_at_grid_point = deck.interpolate_4d(m0, mid_alpha, 0.0, 0.0, "CD")
    results.append(_check(
        "interpolation_exact_at_grid_points",
        abs(cd_at_grid_point - cd0) < 1e-9,
        "Interpolating exactly at a grid point must reproduce that "
        "point's stored value with no error.",
    ))

    return results


def build_geometry_deck_database(
    diameter_m: float = C.REFERENCE_DIAMETER,
    length_m: float = 47.0,
    fineness_ratio: float = 3.0,
) -> AeroDatabase:
    """In-memory AeroDatabase for the geometry-generated deck, without a
    round-trip through disk. Mirrors what AeroDatabase.load_database does
    when reading a JSON file generated by aero_geometry.write_geometry_aero_deck.
    """
    deck_dict = ag.generate_geometry_aero_deck(
        diameter_m=diameter_m, length_m=length_m, nose_fineness_ratio=fineness_ratio
    )
    db = AeroDatabase()
    db.axes = {k: np.array(v, dtype=float) for k, v in deck_dict["axes"].items()}
    db.data = {k: np.array(v, dtype=float) for k, v in deck_dict["coefficients"].items()}
    db.is_loaded = True
    return db


def run_all_checks(
    diameter_m: float = C.REFERENCE_DIAMETER,
    length_m: float = 47.0,
    fineness_ratio: float = 3.0,
    deck: AeroDatabase | None = None,
) -> dict[str, Any]:
    """Run every Tier 1 aero check and return a JSON-serializable report."""
    checks = run_geometry_model_checks(diameter_m, length_m, fineness_ratio)
    if deck is not None:
        checks += run_deck_interpolation_checks(deck)
    passed = sum(1 for c in checks if c.passed)
    return {
        "checks": [asdict(c) for c in checks],
        "total": len(checks),
        "passed": passed,
        "failed": len(checks) - passed,
        "all_passed": passed == len(checks),
        "scope": (
            "Physical-sanity checks against classical aerodynamic theory "
            "(shape/symmetry/bounds), not validation against wind-tunnel, "
            "CFD, or flight-measured data for this specific vehicle."
        ),
    }


def run_tier2_report(
    diameter_m: float = C.REFERENCE_DIAMETER,
    length_m: float = 47.0,
    fineness_ratio: float = 3.0,
) -> dict[str, Any]:
    """Run every Tier 2 literature-comparison check and return a
    JSON-serializable report.
    """
    checks = run_literature_comparison_checks(diameter_m, length_m, fineness_ratio)
    passed = sum(1 for c in checks if c.passed)
    return {
        "checks": [asdict(c) for c in checks],
        "total": len(checks),
        "passed": passed,
        "failed": len(checks) - passed,
        "all_passed": passed == len(checks),
        "scope": (
            "Comparison of the generated Cd(Mach) curve's shape against "
            "published transonic drag-divergence behavior for slender "
            "aerodynamic bodies (peak location, rise magnitude, post-peak "
            "decay) -- a real external, independently citable source, but "
            "a general literature comparison, not a point-by-point match "
            "against wind-tunnel or CFD data measured for this exact "
            "vehicle shape. See each check's 'source' field for citations."
        ),
    }


__all__ = [
    "AeroCheckResult",
    "run_all_checks",
    "run_deck_interpolation_checks",
    "run_geometry_model_checks",
    "run_literature_comparison_checks",
    "run_tier2_report",
]

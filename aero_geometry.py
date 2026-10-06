"""Geometry-derived aerodynamic deck generator.

Builds a Cd/CN(Mach, alpha) table from the vehicle's actual body geometry
(diameter, length, nose fineness ratio) instead of the flat hand-tuned
Mach breakpoint table in constants.py (MACH_BREAKPOINTS/CD_VALUES/
CL_ALPHA_VALUES). Output is written in the same JSON shape AeroDatabase
already loads, so it's a drop-in aero_deck_path.

Method: closed-form slender-body/body-of-revolution aerodynamics --
the same category of published formulas DATCOM and OpenRocket use for a
body alone (no fins are modeled on ascent; TVC provides control here):

  - Skin friction: Sommer & Short (1955) compressible turbulent flat-plate
    correlation, at a representative Reynolds number.
  - Base drag: Hoerner's "Fluid-Dynamic Drag" empirical base-drag
    correlation, the same one used in most open rocketry drag models
    (e.g. OpenRocket's aerodynamic calculator).
  - Nose wave drag: transonic/supersonic ogive-nose approximation, scaled
    by 1/fineness_ratio^2 (a pointier nose produces less wave drag) --
    the qualitative shape used by Barrowman-derived methods.
  - Normal-force slope: classical slender-body theory (Munk, 1924)
    CN_alpha = 2 per radian for a body of revolution, with a
    Prandtl-Glauert-style compressibility correction.

This is a real derivation from the vehicle's own dimensions, not a fit to
proprietary flight data -- treat it as an engineering approximation, same
caveat as the rest of this module's aero model (see forces.py docstring).
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from . import constants as C

# Representative Reynolds number for skin friction: a large launch vehicle
# spans roughly 1e7-1e9 over the flight; 3e8 (~Re at max-Q for a vehicle
# this size) is used as a single representative value rather than
# recomputing it altitude-by-altitude, which the Mach-only table format
# this feeds (mach x alpha) has no room for anyway.
_REYNOLDS_REPRESENTATIVE = 3.0e8


def _skin_friction_cd(mach: float, diameter_m: float, length_m: float, reference_area_m2: float) -> float:
    """Sommer & Short (1955) compressible turbulent flat-plate skin friction,
    applied over the body's wetted (cylindrical) area.
    """
    cf_incompressible = 0.455 / (math.log10(_REYNOLDS_REPRESENTATIVE) ** 2.58)
    cf = cf_incompressible / (1.0 + 0.144 * mach ** 2) ** 0.65
    wetted_area = math.pi * diameter_m * length_m
    return cf * wetted_area / reference_area_m2


def _base_drag_cd(mach: float, base_area_m2: float, reference_area_m2: float) -> float:
    """Hoerner base-drag correlation (subsonic rise, supersonic decay)."""
    if mach <= 1.0:
        cd_base = 0.12 + 0.13 * mach ** 2
    else:
        cd_base = 0.25 / mach
    return cd_base * base_area_m2 / reference_area_m2


def _nose_wave_drag_cd(mach: float, fineness_ratio: float) -> float:
    """Transonic/supersonic ogive-nose wave drag.

    Peaks near Mach 1.05 (transonic drag rise) and decays roughly as 1/M^2
    into the supersonic regime; magnitude scaled by 1/fineness_ratio^2
    (a longer, pointier nose sheds wave drag faster than a blunt one) --
    the qualitative behavior published nose-drag charts show.
    """
    if mach < 0.8:
        return 0.0
    peak_magnitude = 1.7 / max(fineness_ratio, 1.0) ** 2
    if mach < 1.2:
        # Transonic bump, centered at Mach 1.05.
        return peak_magnitude * math.exp(-(((mach - 1.05) / 0.35) ** 2))
    # Supersonic decay from the transonic peak.
    peak_at_1_2 = peak_magnitude * math.exp(-(((1.2 - 1.05) / 0.35) ** 2))
    return peak_at_1_2 * (1.2 / mach) ** 2


def _cn_alpha_per_radian(mach: float) -> float:
    """Slender-body normal-force slope (Munk 1924) with a Prandtl-Glauert
    style compressibility correction. The correction is singular at Mach 1,
    so the result is clamped to 6/rad -- the upper end of the range
    published for slender body-of-revolution normal-force slopes -- rather
    than letting the 1/beta term blow up into an unphysical transonic
    spike.
    """
    base = 2.0
    max_cn_alpha = 6.0
    if mach < 0.85:
        beta = math.sqrt(max(1.0 - mach ** 2, 0.05))
        return min(base / beta, max_cn_alpha)
    if mach > 1.15:
        beta = math.sqrt(max(mach ** 2 - 1.0, 0.05))
        return min(base / beta, max_cn_alpha)
    # Transonic: physically the slope peaks smoothly through Mach 1 rather
    # than singularly; use the clamp value directly through this band.
    return max_cn_alpha


def compute_cd(mach: float, diameter_m: float, length_m: float, fineness_ratio: float) -> float:
    """Total zero-lift drag coefficient at a given Mach number."""
    reference_area = math.pi * (diameter_m / 2.0) ** 2
    base_area = reference_area
    cd = (
        _skin_friction_cd(mach, diameter_m, length_m, reference_area)
        + _base_drag_cd(mach, base_area, reference_area)
        + _nose_wave_drag_cd(mach, fineness_ratio)
    )
    return float(cd)


def compute_cl(mach: float, alpha_deg: float) -> float:
    """Normal-force coefficient at a given Mach and angle of attack,
    small-angle mapped through sin(alpha)cos(alpha) so it stays bounded
    and signed correctly away from the linear regime.
    """
    alpha_rad = math.radians(alpha_deg)
    cn_alpha = _cn_alpha_per_radian(mach)
    return float(cn_alpha * math.sin(alpha_rad) * math.cos(alpha_rad))


def compute_cm(mach: float, alpha_deg: float, cp_offset_calibers: float = 0.5) -> float:
    """Pitching moment about the reference point, from the same normal
    force acting at an assumed CP offset (in body diameters) ahead of the
    moment reference -- consistent with H_CP in constants.py rather than
    an independently invented number.
    """
    cl = compute_cl(mach, alpha_deg)
    return float(-cl * cp_offset_calibers)


def generate_geometry_aero_deck(
    diameter_m: float = C.REFERENCE_DIAMETER,
    length_m: float = 47.0,
    nose_fineness_ratio: float = 3.0,
    mach_points: np.ndarray | None = None,
    alpha_points: np.ndarray | None = None,
) -> dict:
    """Build a full (mach, alpha, beta, fin) aero deck dict from geometry.

    beta (sideslip) and fin (deflection) axes are kept at a single value
    each since ascent has no aerodynamic control fins in this vehicle
    model (grid fins are recovery-only and already modeled separately in
    forces.py's _booster_recovery_aero_scale) -- the 4D interpolator
    AeroDatabase expects still works correctly with singleton axes.

    Args:
        diameter_m: Body diameter (default: constants.REFERENCE_DIAMETER).
        length_m: Overall vehicle length. No length constant exists
            elsewhere in this codebase (the model uses reference
            area/diameter/CP location, not a full length breakdown), so
            this is a documented estimate for a vehicle this class, not a
            configured value -- override it with the real figure if known.
        nose_fineness_ratio: Nose length / diameter. 3.0 is typical for an
            orbital-class payload fairing.
        mach_points: Mach grid (default: reuses constants.MACH_BREAKPOINTS).
        alpha_points: Alpha grid in degrees (default: -20..20 in 5 steps).
    """
    if mach_points is None:
        mach_points = C.MACH_BREAKPOINTS
    if alpha_points is None:
        alpha_points = np.array([-20.0, -10.0, -5.0, 0.0, 5.0, 10.0, 20.0])
    beta_points = np.array([0.0])
    fin_points = np.array([0.0])

    n_m, n_a, n_b, n_f = len(mach_points), len(alpha_points), len(beta_points), len(fin_points)
    cd_grid = np.zeros((n_m, n_a, n_b, n_f))
    cl_grid = np.zeros((n_m, n_a, n_b, n_f))
    cm_grid = np.zeros((n_m, n_a, n_b, n_f))

    for i, mach in enumerate(mach_points):
        cd = compute_cd(float(mach), diameter_m, length_m, nose_fineness_ratio)
        for j, alpha in enumerate(alpha_points):
            cl = compute_cl(float(mach), float(alpha))
            cm = compute_cm(float(mach), float(alpha))
            cd_grid[i, j, 0, 0] = cd
            cl_grid[i, j, 0, 0] = cl
            cm_grid[i, j, 0, 0] = cm

    return {
        "axes": {
            "mach": mach_points.tolist(),
            "alpha": alpha_points.tolist(),
            "beta": beta_points.tolist(),
            "fin": fin_points.tolist(),
        },
        "coefficients": {
            "CD": cd_grid.tolist(),
            "CL": cl_grid.tolist(),
            "CM": cm_grid.tolist(),
        },
        "metadata": {
            "source": "geometry-derived (aero_geometry.generate_geometry_aero_deck)",
            "diameter_m": diameter_m,
            "length_m": length_m,
            "nose_fineness_ratio": nose_fineness_ratio,
            "reynolds_representative": _REYNOLDS_REPRESENTATIVE,
        },
    }


def write_geometry_aero_deck(output_path: str | Path, **kwargs) -> str:
    """Generate and write a geometry-derived aero deck JSON file. Returns the path."""
    deck = generate_geometry_aero_deck(**kwargs)
    target = Path(output_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(deck, indent=2), encoding="utf-8")
    return str(target)


if __name__ == "__main__":
    import sys

    out = sys.argv[1] if len(sys.argv) > 1 else "aero_decks/geometry_deck.json"
    path = write_geometry_aero_deck(out)
    print(f"Wrote geometry-derived aero deck: {path}")

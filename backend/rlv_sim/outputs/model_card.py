"""What the simulator models, from which sources, and where it stops being valid.

Served by GET /api/model for the app's "Model & limits" page. The long-form
provenance is docs/MATHEMATICAL_REFERENCES.md; keep the two in step.
"""

from __future__ import annotations

from typing import Any

# area, model, basis (standard / published source / project assumption),
# validity limit, NASA-STD-7009-style input pedigree (0 none .. 4 flight data).
MODELS: tuple[dict[str, Any], ...] = (
    {
        "area": "Rigid-body dynamics",
        "model": "6-DOF, ECI frame, quaternion attitude; I*w_dot = tau - w x (I*w) - I_dot*w - jet damping",
        "basis": "Standard rigid-body mechanics; per-axis jet damping (pitch/yaw use the nozzle-exit lever arm, "
                 "roll uses the engine-ring and nozzle-exit radii)",
        "limit": "Inertia varies linearly with propellant between full/empty values; no slosh, no flexible body.",
        "pedigree": 2,
    },
    {
        "area": "Integration",
        "model": "Fixed-step RK4, quaternion renormalised each sub-step, exact split at propellant exhaustion",
        "basis": "Standard numerical method",
        "limit": "No error estimator or adaptive step; step sizes chosen empirically per phase.",
        "pedigree": 2,
    },
    {
        "area": "Gravity",
        "model": "Central, J2, or EGM96 spherical harmonics",
        "basis": "EGM96 coefficients (NASA/NGA)",
        "limit": "EGM96 truncated to degree/order 6 for speed (the full model runs to 360).",
        "pedigree": 3,
    },
    {
        "area": "Atmosphere & wind",
        "model": "US-76 layers; mean wind profile plus optional altitude-varying wind modes",
        "basis": "US Standard Atmosphere 1976 (published); wind modes are a GRAM-like layered perturbation",
        "limit": "Upper extension is isothermal; the 'GRAM' option is a smooth scaling, not NASA GRAM. "
                 "Wind modes are not calibrated to a launch site.",
        "pedigree": 2,
    },
    {
        "area": "Aerodynamics",
        "model": "Drag, lift, normal force and pitching moment from Mach tables or an imported deck; "
                 "bounded post-stall cross-flow",
        "basis": "Geometry/literature estimates; descent drag scale fitted to Falcon 9 webcast telemetry",
        "limit": "No wind-tunnel or CFD data for this vehicle. Grid-fin authority is a flat-plate estimate "
                 "and optimistic versus a real lattice fin.",
        "pedigree": 1,
    },
    {
        "area": "Propulsion & mass",
        "model": "mdot = T / (Isp * g0); lumped thrust per stage; engine spool-up/down transients",
        "basis": "Rocket equation (standard); Falcon 9 FT stage masses and thrust from public sources",
        "limit": "One lumped engine per stage: no per-engine throttling or engine-out. CG stations and "
                 "inertias are project estimates.",
        "pedigree": 2,
    },
    {
        "area": "Aerothermal",
        "model": "Sutton-Graves stagnation-point convective heating, lumped skin temperature",
        "basis": "Sutton & Graves (1971)",
        "limit": "Convection only, no radiation cooling; skin temperature is a first-order lag.",
        "pedigree": 2,
    },
    {
        "area": "Ascent guidance",
        "model": "Pitchover and gravity turn with a max-q throttle bucket",
        "basis": "Project steering logic",
        "limit": "Not IGM/PEG or an optimised trajectory.",
        "pedigree": 1,
    },
    {
        "area": "Booster recovery guidance",
        "model": "Flip, boostback targeting, entry burn, suicide-burn landing with ZEM/ZEV divert and a "
                 "final-approach position hold; optional G-FOLD or a user plugin",
        "basis": "ZEM/ZEV (Guo et al. 2013), G-FOLD lossless convexification (Acikmese & Ploen 2007); "
                 "gains tuned for this vehicle",
        "limit": "Guidance knows the true drag scale (no drag estimation), which flatters robustness. "
                 "G-FOLD needs cvxpy and is not drag-aware.",
        "pedigree": 1,
    },
    {
        "area": "Navigation",
        "model": "GPS, IMU propagation and landing altimeter with configurable noise",
        "basis": "Project demonstration models",
        "limit": "No fusion filter, alignment or Earth-rate correction; dead-reckoning errors grow unphysically.",
        "pedigree": 1,
    },
)

KNOWN_LIMITS: tuple[str, ...] = (
    ("Booster landing succeeds in 98% of 100 seed-42 Monte Carlo runs at the default descent drag, "
    "but 88% with the drag fitted to Falcon 9 telemetry (scale 0.68): tailwind cases arrive 100-380 m "
    "off target. 100 runs bound the success rate only to 94.8% (90% confidence)."),
    "Guidance uses the true drag scale; a real vehicle must estimate it in flight.",
    ("Ascent MECO comes 6-10% early against Falcon 9 webcast data because the sim holds back a larger "
    "recovery reserve; peak dynamic pressure is 9-29% high (throttle-bucket depth is not published)."),
    "Stage-2 recovery is experimental and usually propellant-starved.",
    "All dispersion sigmas are engineering estimates (pedigree 1); none come from sourced 1-sigma tables.",
    "Research and education tool: not flight-qualified, not for flight-critical use.",
)


def model_card() -> dict[str, Any]:
    from rlv_sim.outputs.manifest import credibility_block

    return {
        "models": list(MODELS),
        "known_limits": list(KNOWN_LIMITS),
        "credibility": credibility_block(),
        "references": "docs/MATHEMATICAL_REFERENCES.md, docs/VALIDATION.md, docs/MONTE_CARLO_RESULTS.md",
    }

"""G-FOLD convex landing guidance: solver, guidance integration and toggles."""

import numpy as np
import pytest

from rlv_sim import _gfold
from rlv_sim import constants as C
from rlv_sim._guidance_booster import compute_booster_guidance
from rlv_sim._guidance_common import create_guidance_state
from rlv_sim.config_factory import create_default_config
from rlv_sim.recovery import target_landing_site_eci

needs_cvxpy = pytest.mark.skipif(not _gfold.AVAILABLE, reason="cvxpy not installed")

SITE = np.array([C.R_EARTH, 0.0, 0.0])
LIMITS = _gfold.DescentLimits(
    thrust_min_n=0.35 * C.LANDING_THRUST,
    thrust_max_n=C.LANDING_THRUST,
    mass_flow_per_newton=1.0 / (282.0 * 9.80665),
    dry_mass_kg=22200.0,
    max_tilt_deg=15.0,
    glideslope_deg=5.0,
    gravity_mps2=9.81,
)


def _state(offset_enu, velocity_enu):
    frame = _gfold.local_frame(SITE)
    return SITE + frame.T @ np.asarray(offset_enu, float), frame.T @ np.asarray(velocity_enu, float)


@needs_cvxpy
def test_plan_reaches_gate_and_respects_limits():
    r, v = _state([30.0, 0.0, 300.0], [-3.0, 0.0, -80.0])
    plan = _gfold.plan_descent(r, v, 26000.0, 0.0, SITE, LIMITS, tf_guess_s=6.0,
                               gate_altitude_m=15.0, gate_descent_rate_mps=3.0)
    assert plan is not None
    assert 22200.0 < plan.final_mass_kg < 26000.0
    accel = plan.accel_enu
    tilt = np.degrees(np.arccos(np.clip(accel[:, 2] / np.linalg.norm(accel, axis=1), -1, 1)))
    assert tilt.max() <= 15.0 + 0.5
    # Thrust/mass never exceeds what the engine can produce at the final mass.
    assert np.linalg.norm(accel, axis=1).max() <= C.LANDING_THRUST / plan.final_mass_kg * 1.01


@needs_cvxpy
def test_infeasible_descent_returns_none():
    # Far too fast to stop in 300 m with this engine.
    r, v = _state([0.0, 0.0, 300.0], [0.0, 0.0, -400.0])
    assert _gfold.plan_descent(r, v, 26000.0, 0.0, SITE, LIMITS, tf_guess_s=2.0) is None


@needs_cvxpy
def test_command_interpolates_plan():
    r, v = _state([0.0, 0.0, 300.0], [0.0, 0.0, -80.0])
    plan = _gfold.plan_descent(r, v, 26000.0, 0.0, SITE, LIMITS, tf_guess_s=6.0)
    direction, throttle = _gfold.command_from_plan(plan, 0.5 * plan.time_of_flight, 25000.0, C.LANDING_THRUST)
    assert np.linalg.norm(direction) == pytest.approx(1.0)
    assert 0.0 < throttle <= 1.0
    assert _gfold.command_from_plan(plan, plan.time_of_flight + 1.0, 25000.0, C.LANDING_THRUST) is None


def test_config_rejects_unknown_guidance():
    with pytest.raises(ValueError, match="booster_landing_guidance"):
        create_default_config(booster_landing_guidance="magic")


@needs_cvxpy
def test_landing_guidance_uses_gfold_when_enabled():
    cfg = create_default_config(booster_landing_guidance="gfold")
    site = target_landing_site_eci(0.0, cfg.booster_landing_target_downrange_km, config=cfg)
    up = site / np.linalg.norm(site)
    r = up * (C.R_EARTH + 200.0)
    v = np.cross([0.0, 0.0, C.EARTH_ROTATION_RATE], r) - 80.0 * up
    gs = create_guidance_state()
    gs.booster_landing_burn_started = True
    out, gs = compute_booster_guidance(r, v, 0.0, cfg.stage1_dry_mass + 10000.0, "BOOSTER_LANDING", gs, cfg)
    assert out["landing_guidance_mode"] == "gfold"
    assert gs.gfold_plan is not None
    assert out["thrust_on"] is True


def test_landing_guidance_defaults_to_heuristic():
    cfg = create_default_config()
    site = target_landing_site_eci(0.0, cfg.booster_landing_target_downrange_km, config=cfg)
    up = site / np.linalg.norm(site)
    r = up * (C.R_EARTH + 200.0)
    v = np.cross([0.0, 0.0, C.EARTH_ROTATION_RATE], r) - 80.0 * up
    gs = create_guidance_state()
    gs.booster_landing_burn_started = True
    out, gs = compute_booster_guidance(r, v, 0.0, cfg.stage1_dry_mass + 10000.0, "BOOSTER_LANDING", gs, cfg)
    assert out["landing_guidance_mode"] == "heuristic"
    assert gs.gfold_plan is None


def test_api_toggle_maps_to_config():
    import asyncio

    from rlv_sim.server import frontend_config_defaults, map_config

    defaults = asyncio.run(frontend_config_defaults())
    assert "gfold_guidance" in defaults["recovery"]
    recovery = {k: v for k, v in defaults["recovery"].items() if k != "gfold_available"}
    setup = {
        "vehicle": defaults["vehicle"], "mission": defaults["mission"],
        "recovery": {**recovery, "gfold_guidance": True}, "physics": defaults["physics"],
    }
    from rlv_sim.api_models import SimulationSetup

    if _gfold.AVAILABLE:
        assert map_config(SimulationSetup(**setup)).booster_landing_guidance == "gfold"
    else:
        with pytest.raises(ValueError, match="cvxpy"):
            map_config(SimulationSetup(**setup))

"""Unit tests for booster recovery guidance (flip, boostback, entry, landing)."""

import numpy as np
import pytest

from rlv_sim import constants as C
from rlv_sim._guidance_booster import compute_booster_guidance
from rlv_sim._guidance_common import create_guidance_state
from rlv_sim.config_factory import create_default_config
from rlv_sim.recovery import target_landing_site_eci
from rlv_sim.utils import compute_relative_velocity


@pytest.fixture(scope="module")
def cfg():
    return create_default_config()


def _local_frame(cfg):
    site = target_landing_site_eci(0.0, cfg.booster_landing_target_downrange_km, config=cfg)
    up = site / np.linalg.norm(site)
    east = np.cross([0.0, 0.0, 1.0], up)
    east /= np.linalg.norm(east)
    north = np.cross(up, east)
    return up, east, north


def _booster_state(cfg, altitude_m, east_offset_m=0.0, v_east=0.0, v_up=0.0):
    """Inertial (r, v) above the landing site with a ground-relative velocity."""
    up, east, _ = _local_frame(cfg)
    r = up * (C.R_EARTH + altitude_m) + east * east_offset_m
    omega = np.array([0.0, 0.0, C.EARTH_ROTATION_RATE])
    v = np.cross(omega, r) + v_east * east + v_up * up
    return r, v


def _mass(cfg, propellant_kg=10000.0):
    return float(cfg.stage1_dry_mass) + propellant_kg


def _angle_deg(a, b):
    a = np.asarray(a) / np.linalg.norm(a)
    b = np.asarray(b) / np.linalg.norm(b)
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b), -1.0, 1.0))))


def _guide(cfg, phase, r, v, m, gs=None):
    out, gs = compute_booster_guidance(r, v, 0.0, m, phase, gs or create_guidance_state(), cfg)
    return out, gs


def test_flip_points_retrograde_with_engine_off(cfg):
    r, v = _booster_state(cfg, 70000.0, east_offset_m=60000.0, v_east=1500.0, v_up=500.0)
    out, _ = _guide(cfg, "BOOSTER_FLIP", r, v, _mass(cfg))
    retrograde = -compute_relative_velocity(r, v, cfg.runtime_wind_offset_mps)
    assert out["thrust_on"] is False
    assert _angle_deg(out["thrust_direction"], retrograde) < 1e-6


def test_boostback_burns_at_fixed_throttle(cfg):
    r, v = _booster_state(cfg, 70000.0, east_offset_m=60000.0, v_east=1500.0, v_up=500.0)
    out, _ = _guide(cfg, "BOOSTER_BOOSTBACK", r, v, _mass(cfg, 30000.0))
    assert out["thrust_on"] is True
    assert out["throttle"] == pytest.approx(0.90)
    assert np.linalg.norm(out["thrust_direction"]) == pytest.approx(1.0)


def test_entry_burn_fires_when_fast_and_far(cfg):
    r, v = _booster_state(cfg, 50000.0, east_offset_m=100000.0, v_east=-300.0, v_up=-1500.0)
    out, _ = _guide(cfg, "BOOSTER_ENTRY", r, v, _mass(cfg, 15000.0))
    retrograde = -compute_relative_velocity(r, v, cfg.runtime_wind_offset_mps)
    assert out["thrust_on"] is True
    assert out["throttle"] == pytest.approx(0.45)
    # Steering stays inside the 30 deg cone around retrograde.
    assert _angle_deg(out["thrust_direction"], retrograde) <= 30.0 + 1e-6


def test_entry_coasts_below_speed_gate(cfg):
    r, v = _booster_state(cfg, 50000.0, east_offset_m=100000.0, v_up=-100.0)
    out, _ = _guide(cfg, "BOOSTER_ENTRY", r, v, _mass(cfg, 15000.0))
    assert out["thrust_on"] is False


def test_landing_burn_waits_above_ignition_corridor(cfg):
    r, v = _booster_state(cfg, 10000.0, v_up=-50.0)
    out, gs = _guide(cfg, "BOOSTER_LANDING", r, v, _mass(cfg))
    assert out["thrust_on"] is False
    assert gs.booster_landing_burn_started is False


def test_landing_burn_ignites_low_and_stays_inside_touchdown_cone(cfg):
    # Suicide-burn ignition at 80 m/s descent is ~320 m (1.8x safety factor).
    r, v = _booster_state(cfg, 200.0, east_offset_m=20.0, v_east=5.0, v_up=-80.0)
    out, gs = _guide(cfg, "BOOSTER_LANDING", r, v, _mass(cfg))
    assert gs.booster_landing_burn_started is True
    assert out["thrust_on"] is True
    assert 0.0 < out["throttle"] <= 1.0
    cone = min(cfg.booster_terminal_attitude_error_max_deg, cfg.landing_leg_max_tilt_deg)
    assert _angle_deg(out["thrust_direction"], r / np.linalg.norm(r)) <= cone + 1e-6


def test_landing_burn_stays_latched(cfg):
    gs = create_guidance_state()
    gs.booster_landing_burn_started = True
    # Above the ignition corridor, a latched burn must keep burning.
    r, v = _booster_state(cfg, 10000.0, v_up=-200.0)
    out, gs = _guide(cfg, "BOOSTER_LANDING", r, v, _mass(cfg), gs)
    assert out["thrust_on"] is True
    assert gs.booster_landing_burn_started is True


def test_landing_engine_cuts_just_above_touchdown(cfg):
    gs = create_guidance_state()
    gs.booster_landing_burn_started = True
    r, v = _booster_state(cfg, 5.0, v_up=-1.0)
    out, _ = _guide(cfg, "BOOSTER_LANDING", r, v, _mass(cfg), gs)
    assert out["throttle"] == 0.0


def test_guidance_without_config_uses_defaults(cfg):
    r, v = _booster_state(cfg, 70000.0, east_offset_m=60000.0, v_east=1500.0, v_up=500.0)
    out, _ = compute_booster_guidance(r, v, 0.0, _mass(cfg), "BOOSTER_FLIP")
    assert out["thrust_on"] is False

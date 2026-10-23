"""Navigation sensors and the abort monitor (no tests covered them before)."""

import numpy as np
import pytest

from rlv_sim.config.factory import create_test_config
from rlv_sim.core import constants as C
from rlv_sim.core.state import State
from rlv_sim.guidance.abort import AbortMonitor
from rlv_sim.vehicle.navigation import update_navigation_estimate


def _state(t=0.0, r=None, v=None, cfg=None):
    return State(
        r=np.array([C.R_EARTH + 50_000.0, 0.0, 0.0]) if r is None else r,
        v=np.array([0.0, 1500.0, 200.0]) if v is None else v,
        q=np.array([1.0, 0.0, 0.0, 0.0]), omega=np.zeros(3), m=30_000.0, t=t, sim_config=cfg,
    )


def test_orbiter_and_booster_draw_different_gps_noise():
    cfg = create_test_config(enable_gps=True, gps_seed=7)
    s = _state(cfg=cfg)
    orb = update_navigation_estimate(None, s, cfg, 0.05, vehicle_type="orbiter").estimate
    bst = update_navigation_estimate(None, s, cfg, 0.05, vehicle_type="booster").estimate
    assert not np.allclose(orb.position_eci_m, bst.position_eci_m)


def test_gps_only_estimate_coasts_between_fixes():
    cfg = create_test_config(enable_gps=True, gps_update_period_s=1.0, gps_seed=3,
                             gps_position_sigma_m=0.0, gps_velocity_sigma_mps=0.0)
    s0 = _state(cfg=cfg)
    nav = update_navigation_estimate(None, s0, cfg, 0.1)
    s1 = _state(t=0.5, r=s0.r + s0.v * 0.5, cfg=cfg)
    nav = update_navigation_estimate(nav, s1, cfg, 0.1)
    # Without propagation the estimate would stay ~750 m behind.
    assert nav.estimate.position_error_m < 5.0
    assert nav.estimate.time_s == pytest.approx(0.5)


def test_body_rate_abort_threshold_is_configurable():
    monitor = AbortMonitor(max_tumble_rate=0.2)
    r = np.array([C.R_EARTH + 200_000.0, 0.0, 0.0])
    out = monitor.check(r, np.array([0.0, 7000.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]),
                        np.array([0.0, 0.3, 0.0]), t=100.0)
    assert out["abort"] and "Tumble" in out["reason"]
    cfg = create_test_config(abort_max_body_rate_rad_s=0.2)
    assert cfg.abort_max_body_rate_rad_s == 0.2

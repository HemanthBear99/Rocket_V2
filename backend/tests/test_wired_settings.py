"""Settings that used to do nothing now change the simulation."""

import numpy as np
import pytest

from rlv_sim.config.factory import create_test_config
from rlv_sim.core import constants as C
from rlv_sim.guidance.ascent import launch_azimuth_rad
from rlv_sim.guidance.recovery import target_landing_site_eci
from rlv_sim.physics.forces import compute_configured_atmosphere_properties


def test_launch_azimuth_follows_target_inclination():
    lat = 28.608402
    # Below the launch latitude: unreachable, clamped to due east.
    assert launch_azimuth_rad(create_test_config(target_inclination_deg=28.5)) == pytest.approx(np.pi / 2)
    # sin(Az) = cos(i) / cos(lat)
    cfg = create_test_config(target_inclination_deg=51.6)
    expected = np.arcsin(np.cos(np.radians(51.6)) / np.cos(np.radians(lat)))
    assert launch_azimuth_rad(cfg) == pytest.approx(expected)
    assert launch_azimuth_rad(cfg) < np.pi / 2  # north of east


def test_movable_target_shifts_an_explicit_site():
    base = {"booster_landing_site_lat_deg": 28.6, "booster_landing_site_lon_deg": -80.6,
                "runtime_wind_offset_mps": 8.0}
    fixed = target_landing_site_eci(0.0, 0.0, config=create_test_config(**base))
    moved = target_landing_site_eci(
        0.0, 0.0, config=create_test_config(enable_movable_landing_target=True, **base))
    assert np.linalg.norm(moved - fixed) > 1.0


def test_gram_option_changes_density_with_latitude():
    r_equator = np.array([C.R_EARTH + 5000.0, 0.0, 0.0])
    plain = compute_configured_atmosphere_properties(5000.0, create_test_config(), r=r_equator)[2]
    gram = compute_configured_atmosphere_properties(
        5000.0, create_test_config(enable_high_fidelity_gram=True), r=r_equator)[2]
    assert gram != pytest.approx(plain)

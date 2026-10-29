"""EGM96 coefficients and Earth-fixed evaluation of the geopotential."""

import numpy as np
import pytest

from rlv_sim.core import constants as C
from rlv_sim.physics import forces

# Spot values from the official EGM96 file (tide-free, normalized).
OFFICIAL = {
    ("C", 2, 2): 0.243914352398e-05, ("S", 2, 2): -0.140016683654e-05,
    ("S", 3, 2): -0.619025944205e-06, ("S", 3, 3): 0.141435626958e-05,
    ("C", 5, 3): -0.451955406071e-06, ("C", 6, 2): 0.481732442832e-07,
    ("S", 5, 3): -0.214847190624e-06,
}


@pytest.mark.parametrize("key", sorted(OFFICIAL))
def test_coefficients_match_official_egm96(key):
    table = forces._EGM96_C if key[0] == "C" else forces._EGM96_S
    assert table[(key[1], key[2])] == pytest.approx(OFFICIAL[key], rel=1e-12)


def test_geopotential_rotates_with_the_earth():
    r0 = np.array([4.2e6, 3.1e6, 3.6e6])
    a0 = forces.compute_egm96_gravity_accel(r0, t=0.0)
    t = 3600.0
    th = C.EARTH_ROTATION_RATE * t
    rot = np.array([[np.cos(th), -np.sin(th), 0.0], [np.sin(th), np.cos(th), 0.0], [0.0, 0.0, 1.0]])
    a_t = forces.compute_egm96_gravity_accel(rot @ r0, t=t)
    # A point fixed on the Earth feels the same Earth-fixed gravity.
    assert np.allclose(rot.T @ a_t, a0, rtol=1e-12, atol=1e-15)

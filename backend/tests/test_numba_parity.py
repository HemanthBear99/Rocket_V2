"""Compiled kernels must match the pure-Python paths they mirror."""

import numpy as np
import pytest

from rlv_sim.config.factory import create_default_config
from rlv_sim.core import constants as C
from rlv_sim.core import utils
from rlv_sim.guidance import recovery
from rlv_sim.physics import forces, numba_kernels

pytestmark = pytest.mark.skipif(not numba_kernels.AVAILABLE, reason="numba not installed")


def _python_path(monkeypatch):
    monkeypatch.setattr(numba_kernels, "AVAILABLE", False)


@pytest.mark.parametrize("r", [
    np.array([C.R_EARTH + 400e3, 0.0, 0.0]),
    np.array([3.1e6, -4.2e6, 3.9e6]),
])
def test_egm96_kernel_matches_python(monkeypatch, r):
    compiled = forces.compute_egm96_gravity_accel(r)
    _python_path(monkeypatch)
    reference = forces.compute_egm96_gravity_accel(r)
    assert np.allclose(compiled, reference, rtol=1e-12, atol=0.0)


def _predict(mass_kg=None):
    cfg = create_default_config()
    r = np.array([C.R_EARTH + 20_000.0, 0.0, 0.0])
    v = np.array([-300.0, 150.0, 400.0])
    return recovery._propagate_2body_to_surface(r, v, config=cfg, mass_kg=mass_kg, aero_mode="BOOSTER_ENTRY")


@pytest.mark.parametrize("mass_kg", [None, 30_000.0])
def test_impact_predictor_kernel_matches_python(monkeypatch, mass_kg):
    compiled = _predict(mass_kg)
    _python_path(monkeypatch)
    reference = _predict(mass_kg)
    assert np.allclose(compiled[0], reference[0], rtol=0.0, atol=1e-3)
    assert compiled[1] == pytest.approx(reference[1], abs=1e-6)


@pytest.mark.xfail(strict=True, reason="RC8: compiled predictor omits runtime wind-profile modes")
def test_impact_predictor_kernel_matches_python_with_wind_modes(monkeypatch):
    with utils.wind_profile_context(((8.0, 3000.0, 0.4),)):
        compiled = _predict(30_000.0)
        _python_path(monkeypatch)
        reference = _predict(30_000.0)
    assert np.allclose(compiled[0], reference[0], rtol=0.0, atol=1e-3)

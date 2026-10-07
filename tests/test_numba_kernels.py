"""Compiled kernels must match the pure-Python gravity and impact predictor."""

import numpy as np
import pytest

from rlv_sim import _numba_kernels, forces, recovery
from rlv_sim import constants as C
from rlv_sim.config_factory import create_default_config

pytestmark = pytest.mark.skipif(not _numba_kernels.AVAILABLE, reason="numba not installed")


def _random_positions(rng, count, max_alt):
    for _ in range(count):
        up = rng.normal(size=3)
        up /= np.linalg.norm(up)
        yield up, up * (C.R_EARTH + rng.uniform(0.0, max_alt))


def test_egm96_kernel_matches_python(monkeypatch):
    rng = np.random.default_rng(1)
    for _, r in _random_positions(rng, 200, 2.0e6):
        compiled = forces.compute_egm96_gravity_accel(r)
        monkeypatch.setattr(_numba_kernels, "AVAILABLE", False)
        reference = forces.compute_egm96_gravity_accel(r)
        monkeypatch.setattr(_numba_kernels, "AVAILABLE", True)
        np.testing.assert_allclose(compiled, reference, rtol=1e-13, atol=0.0)


def test_impact_predictor_kernel_matches_python(monkeypatch):
    cfg = create_default_config()
    rng = np.random.default_rng(2)
    for up, r in _random_positions(rng, 20, 80.0e3):
        r = up * (C.R_EARTH + 5.0e3) + (r - up * C.R_EARTH)
        v = rng.normal(size=3) * 800.0 - up * rng.uniform(100.0, 1500.0)
        kwargs = {"mass_kg": 30000.0, "config": cfg, "aero_mode": "BOOSTER_ENTRY"}
        compiled = recovery._propagate_2body_to_surface(r, v, **kwargs)
        monkeypatch.setattr(_numba_kernels, "AVAILABLE", False)
        reference = recovery._propagate_2body_to_surface(r, v, **kwargs)
        monkeypatch.setattr(_numba_kernels, "AVAILABLE", True)
        assert (compiled is None) == (reference is None)
        if compiled is not None:
            np.testing.assert_allclose(compiled[0], reference[0], rtol=0.0, atol=1e-6)
            assert compiled[1] == pytest.approx(reference[1], abs=1e-9)

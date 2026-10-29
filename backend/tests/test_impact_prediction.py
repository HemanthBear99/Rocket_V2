"""Ballistic impact time from the RTLS predictor."""

import numpy as np
import pytest

from rlv_sim.core import constants as C
from rlv_sim.guidance import recovery
from rlv_sim.physics import numba_kernels


@pytest.mark.parametrize("compiled", [False, True])
@pytest.mark.parametrize("h", [1000.0, 1040.0, 5000.0])
def test_drag_free_drop_time(monkeypatch, compiled, h):
    if compiled and not numba_kernels.AVAILABLE:
        pytest.skip("numba not installed")
    monkeypatch.setattr(numba_kernels, "AVAILABLE", compiled)
    r = np.array([C.R_EARTH + h, 0.0, 0.0])
    _, t_impact = recovery._propagate_2body_to_surface(r, np.zeros(3), dt=2.0)
    g = C.MU_EARTH / (C.R_EARTH + h / 2.0) ** 2
    # Linear interpolation inside a 2 s step: error well under one step.
    assert t_impact == pytest.approx(np.sqrt(2.0 * h / g), abs=0.15)

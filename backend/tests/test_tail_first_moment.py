"""Tail-first CP relocation is booster geometry, chosen by vehicle model."""

import numpy as np

from rlv_sim.config.factory import create_test_config
from rlv_sim.core import constants as C
from rlv_sim.physics.forces import compute_aerodynamic_moment


def _moment(vehicle_model):
    cfg = create_test_config()
    r = np.array([C.R_EARTH + 20_000.0, 0.0, 0.0])
    # Body z = inertial z (identity attitude); fall tail-first with some AoA.
    v = np.array([80.0, 0.0, -600.0]) + np.cross([0.0, 0.0, C.EARTH_ROTATION_RATE], r)
    q = np.array([1.0, 0.0, 0.0, 0.0])
    # Same CG for both: it sits inside the booster's recovery CG range.
    return compute_aerodynamic_moment(r, v, q, cg_pos_z=25.0, config=cfg,
                                      vehicle_model=vehicle_model)


def test_tail_first_cp_applies_only_to_booster():
    booster = _moment("booster")
    stacked = _moment("stacked")
    assert np.linalg.norm(booster) > 0.0
    # A CG-band check would treat both alike; the vehicle model must decide.
    assert not np.allclose(booster, stacked)

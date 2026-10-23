"""Propellant-exhaustion step split terminates and lands on the dry mass."""

import numpy as np

from rlv_sim.config.factory import create_test_config
from rlv_sim.core import constants as C
from rlv_sim.core.state import State
from rlv_sim.physics.dynamics import DynamicsContext
from rlv_sim.physics.integrators import rk4_step


def test_split_at_rounding_edge_terminates():
    cfg = create_test_config()
    ctx = DynamicsContext(
        thrust_on=True, throttle=1.0, dry_mass=4000.0, stage=2, vehicle_model="orbiter",
        booster_aero_mode=None, thrust_magnitude_override=None, grid_fin_command=None, config=cfg,
    )
    state = State(r=np.array([C.R_EARTH + 300_000.0, 0.0, 0.0]), v=np.array([0.0, 7600.0, 0.0]),
                  q=np.array([1.0, 0.0, 0.0, 0.0]), omega=np.zeros(3), m=6675.133319624401,
                  t=0.0, sim_config=cfg)
    new = rk4_step(state, np.zeros(3), 20.0, ctx=ctx)
    assert new.m == 4000.0
    assert new.t == 20.0

"""Separation kicks conserve momentum and act in the right frame."""

import numpy as np
import pytest

from rlv_sim.config.factory import create_default_config
from rlv_sim.core import constants as C
from rlv_sim.core.frames import direction_to_quaternion, rotate_vector_by_quaternion
from rlv_sim.core.state import State
from rlv_sim.mission.vehicle_split import _compute_separation_adjustments
from rlv_sim.physics.mass import compute_inertia_tensor


def _split():
    cfg = create_default_config()
    q = direction_to_quaternion(np.array([0.3, 0.8, 0.52]))
    state = State(r=np.array([C.R_EARTH + 70e3, 0.0, 0.0]), v=np.array([500.0, 1800.0, 300.0]),
                  q=q, omega=np.zeros(3), m=200_000.0, t=150.0, sim_config=cfg)
    m_o = cfg.stage2_dry_mass + cfg.stage2_prop_mass + cfg.payload_mass
    m_b = cfg.stage1_dry_mass + cfg.stage1_landing_fuel_reserve_kg
    return cfg, state, m_o, m_b, _compute_separation_adjustments(state, m_o, m_b, config=cfg)


def test_linear_and_angular_momentum_conserved():
    cfg, state, m_o, m_b, (dv_o, dv_b, dw_o, dw_b) = _split()
    assert np.allclose(m_o * dv_o + m_b * dv_b, 0.0, atol=1e-9)
    i_o = compute_inertia_tensor(m_o, "orbiter", config=cfg)[0, 0]
    i_b = compute_inertia_tensor(m_b, "booster", cfg.stage1_landing_fuel_reserve_kg, config=cfg)[0, 0]
    assert np.allclose(i_o * dw_o + i_b * dw_b, 0.0, atol=1e-6)
    assert np.linalg.norm(dw_o - dw_b) == pytest.approx(cfg.separation_tumble_rate)


def test_tumble_axis_is_perpendicular_to_body_thrust_axis():
    _, state, _, _, (_, _, dw_o, _) = _split()
    # Spin axis is perpendicular to the separation (thrust) axis: in body
    # axes that means no component along body +z.
    assert abs(dw_o[2]) < 1e-12
    assert np.linalg.norm(rotate_vector_by_quaternion(dw_o, state.q)) == pytest.approx(np.linalg.norm(dw_o))

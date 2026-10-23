"""Per-axis jet damping (physics.dynamics.compute_jet_damping_torque)."""

import numpy as np
import pytest

from rlv_sim.config.factory import create_test_config
from rlv_sim.core import constants as C
from rlv_sim.physics.dynamics import DynamicsContext, compute_jet_damping_torque


def _ctx(vehicle_model="booster", mode="BOOSTER_LANDING", stage=1):
    return DynamicsContext(
        thrust_on=True, throttle=1.0, dry_mass=C.DRY_MASS, stage=stage,
        vehicle_model=vehicle_model, booster_aero_mode=mode,
        thrust_magnitude_override=None, grid_fin_command=None,
        config=create_test_config(),
    )


def test_no_flow_no_damping():
    tau = compute_jet_damping_torque(0.0, 20.0, np.ones(3), _ctx())
    assert np.allclose(tau, 0.0)


def test_centre_engine_roll_damping_is_tiny():
    """Single centre-engine landing burn: roll damping only from the exit disk."""
    mdot, l = -280.0, 20.0
    omega = np.array([0.0, 0.0, 1.0])
    tau = compute_jet_damping_torque(mdot, l, omega, _ctx(mode="BOOSTER_LANDING"))
    expected_roll = 280.0 * 0.5 * C.STAGE1_NOZZLE_EXIT_RADIUS_M ** 2
    assert tau[2] == pytest.approx(expected_roll)
    # The old model used mdot * l^2 for roll: >1000x larger here.
    assert tau[2] * 1000 < 280.0 * l ** 2


def test_pitch_damping_keeps_axial_lever_arm():
    mdot, l = -2500.0, 30.0
    tau = compute_jet_damping_torque(mdot, l, np.array([1.0, 0.0, 0.0]), _ctx(vehicle_model="stacked", mode=None))
    assert tau[0] == pytest.approx(2500.0 * l ** 2, rel=0.01)
    assert tau[1] == 0.0 and tau[2] == 0.0


def test_ring_engines_raise_roll_damping():
    mdot = -2500.0
    omega = np.array([0.0, 0.0, 1.0])
    nine = compute_jet_damping_torque(mdot, 30.0, omega, _ctx(vehicle_model="stacked", mode=None))[2]
    three = compute_jet_damping_torque(mdot, 30.0, omega, _ctx(mode="BOOSTER_BOOSTBACK"))[2]
    one = compute_jet_damping_torque(mdot, 30.0, omega, _ctx(mode="BOOSTER_LANDING"))[2]
    assert nine > three > one > 0.0
    # Roll stays a small fraction of pitch even with all nine engines.
    pitch = compute_jet_damping_torque(mdot, 30.0, np.array([1.0, 0.0, 0.0]), _ctx(vehicle_model="stacked", mode=None))[0]
    assert nine < 0.01 * pitch

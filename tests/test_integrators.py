"""Unit tests for rlv_sim.integrators.rk4_step.

Covers the behaviors identified in the verified test architecture report as
provable test cases: quaternion-norm preservation across RK4 substeps, mass
monotonicity/floor clamping, the propellant-depletion mid-step sub-stepping
recursion, and the documented input-validation errors.
"""

import numpy as np
import pytest

from rlv_sim import constants as C
from rlv_sim.config_factory import create_default_config, create_test_config
from rlv_sim.dynamics import DynamicsContext
from rlv_sim.integrators import integrate, rk4_step
from rlv_sim.mass import compute_mass_derivative
from rlv_sim.state import State


def _leo_state(m: float, omega=None) -> State:
    return State(
        r=np.array([C.R_EARTH + 400_000.0, 0.0, 0.0]),
        v=np.array([0.0, 7669.0, 0.0]),
        q=np.array([1.0, 0.0, 0.0, 0.0]),
        omega=np.zeros(3) if omega is None else np.asarray(omega, dtype=float),
        m=m,
        t=0.0,
    )


@pytest.fixture
def coast_ctx():
    config = create_test_config()
    return DynamicsContext(
        thrust_on=False,
        throttle=0.0,
        dry_mass=C.DRY_MASS,
        stage=1,
        vehicle_model="orbiter",
        config=config,
    )


class TestQuaternionNormPreservation:
    def test_norm_preserved_over_many_coast_steps(self, coast_ctx):
        state = _leo_state(m=C.DRY_MASS + 5000.0, omega=[0.05, 0.02, -0.01])
        for _ in range(50):
            state = rk4_step(state, torque=np.zeros(3), dt=0.05, ctx=coast_ctx)
            assert np.linalg.norm(state.q) == pytest.approx(1.0, abs=1e-9)

    def test_norm_preserved_with_nonzero_torque(self, coast_ctx):
        state = _leo_state(m=C.DRY_MASS + 5000.0)
        torque = np.array([1000.0, -500.0, 200.0])
        for _ in range(20):
            state = rk4_step(state, torque=torque, dt=0.05, ctx=coast_ctx)
            assert np.linalg.norm(state.q) == pytest.approx(1.0, abs=1e-9)

    def test_state_stays_finite(self, coast_ctx):
        state = _leo_state(m=C.DRY_MASS + 5000.0, omega=[0.1, -0.1, 0.05])
        for _ in range(30):
            state = rk4_step(state, torque=np.zeros(3), dt=0.05, ctx=coast_ctx)
        assert np.all(np.isfinite(state.to_vector()))


class TestMassBehaviorDuringCoast:
    def test_mass_unchanged_when_thrust_off(self, coast_ctx):
        m0 = C.DRY_MASS + 5000.0
        state = _leo_state(m=m0)
        state = rk4_step(state, torque=np.zeros(3), dt=0.05, ctx=coast_ctx)
        assert state.m == pytest.approx(m0)


class TestPropellantDepletionSubStepping:
    def test_mass_clamps_exactly_to_dry_mass_when_depleted_mid_step(self):
        config = create_default_config()
        dry_mass = C.STAGE2_DRY_MASS
        m0 = dry_mass + 50.0  # small propellant margin

        mdot0 = compute_mass_derivative(
            m0,
            thrust_on=True,
            throttle=1.0,
            dry_mass=dry_mass,
            stage=2,
            thrust_scale=config.runtime_thrust_scale,
            isp_scale=config.runtime_isp_scale,
            stage2_thrust_vac=config.stage2_thrust_vac,
            stage2_isp_vac=config.stage2_isp_vac,
        )
        assert mdot0 < 0.0, "expected active mass flow at full throttle"

        # Choose a dt several times longer than the time-to-dry so the
        # recursive powered->coast split inside rk4_step is exercised.
        time_to_dry = 50.0 / abs(mdot0)
        dt = time_to_dry * 4.0

        ctx = DynamicsContext(
            thrust_on=True,
            throttle=1.0,
            dry_mass=dry_mass,
            stage=2,
            vehicle_model="orbiter",
            config=config,
        )
        state = _leo_state(m=m0)
        new_state = rk4_step(state, torque=np.zeros(3), dt=dt, ctx=ctx)

        assert new_state.m == pytest.approx(dry_mass, abs=1e-6)
        assert new_state.m >= dry_mass - 1e-6
        assert new_state.t == pytest.approx(dt)

    def test_no_depletion_when_dt_small_relative_to_propellant(self):
        config = create_default_config()
        dry_mass = C.STAGE2_DRY_MASS
        m0 = dry_mass + 50_000.0  # large propellant margin

        ctx = DynamicsContext(
            thrust_on=True,
            throttle=1.0,
            dry_mass=dry_mass,
            stage=2,
            vehicle_model="orbiter",
            config=config,
        )
        state = _leo_state(m=m0)
        new_state = rk4_step(state, torque=np.zeros(3), dt=0.05, ctx=ctx)
        assert new_state.m < m0  # propellant was consumed
        assert new_state.m > dry_mass  # but not exhausted


class TestInputValidation:
    def test_non_positive_dt_raises(self, coast_ctx):
        state = _leo_state(m=C.DRY_MASS + 1000.0)
        with pytest.raises(ValueError, match="dt"):
            rk4_step(state, torque=np.zeros(3), dt=0.0, ctx=coast_ctx)

    def test_negative_dt_raises(self, coast_ctx):
        state = _leo_state(m=C.DRY_MASS + 1000.0)
        with pytest.raises(ValueError, match="dt"):
            rk4_step(state, torque=np.zeros(3), dt=-0.1, ctx=coast_ctx)

    def test_wrong_torque_shape_raises(self, coast_ctx):
        state = _leo_state(m=C.DRY_MASS + 1000.0)
        with pytest.raises(ValueError, match="shape"):
            rk4_step(state, torque=np.array([1.0, 2.0]), dt=0.05, ctx=coast_ctx)

    def test_nan_torque_raises(self, coast_ctx):
        state = _leo_state(m=C.DRY_MASS + 1000.0)
        with pytest.raises(ValueError, match="NaN"):
            rk4_step(state, torque=np.array([np.nan, 0.0, 0.0]), dt=0.05, ctx=coast_ctx)


class TestIntegrateDispatcher:
    def test_unknown_method_raises(self, coast_ctx):
        state = _leo_state(m=C.DRY_MASS + 1000.0)
        with pytest.raises(ValueError, match="Unknown integration method"):
            integrate(state, torque=np.zeros(3), dt=0.05, ctx=coast_ctx, method="euler")

    def test_rk4_method_delegates_to_rk4_step(self, coast_ctx):
        state = _leo_state(m=C.DRY_MASS + 1000.0)
        result = integrate(state, torque=np.zeros(3), dt=0.05, ctx=coast_ctx, method="rk4")
        assert result.t == pytest.approx(0.05)

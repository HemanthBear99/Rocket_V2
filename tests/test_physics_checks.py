"""Unit tests for rlv_sim.physics_checks — the invariant-gate functions.

These are pure functions with no simulation dependency, and are confirmed
(via grep of _run_simulation.py:170 and _run_full_mission.py:71,223) to be
called every step in both single-vehicle and full-mission hot paths via
validate_state(..., abort_on_error=True). That resolves a prior open
question about whether these invariants are actually enforced live.
"""

import numpy as np
import pytest

from rlv_sim import constants as C
from rlv_sim.physics_checks import (
    ValidationError,
    check_angular_velocity_reasonable,
    check_finite_state,
    check_mass_valid,
    check_position_valid,
    check_quaternion_norm,
    check_velocity_reasonable,
    compute_total_energy,
    validate_energy_conservation,
    validate_state,
)
from rlv_sim.state import State


def _valid_state(**overrides):
    defaults = {
        "r": np.array([C.R_EARTH + 400_000.0, 0.0, 0.0]),
        "v": np.array([0.0, 7669.0, 0.0]),
        "q": np.array([1.0, 0.0, 0.0, 0.0]),
        "omega": np.zeros(3),
        "m": C.DRY_MASS + 1000.0,
        "t": 0.0,
    }
    defaults.update(overrides)
    return State(**defaults)


class TestCheckFiniteState:
    def test_finite_state_passes(self):
        assert check_finite_state(_valid_state()) is True

    @pytest.mark.parametrize("field", ["r", "v", "omega"])
    def test_nan_vector_raises(self, field):
        state = _valid_state()
        setattr(state, field, np.array([np.nan, 0.0, 0.0]))
        with pytest.raises(ValidationError):
            check_finite_state(state)

    def test_nan_mass_raises(self):
        state = _valid_state()
        state.m = float("nan")
        with pytest.raises(ValidationError):
            check_finite_state(state)

    def test_inf_velocity_raises(self):
        state = _valid_state()
        state.v = np.array([np.inf, 0.0, 0.0])
        with pytest.raises(ValidationError):
            check_finite_state(state)


class TestCheckQuaternionNorm:
    def test_unit_quaternion_passes(self):
        assert check_quaternion_norm(np.array([1.0, 0.0, 0.0, 0.0])) is True

    def test_within_tolerance_passes(self):
        q = np.array([1.0 + C.QUATERNION_NORM_TOL / 2, 0.0, 0.0, 0.0])
        assert check_quaternion_norm(q, tolerance=C.QUATERNION_NORM_TOL) is True

    def test_grossly_denormalized_raises(self):
        with pytest.raises(ValidationError):
            check_quaternion_norm(np.array([2.0, 0.0, 0.0, 0.0]))

    def test_zero_quaternion_raises(self):
        with pytest.raises(ValidationError):
            check_quaternion_norm(np.zeros(4))


class TestCheckPositionValid:
    def test_surface_altitude_passes(self):
        assert check_position_valid(np.array([C.R_EARTH, 0.0, 0.0])) is True

    def test_inside_earth_raises(self):
        with pytest.raises(ValidationError):
            check_position_valid(np.array([C.R_EARTH * 0.4, 0.0, 0.0]))

    def test_exactly_at_half_radius_boundary(self):
        # Boundary is `< R_EARTH * 0.5` -> exactly half radius should pass.
        assert check_position_valid(np.array([C.R_EARTH * 0.5, 0.0, 0.0])) is True


class TestCheckMassValid:
    def test_mass_within_bounds_passes(self):
        assert check_mass_valid(C.DRY_MASS + 500.0, dry_mass=C.DRY_MASS) is True

    def test_mass_below_dry_mass_raises(self):
        with pytest.raises(ValidationError):
            check_mass_valid(C.DRY_MASS * 0.9, dry_mass=C.DRY_MASS)

    def test_mass_slightly_below_dry_mass_tolerated(self):
        # 0.999 * dry_mass undershoot is explicitly allowed (numerical slack).
        assert check_mass_valid(C.DRY_MASS * 0.9995, dry_mass=C.DRY_MASS) is True

    def test_mass_above_max_raises(self):
        with pytest.raises(ValidationError):
            check_mass_valid(C.INITIAL_MASS * 1.5, dry_mass=C.DRY_MASS)

    def test_custom_max_mass_respected(self):
        with pytest.raises(ValidationError):
            check_mass_valid(6000.0, dry_mass=1000.0, max_mass=5000.0)


class TestCheckVelocityReasonable:
    def test_orbital_velocity_passes(self):
        assert check_velocity_reasonable(np.array([0.0, 7800.0, 0.0])) is True

    def test_excessive_velocity_raises(self):
        with pytest.raises(ValidationError):
            check_velocity_reasonable(np.array([20000.0, 0.0, 0.0]))


class TestCheckAngularVelocityReasonable:
    def test_small_omega_passes(self):
        assert check_angular_velocity_reasonable(np.array([0.1, 0.0, 0.0])) is True

    def test_tumbling_omega_raises(self):
        with pytest.raises(ValidationError):
            check_angular_velocity_reasonable(np.array([15.0, 0.0, 0.0]))


class TestValidateStateAggregate:
    def test_valid_state_passes_abort_mode(self):
        ok, msg = validate_state(_valid_state(), abort_on_error=True)
        assert ok is True
        assert msg is None

    def test_invalid_state_raises_when_abort_on_error(self):
        state = _valid_state(m=float("nan"))
        with pytest.raises(ValidationError):
            validate_state(state, abort_on_error=True)

    def test_invalid_state_returns_false_when_not_abort(self):
        state = _valid_state()
        state.q = np.array([5.0, 0.0, 0.0, 0.0])
        ok, msg = validate_state(state, abort_on_error=False)
        assert ok is False
        assert msg is not None and "Quaternion" in msg

    def test_first_failing_check_short_circuits(self):
        # Non-finite state should be reported (finite check runs first),
        # not a downstream check like mass/velocity.
        state = _valid_state(m=float("nan"))
        ok, msg = validate_state(state, abort_on_error=False)
        assert ok is False
        assert "non-finite" in msg


class TestEnergyConservation:
    def test_circular_orbit_energy_is_negative(self):
        r = np.array([C.R_EARTH + 400_000.0, 0.0, 0.0])
        v = np.array([0.0, 7669.0, 0.0])
        e = compute_total_energy(r, v, m=1000.0)
        assert e < 0.0

    def test_zero_radius_returns_zero(self):
        assert compute_total_energy(np.zeros(3), np.zeros(3), m=1000.0) == 0.0

    def test_negative_mass_returns_zero(self):
        r = np.array([C.R_EARTH, 0.0, 0.0])
        assert compute_total_energy(r, np.zeros(3), m=-1.0) == 0.0

    def test_conserved_energy_is_valid(self):
        result = validate_energy_conservation(E_current=-1000.0, E_previous=-1000.0, dt=0.05)
        assert result["valid"] is True
        assert result["relative_error"] == 0.0

    def test_large_energy_jump_is_invalid(self):
        result = validate_energy_conservation(E_current=-500.0, E_previous=-1000.0, dt=0.05)
        assert result["valid"] is False
        assert result["relative_error"] == pytest.approx(0.5)

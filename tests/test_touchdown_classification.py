"""Unit tests for rlv_sim.recovery_hardware.assess_touchdown_contact.

Locks in the classification precedence documented in the verified test
architecture report: landing-leg lock > terminal tilt > speed/pad matrix.
Also includes a regression test that reproduces the CURRENT FAILURE observed
in scratch_full_run2.log (S2 drone-ship touchdown at 30.77 m/s against a
5.0 m/s limit) so that failure mode is asserted, not silently accepted as a
passing baseline.
"""

import numpy as np
import pytest

from rlv_sim import constants as C
from rlv_sim.config_factory import create_test_config
from rlv_sim.recovery_hardware import (
    LandingLegState,
    LandingLegStatus,
    assess_touchdown_contact,
)
from rlv_sim.state import State


def _touchdown_state(vertical_speed_mps: float, horizontal_speed_mps: float = 0.0):
    """A state resting on the surface with a given ground-relative descent rate."""
    r = np.array([C.R_EARTH, 0.0, 0.0])
    # v is in the inertial (ECI) frame; compute_ground_relative_velocity
    # subtracts Earth rotation, so express v directly as the desired
    # ground-relative velocity plus the Earth-rotation term at this r.
    omega_earth = np.array([0.0, 0.0, C.EARTH_ROTATION_RATE])
    v_earth_rotation = np.cross(omega_earth, r)
    v_ground = np.array([-vertical_speed_mps, horizontal_speed_mps, 0.0])
    v = v_ground + v_earth_rotation
    return State(r=r, v=v, q=np.array([1.0, 0.0, 0.0, 0.0]), omega=np.zeros(3), m=30000.0, t=100.0)


@pytest.fixture
def config():
    return create_test_config(
        landing_leg_max_touchdown_speed_mps=5.0,
        landing_leg_max_tilt_deg=10.0,
        booster_pad_tolerance_m=100.0,
        enable_landing_legs=True,
    )


class TestLegDeploymentPrecedence:
    def test_legs_not_locked_fails_regardless_of_speed(self, config):
        state = _touchdown_state(vertical_speed_mps=0.5)
        legs = LandingLegState(status=LandingLegStatus.DEPLOYING, deployed_fraction=0.5)
        result = assess_touchdown_contact(
            state, config, site_error_m=0.0, leg_state=legs, terminal_attitude_error_deg=0.0
        )
        assert result.success is False
        assert result.status == "leg_deployment_failure"
        assert "landing_legs" in result.failed_criteria

    def test_legs_locked_and_gentle_speed_proceeds_to_speed_gate(self, config):
        state = _touchdown_state(vertical_speed_mps=1.0)
        legs = LandingLegState(status=LandingLegStatus.LOCKED, deployed_fraction=1.0)
        result = assess_touchdown_contact(
            state, config, site_error_m=0.0, leg_state=legs, terminal_attitude_error_deg=0.0
        )
        assert result.status != "leg_deployment_failure"


class TestTiltPrecedence:
    def test_excess_tilt_fails_even_at_zero_speed(self, config):
        state = _touchdown_state(vertical_speed_mps=0.0)
        legs = LandingLegState(status=LandingLegStatus.LOCKED, deployed_fraction=1.0)
        result = assess_touchdown_contact(
            state, config, site_error_m=0.0, leg_state=legs, terminal_attitude_error_deg=45.0
        )
        assert result.success is False
        assert result.status == "tipover"
        assert "terminal_attitude_error" in result.failed_criteria

    def test_tilt_within_limit_proceeds_to_speed_gate(self, config):
        state = _touchdown_state(vertical_speed_mps=1.0)
        legs = LandingLegState(status=LandingLegStatus.LOCKED, deployed_fraction=1.0)
        result = assess_touchdown_contact(
            state, config, site_error_m=0.0, leg_state=legs, terminal_attitude_error_deg=5.0
        )
        assert result.status != "tipover"


class TestSpeedPadMatrix:
    LOCKED = LandingLegState(status=LandingLegStatus.LOCKED, deployed_fraction=1.0)

    @pytest.mark.parametrize(
        "vertical_speed,site_error,expected_status,expected_success",
        [
            (2.0, 10.0, "landing_success", True),
            (2.0, 500.0, "offsite_touchdown", False),
            (7.0, 10.0, "hard_landing", False),
            (7.0, 500.0, "hard_offsite_landing", False),
            (15.0, 10.0, "crash_landing", False),
            (15.0, 500.0, "offsite_crash_landing", False),
        ],
    )
    def test_matrix(self, config, vertical_speed, site_error, expected_status, expected_success):
        state = _touchdown_state(vertical_speed_mps=vertical_speed)
        result = assess_touchdown_contact(
            state, config, site_error_m=site_error, leg_state=self.LOCKED, terminal_attitude_error_deg=0.0
        )
        assert result.status == expected_status
        assert result.success is expected_success

    def test_success_requires_empty_failed_criteria(self, config):
        state = _touchdown_state(vertical_speed_mps=2.0)
        result = assess_touchdown_contact(
            state, config, site_error_m=10.0, leg_state=self.LOCKED, terminal_attitude_error_deg=0.0
        )
        assert result.failed_criteria == []
        assert result.success is True


class TestS2DroneShipCurrentFailure:
    """Regression guardrail for the observed CURRENT FAILURE.

    scratch_full_run2.log recorded an S2 drone-ship touchdown at 30.77 m/s
    against a configured 5.0 m/s limit ("S2 DRONE-SHIP CRASH"). This is NOT
    a working baseline -- do not change this test to assert success. It
    exists so that if a future change to guidance/control accidentally
    "fixes" the classification logic itself (rather than the actual descent
    profile) instead of the underlying guidance bug, this test still forces
    an explicit, deliberate decision about the change.
    """

    def test_3077_mps_touchdown_is_classified_as_crash(self, config):
        state = _touchdown_state(vertical_speed_mps=30.77)
        legs = LandingLegState(status=LandingLegStatus.LOCKED, deployed_fraction=1.0)
        result = assess_touchdown_contact(
            state, config, site_error_m=0.0, leg_state=legs, terminal_attitude_error_deg=0.0
        )
        assert result.success is False
        assert result.status == "crash_landing"
        assert "touchdown_speed" in result.failed_criteria
        assert result.touchdown_speed_mps == pytest.approx(30.77, abs=0.1)

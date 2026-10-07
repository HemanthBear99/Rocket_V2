"""Unit tests for rlv_sim.control — the attitude control law.

Covers the 7 testable branches identified in the verified test architecture
report: gain scheduling (scalar vs per-axis), controller resolution by
mission phase, integral engagement/reset on controller or phase switch,
RCS-only deadband, anti-windup clamping, and torque saturation.
"""

import numpy as np
import pytest

from rlv_sim import constants as C
from rlv_sim.config_factory import create_test_config
from rlv_sim.control import (
    ControlState,
    _apply_integral_anti_windup,
    _schedule_gains,
    compute_thrust_axis_error,
    create_control_state,
    pd_control_law,
    prepare_control_state,
    resolve_attitude_controller,
)
from rlv_sim.mission_manager import MissionPhase


class TestScheduleGains:
    def test_scalar_inertia_scales_all_axes_equally(self):
        kp, kd, ki = _schedule_gains(C.IXX_FULL, None, None, None)
        assert kp == pytest.approx(np.full(3, C.KP_ATTITUDE))
        assert kd == pytest.approx(np.full(3, C.KD_ATTITUDE))
        assert ki == pytest.approx(np.full(3, C.KI_ATTITUDE))

    def test_half_inertia_halves_gains(self):
        kp, _kd, _ki = _schedule_gains(C.IXX_FULL / 2, None, None, None)
        assert kp == pytest.approx(np.full(3, C.KP_ATTITUDE / 2))

    def test_per_axis_inertia_scales_independently(self):
        inertia = np.array([C.IXX_FULL, C.IXX_FULL / 2, C.IXX_FULL / 23.0])
        kp, _kd, _ki = _schedule_gains(inertia, None, None, None)
        assert kp[0] == pytest.approx(C.KP_ATTITUDE)
        assert kp[1] == pytest.approx(C.KP_ATTITUDE / 2)
        assert kp[2] == pytest.approx(C.KP_ATTITUDE / 23.0, rel=1e-3)

    def test_gain_ratio_capped_at_one(self):
        # Inertia larger than the reference must not amplify gains above base.
        kp, _, _ = _schedule_gains(C.IXX_FULL * 5, None, None, None)
        assert kp == pytest.approx(np.full(3, C.KP_ATTITUDE))

    def test_none_inertia_uses_reference_gains_directly(self):
        kp, _kd, _ki = _schedule_gains(None, None, None, None)
        assert kp == pytest.approx(np.full(3, C.KP_ATTITUDE))

    def test_explicit_gain_overrides_respected(self):
        kp, kd, ki = _schedule_gains(C.IXX_FULL, 1.0, 2.0, 3.0)
        assert kp == pytest.approx(np.ones(3))
        assert kd == pytest.approx(np.full(3, 2.0))
        assert ki == pytest.approx(np.full(3, 3.0))


class TestResolveAttitudeController:
    def test_booster_landing_auto_resolves_to_pid(self):
        config = create_test_config(recovery_attitude_controller="auto", attitude_controller="pd")
        result = resolve_attitude_controller(MissionPhase.BOOSTER_LANDING, "booster", config)
        assert result == "pid"

    def test_booster_landing_explicit_pd_respected(self):
        config = create_test_config(recovery_attitude_controller="pd", attitude_controller="pd")
        result = resolve_attitude_controller(MissionPhase.BOOSTER_LANDING, "booster", config)
        assert result == "pd"

    def test_s2_landing_with_recovery_enabled_resolves_to_pid(self):
        config = create_test_config(
            recovery_attitude_controller="auto", enable_s2_recovery=True, attitude_controller="pd"
        )
        result = resolve_attitude_controller(MissionPhase.S2_LANDING, "orbiter", config)
        assert result == "pid"

    def test_s2_landing_without_recovery_enabled_uses_default(self):
        config = create_test_config(
            recovery_attitude_controller="auto", enable_s2_recovery=False, attitude_controller="pd"
        )
        result = resolve_attitude_controller(MissionPhase.S2_LANDING, "orbiter", config)
        assert result == "pd"

    def test_non_landing_phase_uses_default_controller(self):
        config = create_test_config(attitude_controller="pd", recovery_attitude_controller="pid")
        result = resolve_attitude_controller(MissionPhase.ASCENT, "stacked", config)
        assert result == "pd"

    def test_booster_landing_but_orbiter_vehicle_model_uses_default(self):
        # Guards against the recovery controller leaking into the wrong vehicle model.
        config = create_test_config(attitude_controller="pd", recovery_attitude_controller="pid")
        result = resolve_attitude_controller(MissionPhase.BOOSTER_LANDING, "orbiter", config)
        assert result == "pd"


class TestPrepareControlState:
    def test_fresh_state_created_when_none(self):
        state = prepare_control_state(None, "pd", "ASCENT")
        assert isinstance(state, ControlState)

    def test_integral_reset_on_switch_to_pid(self):
        state = ControlState(integral_error=np.array([1.0, 2.0, 3.0]), active_controller="pd")
        state = prepare_control_state(state, "pid", "BOOSTER_LANDING")
        assert np.allclose(state.integral_error, 0.0)
        assert state.active_controller == "pid"

    def test_integral_reset_on_phase_change_while_pid(self):
        state = ControlState(
            integral_error=np.array([1.0, 2.0, 3.0]),
            active_controller="pid",
            last_phase="BOOSTER_ENTRY",
        )
        state = prepare_control_state(state, "pid", "BOOSTER_LANDING")
        assert np.allclose(state.integral_error, 0.0)

    def test_integral_preserved_when_same_phase_and_controller(self):
        state = ControlState(
            integral_error=np.array([1.0, 2.0, 3.0]),
            active_controller="pid",
            last_phase="BOOSTER_LANDING",
        )
        state = prepare_control_state(state, "pid", "BOOSTER_LANDING")
        assert np.allclose(state.integral_error, [1.0, 2.0, 3.0])

    def test_integral_not_reset_when_switching_to_pd(self):
        state = ControlState(
            integral_error=np.array([1.0, 2.0, 3.0]), active_controller="pid", last_phase="X"
        )
        state = prepare_control_state(state, "pd", "Y")
        assert np.allclose(state.integral_error, [1.0, 2.0, 3.0])


class TestPdControlLaw:
    def test_zero_error_and_omega_gives_zero_torque(self):
        torque, _ = pd_control_law(np.zeros(3), 0.0, np.zeros(3), inertia=C.IXX_FULL)
        assert np.allclose(torque, 0.0)

    def test_proportional_term_drives_positive_torque(self):
        torque, _ = pd_control_law(
            np.array([0.01, 0.0, 0.0]), 0.02, np.zeros(3), inertia=C.IXX_FULL
        )
        assert torque[0] > 0.0

    def test_torque_saturates_at_max_torque(self):
        torque, _ = pd_control_law(
            np.array([1.0, 0.0, 0.0]), np.pi, np.zeros(3), inertia=C.IXX_FULL, max_torque=1000.0
        )
        assert np.linalg.norm(torque) == pytest.approx(1000.0, rel=1e-6)

    def test_rcs_only_deadband_zeros_small_error(self):
        # max_torque < 50000 puts the controller in RCS-only mode with a
        # 3.5 deg deadband; a 1 deg error should be zeroed out.
        small_angle = np.radians(1.0)
        q_ev = np.array([np.sin(small_angle / 2), 0.0, 0.0])
        torque, _ = pd_control_law(
            q_ev, small_angle, np.zeros(3), inertia=1000.0, max_torque=10000.0
        )
        assert np.allclose(torque, 0.0)

    def test_rcs_only_deadband_does_not_suppress_larger_error(self):
        large_angle = np.radians(10.0)
        q_ev = np.array([np.sin(large_angle / 2), 0.0, 0.0])
        torque, _ = pd_control_law(
            q_ev, large_angle, np.zeros(3), inertia=1000.0, max_torque=10000.0
        )
        assert not np.allclose(torque, 0.0)

    def test_pid_integral_accumulates_over_steps(self):
        cs = create_control_state()
        cs.active_controller = "pid"
        q_ev = np.array([0.001, 0.0, 0.0])
        _, cs1 = pd_control_law(
            q_ev, 0.002, np.zeros(3), inertia=C.IXX_FULL, control_state=cs, dt=0.1, controller="pid"
        )
        integral_after_one = cs1.integral_error.copy()
        _, cs2 = pd_control_law(
            q_ev, 0.002, np.zeros(3), inertia=C.IXX_FULL, control_state=cs1, dt=0.1, controller="pid"
        )
        assert np.linalg.norm(cs2.integral_error) >= np.linalg.norm(integral_after_one)

    def test_pd_mode_never_integrates(self):
        cs = create_control_state()
        q_ev = np.array([0.001, 0.0, 0.0])
        _, cs_after = pd_control_law(
            q_ev, 0.002, np.zeros(3), inertia=C.IXX_FULL, control_state=cs, dt=0.1, controller="pd"
        )
        assert np.allclose(cs_after.integral_error, 0.0)

    def test_integral_freezes_when_pre_saturated(self):
        cs = create_control_state()
        cs.integral_error = np.array([1.0, 0.0, 0.0])
        q_ev = np.array([1.0, 0.0, 0.0])
        _, cs_after = pd_control_law(
            q_ev,
            np.pi,
            np.zeros(3),
            inertia=C.IXX_FULL,
            max_torque=1000.0,
            control_state=cs,
            dt=0.1,
            controller="pid",
        )
        # Torque is already saturating at max_torque before adding the
        # integral term, so the integral accumulator must not grow further.
        assert np.allclose(cs_after.integral_error, [1.0, 0.0, 0.0])


class TestIntegralAntiWindup:
    def test_transverse_clamped_to_windup_fraction_budget(self):
        integral = np.array([100.0, 0.0, 0.0])
        ki = np.array([1000.0, 1000.0, 1000.0])
        clamped = _apply_integral_anti_windup(integral, ki, max_torque=1000.0, windup_fraction=0.5)
        assert np.linalg.norm(ki[0:2] * clamped[0:2]) <= 500.0 + 1e-6

    def test_roll_clamped_independently_with_max_roll_torque(self):
        integral = np.array([0.0, 0.0, 100.0])
        ki = np.array([1000.0, 1000.0, 1000.0])
        clamped = _apply_integral_anti_windup(
            integral, ki, max_torque=100000.0, windup_fraction=0.5, max_roll_torque=200.0
        )
        assert abs(ki[2] * clamped[2]) <= 100.0 + 1e-6

    def test_zero_budget_zeros_integral(self):
        integral = np.array([5.0, 5.0, 5.0])
        ki = np.array([1.0, 1.0, 1.0])
        clamped = _apply_integral_anti_windup(integral, ki, max_torque=0.0, windup_fraction=0.5)
        assert np.allclose(clamped, 0.0)


class TestThrustAxisError:
    def test_aligned_direction_gives_zero_error(self):
        q = np.array([1.0, 0.0, 0.0, 0.0])
        q_ev, angle = compute_thrust_axis_error(q, np.array([0.0, 0.0, 1.0]))
        assert angle == pytest.approx(0.0, abs=1e-9)
        assert np.allclose(q_ev, 0.0)

    def test_perpendicular_direction_gives_90_degree_error(self):
        q = np.array([1.0, 0.0, 0.0, 0.0])
        _q_ev, angle = compute_thrust_axis_error(q, np.array([1.0, 0.0, 0.0]))
        assert angle == pytest.approx(np.pi / 2, rel=1e-6)

    def test_opposite_direction_gives_180_degree_error(self):
        q = np.array([1.0, 0.0, 0.0, 0.0])
        _q_ev, angle = compute_thrust_axis_error(q, np.array([0.0, 0.0, -1.0]))
        assert angle == pytest.approx(np.pi, rel=1e-6)

    def test_zero_desired_direction_returns_zero_error(self):
        q = np.array([1.0, 0.0, 0.0, 0.0])
        q_ev, angle = compute_thrust_axis_error(q, np.zeros(3))
        assert angle == 0.0
        assert np.allclose(q_ev, 0.0)

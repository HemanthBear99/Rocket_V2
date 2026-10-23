"""Final-approach position hold (guidance.booster._lateral_divert)."""

import numpy as np
import pytest

from rlv_sim.guidance.booster import _lateral_divert

R_ERR = np.array([40.0, 0.0, 0.0])
V_H = np.array([-5.0, 0.0, 0.0])
A_ZEV = np.array([3.0, 0.0, 0.0])


def _divert(h=100.0, t_go=10.0, budget=50.0, **hold):
    return _lateral_divert(
        V_H, 2.0, h, t_go, A_ZEV, np.zeros(3), budget,
        tau_min=1.5, terminal_horizontal_capture=False,
        terminal_inside_pad_capture=False, pad_reachable=True,
        r_err_horiz=R_ERR, **hold,
    )


def test_hold_disabled_is_velocity_nulling():
    a, active = _divert(position_hold_tau_min_s=0.0)
    assert not active
    assert np.allclose(a, A_ZEV)


def test_hold_is_critically_damped_law_above_fade():
    a, active = _divert(position_hold_tau_min_s=3.0, position_hold_floor_m=30.0)
    tau = 5.0  # max(0.5 * t_go, 3.0)
    assert active
    assert np.allclose(a, R_ERR / tau ** 2 - 2.0 * V_H / tau)


@pytest.mark.parametrize("h, weight", [(30.0, 0.0), (45.0, 0.5), (60.0, 1.0)])
def test_hold_fades_into_velocity_nulling(h, weight):
    a, _ = _divert(h=h, position_hold_tau_min_s=3.0, position_hold_floor_m=30.0)
    tau = 5.0
    a_hold = R_ERR / tau ** 2 - 2.0 * V_H / tau
    assert np.allclose(a, weight * a_hold + (1.0 - weight) * A_ZEV)


def test_vertical_priority_disables_hold():
    a, active = _divert(position_hold_tau_min_s=3.0, vertical_priority=True)
    assert not active
    assert np.allclose(a, A_ZEV)


def test_speed_cap_limits_closing_speed():
    # Zero cap: the hold may only brake the current horizontal velocity.
    a, _ = _divert(position_hold_tau_min_s=3.0, position_hold_speed_cap_mps=0.0)
    assert np.allclose(a, -2.0 * V_H / 5.0)


def test_budget_limits_hold():
    a, _ = _divert(budget=0.5, position_hold_tau_min_s=3.0)
    assert np.linalg.norm(a) == pytest.approx(0.5)

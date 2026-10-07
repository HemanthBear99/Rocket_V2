"""Regression coverage for gravity, configured mass limits, and mission reports."""

import importlib
import json
import math

import numpy as np
import pytest
from numpy.polynomial import Legendre

from rlv_sim import constants as C
from rlv_sim import forces
from rlv_sim._main_models import SimulationLog
from rlv_sim.campaign import _run_case
from rlv_sim.config_factory import create_test_config
from rlv_sim.mission_summary import (
    _assess_s2_initial_orbit,
    _extract_s2_recovery,
    assess_full_mission,
    write_mission_summary,
)

runner = importlib.import_module("rlv_sim._run_full_mission")


@pytest.mark.parametrize("latitude", [-90, -45, 0, 45, 90])
@pytest.mark.parametrize("altitude", [0, 400_000, 2_000_000])
def test_c20_gravity_matches_analytic_j2(monkeypatch, latitude, altitude):
    c20 = forces._EGM96_C[(2, 0)]
    monkeypatch.setattr(forces, "_EGM96_C", {(2, 0): c20})
    monkeypatch.setattr(forces, "_EGM96_S", {})
    phi = np.radians(latitude)
    r = (C.R_EARTH + altitude) * np.array([np.cos(phi), 0, np.sin(phi)])
    expected = forces.compute_gravity_force(r, 1, gravity_model="j2", j2=-c20 * np.sqrt(5))
    np.testing.assert_allclose(forces.compute_egm96_gravity_accel(r), expected, atol=1e-12)


def _reference_potential(r):
    """Independent polynomial definition, rather than the runtime recurrence."""
    radius = np.linalg.norm(r)
    sin_phi = r[2] / radius
    cos_phi = np.hypot(r[0], r[1]) / radius
    longitude = np.arctan2(r[1], r[0])
    total = 1.0
    for n in range(2, forces._EGM96_MAX_DEG + 1):
        for m in range(n + 1):
            normalization = np.sqrt((1 if m == 0 else 2) * (2 * n + 1)
                                    * math.factorial(n - m) / math.factorial(n + m))
            p = normalization * cos_phi ** m * Legendre.basis(n).deriv(m)(sin_phi)
            coefficient = (forces._EGM96_C.get((n, m), 0) * np.cos(m * longitude)
                           + forces._EGM96_S.get((n, m), 0) * np.sin(m * longitude))
            total += (C.R_EARTH_EQUATORIAL_WGS84 / radius) ** n * p * coefficient
    return C.MU_EARTH / radius * total


@pytest.mark.parametrize("direction", [[1, 2, 3], [-2, 1, -1], [0, 0, 1], [0, 0, -1]])
def test_full_gravity_matches_independent_potential_gradient(direction):
    direction = np.array(direction, dtype=float)
    r = (C.R_EARTH + 400_000) * direction / np.linalg.norm(direction)
    step = 10.0
    expected = np.array([
        (_reference_potential(r + offset) - _reference_potential(r - offset)) / (2 * step)
        for offset in np.eye(3) * step
    ])
    np.testing.assert_allclose(forces.compute_egm96_gravity_accel(r), expected, atol=2e-8, rtol=0)


@pytest.mark.parametrize("s2_recovery", [False, True])
@pytest.mark.parametrize("stop_immediately", [False, True])
def test_preseparation_mission_can_be_assessed_and_written(tmp_path, s2_recovery, stop_immediately):
    config = create_test_config(dt=0.1, max_time=0.2, enable_s2_recovery=s2_recovery)
    result = runner.run_full_mission(
        config=config, verbose=False,
        control_callback=(lambda: "stop") if stop_immediately else None,
    )
    assessment = assess_full_mission(result, config)
    assert not assessment.mission_success
    assert not assessment.landing.success
    assert "separation" in assessment.failed_criteria
    assert assessment.max_recovery_q_pa is None
    write_mission_summary(result, config, tmp_path)
    report = json.loads((tmp_path / "mission_summary.json").read_text())
    assert report["mission_success"] is False


def test_short_campaign_case_is_mission_failure_not_simulation_error():
    row = _run_case(create_test_config(max_time=0.2), {
        "case_id": "baseline", "mode": "sensitivity", "z_scores": {},
    })
    assert row["status"] == "mission_failed"
    assert "separation" in row["failed_criteria"]
    assert "error" not in row


@pytest.mark.parametrize("achieved_phase", ["ORBIT_ACHIEVED", "S2_ORBIT_HOLD", None])
def test_s2_report_uses_achieved_or_last_insertion_state(achieved_phase):
    config = create_test_config(enable_s2_recovery=True)
    result = runner.run_full_mission(config=config, verbose=False, control_callback=lambda: "stop")
    radius = C.R_EARTH + config.orbit_target_altitude_m
    speed = np.sqrt(C.MU_EARTH / radius)
    log = SimulationLog()
    log._data = {
        "time": [100., 300., 400.],
        "phase_name": ["ORBIT_INSERTION", achieved_phase or "ORBIT_INSERTION", "S2_DEORBIT"],
        "position_x": [radius] * 3, "position_y": [0.] * 3, "position_z": [0.] * 3,
        "velocity_x": [0.] * 3, "velocity_y": [6000., speed if achieved_phase else 6500., 5000.],
        "velocity_z": [0.] * 3, "mass": [20000.] * 3,
    }
    result.orbiter_log = log
    assessment = _assess_s2_initial_orbit(result, config)
    recovery = _extract_s2_recovery(result, config)
    assert assessment.success is (achieved_phase is not None)
    assert recovery["orbit_achieved"] is assessment.success
    assert recovery["achieved_perigee_km"] == round(assessment.perigee_altitude_km, 1)
    if achieved_phase:
        assert assessment.perigee_altitude_km == pytest.approx(config.orbit_target_altitude_m / 1000)
    else:
        assert "ORBIT ACHIEVED" not in assessment.reason
        assert assessment.perigee_altitude_km > -3000  # last, not first, insertion sample


def test_separated_vehicles_use_configured_mass_limits(monkeypatch):
    config = create_test_config(
        dt=0.05, max_time=6.0, stage1_dry_mass=10000., stage2_dry_mass=2000.,
        payload_mass=3000., stage1_prop_mass=5000., stage1_landing_fuel_reserve_kg=5000.,
        gravity_model="central", enable_j2=False, enable_drag=False, enable_lift=False,
    )
    # Start at MECO altitude with only the reserved first-stage propellant.
    # The real phase manager, separation, and dual-vehicle integrator still run.
    initial = runner._create_configured_initial_state(config)
    initial.r = np.array([C.R_EARTH + 200000., 0., 0.])
    initial.v = np.array([0., 3000., 0.])
    monkeypatch.setattr(runner, "_create_configured_initial_state", lambda cfg: initial.copy())
    result = runner.run_full_mission(config=config, verbose=False)
    assert result.separation_time is not None
    for log, state, dry_mass in (
        (result.orbiter_log, result.orbiter_final_state, 5000.),
        (result.booster_log, result.booster_final_state, 10000.),
    ):
        assert log.get_series("time")
        assert state.m >= dry_mass
        assert log.propellant_remaining_kg[-1] == pytest.approx(log.mass[-1] - dry_mass)
    assert "Validation failure" not in result.booster_reason

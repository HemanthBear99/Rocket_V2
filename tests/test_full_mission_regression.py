"""End-to-end regression test for the nominal full mission.

Runs the research-profile mission (ascent -> separation -> orbit insertion +
booster return-to-launch-site landing) and pins the key outcomes. Any physics,
guidance or integration change that shifts these numbers fails here instead of
silently degrading the mission.

The run takes ~35 s, so it is marked ``slow`` and excluded from the default
test run. Run it with::

    pytest -m slow

Reference values were recorded from the nominal mission on 2026-10-07
(orbit_coast_max_dt = 0.5 s, q-hold throttle at 32 kPa, entry ZEV gain 0.5, vehicle-derived thrust). When a
change *intentionally* alters the trajectory, re-run the mission, confirm the
new outcome is correct, and update the references below.
"""

import pytest

from rlv_sim._run_full_mission import run_full_mission
from rlv_sim.config_factory import create_default_config
from rlv_sim.mission_summary import assess_full_mission

pytestmark = pytest.mark.slow

REFERENCE = {
    "separation_time_s": 129.55,
    "perigee_km": 326.73,
    "apogee_km": 435.01,
    "eccentricity": 0.00802,
    "touchdown_speed_mps": 4.41,
    "site_error_m": 0.31,
}


@pytest.fixture(scope="module")
def nominal_mission():
    config = create_default_config()
    result = run_full_mission(config=config, verbose=False)
    return result, assess_full_mission(result, config)


def test_nominal_mission_succeeds(nominal_mission):
    result, assessment = nominal_mission
    assert assessment.mission_success, assessment.failed_criteria
    assert result.orbiter_success
    assert result.booster_landing_success
    assert assessment.landing.status == "landing_success"


def test_separation_time_unchanged(nominal_mission):
    result, _ = nominal_mission
    assert result.separation_time == pytest.approx(REFERENCE["separation_time_s"], abs=0.1)


def test_orbit_elements_unchanged(nominal_mission):
    _, assessment = nominal_mission
    orbit = assessment.orbit
    assert orbit.perigee_altitude_km == pytest.approx(REFERENCE["perigee_km"], abs=2.0)
    assert orbit.apogee_altitude_km == pytest.approx(REFERENCE["apogee_km"], abs=2.0)
    assert orbit.eccentricity == pytest.approx(REFERENCE["eccentricity"], abs=5e-4)


def test_booster_landing_unchanged(nominal_mission):
    result, assessment = nominal_mission
    # Touchdown metrics are carried as numbers, not parsed from reason text.
    assert result.booster_touchdown_speed_mps == pytest.approx(assessment.landing.touchdown_speed_mps)
    assert result.booster_site_error_m == pytest.approx(assessment.landing.site_error_m)
    landing = assessment.landing
    assert landing.touchdown_speed_mps == pytest.approx(REFERENCE["touchdown_speed_mps"], abs=0.25)
    assert landing.site_error_m == pytest.approx(REFERENCE["site_error_m"], abs=5.0)

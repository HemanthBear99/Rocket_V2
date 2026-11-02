"""One authority for the landing outcome: flags, reasons and assessment agree."""

from dataclasses import replace

import pytest

from rlv_sim.config.factory import create_test_config
from rlv_sim.mission.step import _booster_landing_success
from rlv_sim.outputs.summary import classify_landing
from rlv_sim.vehicle.recovery_hardware import hard_landing_speed_limit


def test_low_propellant_margin_is_a_failure_not_a_success():
    cfg = create_test_config()
    landing = classify_landing(3.0, 5.0, cfg, propellant_remaining_kg=10.0)
    assert not landing.success
    assert "propellant_margin" in landing.failed_criteria
    assert not landing.reason.startswith("LANDING SUCCESS")


def test_soft_pad_landing_with_margin_succeeds():
    cfg = create_test_config()
    landing = classify_landing(3.0, 5.0, cfg, propellant_remaining_kg=5000.0)
    assert landing.success and landing.reason.startswith("LANDING SUCCESS")


@pytest.mark.parametrize("enforce, expected", [(True, False), (False, True)])
def test_pad_enforcement_decides_offsite_soft_landing(enforce, expected):
    cfg = create_test_config(booster_enforce_pad_landing=enforce)
    landing = classify_landing(3.0, 400.0, cfg, propellant_remaining_kg=5000.0)
    assert landing.success is expected


def test_one_hard_landing_band():
    cfg = create_test_config()
    assert hard_landing_speed_limit(cfg) == 10.0
    # A 10 m/s success limit must still leave a hard band above it.
    assert hard_landing_speed_limit(replace(cfg, landing_leg_max_touchdown_speed_mps=10.0)) == 20.0
    assert classify_landing(11.0, 5.0, cfg).status == "crash_landing"


class _Mgr:
    touchdown_success = True


def test_success_flag_reads_the_recorded_outcome():
    assert _booster_landing_success(_Mgr()) is True
    assert _booster_landing_success(object()) is False

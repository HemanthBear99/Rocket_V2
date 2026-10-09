"""Vehicle builder: thrust levels and reference area derive from the vehicle."""

import asyncio
import math

import pytest

from rlv_sim import constants as C
from rlv_sim.api_models import SimulationSetup
from rlv_sim.config_factory import create_default_config


def test_burn_thrust_derives_from_engine_cluster():
    cfg = create_default_config(stage1_engine_count=7, stage1_engine_thrust_n=1.0e6,
                                boostback_engine_count=2, entry_engine_count=2,
                                landing_engine_count=1)
    assert cfg.stage1_thrust_n == pytest.approx(7.0e6)
    assert cfg.boostback_thrust_n == pytest.approx(2.0e6)
    assert cfg.entry_thrust_n == pytest.approx(2.0e6)
    assert cfg.landing_thrust_n == pytest.approx(1.0e6)


def test_reference_area_derives_from_diameter():
    cfg = create_default_config(vehicle_diameter_m=4.0)
    assert cfg.reference_area_m2 == pytest.approx(math.pi * 4.0)


def test_default_vehicle_matches_reference_constants():
    cfg = create_default_config()
    assert cfg.stage1_thrust_n == pytest.approx(C.THRUST_MAGNITUDE)
    assert cfg.landing_thrust_n == pytest.approx(C.LANDING_THRUST)
    assert cfg.entry_thrust_n == pytest.approx(C.ENTRY_THRUST)


@pytest.mark.parametrize("override", [
    {"stage1_engine_count": 0},
    {"landing_engine_count": 2.5},
    {"entry_engine_count": 12},          # more than the 9 stage-1 engines
    {"vehicle_diameter_m": 0.0},
    {"stage1_engine_thrust_n": -1.0},
])
def test_invalid_vehicle_rejected(override):
    with pytest.raises(ValueError):
        create_default_config(**override)


def test_ui_vehicle_fields_round_trip_to_config():
    from rlv_sim.server import frontend_config_defaults, map_config

    defaults = asyncio.run(frontend_config_defaults())
    vehicle = {**defaults["vehicle"], "stage1_engines": 6, "thrust_sl": 6.0e6, "diameter_m": 4.5}
    recovery = {k: v for k, v in defaults["recovery"].items() if k != "gfold_available"}
    setup = SimulationSetup(vehicle=vehicle, mission=defaults["mission"],
                            recovery=recovery, physics=defaults["physics"])
    cfg = map_config(setup)
    assert cfg.stage1_engine_count == 6
    assert cfg.stage1_engine_thrust_n == pytest.approx(1.0e6)
    assert cfg.stage1_thrust_n == pytest.approx(6.0e6)
    assert cfg.vehicle_diameter_m == pytest.approx(4.5)

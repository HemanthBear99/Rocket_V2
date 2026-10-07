"""Unit tests for rlv_sim.config_definition / config_factory.

Covers SimulationConfig.validate() boundary behavior and the confirmed
divergence between create_test_config and create_default_config (the
former does NOT inherit the "research profile" overrides -- flagged in the
verified test architecture report, section 24/27).
"""

import dataclasses

import pytest

from rlv_sim.config_definition import SimulationConfig
from rlv_sim.config_factory import (
    create_default_config,
    create_demo_config,
    create_test_config,
)


class TestValidateAcceptsGoodConfig:
    def test_default_config_validates(self):
        config = create_default_config()
        config.validate()  # should not raise

    def test_test_config_validates(self):
        config = create_test_config()
        config.validate()

    def test_demo_config_validates(self):
        config = create_demo_config()
        config.validate()


class TestValidateRejectsBadConfig:
    def test_negative_dt_raises(self):
        with pytest.raises(ValueError, match="dt must be positive"):
            create_test_config(dt=-0.1)

    def test_zero_max_time_raises(self):
        with pytest.raises(ValueError, match="max_time must be positive"):
            create_test_config(max_time=0.0)

    def test_negative_stage1_dry_mass_raises(self):
        with pytest.raises(ValueError, match="stage1_dry_mass must be positive"):
            create_test_config(stage1_dry_mass=-100.0)

    def test_bad_attitude_controller_literal_raises(self):
        with pytest.raises(ValueError, match="attitude_controller"):
            create_test_config(attitude_controller="bang-bang")

    def test_negative_kp_attitude_raises(self):
        with pytest.raises(ValueError, match="kp_attitude"):
            create_test_config(kp_attitude=-1.0)

    def test_out_of_range_launch_latitude_raises(self):
        with pytest.raises(ValueError, match="launch_site_lat_deg"):
            create_test_config(launch_site_lat_deg=200.0)

    def test_multiple_errors_all_reported(self):
        with pytest.raises(ValueError) as excinfo:
            create_test_config(dt=-1.0, max_time=-1.0)
        message = str(excinfo.value)
        assert "dt must be positive" in message
        assert "max_time must be positive" in message


class TestCreateTestConfigDoesNotInheritResearchProfile:
    """Confirmed divergence: create_test_config builds a bare SimulationConfig
    rather than layering on create_default_config's research-profile
    overrides. Anyone asserting research-profile values (e.g.
    stage1_landing_fuel_reserve_kg=67500.0) against a create_test_config
    instance will get the raw dataclass default instead.
    """

    def test_landing_fuel_reserve_differs_from_default_profile(self):
        default_cfg = create_default_config()
        test_cfg = create_test_config()
        bare_cfg = SimulationConfig(dt=0.1, max_time=10.0, verbose=False)

        assert default_cfg.stage1_landing_fuel_reserve_kg == 67500.0
        assert test_cfg.stage1_landing_fuel_reserve_kg == bare_cfg.stage1_landing_fuel_reserve_kg
        assert test_cfg.stage1_landing_fuel_reserve_kg != default_cfg.stage1_landing_fuel_reserve_kg


class TestConfigRoundTripOverrides:
    def test_override_applied_and_validated(self):
        config = create_test_config(dt=0.02, max_time=5.0)
        assert config.dt == 0.02
        assert config.max_time == 5.0

    def test_frozen_dataclass_rejects_attribute_mutation(self):
        config = create_test_config()
        with pytest.raises(dataclasses.FrozenInstanceError):
            config.dt = 0.5

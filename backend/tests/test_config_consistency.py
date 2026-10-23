"""Guards against settings that do nothing and defaults that drift apart."""

import dataclasses
import pathlib
import re

from rlv_sim.config.definition import SimulationConfig
from rlv_sim.config.factory import create_default_config
from rlv_sim.config.field_docs import FIELD_DOCS

_PACKAGE = pathlib.Path(__file__).resolve().parents[1] / "rlv_sim"

# Read only inside config/ through a derived property or the factory.
CONSUMED_IN_CONFIG = {
    "auto_size_recovery_reserve",          # factory._with_overrides
    "booster_landing_planning_throttle",   # SimulationConfig.landing_planning_thrust_n
    "stage1_engine_thrust_n",              # SimulationConfig.stage1_thrust_n
}

# Known dead settings still being wired or retired (root-cause plan, RC4).
# Remove an entry when its field is wired or retired; the test fails if a
# listed field becomes read (stale list) or a new dead field appears.
PENDING_WIRING = {
    "max_gimbal_angle_deg",
    "stage_sep_velocity",
    "target_inclination_deg",
    "atmosphere_model",
}

# Factory values that differ from the dataclass defaults (RC5 removes them).
PROFILE_OVERRIDES = {
    "stage1_landing_fuel_reserve_kg",
    "recovery_reserve_wind_sensitivity_kg_per_mps",
    "booster_entry_budget_kg",
    "booster_landing_reserve_kg",
    "booster_entry_burn_min_altitude_m",
    "booster_landing_ignition_safety_factor",
    "booster_landing_ignition_ceiling_m",
    "booster_landing_propellant_margin_kg",
    "booster_landing_vertical_propellant_safety_factor",
    "booster_powered_descent_lead_time_s",
    "booster_max_divert_accel_near_mps2",
    "booster_divert_fraction_high",
    "booster_divert_fraction_mid",
    "booster_divert_fraction_low",
    "booster_divert_band_high_alt_m",
}


def _unread_fields() -> set[str]:
    src = "\n".join(
        p.read_text(encoding="utf-8")
        for p in _PACKAGE.rglob("*.py")
        if not {"config", "server"} & set(p.relative_to(_PACKAGE).parts)
    )
    return {
        f.name for f in dataclasses.fields(SimulationConfig)
        if not re.search(r"(\.|['\"])" + re.escape(f.name) + r"\b", src)
    }


def test_every_setting_is_read_by_the_simulator():
    assert _unread_fields() - CONSUMED_IN_CONFIG == PENDING_WIRING


def test_factory_defaults_match_dataclass_defaults():
    profile = create_default_config()
    differing = {
        f.name for f in dataclasses.fields(SimulationConfig)
        if f.default is not dataclasses.MISSING and f.default != getattr(profile, f.name)
    }
    assert differing == PROFILE_OVERRIDES


def test_every_setting_has_help_text():
    names = {f.name for f in dataclasses.fields(SimulationConfig)} - {"verbose", "aero_deck_path"}
    assert names <= set(FIELD_DOCS)

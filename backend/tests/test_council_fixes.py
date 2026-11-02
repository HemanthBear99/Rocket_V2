"""Regression tests for the council-approved physics and safety fixes."""

import numpy as np
import pytest

from rlv_sim.aero import deck_import as aero_import
from rlv_sim.config.factory import create_default_config, create_test_config
from rlv_sim.core import constants as C
from rlv_sim.guidance import plugins
from rlv_sim.mission.manager_helpers import compute_orbit_metrics
from rlv_sim.outputs.summary import assess_orbit
from rlv_sim.physics.mass import (
    PAYLOAD_CG_Z,
    S2_DRY_CG_Z,
    S2_PROP_CG_Z,
    compute_center_of_mass,
    compute_mass_flow_rate,
)

CSV = "mach,alpha_deg,CD\n" + "\n".join(
    f"{m},{a},0.3" for m in (0.2, 2.0) for a in (-10, 0, 10)
)


def test_stage1_flow_follows_configured_thrust():
    """Same Isp at any configured thrust: more thrust costs more propellant."""
    base = compute_mass_flow_rate(stage=1, stage1_thrust_n=C.THRUST_MAGNITUDE)
    doubled = compute_mass_flow_rate(stage=1, stage1_thrust_n=2.0 * C.THRUST_MAGNITUDE)
    assert base == pytest.approx(-C.MASS_FLOW_RATE)
    assert doubled == pytest.approx(2.0 * base)


def test_mass_dispersion_is_structure_not_propellant():
    nominal = create_test_config()
    heavy = create_test_config(runtime_initial_mass_offset_kg=5000.0)
    # The MECO floor rises with the offset, so ascent propellant is unchanged.
    assert heavy.meco_mass_kg - nominal.meco_mass_kg == pytest.approx(5000.0)


def test_orbiter_cg_continues_the_stack_stations():
    cfg = create_test_config()
    full = cfg.stage2_dry_mass + cfg.stage2_prop_mass + cfg.payload_mass
    empty = cfg.stage2_dry_mass + cfg.payload_mass
    expected_full = (cfg.stage2_dry_mass * S2_DRY_CG_Z + cfg.stage2_prop_mass * S2_PROP_CG_Z
                     + cfg.payload_mass * PAYLOAD_CG_Z) / full
    assert compute_center_of_mass(full, "orbiter", config=cfg) == pytest.approx(expected_full)
    # Burning the low-mounted propellant raises the CG, not lowers it.
    assert compute_center_of_mass(empty, "orbiter", config=cfg) > expected_full


@pytest.mark.parametrize("field", ["launch_site_altitude_m", "booster_landing_site_altitude_m"])
def test_raised_sites_are_rejected(field):
    with pytest.raises(ValueError, match=field):
        create_test_config(**{field: 50.0})


def test_aborted_run_is_not_reported_as_orbit():
    cfg = create_default_config(verbose=False)
    r_mag = C.R_EARTH + cfg.orbit_target_altitude_m
    from rlv_sim.core.state import State

    state = State(r=np.array([r_mag, 0.0, 0.0]), v=np.array([0.0, np.sqrt(C.MU_EARTH / r_mag), 0.0]),
                  q=np.array([1.0, 0.0, 0.0, 0.0]), omega=np.zeros(3), m=5000.0, t=600.0,
                  sim_config=cfg, dry_mass_kg=cfg.stage2_dry_mass + cfg.payload_mass)
    assert compute_orbit_metrics(state)["energy"] < 0.0
    assert assess_orbit(state, cfg, "ORBIT ACHIEVED").success
    aborted = assess_orbit(state, cfg, "ABORT - TUMBLE: body rate")
    assert not aborted.success


def test_listing_plugins_does_not_run_them(tmp_path, monkeypatch):
    monkeypatch.setenv("RLV_PLUGIN_DIR", str(tmp_path))
    (tmp_path / "sneaky.py").write_text(
        "NAME = 'Sneaky'\nraise SystemExit('executed while listing')\n"
        "def landing_command(state):\n    return [0, 0, 0]\n"
    )
    rows = {p["name"]: p for p in plugins.list_plugins()}
    assert rows["sneaky"]["title"] == "Sneaky"
    assert rows["sneaky"]["error"] is None


def test_deck_axes_must_be_numeric():
    deck = aero_import.parse_csv(CSV)
    deck["axes"]["mach"] = ["a", "b"]
    assert any("only numbers" in e for e in aero_import.validate_deck(deck))


def test_deck_import_refuses_silent_overwrite(tmp_path, monkeypatch):
    monkeypatch.setattr(aero_import, "user_aero_dir", lambda: tmp_path)
    aero_import.import_deck(CSV.encode(), "t.csv", name="same")
    with pytest.raises(FileExistsError):
        aero_import.import_deck(CSV.encode(), "t.csv", name="same")
    aero_import.import_deck(CSV.encode(), "t.csv", name="same", overwrite=True)

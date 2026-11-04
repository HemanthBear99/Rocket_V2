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


def test_mass_dispersion_is_propellant_load():
    nominal = create_test_config()
    heavy = create_test_config(runtime_initial_mass_offset_kg=5000.0)
    # A loading error changes the burn, not the MECO mass or the booster's reserve.
    assert heavy.meco_mass_kg == pytest.approx(nominal.meco_mass_kg)


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


# ---------------------------------------------------------------- second council round
from rlv_sim.config.factory import config_to_dict
from rlv_sim.core.state import State
from rlv_sim.guidance.common import create_guidance_state
from rlv_sim.guidance.orbit import compute_deorbit_guidance
from rlv_sim.mission import campaign, step
from rlv_sim.mission.manager import MissionManager


def _state(cfg, alt_m, v, m):
    return State(r=np.array([C.R_EARTH + alt_m, 0.0, 0.0]), v=np.asarray(v, float),
                 q=np.array([1.0, 0.0, 0.0, 0.0]), omega=np.zeros(3), m=m, t=500.0, sim_config=cfg)


def test_landing_ignition_plans_with_dispersed_thrust():
    """Weaker engines must light the landing burn higher."""
    def corridor_top(scale):
        cfg = create_default_config(verbose=False, runtime_thrust_scale=scale)
        mgr = MissionManager(vehicle_type="booster", initial_mass=30_000.0, config=cfg)
        # Falling at 250 m/s, co-rotating with the ground.
        st = _state(cfg, 3000.0, [-250.0, C.EARTH_ROTATION_RATE * (C.R_EARTH + 3000.0), 0.0], 30_000.0)
        return mgr._ignition_corridor_top(st)
    assert corridor_top(0.9) > corridor_top(1.0)


def test_deorbit_burn_measures_dv_with_configured_isp():
    def thrust_on(isp):
        cfg = create_default_config(verbose=False, stage2_isp_vac=isp)
        r_mag = C.R_EARTH + 400_000.0
        gs = create_guidance_state()
        gs.deorbit_start_mass = 20_000.0
        # Burned enough for 310 m/s at 348 s Isp: past the 300 m/s cap only at that Isp.
        m = 20_000.0 / np.exp(310.0 / (348.0 * C.G0))
        out, _ = compute_deorbit_guidance(np.array([r_mag, 0.0, 0.0]),
                                          np.array([0.0, np.sqrt(C.MU_EARTH / r_mag), 0.0]),
                                          600.0, m, gs=gs, config=cfg)
        return out["thrust_on"]
    assert thrust_on(348.0) is False
    assert thrust_on(300.0) is True


def test_mc_mass_scatter_scales_with_configured_liftoff_mass():
    base = create_default_config(verbose=False, payload_mass=20_000.0)
    case = {"case_id": "x", "mode": "monte-carlo", "z_scores": {"mass": 1.0}}
    liftoff = (base.stage1_dry_mass + base.stage1_prop_mass + base.stage2_dry_mass
               + base.stage2_prop_mass + base.payload_mass)
    applied = campaign.apply_case(base, case)
    assert applied.runtime_initial_mass_offset_kg == pytest.approx(liftoff * campaign._MC_MASS_DISPERSION)


def test_exported_config_reloads_without_double_wind_reserve():
    cfg = create_default_config(verbose=False, runtime_wind_offset_mps=-5.0)
    again = create_default_config(**config_to_dict(cfg))
    assert again.stage1_landing_fuel_reserve_kg == pytest.approx(cfg.stage1_landing_fuel_reserve_kg)


def test_guidance_plans_on_forecast_wind_not_turbulence(monkeypatch):
    seen = {}
    real = step._run_guidance

    def spy(state, gs, mgr, config, dt):
        seen["wind"] = config.runtime_wind_offset_mps
        return real(state, gs, mgr, config, dt)

    monkeypatch.setattr(step, "_run_guidance", spy)
    forecast = create_test_config()
    gusty = create_test_config(runtime_wind_offset_mps=12.0)
    from rlv_sim.core.state import create_initial_state
    from rlv_sim.guidance.common import compute_local_vertical
    from rlv_sim.vehicle.actuator import ActuatorState

    st = create_initial_state(gusty)
    mgr = MissionManager(vehicle_type="ascent", initial_mass=st.m, config=gusty)
    step.simulation_step(st, ActuatorState(thrust_dir=compute_local_vertical(st.r)), mgr, 0.05,
                         dry_mass=gusty.meco_mass_kg, config=gusty, guidance_config=forecast)
    assert seen["wind"] == forecast.runtime_wind_offset_mps


def test_stage1_isp_is_configurable():
    base = compute_mass_flow_rate(stage=1, stage1_thrust_n=C.THRUST_MAGNITUDE)
    half_isp = compute_mass_flow_rate(stage=1, stage1_thrust_n=C.THRUST_MAGNITUDE,
                                      stage1_isp_sl=C.ISP / 2.0)
    assert half_isp == pytest.approx(2.0 * base)
    from rlv_sim.mission.planner import plan_mission

    nominal = plan_mission(create_default_config(verbose=False))
    better = plan_mission(create_default_config(verbose=False, stage1_isp_sl=C.ISP * 1.1,
                                                stage1_isp_vac=C.ISP_VAC * 1.1))
    assert better != nominal


def test_throttle_deadzone_cuts_off_propellant_flow():
    """Throttle below 1% must produce zero mass flow, consistent with zero thrust."""
    assert compute_mass_flow_rate(thrust_on=True, throttle=0.005, stage=1) == 0.0
    assert compute_mass_flow_rate(thrust_on=True, throttle=0.0, stage=1) == 0.0
    assert compute_mass_flow_rate(thrust_on=True, throttle=0.02, stage=1) < 0.0


def test_stage2_recovery_torque_authority():
    """Stage-2 recovery attitude torque must not exceed Stage-2 max torque."""
    cfg = create_test_config()
    s2_recovery_t = step._stage_max_torque(cfg, stage=2, vehicle_model="stage2", s2_recovery_attitude=True)
    s2_nominal_t = step._stage_max_torque(cfg, stage=2, vehicle_model="stage2", s2_recovery_attitude=False)
    booster_t = step._stage_max_torque(cfg, stage=1, vehicle_model="booster", s2_recovery_attitude=False)
    assert s2_recovery_t == pytest.approx(s2_nominal_t)
    assert s2_recovery_t < booster_t


def test_campaign_run_case_catches_invalid_case():
    """_run_case must catch configuration errors gracefully as an error status."""
    cfg = create_test_config()
    bad_case = {"case_id": "bad_001", "mode": "monte-carlo", "z_scores": {"thrust": "not-a-number"}}
    row = campaign._run_case(cfg, bad_case)
    assert row["status"] == "error"
    assert row["case_id"] == "bad_001"
    assert "ValueError" in row["error"]


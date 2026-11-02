"""Settings schema, guidance plugins, wind-mode overrides, configurable mass
properties, campaign case building and the model card."""

import dataclasses
import textwrap

import numpy as np
import pytest

from rlv_sim.config.definition import SimulationConfig
from rlv_sim.config.factory import create_default_config, create_test_config
from rlv_sim.config.field_docs import FIELD_DOCS
from rlv_sim.config.schema import config_schema
from rlv_sim.guidance import plugins
from rlv_sim.mission import campaign
from rlv_sim.physics import mass
from rlv_sim.physics.dynamics import DynamicsContext, compute_jet_damping_torque


# ------------------------------------------------------------------ schema
def test_schema_covers_every_user_facing_field():
    names = {row["name"] for row in config_schema()}
    fields = {f.name for f in dataclasses.fields(SimulationConfig)}
    assert names == fields - {"verbose", "aero_deck_path"}
    assert set(FIELD_DOCS) == names
    assert all(row["description"] for row in config_schema())


def test_schema_bounds_match_validation():
    rows = {row["name"]: row for row in config_schema()}
    assert rows["max_gimbal_angle_deg"]["max"] == 30.0
    assert rows["stage1_nozzle_exit_radius_m"]["min_exclusive"] is True
    assert rows["booster_terminal_position_hold_tau_s"]["min"] == 0.0
    assert rows["booster_landing_guidance"]["choices"] == ["heuristic", "gfold", "plugin"]
    assert rows["runtime_wind_profile_modes"]["kind"] == "wind_modes"
    # Every bounded default sits inside its own bounds.
    for row in rows.values():
        if "min" in row and isinstance(row["default"], (int, float)) and not isinstance(row["default"], bool):
            assert row["default"] >= row["min"], row["name"]


# ------------------------------------------------------------------ plugins
@pytest.fixture
def plugin_dir(tmp_path, monkeypatch):
    monkeypatch.setenv("RLV_PLUGIN_DIR", str(tmp_path))
    plugins._loaded.clear()
    yield tmp_path
    plugins._loaded.clear()


def _state(**overrides):
    base = {
        "t": 500.0, "altitude_m": 100.0, "up": np.array([1.0, 0.0, 0.0]),
        "position_error_m": np.zeros(3), "velocity_mps": np.array([-20.0, 0.0, 0.0]),
        "vertical_speed_mps": -20.0, "horizontal_velocity_mps": np.zeros(3), "mass_kg": 30000.0,
        "propellant_kg": 5000.0, "max_accel_mps2": 30.0, "gravity_mps2": 9.8, "time_to_go_s": 8.0,
        "tilt_limit_deg": 15.0, "target_descent_rate_mps": 3.0,
    }
    base.update(overrides)
    return plugins.LandingState(**base)


def test_bundled_example_is_listed_and_valid(plugin_dir):
    names = {p["name"]: p for p in plugins.list_plugins()}
    assert names["example_zem_zev"]["source"] == "bundled"
    assert names["example_zem_zev"]["error"] is None
    direction, throttle = plugins.landing_command("example_zem_zev", _state())
    assert direction == pytest.approx([1.0, 0.0, 0.0])
    assert 0.0 < throttle <= 1.0


def test_user_plugin_command_becomes_direction_and_throttle(plugin_dir):
    (plugin_dir / "half_g.py").write_text(textwrap.dedent("""
        NAME = "Half"
        def landing_command(state):
            return [0.0, 15.0, 0.0]
    """))
    direction, throttle = plugins.landing_command("half_g", _state())
    assert direction == pytest.approx([0.0, 1.0, 0.0])
    assert throttle == pytest.approx(0.5)
    assert plugins.list_plugins()[0]["title"] == "Half"


@pytest.mark.parametrize("body, message", [
    ("def landing_command(state):\n    raise ValueError('boom')\n", "raised: boom"),
    ("def landing_command(state):\n    return [1.0, 2.0]\n", "must return 3 numbers"),
    ("def landing_command(state):\n    return [float('nan'), 0, 0]\n", "non-finite"),
])
def test_plugin_failures_are_reported(plugin_dir, body, message):
    (plugin_dir / "bad.py").write_text(body)
    with pytest.raises(RuntimeError, match=message):
        plugins.landing_command("bad", _state())


def test_plugin_names_are_validated(plugin_dir):
    assert "must name a plugin" in plugins.plugin_error("../evil")
    assert "not found" in plugins.plugin_error("missing")
    # Validation must not execute plugin code; a broken plugin fails at mission start.
    (plugin_dir / "nofunc.py").write_text("raise SystemExit('executed during validation')\n")
    assert plugins.plugin_error("nofunc") is None
    cfg = create_test_config(booster_landing_guidance="plugin", booster_landing_plugin="nofunc")
    with pytest.raises(SystemExit, match="executed during validation"):
        plugins.require_loadable(cfg)
    assert plugins.plugin_error("example_zem_zev") is None
    with pytest.raises(ValueError, match="booster_landing_plugin"):
        create_test_config(booster_landing_guidance="plugin", booster_landing_plugin="missing")
    cfg = create_test_config(booster_landing_guidance="plugin", booster_landing_plugin="example_zem_zev")
    assert cfg.booster_landing_plugin == "example_zem_zev"


# ------------------------------------------------------------------ wind modes
def test_wind_modes_from_json_lists_become_tuples():
    cfg = create_test_config(runtime_wind_profile_modes=[[2.0, 5000.0, 0.5]])
    assert cfg.runtime_wind_profile_modes == ((2.0, 5000.0, 0.5),)


@pytest.mark.parametrize("modes", [[[2.0, -1.0, 0.0]], [[2.0, 5000.0]], [[float("inf"), 5000.0, 0.0]]])
def test_bad_wind_modes_are_rejected(modes):
    with pytest.raises(ValueError, match="runtime_wind_profile_modes"):
        create_test_config(runtime_wind_profile_modes=modes)


def test_scales_must_be_positive():
    with pytest.raises(ValueError, match="booster_descent_drag_scale"):
        create_test_config(booster_descent_drag_scale=-1.0)


# ------------------------------------------------------------------ mass properties
def test_payload_free_reference_vehicle_matches_constants():
    # Same stage masses as core.constants and no payload: config and
    # constants paths must agree exactly.
    cfg = create_default_config(payload_mass=0.0)
    for model in ("stacked", "booster"):
        for m in (400_000.0, 30_000.0):
            reserve = cfg.stage1_landing_fuel_reserve_kg
            assert np.array_equal(
                mass.compute_inertia_tensor(m, model, reserve, config=cfg),
                mass.compute_inertia_tensor(m, model, reserve),
            )
    reserve = cfg.stage1_landing_fuel_reserve_kg
    assert mass.compute_center_of_mass(30_000.0, "booster", reserve, config=cfg) == mass.compute_center_of_mass(
        30_000.0, "booster", reserve)


def test_payload_is_not_counted_as_stage1_propellant():
    cfg = create_default_config(payload_mass=8000.0)
    liftoff = (cfg.stage1_dry_mass + cfg.stage1_prop_mass + cfg.stage2_dry_mass
               + cfg.stage2_prop_mass + cfg.payload_mass)
    _, s1_prop, _, _, payload = mass._stacked_component_masses(liftoff, cfg)
    assert s1_prop == pytest.approx(cfg.stage1_prop_mass)
    assert payload == 8000.0
    # Payload raises the stack CG above the payload-free vehicle.
    no_payload = create_default_config(payload_mass=0.0)
    assert mass.compute_center_of_mass(liftoff, config=cfg) > mass.compute_center_of_mass(
        liftoff - 8000.0, config=no_payload)


def test_gimbal_limit_setting_sets_tvc_authority():
    from rlv_sim.mission.step import _max_gimbal_rad

    assert _max_gimbal_rad(create_test_config(max_gimbal_angle_deg=6.0)) == pytest.approx(np.radians(6.0))


def test_mass_properties_follow_config():
    cfg = create_test_config(booster_ixx_empty_kgm2=1.0e6, booster_cg_empty_m=30.0)
    dry = cfg.stage1_dry_mass
    tensor = mass.compute_inertia_tensor(dry, "booster", config=cfg)
    assert tensor[0, 0] == pytest.approx(1.0e6)
    assert mass.compute_center_of_mass(dry, "booster", config=cfg) == pytest.approx(30.0)


def test_jet_damping_uses_configured_nozzle_geometry():
    cfg = create_test_config(stage1_engine_ring_radius_m=0.0, stage1_nozzle_exit_radius_m=1.0)
    ctx = DynamicsContext(
        thrust_on=True, throttle=1.0, dry_mass=None, stage=1, vehicle_model="booster",
        booster_aero_mode="BOOSTER_LANDING", thrust_magnitude_override=None, grid_fin_command=None,
        config=cfg,
    )
    tau = compute_jet_damping_torque(-100.0, 20.0, np.array([0.0, 0.0, 1.0]), ctx)
    assert tau[2] == pytest.approx(100.0 * 0.5 * 1.0 ** 2)


# ------------------------------------------------------------------ campaigns
def test_build_cases_modes_and_replay():
    assert len(campaign.build_cases("monte_carlo", runs=5, seed=3)) == 5
    sweep = campaign.build_cases("sweep_3sigma", dispersions="extended")
    assert len(sweep) == 1 + 2 * len(campaign.ALL_PARAMETERS)
    assert sweep[1]["z_scores"] == {campaign.ALL_PARAMETERS[0]: -3.0}
    replay = campaign.build_cases("monte-carlo", runs=100, seed=42, case_ids=["mc_0011"])
    assert [c["case_id"] for c in replay] == ["mc_0011"]
    assert replay[0]["z_scores"] == campaign.sample_monte_carlo_cases(100, 42)[10]["z_scores"]
    with pytest.raises(ValueError, match="Unknown case id"):
        campaign.build_cases("sensitivity", case_ids=["nope"])
    with pytest.raises(ValueError, match="mode must be"):
        campaign.build_cases("grid")


def test_server_replays_campaign_case():
    import asyncio

    from rlv_sim.server.api_models import SimulationSetup
    from rlv_sim.server.app import frontend_config_defaults, map_config

    d = asyncio.run(frontend_config_defaults())
    setup = {
        "vehicle": d["vehicle"], "mission": d["mission"], "physics": d["physics"],
        "recovery": {k: v for k, v in d["recovery"].items() if k != "gfold_available"},
        "campaign_case": {"mode": "monte_carlo", "runs": 100, "seed": 42, "case_id": "mc_0011"},
    }
    cfg = map_config(SimulationSetup(**setup))
    case = campaign.sample_monte_carlo_cases(100, 42)[10]
    assert cfg.runtime_wind_offset_mps == pytest.approx(case["z_scores"]["wind"] * 5.0)


# ------------------------------------------------------------------ model card
def test_model_card_has_models_limits_and_credibility():
    from rlv_sim.outputs.model_card import model_card

    card = model_card()
    assert len(card["models"]) >= 8
    assert all({"area", "model", "basis", "limit", "pedigree"} <= set(m) for m in card["models"])
    assert card["known_limits"]
    assert card["credibility"]["input_pedigree_score"] == 1


@pytest.mark.slow
def test_example_plugin_flies_a_full_landing():
    from rlv_sim.mission.full_mission import run_full_mission

    cfg = create_default_config(verbose=False, booster_landing_guidance="plugin",
                                booster_landing_plugin="example_zem_zev")
    result = run_full_mission(config=cfg, verbose=False)
    assert result.booster_landing_success
    assert result.booster_site_error_m < cfg.booster_pad_tolerance_m

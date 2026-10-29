"""CLI argument/config mapping and plot-generation smoke tests."""

import json
import os

import matplotlib

matplotlib.use("Agg")

import pytest

from rlv_sim.cli import _resolve_output_dir, build_config_from_args, parse_args


def test_default_args_build_research_config():
    args = parse_args([])
    assert args.mission == "full"
    assert args.nav_mode == "truth"
    config = build_config_from_args(args)
    assert config.dt == 0.05
    assert not config.enable_demo_mode


def test_demo_flag_uses_coarse_timestep():
    config = build_config_from_args(parse_args(["--demo"]))
    assert config.enable_demo_mode
    assert config.dt == 0.25


def test_demo_profile_keeps_explicit_dt():
    config = build_config_from_args(parse_args(["--profile", "demo", "--dt", "0.1"]))
    assert config.dt == 0.1


def test_overrides_are_applied():
    args = parse_args([
        "--max-time", "120", "--dt", "0.02", "--nav-mode", "onboard",
        "--attitude-controller", "pid", "--recovery-attitude-controller", "pd",
        "--landing-guidance", "heuristic", "--disable-abort-modes",
    ])
    config = build_config_from_args(args)
    assert config.max_time == 120
    assert config.dt == 0.02
    assert config.enable_gps and config.enable_imu and config.use_navigation_estimate_for_guidance
    assert config.attitude_controller == "pid"
    assert config.recovery_attitude_controller == "pd"
    assert config.booster_landing_guidance == "heuristic"
    assert not config.enable_abort_modes


def test_enable_abort_modes():
    assert build_config_from_args(parse_args(["--enable-abort-modes"])).enable_abort_modes


def test_reuse_stage2_extends_max_time():
    base = build_config_from_args(parse_args([]))
    config = build_config_from_args(parse_args(["--reuse-stage2", "--s2-orbit-hold", "100"]))
    assert config.enable_s2_recovery
    assert config.s2_orbit_hold_time_s == 100.0
    assert config.max_time >= base.max_time


def test_reuse_stage2_respects_explicit_max_time():
    config = build_config_from_args(parse_args(["--reuse-stage2", "--max-time", "500"]))
    assert config.max_time == 500


def test_config_json_overrides_profile(tmp_path):
    path = tmp_path / "cfg.json"
    path.write_text(json.dumps({"dt": 0.07}), encoding="utf-8")
    config = build_config_from_args(parse_args(["--demo", "--config-json", str(path)]))
    assert config.dt == 0.07
    assert not config.enable_demo_mode


def test_invalid_choice_exits():
    with pytest.raises(SystemExit):
        parse_args(["--nav-mode", "bogus"])


def test_resolve_output_dir(tmp_path):
    assert _resolve_output_dir(parse_args(["-o", str(tmp_path)])) == str(tmp_path)
    assert _resolve_output_dir(parse_args(["-o", "rel"])) == os.path.join(os.getcwd(), "rel")


@pytest.mark.slow
def test_generate_all_plots_from_short_ascent(tmp_path):
    from rlv_sim.config.factory import create_default_config
    from rlv_sim.mission.single_vehicle import run_simulation
    from rlv_sim.outputs.plots.mission import generate_all_plots

    config = create_default_config(dt=0.1, max_time=60.0)
    _, log, _ = run_simulation(verbose=False, config=config)
    files = generate_all_plots(log, str(tmp_path))
    assert files
    assert all(os.path.isfile(f) for f in files)

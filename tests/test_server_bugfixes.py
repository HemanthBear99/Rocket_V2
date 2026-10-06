"""Regression tests for confirmed server.py bugs fixed after the audit.

1. demo_mode used to silently overwrite a caller-supplied `dt` in
   map_config() with create_demo_config()'s dt (0.1), discarding the
   user's explicit choice with no warning. Fixed to only toggle the demo
   coast-skipping behavior.
2. CampaignRequest.workers was accepted/validated (1-16) but never read in
   _run_campaign_task -- server-triggered campaigns always ran sequentially
   regardless of the requested worker count. Fixed to mirror
   campaign.run_campaign()'s ProcessPoolExecutor fan-out when workers > 1.
"""

import pytest

from rlv_sim.api_models import (
    MissionSetup,
    PhysicsSetup,
    RecoverySetup,
    SimulationSetup,
    VehicleSetup,
)
from rlv_sim.server import CampaignRequest, map_config


def _base_setup(**overrides) -> SimulationSetup:
    vehicle = VehicleSetup(
        stage1_dry_mass=22200.0,
        stage1_prop_mass=410900.0,
        stage2_dry_mass=4000.0,
        stage2_prop_mass=107500.0,
        payload_mass=8000.0,
        thrust_sl=7.607e6,
        thrust_vac=934000.0,
        isp_sl=282.0,
        isp_vac=311.0,
        max_gimbal_deg=15.0,
        throttle_min=0.35,
        throttle_max=1.0,
    )
    mission = MissionSetup(
        target_alt_km=200.0,
        target_inclination_deg=28.5,
        orbit_insertion_start_alt_km=120.0,
        stage_sep_velocity=2200.0,
        dt=0.02,
        t_max=600.0,
    )
    recovery = RecoverySetup(
        landing_lat=28.5,
        landing_lon=-80.5,
        grid_fins=True,
        landing_legs=True,
    )
    physics = PhysicsSetup(j2=True, atmosphere=True, drag=True, lift=True, sensor_noise=False)
    kwargs = dict(
        vehicle=vehicle,
        mission=mission,
        recovery=recovery,
        physics=physics,
        realtime_mode=False,
        demo_mode=False,
        enable_s2_recovery=False,
    )
    kwargs.update(overrides)
    return SimulationSetup(**kwargs)


class TestDemoModeDoesNotOverrideDt:
    def test_explicit_dt_preserved_with_demo_mode_on(self):
        setup = _base_setup(demo_mode=True)
        config = map_config(setup)
        # Previously this would have been forced to create_demo_config().dt
        # (0.1) regardless of the caller's explicit mission.dt=0.02.
        assert config.dt == pytest.approx(0.02)

    def test_demo_flag_and_coast_dt_still_applied(self):
        setup = _base_setup(demo_mode=True)
        config = map_config(setup)
        assert config.enable_demo_mode is True
        assert config.demo_coast_max_dt > 0

    def test_dt_unaffected_when_demo_mode_off(self):
        setup = _base_setup(demo_mode=False)
        config = map_config(setup)
        assert config.dt == pytest.approx(0.02)
        assert config.enable_demo_mode is False


class TestCampaignWorkersWired:
    def test_workers_field_still_validated_1_to_16(self):
        with pytest.raises(Exception):
            CampaignRequest(workers=0)
        with pytest.raises(Exception):
            CampaignRequest(workers=17)
        assert CampaignRequest(workers=8).workers == 8

    def test_sequential_and_parallel_paths_reachable_by_reading_source(self):
        # A direct process-pool integration test would spawn real OS
        # processes and is out of scope for the fast unit suite; instead
        # confirm the fix is actually present: _run_campaign_task must
        # branch on meta["workers"] and use ProcessPoolExecutor when >1,
        # rather than always looping sequentially.
        import inspect

        from rlv_sim.server import _run_campaign_task

        source = inspect.getsource(_run_campaign_task)
        assert "workers" in source
        assert "ProcessPoolExecutor" in source

    def test_sequential_path_actually_runs_all_cases(self):
        from rlv_sim.config_factory import create_test_config
        from rlv_sim.server import _run_campaign_task

        base = create_test_config(dt=1.0, max_time=3.0, verbose=False)
        meta = {
            "config": base,
            "mode": "sensitivity",
            "runs": 0,
            "seed": 1,
            "workers": 1,
            "status": "running",
            "cancel": False,
        }
        _run_campaign_task("test-campaign-seq", meta)
        assert meta["status"] == "completed"
        assert meta["completed"] == meta["total"]
        assert len(meta["results"]) == meta["total"]

    def test_parallel_path_actually_runs_all_cases(self):
        from rlv_sim.config_factory import create_test_config
        from rlv_sim.server import _run_campaign_task

        base = create_test_config(dt=1.0, max_time=3.0, verbose=False)
        meta = {
            "config": base,
            "mode": "sensitivity",
            "runs": 0,
            "seed": 1,
            "workers": 2,
            "status": "running",
            "cancel": False,
        }
        _run_campaign_task("test-campaign-par", meta)
        assert meta["status"] == "completed"
        assert meta["completed"] == meta["total"]
        assert len(meta["results"]) == meta["total"]
        result_ids = {row["case_id"] for row in meta["results"]}
        assert "baseline" in result_ids

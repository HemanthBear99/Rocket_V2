"""Mission planner: rocket-equation feasibility matches simulated outcomes."""

import pytest

from rlv_sim.config_factory import create_default_config
from rlv_sim.mission_planner import plan_mission


@pytest.mark.parametrize(("payload", "verdict"), [(8000.0, "ok"), (11000.0, "warn"), (12000.0, "fail")])
def test_payload_feasibility_matches_simulation(payload, verdict):
    # Full-mission runs: 11 t reaches orbit, 12 t runs S2 out of propellant.
    assert plan_mission(create_default_config(payload_mass=payload))["verdict"] == verdict


def test_low_thrust_vehicle_fails_liftoff_check():
    cfg = create_default_config(stage1_engine_count=6)
    checks = {c["name"]: c for c in plan_mission(cfg)["checks"]}
    assert checks["liftoff_twr"]["status"] == "fail"


def test_plan_endpoint_round_trip():
    import asyncio

    from rlv_sim.api_models import SimulationSetup
    from rlv_sim.server import frontend_config_defaults, plan_mission_endpoint

    d = asyncio.run(frontend_config_defaults())
    recovery = {k: v for k, v in d["recovery"].items() if k != "gfold_available"}
    setup = SimulationSetup(vehicle=d["vehicle"], mission=d["mission"], recovery=recovery, physics=d["physics"])
    assert asyncio.run(plan_mission_endpoint(setup))["verdict"] == "ok"

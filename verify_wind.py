"""Verify movable target + wind feed-forward across wind and dispersion."""
from rlv_sim._run_full_mission import run_full_mission
from rlv_sim.config_factory import create_demo_config
from rlv_sim.mission_summary import assess_full_mission

CASES = [
    ("nominal", {}),
    ("wind+12 fixed-target", {"runtime_wind_offset_mps": 12.0}),
    ("wind+12 movable", {"runtime_wind_offset_mps": 12.0, "enable_movable_landing_target": True}),
    ("wind-12 movable", {"runtime_wind_offset_mps": -12.0, "enable_movable_landing_target": True}),
    ("wind+25 movable", {"runtime_wind_offset_mps": 25.0, "enable_movable_landing_target": True}),
    ("wind-25 movable", {"runtime_wind_offset_mps": -25.0, "enable_movable_landing_target": True}),
    ("mass+800 wind+20 mov", {"runtime_initial_mass_offset_kg": 800.0,
                              "runtime_wind_offset_mps": 20.0,
                              "enable_movable_landing_target": True}),
    ("thrust.98 wind+20 mov", {"runtime_thrust_scale": 0.98,
                               "runtime_wind_offset_mps": 20.0,
                               "enable_movable_landing_target": True}),
]

for name, ov in CASES:
    cfg = create_demo_config(**ov)
    res = run_full_mission(config=cfg, verbose=False)
    a = assess_full_mission(res, cfg)
    print(
        "%-22s success=%-5s landing=%-20s speed=%-6s site_err=%-8s ecc=%.4f %s"
        % (
            name,
            a.mission_success,
            a.landing.status,
            a.landing.touchdown_speed_mps,
            a.landing.site_error_m,
            a.orbit.eccentricity,
            ("failed=" + str(a.failed_criteria)) if a.failed_criteria else "",
        )
    )
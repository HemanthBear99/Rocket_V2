"""Factory functions for common SimulationConfig profiles."""

from typing import Any

from .config_definition import SimulationConfig


def _with_overrides(config: SimulationConfig, **overrides: Any) -> SimulationConfig:
    if overrides:
        config = SimulationConfig(**{**config.__dict__, **overrides})
    config.validate()
    return config


def create_default_config(**overrides: Any) -> SimulationConfig:
    """Create a default research-profile configuration."""
    return _with_overrides(SimulationConfig(
        stage1_landing_fuel_reserve_kg=67500.0,
        booster_entry_budget_kg=12500.0,
        booster_entry_burn_min_altitude_m=5000.0,
        booster_landing_reserve_kg=5000.0,
        booster_landing_propellant_margin_kg=750.0,
        booster_landing_vertical_propellant_safety_factor=1.25,
        booster_landing_ignition_safety_factor=1.8,
        booster_landing_ignition_ceiling_m=8000.0,
        booster_powered_descent_lead_time_s=28.0,
        booster_divert_fraction_high=0.72,
        booster_divert_fraction_mid=0.62,
        booster_divert_fraction_low=0.40,
        booster_max_divert_accel_far_mps2=40.0,
        booster_max_divert_accel_near_mps2=25.0,
        booster_late_entry_east_bias_m=5000.0,
        booster_fuel_critical_ratio=0.35,
        booster_divert_band_high_alt_m=1200.0,
        booster_terminal_capture_floor_kg=2500.0,
        booster_terminal_capture_floor_margin_kg=0.0,
    ), **overrides)


                                                     
                                                                   


def create_demo_config(**overrides: Any) -> SimulationConfig:
    """Create a faster research-profile configuration for live demos."""
    return create_default_config(
                                                                          
                                                                            
        dt=0.1,
        enable_demo_mode=True,
        demo_coast_max_dt=2.0,
        **overrides,
    )


def create_test_config(
    dt: float = 0.1,
    max_time: float = 10.0,
    **overrides: Any,
) -> SimulationConfig:
    """Create a short-duration test configuration."""
    return _with_overrides(
        SimulationConfig(dt=dt, max_time=max_time, verbose=False),
        **overrides,
    )


__all__ = [
    "create_default_config",
    "create_demo_config",
    "create_test_config",
]

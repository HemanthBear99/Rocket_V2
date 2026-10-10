"""Factory functions for common SimulationConfig profiles."""

from typing import Any

from .config_definition import SimulationConfig

# Fields removed because nothing read them; still accepted (and ignored) so
# older saved configs keep loading.
RETIRED_FIELDS = frozenset((
    "ascent_max_q_prediction_horizon_s",
    "s2_landing_start_altitude_m",
    "landing_target_cross_track_offset_m",
    "landing_leg_footprint_radius_m",
    "booster_terminal_capture_floor_margin_kg",
    "booster_late_entry_east_bias_m",
    "booster_late_entry_bias_scale_m",
    "booster_late_entry_bias_miss_ref_m",
    "booster_late_entry_bias_miss_cutoff_m",
    "booster_terminal_pad_capture_alt_m",
    "booster_terminal_offsite_chase_min_alt_m",
    "booster_terminal_offsite_chase_max_alt_m",
    "booster_terminal_capture_geom_band_m",
    "booster_terminal_capture_geom_altitude_m",
    "booster_terminal_capture_geom_t_go_min_s",
    "booster_terminal_capture_geom_t_go_max_s",
    "booster_terminal_capture_geom_retrograde_blend",
    "booster_terminal_capture_geom_max_throttle",
    "imu_gyro_bias",
    "imu_gyro_noise",
    "zero_tolerance",
    "energy_violation_terminates",
))


def _with_overrides(config: SimulationConfig, **overrides: Any) -> SimulationConfig:
    overrides = {k: v for k, v in overrides.items() if k not in RETIRED_FIELDS}
    if overrides:
        config = SimulationConfig(**{**config.__dict__, **overrides})
    if config.auto_size_recovery_reserve:
        from .mission_planner import size_recovery_reserve

        config = SimulationConfig(**{**config.__dict__,
                                     "stage1_landing_fuel_reserve_kg": size_recovery_reserve(config)})
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
        booster_fuel_critical_ratio=0.35,
        booster_divert_band_high_alt_m=1200.0,
        booster_terminal_capture_floor_kg=2500.0,
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

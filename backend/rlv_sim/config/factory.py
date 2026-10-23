"""Factory functions for common SimulationConfig profiles."""

from typing import Any

from rlv_sim.config.definition import SimulationConfig

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
    modes = overrides.get("runtime_wind_profile_modes")
    if isinstance(modes, list):
        # JSON clients send lists; the frozen config holds hashable tuples.
        overrides["runtime_wind_profile_modes"] = tuple(
            tuple(mode) if isinstance(mode, list) else mode for mode in modes
        )
    if overrides:
        config = SimulationConfig(**{**config.__dict__, **overrides})
    if config.auto_size_recovery_reserve:
        from rlv_sim.mission.planner import size_recovery_reserve

        config = SimulationConfig(**{**config.__dict__,
                                     "stage1_landing_fuel_reserve_kg": size_recovery_reserve(config)})
    if config.recovery_reserve_wind_sensitivity_kg_per_mps > 0.0:
        from rlv_sim.mission.planner import wind_reserve_adjustment_kg

        config = SimulationConfig(**{
            **config.__dict__,
            "stage1_landing_fuel_reserve_kg": (
                config.stage1_landing_fuel_reserve_kg + wind_reserve_adjustment_kg(config)
            ),
        })
    config.validate()
    return config


def create_default_config(**overrides: Any) -> SimulationConfig:
    """Create a default research-profile configuration.

    The research profile is the dataclass defaults; this only applies the
    override hooks (auto-sized and wind-aware reserves) and validation.
    """
    return _with_overrides(SimulationConfig(), **overrides)


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

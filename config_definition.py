"""
RLV Phase-I Ascent Simulation - Configuration Definitions

This module provides the core configuration dataclass for the RLV simulation,
separating configuration definitions from factory logic.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from . import constants as C

AttitudeController = Literal["pd", "pid"]
RecoveryAttitudeController = Literal["pd", "pid", "auto"]


VALIDATED_RUNTIME_FIELDS = (
    "dt", "max_time",
    "kp_attitude", "kd_attitude", "ki_attitude",
    "attitude_controller", "recovery_attitude_controller",
    "attitude_integral_windup_fraction", "max_torque",
    "max_gimbal_angle_deg", "min_engine_throttle_fraction",
    "max_engine_throttle_fraction",
    "gravity_turn_start_altitude", "gravity_turn_transition_range",
    "min_velocity_for_turn", "pitchover_start_altitude",
    "pitchover_end_altitude", "pitchover_angle",
    "ascent_max_q_target_pa", "ascent_max_q_prediction_horizon_s",
    "stage1_landing_fuel_reserve_kg", "launch_site_lat_deg",
    "launch_site_lon_deg", "launch_site_altitude_m",
    "booster_boostback_budget_kg", "booster_entry_budget_kg",
    "booster_boostback_impact_corridor_m",
    "booster_landing_reserve_kg", "booster_flip_min_time_s",
    "booster_apogee_target_km", "booster_entry_interface_altitude_m",
    "booster_entry_burn_min_altitude_m", "booster_entry_burn_min_speed_mps",
    "booster_landing_ignition_safety_factor", "booster_landing_ignition_ceiling_m",
    "booster_recovery_max_q_pa", "booster_recovery_max_q_alpha_pa_rad",
    "booster_terminal_attitude_error_max_deg", "booster_landing_propellant_margin_kg",
    "booster_landing_vertical_propellant_safety_factor",
    "booster_terminal_capture_landing_reserve_fraction",
    "booster_tail_first_cp_z_m",
    "booster_landing_target_downrange_km", "booster_landing_site_lat_deg",
    "booster_landing_site_lon_deg", "booster_landing_site_altitude_m",
    "booster_pad_tolerance_m", "booster_enforce_pad_landing",
    "enable_grid_fins", "enable_landing_legs", "grid_fin_deploy_altitude_m",
    "grid_fin_max_deflection_deg", "grid_fin_area_m2", "grid_fin_cl_max",
    "grid_fin_cd_increment", "landing_leg_deploy_altitude_m",
    "landing_leg_deploy_time_s", "landing_leg_max_touchdown_speed_mps",
    "landing_leg_max_tilt_deg", "landing_leg_footprint_radius_m",
    "booster_powered_descent_lead_time_s",
    "orbit_target_altitude_m", "orbit_altitude_tolerance_m",
    "orbit_ecc_max", "orbit_insertion_start_altitude_m",
    "orbit_insertion_timeout_s",
    "enable_atmosphere", "enable_drag", "enable_lift",
    "runtime_atmosphere_density_scale",
    "quaternion_norm_tolerance", "zero_tolerance",
    "enable_gps", "gps_position_sigma_m", "gps_velocity_sigma_mps",
    "gps_update_period_s", "gps_seed", "wind_seed",
    "enable_imu", "enable_landing_altimeter",
    "landing_altimeter_max_altitude_m", "landing_altimeter_altitude_sigma_m",
    "landing_altimeter_velocity_sigma_mps", "use_navigation_estimate_for_guidance",
    "verbose",
    "stage1_dry_mass", "stage1_prop_mass", "stage2_dry_mass", "stage2_prop_mass", "payload_mass",
    "stage2_thrust_vac", "stage2_isp_vac", "stage_sep_velocity", "target_inclination_deg",
)


EXPERIMENTAL_FEATURE_FLAGS = (
    "enable_j2", "enable_engine_transients", "abort_on_q_alpha_limit",
    "enable_separation_dynamics",
    "enable_upper_atmosphere", "enable_stochastic_wind",
    "enable_rcs", "enable_abort_modes", "enable_s2_recovery",
)


def _validate_aero_deck_path(deck_path: str) -> str | None:
    """Return an error message if deck_path is not an allowed aero deck.
    Only .json files inside the package's own aero_decks directory are
    accepted. The allowed root is resolved relative to this module so the
    check does not depend on the process working directory.
    """
    try:
        candidate = Path(deck_path)
    except (TypeError, ValueError):
        return f"aero_deck_path must be a string path, got {type(deck_path).__name__}"

    if candidate.suffix.lower() != ".json":
        return f"aero_deck_path must point to a .json deck, got {deck_path!r}"

    allowed_root = (Path(__file__).resolve().parent / "aero_decks").resolve()
    try:
        resolved = candidate.resolve()
    except OSError:
        return f"aero_deck_path could not be resolved: {deck_path!r}"

    if resolved.parent != allowed_root:
        return (
            f"aero_deck_path must be a .json file directly inside {allowed_root}, "
            f"got {deck_path!r}"
        )
    if not resolved.is_file():
        return f"aero_deck_path does not exist: {deck_path!r}"
    return None


_FIELD_RULES = {
    # kind: (predicate that flags an INVALID value, message phrase)
    "positive": (lambda value: value <= 0, "must be positive"),
    "nonneg": (lambda value: value < 0, "must be >= 0"),
    "at_least_1": (lambda value: value < 1.0, "must be >= 1.0"),
}


def _field_errors(config, rules) -> list[str]:
    """Return messages for ``(field_name, kind)`` rules that the config violates."""
    errors = []
    for name, kind in rules:
        is_invalid, phrase = _FIELD_RULES[kind]
        value = getattr(config, name)
        if is_invalid(value):
            errors.append(f"{name} {phrase}, got {value}")
    return errors

@dataclass(frozen=True)
class SimulationConfig:
    """
    Immutable configuration for simulation parameters.

    Using frozen=True ensures configs cannot be accidentally modified.
    Create new configs via dataclass replace() if needed.

    All validation is inlined in validate() — no sub-config classes.
    """


    dt: float = C.DT
    max_time: float = C.MAX_TIME


    enable_demo_mode: bool = False
    demo_coast_max_dt: float = 2.0
    # Orbiter step size while coasting above the atmosphere with the engine
    # off, once the booster has finished. Unpowered orbital arcs are smooth,
    # so a coarser RK4 step is accurate there. The step used is
    # max(dt, orbit_coast_max_dt); set it to dt or below to disable.
    orbit_coast_max_dt: float = 0.5


    kp_attitude: float = C.KP_ATTITUDE
    kd_attitude: float = C.KD_ATTITUDE
    ki_attitude: float = C.KI_ATTITUDE
    attitude_controller: AttitudeController = "pd"
    recovery_attitude_controller: RecoveryAttitudeController = "auto"
    attitude_integral_windup_fraction: float = C.ATTITUDE_INTEGRAL_WINDUP_FRACTION
    max_torque: float = C.MAX_TORQUE
    max_gimbal_angle_deg: float = 15.0


    gravity_turn_start_altitude: float = C.GRAVITY_TURN_START_ALTITUDE
    gravity_turn_transition_range: float = C.GRAVITY_TURN_TRANSITION_RANGE
    min_velocity_for_turn: float = C.MIN_VELOCITY_FOR_TURN
    pitchover_start_altitude: float = C.PITCHOVER_START_ALTITUDE
    pitchover_end_altitude: float = C.PITCHOVER_END_ALTITUDE
    pitchover_angle: float = C.PITCHOVER_ANGLE
    ascent_max_q_target_pa: float = 27000.0
    ascent_max_q_prediction_horizon_s: float = 1.0
    stage1_landing_fuel_reserve_kg: float = C.STAGE1_LANDING_FUEL_RESERVE
    launch_site_lat_deg: float = C.DEFAULT_LAUNCH_SITE_LAT_DEG
    launch_site_lon_deg: float = C.DEFAULT_LAUNCH_SITE_LON_DEG
    launch_site_altitude_m: float = 0.0
    orbit_target_altitude_m: float = C.TARGET_ORBIT_ALTITUDE


    orbit_altitude_tolerance_m: float = 75000.0
    orbit_ecc_max: float = 0.02
    orbit_insertion_start_altitude_m: float = 85000.0
    orbit_insertion_timeout_s: float = 5000.0
    enable_s2_recovery: bool = False
    s2_orbit_hold_time_s: float = 600.0
    s2_entry_interface_altitude_m: float = 70000.0
    s2_landing_start_altitude_m: float = 15000.0
    s2_deorbit_target_perigee_m: float = -80000.0
    s2_entry_drag_scale: float = 45.0
    s2_touchdown_speed_limit_mps: float = 5.0
    # Propellant (kg, above stage2_dry_mass+payload_mass) that
    # orbit-insertion (compute_orbit_insertion_guidance) and deorbit
    # (compute_deorbit_guidance) burns will not consume below, reserved
    # for the S2_LANDING drone-ship burn. This is a genuine capability
    # tradeoff, not a free fix -- default is 0.0 (no reserve, matches
    # pre-existing behavior) because this vehicle's propellant budget has
    # essentially no slack: with the default research-profile config, a
    # full mission naturally leaves only ~1720.9 kg after
    # insertion+deorbit (orbit-insertion stops on orbit_ok, not on running
    # low on fuel -- it is not currently propellant-limited at all).
    # Investigation into the S2 landing-burn CURRENT FAILURE (see the "S2
    # DRONE-SHIP CRASH" regression test) found the control law itself
    # (compute_s2_entry_landing_guidance) is sound -- a standalone probe
    # replaying the real failing run's ignition state landed cleanly at
    # 1.85 m/s given ~4200 kg of propellant at ignition, versus the 1720.9
    # kg actually available. But setting this reserve to 4000 in an
    # end-to-end run caused ORBIT INSERTION ITSELF to fail (perigee ended
    # at -208.4 km, re-entered mid-insertion) -- confirming insertion
    # needs essentially all the propellant this config's stage2_prop_mass
    # provides. A meaningful reserve here requires either increasing
    # stage2_prop_mass, making orbit-insertion/deorbit guidance more
    # propellant-efficient, or reducing the downrange miss the landing
    # burn has to correct for -- none of which this field alone can
    # provide. Left in place (rather than removed) so a config that DOES
    # have propellant headroom (e.g. a larger stage2_prop_mass) can opt in.
    s2_landing_propellant_reserve_kg: float = 0.0


    stage1_dry_mass: float = C.STAGE1_DRY_MASS
    stage1_prop_mass: float = C.STAGE1_PROPELLANT_MASS
    stage2_dry_mass: float = C.STAGE2_DRY_MASS
    stage2_prop_mass: float = C.STAGE2_PROPELLANT_MASS
    payload_mass: float = 8000.0


    stage2_thrust_vac: float = C.STAGE2_THRUST
    stage2_isp_vac: float = C.STAGE2_ISP_VAC
    stage_sep_velocity: float = 2200.0
    target_inclination_deg: float = 28.5


    booster_boostback_budget_kg: float = 43000.0
    booster_boostback_impact_corridor_m: float = 10000.0
    booster_entry_budget_kg: float = 5700.0
    booster_landing_reserve_kg: float = 6000.0
    booster_flip_min_time_s: float = 11.0
    booster_apogee_target_km: float = 85.0
    booster_entry_interface_altitude_m: float = 70000.0
    booster_entry_burn_min_altitude_m: float = 20000.0
    booster_entry_burn_min_speed_mps: float = 250.0
    booster_landing_ignition_safety_factor: float = 1.52
    booster_landing_ignition_ceiling_m: float = 20000.0
    booster_recovery_max_q_pa: float = 90000.0
    booster_recovery_max_q_alpha_pa_rad: float = 150000.0
    booster_terminal_attitude_error_max_deg: float = 15.0
    booster_landing_propellant_margin_kg: float = 350.0
    booster_landing_vertical_propellant_safety_factor: float = 1.20
    booster_terminal_capture_landing_reserve_fraction: float = 1.0
    booster_tail_first_cp_z_m: float = C.STAGE1_RECOVERY_CP
    booster_landing_target_downrange_km: float = 0.0
    # Diagnostic recovery-trajectory planner (planner_* telemetry columns).
    # It never steers the vehicle and costs ~10% of run time, so it is off by
    # default; enable it to log candidate-trajectory scores.
    enable_recovery_planner_diagnostics: bool = False
    # Booster landing-burn guidance law (the 6-DOF dynamics are identical):
    #   "heuristic" - latched suicide burn with ZEM/ZEV divert (default)
    #   "gfold"     - G-FOLD convex minimum-fuel powered descent, re-planned
    #                 every gfold_replan_period_s (needs the optional cvxpy)
    booster_landing_guidance: str = "heuristic"
    gfold_replan_period_s: float = 1.0
    gfold_nodes: int = 25
    gfold_glideslope_deg: float = 5.0
    # Terminal gate: G-FOLD flies to this height above the pad, descending at
    # the landing target rate; the vertical-descent law takes over below it.
    gfold_terminal_altitude_m: float = 15.0
    booster_landing_site_lat_deg: float | None = None
    booster_landing_site_lon_deg: float | None = None
    booster_landing_site_altitude_m: float = 0.0
    # Physical TVC moment arm from the engine gimbal plane to the vehicle CG.
    # None keeps the historical constants.py values. A real Falcon-class
    # Merlin TVC gimbals within a few metres, so an explicit value well below
    # the legacy 21 m booster arm is the more physical choice when pitch
    # authority needs to be conservative.
    tvc_lever_arm_m: float | None = None
    booster_pad_tolerance_m: float = 50.0
    booster_enforce_pad_landing: bool = True
    # --- Movable (drone-ship style) landing target ---
    # Falcon-class recovery does not fly to a fixed point: a drone ship is
    # towed downrange so the vehicle's wind-dispersed trajectory intersects it.
    # When enable_movable_landing_target is set, the effective target downrange
    # is offset from booster_landing_target_downrange_km by a wind-propagated
    # downrange dispersion, so the target meets the vehicle instead of the
    # vehicle being forced onto a fixed pad. This models the dispatch/targeting
    # layer, NOT any vehicle capability.
    enable_movable_landing_target: bool = False
    # Wind only deflects the vehicle where there is air to couple through:
    # measured overshoot is ~734 m for a 12 m/s wind delta, implying ~61 s of
    # effective drag-coupled exposure, NOT the whole ~170 s recovery. The
    # previous 150 s default over-shifted the target ~2.4x and made the landing
    # error worse (810 m vs a 750 m fixed-target miss).
    landing_target_wind_exposure_s: float = 61.0
    landing_target_wind_gain: float = 1.0
    landing_target_max_downrange_offset_km: float = 40.0
    # Ground-relative cross-track offset applied to the target, signed along
    # the local east axis (positive = east). Used for drone-ship stations
    # displaced along the flight corridor.
    landing_target_cross_track_offset_m: float = 0.0
    enable_grid_fins: bool = True
    enable_landing_legs: bool = True
    grid_fin_deploy_altitude_m: float = 70000.0
    grid_fin_max_deflection_deg: float = 25.0
    grid_fin_area_m2: float = 8.0
    grid_fin_drag_area_m2: float = C.GRID_FIN_COUNT * C.GRID_FIN_AREA_M2
    grid_fin_cl_max: float = 1.2
    grid_fin_cd_increment: float = 0.08
    landing_leg_deploy_altitude_m: float = 1500.0
    landing_leg_deploy_time_s: float = 3.0
    landing_leg_max_touchdown_speed_mps: float = 6.0
    landing_leg_max_tilt_deg: float = 15.0
    landing_leg_footprint_radius_m: float = 8.0
    booster_powered_descent_lead_time_s: float = 15.0


    booster_terminal_capture_floor_kg: float = 2500.0
    booster_terminal_capture_floor_margin_kg: float = 0.0
    booster_late_entry_east_bias_m: float = 3200.0
    booster_late_entry_bias_ref_altitude_m: float = 35000.0
    booster_late_entry_bias_scale_m: float = 30000.0
    booster_late_entry_bias_miss_ref_m: float = 1200.0
    booster_late_entry_bias_miss_cutoff_m: float = 400.0
    booster_max_divert_accel_far_mps2: float = 40.0
    booster_max_divert_accel_near_mps2: float = 40.0
    booster_divert_accel_miss_threshold_m: float = 3000.0
    booster_divert_fraction_high: float = 0.55
    booster_divert_fraction_mid: float = 0.45
    booster_divert_fraction_low: float = 0.28
    booster_divert_band_high_alt_m: float = 500.0
    booster_divert_band_low_alt_m: float = 150.0
    booster_vertical_load_risk_ratio: float = 0.72
    booster_terminal_descent_overspeed_factor: float = 4.0
    booster_fuel_critical_ratio: float = 0.35
    booster_terminal_hcapture_tstop_min_s: float = 1.5
    booster_terminal_hcapture_tstop_max_s: float = 3.0
    booster_terminal_pad_capture_alt_m: float = 150.0
    booster_terminal_offsite_chase_min_alt_m: float = 80.0
    booster_terminal_offsite_chase_max_alt_m: float = 10000.0
    booster_terminal_capture_geom_band_m: float = 1600.0
    booster_terminal_capture_geom_altitude_m: float = 8500.0
    booster_terminal_capture_geom_t_go_min_s: float = 12.0
    booster_terminal_capture_geom_t_go_max_s: float = 24.0
    booster_terminal_capture_geom_retrograde_blend: float = 0.74
    booster_terminal_capture_geom_max_throttle: float = 0.10


    enable_atmosphere: bool = True
    enable_high_fidelity_gram: bool = False
    enable_drag: bool = True
    enable_lift: bool = True
    runtime_atmosphere_density_scale: float = 1.0
    enable_j2: bool = True
    j2_coefficient: float = 1.08263e-3
    gravity_model: str = "egm96"
    enable_engine_transients: bool = True
    engine_spool_up_time: float = 1.5
    engine_spool_down_time: float = 0.8
    abort_on_q_alpha_limit: bool = False
    q_alpha_max: float = 50000.0
    enable_separation_dynamics: bool = True
    separation_delta_v: float = 1.5
    separation_tumble_rate: float = 0.02
    atmosphere_model: str = "us76_deterministic"
    enable_upper_atmosphere: bool = False
    enable_stochastic_wind: bool = False
    wind_turbulence_intensity: float = 2.0
    wind_gust_magnitude: float = 15.0
    enable_rcs: bool = True
    rcs_thrust: float = C.RCS_THRUST_PER_THRUSTER
    rcs_isp: float = C.RCS_ISP
    rcs_num_thrusters: int = 8
    rcs_propellant_mass: float = C.RCS_PROPELLANT_MASS

    imu_accel_bias: float = 0.001
    imu_accel_noise: float = 0.01
    imu_gyro_bias: float = 1e-5
    imu_gyro_noise: float = 1e-4
    enable_gps: bool = False
    gps_position_sigma_m: float = 3.0
    gps_velocity_sigma_mps: float = 0.05
    gps_update_period_s: float = 0.2
    gps_seed: int | None = 42
    # Previously the stochastic wind RNG silently reused the raw gps_seed
    # value with no offset (unlike IMU=gps_seed+1 / altimeter=gps_seed+2),
    # despite its GPS-specific name -- callers had no way to vary wind
    # noise independently of GPS noise. None (default) preserves the
    # legacy derived-from-gps_seed behavior (gps_seed + 3, matching the
    # existing IMU/altimeter offset pattern); set explicitly to seed wind
    # independently.
    wind_seed: int | None = None
    enable_imu: bool = False
    enable_landing_altimeter: bool = False
    landing_altimeter_max_altitude_m: float = 5000.0
    landing_altimeter_altitude_sigma_m: float = 0.25
    landing_altimeter_velocity_sigma_mps: float = 0.05
    use_navigation_estimate_for_guidance: bool = False
    enable_abort_modes: bool = False
    abort_q_alpha_threshold: float = 60000.0
    abort_attitude_threshold: float = 30.0


    quaternion_norm_tolerance: float = C.QUATERNION_NORM_TOL
    zero_tolerance: float = 1e-10


    enable_run_manifest: bool = True
    energy_violation_terminates: bool = False
    min_engine_throttle_fraction: float = 0.35
    max_engine_throttle_fraction: float = 1.0


    runtime_thrust_scale: float = 1.0
    runtime_isp_scale: float = 1.0
    runtime_initial_mass_offset_kg: float = 0.0
    runtime_wind_offset_mps: float = 0.0
    aero_deck_path: str | None = None


    verbose: bool = True

    @classmethod
    def validated_runtime_fields(cls) -> tuple[str, ...]:
        return VALIDATED_RUNTIME_FIELDS

    @classmethod
    def experimental_feature_flags(cls) -> tuple[str, ...]:
        return EXPERIMENTAL_FEATURE_FLAGS

    def experimental_flags_state(self) -> dict[str, bool]:
        return {name: bool(getattr(self, name)) for name in EXPERIMENTAL_FEATURE_FLAGS}

    @property
    def meco_mass_kg(self) -> float:
        """Stacked-vehicle mass at main-engine cutoff.

        Stage 1 dry + stage 2 wet + payload + the stage 1 landing reserve.
        Single source of truth for ascent guidance, the mission manager and
        the ascent loop's dry-mass floor.
        """
        return (
            float(self.stage1_dry_mass)
            + float(self.stage2_dry_mass)
            + float(self.stage2_prop_mass)
            + float(self.payload_mass)
            + float(self.stage1_landing_fuel_reserve_kg)
        )

    def validate(self) -> None:
        """Validate all configuration fields.  Raises ValueError on first
        batch of errors with a readable message."""
        errors: list[str] = []

        # aero_deck_path is consumed as a filesystem path
        # (aero_database.py does open(path) + json.load) and is reachable
        # from untrusted input via the server's advanced_overrides escape
        # hatch, which is typed dict[str, float|int|bool|str] and merged
        # straight into this config. It was previously never validated,
        # giving any API caller an arbitrary-file-read / file-existence
        # oracle. Confine it to a real .json file inside the package's own
        # aero_decks directory.
        if self.aero_deck_path is not None:
            deck_error = _validate_aero_deck_path(self.aero_deck_path)
            if deck_error is not None:
                errors.append(deck_error)


        errors += _field_errors(self, (
            ("kp_attitude", "nonneg"),
            ("kd_attitude", "nonneg"),
            ("ki_attitude", "nonneg"),
        ))
        if self.attitude_controller not in ("pd", "pid"):
            errors.append(f"attitude_controller must be 'pd' or 'pid', got {self.attitude_controller!r}")
        if self.recovery_attitude_controller not in ("pd", "pid", "auto"):
            errors.append(
                "recovery_attitude_controller must be 'pd', 'pid', or 'auto', "
                f"got {self.recovery_attitude_controller!r}"
            )
        if not (0.0 < self.attitude_integral_windup_fraction <= 1.0):
            errors.append(
                f"attitude_integral_windup_fraction must be in (0, 1], got {self.attitude_integral_windup_fraction}"
            )
        errors += _field_errors(self, (
            ("max_torque", "positive"),
            ("stage1_dry_mass", "positive"),
            ("stage1_prop_mass", "positive"),
            ("stage2_dry_mass", "positive"),
            ("stage2_prop_mass", "positive"),
            ("payload_mass", "nonneg"),
            ("stage2_thrust_vac", "positive"),
            ("stage2_isp_vac", "positive"),
        ))
        if not (0.0 <= self.max_gimbal_angle_deg <= 30.0):
            errors.append(f"max_gimbal_angle_deg must be in [0, 30], got {self.max_gimbal_angle_deg}")
        if not (0.0 < self.min_engine_throttle_fraction <= 1.0):
            errors.append(
                f"min_engine_throttle_fraction must be in (0, 1], got {self.min_engine_throttle_fraction}"
            )
        if not (0.0 < self.max_engine_throttle_fraction <= 1.0):
            errors.append(
                f"max_engine_throttle_fraction must be in (0, 1], got {self.max_engine_throttle_fraction}"
            )
        if self.min_engine_throttle_fraction > self.max_engine_throttle_fraction:
            errors.append(
                "min_engine_throttle_fraction must be <= max_engine_throttle_fraction, got "
                f"{self.min_engine_throttle_fraction} > {self.max_engine_throttle_fraction}"
            )


        if not (-90.0 <= self.launch_site_lat_deg <= 90.0):
            errors.append(f"launch_site_lat_deg must be in [-90, 90], got {self.launch_site_lat_deg}")
        if not (-180.0 <= self.launch_site_lon_deg <= 180.0):
            errors.append(f"launch_site_lon_deg must be in [-180, 180], got {self.launch_site_lon_deg}")
        errors += _field_errors(self, (
            ("stage1_landing_fuel_reserve_kg", "nonneg"),
            ("orbit_target_altitude_m", "positive"),
            ("orbit_altitude_tolerance_m", "nonneg"),
        ))
        if not (0.0 < self.orbit_ecc_max < 1.0):
            errors.append(f"orbit_ecc_max must be in (0, 1), got {self.orbit_ecc_max}")
        errors += _field_errors(self, (
            ("orbit_insertion_start_altitude_m", "positive"),
            ("orbit_insertion_timeout_s", "positive"),
            ("s2_orbit_hold_time_s", "nonneg"),
            ("s2_entry_interface_altitude_m", "positive"),
            ("s2_landing_start_altitude_m", "positive"),
            ("s2_entry_drag_scale", "at_least_1"),
        ))
        if self.s2_deorbit_target_perigee_m >= self.s2_entry_interface_altitude_m:
            errors.append(
                "s2_deorbit_target_perigee_m must be below s2_entry_interface_altitude_m, "
                f"got {self.s2_deorbit_target_perigee_m}"
            )
        errors += _field_errors(self, (
            ("s2_touchdown_speed_limit_mps", "positive"),
            ("s2_landing_propellant_reserve_kg", "nonneg"),
            ("booster_landing_reserve_kg", "nonneg"),
            ("booster_flip_min_time_s", "nonneg"),
            ("booster_landing_ignition_safety_factor", "at_least_1"),
            ("booster_landing_ignition_ceiling_m", "positive"),
            ("booster_recovery_max_q_pa", "positive"),
            ("booster_recovery_max_q_alpha_pa_rad", "positive"),
            ("booster_terminal_attitude_error_max_deg", "nonneg"),
            ("booster_landing_propellant_margin_kg", "nonneg"),
        ))
        if self.tvc_lever_arm_m is not None and self.tvc_lever_arm_m <= 0:
            errors.append(f"tvc_lever_arm_m must be positive, got {self.tvc_lever_arm_m}")
        errors += _field_errors(self, (
            ("booster_pad_tolerance_m", "positive"),
            ("landing_target_wind_exposure_s", "nonneg"),
            ("landing_target_wind_gain", "nonneg"),
            ("landing_target_max_downrange_offset_km", "positive"),
            ("grid_fin_deploy_altitude_m", "nonneg"),
            ("grid_fin_max_deflection_deg", "positive"),
            ("grid_fin_area_m2", "positive"),
            ("grid_fin_drag_area_m2", "positive"),
            ("grid_fin_cl_max", "positive"),
            ("grid_fin_cd_increment", "nonneg"),
            ("landing_leg_deploy_altitude_m", "nonneg"),
            ("landing_leg_deploy_time_s", "positive"),
            ("landing_leg_max_touchdown_speed_mps", "positive"),
            ("landing_leg_max_tilt_deg", "nonneg"),
            ("landing_leg_footprint_radius_m", "positive"),
            ("booster_powered_descent_lead_time_s", "nonneg"),
        ))
        has_explicit_lat = self.booster_landing_site_lat_deg is not None
        has_explicit_lon = self.booster_landing_site_lon_deg is not None
        if has_explicit_lat != has_explicit_lon:
            errors.append(
                "booster_landing_site_lat_deg and booster_landing_site_lon_deg must either both be set or both be None"
            )
        if self.booster_landing_site_lat_deg is not None and not (-90.0 <= self.booster_landing_site_lat_deg <= 90.0):
            errors.append(
                f"booster_landing_site_lat_deg must be in [-90, 90], got {self.booster_landing_site_lat_deg}"
            )
        if self.booster_landing_site_lon_deg is not None and not (-180.0 <= self.booster_landing_site_lon_deg <= 180.0):
            errors.append(
                f"booster_landing_site_lon_deg must be in [-180, 180], got {self.booster_landing_site_lon_deg}"
            )
        if has_explicit_lat and abs(self.booster_landing_target_downrange_km) > 1e-10:
            errors.append(
                "booster_landing_target_downrange_km must be 0 when explicit landing-site coordinates are provided"
            )


        if self.dt <= 0:
            errors.append(f"dt must be positive, got {self.dt}")
        if self.demo_coast_max_dt <= 0:
            errors.append(
                f"demo_coast_max_dt must be positive, got {self.demo_coast_max_dt}"
            )
        elif self.demo_coast_max_dt < self.dt:
            errors.append(
                "demo_coast_max_dt must be >= dt "
                f"({self.demo_coast_max_dt} < {self.dt})"
            )
        if self.booster_landing_guidance not in ("heuristic", "gfold"):
            errors.append(
                "booster_landing_guidance must be 'heuristic' or 'gfold', "
                f"got {self.booster_landing_guidance!r}"
            )
        if self.gfold_replan_period_s <= 0:
            errors.append(f"gfold_replan_period_s must be positive, got {self.gfold_replan_period_s}")
        if not (5 <= int(self.gfold_nodes) <= 200):
            errors.append(f"gfold_nodes must be in [5, 200], got {self.gfold_nodes}")
        if self.gfold_terminal_altitude_m < 0:
            errors.append(f"gfold_terminal_altitude_m must be >= 0, got {self.gfold_terminal_altitude_m}")
        if not (0.0 < self.gfold_glideslope_deg < 90.0):
            errors.append(f"gfold_glideslope_deg must be in (0, 90), got {self.gfold_glideslope_deg}")
        if self.orbit_coast_max_dt <= 0:
            errors.append(f"orbit_coast_max_dt must be positive, got {self.orbit_coast_max_dt}")
        errors += _field_errors(self, (
            ("max_time", "positive"),
            ("separation_delta_v", "nonneg"),
            ("separation_tumble_rate", "nonneg"),
            ("q_alpha_max", "nonneg"),
            ("gps_position_sigma_m", "nonneg"),
            ("gps_velocity_sigma_mps", "nonneg"),
            ("gps_update_period_s", "positive"),
            ("imu_accel_bias", "nonneg"),
            ("imu_accel_noise", "nonneg"),
            ("imu_gyro_bias", "nonneg"),
            ("imu_gyro_noise", "nonneg"),
            ("landing_altimeter_max_altitude_m", "positive"),
            ("landing_altimeter_altitude_sigma_m", "nonneg"),
            ("landing_altimeter_velocity_sigma_mps", "nonneg"),
            ("abort_q_alpha_threshold", "nonneg"),
            ("abort_attitude_threshold", "nonneg"),
            ("runtime_thrust_scale", "positive"),
            ("runtime_isp_scale", "positive"),
        ))


        if C.INITIAL_MASS + self.runtime_initial_mass_offset_kg <= C.DRY_MASS + self.stage1_landing_fuel_reserve_kg:
            errors.append(
                "runtime_initial_mass_offset_kg leaves no ascent propellant above MECO mass"
            )

        if errors:
            raise ValueError("SimulationConfig validation failed:\n  " + "\n  ".join(errors))


__all__ = [
    'EXPERIMENTAL_FEATURE_FLAGS',
    'VALIDATED_RUNTIME_FIELDS',
    'AttitudeController',
    'RecoveryAttitudeController',
    'SimulationConfig',
]


# Transitional nested-profile views. The flat fields above remain the canonical
# constructor/API surface until callers migrate to these grouped objects.
@dataclass(frozen=True)
class VehicleProfile:
    stage1_dry_mass: float
    stage1_prop_mass: float
    stage2_dry_mass: float
    stage2_prop_mass: float
    payload_mass: float
    stage2_thrust_vac: float
    stage2_isp_vac: float


@dataclass(frozen=True)
class MissionProfile:
    launch_site_lat_deg: float
    launch_site_lon_deg: float
    launch_site_altitude_m: float
    orbit_target_altitude_m: float
    target_inclination_deg: float
    orbit_insertion_start_altitude_m: float
    stage_sep_velocity: float


@dataclass(frozen=True)
class RecoveryProfile:
    booster_landing_target_downrange_km: float
    booster_landing_site_lat_deg: float | None
    booster_landing_site_lon_deg: float | None
    booster_landing_site_altitude_m: float
    booster_pad_tolerance_m: float
    booster_enforce_pad_landing: bool
    enable_grid_fins: bool
    enable_landing_legs: bool
    enable_s2_recovery: bool


@dataclass(frozen=True)
class PhysicsProfile:
    enable_atmosphere: bool
    enable_high_fidelity_gram: bool
    enable_drag: bool
    enable_lift: bool
    enable_j2: bool
    gravity_model: str
    atmosphere_model: str
    enable_upper_atmosphere: bool
    enable_stochastic_wind: bool


@dataclass(frozen=True)
class RuntimeProfile:
    dt: float
    max_time: float
    verbose: bool
    enable_engine_transients: bool
    enable_rcs: bool
    enable_gps: bool
    enable_imu: bool
    enable_landing_altimeter: bool
    enable_abort_modes: bool
    runtime_thrust_scale: float
    runtime_isp_scale: float
    runtime_initial_mass_offset_kg: float
    runtime_wind_offset_mps: float


def _vehicle_profile(config: SimulationConfig) -> VehicleProfile:
    return VehicleProfile(
        stage1_dry_mass=config.stage1_dry_mass,
        stage1_prop_mass=config.stage1_prop_mass,
        stage2_dry_mass=config.stage2_dry_mass,
        stage2_prop_mass=config.stage2_prop_mass,
        payload_mass=config.payload_mass,
        stage2_thrust_vac=config.stage2_thrust_vac,
        stage2_isp_vac=config.stage2_isp_vac,
    )


def _mission_profile(config: SimulationConfig) -> MissionProfile:
    return MissionProfile(
        launch_site_lat_deg=config.launch_site_lat_deg,
        launch_site_lon_deg=config.launch_site_lon_deg,
        launch_site_altitude_m=config.launch_site_altitude_m,
        orbit_target_altitude_m=config.orbit_target_altitude_m,
        target_inclination_deg=config.target_inclination_deg,
        orbit_insertion_start_altitude_m=config.orbit_insertion_start_altitude_m,
        stage_sep_velocity=config.stage_sep_velocity,
    )


def _recovery_profile(config: SimulationConfig) -> RecoveryProfile:
    return RecoveryProfile(
        booster_landing_target_downrange_km=config.booster_landing_target_downrange_km,
        booster_landing_site_lat_deg=config.booster_landing_site_lat_deg,
        booster_landing_site_lon_deg=config.booster_landing_site_lon_deg,
        booster_landing_site_altitude_m=config.booster_landing_site_altitude_m,
        booster_pad_tolerance_m=config.booster_pad_tolerance_m,
        booster_enforce_pad_landing=config.booster_enforce_pad_landing,
        enable_grid_fins=config.enable_grid_fins,
        enable_landing_legs=config.enable_landing_legs,
        enable_s2_recovery=config.enable_s2_recovery,
    )


def _physics_profile(config: SimulationConfig) -> PhysicsProfile:
    return PhysicsProfile(
        enable_atmosphere=config.enable_atmosphere,
        enable_high_fidelity_gram=config.enable_high_fidelity_gram,
        enable_drag=config.enable_drag,
        enable_lift=config.enable_lift,
        enable_j2=config.enable_j2,
        gravity_model=config.gravity_model,
        atmosphere_model=config.atmosphere_model,
        enable_upper_atmosphere=config.enable_upper_atmosphere,
        enable_stochastic_wind=config.enable_stochastic_wind,
    )


def _runtime_profile(config: SimulationConfig) -> RuntimeProfile:
    return RuntimeProfile(
        dt=config.dt,
        max_time=config.max_time,
        verbose=config.verbose,
        enable_engine_transients=config.enable_engine_transients,
        enable_rcs=config.enable_rcs,
        enable_gps=config.enable_gps,
        enable_imu=config.enable_imu,
        enable_landing_altimeter=config.enable_landing_altimeter,
        enable_abort_modes=config.enable_abort_modes,
        runtime_thrust_scale=config.runtime_thrust_scale,
        runtime_isp_scale=config.runtime_isp_scale,
        runtime_initial_mass_offset_kg=config.runtime_initial_mass_offset_kg,
        runtime_wind_offset_mps=config.runtime_wind_offset_mps,
    )


SimulationConfig.vehicle = property(_vehicle_profile)
SimulationConfig.mission = property(_mission_profile)
SimulationConfig.recovery = property(_recovery_profile)
SimulationConfig.physics = property(_physics_profile)
SimulationConfig.runtime = property(_runtime_profile)


def _from_profiles(
    cls,
    *,
    vehicle: VehicleProfile | None = None,
    mission: MissionProfile | None = None,
    recovery: RecoveryProfile | None = None,
    physics: PhysicsProfile | None = None,
    runtime: RuntimeProfile | None = None,
    **overrides,
) -> SimulationConfig:
    """Build the legacy flat config from grouped profile objects.

    Explicit flat ``overrides`` win, which keeps CLI/API compatibility while
    allowing new callers to migrate one profile at a time.
    """
    values = {}
    if vehicle is not None:
        values.update({
            "stage1_dry_mass": vehicle.stage1_dry_mass,
            "stage1_prop_mass": vehicle.stage1_prop_mass,
            "stage2_dry_mass": vehicle.stage2_dry_mass,
            "stage2_prop_mass": vehicle.stage2_prop_mass,
            "payload_mass": vehicle.payload_mass,
            "stage2_thrust_vac": vehicle.stage2_thrust_vac,
            "stage2_isp_vac": vehicle.stage2_isp_vac,
        })
    if mission is not None:
        values.update({
            "launch_site_lat_deg": mission.launch_site_lat_deg,
            "launch_site_lon_deg": mission.launch_site_lon_deg,
            "launch_site_altitude_m": mission.launch_site_altitude_m,
            "orbit_target_altitude_m": mission.orbit_target_altitude_m,
            "target_inclination_deg": mission.target_inclination_deg,
            "orbit_insertion_start_altitude_m": mission.orbit_insertion_start_altitude_m,
            "stage_sep_velocity": mission.stage_sep_velocity,
        })
    if recovery is not None:
        values.update({
            "booster_landing_target_downrange_km": recovery.booster_landing_target_downrange_km,
            "booster_landing_site_lat_deg": recovery.booster_landing_site_lat_deg,
            "booster_landing_site_lon_deg": recovery.booster_landing_site_lon_deg,
            "booster_landing_site_altitude_m": recovery.booster_landing_site_altitude_m,
            "booster_pad_tolerance_m": recovery.booster_pad_tolerance_m,
            "booster_enforce_pad_landing": recovery.booster_enforce_pad_landing,
            "enable_grid_fins": recovery.enable_grid_fins,
            "enable_landing_legs": recovery.enable_landing_legs,
            "enable_s2_recovery": recovery.enable_s2_recovery,
        })
    if physics is not None:
        values.update({
            "enable_atmosphere": physics.enable_atmosphere,
            "enable_high_fidelity_gram": physics.enable_high_fidelity_gram,
            "enable_drag": physics.enable_drag,
            "enable_lift": physics.enable_lift,
            "enable_j2": physics.enable_j2,
            "gravity_model": physics.gravity_model,
            "atmosphere_model": physics.atmosphere_model,
            "enable_upper_atmosphere": physics.enable_upper_atmosphere,
            "enable_stochastic_wind": physics.enable_stochastic_wind,
        })
    if runtime is not None:
        values.update({
            "dt": runtime.dt,
            "max_time": runtime.max_time,
            "verbose": runtime.verbose,
            "enable_engine_transients": runtime.enable_engine_transients,
            "enable_rcs": runtime.enable_rcs,
            "enable_gps": runtime.enable_gps,
            "enable_imu": runtime.enable_imu,
            "enable_landing_altimeter": runtime.enable_landing_altimeter,
            "enable_abort_modes": runtime.enable_abort_modes,
            "runtime_thrust_scale": runtime.runtime_thrust_scale,
            "runtime_isp_scale": runtime.runtime_isp_scale,
            "runtime_initial_mass_offset_kg": runtime.runtime_initial_mass_offset_kg,
            "runtime_wind_offset_mps": runtime.runtime_wind_offset_mps,
        })
    values.update(overrides)
    return cls(**values)


SimulationConfig.from_profiles = classmethod(_from_profiles)
__all__.extend([
    "MissionProfile",
    "PhysicsProfile",
    "RecoveryProfile",
    "RuntimeProfile",
    "VehicleProfile",
])


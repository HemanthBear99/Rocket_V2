"""One-line help text per SimulationConfig field, shown in the app's settings panel."""

FIELD_DOCS: dict[str, str] = {
    "dt": (
        "Integration time step for the simulation loop. Smaller steps improve accuracy but increase "
        "run time."
    ),
    "max_time": (
        "Maximum simulated mission time before the run is stopped if no end condition is reached "
        "first."
    ),
    "enable_demo_mode": (
        "Enables the demo preset, which uses coarser time steps during coast phases for faster, less "
        "precise runs."
    ),
    "demo_coast_max_dt": (
        "Largest time step allowed during coast phases when demo mode is on."
    ),
    "orbit_coast_max_dt": (
        "Largest time step used on unpowered orbital coast arcs with the engine off, where a coarser "
        "step stays accurate."
    ),
    "kp_attitude": (
        "Proportional gain of the attitude controller; higher values correct attitude errors more "
        "aggressively."
    ),
    "kd_attitude": (
        "Derivative gain of the attitude controller; higher values damp rotation rate and reduce "
        "overshoot."
    ),
    "ki_attitude": (
        "Integral gain of the attitude controller; removes steady attitude error over time but can "
        "cause overshoot."
    ),
    "attitude_controller": (
        "Attitude control law used during ascent: 'pd' for proportional-derivative or 'pid' with "
        "integral action."
    ),
    "recovery_attitude_controller": (
        "Attitude control law used during recovery: 'pd', 'pid', or 'auto'."
    ),
    "attitude_integral_windup_fraction": (
        "Caps how far the attitude integrator can accumulate, as a fraction of available control "
        "authority."
    ),
    "max_torque": (
        "Maximum attitude control torque the stack can apply; the booster's torque limit scales from "
        "it."
    ),
    "max_gimbal_angle_deg": (
        "Largest engine gimbal deflection thrust vector control may command, limiting steering "
        "authority."
    ),
    "gravity_turn_start_altitude": (
        "Altitude at which ascent guidance begins blending from the pitch program into a gravity "
        "turn."
    ),
    "gravity_turn_transition_range": (
        "Altitude span over which the gravity-turn blend ramps in after its start; larger values make "
        "it gentler."
    ),
    "min_velocity_for_turn": (
        "Speed below which ascent guidance keeps the vehicle vertical rather than starting the turn."
    ),
    "pitchover_start_altitude": (
        "Altitude at which the pitchover maneuver begins tilting the vehicle off vertical."
    ),
    "pitchover_end_altitude": (
        "Altitude at which the pitchover maneuver completes and the pitch angle reaches its target."
    ),
    "pitchover_angle": (
        "Tilt from vertical that the vehicle is commanded to reach during the pitchover maneuver."
    ),
    "ascent_max_q_target_pa": (
        "Dynamic pressure the ascent throttle law holds during max-q, keeping structural loads below "
        "the limit."
    ),
    "ascent_throttle_bucket_start_s": (
        "Mission time at which a scheduled throttle-down through max-q begins; disabled while end is "
        "not after start."
    ),
    "ascent_throttle_bucket_end_s": (
        "Mission time at which the scheduled max-q throttle-down ends and full throttle is allowed "
        "again."
    ),
    "ascent_throttle_bucket_fraction": (
        "Maximum engine throttle allowed during the scheduled max-q throttle-down window."
    ),
    "stage1_landing_fuel_reserve_kg": (
        "Booster propellant held back for the landing burn so the booster has enough fuel to land."
    ),
    "auto_size_recovery_reserve": (
        "Enables sizing the booster landing propellant reserve from the vehicle's performance instead "
        "of the fixed value."
    ),
    "recovery_reserve_wind_sensitivity_kg_per_mps": (
        "Extra booster propellant reserved for each unit of forecast headwind; zero disables the wind "
        "adjustment."
    ),
    "launch_site_lat_deg": (
        "Geodetic latitude of the launch site, which sets the starting position on the Earth model."
    ),
    "launch_site_lon_deg": (
        "Geodetic longitude of the launch site, which sets the starting position on the Earth model."
    ),
    "launch_site_altitude_m": (
        "Height of the launch pad above the reference Earth surface at the launch site."
    ),
    "orbit_target_altitude_m": (
        "Target circular orbit altitude that orbit insertion aims to reach."
    ),
    "orbit_altitude_tolerance_m": (
        "Allowed deviation from the target orbit altitude for the insertion to count as successful."
    ),
    "orbit_ecc_max": (
        "Maximum orbital eccentricity accepted at insertion; lower values require a more circular "
        "orbit."
    ),
    "orbit_insertion_start_altitude_m": (
        "Altitude above which the orbit insertion phase is allowed to start."
    ),
    "orbit_insertion_timeout_s": (
        "Maximum time the orbit insertion phase may run before it is declared failed."
    ),
    "enable_s2_recovery": (
        "Enables recovery of the upper stage after orbit insertion, including deorbit and landing."
    ),
    "s2_orbit_hold_time_s": (
        "Time the upper stage holds its orbit before starting the deorbit sequence."
    ),
    "s2_entry_interface_altitude_m": (
        "Altitude at which the upper stage counts as entering the atmosphere during deorbit."
    ),
    "s2_deorbit_target_perigee_m": (
        "Target perigee altitude for the upper stage deorbit burn; negative values put perigee below "
        "the surface."
    ),
    "s2_entry_drag_scale": (
        "Multiplier on atmospheric drag during upper stage re-entry; higher values slow the stage "
        "more."
    ),
    "s2_touchdown_speed_limit_mps": (
        "Maximum surface-relative speed at which the upper stage touchdown counts as a successful "
        "landing."
    ),
    "s2_landing_propellant_reserve_kg": (
        "Propellant kept aside from the orbit insertion and deorbit burns so the upper stage retains "
        "fuel to land."
    ),
    "stage1_dry_mass": (
        "Mass of the booster structure and engines with no propellant loaded."
    ),
    "stage1_prop_mass": (
        "Propellant mass loaded in the booster at liftoff."
    ),
    "stage2_dry_mass": (
        "Mass of the upper stage structure and engines with no propellant loaded."
    ),
    "stage2_prop_mass": (
        "Propellant mass loaded in the upper stage at separation."
    ),
    "payload_mass": (
        "Mass of the payload carried to orbit by the upper stage."
    ),
    "stage2_thrust_vac": (
        "Upper stage engine thrust in vacuum, which sets its acceleration during burns."
    ),
    "stage1_engine_count": (
        "Number of booster engines; total thrust for every burn and the reference area are derived "
        "from this count."
    ),
    "stage1_engine_thrust_n": (
        "Sea-level thrust of a single booster engine; with the engine count it sets booster thrust."
    ),
    "boostback_engine_count": (
        "Number of engines fired during the booster boostback burn."
    ),
    "entry_engine_count": (
        "Number of engines fired during the booster re-entry burn."
    ),
    "landing_engine_count": (
        "Number of engines fired during the booster landing burn."
    ),
    "vehicle_diameter_m": (
        "Outer diameter of the vehicle, which sets the reference area for drag and lift."
    ),
    "stage2_isp_vac": (
        "Vacuum specific impulse of the upper stage engine, which sets how much velocity each unit of "
        "propellant gives."
    ),
    "stage_sep_velocity": (
        "Speed of the stack at the moment the booster and upper stage separate."
    ),
    "target_inclination_deg": (
        "Inclination of the target orbit relative to the equator, which sets the orbital plane to "
        "reach."
    ),
    "booster_boostback_budget_kg": (
        "Booster propellant allowed for the boostback burn; the burn ends once this much has been "
        "used."
    ),
    "booster_boostback_impact_corridor_m": (
        "Acceptable miss distance of the booster's predicted impact point from its landing corridor "
        "during boostback."
    ),
    "booster_entry_budget_kg": (
        "Booster propellant allowed for the re-entry burn."
    ),
    "booster_landing_reserve_kg": (
        "Booster propellant kept aside for the landing burn and held back from earlier burns."
    ),
    "booster_flip_min_time_s": (
        "Minimum time the booster flip maneuver is allowed to take, so the turnaround is not "
        "commanded faster."
    ),
    "booster_apogee_target_km": (
        "Target apogee altitude of the booster after boostback."
    ),
    "booster_entry_interface_altitude_m": (
        "Altitude at which the booster is considered to enter the atmosphere during re-entry."
    ),
    "booster_entry_burn_min_altitude_m": (
        "Lowest altitude at which the booster entry burn is allowed to ignite."
    ),
    "booster_entry_burn_min_speed_mps": (
        "Minimum booster speed required before the entry burn is allowed to start."
    ),
    "booster_entry_target_descent_rate_mps": (
        "Descent rate that the booster entry capture logic brakes toward during re-entry."
    ),
    "booster_entry_drag_credit": (
        "Enables crediting aerodynamic drag against the entry capture's braking demand, reducing "
        "propellant use."
    ),
    "booster_terminal_divert_time_constant_s": (
        "Shortest time constant for nulling the booster's horizontal velocity during the landing "
        "burn; smaller corrects faster."
    ),
    "booster_terminal_position_hold_tau_s": (
        "Time constant for holding the booster over the pad on final approach; zero uses velocity "
        "nulling only."
    ),
    "booster_terminal_position_hold_floor_m": (
        "Altitude below which the final-approach position hold fades out, leaving only velocity "
        "nulling near touchdown."
    ),
    "booster_entry_divert_tilt_deg": (
        "Maximum tilt from vertical the booster entry capture may use to steer toward the landing "
        "site."
    ),
    "booster_landing_ignition_safety_factor": (
        "Multiplier on the estimated landing burn start altitude; higher values start the burn "
        "earlier for more margin."
    ),
    "booster_landing_planning_throttle": (
        "Fraction of full landing thrust assumed when planning ignition; below one starts the burn "
        "earlier and keeps throttle in reserve."
    ),
    "booster_landing_ignition_ceiling_m": (
        "Highest altitude at which the landing burn may be ignited."
    ),
    "booster_recovery_max_q_pa": (
        "Dynamic pressure limit for booster recovery; peaks above it are flagged in the results."
    ),
    "booster_recovery_max_q_alpha_pa_rad": (
        "Limit on the dynamic pressure and angle-of-attack product during booster recovery; peaks "
        "above it are flagged."
    ),
    "booster_terminal_attitude_error_max_deg": (
        "Maximum attitude error allowed at terminal descent; larger errors are flagged in the "
        "results."
    ),
    "booster_landing_propellant_margin_kg": (
        "Extra propellant held above the landing reserve as a safety buffer for the landing burn."
    ),
    "booster_landing_vertical_propellant_safety_factor": (
        "Multiplier on the propellant needed for the vertical landing burn, adding margin for "
        "uncertainty."
    ),
    "booster_terminal_capture_landing_reserve_fraction": (
        "Fraction of the booster landing reserve the terminal capture logic may count as usable."
    ),
    "booster_tail_first_cp_z_m": (
        "Centre-of-pressure position used for the booster's tail-first aerodynamics, which sets its "
        "stability."
    ),
    "booster_landing_target_downrange_km": (
        "Downrange distance of the booster landing target from the launch site."
    ),
    "enable_recovery_planner_diagnostics": (
        "Enables an offline recovery planner that logs extra diagnostic telemetry without steering "
        "the vehicle; slows runs noticeably."
    ),
    "booster_landing_guidance": (
        "Booster landing-burn law: 'heuristic' for a latched burn with divert, 'gfold' for a convex- "
        "optimized burn, or 'plugin'."
    ),
    "booster_landing_plugin": (
        "Name of a user plugin file that supplies landing guidance when landing guidance is set to "
        "plugin."
    ),
    "gfold_replan_period_s": (
        "How often the G-FOLD landing solver recomputes its trajectory during the landing burn."
    ),
    "gfold_nodes": (
        "Number of discrete time points the G-FOLD solver uses across the landing trajectory; more "
        "nodes cost more compute."
    ),
    "gfold_glideslope_deg": (
        "Maximum angle from vertical allowed on the G-FOLD glide slope toward the landing pad."
    ),
    "gfold_terminal_altitude_m": (
        "Height above the pad where G-FOLD hands control to the vertical descent law."
    ),
    "booster_landing_site_lat_deg": (
        "Latitude of a custom booster landing site; leave empty to use the default landing target."
    ),
    "booster_landing_site_lon_deg": (
        "Longitude of a custom booster landing site; leave empty to use the default landing target."
    ),
    "booster_landing_site_altitude_m": (
        "Ground altitude of the custom booster landing site."
    ),
    "tvc_lever_arm_m": (
        "Distance from the engine gimbal plane to the vehicle centre of mass, which sets thrust- "
        "vector torque. Empty keeps the default."
    ),
    "booster_pad_tolerance_m": (
        "Radius around the landing pad within which a booster landing counts as on target."
    ),
    "booster_enforce_pad_landing": (
        "If on, a touchdown outside the pad tolerance ends the mission as an off-site landing failure."
    ),
    "enable_movable_landing_target": (
        "Enables a landing target that drifts downrange with wind, like a towed drone ship, instead "
        "of a fixed point."
    ),
    "landing_target_wind_exposure_s": (
        "Effective time over which wind pushes the movable landing target; sets how far wind offsets "
        "it."
    ),
    "landing_target_wind_gain": (
        "Scale factor on how strongly wind moves the movable landing target."
    ),
    "landing_target_max_downrange_offset_km": (
        "Largest downrange shift the movable landing target can take due to wind."
    ),
    "enable_grid_fins": (
        "Enables booster grid fins, which steer the booster during atmospheric descent."
    ),
    "enable_landing_legs": (
        "Enables booster landing legs, which deploy before touchdown."
    ),
    "grid_fin_deploy_altitude_m": (
        "Altitude below which the grid fins deploy during booster descent."
    ),
    "grid_fin_max_deflection_deg": (
        "Largest grid fin deflection the steering controller may command."
    ),
    "grid_fin_area_m2": (
        "Lifting area of the grid fins, which sets how much steering force they can produce."
    ),
    "grid_fin_drag_area_m2": (
        "Effective drag area the grid fins add during descent."
    ),
    "grid_fin_cl_max": (
        "Maximum lift coefficient of the grid fins, which sets their steering force at full "
        "deflection."
    ),
    "grid_fin_cd_increment": (
        "Extra drag coefficient the grid fins add when deployed and deflected."
    ),
    "landing_leg_deploy_altitude_m": (
        "Altitude below which the landing legs are deployed."
    ),
    "landing_leg_deploy_time_s": (
        "Time the landing legs take to fully deploy once commanded."
    ),
    "landing_leg_max_touchdown_speed_mps": (
        "Highest touchdown speed the landing legs are rated to survive; faster landings fail the "
        "pass/fail check."
    ),
    "landing_leg_max_tilt_deg": (
        "Largest tilt from vertical at touchdown that the landing legs can tolerate."
    ),
    "booster_powered_descent_lead_time_s": (
        "Time before the predicted landing-burn point at which powered descent is started."
    ),
    "booster_terminal_capture_floor_kg": (
        "Minimum propellant the terminal capture logic keeps in reserve during the landing approach."
    ),
    "booster_late_entry_bias_ref_altitude_m": (
        "Altitude below which the near-pad return applies its late-entry bias."
    ),
    "booster_max_divert_accel_far_mps2": (
        "Maximum lateral acceleration used to divert toward the pad when the booster is far from the "
        "target."
    ),
    "booster_max_divert_accel_near_mps2": (
        "Maximum lateral acceleration used to divert toward the pad when the booster is close to the "
        "target."
    ),
    "booster_divert_accel_miss_threshold_m": (
        "Predicted miss distance that switches the divert acceleration limit between the far and near "
        "values."
    ),
    "booster_divert_fraction_high": (
        "Fraction of available divert effort allowed in the high altitude band."
    ),
    "booster_divert_fraction_mid": (
        "Fraction of available divert effort allowed in the middle altitude band."
    ),
    "booster_divert_fraction_low": (
        "Fraction of available divert effort allowed in the low altitude band."
    ),
    "booster_divert_band_high_alt_m": (
        "Altitude above which the high divert fraction applies."
    ),
    "booster_divert_band_low_alt_m": (
        "Altitude below which the low divert fraction applies."
    ),
    "booster_vertical_load_risk_ratio": (
        "Vertical load ratio above which the terminal landing logic treats the descent as a load "
        "risk."
    ),
    "booster_terminal_descent_overspeed_factor": (
        "Multiple of the target descent rate above which terminal descent counts as overspeed."
    ),
    "booster_fuel_critical_ratio": (
        "Fraction of remaining landing fuel below which the booster is treated as fuel-critical."
    ),
    "booster_terminal_hcapture_tstop_min_s": (
        "Shortest stop time allowed for the terminal horizontal capture maneuver."
    ),
    "booster_terminal_hcapture_tstop_max_s": (
        "Longest stop time allowed for the terminal horizontal capture maneuver."
    ),
    "enable_atmosphere": (
        "Enables the atmosphere model so drag, lift and density effects are included; off gives "
        "vacuum flight."
    ),
    "enable_high_fidelity_gram": (
        "Enables a smooth analytic density modifier on top of US-76 (GRAM-like, not the NASA GRAM "
        "model)."
    ),
    "enable_drag": (
        "Enables aerodynamic drag on the vehicle."
    ),
    "enable_lift": (
        "Enables aerodynamic lift on the vehicle."
    ),
    "runtime_atmosphere_density_scale": (
        "Multiplier on atmospheric density applied at run time; 1.0 is nominal."
    ),
    "enable_j2": (
        "Enables the Earth oblateness (J2) term in gravity, which slightly perturbs orbits."
    ),
    "j2_coefficient": (
        "Earth oblateness coefficient used when the J2 gravity term is enabled."
    ),
    "gravity_model": (
        "Gravity model: 'central' for point mass, 'j2' for point mass with oblateness, or 'egm96' for "
        "the full Earth model."
    ),
    "enable_engine_transients": (
        "Enables engine spool-up and spool-down delays, so thrust ramps rather than switching "
        "instantly."
    ),
    "engine_spool_up_time": (
        "Time for engine thrust to ramp up to full after ignition."
    ),
    "engine_spool_down_time": (
        "Time for engine thrust to ramp down to zero after shutdown."
    ),
    "abort_on_q_alpha_limit": (
        "If on, the run aborts when the dynamic pressure and angle-of-attack product exceeds its "
        "limit."
    ),
    "q_alpha_max": (
        "Limit on the dynamic pressure and angle-of-attack product; exceeding it aborts when the "
        "check is on."
    ),
    "enable_separation_dynamics": (
        "Enables simulated stage separation dynamics, including the separation velocity change and "
        "tumble."
    ),
    "separation_delta_v": (
        "Velocity change the booster receives at separation, pushing it away from the upper stage."
    ),
    "separation_tumble_rate": (
        "Rotation rate the booster picks up at separation, causing it to tumble until controlled."
    ),
    "atmosphere_model": (
        "Atmosphere model name; the default is a deterministic US Standard Atmosphere 1976 model."
    ),
    "enable_upper_atmosphere": (
        "Enables the upper-atmosphere density model used for high-altitude drag and abort checks."
    ),
    "enable_stochastic_wind": (
        "Enables random wind turbulence and occasional gusts, drawn from seeded random numbers."
    ),
    "wind_turbulence_intensity": (
        "Standard deviation of random wind turbulence when stochastic wind is enabled."
    ),
    "wind_gust_magnitude": (
        "Size of the occasional wind gusts applied when stochastic wind is enabled."
    ),
    "enable_rcs": (
        "Enables the reaction control system thrusters used for fine attitude control."
    ),
    "rcs_thrust": (
        "Thrust of each reaction control thruster."
    ),
    "rcs_isp": (
        "Specific impulse of the reaction control thrusters, which sets how efficiently their "
        "propellant is used."
    ),
    "rcs_num_thrusters": (
        "Number of reaction control thrusters available for attitude control."
    ),
    "rcs_propellant_mass": (
        "Propellant mass loaded in the reaction control system."
    ),
    "imu_accel_bias": (
        "Constant acceleration bias of the inertial measurement unit, which offsets navigation "
        "estimates."
    ),
    "imu_accel_noise": (
        "Random noise on the inertial measurement unit's acceleration readings."
    ),
    "enable_gps": (
        "Enables simulated GPS measurements that feed the navigation estimate."
    ),
    "gps_position_sigma_m": (
        "Standard deviation of the simulated GPS position error."
    ),
    "gps_velocity_sigma_mps": (
        "Standard deviation of the simulated GPS velocity error."
    ),
    "gps_update_period_s": (
        "Time between simulated GPS measurements."
    ),
    "gps_seed": (
        "Random seed for the simulated GPS error; the same seed reproduces the same errors."
    ),
    "wind_seed": (
        "Random seed for stochastic wind; empty derives it from the GPS seed."
    ),
    "enable_imu": (
        "Enables a simulated inertial measurement unit whose noisy readings feed navigation."
    ),
    "enable_landing_altimeter": (
        "Enables a simulated landing radar altimeter that gives height and vertical speed near the "
        "ground."
    ),
    "landing_altimeter_max_altitude_m": (
        "Highest altitude at which the landing altimeter returns readings."
    ),
    "landing_altimeter_altitude_sigma_m": (
        "Standard deviation of the landing altimeter's height error."
    ),
    "landing_altimeter_velocity_sigma_mps": (
        "Standard deviation of the landing altimeter's vertical speed error."
    ),
    "use_navigation_estimate_for_guidance": (
        "If on, guidance uses the navigation estimate of the state rather than the true simulated "
        "state."
    ),
    "enable_abort_modes": (
        "Enables abort logic that stops the mission when an abort threshold is crossed."
    ),
    "abort_q_alpha_threshold": (
        "Dynamic pressure and angle-of-attack product that triggers an abort when abort modes are on."
    ),
    "abort_attitude_threshold": (
        "Attitude error that triggers an abort when abort modes are on."
    ),
    "quaternion_norm_tolerance": (
        "Allowed deviation of the attitude quaternion's length from one before it is renormalized."
    ),
    "enable_run_manifest": (
        "Enables writing a run manifest file that records the configuration and outputs of each run."
    ),
    "min_engine_throttle_fraction": (
        "Lowest throttle setting the engines can run at, as a fraction of full thrust."
    ),
    "max_engine_throttle_fraction": (
        "Highest throttle setting the engines can run at, as a fraction of full thrust."
    ),
    "runtime_thrust_scale": (
        "Multiplier on engine thrust applied at run time, for dispersion and sensitivity studies."
    ),
    "runtime_isp_scale": (
        "Multiplier on engine specific impulse applied at run time, for dispersion studies."
    ),
    "runtime_initial_mass_offset_kg": (
        "Offset added to the initial vehicle mass at run time, for mass dispersion studies."
    ),
    "runtime_wind_offset_mps": (
        "Constant wind speed offset added to the mean wind; a negative value models a headwind."
    ),
    "runtime_wind_profile_modes": (
        "Altitude-dependent wind perturbation modes added to the mean wind profile; empty uses the "
        "mean only."
    ),
    "booster_descent_drag_scale": (
        "Multiplier on booster drag during re-entry and landing, fitted to flight data."
    ),
    "stage1_engine_ring_radius_m": (
        "Radius of the booster engine ring, used with nozzle geometry to model jet damping of "
        "rotation."
    ),
    "stage1_nozzle_exit_radius_m": (
        "Exit radius of a booster engine nozzle, used to model jet damping of rotation."
    ),
    "stage2_nozzle_exit_radius_m": (
        "Exit radius of an upper stage engine nozzle, used to model jet damping of rotation."
    ),
    "stack_ixx_full_kgm2": (
        "Moment of inertia about the x axis of the full stack, with propellant loaded."
    ),
    "stack_ixx_empty_kgm2": (
        "Moment of inertia about the x axis of the stack once its propellant is spent."
    ),
    "stack_izz_full_kgm2": (
        "Moment of inertia about the z axis of the full stack, with propellant loaded."
    ),
    "stack_izz_empty_kgm2": (
        "Moment of inertia about the z axis of the stack once its propellant is spent."
    ),
    "booster_cg_full_m": (
        "Distance from the booster base to its centre of mass with the full recovery reserve on board."
    ),
    "booster_cg_empty_m": (
        "Distance from the booster base to its centre of mass once the recovery reserve is spent."
    ),
    "booster_ixx_full_kgm2": (
        "Moment of inertia about the x axis of the booster with propellant loaded."
    ),
    "booster_ixx_empty_kgm2": (
        "Moment of inertia about the x axis of the booster once its propellant is spent."
    ),
    "booster_izz_full_kgm2": (
        "Moment of inertia about the z axis of the booster with propellant loaded."
    ),
    "booster_izz_empty_kgm2": (
        "Moment of inertia about the z axis of the booster once its propellant is spent."
    ),
}

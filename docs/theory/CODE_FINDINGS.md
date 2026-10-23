# Code findings from the theory-manual review

Found by reading the code while writing the manual (2026-10-23). These are not fixed unless marked. Line numbers refer to that date.

## Highest impact first
1. **Gravity coefficients.** The table labelled EGM96 differs from published values for C22, S22, S32 and S33; S32 and S33 have the opposite sign, and C53/C62 are missing. Also, the longitude-dependent terms don't rotate with the Earth. (Ch3, Ch1)
2. **Booster jet damping always used 9 engines.** The phase names didn't match (`dynamics.py`). Fixed in the working tree, not yet tested or committed. (Ch2, Ch4, Ch5)
3. **Impact-time interpolation is off.** `recovery.py:436` and `numba_kernels.py:258` compute the time as `elapsed - dt*(1-f)` instead of `elapsed - f*dt`. That is up to ±2 s, or about 0.8 km of pad motion in boostback targeting. (Ch7, Ch2)
4. **Grid-fin drag is counted twice**, with two disagreeing area fields (16 vs 8 m²). This is likely most of the ~2x excess descent drag that the 0.68 fit compensates for. (Ch4)
5. **Landing success flags disagree.** A landing below the propellant margin is reported as "LANDING SUCCESS" while the assessment fails it. The contact status is never logged, and turning landing legs off always fails the assessment. (Ch1, Ch8, Ch10)
6. **Settings that do nothing:**
   - `max_gimbal_angle_deg`, `target_inclination_deg`, `stage_sep_velocity` and `atmosphere_model` are never read.
   - `enable_high_fidelity_gram` never takes effect.
   - `min_engine_throttle_fraction` is not applied on the booster path. (Ch3, Ch5, Ch6, Ch8, Ch9)
7. **Mass properties ignore the configured vehicle.** Stacked and orbiter CG and inertia use constant masses, payload is counted as propellant, and the orbiter uses the stack's CP and its CG lever arm, making S2 jet damping 10–30x too strong. (Ch2, Ch5)
8. **Pitching-moment sign.** For imported aero decks it is reversed from aircraft convention, so normal CFD data would be applied as destabilising. (Ch4)
9. **Separation tumble** is added in the wrong frame and is not momentum-conserving. (Ch1, Ch6)
10. **No skin-temperature model**, though the docs and the model card claim one. (Ch4)

## By chapter (raw notes)

## Ch11 Monte Carlo & validation
- gps_noise dispersion inert by default (enable_gps=false, use_navigation_estimate_for_guidance=false; navigation.py:73).
- _run_case calls apply_case outside try (campaign.py:330): a dispersed config failing validate() aborts the whole campaign.
- drag_fit.py DOCS = ROOT.parent/"docs" -> writes VALIDATION_DRAG.md into backend/rlv_sim/docs (flight_compare fixed to parents[2]).
- Stale numbers: VALIDATION_DRAG.md (94%->86%, old module path rlv_sim.campaign); VALIDATION.md (78%->59% entry drag credit).
- sim_drag_area computes drag/q/mass*mass (no-op).
- MECO sampling asymmetric (sim searchsorted-1, flight searchsorted); sim speed air-relative vs flight surface-relative.
- Mass sigma = 0.5% of C.INITIAL_MASS (544.6 t, no payload), not configured mass.
- Guidance uses true drag/density/wind; wind reserve assumes perfect forecast of dispersed wind.
- Manifest credits booster descent drag pedigree 2 although default run uses 1.0.
- 3-sigma sweep rows carry mode="sensitivity"; extended wind-mode sweeps use phase 0.
- p0.135/p99.865 at n=100 are effectively sample min/max.
## Ch10 Navigation, aborts, assessment
- No nav filter: GPS fix overwrites estimate; IMU dead-reckoning between fixes; no gyro (attitude control uses truth).
- Onboard nav mode replaces only r, v, t inputs to guidance; everything else uses truth.
- Aborts only terminate the run (no abort trajectory); body-rate 1 rad/s threshold not configurable; abort monitor skipped in booster recovery.
- enable_landing_legs=False -> assessment always reports leg_deployment_failure (summary.py:415).
- touchdown_contact_status always logged '' (step.py:226) -> contact overlay summary.py:422-430 never runs; tipover can show landing.success=True.
- Short propellant margin: termination reason still "LANDING SUCCESS ... but" -> booster_landing_success True while assessment fails.
- booster_enforce_pad_landing only changes the wording.
- Orbiter and booster GPS receivers share gps_seed -> identical noise after separation.
- GPS without IMU freezes estimate between fixes (~1.5 km stale at orbital speed).
- IMU specific force from differencing truth with point-mass gravity -> J2/EGM96 never contribute error; one-step lag.
- Altimeter references spherical Earth radius, not configured surface.
- q_alpha_max 50 kPa·rad hard limit < 150 kPa·rad recovery gate.
- Propellant margin default 350 kg (dataclass) vs 750 kg (factory).
- No tests for navigation.py, abort.py, --nav-mode onboard, abort flags.
- Ascent max-q gate 35 kPa is a constant, not config.
## Ch3 Environment
- enable_high_fidelity_gram is dead code: compute_configured_atmosphere_properties applies latitude factor only when r passed; no caller passes r. Even active it's a +-4% factor, not NASA GRAM.
- EGM96 coefficient table doesn't match published EGM96: C22,S22,S32,S33 differ (S32,S33 sign flipped); C53,C62 missing; S53=0; C22 looks EGM2008. Regenerate from egm96_to360.ascii (agent's values from web snippets; confirm against official file).
- Tesseral gravity terms don't rotate with Earth (longitude from ECI, no omega*t).
- "Headwind" label: wind blows east; negative offset = headwind on ascent; on westward RTLS return positive offset is geometric headwind. Label matches MC-measured fuel effect, not return-leg geometry.
- Numba impact predictor ignores runtime_wind_profile_modes; Python fallback includes them.
- Stochastic wind: per-step white noise (dt-dependent spectrum), one-step gusts, only along mean wind axis; guidance sees it.
- US-76 evaluated at geometric not geopotential altitude (~1% density error stratosphere).
- Density scale also scales ambient pressure -> changes thrust back-pressure.
- single_vehicle.py:343 enable_upper_atm=True vs config default False.
- atmosphere_model label unused; mean wind uncapped with altitude.
- enable_j2=True has no effect unless gravity_model=central.
- Altitude above 6371 km sphere; launch latitude treated as geocentric.
## Ch9 Attitude control
- Roll never commanded, only rate-damped; q_commanded only logged.
- control.py:233-235 docstring wn~1.49, zeta~0.50 wrong (error is sin(theta/2)); actual wn 1.054, zeta 0.705 (booster 0.943, 0.631).
- max_gimbal_angle_deg is never read by physics (uses C.MAX_GIMBAL_ANGLE) though validated & exposed in UI.
- vehicle/actuator.py docstring says second-order gimbal; actually first-order 25 deg/s rate limit on attitude reference direction; real gimbal follows torque instantly.
- S2 recovery phases: torque limit fixed at booster 1.2e7 N·m, allocation skipped (S2 real TVC ~2.95e6).
- TVC lever arms constant 32/12/21 m, don't follow CG; forces.py:759-766 comment & MATH_REF §6 conflict ("a few metres", TVC roll authority).
- RCS lever arm 20 m all vehicles; RCS authority full until propellant gone; RCS propellant not deducted; S2 torque limit hard-coded 2e6 (step.py:627).
- Controller uses true q, omega.
- Untested: _allocate_attitude_torque, _available_attitude_torque_limit, _stage_max_torque, update_actuator, _gimbaled_thrust_body_vector, RCS propellant model.
- Booster divert tau 1.5 s only ~1.4x slower than attitude loop.
## Ch8 Landing guidance
- min_engine_throttle_fraction (0.35) never applied on booster heuristic/plugin path; only G-FOLD uses it. plugins.py docstring claims throttle range enforced.
- Unreachable-pad branch recomputed budget (booster.py:1054-1058) omits reserve authority alpha_r.
- Horizontal capture: alpha_r applied twice (booster.py:1041 and 1050).
- Tilt cone (tan15=0.27) is the real lateral limit vs divert fractions up to 0.72; _limit_direction_cone keeps throttle -> excess becomes vertical braking (~+19%).
- Engine cut whenever descent and horizontal speed both <1 m/s at any altitude (booster.py:1092-1094).
- Heuristic/plugin use landing thrust without runtime_thrust_scale; G-FOLD and plant use scaled.
- Velocity-nulling gain 2/t_go (2x min-energy gain), unexplained.
- gfold_glideslope_deg doc says max angle from vertical; code uses min elevation above horizontal (5 deg very loose).
- _booster_landing_success (step.py:85-93) ignores propellant margin; assess_full_mission fails it.
- Two tilt limits: contact uses landing_leg_max_tilt_deg, summary uses booster_terminal_attitude_error_max_deg.
- estimate_suicide_burn uses standard atmosphere and C.STAGE1_DRY_MASS, not configured ones.
- Factory vs dataclass defaults differ (ignition safety factor 1.8 vs 1.52; ceiling 8000 vs 20000 m).
## Ch5 Propulsion/mass/actuators
- FIXED: _active_engine_count compared "BOOSTBACK"/"ENTRY"/"LANDING" but phase names are BOOSTER_*; booster jet damping always used 9 engines.
- Stack/orbiter mass properties use constants masses, not config; payload counted as propellant (S1 prop overstated 8 t; orbiter never reaches empty).
- Orbiter CG 58-63 m vs stack S2 stations 56.1 m -> CG jump at separation.
- RCS propellant not deducted; each vehicle starts fresh 300 kg.
- Throttle-efficiency factor (0.96+0.04*delta) and S2 atmospheric penalty reduce thrust not mdot -> effective Isp drop.
- Thrust zero below 1% throttle but propellant still flows (forces.py:867).
- min_engine_throttle_fraction not enforced in propulsion path; booster entry capture commands 4-5%.
- Spool limit tracks throttle value not on/off flag: thrust-off with nonzero throttle -> next ignition without spool-up.
- TVC torque authority uses rated thrust; gimbal force uses actual thrust (~10% vacuum difference).
- One tvc_lever_arm_m override applies to all stages; lever arm fixed.
- validate() mass-offset check uses C.INITIAL_MASS/C.DRY_MASS (definition.py:681).
- Thrust model ignores GRAM modifier; density scale changes ambient pressure -> S1 thrust.
- No unit tests: spool transient, actuator, RCS propellant, gimbal deflection, S1 pressure compensation.
## Ch4 Aerodynamics
- No skin-temperature model: thermal_* telemetry columns never set (300 K/0/0); model_card and MATH_REF §7 claim a lumped first-order skin temperature. Heating telemetry-only.
- Deck pitching-moment sign reversed vs aircraft convention: positive deck Cm restores; imported CFD deck with normal stability applied as destabilising. Comment forces.py:980 contradicts math. Geometry deck Cm=-0.5CL destabilising in sim terms but validation calls it "static stability".
- Lift uses CL_ALPHA_VALUES (2.0-0.4/rad, cap 1.2); moment uses C_N_ALPHA=4/rad cap 1.6.
- CP always 33 m for every vehicle (dynamics.py:288); booster tail-first sin^2 blend no-op -> CP 31 m.
- Grid-fin drag double counted (16 m^2 x Cd1.2 in recovery drag scale + Cd increment 0.08 on 8 m^2); area fields disagree 16 vs 8. Likely explains much of the ~2x excess descent drag in VALIDATION_DRAG.md.
- Grid-fin force ignores attitude & q; ideal lateral force, no fin moment; instant deploy below 70 km.
- Recovery drag/moment scales from phase name, not hardware state (ignore enable_grid_fins, enable_landing_legs, deploy altitude); fins half deployed in coast; 1.3 plume factor whole entry; moment multiplier up to 4.2x also scales damping.
- Deck fin-angle lookup double degrees conversion (forces.py:1173), harmless now. Drag lookup alpha ignores wind, lift/moment include it.
- Landing-leg FAILED status never assigned.
- Geometry deck = body-only empirical buildup (not Barrowman as mentioned); not default.
- No unit tests for drag/lift/moment, post-stall, recovery scale, grid fins, leg state, heating.
## Ch1 Architecture
- Orbiter/booster clocks drift when dt>0.05 (booster capped 0.05 s); display shows different times.
- separation_time recorded MECO+3 s but stack flies 2 s more before split.
- Hard-landing limits differ: contact 12 m/s vs summary 10 m/s (comment says they match).
- Separation tumble: inertial-axis delta-omega added to body-frame omega.
- launch_site_altitude_m>0: ground projected to R_E+h but altitude from R_E -> touchdown test h<=0.1 maybe unreachable (not run).
- Downrange telemetry uses C.INITIAL_POSITION, not configured site.
- Stale docstrings: manager.py MECO at dry mass, Knudsen entry test.
- Unreachable: PRELAUNCH, APOGEE_REACHED phases, "ascent_only" termination branch.
- Orbit achieved as soon as within +-75 km; nominal perigee 326.7 km just inside 325 km edge.
## Ch2 Dynamics
- Rare RecursionError in the exhaustion split: rounding can re-trigger the split (reproduced at S2 m=6675.133319624401 kg, dt 20 s).
- Orbiter jet-damping lever arm uses the stack-base CG (58–63 m) instead of the distance to the S2 nozzle: S2 pitch/yaw damping is 10–30x too strong.
- Orbiter aero moment uses the stack CP of 33 m.
- Stacked inertia/CG ignore config masses and payload; prop fraction pinned at 1 for the first ~3 s.
- Stage-1 mass flow ignores `stage1_engine_thrust_n` (constant 7.607 MN reference).
- The roll -Idot*omega term outweighs roll jet damping: slow roll spin-up.
- Energy check is dead at defaults (needs central gravity, J2 off). Logged forces come from the end of the step.
- Slosh/flex telemetry columns are always 0.
## Ch6 Ascent and orbit
- `target_inclination_deg` and `stage_sep_velocity` are unused; inclination is set by launch latitude (29.1°).
- Pitch-command jump of up to 9° at 1000 m (kick end vs PID floor).
- No apogee-overshoot recovery (coasts to the 5000 s timeout). S2 never ignites if its post-separation apogee is below 70 km.
- `orbit_ok` checks differ between the manager and the guidance (perigee > 100 km term).
- RAISE_APOGEE assumes full thrust when throttled. The insertion AoA limit uses inertial velocity. The deorbit Δv cap uses the constant Isp.
- VALIDATION.md "MECO time" is really separation time (+3 s), so the early-MECO gap is understated by ~2%.
## Ch7 Boostback and entry
- Impact-time interpolation bug (see top list); the landing hand-off may depend on it (unconfirmed).
- The predictor freezes the current phase's drag multiplier (1.0 in boostback vs ~3.6 later).
- The movable target is silently ignored when an explicit landing lat/lon is set.
- Flip alignment is checked against inertial retrograde, while guidance points at air-relative retrograde.
- `booster_apogee_target_km` 85 km is a weak blend; the nominal apogee is ~138.7 km.
- `booster_entry_budget_kg` is not enforced during entry. The 0.45 entry burn effectively never runs (terminal capture is active instead).
- q and q·alpha limits are not guidance constraints, only post-flight checks.
- Grid-fin law uses the full 2/t_go ZEV term, which entry capture halves.
- The planner check uses 3600 m/s without the 50 m/s sizing margin.
- Dataclass and factory defaults disagree (entry min altitude 20000 vs 5000; wind exposure 61 vs fallback 150).
- `predicted_landing_error_m` telemetry during flip/boostback/coast is the current distance, not a prediction.
## Ch5 Propulsion, mass, actuators
- The throttle-efficiency factor and the S2 atmospheric penalty cut thrust but not mass flow, so effective Isp drops.
- Thrust is zero below 1% throttle, but propellant still flows (`forces.py:867`).
- The spool limit tracks the throttle value, not the on/off flag, so a re-ignition can skip spool-up.
- TVC authority uses rated thrust; the gimbal force uses actual thrust (~10% apart in vacuum).
- RCS propellant is not deducted from vehicle mass; each vehicle starts with a fresh 300 kg.
- One `tvc_lever_arm_m` override applies to all stages; validate() checks the mass offset against constants.
- The actuator docstring says "second-order"; the code is a first-order 25°/s rate limit. The real gimbal has no dynamics.
## Ch4 Aerodynamics
- Lift uses `CL_ALPHA_VALUES`; the moment uses `C_N_ALPHA` = 4/rad. The CP is a fixed 33 m (booster tail-first 31 m).
- The grid-fin force ignores attitude and q, with no fin moment and instant deploy. Recovery drag scales follow the phase name, not the hardware state.
- The deck fin-angle lookup converts degrees twice (harmless today). The drag alpha ignores wind.
- The landing-leg FAILED state is never assigned. The geometry deck is an empirical body buildup, not Barrowman.
## Ch1 Architecture
- Orbiter and booster clocks drift apart when dt > 0.05 s. `separation_time` is 2 s before the actual split.
- The hard-landing limits differ (12 vs 10 m/s). A raised launch-site altitude may make touchdown unreachable (not run).
- Downrange telemetry uses the constant launch site. Some phases and branches are unreachable.
- The orbit is "achieved" on first entry to the ±75 km band (perigee 326.7 km against a 325 km edge).

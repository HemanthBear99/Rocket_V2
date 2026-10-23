---
last_mapped: 2026-10-22
version: 0.3.0 (Vyom)
---

# Vyom (`rlv_sim`) — Full Codebase Map

Two-stage reusable launch vehicle 6-DOF simulator. Stacked ascent, then separation into an orbiter (S2 orbit insertion, optional S2 recovery) and a booster (flip, boostback, entry, landing). Front ends: CLI, FastAPI + Three.js web UI, pywebview desktop app. Package paths (`rlv_sim/...`) are relative to `backend/`.

## 0. Repository layout

| Folder | Contents | Ship it when |
|---|---|---|
| `backend/` | Python package `rlv_sim/` (core, physics, aero, vehicle, guidance, mission, config, outputs, server, validation), `tests/`, `tools/`, `pyproject.toml`, `uv.lock` | client needs the simulator, CLI or API server |
| `frontend/` | Static web UI (`index.html`, `css/`, `js/`, icons) | client needs the UI (served by backend, or any static host with backend URL set in Settings) |
| `desktop/` | `desktop_app.py`, `desktop_app.spec`, `installer/vyom.iss`, `version_info.txt`, `dist/` | client needs the Windows app (ship `dist/` output) |
| `website/` | Next.js marketing site | deployed separately |
| `docs/` | Documentation | internal |

Frontend lookup: `rlv_sim/server/app_paths.frontend_dir()` uses `RLV_FRONTEND_DIR`, else `frontend/` inside the PyInstaller bundle, else repo `frontend/`.

## 1. Physics core

### State (`rlv_sim/core/state.py`)
- `State`: `r`(3) ECI m, `v`(3) ECI m/s, `q`(4) quaternion `[w,x,y,z]` body→ECI, `omega`(3) body rad/s, `m`, `t`.
- Vector form (14): `[r 0:3, v 3:6, q 6:10, omega 10:13, m 13]`. Non-unit `q` raises.
- `create_initial_state(config)`: launch lat/lon, `v0 = ω_E × r0`, body +Z = local up.

### Frames (`rlv_sim/core/frames.py`)
- Scalar-first quaternions; `v_inertial = R(q) v_body`. Body **+Z = thrust axis**.
- `quaternion_derivative` (q̇ = ½Ω(ω)q), `direction_to_quaternion`, `quaternion_to_euler_zyx` (output only).
- ECI inertial, Earth rotates about +Z. Air-relative velocity `v − ω_E×r − wind` (`utils.compute_relative_velocity`).

### Data flow per step
1. `rlv_sim/mission/step.py` updates actuator, RCS, navigation; builds `DynamicsContext`.
2. `integrators.rk4_step(state, torque, dt, ctx)` — torque and ctx held constant (zero-order hold).
3. `dynamics.compute_state_derivative` → `rlv_sim/physics/mass.py` (inertia, CG, ṁ, İ) + `forces.compute_aerodynamic_moment` + `compute_linear_acceleration`.
4. `forces._compute_force_breakdown`: gravity + thrust + drag + lift + grid fins. Same breakdown exported by `compute_specific_forces` for telemetry.

### Modules
| Module | Role |
|---|---|
| `rlv_sim/physics/dynamics.py` | `DynamicsContext` (thrust_on, throttle, stage, vehicle_model stacked/orbiter/booster, aero mode, grid fins). r̈ = F/m; ω̇ = I⁻¹(τ − ω×Iω − İω + jet damping). Torque x/y also sets TVC deflection. Aero moments off above `AERO_DISABLE_ALTITUDE`. |
| `rlv_sim/physics/integrators.py` | RK4; renormalises q, clamps m ≥ dry, splits step at propellant depletion. `integrate()` dispatcher. |
| `rlv_sim/physics/forces.py` | Gravity (central / J2 / EGM96 deg-6), US-76 atmosphere, pressure-compensated S1 thrust and vacuum S2, Mach Cd table or aero deck, slender-body lift with 20–45° post-stall blend, CP/CG moment, booster recovery aero scaling, S2 entry drag ×45, Sutton-Graves heating, engine transients. |
| `rlv_sim/physics/mass.py` | ṁ = T/(Isp·g0) with runtime scales; stacked CG (stations z = 24/19/59/56 m); diagonal inertia, İ, exhaustion check. |
| `rlv_sim/core/constants.py` | μ, R_EARTH 6371 km (WGS84 eq. radius for J2/EGM96), masses, 7.607 MN thrust, Isp 282 s, Cd 0.42, A_ref 10.75 m², inertia, gains, torque limits, RCS, wind. |
| `rlv_sim/physics/numba_kernels.py` | Optional njit `egm96_accel`, US-76, `propagate_to_surface`; `AVAILABLE` flag with pure-Python fallback. |
| `rlv_sim/physics/high_fidelity_atmosphere.py` | ECI→WGS-84 geodetic; `compute_gram_atmosphere` density modifier (only if `enable_high_fidelity_gram`). |
| `rlv_sim/aero/database.py` | `AeroDatabase`: JSON deck, n-linear interp of CD/CL/CM over Mach, α, β, fin angle. |
| `rlv_sim/aero/geometry.py` | Deck from geometry (Sommer-Short friction, Hoerner base drag, nose wave drag, Munk CNα). |
| `rlv_sim/aero/deck_import.py` / `rlv_sim/aero/validation.py` | CSV/JSON deck import into the user data `aero_decks/` folder (bundled decks in `rlv_sim/aero/decks/`); sanity + literature (Tier-2) checks. |
| `rlv_sim/vehicle/rcs.py` | `RCSState`; mass flow, propellant update, available torque. RCS propellant tracked outside `State.m`. |
| `rlv_sim/vehicle/actuator.py` | `ActuatorState`; rate-limited thrust-direction slew. |
| `rlv_sim/vehicle/navigation.py` | `GPSReceiver`, `IMUPropagator`, `LandingAltimeter`, `NavigationState`. Estimate feeds guidance only; integrator uses truth. |
| `rlv_sim/core/utils.py` | `cross3`, `vec_norm` (use in hot paths), power-law wind, relative velocity, AoA. |

### Known physics limitations
- Variable-mass angular momentum incomplete (documented in code).
- Diagonal axisymmetric inertia; spherical Earth for altitude unless GRAM; EGM96 truncated to degree 6.
- Aero tables and recovery scale factors are engineering approximations.

## 2. Mission, guidance and control

### Phases (`mission_manager.MissionPhase`, `MissionManager.update`)
- **Stacked:** ASCENT → COAST → STAGE_SEPARATION → S2_COAST_TO_APOGEE.
- **Orbiter:** S2_COAST_TO_APOGEE → ORBIT_INSERTION → ORBIT_ACHIEVED | ORBIT_FAILED; with `enable_s2_recovery` → S2_ORBIT_HOLD → S2_DEORBIT → S2_ENTRY → S2_LANDING.
- **Booster:** BOOSTER_FLIP → BOOSTBACK → COAST → ENTRY → LANDING.
- PRELAUNCH and APOGEE_REACHED are defined but never reached.

### Transitions
| From → To | Condition |
|---|---|
| ASCENT → COAST (MECO) | mass ≤ `meco_mass_kg` (includes S1 landing reserve) |
| COAST → STAGE_SEPARATION | 3 s after MECO |
| STAGE_SEPARATION → S2_COAST | 2 s after separation |
| Coast → ORBIT_INSERTION | alt > `orbit_insertion_start_altitude_m`, or Vr < −20 m/s above entry interface |
| ORBIT_INSERTION → ACHIEVED | energy < 0, perigee/apogee in tolerance, e ≤ `orbit_ecc_max` |
| ORBIT_INSERTION → FAILED | S2 propellant out; alt < 80 km with Vr < −50 m/s and perigee < 100 km; or timeout |
| S2_ORBIT_HOLD → DEORBIT | hold time elapsed and at apoapsis (Vr + → −) |
| S2_DEORBIT → ENTRY → LANDING | below entry interface descending; then below 15 km descending |
| BOOSTER_FLIP → BOOSTBACK | within 15° of retrograde after min time, or 30 s |
| BOOSTBACK → COAST | ≥15 s burn + impact/velocity targeting met; or budget used; or reserve guard |
| COAST → ENTRY | below `booster_entry_interface_altitude_m`, descending |
| ENTRY → LANDING | suicide-burn estimator says ignite, or near-pad handoff (≥8 s to impact) |

Recovery phases use a finer max step (`_RECOVERY_FINE_STEP_PHASES`).

### Guidance (`_simulation_step._run_guidance`)
- Can use navigation estimate (`use_navigation_estimate_for_guidance`).
- **Ascent** (`rlv_sim/guidance/ascent.py`): altitude-scheduled flight-path angle, PID tracking, Max-Q throttle hold, AoA limit. Not IGM/PEG.
- **Coast/separation:** hold last ascent direction or prograde, engine off.
- **Orbit insertion** (`rlv_sim/guidance/orbit.py`): RAISE_APOGEE, COAST_TO_CIRCULARIZATION, CIRCULARIZE (latched), HOLD_ORBIT, FAILED_UNREACHABLE.
- **S2 recovery:** full-throttle retrograde deorbit to negative perigee, unpowered retrograde entry, vertical suicide burn onto drone ship.
- **Booster** (`rlv_sim/guidance/booster.py`): retrograde flip; boostback via impact-point feedback and candidate scoring; `_entry_guidance`; latched suicide burn `a_z = g + (v² − v_t²)/(2h)` with ZEM/ZEV divert.
- **G-FOLD** (`rlv_sim/guidance/gfold.py`, `booster_landing_guidance="gfold"`): lossless-convexification min-fuel (cvxpy), replans every `gfold_replan_period_s`, heuristic fallback below `gfold_terminal_altitude_m` or on solve failure.
- `rlv_sim/guidance/recovery.py`: ballistic impact predictor, suicide-burn estimate, targeting, landing site in ECI. `rlv_sim/vehicle/recovery_hardware.py`: grid fins, landing legs.

### Control loop (`simulation_step`)
1. Guidance.
2. Actuator rate limit (skipped in booster and S2 recovery phases).
3. Engine spool transients; throttle zero when propellant gone.
4. `rlv_sim/vehicle/control.py` PD/PID on quaternion thrust-axis error, gains scaled by inertia vs `IXX_FULL`, anti-windup, integral reset on phase/controller change. PID forced for landings when `recovery_attitude_controller="auto"`.
5. Torque split TVC/RCS by authority (S2 recovery uses fixed cap).
6. RK4 integrate, ground and pad contact.
7. `check_termination`, `physics_checks.validate_state`.
8. `abort.AbortMonitor` (if `enable_abort_modes`): PAD_ABORT (t<30 s), RTLS (<40 km), DOWNRANGE_ABORT (<80 km and <3 km/s), else ATO.

### Mission runner (`rlv_sim/mission/full_mission.py`)
- `_run_ascent`: stacked vehicle, dt ≤ `STACKED_ASCENT_MAX_DT`.
- `_run_dual_vehicles`: fork at separation (`rlv_sim/mission/vehicle_split.py`). Orbiter = S2 wet + payload; booster = S1 dry + reserve. Optional separation Δv and tumble. Separate manager, guidance state, log and abort monitor per vehicle; stepped alternately. Orbiter takes larger coast steps once booster is done.
- `rlv_sim/mission/single_vehicle.py`: legacy single-vehicle loop.

### Campaigns (`rlv_sim/mission/campaign.py`)
- Dispersions (1σ): thrust 2%, Isp 1%, mass 0.5%, wind 5 m/s, density 5%, pitchover 0.25°.
- Sensitivity (baseline ± 1σ each) or Monte Carlo (seed 42, 100 runs, ±3σ clip).
- `apply_case` → `runtime_*` overrides; `_run_case` → `assess_full_mission` + landing miss E/N.
- `ProcessPoolExecutor` parallelism; Wilson intervals; 17 metric stats; `write_campaign_artifacts`.
- `rlv_sim/mission/planner.py`: simulation-free Δv budget and recovery reserve sizing.

## 3. Config and outputs

### Config
- `config_definition.SimulationConfig`: one frozen dataclass, ~250 fields (integration, attitude, ascent, launch site, orbit targets, masses, booster recovery, G-FOLD, landing site, grid fins/legs, physics models, sensors, abort, `runtime_*`, `aero_deck_path`).
- Derived: `stage1_thrust_n`, `boostback_thrust_n`, `entry_thrust_n`, `landing_thrust_n`, `reference_area_m2`, `meco_mass_kg`.
- `validate()` aggregates errors into one `ValueError`; `aero_deck_path` restricted to `rlv_sim/aero/decks/*.json` or the user aero folder.
- Grouped views: `VehicleProfile`, `MissionProfile`, `RecoveryProfile`, `PhysicsProfile`, `RuntimeProfile`; `from_profiles(...)`.
- `config_factory`: `create_default_config` (research profile), `create_demo_config` (dt 0.1), `create_test_config` (dt 0.1, 10 s). `_with_overrides` drops 22 `RETIRED_FIELDS`, auto-sizes reserve if enabled.
- `config_io.load_config_json`: JSON object of overrides; unknown keys raise.
- `config/schema.config_schema()`: every field with kind, default, unit, bounds (mirrored from `validate()`), group and help text from `config/field_docs.FIELD_DOCS`; drives the app's All Settings tab.
- Vehicle geometry and mass properties are fields (`stage1_engine_ring_radius_m`, nozzle-exit radii, `stack_*`/`booster_*` inertias, `booster_cg_*_m`); `physics/mass.py` and jet damping read them, falling back to `core.constants`.

### Outputs
| Output | Source | Contents |
|---|---|---|
| Telemetry CSV | `_main_models.SimulationLog.to_csv` | ~120 columns: kinematics, lat/lon, mass, attitude, quaternions, torques, forces, Mach, q, AoA, heating, propellant, grid fins/legs, landing prediction, G-FOLD, nav errors, phase, RCS, thermal/slosh/flex |
| `FullMissionResult` | `rlv_sim/mission/models.py` | ascent/orbiter/booster logs, final states, reasons, landing metrics |
| Live telemetry | `rlv_sim/outputs/telemetry.py` | per-point dicts for WebSocket; `extract_mission_progress` |
| `mission_summary.json/.md` | `rlv_sim/outputs/summary.py` | `MissionAssessment`, `OrbitAssessment`, `LandingAssessment` |
| `run_manifest.json` | `rlv_sim/outputs/manifest.py` | schema 1.0: timestamp, version, platform, argv, full config, lockfile SHA-256, artifacts, assessment |
| Report comparison | `rlv_sim/outputs/report_compare.py` | tolerances: perigee/apogee 5 km, e 0.01, touchdown 2 m/s, site 100 m |
| Plots | `rlv_sim/outputs/plots/general.py` (01–58), `rlv_sim/outputs/plots/mission.py` (59–82) | 300 dpi PNG + `plot_manifest.json` |

### Storage and licensing
- `rlv_sim/server/app_paths.py`: `RLV_DATA_DIR` or `%APPDATA%\Vyom` / `~/.vyom`; one-time migration from Boostback folder.
- `rlv_sim/server/storage.py`: `scenarios/<slug>.json`, `runs/<kind>_<stamp>_<slug>.json`; IDs `^[a-z0-9_-]{1,80}$`.
- `rlv_sim/server/licensing.py`: offline Ed25519 keys `b64url(json).b64url(sig)`, `<data>/license.key`, 14-day trial, enforced only in frozen exe or `RLV_ENFORCE_LICENSE=1`.
- `rlv_sim/core/result_types.py`: `ControlOutput`, `ForceBreakdown` TypedDicts.

## 4. Front ends, packaging, tooling

### CLI (`rlv_sim/cli.py`, `python -m rlv_sim` or `rlv-sim`)
Flags: `--profile`, `--demo`, `--config-json`, `-o/--output-dir`, `-q`, `--no-plots`, `--no-telemetry`, `--mission`, `--dt`, `--max-time`, `--nav-mode`, `--enable/disable-abort-modes`, `--validation-report`, `--attitude-controller`, `--landing-guidance`, `--recovery-attitude-controller`. `rlv_sim/mission/entry_points.py` is a compat re-export.

### API (`rlv_sim/server/app.py`, auth on all routes except health)
- `GET /`, `GET /api/health`, `GET /api/about`, `POST /api/license`
- `/api/scenarios`, `/api/runs` (list, create, get, delete)
- `/api/aero/import`, `/api/aero/decks`, `/api/aero/validation[/tier2]`
- `/api/config/defaults`, `/api/config/validate`, `/api/mission/plan`
- `/api/simulations/start`, `/{id}/pause|resume|stop`, `GET /{id}`; WebSocket `/ws/simulations/{id}/telemetry`
- `/api/campaigns/start` (modes `monte_carlo|sensitivity|sweep_3sigma`, `dispersions`, `case_ids`), `GET /{id}`, `/{id}/cancel`
- `/api/config/schema`, `/api/plugins`, `/api/model` (model card), `/api/validation/flights` (cached Falcon 9 comparison)
- `SimulationSetup.advanced_overrides` accepts any config field (lists for wind modes, null for optional fields); `campaign_case` replays one campaign case.
- Request models in `rlv_sim/server/api_models.py` (pydantic v2).

### Web UI (`frontend/`, mounted at `/static`)
`app.js`, `charts.js`, `report_charts.js`, `rocket_viz.js` (Three.js). Views: config (tabs incl. Wind & Dispersions and All Settings), telemetry, report, campaign (drivers table, replay, re-run failed), compare, validation, model, settings.

### Guidance plugins
`guidance/plugins.py`: user files in `app_paths.user_plugin_dir()` (`%APPDATA%\Vyom\plugins`, `RLV_PLUGIN_DIR`) or `guidance/plugin_examples/`; `landing_command(state) -> accel` replaces the landing-burn law when `booster_landing_guidance="plugin"`. Selected by name only; no upload or path loading.

### Desktop and release
1. `pip install -e .[desktop]` (extras: `desktop`, `test`, `fast`=numba, `gfold`=cvxpy).
2. `desktop/desktop_app.py`: uvicorn thread on free port (prefers 8756), pywebview window.
3. `pyinstaller desktop_app.spec` (from `desktop/`) → `desktop/dist/Vyom.exe`.
4. `ISCC installer\vyom.iss` (from `desktop/`) → `desktop/dist/VyomSetup-0.3.0.exe` (per-user, data kept on uninstall).

### Other
- `backend/tools/issue_license.py`: issue keys with private PEM.
- `rlv_sim/validation/`: `flight_compare.py` (Falcon 9 webcast telemetry), `drag_fit.py`; data for NROL-76, Orbcomm OG2, CRS-11, ZUMA.
- `website/`: Next.js 14 marketing site (React Three Fiber, GSAP).

## 5. Tests and health (2026-10-22)

- Fast suite: **248 passed, 6 skipped** (cvxpy, numba absent); Playwright UI test passes; `ruff check .` clean. Slow suite not run.
- `pytest` default `-m "not slow"`; slow marker on booster guidance, flight validation, full-mission regression, vehicle builder, wind robustness.
- No dedicated tests: `abort`, `rcs`, `recovery`, `recovery_hardware`, `navigation`, `frames`, `actuator`, `telemetry`, `report_io`, `mission_summary`, `run_manifest`, `high_fidelity_atmosphere`, `cli`, `desktop_app`, `outputs/plots/*`.
- `backend/tests/test_telemetry_csv.cjs` needs a JS runner (`node --test`).

## 6. Open issues

1. Default gimbal direction in `rlv_sim/vehicle/actuator.py` is body X while thrust is body Z (possible bug).
2. Plot filename clashes: two `68_` files, two `10_` files; docstring says `11_flight_path_angle` but file is `16_`.
3. No desktop build exists (old Boostback builds deleted 2026-10-22); license tool uses `~/.boostback_signing`.
4. Version 0.3.0 hard-coded in `backend/pyproject.toml` and `desktop/installer/vyom.iss`.
5. PRELAUNCH and APOGEE_REACHED phases unreachable.
6. Some tests assert by reading source text (brittle).

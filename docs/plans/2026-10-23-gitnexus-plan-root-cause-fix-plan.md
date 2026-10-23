# GitNexus Engineering Plan

> Task: Root-cause fix plan for the whole Vyom repo (backend physics/guidance/assessment, frontend/API, desktop exe, docs).
> Evidence verified at commit 760c0e409374f2b4b531356a2334e89cf1aa3569; GitNexus index **not used**: the repo has no `.gitnexus` index. **Fallback mode**: every finding is source-derived (targeted reads/greps, plus the seeded theory-manual review). No graph or PDG evidence exists.
> Evidence provenance schema 2; global dirty digest cea5994b7b1f08f3952f4c782eaeba1147c55455e248b09f6b6bfcdcaa723fe1; cited-path manifest 37 sorted entries; exact generated plan path excluded.
> Plan written with the ordinary file tool, with the user's approval. The skill's safe `write-plan` helper refuses win32 ("requires Linux /proc/self/fd or macOS O_DIRECTORY/O_NOFOLLOW").

## 1. Objective

Fix every defect found in the theory-manual review (`docs/theory/CODE_FINDINGS.md`, about 100 items) by **root cause rather than symptom**, so that:

- the physics, guidance and assessment do what the documentation says;
- every user-facing setting has a real effect, or is removed;
- one source of truth exists for vehicle properties, defaults and mission outcomes;
- frontend, API, exe and docs match the fixed backend.

Acceptance:

- each finding is either fixed with a test, or recorded as an explicit modelling limit with a decision;
- the fast suite, slow suite and ruff all pass;
- Monte Carlo is re-baselined once at the end;
- the exe is rebuilt and smoke-tested.

## 2. Current Behaviour

The defects cluster into a small number of systemic causes, rather than one hundred independent bugs. Verified examples:

- **Vehicle data in two places.** Mass properties, MECO and validation read `core/constants.py` instead of the config [verified: `physics/mass.py` uses `C.STAGE1_PROPELLANT_MASS`, `C.DRY_MASS`]. `definition.py` `validate()` uses `C.INITIAL_MASS`/`C.DRY_MASS` [inferred, Ch5/Ch1 review]. TVC authority uses `C.MAX_GIMBAL_ANGLE` at `mission/step.py:244,277,640`, while `max_gimbal_angle_deg` is only validated (`config/definition.py:503`) [verified].
- **Identity by string.** Jet damping compared `"LANDING"` against the phase name `"BOOSTER_LANDING"` [verified; fixed in the working tree, untested]. Landing success is `str(reason).startswith("LANDING SUCCESS")` (`mission/step.py:85-93`) [verified]. Recovery drag scales key off the phase name, not the hardware state [inferred, Ch4].
- **Outcome truth split three ways:**
  - the termination reason string;
  - `assess_full_mission` (`outputs/summary.py`);
  - touchdown contact. `touchdown_contact_status` is always `''` (`mission/step.py:226`) [verified], so `summary.py`'s contact overlay never fires [verified, `summary.py:405-432`]. The leg gate fails whenever legs are disabled [verified: status != LOCKED → failure].
  - The hard-landing limit is `2 × success` = 12 m/s (`summary.py:212`) against a 10 m/s path [inferred, Ch1].
- **Settings never wired.** `target_inclination_deg`, `stage_sep_velocity` and `atmosphere_model` have no reader outside config and server [verified by grep]. `enable_high_fidelity_gram` acts only when `r` is passed (`forces.py:212-215`), and no caller passes it [verified line; caller claim inferred, Ch3]. `min_engine_throttle_fraction` is applied only in `guidance/ascent.py` and G-FOLD (`booster.py:797`) [verified].
- **Default drift.** 15 fields have dataclass defaults that differ from `create_default_config()` (e.g. the ignition safety factor 1.52 vs 1.8, the entry minimum altitude 20000 vs 5000) [verified by script].
- **Numerical defects.** The impact-time interpolation returns `elapsed - dt*(1-f)` where it should return `elapsed - f*dt` (`guidance/recovery.py:436`, duplicated at `physics/numba_kernels.py:258`) [verified]. EGM96 longitude comes from ECI `arctan2(y, x)` without Earth rotation (`forces.py:434`) [verified]. Grid-fin drag is applied in `forces.py:1071-1076` and again in `vehicle/recovery_hardware.py:258-268` [verified]. Separation Δω is split by mass and added in ECI axes (`mission/vehicle_split.py:56-57`) [verified].
- **Two implementations that drift.** The Numba kernels duplicate the Python predictor and gravity code, and the Numba predictor omits wind modes [inferred, Ch3/Ch7].
- **Docs claim things the code doesn't do:** skin temperature (no writer for `thermal_wall_temp_k` beyond the default 300 in `mission/models.py:246`) [verified], the actuator "second order", controller ωn, VALIDATION*.md numbers, and `drag_fit.py` writing to `backend/rlv_sim/docs` (`DOCS = ROOT.parent / "docs"`, line 30) [verified].

## 3. Relevant Architecture

The per-step loop runs `mission/full_mission.py` → `mission/step.py`. Step calls:

- guidance: `guidance/ascent.py`, `orbit.py`, `booster.py`, `recovery.py`, `gfold.py`, `plugins.py`;
- control: `vehicle/control.py`, `actuator.py`, `rcs.py`;
- the RK4 integrator `physics/integrators.py`, which calls `physics/dynamics.py`, which calls `physics/forces.py` and `physics/mass.py`. `physics/numba_kernels.py` mirrors parts of these.

Config is the frozen `config/definition.py`; profiles come from `config/factory.py`; help text lives in `config/field_docs.py`; the UI schema is `config/schema.py`. Outcomes are produced by `outputs/summary.py`. The server is `server/app.py` (`map_config`), and the UI is `frontend/js/app.js` (curated tabs plus the schema-driven All Settings tab). The exe build is `desktop/desktop_app.spec`. Repo conventions (`AGENTS.md`): new or changed fields must keep current behaviour by default, be validated, carry a `field_docs` line, and change defaults only after a Monte Carlo comparison.

## 4. GitNexus Findings

Not available: no index exists, so the plan runs in fallback mode. Callers and dependents were approximated with grep (source-derived):

- `C.MAX_GIMBAL_ANGLE` readers: `mission/step.py:244,277,640`; `physics/forces.py:796` (docstring).
- `hard_landing_speed_limit` readers: `mission/step.py:491`, `outputs/summary.py:234`.
- Grid-fin area/Cd readers: `physics/forces.py:1071-1076`, `vehicle/recovery_hardware.py:183,258,268`.
- The impact-time formula is duplicated in exactly 2 places (`recovery.py:436`, `numba_kernels.py:258`).

Run `npx gitnexus analyze --index-only` before executing steps 3, 5 and 6 to get a real blast radius.

## 5. Statement-Level PDG Findings

Not available (no PDG layer). Ordering constraints were derived from source instead:

- (a) **Assessment first.** Outcome consolidation (RC3) must land before any physics fix whose effect is judged by success flags.
- (b) **Python and Numba change together.** The impact-time fix must change both copies in the same commit, plus a parity test.
- (c) **Re-fit drag after the grid-fin fix.** That fix invalidates `booster_descent_drag_scale=0.68` (`docs/VALIDATION_DRAG.md`), so the drag re-fit depends on it.
- (d) **Re-baseline once.** Regression references and Monte Carlo baselines are regenerated once, at the final step.

## 6. Proposed Changes

Grouped by root cause (RC). "Physics Δ" means outputs change: run the slow suite now, and include the change in the final Monte Carlo re-baseline.

**RC0. Land the in-flight jet-damping fix.**
- `physics/dynamics.py` `_active_engine_count`: already matches `BOOSTER_*` names.
- `tests/test_research_features.py:149`: switch to `"BOOSTER_LANDING"`.
- Better: replace the string checks with `MissionPhase` membership (see RC2).
- Physics Δ: booster roll damping in a 1-engine landing drops from about 644 to about 32 kg·m²/s.

**RC1. Vehicle properties from the config, not constants.**
- `physics/mass.py`: stack and orbiter CG/inertia and propellant fraction from the configured stage masses. Payload belongs in dry/payload, not propellant.
- Mass flow: derived from `stage1_engine_thrust_n` and the configured Isp (Ch2: currently the constant 7.607 MN).
- `config/definition.py` `validate()`: mass-offset checks against the configured masses.
- Orbiter lever arm and CP measured from the S2 base (Ch2: jet damping 10–30× too strong; CP 33 m for all vehicles at `dynamics.py:288`).
- TVC authority from `max_gimbal_angle_deg` (`step.py:244,277,640`).
- Telemetry downrange from the configured launch site.
- Altimeter reference radius.
- Physics Δ.

**RC2. Typed identity instead of string matching.**
- Phase checks use the `MissionPhase` enum (`dynamics.py`, `step.py:827`).
- Recovery drag/moment scales follow the hardware state: deployed fins/legs and `enable_grid_fins`/`enable_landing_legs` (Ch4).
- Landing outcome becomes a structured result (enum + reason), not a string prefix.
- Physics Δ, from the scale change.

**RC3. One authority for mission outcome.**
- Make `assess_full_mission` (or a shared `assess_touchdown`) the only judge. `_booster_landing_success` (`step.py:85-93`) calls it.
- Log the actual contact status at touchdown (`step.py:226`).
- Skip the leg gate when legs are disabled (`summary.py:415`).
- One hard-landing limit (`summary.py:190-212` against the contact path).
- One tilt limit (`landing_leg_max_tilt_deg` vs `booster_terminal_attitude_error_max_deg`).
- `booster_enforce_pad_landing` given real meaning, or removed.
- One `orbit_ok` shared by manager and guidance.
- Reason wording stops claiming success when the margin fails.
- No dynamics change; success counts may change.

**RC4. Every setting is consumed, or deleted.** Add a guard test first (see §8), then decide per field:
- `max_gimbal_angle_deg`: wire it (RC1).
- `min_engine_throttle_fraction`: enforce on the booster heuristic and plugin path (`booster.py`), and fix the `plugins.py` docstring.
- `enable_high_fidelity_gram`: pass `r` from callers (`forces.py:212-215`), or remove the field.
- `atmosphere_model`: remove it, or make it a real selector.
- `target_inclination_deg`: implement launch-azimuth steering (`guidance/ascent.py` kick direction), or remove it from API/UI. This needs a decision (§12).
- `stage_sep_velocity`: define and wire it, or remove it.
- `booster_entry_budget_kg`: enforce during entry.
- Movable target with an explicit lat/lon: apply or reject in `validate()`.
- `booster_apogee_target_km`: document it as a weak blend, or strengthen it.
- q and q·α limits: either constrain guidance, or document them as post-flight checks.
- Physics Δ for the wired fields.

**RC5. One default source.**
- Move the 15 factory-only values into the dataclass defaults (or the reverse, with a test asserting `create_default_config()` == dataclass defaults apart from documented profile overrides).
- Update `field_docs.py` and the schema defaults.
- No Δ for the default profile.

**RC6. Physics and numerics defects.** Each item gets its own test.
1. **EGM96 coefficients** (`forces.py:66` `_EGM96_C`/`_EGM96_S`): regenerate from the official `egm96_to360.ascii` to degree/order 6.
2. **Earth rotation in gravity:** use Earth-fixed longitude `λ = atan2(y,x) − ω_E·t` (`forces.py:434`), which needs `t` threaded through. Mirror both in `numba_kernels.py`.
3. **Impact time** = `elapsed − f·dt` in `recovery.py:436` and `numba_kernels.py:258`.
4. **Grid-fin drag:** remove the double count, choosing one model (`forces.py:1071-1076` vs `recovery_hardware.py:258-268`) and one area field. Then **re-fit `booster_descent_drag_scale`** (`validation/drag_fit.py`).
5. **Separation:** Δω in body axes, split by inertia so angular momentum is conserved (`vehicle_split.py:56-57`). Separation time recorded at the split.
6. **Exhaustion split:** an exact remaining-propellant time step with a guaranteed non-recursive termination (`physics/integrators.py`); covers the RecursionError case at m = 6675.133319624401 kg.
7. **Aero deck Cm sign:** adopt the aircraft convention for imported decks and convert at load (`aero/database.py`, `forces.py:980`), plus a deck-convention field.
8. **Thrust model:** mass flow follows the thrust efficiency and S2 atmospheric penalty (or the thrust penalty is removed); no propellant flow below the 1% cutoff (`forces.py:867`); spool state tracks on/off; TVC authority uses actual thrust; lever arm from CG to gimbal.
9. **Stage-2 recovery torque limits** from S2 actuators (`step.py:627`, S2 phases).
10. **US-76 geopotential altitude;** the density scale stops scaling back-pressure (optional, low impact).
- Physics Δ for all of these.

**RC7. Guidance consistency** (`guidance/booster.py`, `recovery.py`):
- landing thrust includes `runtime_thrust_scale`;
- reserve authority applied exactly once, in every branch (`booster.py:1041,1050,1054-1058`);
- the hover-cut rule is altitude-gated (`booster.py:1092-1094`);
- predictor drag multiplier follows the predicted phase;
- flip uses the same retrograde reference in manager and guidance;
- the 2/t_go nulling gain is documented or changed;
- `gfold_glideslope_deg` doc corrected (`field_docs.py`);
- `estimate_suicide_burn` uses the configured atmosphere and dry mass;
- RAISE_APOGEE throttle-aware; insertion AoA air-relative; deorbit uses `stage2_isp_vac` (`guidance/orbit.py`);
- the pitch-kick step at 1000 m smoothed (`guidance/ascent.py`);
- an apogee-overshoot mode, or an explicit failure.
- Physics Δ.

**RC8. Python/Numba parity.** One shared reference test compares `numba_kernels.propagate_to_surface` and `egm96_accel` with the Python paths, including wind modes (currently missing in Numba). Fix the gap.

**RC9. Navigation and aborts** (`vehicle/navigation.py`, `guidance/abort.py`):
- separate seeds per vehicle after the split;
- IMU specific force from the true non-gravitational acceleration;
- GPS-only propagation between fixes;
- the configurable body-rate threshold;
- `q_alpha_max` vs the recovery gate reconciled;
- first tests for both modules.
- Δ only in `--nav-mode onboard` and abort-enabled runs.

**RC10. Honest missing models.**
- **Skin temperature:** implement a lumped wall model fed by Sutton-Graves, or remove the claim (`outputs/model_card.py`, `MATHEMATICAL_REFERENCES.md` §7) and the dead telemetry columns.
- **Slosh/flex columns:** remove them, or document them as reserved.
- **RCS propellant:** deduct it from vehicle mass and carry it across the split.
- **Landing leg** `FAILED`: assign it, or remove it.

**RC11. Docs follow code.**
- Fix the stale docstrings: `vehicle/actuator.py` "second-order", `mission/manager.py` MECO/Knudsen, `vehicle/control.py:233-235` ωn/ζ.
- `drag_fit.py` writes to the repo `docs/`, and its line-71 no-op is removed.
- Regenerate VALIDATION.md and VALIDATION_DRAG.md; the MECO time becomes true MECO, not separation (`flight_compare.py:57`).
- Update `MATHEMATICAL_REFERENCES.md`, `model_card.py` KNOWN_LIMITS, theory chapters 1–11 and `CODE_FINDINGS.md` (mark items fixed).

**RC12. Frontend, API and exe.**
- After RC4: drop removed fields from `server/app.py` `map_config`/defaults, `api_models.py`, `config/schema.py` FORM_FIELDS and the `frontend/js/app.js` FIELD_META. Add UI for newly wired fields (inclination, if implemented).
- Telemetry `predicted_landing_error_m` naming (Ch7).
- Rebuild the exe (`desktop/desktop_app.spec`); smoke-test the API pages, settings schema, plugin list and flight validation inside the exe.

## 7. Implementation Sequence

Each step is a commit; the tree stays green after each.

1. **RC0:** finish the jet-damping fix and its tests. Run the fast and slow suites.
2. **Guard tests first** (see §8): "every config field is read" (xfail list = the current dead fields), dataclass-vs-factory defaults, Numba parity, outcome consistency. They document the current failures as expected failures.
3. **RC5 defaults**, then **RC3 outcome authority**. Assessment semantics now settle, before any physics changes.
4. **RC2 typed identity.**
5. **RC1 config-sourced vehicle properties.**
6. **RC4 wire or remove the dead settings.** Clear the xfails as each field is resolved; the guard test now passes fully.
7. **RC6**, one sub-commit each, in this order: 3 (impact time), 6 (exhaustion), 5 (separation), 2 + 1 (gravity), 8 (thrust), 9 (S2 torque), 7 (Cm sign), 4 (grid-fin drag + drag re-fit), 10 (atmosphere). Run the slow suite after each; do not edit regression references yet.
8. **RC8 parity fixes** (alongside RC6-2/3 where they touch Numba).
9. **RC7 guidance consistency.**
10. **RC9 navigation/abort,** with new tests.
11. **RC10 missing models** (decisions from §12).
12. **RC11 docs, and RC12 frontend/API.**
13. **Re-baseline once:**
    - regenerate the regression references;
    - re-run `flight_compare` and `drag_fit`;
    - run the 100-run seed-42 Monte Carlo at drag 1.0 and at the re-fitted drag;
    - update MONTE_CARLO_RESULTS.md, `model_card.py` numbers, AGENTS.md and the theory chapters.
14. **Rebuild and smoke-test the exe;** merge to `main` after review.

## 8. Test Strategy

**New tests:**
- **`tests/test_config_consistency.py`:**
  - every `SimulationConfig` field is read by non-config code (AST/grep scan) or is on an explicit allowlist;
  - factory defaults equal dataclass defaults, except for a documented override list;
  - `field_docs` covers every field.
- **`tests/test_numba_parity.py`:** EGM96 acceleration and the impact predictor agree with Python to 1e-9 relative, including with wind modes. Skipped without numba.
- **`tests/test_outcome_authority.py`:**
  - touchdown with low propellant margin → both flags fail;
  - legs disabled → no leg failure;
  - tipover → contact status logged and failure;
  - speed between 10 and 12 m/s → a single classification.
- **`tests/test_impact_prediction.py`:** a drag-free drop from rest gives `t = √(2h/g)` within one predictor step's interpolation error.
- **`tests/test_separation.py`:** angular momentum is conserved within 1e-9; Δω is in body axes.
- **`tests/test_integrator_exhaustion.py`:** the m = 6675.133319624401, dt = 20 s case terminates.
- **`tests/test_gravity.py`:** C/S values equal the official table entries; the gravity vector in ECEF is constant for a body fixed in ECEF over time.
- **`tests/test_grid_fin_drag.py`:** fin drag is counted once.
- **`tests/test_navigation.py`, `tests/test_abort.py`:** seeds are distinct per vehicle; IMU bias statistics; abort triggers.

**Existing tests to update:**
- `tests/test_jet_damping.py`, `tests/test_research_features.py` (phase names);
- `tests/test_full_mission_regression.py` references, regenerated once at step 13;
- `tests/test_wind_robustness.py` and `tests/test_booster_guidance.py` thresholds, re-checked after RC6/RC7.

**Commands** (from `backend/`):
- `.venv/Scripts/python -m pytest -q`
- `.venv/Scripts/python -m pytest -m slow` (run per file if memory is tight)
- `.venv/Scripts/python -m ruff check .`
- `.venv/Scripts/python -m rlv_sim.mission.campaign --mode monte-carlo --runs 100 --seed 42 --workers 10`
- `python -m rlv_sim.validation.flight_compare`
- `python -m rlv_sim.validation.drag_fit`

## 9. Risk and Impact Analysis

- **High-risk symbols:** `physics/mass.py` (every step), `forces.py` gravity and aero, `guidance/booster.py` landing, `outputs/summary.py` (all success metrics).
- **Monte Carlo numbers will move.** Gravity, grid-fin drag and the drag re-fit, mass properties and the impact time together shift success rates. The published 98%/88% and VALIDATION numbers become invalid until step 13.
- **Compatibility:**
  - Removing fields breaks saved scenarios and run JSONs that contain them. Add them to `RETIRED_FIELDS` (`config/factory.py`) so old files still load.
  - The API schema changes for `stage_sep_velocity`/`target_inclination_deg`.
- **Performance:** threading `t` into gravity, and the Numba parity work, must keep the hot-path rule (`cross3`/`vec_norm`).
- **No dependency graph.** The blast radius is grep-based (no GitNexus index), so it may miss indirect callers. Index before steps 5–7.
- **Memory:** the slow suite was killed once by low memory. Run it per file.

## 10. Files Expected to Change

| File | Symbols | Reason |
| ---- | ------- | ------ |
| backend/rlv_sim/physics/mass.py | compute_center_of_mass, compute_inertia_tensor, compute_inertia_derivative, mass flow | RC1 |
| backend/rlv_sim/physics/dynamics.py | _active_engine_count, CP/lever arm | RC0, RC1, RC2 |
| backend/rlv_sim/physics/forces.py | _EGM96_C/_S, egm96 gravity, recovery scales, grid fin, thrust, Cm | RC2, RC4, RC6 |
| backend/rlv_sim/physics/numba_kernels.py | egm96_accel, propagate_to_surface | RC6, RC8 |
| backend/rlv_sim/physics/integrators.py | exhaustion split | RC6 |
| backend/rlv_sim/guidance/recovery.py | impact predictor, estimate_suicide_burn | RC6, RC7 |
| backend/rlv_sim/guidance/booster.py | _landing_guidance, budgets, cutoff | RC4, RC7 |
| backend/rlv_sim/guidance/ascent.py, orbit.py, gfold.py, plugins.py | kick/inclination, insertion, docs | RC4, RC7 |
| backend/rlv_sim/mission/step.py | _booster_landing_success, contact logging, TVC authority, S2 torque | RC1, RC3, RC6 |
| backend/rlv_sim/mission/vehicle_split.py, manager.py | separation, orbit_ok, docstrings | RC3, RC6, RC11 |
| backend/rlv_sim/outputs/summary.py, model_card.py | assessment, limits | RC3, RC10, RC11 |
| backend/rlv_sim/vehicle/navigation.py, control.py, actuator.py, rcs.py, recovery_hardware.py; guidance/abort.py | nav, docs, RCS mass, fins | RC6, RC9, RC10, RC11 |
| backend/rlv_sim/config/definition.py, factory.py, field_docs.py, schema.py | defaults, removed/wired fields | RC4, RC5 |
| backend/rlv_sim/aero/database.py | Cm convention | RC6 |
| backend/rlv_sim/validation/drag_fit.py, flight_compare.py | paths, MECO | RC11 |
| backend/rlv_sim/server/app.py, api_models.py; frontend/js/app.js | exposed fields | RC12 |
| backend/tests/* | new and updated tests | §8 |
| docs/*.md, docs/theory/*.md | results, models | RC11, step 13 |
| desktop/desktop_app.spec | only if bundle data changes | RC12 |

## 11. Reusable Implementation Context

```yaml
implementation_context:
  task_summary: 'Root-cause fixes for the theory-review findings across backend, frontend/API, exe and docs; re-baseline once at the end.'
  acceptance_criteria:
    - 'Every CODE_FINDINGS.md item is fixed with a test, or recorded as a decided limit'
    - 'Guard tests pass: config-field consumption, default parity, numba parity, outcome authority'
    - 'Fast and slow suites pass and ruff is clean; regression references regenerated once'
    - 'Monte Carlo, VALIDATION*.md, model_card and theory docs updated from the re-baseline'
    - 'Exe rebuilt and smoke-tested'
  evidence_provenance: {"schema_version": 2, "head_commit": "760c0e409374f2b4b531356a2334e89cf1aa3569", "generated_plan_path": "docs/plans/2026-10-23-gitnexus-plan-root-cause-fix-plan.md", "global_dirty_digest": {"algorithm": "sha256", "canonicalization": "gitnexus-evidence-provenance-v2 NUL-framed UTF-8 records", "value": "cea5994b7b1f08f3952f4c782eaeba1147c55455e248b09f6b6bfcdcaa723fe1"}, "cited_path_manifest": [{"path": "backend/rlv_sim/config/definition.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:eba9ab010363e240ae6e70e2fae2d9a7cf74e8f35f4cdd1e2dcae9924d93b767", "index_digest": "sha256:eba9ab010363e240ae6e70e2fae2d9a7cf74e8f35f4cdd1e2dcae9924d93b767", "worktree_digest": "sha256:eba9ab010363e240ae6e70e2fae2d9a7cf74e8f35f4cdd1e2dcae9924d93b767", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/config/factory.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:5abbceb875a31b0277da681bb716fead14a185ff31aee08b5366a24191640202", "index_digest": "sha256:5abbceb875a31b0277da681bb716fead14a185ff31aee08b5366a24191640202", "worktree_digest": "sha256:e06ee184f62fc122bee3e5f63a0832faf4b428e6e174ef25c7325d42e942d5eb", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/config/field_docs.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:660f665ba22721f65dbe5b23df186700d2c82854c76d8594bc9332f4ad2284e1", "index_digest": "sha256:660f665ba22721f65dbe5b23df186700d2c82854c76d8594bc9332f4ad2284e1", "worktree_digest": "sha256:a390d82a1858f675ec23299ddf9923cb85c6398ae81461cf9a40eb922b93a291", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/guidance/ascent.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:1d38b3b4bfc0c455503dbf69645ef2289c5954f890c4f7d8b1b5facedc743281", "index_digest": "sha256:1d38b3b4bfc0c455503dbf69645ef2289c5954f890c4f7d8b1b5facedc743281", "worktree_digest": "sha256:8f41cd7a9425d8edabf51002effe1c5f85d5aebc98df4943fed2af60cf8129ae", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/guidance/booster.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:26aefac83e25d26457c6d78926b96ea31c664a88de7aac02ce045d9294d0a709", "index_digest": "sha256:26aefac83e25d26457c6d78926b96ea31c664a88de7aac02ce045d9294d0a709", "worktree_digest": "sha256:26aefac83e25d26457c6d78926b96ea31c664a88de7aac02ce045d9294d0a709", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/guidance/gfold.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:0723745a3a2ce9693870d82d157cb6541dc3ca70daf26de2376ffcb76536cf64", "index_digest": "sha256:0723745a3a2ce9693870d82d157cb6541dc3ca70daf26de2376ffcb76536cf64", "worktree_digest": "sha256:65f5f653a8644cac0ab888760f96be9958e02ca575413735182304d841621ebb", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/guidance/orbit.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:39b38cab3b90846ee0fdaf1e964a1992e87ee8fadbd73d36197ed0e68f696a95", "index_digest": "sha256:39b38cab3b90846ee0fdaf1e964a1992e87ee8fadbd73d36197ed0e68f696a95", "worktree_digest": "sha256:6425958b950605c9629c160a0e900263a8a272d033e619fa5b6fe9332cc6d664", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/guidance/plugins.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:2d24b084607864289f8a91bc55add8c199b2dd247e4c6045c24ffecb48c0696e", "index_digest": "sha256:2d24b084607864289f8a91bc55add8c199b2dd247e4c6045c24ffecb48c0696e", "worktree_digest": "sha256:2d24b084607864289f8a91bc55add8c199b2dd247e4c6045c24ffecb48c0696e", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/guidance/recovery.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:07cedb04094db2478886b82d8704a80bb7558e5ce8d4bcf7b1eb5d18ae948e26", "index_digest": "sha256:07cedb04094db2478886b82d8704a80bb7558e5ce8d4bcf7b1eb5d18ae948e26", "worktree_digest": "sha256:8dacdaf2eb883a5e13ffb640e37d3cafe0a2c645335126f98ae85bc4dd7b8cf1", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/mission/campaign.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:5199f45d3925580f9a7563b056c4718116fd55defcae3dedfed4c0f9b161982d", "index_digest": "sha256:5199f45d3925580f9a7563b056c4718116fd55defcae3dedfed4c0f9b161982d", "worktree_digest": "sha256:59c9dc1af04ea3c4d07d4a8e45d56eb34f3dbb84761d8161323c20bdcf52f169", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/mission/manager.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:19ff32de22ce37d6ccf08b8bd1572fa4f9ec855efbf81548450387a0d2b84e7d", "index_digest": "sha256:19ff32de22ce37d6ccf08b8bd1572fa4f9ec855efbf81548450387a0d2b84e7d", "worktree_digest": "sha256:de85f278a081c6a774d1debf9aac5fba993076eb3e4604a0c9bd7730be994fd9", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/mission/step.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:c6bb2f5e926fff91cf108f88fed46fa3fde8b89d18f1ad27c4dcf4d03f249a05", "index_digest": "sha256:c6bb2f5e926fff91cf108f88fed46fa3fde8b89d18f1ad27c4dcf4d03f249a05", "worktree_digest": "sha256:8625a95b118f9b5092fbf2457e01863130188eb8edf179fb32facc182ad33315", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/mission/vehicle_split.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:1280f422357428839a619c227f7b8fe85b50f590900566313397e4e159e8e090", "index_digest": "sha256:1280f422357428839a619c227f7b8fe85b50f590900566313397e4e159e8e090", "worktree_digest": "sha256:4a83df0d841d512983a2f867d0482ae41befc4767ad25bfe2420e9b8f678c92d", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/outputs/model_card.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:bff9ec146bb7607e062eb7349fa92f99128d24bac80f1a850667147ece97485a", "index_digest": "sha256:bff9ec146bb7607e062eb7349fa92f99128d24bac80f1a850667147ece97485a", "worktree_digest": "sha256:bff9ec146bb7607e062eb7349fa92f99128d24bac80f1a850667147ece97485a", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/outputs/summary.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:ffb3f11068b752dc5ac0c7b55f33df75d5dcd4c6bf0fe4bb2a5a0d7cbd93b369", "index_digest": "sha256:ffb3f11068b752dc5ac0c7b55f33df75d5dcd4c6bf0fe4bb2a5a0d7cbd93b369", "worktree_digest": "sha256:050ddab8e025efbcc4925f3da3b28a5536d608f6bc3825b8b8d4bcc106fc4a5e", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/physics/dynamics.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "unstaged", "rename_from": null, "rename_to": null, "head_digest": "sha256:21e9304e1a517cb0e3f668cd8d5360fd40b8f2c87656198807557a573945b6f8", "index_digest": "sha256:21e9304e1a517cb0e3f668cd8d5360fd40b8f2c87656198807557a573945b6f8", "worktree_digest": "sha256:53187f9e1fd47a40ac1d14e922cabee2379798771c4b0df630b64fda1f6f84a1", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/physics/forces.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:60c9b81cce1cde330dca3947efaa5562a956aaa15c738c3eb02cca0adc0c8a40", "index_digest": "sha256:60c9b81cce1cde330dca3947efaa5562a956aaa15c738c3eb02cca0adc0c8a40", "worktree_digest": "sha256:8762096d0dd27adfb517649b0f8d91624a4c0ac3b4d9c5bb5d5403f35040a7f2", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/physics/integrators.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:bba3b03d03697f62a6330eb5e581887e2620a2826a33c15549f86f4e12f5bb9c", "index_digest": "sha256:bba3b03d03697f62a6330eb5e581887e2620a2826a33c15549f86f4e12f5bb9c", "worktree_digest": "sha256:a5d96eb38b80d8acf8f0f7dd6889ffb27757431d7b53b2fb7627c679427f9d71", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/physics/mass.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:606675ac2c46a17e6b3babb496fe33d7d75eee06e53a0125af1ef23c1826cce2", "index_digest": "sha256:606675ac2c46a17e6b3babb496fe33d7d75eee06e53a0125af1ef23c1826cce2", "worktree_digest": "sha256:1c2d2c1ce42894346831dbc654d1c0361e1f384c4111d9470f9de03612ab3dca", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/physics/numba_kernels.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:934dc499deb477d8505a3bd366b5fed4a236b7d9601c587241f5a0581755b03d", "index_digest": "sha256:934dc499deb477d8505a3bd366b5fed4a236b7d9601c587241f5a0581755b03d", "worktree_digest": "sha256:04bf3f18066d9b8d7f2050e92fbfae7167f47445ecfe46dffde8cf35624b83bb", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/server/app.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:fc39f79d12d594853e25260e7b2dae86ffa3a9312fb3ed607c1d22fbd41d6c67", "index_digest": "sha256:fc39f79d12d594853e25260e7b2dae86ffa3a9312fb3ed607c1d22fbd41d6c67", "worktree_digest": "sha256:1c88380ae88898aa9a935cb449eb3ef738060c794a5ed2a79d9f1cc935e44ce5", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/validation/drag_fit.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:3aac0254c5465a317d18e5775078882f79c1dd0ad1c9820edb7083fc6ccf9f26", "index_digest": "sha256:3aac0254c5465a317d18e5775078882f79c1dd0ad1c9820edb7083fc6ccf9f26", "worktree_digest": "sha256:7c9621ba9ce8f548cc948fece81c6cd9c8129640def2e03707413239d54bac43", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/validation/flight_compare.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:6937e25264790f8be3917a46d3d2160f3d4c7d07cd3c5aaee1421d5fb2fbef49", "index_digest": "sha256:6937e25264790f8be3917a46d3d2160f3d4c7d07cd3c5aaee1421d5fb2fbef49", "worktree_digest": "sha256:b209674938e167d6143a3273ed2ef34661e6b53f1c1dfa648f6b6d2bcddf9407", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/vehicle/actuator.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:fd4eaa21c08488fbba2660dec4e6a0673d1724b68dc91e99d6369ce8e63ca6bc", "index_digest": "sha256:fd4eaa21c08488fbba2660dec4e6a0673d1724b68dc91e99d6369ce8e63ca6bc", "worktree_digest": "sha256:5bcad139304bd3ea28ce168a7c8d2d23f6fff0775e625960b4f6344b73f191ef", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/vehicle/control.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:6a538192b7a6ccb5f058bbe5e1528d33b784090235c3184ba9eee855fe5d98dc", "index_digest": "sha256:6a538192b7a6ccb5f058bbe5e1528d33b784090235c3184ba9eee855fe5d98dc", "worktree_digest": "sha256:e81c394055e3b6605c859324c0482f72792110365420f06c96d99ee73a0bf5c7", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/vehicle/navigation.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:a905f0fb078cad1f6c7cce6a3b457174d6af83b1b6a1f73d615cdfbbd75454c3", "index_digest": "sha256:a905f0fb078cad1f6c7cce6a3b457174d6af83b1b6a1f73d615cdfbbd75454c3", "worktree_digest": "sha256:5358337fb07fac19c8cece6aa2fc5dcf9a794b871293316b44e1882fe802f6a9", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/vehicle/rcs.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:f23fef92c8716872ef51317573641c89b33512e1f122521fe40e0b5d8da57b9a", "index_digest": "sha256:f23fef92c8716872ef51317573641c89b33512e1f122521fe40e0b5d8da57b9a", "worktree_digest": "sha256:064c0f08211c8bb1436eedd471da900b55d75678f01e0f434fc0868d436e90b4", "untracked_digest": "absent"}, {"path": "backend/rlv_sim/vehicle/recovery_hardware.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:8b2a0b7906a91c03c69ae704a63be4e86b6e86c57b6060513e122c998d5a10da", "index_digest": "sha256:8b2a0b7906a91c03c69ae704a63be4e86b6e86c57b6060513e122c998d5a10da", "worktree_digest": "sha256:008a5009e6be9a58fb1555c3c0192c46522eb265f6c4dfb60602be0a2acf9ea7", "untracked_digest": "absent"}, {"path": "backend/tests/test_jet_damping.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "unstaged", "rename_from": null, "rename_to": null, "head_digest": "sha256:6c55e43cd1140d0cc53956377dadad92436b1e9416862485ae992f07e7f514dd", "index_digest": "sha256:6c55e43cd1140d0cc53956377dadad92436b1e9416862485ae992f07e7f514dd", "worktree_digest": "sha256:dafa9d527f3e70418865e6f767ade285dfab508d3f9b7625ac22c32d5b5f3892", "untracked_digest": "absent"}, {"path": "backend/tests/test_research_features.py", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "unstaged", "rename_from": null, "rename_to": null, "head_digest": "sha256:ebf8a05e11ee85415e46853ff3e9468ab3aa1a9242a93977608a3d5fcef1674e", "index_digest": "sha256:ebf8a05e11ee85415e46853ff3e9468ab3aa1a9242a93977608a3d5fcef1674e", "worktree_digest": "sha256:d9ec8d24f50fe1cee3741bccc2c07a133f68e327d12df14d8875fb64f954e0f8", "untracked_digest": "absent"}, {"path": "desktop/desktop_app.spec", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:38f4da4e912f2eb12f7af522772671591614a10dae2dabcc0819822baf53c02f", "index_digest": "sha256:38f4da4e912f2eb12f7af522772671591614a10dae2dabcc0819822baf53c02f", "worktree_digest": "sha256:66d829fdcde5acce9f82d9b31517ca7a5a7c3d135a8c5a5ff9e8e6b915a19742", "untracked_digest": "absent"}, {"path": "docs/MATHEMATICAL_REFERENCES.md", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:dd715d1ab8d5b59d9926e71a94cb4fca6c1d18a0cc2b53aa8dc00037a5a1a444", "index_digest": "sha256:dd715d1ab8d5b59d9926e71a94cb4fca6c1d18a0cc2b53aa8dc00037a5a1a444", "worktree_digest": "sha256:a6b91f57b06031e9291813d80adf505d7dd095379d96b93e4f9f7dd6eb59a241", "untracked_digest": "absent"}, {"path": "docs/MONTE_CARLO_RESULTS.md", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:c6e996a92c0af67a9c90bef67cc29bc7969c5467e61b7f7a49c0fc85578aa99d", "index_digest": "sha256:c6e996a92c0af67a9c90bef67cc29bc7969c5467e61b7f7a49c0fc85578aa99d", "worktree_digest": "sha256:e5fb2f0e75d986fe68af9370cdba103a9673b960535c4f0d7716977dbdfe6d0b", "untracked_digest": "absent"}, {"path": "docs/VALIDATION.md", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:99d6a1565d251f3a5aa8d87b0fca9e61e4eb90e67fb7e2c752b96bcb631acf7f", "index_digest": "sha256:99d6a1565d251f3a5aa8d87b0fca9e61e4eb90e67fb7e2c752b96bcb631acf7f", "worktree_digest": "sha256:e61ed131a49792198905f862fd8c6ad93a67b7851e864c8693c5fb9d95ae27d1", "untracked_digest": "absent"}, {"path": "docs/VALIDATION_DRAG.md", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:fb26ccbfb4cf51af954b06b99a8f65e6b549e3c9f786bdf73e8f1961576471a2", "index_digest": "sha256:fb26ccbfb4cf51af954b06b99a8f65e6b549e3c9f786bdf73e8f1961576471a2", "worktree_digest": "sha256:35c940f5bffdc44186aacb78f11cb24390f5ff41bc919b37f1d68af2909bfba5", "untracked_digest": "absent"}, {"path": "docs/theory/CODE_FINDINGS.md", "object_kind": {"head": "absent", "index": "absent", "worktree": "absent", "untracked": "regular"}, "state": "untracked", "rename_from": null, "rename_to": null, "head_digest": "absent", "index_digest": "absent", "worktree_digest": "absent", "untracked_digest": "sha256:258b7d5c91570ca99677beb6143f149ea087c4ccec8fe7d48cd552e32773b81d"}, {"path": "frontend/js/app.js", "object_kind": {"head": "regular", "index": "regular", "worktree": "regular", "untracked": "absent"}, "state": "clean", "rename_from": null, "rename_to": null, "head_digest": "sha256:1122c2189f199d22241669da3249b02ba4af9f16f0f2d8bd79c49a3c3762c2c9", "index_digest": "sha256:1122c2189f199d22241669da3249b02ba4af9f16f0f2d8bd79c49a3c3762c2c9", "worktree_digest": "sha256:1122c2189f199d22241669da3249b02ba4af9f16f0f2d8bd79c49a3c3762c2c9", "untracked_digest": "absent"}]}
  primary_symbols:
    - {symbol: _booster_landing_success, file: backend/rlv_sim/mission/step.py, lines: '85-93', role: 'string-prefix outcome flag (RC3)'}
    - {symbol: propagate impact time, file: backend/rlv_sim/guidance/recovery.py, lines: '425-436', role: 'impact-time interpolation bug (RC6)'}
    - {symbol: compute_inertia_tensor, file: backend/rlv_sim/physics/mass.py, lines: 'see file', role: 'constants-sourced mass properties (RC1)'}
    - {symbol: _EGM96_C, file: backend/rlv_sim/physics/forces.py, lines: '66, 434', role: 'coefficients and ECI longitude (RC6)'}
    - {symbol: separation delta-v/omega, file: backend/rlv_sim/mission/vehicle_split.py, lines: '45-58', role: 'tumble frame and momentum (RC6)'}
  related_symbols:
    - {symbol: propagate_to_surface, relationship: 'duplicate of the recovery predictor', relevance: 'numba_kernels.py:258 has the same bug'}
    - {symbol: hard_landing_speed_limit, relationship: 'CALLED BY step.py:491, summary.py:234', relevance: 'single outcome threshold'}
    - {symbol: C.MAX_GIMBAL_ANGLE, relationship: 'read at step.py:244,277,640', relevance: 'replace with config'}
    - {symbol: grid fin drag, relationship: 'forces.py:1071-1076 and recovery_hardware.py:258-268', relevance: 'double count'}
  execution_path:
    - 'full_mission -> step -> guidance -> control/actuator -> integrators.RK4 -> dynamics -> forces/mass -> step contact check -> summary.assess_full_mission'
  pdg_constraints: []  # no PDG layer; ordering constraints in plan section 5
  architectural_patterns:
    - {pattern: 'new config field keeps current default, validated, with a field_docs line', example_location: backend/rlv_sim/config/definition.py, usage_guidance: 'AGENTS.md conventions'}
    - {pattern: 'retired fields tolerated on load', example_location: backend/rlv_sim/config/factory.py RETIRED_FIELDS, usage_guidance: 'use when removing dead settings'}
  files_to_modify: 'see plan section 10'
  tests: 'see plan section 8 scenario lists'
  verification_commands:
    - 'cd backend && .venv/Scripts/python -m pytest -q'
    - 'cd backend && .venv/Scripts/python -m pytest -m slow'
    - 'cd backend && .venv/Scripts/python -m ruff check .'
    - 'cd backend && .venv/Scripts/python -m rlv_sim.mission.campaign --mode monte-carlo --runs 100 --seed 42 --workers 10'
  risks:
    - 'Success metrics and published numbers shift; republish only after step 13'
    - 'Removed fields break old scenario files unless added to RETIRED_FIELDS'
    - 'Grep-only blast radius; index with gitnexus before steps 5-7'
  assumptions:
    - 'Chapter-writer findings marked [inferred] are re-read at the cited line before each fix'
    - 'Official EGM96 coefficient file is obtainable (check NGA/NASA source)'
  open_questions: 'see plan section 12'
  avoid:
    - 'Do not repeat full repository discovery; start from docs/theory/CODE_FINDINGS.md'
    - 'Do not regenerate regression references or Monte Carlo baselines before step 13'
    - 'Do not change a default without a Monte Carlo comparison (AGENTS.md)'
    - 'Do not fix the Python predictor without the numba twin in the same commit'
```

## 12. Assumptions and Open Questions

**Assumptions:**
- `[assumed]` Items tagged [inferred] come from the chapter writers' source reads (Ch1–Ch11), not re-read during this planning run. The executor re-verifies each at its line before fixing.
- `[assumed]` The EGM96 discrepancies (S32/S33 sign, C53/C62 missing) were checked by the Ch3 writer against web copies, not the official file.

**Open questions (owner decisions):**
1. `target_inclination_deg`: implement azimuth steering (real feature), or remove it from API/UI?
2. `stage_sep_velocity`: what should it mean (an MECO speed target?), or should it be removed?
3. Grid fins: keep the explicit fin-force model and drop the drag-scale contribution, or the reverse? Either way, re-fit the 0.68 drag scale afterwards.
4. Skin temperature: implement a lumped model, or remove the claims and columns?
5. Aero deck Cm: adopt the aircraft convention, which breaks any user decks made under the current sign?
6. `enable_high_fidelity_gram` / `atmosphere_model`: wire or remove?
7. Abort modes: keep them as terminate-only, or add flown abort trajectories (large)?

**Deferred** (not root-cause fixes; separate projects): guidance using estimated rather than true drag/wind (online estimation), flown abort trajectories, a full INS/GNSS filter, per-engine thrust and engine-out, slosh and flexible-body dynamics.

## 13. Definition of Done

- Every `CODE_FINDINGS.md` entry is marked fixed (with a commit) or decided-limit (with a doc line).
- The guard tests pass with no xfail entries left.
- The fast suite, the full slow suite and ruff all pass on the final commit.
- Regression references, VALIDATION.md, VALIDATION_DRAG.md, MONTE_CARLO_RESULTS.md, `model_card.py`, AGENTS.md and theory chapters 1–11 reflect the re-baselined results.
- The exe is rebuilt from the final commit; health, schema, plugins, model and validation endpoints respond inside it.

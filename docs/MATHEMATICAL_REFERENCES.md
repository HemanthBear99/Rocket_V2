# Mathematical References and Model Validity Limits

Provenance record for the physics in this package. Nine modules cite it as the
authority for their equation forms and validity limits (`forces.py`,
`dynamics.py`, `mass.py`, `control.py`, `integrators.py`, `navigation.py`,
`recovery.py`, `_guidance_ascent.py`, `_guidance_booster.py`,
`_guidance_orbit.py`). Each module docstring names the section that applies.

**Scope.** Records (a) which equation forms are standard textbook mechanics,
(b) which are published reference data, and (c) which are project assumptions
or engineering approximations. It does **not** claim a validation report against
flight data — none exists for this vehicle.

## 1. Six-degree-of-freedom rigid-body dynamics

`dynamics.py`: rotational `I*omega_dot = tau - omega x (I*omega)`, translational
`r_ddot = F_total / m`, quaternion `q_dot = 0.5 * Omega(omega) * q`.

**Validity limit.** Changing inertia (`I_dot` from propellant flow) is a
first-order approximation, **not** a complete variable-mass angular-momentum
derivation: a real vehicle also exchanges angular momentum with departing
propellant. Equation forms correct; inertia model simplified.

## 2. Numerical integration

`integrators.py`: classical RK4, quaternion renormalized after each sub-step,
plus an exact-time sub-step split when a step straddles propellant exhaustion
(recursion depth bounded at 2).

**Validity limit.** Fixed-step, no error estimator, no adaptive sub-stepping.
Timestep adequacy is empirical (phase caps in
`MissionManager.recovery_max_step_dt`, `C.STACKED_ASCENT_MAX_DT`), not a formal
convergence study.

## 3. Earth gravity

`forces.py`: `"central"` `-mu*r/|r|^3`; `"j2"` central plus second-degree
zonal; `"egm96"` spherical harmonics through degree/order 6.

**Validity limit.** EGM96 coefficients are public NASA/NGA data but are
**truncated to degree/order 6** for runtime; the full model runs to 360. The
degree-2 case reproduces the `j2` formulation exactly.

## 4. Standard atmosphere

US-76 layer structure, optional analytic extension above 84.852 km, optional
`enable_high_fidelity_gram` density modifier.

**Validity limit.** Layers and lapse rates are published data. The upper
extension is an isothermal exponential approximation, not a thermospheric model.
The GRAM modifier is a smooth analytic scaling, **not** the GRAM model.

## 5. Aerodynamics and ascent guidance

Drag, lift, normal force, pitching moment. Coefficients come from an optional
tabular deck or from `MACH_BREAKPOINTS`/`CD_VALUES`/`CL_ALPHA_VALUES`.

**Validity limit.** Mach tables are **engineering approximations, not vehicle
test data**. The `aero_geometry` deck is a real derivation from vehicle
geometry, but its `length_m` default of 47 m is a documented estimate.

Post-stall uses `compute_post_stall_normal_coefficient`: linear below stall
onset, rising monotonically to a bounded ceiling above it. The earlier form
blended an unbounded linear term against sin-squared cross-flow and was
## 6. Propulsion and variable mass

`mass.py`: `mdot = T / (Isp * g0)` with a dry-mass floor. Stage-2 flow always
uses the same thrust/Isp pair as the force model, so effective Isp cannot drift
from configuration.

**Validity limit.** Component inertias and CG stations are project assumptions.
Thrust is a **single lumped magnitude** per stage, not nine discrete engines: no
per-engine throttling, no out-of-family detection, no cluster behaviour.

The TVC moment arm is the distance from the engine gimbal plane to the CG. The
historical constant (21 m, booster) implies a gimbal envelope several times
larger than any flown vehicle and yields ~47 rad/s^2 roll authority at landing
thrust. `tvc_lever_arm_m` makes this a measurable sensitivity; physical
Falcon-class values are a few metres.

## 7. Aerothermal heating

`compute_aerodynamic_heating` uses Sutton-Graves stagnation-point convection.

**Validity limit.** Convective component only; radiation cooling is not
modelled. Skin temperature is a lumped first-order lag, not a conduction model.

## 8. Recovery guidance

`recovery.py`: two-body propagation, work-energy and rocket equation for
boostback targeting, ballistic impact prediction, suicide-burn ignition.

**Validity limit.** The impact predictor integrates at fixed `dt = 2.0` s,
`max_steps = 300`, falling back to a drag-free straight line when exhausted with
no warning. Grid-fin authority is a flat-plate `cl_max` over a lumped area that
is **generous** versus a real titanium cascade fin, so recovery lateral
authority is an optimistic bound.

## 9. Mission phase transitions

`mission_manager.py` advances on **physical** criteria — propellant depletion,
radial-velocity sign change at apogee, entry-interface altitude, suicide-burn
ignition — not elapsed time. Recovery phases cap the integration step.

**Validity limit.** Inherits every approximation above.

## 10. GPS/INS navigation

`navigation.py`: GPS, IMU propagation, landing altimeter, configurable sigmas
and biases, selectable via `--nav-mode`.

**Validity limit.** Noisy-measurement and dead-reckoning demonstration, not a
complete INS/GNSS solution. No alignment, no fusion filter, no Earth-rate
correction; errors grow unphysically with time.

## 11. Powered descent

Terminal guidance is ZEM/ZEV-style with a suicide-burn ignition solution,
propellant reserves, and grid-fin / landing-leg deployment state machines.

**Validity limit.** Tuned for this vehicle. Entry-descent time-to-go must reflect
real ballistic time-to-impact; an altitude/descent-rate estimate over-commands
the `6/t^2` ZEM gain several-fold and saturates the fins mid-descent.

## Appendix: what is NOT claimed

- No published validation report exists against external reference data.
- Mach tables, grid-fin/leg drag scaling and the GRAM modifier are engineering
  approximations.
- Stage-2 recovery (`enable_s2_recovery`) is experimental and propellant-starved
  by default.
- Geometry and masses correspond closely to a Falcon 9 v1.0-class booster, but
  this is a research model and is not flight-qualified software.
non-monotone (peaking near 25 deg, falling ~50% by 45 deg), giving negative
aerodynamic stiffness in deep stall.

`_guidance_ascent` is explicitly **not** IGM/PEG/collocation. `_guidance_orbit`
is project steering logic. `_guidance_booster` is ZEM/ZEV-inspired with
constants tuned for this vehicle.
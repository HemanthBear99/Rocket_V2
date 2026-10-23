# Vyom theory manual

What the simulator computes, the equations it uses (with source line references), where each model comes from, its parameters, diagrams, and where it stops being valid. Equations are LaTeX and diagrams are Mermaid; both render on GitHub.

Every model is labelled as one of:
- **Standard**: textbook mechanics.
- **Published**: a named paper or data set.
- **Project heuristic**: tuned for this vehicle, with no external source.

| # | Chapter | Covers |
|---|---|---|
| 1 | [Overview and architecture](01_overview_and_architecture.md) | Mission flow, per-step loop, frames, phase state machines, success criteria |
| 2 | [Dynamics and integration](02_dynamics_and_integration.md) | 6-DOF equations, quaternions, variable mass, jet damping, RK4 |
| 3 | [Environment](03_environment.md) | Earth model, central/J2/EGM96 gravity, US-76 atmosphere, wind |
| 4 | [Aerodynamics and heating](04_aerodynamics_and_heating.md) | Coefficients, decks, post-stall, CP, grid fins, legs, Sutton-Graves |
| 5 | [Propulsion, mass, actuators](05_propulsion_mass_actuators.md) | Thrust, mass flow, reserves, CG/inertia, throttle, TVC, RCS |
| 6 | [Ascent and orbit guidance](06_ascent_and_orbit_guidance.md) | Pitchover, gravity turn, max-q, MECO, separation, insertion |
| 7 | [Booster boostback and entry](07_booster_boostback_and_entry.md) | Flip, impact prediction, boostback steering, entry burn, grid fins |
| 8 | [Landing guidance](08_landing_guidance.md) | Ignition, vertical law, ZEM/ZEV, position hold, G-FOLD, plugins |
| 9 | [Attitude control](09_attitude_control.md) | Quaternion error, PD/PID, gain scheduling, torque allocation |
| 10 | [Navigation, aborts, assessment](10_navigation_aborts_assessment.md) | Sensor models, abort triggers, pass/fail gates |
| 11 | [Monte Carlo and validation](11_monte_carlo_and_validation.md) | Dispersions, statistics, Falcon 9 comparison, credibility |

[CODE_FINDINGS.md](CODE_FINDINGS.md) lists the inconsistencies and possible bugs found while writing the manual, for review. [STYLE.md](STYLE.md) holds the writing rules.

# Theory manual: writing guide

Shared rules for every chapter of the Vyom theory manual (`docs/theory/`). The
reader is an aerospace engineer or researcher who has never seen the code and
must be able to (a) understand what is simulated and how, (b) check every
equation against the source, and (c) see what is textbook, what is published
research, and what is a project-specific heuristic.

## Ground rules

1. **Document what the code does, not what it should do.** Every equation must
   be the one the code implements, with its source location
   (`backend/rlv_sim/physics/dynamics.py:142`). If the code differs from the
   textbook form, give the textbook form, then the implemented form, and say
   why or note "difference not explained in code".
2. **Classify every model**: *Standard* (textbook mechanics), *Published*
   (a named paper or data set), or *Project heuristic* (tuned for this
   vehicle, no external source). Heuristics are fine; hiding them is not.
3. **References must be real.** Cite only works you are certain exist, with
   authors, year, title and venue (and DOI only if you are sure of it). If
   no published source applies, write "Project heuristic; no external
   reference". Never invent a citation, page number or DOI.
4. **Numbers come from the code.** Default values come from
   `create_default_config()` (dump: `C:/Users/appal/AppData/Local/Temp/claude/d--rlv-sim/0f6dea4d-7ce0-46c3-8de7-705cc7687aab/scratchpad/default_config.json`)
   or `core/constants.py`. Quote the config field name next to each tunable.
5. **Do not edit any code.** Write only your own chapter file.

## Structure of a chapter

```
# N. Title
> One-paragraph summary: what this subsystem does in the mission and its fidelity level.

## N.1 Role in the simulation      (where it is called from, inputs, outputs, a diagram)
## N.2 ... one section per model/law
    - Physical idea in plain words (2-5 sentences)
    - Equations (implemented form), every symbol defined, units
    - Implementation notes (source file:line, discrete-time details, limits, clamps, fallbacks)
    - Parameters table: symbol | config field / constant | default | unit | meaning
    - Classification (Standard / Published / Project heuristic) + references
    - Validity limits and known issues
## N.x Verification            (tests that cover it: tests/test_*.py names; validation results if any)
## N.y References              (numbered list, real works only)
```

## Notation (use consistently)

| Symbol | Meaning |
|---|---|
| $\mathbf{r}, \mathbf{v}$ | position, velocity in ECI (m, m/s) |
| $\mathbf{v}_{rel}$ | velocity relative to the rotating atmosphere / ground |
| $\mathbf{q} = [q_w, q_x, q_y, q_z]$ | body-to-ECI attitude quaternion (scalar first) |
| $\boldsymbol{\omega}$ | body angular rate (rad/s), body frame |
| $m, \dot m$ | mass (kg) and mass rate ($\dot m < 0$ when burning) |
| $\mathbf{I}$ | inertia tensor, body frame (kg m²) |
| body $+z$ | thrust axis (nose direction); $x, y$ transverse |
| $h$ | altitude above the reference surface (m) |
| $\hat{\mathbf{u}}$ | local vertical (up) unit vector |
| $T, I_{sp}, g_0$ | thrust (N), specific impulse (s), 9.80665 m/s² |
| $\bar q = \tfrac12 \rho v_{rel}^2$ | dynamic pressure (Pa) |
| $t_{go}$ | time to go (s) |

Vectors bold, unit vectors with a hat, scalars italic. SI units throughout;
state units in every table. Write equations in LaTeX: inline `$...$`, display
`$$...$$` on their own lines (GitHub renders both).

## Diagrams

Use Mermaid code blocks (```` ```mermaid ````), which GitHub renders. Every
chapter needs at least one diagram; good choices are flowcharts of the
decision logic (`flowchart TD`), state machines (`stateDiagram-v2`), sequence
of calls per step (`sequenceDiagram`) and block diagrams of control loops
(`flowchart LR`: reference → controller → actuator → plant → sensor →
feedback). Keep node labels short; put math in the text, not in node labels
(no `$` inside Mermaid). Quote labels containing parentheses or punctuation:
`A["Thrust (N)"]`.

## Tone

Plain, precise, past-the-jargon. Define an acronym on first use (ZEM/ZEV, MECO,
RTLS, TVC). Prefer short paragraphs and tables over long prose. No marketing.

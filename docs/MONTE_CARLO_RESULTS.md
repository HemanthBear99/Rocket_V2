# Monte Carlo results: research-driven fixes (2026-10-22)

All campaigns: `python -m rlv_sim.mission.campaign --mode monte-carlo --runs 100 --seed 42`,
legacy six dispersions, heuristic landing guidance. 100 runs cannot show high reliability:
the one-sided 90% Clopper-Pearson lower bound is reported beside each rate, and showing
99.73% at 90% confidence needs 852 failure-free runs.

| Configuration | Success | 90% lower bound | Orbit | Remaining failures |
|---|---|---|---|---|
| Old roll jet damping (documented baseline) | 94% | — | 100% | headwind starvation |
| Per-axis jet damping | 93% | — | 100% | 5 pad misses of 50–70 m, 2 headwind starvations |
| + wind-aware reserve (383 kg per m/s headwind) | 95% | 90.9% | 100% | 5 pad misses of 50–70 m |
| + terminal position hold, tau floor 3 s (first version) | 95% | 90.9% | 100% | 3 headwind crashes, 1 hard landing, 1 miss |
| + both (first version) | 97% | 93.4% | 100% | 1 hard landing, 2 misses of 50 m |
| + both, hold v2: covers pad-capture branches, 30 m fade, vertical-priority gate, closing-speed cap **(default)** | 98% | 94.8% | 100% | 1 hard landing (7.9 m/s on pad), 1 miss of 51 m with 0.5 t left |
| Realistic descent drag 0.68, current guidance | 87% | — | 100% | 13 tailwind pad misses with 4–10 t propellant left |
| Drag 0.68 + 80% planning throttle | 84% | — | 100% | earlier ignition wastes propellant; crashes |
| Drag 0.68 + both fixes (first version) | 90% | 85.0% | 100% | hard landings and crashes; p99.865 touchdown 26.8 m/s |
| Drag 0.68 + both, hold v2 (default) | 88% | 82.7% | 100% | 10 soft (4.4–4.9 m/s) off-pad landings 53–376 m out, 1 hard landing (7.3 m/s), 1 starvation crash |

Hold v2 rows ran with numba installed and `--workers 10` (about 12 min per 100 runs instead of 35);
numba matches pure Python to round-off.

## Findings

- **Roll jet damping** was overstated 100–750x (it used the CG-to-nozzle distance for roll).
  The per-axis model changes mission success within noise (94% → 93%), so it is kept.
- **Headwind starvation** is a reserve problem. Booster propellant left falls ~383 kg per
  m/s of headwind (fit over the 100 cases). Holding that back at MECO from the forecast wind
  (`recovery_reserve_wind_sensitivity_kg_per_mps`) removed every starvation failure without
  costing orbit success.
- **Pad misses** come from the final approach: below 150 m the guidance only nulls horizontal
  velocity, so the position error at 150 m becomes the miss. The first position hold
  (`booster_terminal_position_hold_tau_s`) fixed most of them at nominal drag but never ran
  inside the pad-capture branches, so a fast arrival still coasted past the pad (the
  per-axis jet damping made one such case, `test_tailwind_touchdown_has_no_lateral_limit_cycle`,
  miss by 52 m). With realistic drag it also chased far-off pads faster than the 15° tilt
  cone could brake, causing 12–30 m/s impacts. Hold v2 covers the capture branches, fades
  into velocity nulling between 60 m and 30 m (`booster_terminal_position_hold_floor_m`),
  yields to vertical priority, and caps closing speed at half what the tilt-limited thrust
  can brake before the floor. It is now the default.
- **Realistic-drag losses start above the final approach.** With drag 0.68 the failing
  tailwind cases reach 250 m altitude 100–380 m off target. The hold now lands them softly
  off the pad instead of crashing, but closing that gap needs entry or landing-burn changes.
- **The realistic-drag loss is not an ignition-timing problem.** The ignition estimator and
  burn law already recompute every step and the predictor uses the true drag scale. The
  research report's "unlatch and plan at 70–80% throttle" fix was tested and made things worse.

## Open work

- Realistic-drag tailwind cases arrive 100–380 m off target at 250 m; fix the divert
  above the final approach (entry and early landing burn).
- Drag-aware G-FOLD (successive convexification) — not attempted.
- Separate guidance drag belief from truth plus online drag estimation; today guidance knows
  the true drag scale, which flatters robustness.
- Release-size campaigns (≥2,000 runs) and the extended dispersion set (`--dispersions extended`).

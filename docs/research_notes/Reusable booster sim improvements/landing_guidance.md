# Powered-Descent / Propulsive Landing Guidance Robust to Drag Uncertainty and Wind

Context: the simulator uses a latched suicide-burn heuristic (a_z = g + (v^2 - v_t^2)/(2h)) with ZEM/ZEV lateral divert, plus optional G-FOLD (cvxpy). With realistic drag (scale 0.68), Monte Carlo success drops from 94% to 86%.

## G-FOLD and successors: how they include drag/aero, and replanning rates

### Takeaway
Lossless convexification (G-FOLD) is exact only for drag-free dynamics, which is why it was built for Mars. For Earth boosters, aero forces dominate. The published path is successive/sequential convexification (SCvx/SCP), which handles drag, lift and free ignition time with solve times under 1 s. In closed loop it is used either for periodic replanning or as a planner paired with a fast MPC tracker (about 4 Hz).

### Cited Findings
- Açıkmeşe & Ploen (2007, JGCD 30:1353–1366) relax the non-convex minimum-fuel powered descent problem into an SOCP. The relaxation is lossless: no feasible part of the original problem is removed, and the global optimum is guaranteed. — [UWaterloo copy](https://ece.uwaterloo.ca/~ece602/Projects/2016/Papers/paper9.pdf); [NASA NTRS](https://ntrs.nasa.gov/citations/20120001230)
- Blackmore, Açıkmeşe & Scharf (2010, JGCD 33:1161–1171) extend this to the case where the target is infeasible. The guidance then minimizes the final distance to the target, then fuel. — [NASA NTRS](https://ntrs.nasa.gov/citations/20120001230); [patent US8489260](https://patents.google.com/patent/US8489260)
- Carson, Açıkmeşe & Blackmore later extended the lossless relaxation to thrust-pointing constraints. — [ACC 2011](https://skoge.folk.ntnu.no/prost/proceedings/acc11/data/papers/1020.pdf)
- Blackmore (NAE Frontiers of Engineering, 2016 symposium, published 2017) makes these points:
  - On Earth, "aerodynamic forces become the primary concern," unlike on Mars, where drag can be neglected in planning.
  - F9R (about 35 t) sees a peak reentry deceleration of about 6 g.
  - SpaceX generates onboard convex solvers with CVXGEN.
  - Divert trajectories must be computed "in a fraction of a second."
  - Dispersions grow during entry because of winds and atmospheric uncertainty, and are removed at boostback and at the landing burn.
  - [NAP ch.10](https://www.nationalacademies.org/read/23659/chapter/10)
- Szmuk & Açıkmeşe (2016, AIAA GNC) present "Successive Convexification for Fuel-Optimal Powered Landing with Aerodynamic Drag and Non-Convex Constraints." — [CPS-VO](https://archive.cps-vo.org/node/51778)
- Szmuk, Reynolds & Açıkmeşe (arXiv 2018; JGCD 43:1399–1413, 2020) give real-time 6-DoF SCvx with these features:
  - a tractable lift and drag model
  - free ignition time
  - state-triggered constraints, such as a velocity-triggered angle-of-attack limit
  - maximum execution time under 0.7 s across feature combinations
  - [arXiv 1811.10803](https://arxiv.org/abs/1811.10803)
- A 2026 arXiv paper (Condensed PIPG SCP) notes that during retropropulsive Earth descent, aerodynamic axial force, lift and side force "may be comparable with the commanded thrust." Exact convex reformulation is then impossible, so sequential convexification is required. — [arXiv 2608.15582](https://arxiv.org/pdf/2608.15582)
- Model Predictive Guidance for RLV landing (arXiv 2405.01264, 2024) uses two SCP problems:
  - A 3-DoF fuel-optimal planner (SOCP) with free ignition time and burn duration. It runs at the start and is re-run only when tracking error exceeds 10 m.
  - A receding-horizon tracker (QP) that models pitch, yaw and throttle as first-order lags. It updates at 4 Hz.
  - Lift is compensated for TVC moment-trim (C_L' = C_L − (l_cp/l_c)·C_z).
  - Solve times: the planner takes under 0.75 s (0.215 s nominal). The tracker takes under 50 ms (12 ms average, 34 ms peak). The authors warn that avionics could be up to 10x slower.
  - [arXiv 2405.01264](https://arxiv.org/html/2405.01264v1)

### Inferences
- The current G-FOLD (lossless convexification) mode probably ignores drag, or treats it as a fixed disturbance. With drag about 1.5x too high, plans are biased: the planner relies on drag deceleration that the real vehicle does not get. Recommended fix: SCvx with linearized drag about the previous iterate, a free ignition time, and replanning or MPC tracking at 1–4 Hz.
- A cheaper fix is to keep the lossless-convexification SOCP but feed in drag acceleration from the previous solution as a known time-varying term, and iterate 2–3 times. This is effectively SCvx without trust regions.

### Gaps
- No published solve-rate or flight-software details for SpaceX's actual landing guidance beyond CVXGEN usage.
- I could not extract the full text of the 2018 SCvx paper (the PDF was unreadable), so its exact drag formulation is not recorded here.

## Polynomial / E-guidance / Apollo-style laws and closed-loop handling of model mismatch

### Takeaway
Apollo-style polynomial laws and ZEM/ZEV laws are closed-form and drag-free. Their robustness comes only from recomputing them every cycle (feedback). Published comparisons show they lose accuracy against aero-aware guidance in Earth landing, by roughly 4 m vs 0.6 m in one study.

### Cited Findings
- Apollo E-guidance assumes thrust acceleration is linear in time. The Apollo descent law assumes it is quadratic. Both are valued for their closed-form simplicity. — [arXiv 2511.18603](https://arxiv.org/pdf/2511.18603)
- A modified polynomial law for lunar terminal descent was compared against ZEM/ZEV and modified-Apollo laws. — [KAIST](https://koasas.kaist.ac.kr/handle/10203/313053)
- A semi-analytical polynomial landing guidance law reached touchdown precision of a few meters in Monte Carlo runs with attitude, thrust and navigation errors. This was for lunar/asteroid landing, with no atmosphere. — [PoliMi 2015](https://re.public.polimi.it/bitstream/11311/940959/4/LUNGP_OA_01-15.pdf)
- In MPG (arXiv 2405.01264), the IC2 cases compare final position and velocity error:

  | Case | Method | ‖r_f‖ | ‖v_f‖ |
  |---|---|---|---|
  | A2 | Proposed | 0.62 m | 0.46 m/s |
  | B | Pure-drag model | 0.78 m | 0.54 m/s |
  | C | Uncompensated lift | 3.66 m | 1.12 m/s |
  | D | Analytic tracking law | 3.99 m | 2.48 m/s |

  The analytic tracking law could not account for aero forces and violated the aero-load limit. — [arXiv 2405.01264](https://arxiv.org/html/2405.01264v1)
- A dual-thrust switching analytical guidance law (arXiv 2026) has these features:
  - It proves the optimal attitude command is piecewise cubic.
  - It adaptively selects the guidance activation (ignition) point and the thrust-switch point.
  - It adds an aerodynamic correction for high L/D.
  - It was tested with 1,000 Monte Carlo samples, including 50% aerodynamic error, and trajectories converged to the target. I could not access the quantitative accuracy numbers.
  - [arXiv 2608.16000](https://arxiv.org/abs/2608.16000)
- Trajectory dispersion control (arXiv 2504.11894, 2025) has these features:
  - It combines a parameterized optimal feedback guidance law with online gPC dispersion prediction and gradient-based tuning of the guidance parameters, so landing-error dispersion meets a constraint.
  - Its disturbance model treats thrust, aero coefficients, density and a polynomial wind field as random variables.
  - The dispersion prediction matches Monte Carlo accuracy at lower cost.
  - [arXiv 2504.11894](https://arxiv.org/abs/2504.11894)
- Shuttle Powered Explicit Guidance (NASA) includes burn ignition-time control for precision landing and compensation for atmospheric drag. — [NTRS 20250011251](https://ntrs.nasa.gov/citations/20250011251)
- CNES/MINES Paris work treats nominal landing guidance as a minimum-effort, free-final-time correction to a reference trajectory, with non-trivial aero and mixed state-control (throttle) constraints. It repeatedly asks in flight whether landing is still feasible. — [AAS 2022](https://cas.minesparis.psl.eu/~petit/papers/aas22/main.pdf); [RB 2022](https://cas.minesparis.psl.eu/~petit/papers/rb22/main.pdf)

### Inferences
- Adding a drag-acceleration feedforward term to the existing ZEM/ZEV law is the analytic analogue of MPG Case B (pure-drag model). MPG Case D (no aero) shows the cost of omitting aero: about 6x worse position error. Either way, the commanded acceleration should be computed as the ZEM/ZEV term minus predicted drag, with the drag scale estimated online (next section).

### Gaps
- I found no paper giving a "robust E-guidance with drag" Monte Carlo for Earth boosters.

## Ignition-time / suicide-burn estimation under drag uncertainty: margins, throttle reserve, adaptive drag estimation

### Takeaway
I found no primary academic source that quantifies a "land at 70–80% throttle" reserve or in-flight C_D estimation for booster landing. The literature handles this through two mechanisms instead:
- free ignition time in the optimizer
- tighter thrust and attitude bounds in planning than in tracking, which leaves authority for disturbances

### Cited Findings
- Free-ignition-time formulations:
  - SCvx 6-DoF picks the optimal ignition time. — [arXiv 1811.10803](https://arxiv.org/abs/1811.10803)
  - The MPG planner treats ignition time as a decision variable. Its nominal ignition time of 3.88 s matched GPOPS-II. — [arXiv 2405.01264](https://arxiv.org/html/2405.01264v1)
- MPG robustness mechanisms:
  - The tracker uses wider attitude and load bounds than the planner.
  - It uses a thrust margin and a soft trust region.
  - Constraints are relaxed at the first node.
  - The authors report no tracker-infeasibility failures.
  - [arXiv 2405.01264](https://arxiv.org/html/2405.01264v1)
- The dual-thrust analytical law adaptively determines the guidance activation point. — [arXiv 2608.16000](https://arxiv.org/abs/2608.16000)
- Blackmore notes that failing to find a feasible solution in time crashes the vehicle, and that a suboptimal solution can exhaust propellant. — [NAP ch.10](https://www.nationalacademies.org/read/23659/chapter/10)

### Inferences
- For the heuristic: the latched a_z = g + (v^2 − v_t^2)/(2h) assumes drag stays roughly as modeled during the burn. If drag is overestimated, ignition comes too late. Practical fixes, none of which have a direct primary source:
  1. Compute ignition from a planning throttle of about 70–80% of max, with drag excluded or set to a conservative low bound, so a shortfall can be absorbed.
  2. Do not latch the burn. Recompute the required deceleration each step with available thrust, and raise throttle as needed.
  3. Estimate drag online. The residual a_meas − a_thrust − g gives D/m directly. Fit k_drag = residual / (0.5 ρ v² S C_D,model / m) with a low-pass filter during the unpowered entry coast, then use k_drag in the ignition predictor.
- Mapping these to the 94%→86% drop, two levers address the root cause, which is planning on drag that is not there: the online drag-scale estimate and planning at reduced throttle.

### Gaps
- No primary source found on Falcon 9 landing throttle reserve, or on online C_D estimation for landing guidance. Flight-specific details are proprietary.

## Published dispersion / Monte Carlo success rates and touchdown accuracy

### Takeaway
Published numbers:
- SpaceX's design targets: dispersions of 10 m (drone ship) and 30 m (land), where the landing ellipse is the 99% region.
- Academic SCP-MPC: about 96% success under ±15% C_D/C_L error and 0–8 m/s wind, with mean miss of about 1.5 m.

The simulator's 86–94% sits below these.

### Cited Findings
- Blackmore: landing precision is defined by a 99% landing ellipse. F9R targets dispersions of 10 m or better (drone ship) and 30 m or better (Cape Canaveral). — [NAP ch.10](https://www.nationalacademies.org/read/23659/chapter/10)
- MPG Monte Carlo (arXiv 2405.01264, 2024):
  - Dispersions:
    - initial position σ = 100 m
    - initial velocity σ = 15 m/s
    - C_D and C_L each ±15% uniform
    - wind 0–8 m/s at a random azimuth
  - Of 500 runs, 5 had no initial plan. Of the remaining 495, 476 succeeded (96.2%), where success means position error under 10 m and velocity error under 2.5 m/s.
  - Final position error: mean 1.47 m, SD 2.40 m. Final velocity error: mean 0.69 m/s, SD 0.72 m/s. Both statistics include failures.
  - Failures clustered at large wind combined with large aero-coefficient error.
  - [arXiv 2405.01264](https://arxiv.org/html/2405.01264v1)
- Dual-thrust analytical law: 1,000 Monte Carlo samples with 50% aero error converged to the target. Numbers not extracted. — [arXiv 2608.16000](https://arxiv.org/abs/2608.16000)
- PoliMi study: SCP with drag gave efficient reference trajectories. Its Monte Carlo runs indicated that added control authority (planar fins) and robust control are needed against aero uncertainty. — [PoliMi](https://re.public.polimi.it/handle/11311/1223390)
- X-33 landing dispersion study (NASA): 1,000 runs, with winds sampled from observed data and other errors zero-mean Gaussian. — [NTRS 20010080460](https://ntrs.nasa.gov/api/citations/20010080460/downloads/20010080460.pdf)

### Inferences
- MPG's ±15% C_D dispersion is narrower than the simulator's 0.68 scale (−32%). A fair benchmark would rerun the simulator's MC with ±15% around a corrected baseline, and separately with ±30%.

### Gaps
- No public Falcon 9 or New Shepard flight touchdown-error statistics.
- No accessible quantitative MC results for the 2504.11894 or 2608.16000 papers.

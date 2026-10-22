# Monte Carlo Dispersion Campaign Practice (launch / entry / landing)

Current sim baseline: 6 dispersed params (thrust 2%, Isp 1%, mass 0.5%, wind 5 m/s, density 5%, pitchover 0.25 deg; 1-sigma, clipped 3-sigma), N=100, seed 42, Wilson CI (94%, 87.5-97.2%).

## Which parameters are dispersed, and with what magnitudes?

### Takeaway
Professional campaigns disperse far more categories than this sim does. These include the initial/entry state, thrust per engine (each dispersed independently, then combined by RSS), mass properties, aero coefficients for each body, full atmosphere and wind profiles, and hardware parameters. I could not retrieve public numeric 1-sigma tables in this session, so magnitudes are listed as gaps.

### Cited Findings
- MER landing Monte Carlo varied entry states, atmosphere and wind profiles, and spacecraft parameters (entry mass and drag coefficients of the capsule, backshell, parachute and airbags). Sets of 2000 entry states were generated per site. — [Adler, MER briefing](https://marsoweb.nas.nasa.gov/landingsites/mer2003/doc/pasadena_01/talks/Adler-MER.pdf); [NTRS 20040095913](https://ntrs.nasa.gov/api/citations/20040095913/downloads/20040095913.pdf)
- MER trajectory analysis (Desai & Knocke) used a Monte Carlo dispersion analysis to assess entry-design robustness — [NTRS 20040095912](https://ntrs.nasa.gov/api/citations/20040095912/downloads/20040095912.pdf)
- InSight defined dispersions for its pre-flight aero database and "for each model," and used them in a statistical Monte Carlo prediction of EDL performance — [NTRS 20200002812](https://ntrs.nasa.gov/api/citations/20200002812/downloads/20200002812.pdf); [NTRS 20200002908](https://ntrs.nasa.gov/api/citations/20200002908/downloads/20200002908.pdf)
- MSL's prime EDL performance sim was an end-to-end POST2 simulation that produced Monte Carlo statistics of flight-software responses. An 8,000-trial Monte Carlo was used in parachute-force reconstruction. — [NTRS 20130011617](https://ntrs.nasa.gov/api/citations/20130011617/downloads/20130011617.pdf); [ISSFD 2012 MSL DSENDS](https://issfd.org/ISSFD_2012/ISSFD23_IN1_5.pdf)
- An aerocapture study ran 8000 cases with intentionally conservative dispersions — [NTRS 20205010695](https://ntrs.nasa.gov/api/citations/20205010695/downloads/main%20ver3.pdf)
- SLS disperses each RS-25 independently with equal sigma, and cluster thrust dispersion is combined by root-sum-square. Solid-booster thrust imbalance was studied with 1000 Monte Carlo motor pairs. — [NTRS 20190000734](https://ntrs.nasa.gov/api/citations/20190000734/downloads/20190000734.pdf); [NTRS 20140012588](https://ntrs.nasa.gov/api/citations/20140012588/downloads/20140012588.pdf)
- SLS navigation uses 6-DOF Monte Carlo plus covariance analysis — [NTRS 20180002041](https://ntrs.nasa.gov/api/citations/20180002041/downloads/20180002041.pdf)
- X-33 dispersion analysis used 1000 Monte Carlo runs — [NTRS 20010080460](https://ntrs.nasa.gov/api/citations/20010080460/downloads/20010080460.pdf)
- FAA guidance (for unguided vehicles) defines 3-sigma dispersion by varying one parameter at a time by its 3-sigma value — [FAA Trajectory Analysis attachment](https://www.faa.gov/sites/faa.gov/files/space/legislation_regulation_guidance/Attachment_1-Trajectory_Analysis.docx)
- A 2026 preprint (not peer reviewed) using a 1000-run 6-DOF GN&C sim found thrust misalignment to be the dominant predictor of impact displacement — [Preprints.org 202609.2181](https://www.preprints.org/manuscript/202609.2181)

### Inferences
- Candidates to add: thrust dispersed per engine, aero coefficient multipliers (for each body and flight regime, especially S1 entry/landing), CG offset and inertia, thrust-vector misalignment, actuator rate/bandwidth/delay, nav/sensor errors, separation-state errors, and landing-gear/touchdown parameters.
- One-at-a-time 3-sigma sensitivity sweeps are a cheap complement for finding dominant drivers.

### Gaps
- No numeric 1-sigma values for aero (%), CG offset, misalignment or nav errors were retrieved. PDF fetches failed with tool errors, and SLS values are generally not public. Pull the tables manually from the Desai & Knocke and InSight papers above.

## How many runs, and how are success probability and confidence stated?

### Takeaway
NASA MSFC practice (Hanson & Beard, NASA/TP-2010-216447) states verification as "requirement met in at least y% of cases with x% consumer risk (e.g., 90% confidence)." EDL projects routinely run 2000-8000 cases. N=100 at 94% success cannot support a high-reliability claim.

### Cited Findings
- Hanson & Beard, "Applying Monte Carlo Simulation to Launch Vehicle Design and Requirements Analysis," NASA/TP-2010-216447 (MSFC, 2010). It covers input distributions, testing the sim, how many runs are needed to verify requirements, and consumer-risk derivations (appendices). — [NTRS 20100038453](https://ntrs.nasa.gov/citations/20100038453)
- Verification must show a requirement is met in at least y% of cases with, e.g., 10% consumer risk (90% confidence). Consumer risk is the chance a finite sample says the requirement is met when it is not. — [NTRS 20100035661](https://ntrs.nasa.gov/api/citations/20100035661/downloads/20100035661.pdf)
- Zero-failure binomial demonstration: C = 1 - R^n, so n = ceil(ln(1-C)/ln R). The result is reported as a lower confidence bound. — [arXiv 2303.03167](https://arxiv.org/pdf/2303.03167); [MetricGate RDT](https://metricgate.com/docs/reliability-demonstration-test/)
- Run counts seen: MER 2000 per site ([Adler](https://marsoweb.nas.nasa.gov/landingsites/mer2003/doc/pasadena_01/talks/Adler-MER.pdf)), MSL 8000 ([NTRS 20130011617](https://ntrs.nasa.gov/api/citations/20130011617/downloads/20130011617.pdf)), aerocapture 8000 ([NTRS 20205010695](https://ntrs.nasa.gov/api/citations/20205010695/downloads/main%20ver3.pdf)), X-33 1000 ([NTRS 20010080460](https://ntrs.nasa.gov/api/citations/20010080460/downloads/20010080460.pdf)).

### Inferences
- My arithmetic: demonstrating 99.73% success at 90% confidence with zero failures needs 852 runs. Zero failures in 100 runs gives a one-sided 90% lower bound of only about 97.7%.
- Report one-sided Clopper-Pearson lower bounds (conservative, suited to requirement verification) alongside Wilson.
- Report percentiles of continuous margins (landing miss, propellant reserve, max-q, heating), e.g., the 99.865th percentile as one-sided 3-sigma. Margins converge much faster than rare-failure rates.
- Use 2000 or more runs for release campaigns and keep N=100 as a smoke test. Log the seed for each case so failures can be replayed.

### Gaps
- Did not extract Hanson & Beard's appendix tables (order-statistic run counts) because the PDF tool failed.

## How are wind profiles dispersed?

### Takeaway
NASA uses Earth-GRAM, which models wind as a mean plus large-scale and small-scale stochastic perturbations scaled to observed sigma. Its Range Reference Atmosphere (RRA) site data comes from balloon and rocketsonde measurements. A single scalar 5 m/s wind falls well short of this.

### Cited Findings
- Earth-GRAM wind is the mean plus large-scale and small-scale perturbations, scaled to observed standard deviations and treated as stochastic processes. The resulting dispersions are approximately Gaussian. — [NTRS 20180006355](https://ntrs.nasa.gov/api/citations/20180006355/downloads/20180006355.pdf); [NTRS 20190020082](https://ntrs.nasa.gov/api/citations/20190020082/downloads/20190020082.pdf)
- Earth-GRAM ships the 1983, 2006 and 2013 RRA databases, with 2013 recommended. They are built from balloon and rocketsonde data. — [NTRS 20180000878](https://ntrs.nasa.gov/api/citations/20180000878/downloads/20180000878.pdf)
- CorrMonte generates hourly cross-correlated profiles that capture wind persistence — [NTRS 20190000733](https://ntrs.nasa.gov/api/citations/20190000733/downloads/20190000733.pdf)
- Earth-GRAM is not a forecast model and misses localized effects. It runs fast and is often embedded in trajectory codes. — [NTRS 20180006355](https://ntrs.nasa.gov/api/citations/20180006355/downloads/20180006355.pdf)
- The Vector Wind Profile Model and GRAM are used with APRA to set design dispersions — [NTRS 20020045392](https://ntrs.nasa.gov/api/citations/20020045392/downloads/20020045392.pdf)
- A NASA Langley memo compared copropagated and precomputed atmosphere profiles in Monte Carlo trajectory sims — [NTRS 19920022213](https://ntrs.nasa.gov/api/citations/19920022213/downloads/19920022213.pdf)

### Inferences
- Use altitude-correlated wind and density profiles, either GRAM-like (mean plus correlated perturbations) or sampled from measured profiles for a site and month. Correlate density with temperature and wind instead of using independent scalars.

### Gaps
- Did not retrieve documentation of the Cape or Vandenberg measured-balloon (Jimsphere) profile databases.

## How are simulators validated against flight data (NASA-STD-7009)?

### Takeaway
NASA-STD-7009A (2016) scores M&S credibility on eight factors, each on a 0-4 scale. Flight reconstruction (e.g., MSL) feeds back into the model uncertainties.

### Cited Findings
- 7009A factors are data pedigree, input pedigree, code verification, solution verification, conceptual validation, referent validation, results uncertainty and results robustness. Each is scored 0-4. — [RePEc 2025 study](https://ideas.repec.org/a/taf/gcmbxx/v28y2025i4p450-463.html)
- The 2008 baseline factors were verification, validation, input pedigree, results uncertainty, results robustness, use history, M&S management and people qualifications — [Zang presentation](https://www3.nd.edu/~powers/vv.presentations/zang.pdf)
- Input pedigree is scored at the lowest-quality input — [NTRS 20160011121](https://ntrs.nasa.gov/api/citations/20160011121/downloads/20160011121.pdf)
- Rev A was approved 2016-07-13, and NASA-HDBK-7009 is the implementation guide — [NTRS 20200003344](https://ntrs.nasa.gov/api/citations/20200003344/downloads/20200003344.pdf)
- JPL preferred an optional checklist over the Credibility Assessment Scale (CAS) — [NTRS 20150021308](https://ntrs.nasa.gov/api/citations/20150021308/downloads/20150021308.pdf)
- MSL reconstruction dispersed CFD force and pressure coefficients by their a-priori uncertainty to compare against flight data — [NCSU dissertation](https://repository.lib.ncsu.edu/server/api/core/bitstreams/125b8c3a-482f-4f48-8bb3-2030b6cd3250/content)

### Inferences
- Add a 7009-style self-assessment to run manifests: verification tests, validation referents (e.g., public flight telemetry) and an input-pedigree entry for each dispersion.

### Gaps
- Factor lists and their per-URL attribution come from search snippets. Verify them against the NASA-STD-7009A text.

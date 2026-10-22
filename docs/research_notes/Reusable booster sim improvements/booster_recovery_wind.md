# Reusable Booster RTLS Recovery: Propellant Reserve and Wind

Note: the searches turned up few primary sources. SpaceX publishes no reserve or delta-v figures. Two key PDFs (Blackmore 2017; Kim et al. arXiv 2609.28759) were found, but their text could not be extracted, and the MDPI RTLS-vs-downrange paper returned HTTP 403. Most numbers below are **community estimates** and are flagged that way.

## Falcon 9 RTLS vs ASDS reserves, burn budgets, MECO conditions

### Takeaway
No official figures exist. Community estimates put remaining propellant at about 56 t for land (RTLS) landings and about 27 t for drone-ship landings. Musk's 2013 estimate was about 15% payload penalty for ASDS and about 30% for RTLS. RTLS MECO comes at about T+2:21–2:30. The simulator's fixed 67.5 t reserve sits above even the RTLS community average.

### Cited Findings
- Musk (2013 press conference): a drone-ship recovery costs about 15% of payload, and RTLS costs about double, roughly 30%. He said RTLS is "not physically possible" for high-energy missions. **Official statement, but dated.** — [CS Monitor 2016](https://csmonitor.com/Science/2016/0119/Why-does-SpaceX-keep-trying-to-land-rockets-on-floating-barges)
- A forum compilation (2018) gives averages of about 56.3 t of propellant remaining at MECO for land landings and about 27.1 t for drone-ship landings, against a 409.5 t first-stage load (about 4.4% for ASDS). **Unofficial estimate; the method is unclear.** — [New Mars forum](https://newmars.com/forums/viewtopic.php?id=7408)
- An uncited estimate says about 1,800 m/s is reserved for boostback, entry and landing combined. **Unverified.** — [Firgelli calculator blog](https://www.firgelliauto.com/es/blogs/engineering-calculators/delta-v-calculator)
- A hobbyist calculation: a booster with about 22.2 t dry mass needs at least 20 t for entry and landing burns from a 2,321 m/s MECO (ASDS-type, no boostback). **Hobbyist estimate.** — [KSP forum](https://forum.kerbalspaceprogram.com/topic/136038-sstos-galore/page/5)
- CRS/RTLS timelines (2016–2018): MECO at about T+2:21–2:30, boostback start at about T+2:39–2:42, entry burn start at about T+6:00–6:30. **Press-kit-derived and official in timing.** — [Spaceflight Now CRS-12](https://spaceflightnow.com/2017/08/13/timeline-for-spacexs-next-station-cargo-launch/); [Spaceflight Now CRS-16](https://spaceflightnow.com/2018/12/05/launch-timeline-for-spacexs-16th-resupply-launch-to-the-space-station/); [Spaceflight Now F9-23](https://spaceflightnow.com/2016/04/07/timeline-for-the-falcon-9-rockets-23rd-flight/)
- MECO occurs at about 2.5 min, about 80 km altitude and about 6,000 km/h (about 1.7 km/s). **Secondary source with no method stated, and probably an RTLS-type profile.** — [Orbital Radar glossary](https://orbitalradar.com/glossary/meco)
- RETALT1 (Springer CEAS) includes a figure of Falcon 9 MECO horizontal and vertical velocities, but the numbers were not retrieved. — [Springer s12567-021-00415-y](https://link.springer.com/10.1007/s12567-021-00415-y)
- The phase structure is boostback, entry burn, free fall, then landing burn. Boostback is omitted for ocean landing. — [MDPI Energies 14/11/3210](https://www.mdpi.com/1996-1073/14/11/3210/htm)

### Inferences
- If real RTLS missions carry about 56 t from MECO (all phases) and the simulator holds 67.5 t, the sim reserve is about 20% larger. This is consistent with the observed 6–10% early MECO. Using about 50–56 t as the RTLS baseline would be defensible, with the caveat that this figure is a forum estimate.
- The CRS timelines put about 10–20 s between MECO and boostback start (flip). For a simulator, MECO-to-entry is about 3.5–4 min.

### Gaps
- No primary source gives per-burn delta-v or durations for boostback, entry or landing. The webcast telemetry databases (e.g. community telemetry extractions of the SpaceX webcast overlays) were not searched.
- No figure was found for how much of the reserve is pure dispersion margin, as opposed to nominal consumption.

## Methods for sizing and adapting the reserve (adaptive reserve, MECO optimization, payload or wind awareness)

### Takeaway
The literature treats MECO conditions as the main design variable. Propellant-optimal RTLS envelopes are built over MECO states, and fuel-optimal boostback guidance is computed by convex optimization. No source describes how SpaceX adapts reserve to day-of-launch wind.

### Cited Findings
- Politecnico di Milano (MDPI Aerospace 11(1):35) builds propellant-optimal RTLS and downrange landing performance envelopes across a range of MECO conditions, under propellant and entry-load constraints. — [MDPI Aerospace 11/1/35](https://www.mdpi.com/2226-4310/11/1/35)
- Kim, Kang, Lee and Jo (2026) present "Fuel-optimal boost-back guidance via successive convexification with a terminal constraint." They describe boostback as the largest descent burn, so it strongly drives payload. The quantitative content was not extracted. — [arXiv 2609.28759](https://arxiv.org/pdf/2609.28759)
- Georgia Tech compares RTLS options (boostback vs glide-back) for a reusable booster stage. — [Georgia Tech repository](https://repository.gatech.edu/entities/publication/20570d01-bc1e-42fe-a906-444a2558273a)
- Blackmore (SpaceX, NAE Bridge 2017) says there is rarely extra propellant for a second landing attempt, so the reserve is a hard constraint. Onboard convex optimization (lossless convexification) solves powered descent in real time. — [Blackmore 2017 PDF](https://gwern.net/doc/technology/2017-blackmore.pdf); [Açıkmeşe/Carson/Blackmore LCvx](https://skoge.folk.ntnu.no/prost/proceedings/acc11/data/papers/1020.pdf)

### Inferences
- A practical adaptive scheme for the simulator: during ascent, predict the recovery propellant needed from the current state plus the forecast wind profile, using a fast boostback/entry/landing propagation or a pre-computed envelope over (MECO velocity, altitude, flight-path angle, headwind). Command MECO when remaining propellant equals the predicted need plus a statistical margin (e.g. the 3-sigma of Monte Carlo consumption). This replaces the fixed 67.5 t with "nominal + k·sigma".
- Payload dependence follows naturally: a heavier payload gives a lower MECO velocity, which needs less boostback delta-v, which allows a smaller reserve.

### Gaps
- No published closed-loop MECO-reserve algorithm was found, from SpaceX or academia.
- No source documents SpaceX's use of day-of-launch wind balloons for booster recovery planning. Their use for ascent loads is common knowledge, but no citation was found.

## Headwind and tailwind effects on boostback and entry propellant; mitigation

### Takeaway
No quantitative study was found that links wind to RTLS propellant. Existing studies treat wind as a dispersion in landing guidance Monte Carlo runs and compensate for it in guidance.

### Cited Findings
- Precision landing guidance with wind modeled as an altitude-dependent random field, alongside thrust and aero dispersions, in Monte Carlo analysis. — [arXiv 2504.11894](https://arxiv.org/pdf/2504.11894)
- A guidance method compensating for wind and unmodeled disturbances during reusable rocket landing (Acta Aeronautica et Astronautica Sinica, 2023). — [BUAA hkxb 2023.28465](https://hkxb.buaa.edu.cn/EN/10.7527/S1000-6893.2023.28465)
- Lateral disturbances during the landing phase amplify propellant slosh, especially when axial acceleration drops suddenly. — [JDSE 2021](https://journal.hep.com.cn/jdse/EN/1160126677089050768)

### Inferences (physics-based, not sourced)
- A headwind on return (i.e. the wind blowing downrange, against the booster's flight back toward the pad) mainly costs propellant in the subsonic final descent and the landing burn, plus any extra boostback needed to put the ballistic trajectory upwind. A cheap mitigation is to bias the boostback aim point upwind by the predicted wind drift integrated over the descent, so that drift is spent aerodynamically rather than with the engine.
- Lofting the boostback (more vertical delta-v) lengthens time of flight and raises wind exposure. A flatter return reduces exposure but increases entry heating.

### Gaps
- No published propellant-per-(m/s headwind) sensitivity for Falcon-class RTLS. This should be derived from the simulator's own Monte Carlo runs.

## Grid-fin entry guidance and aerodynamic divert capability

### Takeaway
Sources confirm that the grid fins provide lift and drag control for steering. No credible published L/D or crossrange figures for Falcon 9 were found.

### Cited Findings
- The grid fins deploy for reentry and "help control lift during reentry," guiding the booster to the pad or drone ship. — [Smithsonian NASM](https://airandspace.si.edu/collection-objects/grid-fin-rocket-launch-vehicle-falcon-9/nasm_A20220607000)
- The fins rotate independently to control lift and drag, coordinated with engine throttling. **Secondary source.** — [BGR](https://www.bgr.com/2193547/what-are-grid-fins-for-on-spacex-falcon-9-rocket/)
- A CFD study of grid-fin optimization for RLVs (drag and hinge moment sensitivity). — [Plymouth Student Scientist](https://pearl.plymouth.ac.uk/tpss/vol15/iss2/9)
- DLR's CALLISTO aerodynamic database work notes that aero performance strongly drives the vehicle's return performance. — [EUCASS2023-510](https://eucass.eu/doi/EUCASS2023-510.pdf)

### Inferences
- A booster flying engine-first at small angle of attack achieves only a low L/D (well under 1). Divert is therefore modest per unit time, but it is free in propellant over several minutes of descent. That makes it the right tool for absorbing wind drift instead of the landing burn.

### Gaps
- No primary L/D, trim angle-of-attack or crossrange numbers. The CALLISTO and RETALT aero databases (EUCASS papers) are the best candidates for retrieval.

## Academic RTLS trajectory studies

### Takeaway
The relevant literature includes CALLISTO (DLR/CNES/JAXA, return to pad), RETALT (EU), Politecnico di Milano envelopes, Georgia Tech RTLS comparisons, and recent convex boostback guidance work.

### Cited Findings
- CALLISTO is a subscale VTVL demonstrator that returns to the launch pad using retropropulsion and landing legs. — [EUCASS2023-431](https://www.eucass.eu/doi/EUCASS2023-431.pdf); [DLR CALLISTO](https://www.dlr.de/en/research-and-transfer/projects-and-missions/callisto/computer-animation-of-callisto-near-the-end-of-its-ascent-shortly-before-engine-shutdown)
- RETALT1 compares its mission profile against Falcon 9 MECO conditions. — [Springer CEAS 2021](https://link.springer.com/10.1007/s12567-021-00415-y)
- Additional sources: an RTLS guidance study at EUCASS 2017 ([EUCASS2017-680](https://www.eucass.eu/doi/EUCASS2017-680.pdf)), a landing guidance study at EUCASS 2019 ([EUCASS2019-0275](https://www.eucass.eu/doi/EUCASS2019-0275.pdf)), and a PoliMi thesis/paper ([GUADJ01-24](https://re.public.polimi.it/retrieve/ba09b835-7e43-4c8a-87ca-86af91e178eb/GUADJ01-24.pdf)).

### Gaps
- Themis, TU Delft and Long March reusable studies were not searched because of the tool-call budget.

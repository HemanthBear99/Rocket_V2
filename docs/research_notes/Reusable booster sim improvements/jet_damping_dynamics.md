# Jet damping and variable-mass rotational dynamics (per axis)

Note: The web search was limited (about 12 tool calls). Several classical primary sources could not be retrieved as text: the DTIC PDFs returned 403 and the arXiv/Basilisk PDFs were compressed. Derivations that are marked as inferences come from standard control-volume angular-momentum balance. Check them against Thomson (*Introduction to Space Dynamics*, Dover, variable-mass chapter) or Ellis & McArthur before relying on them as cited facts.

## Q1. Correct jet damping torque for pitch/yaw and for roll

### Takeaway
Jet damping is the angular momentum carried out through the nozzle exit planes, which is the sum over the exit areas of mdot * r x (omega x r). In pitch and yaw the lever arm is the axial distance from the CG to the exit plane, which gives about mdot*l^2. In roll only the radial distance from the roll axis counts, which gives sum_i mdot_i (d_i^2 + r_e^2/2). The cg_z^2 term must not be applied to roll.

### Cited Findings
- Jet damping is the effect of exhaust removing energy from a rocket's transverse angular motion. It is stabilizing when the distance from the instantaneous CM to the nozzle exit plane exceeds the instantaneous transverse radius of gyration. — [Wikipedia: Jet damping](https://en.wikipedia.org/wiki/Jet_damping), citing van der Ha & Janssens, "Jet Damping and Misalignment Effects During Solid Rocket Motor Burn", AIAA GNC 2003, and Etkin & Reid, *Dynamics of Flight* (1996), p.141.
- Rosser et al. first presented jet damping of rotating rockets. Thomson later analysed it for solid rocket motors. — [Nanjangud & Eke, arXiv:1807.01575](https://ar5iv.arxiv.org/html/1807.01575)
- A small change in nozzle expansion ratio can change the character of the motion variables during substantial mass loss (Sookgaew & Eke 2004). The roll component of fluid motion inside the combustion chamber was studied by Eke, Tran & Sookgaew (2006). — [arXiv:1807.01575](https://ar5iv.arxiv.org/html/1807.01575)
- The general variable-mass balance is 0 = dH*/dt + ∫_S rho h u dS, where h = p x [v_r + omega x p] and u = v_r·n. The flux integral over the exit surface is the jet term. — [Nanjangud & Eke, "On the Angular Momentum of Rockets, Balloons, and Other Variable Mass Systems", arXiv:1612.00884](https://ar5iv.arxiv.org/html/1612.00884)
- Spin (roll) jet damping is treated as a separate effect from transverse damping for rocket-assisted projectiles. — [ARBRL-MR-03052 (DTIC ADA089980), not retrieved, 403](https://apps.dtic.mil/sti/pdfs/ADA089980.pdf)

### Inferences
- Applying the omega x p part of the flux integral above to engine i, with its exit centre at p_i = (x_i, y_i, -l) in body axes and z as the roll axis from the CG, gives:
  - Pitch/yaw: tau_x ≈ -sum_i mdot_i (l^2 + y_i^2) omega_x, and similarly for yaw. Because l is much larger than d_i, this is ≈ -mdot l^2 omega_x.
  - Roll: tau_z = -sum_i mdot_i (x_i^2 + y_i^2 + r_e^2/2) omega_z. The r_e^2/2 term comes from integrating rho u r^2 over a uniform circular exit disk.
  - The off-diagonal terms (-mdot_i x_i y_i, and products of l with x_i or y_i) cancel for a symmetric 8+1 octaweb with all engines running. They do not automatically cancel for engine-subset or engine-out burns.
- Recommended implementation: build a 3x3 "jet inertia" matrix J = sum_i mdot_i [ (p_i·p_i) E - p_i p_i^T ] + diag(·, ·, sum_i mdot_i r_e^2/2), then set tau_damp = J omega. This handles every engine-out and engine-subset case automatically.
- The p x v_r part of the flux is the thrust moment. It is already in the gimbal and thrust torque, so do not add it again.

### Gaps
- No primary-source scan giving Thomson's or Ellis & McArthur's exact per-axis equations was obtained. Pull Thomson's variable-mass chapter or the relevant NASA SP-8xxx before citing a page number.

## Q2. Combining Idot*omega and jet damping, and the full equation

### Takeaway
About the instantaneous CG, in body axes: I omega_dot + omega x (I omega) + Idot omega = M_ext + M_thrust - J omega, plus small internal-flow Coriolis terms. Idot is the true, negative rate of change of the body inertia. The simulator's form is structurally correct provided that Idot has the right sign, that J is computed per axis as above, and that no "effective damping (mdot l^2 - Idot)" is also applied elsewhere.

### Cited Findings
- The pitch-inertia rate due to propellant expulsion is negative, and this produces the well-known pitch jet damping effect. — [DTIC ADA089980 (via search summary)](https://apps.dtic.mil/sti/pdfs/ADA089980.pdf)
- The stability criterion is l > k_transverse, which implies net damping ∝ mdot(l^2 - k^2). — [Wikipedia: Jet damping](https://en.wikipedia.org/wiki/Jet_damping)
- Eke and coworkers derive the governing equations with Kane's method and report that system and combustion-chamber geometry play a crucial role in attitude stability. — [AAS 97-117 abstract](https://space-flight.org/AAS_meetings/1997_winter/abstracts/97-117.html); [AAS 93-107 abstract](https://space-flight.org/AAS_meetings/1993_winter/abstracts/93-107.html)

### Inferences
- The -mdot k^2 part of the stability criterion is the Idot*omega contribution for uniform mass loss. That makes Idot*omega the destabilizing half of the same physics, not a separate damping term.
- Double-counting risks:
  - (a) Using both -Idot*omega and a "net" coefficient mdot(l^2 - k^2). Use either the raw mdot*l^2 with Idot, or the net coefficient without Idot, but never both.
  - (b) Inconsistent reference point. Idot must be the rate of change of inertia about the current CG, including the CG-shift effect, and J must use lever arms from that same CG.
  - (c) Sign. In omega_dot = I^-1(tau - omega x I omega - Idot omega - tau_damp), Idot < 0 makes -Idot*omega destabilizing, which is correct.
- For a booster whose tanks drain from the bottom, pitch Idot is not -mdot k^2. Take it from the mass-properties model, for example by finite-differencing I about the current CG.

### Gaps
- No open-source rocket code was found that explicitly separates these terms with documentation.

## Q3. Magnitudes for a 9-engine octaweb

### Takeaway
At mdot = 2,500 kg/s the roll jet-damping coefficient is about 5.3e3 kg·m²/s. The pitch coefficient is 5.6e5 to 4.0e6 kg·m²/s for l = 15 to 40 m. Roll is about 0.13% to 0.94% of pitch. Using cg_z^2 for roll overstates roll damping by a factor of about 100 to 750.

### Cited Findings
- No source was retrieved for the engine geometry. The inputs below are the assignment's assumptions: outer ring radius 1.5 m, exit radius 0.46 m, mdot 2,500 kg/s.

### Inferences
- Per engine: mdot_i = 2500/9 ≈ 278 kg/s.
- Roll:
  - Outer engines: 8 × 278 × 1.5^2 ≈ 5,000.
  - Exit-disk terms: 2500 × 0.46^2/2 ≈ 265.
  - Total C_roll ≈ 5.3e3 kg·m²/s.
- Pitch: C_pitch ≈ 2500 × l^2. This is 5.6e5 at l = 15 m and 4.0e6 at l = 40 m. The ring term, sum mdot_i y_i^2 ≈ 2.5e3, is negligible.
- Ratio C_roll/C_pitch: 0.94% at l = 15 m and 0.13% at l = 40 m.
- With a booster roll inertia of about 1-3e5 kg·m² (rough hollow-cylinder estimate, not sourced), the roll damping time constant I/C is:
  - with the cg_z^2 bug: about 0.03-0.5 s, so any roll rate is killed almost instantly, which is non-physical;
  - with the correct value: about 20-60 s.
- Centre-engine-only (landing) burn: C_roll ≈ 278 × 0.21/2 ≈ 29 kg·m²/s, which is essentially no roll damping.
- Three-engine burn (centre plus two outer): C_roll ≈ 2 × 278 × 2.25 + 3 × 29 ≈ 1.3e3 kg·m²/s.

### Gaps
- No published Falcon 9 roll inertia or octaweb radius was verified in this research.

## Q4. How open simulators implement it

### Takeaway
No surveyed open simulator was found to document an explicit jet-damping term. JSBSim and Basilisk model mass depletion through time-varying mass properties only, and Basilisk's FUELTANK memo may include mass-flux terms (not verified). No sources were retrieved for NASA Trick/JEOD or NASA 42.

### Cited Findings
- In JSBSim, empty mass and inertia come from configuration, and point masses and fuel load are added. Mass and inertia vary through the tanks. No jet-damping term is documented. — [JSBSim FGMassBalance docs](https://jsbsim.sourceforge.net/JSBSim/classJSBSim_1_1FGMassBalance.html)
- The Basilisk thruster model applies force at a body location and uses the current CM to compute torque. — [Basilisk THRUSTERS tech memo](https://www.hanspeterschaub.info/basilisk/_downloads/a7c0e4dcd6c44dd4aeae283391f7fb78/Basilisk-THRUSTERS20170712.pdf)
- The Basilisk fuelTank module computes its mass-flow derivative from the attached thrusters and contributes to the back-substitution matrices and to energy/momentum. The details are in the FUELTANK memo. — [fuelTank docs](https://www.hanspeterschaub.info/basilisk/_sources/Documentation/simulation/dynamics/FuelTank/fuelTank.rst.txt); [FUELTANK memo](https://www.hanspeterschaub.info/basilisk/_downloads/82026bf39c9f45387eeb14abe20de974/Basilisk-FUELTANK-20171203.pdf)
- Basilisk's thrusterStateEffector had mass depletion hard-coded at 100% (issue #735). The docs state that mass flow is constant while the valve is open. — [Basilisk issue #735](https://github.com/avslab/basilisk/issues/735); [thrusterStateEffector docs](https://hanspeterschaub.info/basilisk/Documentation/simulation/dynamics/Thrusters/thrusterStateEffector/thrusterStateEffector.html)

### Inferences
- The Basilisk FUELTANK memo is the most likely open primary source for mass-depletion back-substitution terms, including the exhaust momentum flux. It should be read next.

### Gaps
- No documentation was retrieved for NASA Trick/JEOD, NASA 42, or OpenRocket.

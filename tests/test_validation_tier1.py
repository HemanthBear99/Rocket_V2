"""Tier 1 physics validation: analytic ground truth, no external data required.

These tests do not compare against proprietary/real flight telemetry -- they
compare the simulator against cases that have an exact, closed-form correct
answer from classical mechanics (energy/angular-momentum conservation in a
two-body orbit, the Tsiolkovsky rocket equation, and the published US
Standard Atmosphere 1976 reference table). This is the cheapest, fastest
tier of the validation plan discussed for making the simulator's numerics
defensible to a technical buyer, and a prerequisite for the heavier tiers
(cross-validation against NASA GMAT, comparison against published real
mission data).

A failure here means the integrator/force models violate physics that has
no ambiguity about the right answer -- this is a stronger and more
fundamental claim than "matches one particular real flight."
"""

import numpy as np
import pytest

from rlv_sim import constants as C
from rlv_sim.config_factory import create_test_config
from rlv_sim.dynamics import DynamicsContext
from rlv_sim.forces import compute_atmosphere_properties
from rlv_sim.integrators import rk4_step
from rlv_sim.physics_checks import compute_total_energy
from rlv_sim.state import State


def _two_body_ctx(config=None):
    """A DynamicsContext with only central gravity active: no thrust, drag,
    lift, J2, or atmosphere -- isolates the RK4 integrator + gravity model
    against the pure two-body problem, which has exact conserved quantities.
    """
    if config is None:
        config = create_test_config(
            enable_drag=False,
            enable_lift=False,
            enable_j2=False,
            enable_atmosphere=False,
            gravity_model="central",
        )
    return DynamicsContext(
        thrust_on=False,
        throttle=0.0,
        dry_mass=C.DRY_MASS,
        stage=1,
        vehicle_model="orbiter",
        config=config,
    )


def _angular_momentum(r: np.ndarray, v: np.ndarray) -> np.ndarray:
    return np.cross(r, v)


class TestKeplerianConservationCircularOrbit:
    """A circular orbit under pure central gravity must conserve energy,
    angular momentum, and orbital radius exactly (up to numerical
    integration error) -- these are analytic invariants of the two-body
    problem, independent of any real-world data.
    """

    def _propagate(self, n_orbits: float = 2.0, dt: float = 2.0):
        altitude = 500_000.0
        r0 = C.R_EARTH + altitude
        v_circular = float(np.sqrt(C.MU_EARTH / r0))
        period = 2.0 * np.pi * np.sqrt(r0 ** 3 / C.MU_EARTH)

        state = State(
            r=np.array([r0, 0.0, 0.0]),
            v=np.array([0.0, v_circular, 0.0]),
            q=np.array([1.0, 0.0, 0.0, 0.0]),
            omega=np.zeros(3),
            m=C.DRY_MASS + 1000.0,
            t=0.0,
        )
        ctx = _two_body_ctx()
        n_steps = int((n_orbits * period) / dt)

        r_mags = [float(np.linalg.norm(state.r))]
        energies = [compute_total_energy(state.r, state.v, state.m)]
        ang_mom = [_angular_momentum(state.r, state.v)]

        for _ in range(n_steps):
            state = rk4_step(state, torque=np.zeros(3), dt=dt, ctx=ctx)
            r_mags.append(float(np.linalg.norm(state.r)))
            energies.append(compute_total_energy(state.r, state.v, state.m))
            ang_mom.append(_angular_momentum(state.r, state.v))

        return r0, v_circular, period, r_mags, energies, ang_mom

    def test_orbital_radius_stays_constant(self):
        r0, _, _, r_mags, _, _ = self._propagate()
        r_mags = np.array(r_mags)
        # Circular orbit: radius must not drift by more than ~0.01% over
        # two full periods for a well-behaved RK4 integrator at this dt.
        assert np.max(np.abs(r_mags - r0)) / r0 < 1e-4

    def test_energy_conserved(self):
        _, _, _, _, energies, _ = self._propagate()
        e0 = energies[0]
        max_relative_drift = max(abs((e - e0) / e0) for e in energies)
        assert max_relative_drift < 1e-4

    def test_angular_momentum_conserved(self):
        _, _, _, _, _, ang_mom = self._propagate()
        h0 = ang_mom[0]
        h0_mag = float(np.linalg.norm(h0))
        max_relative_drift = max(
            float(np.linalg.norm(h - h0)) / h0_mag for h in ang_mom
        )
        assert max_relative_drift < 1e-6

    def test_orbital_period_matches_kepler_third_law(self):
        # After exactly one analytic period the vehicle should return
        # close to its starting position -- an independent check that
        # doesn't rely on the conserved-quantity bookkeeping above.
        r0 = C.R_EARTH + 500_000.0
        v_circular = float(np.sqrt(C.MU_EARTH / r0))
        period = 2.0 * np.pi * np.sqrt(r0 ** 3 / C.MU_EARTH)

        state = State(
            r=np.array([r0, 0.0, 0.0]),
            v=np.array([0.0, v_circular, 0.0]),
            q=np.array([1.0, 0.0, 0.0, 0.0]),
            omega=np.zeros(3),
            m=C.DRY_MASS + 1000.0,
            t=0.0,
        )
        ctx = _two_body_ctx()
        dt = 1.0
        n_steps = int(round(period / dt))
        for _ in range(n_steps):
            state = rk4_step(state, torque=np.zeros(3), dt=dt, ctx=ctx)

        position_error_m = float(np.linalg.norm(state.r - np.array([r0, 0.0, 0.0])))
        circumference_m = 2.0 * np.pi * r0
        # Observed RK4 closure error at dt=1s is ~1.1 km against a ~43,170 km
        # circumference (~0.0025%) -- a tight bound for RK4 truncation
        # error, not a sign of a bug. Bound relative to circumference
        # rather than a fixed meter count so this scales sensibly if the
        # test orbit altitude ever changes.
        assert position_error_m / circumference_m < 5e-5


class TestKeplerianConservationEllipticalOrbit:
    """Same conservation laws, now on a moderately eccentric orbit, so the
    check isn't only exercising the degenerate circular case.
    """

    def test_energy_and_angular_momentum_conserved_over_one_period(self):
        r_perigee = C.R_EARTH + 300_000.0
        r_apogee = C.R_EARTH + 2_000_000.0
        a = 0.5 * (r_perigee + r_apogee)
        # Vis-viva at perigee.
        v_perigee = float(np.sqrt(C.MU_EARTH * (2.0 / r_perigee - 1.0 / a)))
        period = 2.0 * np.pi * np.sqrt(a ** 3 / C.MU_EARTH)

        state = State(
            r=np.array([r_perigee, 0.0, 0.0]),
            v=np.array([0.0, v_perigee, 0.0]),
            q=np.array([1.0, 0.0, 0.0, 0.0]),
            omega=np.zeros(3),
            m=C.DRY_MASS + 1000.0,
            t=0.0,
        )
        ctx = _two_body_ctx()
        dt = 5.0
        n_steps = int(period / dt)

        e0 = compute_total_energy(state.r, state.v, state.m)
        h0 = _angular_momentum(state.r, state.v)
        h0_mag = float(np.linalg.norm(h0))

        max_e_drift = 0.0
        max_h_drift = 0.0
        for _ in range(n_steps):
            state = rk4_step(state, torque=np.zeros(3), dt=dt, ctx=ctx)
            e = compute_total_energy(state.r, state.v, state.m)
            h = _angular_momentum(state.r, state.v)
            max_e_drift = max(max_e_drift, abs((e - e0) / e0))
            max_h_drift = max(max_h_drift, float(np.linalg.norm(h - h0)) / h0_mag)

        assert max_e_drift < 1e-3
        assert max_h_drift < 1e-5

    def test_semi_major_axis_from_vis_viva_matches_specified_orbit(self):
        # Sanity check on the setup itself, not the integrator: confirms
        # the initial state actually encodes the intended orbit before
        # trusting any propagated result from it.
        r_perigee = C.R_EARTH + 300_000.0
        r_apogee = C.R_EARTH + 2_000_000.0
        a_expected = 0.5 * (r_perigee + r_apogee)
        v_perigee = float(np.sqrt(C.MU_EARTH * (2.0 / r_perigee - 1.0 / a_expected)))

        energy = 0.5 * v_perigee ** 2 - C.MU_EARTH / r_perigee
        a_from_energy = -C.MU_EARTH / (2.0 * energy)
        assert a_from_energy == pytest.approx(a_expected, rel=1e-9)


class TestTsiolkovskyRocketEquation:
    """Isolates the mass/thrust integration from gravity and atmosphere by
    placing the vehicle far enough from Earth that gravity is negligible
    (~4e-12 m/s^2 at 1e13 m, versus a commanded thrust acceleration many
    orders of magnitude larger), then checks the RK4-integrated delta-v
    against the closed-form Tsiolkovsky rocket equation:

        delta_v = v_e * ln(m0 / m1),  v_e = Isp_vac * g0
    """

    def test_delta_v_matches_tsiolkovsky_equation(self):
        config = create_test_config(
            enable_drag=False,
            enable_lift=False,
            enable_j2=False,
            enable_atmosphere=False,
        )
        dry_mass = C.STAGE2_DRY_MASS
        m0 = dry_mass + 500.0

        state = State(
            r=np.array([1.0e13, 0.0, 0.0]),  # gravity negligible out here
            v=np.zeros(3),
            q=np.array([1.0, 0.0, 0.0, 0.0]),
            omega=np.zeros(3),
            m=m0,
            t=0.0,
        )
        ctx = DynamicsContext(
            thrust_on=True,
            throttle=1.0,
            dry_mass=dry_mass,
            stage=2,
            vehicle_model="orbiter",
            config=config,
        )

        # rk4_step's internal propellant-depletion sub-stepping only stops
        # applying thrust *within* the single dt it is handed -- it has no
        # memory across separate rk4_step calls. The real simulation loop
        # (_simulation_step.py) re-gates `propellant_available = state.m >
        # dry_mass + eps` before every step and passes thrust_on=False once
        # exhausted; a raw loop over rk4_step must reproduce that same
        # external gate, or thrust keeps being applied for free (zero mass
        # flow, since compute_mass_derivative floors at dry_mass) once the
        # vehicle already sits exactly at dry_mass -- producing unbounded
        # delta-v with no propellant cost. This is not a simulator bug; it
        # is a documented division of responsibility between rk4_step
        # (single-step depletion handling) and the caller (multi-step
        # propellant-exhausted gating).
        from dataclasses import replace as _dc_replace

        dt = 0.5
        for _ in range(20):  # up to 10 s of burn time, well short of 500 kg
            if state.m <= dry_mass + 1e-6:
                ctx = _dc_replace(ctx, thrust_on=False, throttle=0.0)
            state = rk4_step(state, torque=np.zeros(3), dt=dt, ctx=ctx)

        m1 = state.m
        assert m1 < m0, "expected propellant to be consumed"

        simulated_dv = float(np.linalg.norm(state.v))
        v_exhaust = config.stage2_isp_vac * C.G0
        expected_dv = v_exhaust * float(np.log(m0 / m1))

        assert simulated_dv == pytest.approx(expected_dv, rel=1e-3)

    def test_zero_burn_gives_zero_delta_v(self):
        config = create_test_config(enable_drag=False, enable_lift=False, enable_j2=False, enable_atmosphere=False)
        dry_mass = C.STAGE2_DRY_MASS
        state = State(
            r=np.array([1.0e13, 0.0, 0.0]),
            v=np.zeros(3),
            q=np.array([1.0, 0.0, 0.0, 0.0]),
            omega=np.zeros(3),
            m=dry_mass + 500.0,
            t=0.0,
        )
        ctx = DynamicsContext(
            thrust_on=False, throttle=0.0, dry_mass=dry_mass, stage=2,
            vehicle_model="orbiter", config=config,
        )
        state = rk4_step(state, torque=np.zeros(3), dt=0.5, ctx=ctx)
        assert np.linalg.norm(state.v) < 1e-6


class TestUS76AtmosphereReferenceTable:
    """Compares compute_atmosphere_properties against the published US
    Standard Atmosphere 1976 layer-base temperature/pressure values (the
    standard 7-layer breakpoints reproduced in essentially every aerospace
    atmosphere reference table), independent of how this codebase derives
    them internally.
    """

    # (altitude_m, temperature_K, pressure_Pa) at each published US76 layer
    # base -- these are the standard tabulated breakpoint values, not
    # values re-derived from this codebase's own formula.
    REFERENCE_POINTS = [
        (0.0, 288.15, 101325.0),
        (11000.0, 216.65, 22632.1),
        (20000.0, 216.65, 5474.89),
        (32000.0, 228.65, 868.019),
        (47000.0, 270.65, 110.906),
        (51000.0, 270.65, 66.9389),
        (71000.0, 214.65, 3.95642),
    ]

    @pytest.mark.parametrize("altitude,expected_t,expected_p", REFERENCE_POINTS)
    def test_temperature_matches_published_table(self, altitude, expected_t, expected_p):
        t, p, rho, a = compute_atmosphere_properties(altitude, enable_upper_atm=False)
        assert t == pytest.approx(expected_t, abs=0.05)

    @pytest.mark.parametrize("altitude,expected_t,expected_p", REFERENCE_POINTS)
    def test_pressure_matches_published_table(self, altitude, expected_t, expected_p):
        t, p, rho, a = compute_atmosphere_properties(altitude, enable_upper_atm=False)
        assert p == pytest.approx(expected_p, rel=1e-3)

    def test_density_consistent_with_ideal_gas_law(self):
        # rho = P / (R_specific * T) must hold at every altitude given the
        # model's own (T, P) outputs -- an internal-consistency check, not
        # a comparison to an external table.
        for altitude, _, _ in self.REFERENCE_POINTS:
            t, p, rho, a = compute_atmosphere_properties(altitude, enable_upper_atm=False)
            expected_rho = p / (C.R_GAS * t)
            assert rho == pytest.approx(expected_rho, rel=1e-6)

    def test_sea_level_speed_of_sound_is_standard_340_mps(self):
        t, p, rho, a = compute_atmosphere_properties(0.0, enable_upper_atm=False)
        # Standard sea-level speed of sound, widely published as ~340.3 m/s.
        assert a == pytest.approx(340.3, abs=1.0)

    def test_density_decreases_monotonically_with_altitude_in_troposphere(self):
        altitudes = np.linspace(0.0, 11000.0, 12)
        densities = [
            compute_atmosphere_properties(float(h), enable_upper_atm=False)[2]
            for h in altitudes
        ]
        assert all(densities[i] > densities[i + 1] for i in range(len(densities) - 1))

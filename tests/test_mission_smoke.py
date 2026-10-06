"""Fast smoke/regression tests that exercise the real simulation loop.

Unlike the full end-to-end mission (which the verified test architecture
report measured at ~422s wall time for the S2-recovery scenario), these use
a short max_time so the suite stays fast. They are not a substitute for a
full-duration regression baseline -- see the report's "golden-test
candidates" section for what a full-duration golden test should look like
once a passing (non-CURRENT-FAILURE) full-mission configuration exists.
"""

import pytest

from rlv_sim._run_simulation import run_simulation
from rlv_sim.config_factory import create_test_config


class TestShortAscentSmoke:
    """A short ascent-only run must complete without raising and must
    produce a finite, physically valid final state -- this exercises
    validate_state's enforcement in the real hot path (confirmed at
    _run_simulation.py:170), not just the isolated unit tests in
    test_physics_checks.py.
    """

    def test_short_ascent_runs_without_exception(self):
        config = create_test_config(dt=0.5, max_time=20.0, verbose=False)
        final_state, log, reason = run_simulation(
            dt=0.5, max_time=20.0, verbose=False, vehicle_type="ascent", config=config
        )
        assert final_state is not None
        assert reason is not None

    def test_short_ascent_terminates_on_max_time(self):
        # 20s is far short of MECO for the default vehicle, so termination
        # should be driven by the max_time limit, not an anomaly/abort.
        config = create_test_config(dt=0.5, max_time=20.0, verbose=False)
        final_state, log, reason = run_simulation(
            dt=0.5, max_time=20.0, verbose=False, vehicle_type="ascent", config=config
        )
        assert final_state.t == pytest.approx(20.0, abs=0.6)

    def test_final_state_is_physically_valid(self):
        config = create_test_config(dt=0.5, max_time=20.0, verbose=False)
        final_state, log, reason = run_simulation(
            dt=0.5, max_time=20.0, verbose=False, vehicle_type="ascent", config=config
        )
        # Re-run the same invariant checks the hot path enforces; this
        # should never raise for a normal short ascent.
        from rlv_sim.physics_checks import validate_state

        validate_state(final_state, abort_on_error=True)

    def test_mass_decreases_from_liftoff(self):
        config = create_test_config(dt=0.5, max_time=20.0, verbose=False)
        final_state, log, reason = run_simulation(
            dt=0.5, max_time=20.0, verbose=False, vehicle_type="ascent", config=config
        )
        assert final_state.m < config.stage1_dry_mass + config.stage2_dry_mass + config.payload_mass + (
            config.stage1_prop_mass + config.stage2_prop_mass
        )

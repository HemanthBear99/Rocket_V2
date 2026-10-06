"""Regression tests for the S2 landing-propellant-reserve investigation.

Full end-to-end verification of this fix required running the real
multi-thousand-second full mission (see the debug scripts referenced in
session notes) -- too slow for this suite. These tests instead lock in the
isolated mechanism-level behavior directly.

Key finding from that investigation (see s2_landing_propellant_reserve_kg's
docstring in config_definition.py for the full account): the S2 landing
burn's control law (compute_s2_entry_landing_guidance) is itself sound --
a standalone probe replaying the real CURRENT FAILURE's exact ignition
state landed cleanly at 1.85 m/s given ~4200 kg of propellant, versus only
1720.9 kg actually available. But orbit-insertion is not currently
propellant-limited (it stops on achieving a good orbit, not on running
low on fuel) -- setting a nonzero reserve large enough to matter for
landing (~4000 kg) caused orbit insertion ITSELF to fail in an end-to-end
run (perigee -208.4 km, re-entered mid-insertion), because insertion
already consumes essentially the entire propellant budget. The default is
therefore 0.0 (a documented no-op preserving prior behavior) rather than a
value that trades one failure mode for another without the vehicle
carrying more total propellant.
"""

import numpy as np
import pytest

from rlv_sim import constants as C
from rlv_sim._guidance_orbit import (
    compute_deorbit_guidance,
    compute_orbit_insertion_guidance,
)
from rlv_sim.config_factory import create_default_config
from rlv_sim.state import State


class TestReserveDefaultIsNoOp:
    def test_default_reserve_is_zero(self):
        config = create_default_config(enable_s2_recovery=True)
        assert config.s2_landing_propellant_reserve_kg == 0.0

    def test_negative_reserve_rejected(self):
        with pytest.raises(ValueError, match="s2_landing_propellant_reserve_kg"):
            create_default_config(s2_landing_propellant_reserve_kg=-1.0)


class TestOrbitInsertionReserveGating:
    def _suborbital_state(self, m: float, config):
        # Circular-ish orbit well below the target altitude, so the
        # guidance is in a "still needs to raise apogee" regime and
        # FAILED_UNREACHABLE is reachable purely via the propellant gate.
        s2_dry = config.stage2_dry_mass + config.payload_mass
        r_mag = C.R_EARTH + 50_000.0
        v_circ = float(np.sqrt(C.MU_EARTH / r_mag))
        return State(
            r=np.array([r_mag, 0.0, 0.0]),
            v=np.array([0.0, v_circ, 0.0]),
            q=np.array([1.0, 0.0, 0.0, 0.0]),
            omega=np.zeros(3),
            m=m,
            t=0.0,
            dry_mass_kg=s2_dry,
        )

    def test_reserve_triggers_failed_unreachable_when_below_threshold(self):
        config = create_default_config(
            enable_s2_recovery=True, s2_landing_propellant_reserve_kg=500.0
        )
        s2_dry = config.stage2_dry_mass + config.payload_mass
        state = self._suborbital_state(m=s2_dry + 200.0, config=config)  # below reserve
        out, _ = compute_orbit_insertion_guidance(state.r, state.v, state.t, state.m, config=config)
        assert out["orbit_guidance_mode"] == "FAILED_UNREACHABLE"
        assert out["thrust_on"] is False

    def test_same_propellant_level_still_burns_when_above_reserve(self):
        config = create_default_config(
            enable_s2_recovery=True, s2_landing_propellant_reserve_kg=100.0
        )
        s2_dry = config.stage2_dry_mass + config.payload_mass
        state = self._suborbital_state(m=s2_dry + 200.0, config=config)  # above reserve
        out, _ = compute_orbit_insertion_guidance(state.r, state.v, state.t, state.m, config=config)
        assert out["orbit_guidance_mode"] != "FAILED_UNREACHABLE"

    def test_reserve_has_no_effect_when_s2_recovery_disabled(self):
        config = create_default_config(
            enable_s2_recovery=False, s2_landing_propellant_reserve_kg=500.0
        )
        s2_dry = config.stage2_dry_mass + config.payload_mass
        state = self._suborbital_state(m=s2_dry + 200.0, config=config)  # below reserve
        out, _ = compute_orbit_insertion_guidance(state.r, state.v, state.t, state.m, config=config)
        # Reserve is only applied when S2 recovery is enabled -- a
        # non-recovery mission's insertion capability must be unaffected.
        assert out["orbit_guidance_mode"] != "FAILED_UNREACHABLE"

    def test_zero_reserve_matches_unreserved_behavior(self):
        config_a = create_default_config(enable_s2_recovery=True, s2_landing_propellant_reserve_kg=0.0)
        config_b = create_default_config(enable_s2_recovery=False)
        s2_dry = config_a.stage2_dry_mass + config_a.payload_mass
        state = self._suborbital_state(m=s2_dry + 50.0, config=config_a)
        out_a, _ = compute_orbit_insertion_guidance(state.r, state.v, state.t, state.m, config=config_a)
        out_b, _ = compute_orbit_insertion_guidance(state.r, state.v, state.t, state.m, config=config_b)
        assert out_a["orbit_guidance_mode"] == out_b["orbit_guidance_mode"]
        assert out_a["thrust_on"] == out_b["thrust_on"]


class TestDeorbitReserveGating:
    def _deorbitable_state(self, m: float, config):
        s2_dry = config.stage2_dry_mass + config.payload_mass
        r_mag = C.R_EARTH + float(config.orbit_target_altitude_m)
        v_circ = float(np.sqrt(C.MU_EARTH / r_mag))
        return State(
            r=np.array([r_mag, 0.0, 0.0]),
            v=np.array([0.0, v_circ, 0.0]),
            q=np.array([1.0, 0.0, 0.0, 0.0]),
            omega=np.zeros(3),
            m=m,
            t=0.0,
            dry_mass_kg=s2_dry,
        )

    def test_deorbit_burn_disabled_below_reserve(self):
        config = create_default_config(
            enable_s2_recovery=True, s2_landing_propellant_reserve_kg=500.0
        )
        s2_dry = config.stage2_dry_mass + config.payload_mass
        state = self._deorbitable_state(m=s2_dry + 200.0, config=config)
        out, _ = compute_deorbit_guidance(state.r, state.v, state.t, state.m, config=config)
        assert out["thrust_on"] is False

    def test_deorbit_burn_enabled_above_reserve(self):
        config = create_default_config(
            enable_s2_recovery=True, s2_landing_propellant_reserve_kg=100.0
        )
        s2_dry = config.stage2_dry_mass + config.payload_mass
        state = self._deorbitable_state(m=s2_dry + 5000.0, config=config)
        out, _ = compute_deorbit_guidance(state.r, state.v, state.t, state.m, config=config)
        assert out["thrust_on"] is True

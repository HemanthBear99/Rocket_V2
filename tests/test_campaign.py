"""Unit tests for rlv_sim.campaign case generation and determinism.

Verifies the determinism claims from the verified test architecture report:
sample_monte_carlo_cases is reproducible for a fixed (runs, seed) and clipped
to +/-3 sigma; build_sensitivity_cases is deterministic with no RNG.
"""

import pytest

from rlv_sim.campaign import (
    PARAMETERS,
    build_sensitivity_cases,
    sample_monte_carlo_cases,
)


class TestSensitivityCases:
    def test_default_parameters_produce_baseline_plus_pairs(self):
        cases = build_sensitivity_cases()
        assert cases[0]["case_id"] == "baseline"
        assert cases[0]["z_scores"] == {}
        # baseline + (-1sigma, +1sigma) for each of the 6 default parameters
        assert len(cases) == 1 + 2 * len(PARAMETERS)

    def test_each_parameter_gets_a_minus_and_plus_case(self):
        cases = build_sensitivity_cases(parameters=["thrust"])
        ids = {c["case_id"] for c in cases}
        assert ids == {"baseline", "thrust_minus_1sigma", "thrust_plus_1sigma"}

    def test_z_scores_are_plus_minus_one(self):
        cases = build_sensitivity_cases(parameters=["isp"])
        by_id = {c["case_id"]: c for c in cases}
        assert by_id["isp_minus_1sigma"]["z_scores"] == {"isp": -1.0}
        assert by_id["isp_plus_1sigma"]["z_scores"] == {"isp": 1.0}

    def test_unknown_parameter_raises(self):
        with pytest.raises(ValueError, match="Unknown campaign parameter"):
            build_sensitivity_cases(parameters=["warp_drive"])

    def test_no_rng_involved_repeated_calls_identical(self):
        assert build_sensitivity_cases() == build_sensitivity_cases()


class TestMonteCarloCases:
    def test_same_seed_produces_identical_cases(self):
        a = sample_monte_carlo_cases(runs=10, seed=42)
        b = sample_monte_carlo_cases(runs=10, seed=42)
        assert a == b

    def test_different_seed_produces_different_cases(self):
        a = sample_monte_carlo_cases(runs=10, seed=42)
        b = sample_monte_carlo_cases(runs=10, seed=43)
        assert a != b

    def test_case_count_matches_runs(self):
        cases = sample_monte_carlo_cases(runs=25, seed=1)
        assert len(cases) == 25

    def test_case_ids_are_sequential_and_zero_padded(self):
        cases = sample_monte_carlo_cases(runs=3, seed=1)
        assert [c["case_id"] for c in cases] == ["mc_0001", "mc_0002", "mc_0003"]

    def test_all_parameters_present_in_each_case(self):
        cases = sample_monte_carlo_cases(runs=5, seed=7)
        for case in cases:
            assert set(case["z_scores"].keys()) == set(PARAMETERS)

    def test_z_scores_clipped_to_three_sigma(self):
        # A large run count with a fixed seed should still never exceed the
        # documented +/-3.0 clip on the underlying normal draws.
        cases = sample_monte_carlo_cases(runs=500, seed=1)
        all_z = [z for case in cases for z in case["z_scores"].values()]
        assert max(all_z) <= 3.0
        assert min(all_z) >= -3.0

    def test_zero_runs_raises(self):
        with pytest.raises(ValueError, match="runs must be positive"):
            sample_monte_carlo_cases(runs=0, seed=1)

    def test_negative_runs_raises(self):
        with pytest.raises(ValueError, match="runs must be positive"):
            sample_monte_carlo_cases(runs=-5, seed=1)

    def test_mode_field_is_monte_carlo(self):
        cases = sample_monte_carlo_cases(runs=1, seed=1)
        assert cases[0]["mode"] == "monte_carlo"

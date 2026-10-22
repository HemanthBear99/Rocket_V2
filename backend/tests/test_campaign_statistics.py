"""Campaign statistics, extended dispersions and the wind-profile context."""

import math

import numpy as np
import pytest

from rlv_sim.config.factory import create_test_config
from rlv_sim.core import utils
from rlv_sim.mission import campaign


def test_legacy_draws_unchanged_by_extended_set():
    legacy = campaign.sample_monte_carlo_cases(20, 42)
    extended = campaign.sample_monte_carlo_cases(20, 42, dispersions="extended")
    raw = np.clip(np.random.default_rng(42).normal(size=(20, len(campaign.PARAMETERS))), -3, 3)
    for index, (a, b) in enumerate(zip(legacy, extended)):
        for column, parameter in enumerate(campaign.PARAMETERS):
            assert a["z_scores"][parameter] == pytest.approx(raw[index, column])
            assert b["z_scores"][parameter] == a["z_scores"][parameter]
        assert set(campaign.EXTENDED_PARAMETERS) <= set(b["z_scores"])
        assert len(b["wind_mode_phases"]) == 4
        assert a["seed"] == 42


def test_clopper_pearson_matches_known_values():
    # 100/100 successes: one-sided 90% bound is 0.1 ** (1/100).
    assert campaign.clopper_pearson_lower(100, 100, 0.90) == pytest.approx(0.1 ** 0.01, abs=1e-6)
    # Bound sits below the point estimate and rises with confidence lowered.
    lower90 = campaign.clopper_pearson_lower(94, 100, 0.90)
    lower95 = campaign.clopper_pearson_lower(94, 100, 0.95)
    assert lower95 < lower90 < 0.94
    assert campaign.zero_failure_runs_required(0.9973, 0.90) == 852


def test_summary_reports_lower_bounds_and_tails():
    rows = [
        {"case_id": f"mc_{i}", "mode": "monte_carlo", "mission_success": i < 9,
         "touchdown_speed_mps": float(i), "z_scores": {"thrust": 0.1 * i}}
        for i in range(10)
    ]
    summary = campaign.summarize_results(rows)
    assert summary["mission_success_lower_bound_90pct"] < 0.9
    stats = summary["metrics"]["touchdown_speed_mps"]
    assert stats["p00135"] <= stats["p05"] <= stats["p95"] <= stats["p99865"]


def test_extended_case_applies_physical_overrides():
    base = create_test_config()
    case = campaign.sample_monte_carlo_cases(1, 7, dispersions="extended")[0]
    config = campaign.apply_case(base, case)
    assert len(config.runtime_wind_profile_modes) == 4
    z = case["z_scores"]
    expected = base.booster_descent_drag_scale * (1 + z["descent_drag"] * campaign._MC_DESCENT_DRAG_DISPERSION)
    assert config.booster_descent_drag_scale == pytest.approx(max(expected, 0.1))


def test_sweep_3sigma_cases_and_effect_scaling():
    cases = campaign.build_sensitivity_cases(("thrust",), sigma=3.0)
    assert [c["case_id"] for c in cases] == ["baseline", "thrust_minus_3sigma", "thrust_plus_3sigma"]
    rows = [
        {"case_id": "baseline", "mission_success": True, "touchdown_speed_mps": 2.0},
        {"case_id": "thrust_minus_3sigma", "mission_success": True, "touchdown_speed_mps": 1.0},
        {"case_id": "thrust_plus_3sigma", "mission_success": True, "touchdown_speed_mps": 4.0},
    ]
    effects = campaign.summarize_results(rows)["one_sigma_effects"]
    assert effects["thrust"]["touchdown_speed_mps"] == pytest.approx(0.5)


def test_wind_profile_context_is_scoped():
    r = np.array([utils.C.R_EARTH + 8_000.0, 0.0, 0.0])
    base = utils._wind_vector(r)
    modes = ((3.0, 10_000.0, math.pi / 2.0),)
    with utils.wind_profile_context(modes):
        perturbed = utils._wind_vector(r)
    after = utils._wind_vector(r)
    expected_delta = 3.0 * math.sin(2 * math.pi * 8_000.0 / 10_000.0 + math.pi / 2.0)
    assert np.linalg.norm(perturbed) - np.linalg.norm(base) == pytest.approx(expected_delta, abs=1e-6)
    assert np.allclose(after, base)


def test_wind_aware_reserve_adds_propellant_only_for_headwind():
    from rlv_sim.config.factory import create_default_config

    plain = create_default_config()
    head = create_default_config(recovery_reserve_wind_sensitivity_kg_per_mps=400.0,
                                 runtime_wind_offset_mps=-5.0)
    tail = create_default_config(recovery_reserve_wind_sensitivity_kg_per_mps=400.0,
                                 runtime_wind_offset_mps=5.0)
    assert head.stage1_landing_fuel_reserve_kg == pytest.approx(plain.stage1_landing_fuel_reserve_kg + 2000.0)
    assert tail.stage1_landing_fuel_reserve_kg == pytest.approx(plain.stage1_landing_fuel_reserve_kg)


def test_campaign_case_swaps_wind_reserve_without_compounding():
    from rlv_sim.config.factory import create_default_config

    base = create_default_config(recovery_reserve_wind_sensitivity_kg_per_mps=400.0,
                                 runtime_wind_offset_mps=-2.0)
    plain = create_default_config()
    case = {"case_id": "x", "mode": "monte_carlo", "z_scores": {"wind": -1.0}}
    config = campaign.apply_case(base, case)
    # Case wind = -2 - 5 = -7 m/s -> 2800 kg over the plain reserve.
    assert config.stage1_landing_fuel_reserve_kg == pytest.approx(plain.stage1_landing_fuel_reserve_kg + 2800.0)

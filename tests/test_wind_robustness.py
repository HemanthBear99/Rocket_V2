"""Wind robustness of booster recovery, plus guards added with the wind fixes.

The slow tests fly full missions with a steady wind offset and pin the
outcomes reached by the boostback impact feedback, the entry-capture ZEV
weighting and the final-approach velocity divert. Run with ``pytest -m slow``.
"""

import numpy as np
import pytest

from rlv_sim import _gfold, campaign, recovery
from rlv_sim._run_full_mission import run_full_mission
from rlv_sim.config_factory import create_default_config
from rlv_sim.mission_summary import assess_full_mission


def _fly(wind_mps, guidance="heuristic"):
    cfg = create_default_config(runtime_wind_offset_mps=wind_mps, booster_landing_guidance=guidance)
    result = run_full_mission(config=cfg, verbose=False)
    return assess_full_mission(result, cfg)


@pytest.mark.slow
@pytest.mark.parametrize("wind", [-3.0, 3.0])
def test_heuristic_lands_in_wind(wind):
    assessment = _fly(wind)
    assert assessment.landing.status == "landing_success", assessment.landing.reason
    assert assessment.landing.site_error_m < 10.0


@pytest.mark.slow
@pytest.mark.skipif(not _gfold.AVAILABLE, reason="cvxpy not installed")
def test_gfold_lands_in_tailwind_deterministically():
    first = _fly(3.0, "gfold")
    assert first.landing.status == "landing_success", first.landing.reason
    # Cold-started solves: an identical mission gives an identical result.
    second = _fly(3.0, "gfold")
    assert second.landing.touchdown_speed_mps == first.landing.touchdown_speed_mps
    assessment = second
    assert assessment.landing.status == "landing_success", assessment.landing.reason


def test_gfold_without_cvxpy_fails_loudly(monkeypatch):
    monkeypatch.setattr(_gfold, "AVAILABLE", False)
    cfg = create_default_config(booster_landing_guidance="gfold")
    with pytest.raises(ValueError, match="cvxpy"):
        run_full_mission(config=cfg, verbose=False)


def test_gfold_nodes_must_be_integer():
    with pytest.raises(ValueError, match="gfold_nodes"):
        create_default_config(gfold_nodes=25.5)


def test_campaign_cli_accepts_landing_guidance():
    assert campaign.parse_args(["--landing-guidance", "gfold"]).landing_guidance == "gfold"
    assert campaign.parse_args([]).landing_guidance == "heuristic"


def test_impact_prediction_is_memoised_for_identical_inputs():
    cfg = create_default_config()
    site = recovery.target_landing_site_eci(0.0, cfg.booster_landing_target_downrange_km, config=cfg)
    up = site / np.linalg.norm(site)
    r = site + up * 20000.0
    v = -300.0 * up + np.cross([0.0, 0.0, 7.2921159e-5], r)
    first = recovery.estimate_ballistic_impact_to_pad(r, v, 0.0, 0.0, cfg, mass_kg=30000.0)
    again = recovery.estimate_ballistic_impact_to_pad(r.copy(), v.copy(), 0.0, 0.0, cfg, mass_kg=30000.0)
    other = recovery.estimate_ballistic_impact_to_pad(r, v * 1.01, 0.0, 0.0, cfg, mass_kg=30000.0)
    assert again is first
    assert other is not first

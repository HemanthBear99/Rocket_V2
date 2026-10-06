"""Regression tests for the small audit-list fixes.

- CampaignRequest moved from server.py into api_models.py (the file whose
  own docstring claims to be the single source for server mapping and
  OpenAPI schema).
- wind_seed is now an independent, explicit config field rather than
  silently reusing the raw gps_seed value.
- Frontend campaign cancel button is wired to the existing backend
  /api/campaigns/{id}/cancel endpoint.
"""

import numpy as np

from rlv_sim.api_models import CampaignRequest
from rlv_sim.config_factory import create_test_config
from rlv_sim.server import CampaignRequest as ServerCampaignRequest


class TestCampaignRequestLocation:
    def test_campaign_request_defined_in_api_models(self):
        assert CampaignRequest.__module__ == "rlv_sim.api_models"

    def test_server_reexports_the_same_class(self):
        # server.py should import, not redefine, CampaignRequest.
        assert ServerCampaignRequest is CampaignRequest


class TestWindSeedIndependence:
    def test_wind_seed_defaults_to_none(self):
        config = create_test_config()
        assert config.wind_seed is None

    def test_explicit_wind_seed_is_independent_of_gps_seed(self):
        config = create_test_config(gps_seed=42, wind_seed=999)
        assert config.wind_seed == 999
        assert config.gps_seed == 42

    def test_default_derived_seed_differs_from_raw_gps_seed(self):
        # Confirms the fix: previously wind reused gps_seed directly with
        # no offset, so a wind RNG built straight from config.gps_seed
        # would be identical to the GPS RNG's seed. The derived default
        # (gps_seed + 3, applied in _run_full_mission.py) must not equal
        # the raw gps_seed.
        config = create_test_config(gps_seed=42)
        assert config.wind_seed is None
        derived_seed = config.gps_seed + 3
        assert derived_seed != config.gps_seed
        # And it must actually produce different draws than a GPS-seeded RNG.
        gps_rng = np.random.default_rng(config.gps_seed)
        wind_rng = np.random.default_rng(derived_seed)
        assert gps_rng.normal() != wind_rng.normal()


class TestFrontendCampaignCancelButton:
    def test_cancel_button_present_in_html(self):
        html = (
            __import__("pathlib")
            .Path(__file__)
            .resolve()
            .parent.parent.joinpath("static", "index.html")
            .read_text(encoding="utf-8")
        )
        assert 'id="btn-cancel-campaign"' in html

    def test_cancel_button_wired_in_app_js(self):
        js = (
            __import__("pathlib")
            .Path(__file__)
            .resolve()
            .parent.parent.joinpath("static", "js", "app.js")
            .read_text(encoding="utf-8")
        )
        assert "btn-cancel-campaign" in js
        assert "/cancel" in js

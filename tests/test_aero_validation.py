"""Tier 1 aerodynamic validation tests -- see aero_validation.py's module
docstring for what these do and do not prove (physical sanity of the
model's shape, not agreement with wind-tunnel/CFD/flight data).
"""

import pytest

from rlv_sim.aero_database import AeroDatabase
from rlv_sim.aero_validation import (
    build_geometry_deck_database,
    run_all_checks,
    run_deck_interpolation_checks,
    run_tier2_report,
)


class TestGeometryModelPhysicalSanity:
    @pytest.fixture(autouse=True)
    def _report(self):
        self.report = run_all_checks()

    def test_all_geometry_checks_pass(self):
        failed = [c for c in self.report["checks"] if not c["passed"]]
        assert not failed, f"Failed aero sanity checks: {failed}"

    def test_report_scope_is_labeled(self):
        # Guards against this report ever being read as "wind-tunnel
        # validated" -- the scope string must stay attached to the report.
        assert "not validation against wind-tunnel" in self.report["scope"]


class TestAeroDeckInterpolationSanity:
    def test_generated_deck_interpolation_passes(self):
        deck = build_geometry_deck_database()
        checks = run_deck_interpolation_checks(deck)
        failed = [c for c in checks if not c.passed]
        assert not failed, f"Failed interpolation sanity checks: {failed}"

    def test_unloaded_deck_reports_failure_not_exception(self):
        deck = AeroDatabase(filepath=None)
        checks = run_deck_interpolation_checks(deck)
        assert len(checks) == 1
        assert checks[0].name == "deck_loaded"
        assert checks[0].passed is False

    def test_run_all_checks_includes_deck_checks_when_deck_supplied(self):
        deck = build_geometry_deck_database()
        report = run_all_checks(deck=deck)
        names = {c["name"] for c in report["checks"]}
        assert "deck_loaded" in names
        assert "interpolation_stays_within_grid_bounds" in names


class TestTier2LiteratureComparison:
    @pytest.fixture(autouse=True)
    def _report(self):
        self.report = run_tier2_report()

    def test_all_tier2_checks_pass(self):
        failed = [c for c in self.report["checks"] if not c["passed"]]
        assert not failed, f"Failed Tier 2 literature checks: {failed}"

    def test_every_check_carries_a_source_citation(self):
        for check in self.report["checks"]:
            assert check["source"], f"Tier 2 check {check['name']} is missing a source citation"

    def test_report_scope_distinguishes_from_wind_tunnel_data(self):
        assert "not a point-by-point match" in self.report["scope"]

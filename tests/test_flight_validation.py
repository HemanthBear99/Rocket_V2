"""Guard the Falcon 9 telemetry comparison (docs/VALIDATION.md) from regressing."""

import pytest


@pytest.mark.slow
def test_meco_speed_matches_flight_telemetry():
    from rlv_sim.validation.flight_compare import load_flight, simulate

    real = load_flight("SpaceX_CRS-11")
    sim = simulate(6900.0)
    assert sim["meco_v"] == pytest.approx(real["meco_v"], rel=0.05)
    assert sim["landed"]

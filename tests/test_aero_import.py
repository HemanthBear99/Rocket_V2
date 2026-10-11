"""User aero data import (wind tunnel / CFD) -> aero deck."""

import pytest

from rlv_sim import aero_import
from rlv_sim.config_factory import create_test_config

CSV = "mach,alpha_deg,CD,CL\n" + "\n".join(
    f"{m},{a},{0.3 + 0.2 * (m == 1.1) + 0.001 * abs(a)},{0.03 * a}"
    for m in (0.2, 0.8, 1.1, 2.0, 6.0) for a in (-10, 0, 10)
)


def test_csv_grid_becomes_deck():
    deck = aero_import.parse_csv(CSV)
    assert deck["axes"]["mach"] == [0.2, 0.8, 1.1, 2.0, 6.0]
    assert deck["axes"]["alpha"] == [-10.0, 0.0, 10.0]
    assert aero_import.validate_deck(deck) == []
    assert deck["coefficients"]["CD"][2][1][0][0] == pytest.approx(0.5)


def test_incomplete_grid_rejected():
    with pytest.raises(ValueError, match="incomplete"):
        aero_import.parse_csv("\n".join(CSV.splitlines()[:-1]))


def test_nonpositive_cd_rejected():
    deck = aero_import.parse_csv(CSV.replace("\n6.0,0,0.3", "\n6.0,0,-0.3"))
    assert any("positive" in e for e in aero_import.validate_deck(deck))


def test_imported_deck_is_saved_compared_and_flies(tmp_path, monkeypatch):
    from rlv_sim._run_full_mission import run_full_mission

    report = aero_import.import_deck(CSV.encode(), "tunnel.csv", name="pytest deck")
    try:
        assert report["name"] == "user_pytest_deck"
        assert all(c["passed"] for c in report["checks"])
        assert any(r["mach"] == 1.1 and r["cd_imported"] == pytest.approx(0.5) for r in report["comparison"])
        cfg = create_test_config(max_time=20.0, aero_deck_path=report["aero_deck_path"])
        result = run_full_mission(config=cfg, verbose=False)
        assert result.ascent_log.get_series("altitude")[-1] > 0.5
    finally:
        aero_import.DECK_DIR.joinpath("user_pytest_deck.json").unlink(missing_ok=True)

"""Saved scenarios, run history and license keys."""

import os

import pytest

from rlv_sim import licensing, storage


def test_scenario_round_trip_and_bad_ids():
    meta = storage.save_scenario("Heavy payload / 11 t", {"vehicle": {"payload_mass": 11000}})
    assert meta["id"] == "heavy_payload_11_t"
    assert storage.get_scenario(meta["id"])["setup"]["vehicle"]["payload_mass"] == 11000
    assert any(s["id"] == meta["id"] for s in storage.list_scenarios())
    storage.delete_scenario(meta["id"])
    with pytest.raises(FileNotFoundError):
        storage.get_scenario(meta["id"])
    with pytest.raises(ValueError):
        storage.get_scenario("../../etc/passwd")


def test_run_history_filters_by_kind():
    m = storage.save_run("mission", "run a", {"series": {"time": [0, 1]}}, {"success": True})
    c = storage.save_run("campaign", "camp b", {"summary": {}}, {"success_rate": 0.9})
    assert {r["id"] for r in storage.list_runs("mission")} >= {m["id"]}
    assert c["id"] not in {r["id"] for r in storage.list_runs("mission")}
    assert storage.get_run(c["id"])["headline"]["success_rate"] == 0.9
    with pytest.raises(ValueError):
        storage.save_run("bogus", "x", {}, {})


def test_license_key_sign_verify_and_tamper():
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

    other = Ed25519PrivateKey.generate().private_bytes(
        serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8, serialization.NoEncryption())
    forged = licensing.issue_key(other, "Mallory", "pro", 365)
    with pytest.raises(ValueError):
        licensing.verify_key(forged)  # signed by a key that is not the app's
    with pytest.raises(ValueError):
        licensing.verify_key("not.a-key")


def test_trial_blocks_runs_only_when_enforced(monkeypatch):
    monkeypatch.setattr(licensing, "TRIAL_DAYS", 0)
    monkeypatch.delenv("RLV_ENFORCE_LICENSE", raising=False)
    assert licensing.status()["can_run"]  # running from source: not enforced
    monkeypatch.setenv("RLV_ENFORCE_LICENSE", "1")
    st = licensing.status()
    assert st["state"] == "expired" and not st["can_run"]
    with pytest.raises(PermissionError):
        licensing.require_license()
    os.environ.pop("RLV_ENFORCE_LICENSE", None)

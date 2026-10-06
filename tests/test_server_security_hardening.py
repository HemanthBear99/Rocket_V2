"""Regression tests for the security hardening pass.

1. Default bind host changed from "0.0.0.0" to "127.0.0.1" -- a plain
   `python -m rlv_sim.server` run no longer exposes the API on all network
   interfaces unless RLV_HOST is set explicitly.
2. The server now refuses to start (raises during lifespan startup) when
   RLV_ENV=production and no RLV_API_KEYS are configured, instead of only
   logging a warning and starting anyway. RLV_ALLOW_NO_AUTH=1 is the
   explicit, deliberate opt-out.
"""

import asyncio

import pytest

from rlv_sim.server import StartupConfigurationError, _resolve_host, app, lifespan


def _run(coro):
    # No pytest-asyncio dependency in this project; drive the coroutine
    # directly rather than adding a new test dependency for one file.
    return asyncio.run(coro)


class TestDefaultBindHost:
    def test_defaults_to_loopback(self, monkeypatch):
        monkeypatch.delenv("RLV_HOST", raising=False)
        assert _resolve_host() == "127.0.0.1"

    def test_explicit_host_respected(self, monkeypatch):
        monkeypatch.setenv("RLV_HOST", "0.0.0.0")
        assert _resolve_host() == "0.0.0.0"

    def test_explicit_specific_interface_respected(self, monkeypatch):
        monkeypatch.setenv("RLV_HOST", "10.0.0.5")
        assert _resolve_host() == "10.0.0.5"


async def _enter_lifespan_once():
    async with lifespan(app):
        pass


class TestProductionAuthEnforcement:
    def test_refuses_to_start_in_production_without_api_keys(self, monkeypatch):
        monkeypatch.setenv("RLV_ENV", "production")
        monkeypatch.delenv("RLV_API_KEYS", raising=False)
        monkeypatch.delenv("RLV_ALLOW_NO_AUTH", raising=False)
        with pytest.raises(StartupConfigurationError):
            _run(_enter_lifespan_once())

    def test_starts_in_production_with_api_keys_configured(self, monkeypatch):
        monkeypatch.setenv("RLV_ENV", "production")
        monkeypatch.setenv("RLV_API_KEYS", "some-secret-key")
        monkeypatch.delenv("RLV_ALLOW_NO_AUTH", raising=False)
        _run(_enter_lifespan_once())  # should not raise

    def test_explicit_opt_out_allows_unauthenticated_production(self, monkeypatch):
        monkeypatch.setenv("RLV_ENV", "production")
        monkeypatch.delenv("RLV_API_KEYS", raising=False)
        monkeypatch.setenv("RLV_ALLOW_NO_AUTH", "1")
        _run(_enter_lifespan_once())  # should not raise

    def test_non_production_env_unaffected(self, monkeypatch):
        monkeypatch.delenv("RLV_ENV", raising=False)
        monkeypatch.delenv("RLV_API_KEYS", raising=False)
        monkeypatch.delenv("RLV_ALLOW_NO_AUTH", raising=False)
        _run(_enter_lifespan_once())  # local/dev default stays unauthenticated

    def test_desktop_env_unaffected(self, monkeypatch):
        # desktop_app.py sets RLV_ENV=desktop specifically to avoid this
        # check, since the desktop shell binds to 127.0.0.1 only.
        monkeypatch.setenv("RLV_ENV", "desktop")
        monkeypatch.delenv("RLV_API_KEYS", raising=False)
        monkeypatch.delenv("RLV_ALLOW_NO_AUTH", raising=False)
        _run(_enter_lifespan_once())  # should not raise

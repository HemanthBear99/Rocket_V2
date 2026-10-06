"""End-to-end browser test that drives the real web UI via Playwright.

Starts the FastAPI server as a subprocess, launches headless Chromium,
and exercises the Mission Setup -> Live Telemetry -> Stop flow through
actual DOM interactions (not direct API calls).

Requires the ``pytest-playwright`` package and a Chromium browser binary:

    uv sync --extra test
    uv run playwright install chromium

This test is skipped automatically if the ``playwright`` package or its
browser binary is not available in the environment.
"""

import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

pytest.importorskip("playwright.sync_api")

from playwright.sync_api import sync_playwright

_REPO_ROOT = Path(__file__).resolve().parent.parent
_REPO_PARENT = _REPO_ROOT.parent


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def live_server():
    port = _free_port()
    base_url = f"http://127.0.0.1:{port}"
    proc = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "uvicorn",
            "rlv_sim.server:app",
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
        ],
        cwd=str(_REPO_ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env={
            **os.environ,
            "PYTHONPATH": str(_REPO_PARENT) + os.pathsep + os.environ.get("PYTHONPATH", ""),
        },
    )
    try:
        deadline = time.time() + 30
        last_err = None
        import urllib.error
        import urllib.request

        while time.time() < deadline:
            if proc.poll() is not None:
                out = proc.stdout.read().decode(errors="replace") if proc.stdout else ""
                raise RuntimeError(f"server process exited early:\n{out}")
            try:
                with urllib.request.urlopen(f"{base_url}/api/health", timeout=1) as resp:
                    if resp.status == 200:
                        break
            except (urllib.error.URLError, ConnectionError) as e:
                last_err = e
            time.sleep(0.25)
        else:
            raise RuntimeError(f"server did not become healthy in time: {last_err}")

        yield base_url
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=10)


def test_mission_launch_and_stop_via_ui(live_server):
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        try:
            page = browser.new_page()
            page.goto(live_server, wait_until="load")

            # --- Mission Setup: load validated defaults ---------------------
            load_btn = page.locator("#btn-load-defaults")
            load_btn.click()

            # Verify at least one generated field populated with a value.
            page.wait_for_selector("#form-mission input", state="attached")
            first_field_value = page.locator("#form-mission input").first.input_value()
            assert first_field_value.strip() != ""

            # --- Launch simulation -------------------------------------------
            page.locator("#btn-launch").click()

            # --- Live Telemetry view should become visible with RUNNING ------
            page.wait_for_selector("#view-telemetry:not(.hidden)", timeout=15000)
            page.wait_for_function(
                "document.querySelector('#status-text')?.textContent === 'RUNNING'",
                timeout=15000,
            )

            # --- At least one telemetry stat updates to a nonzero value -------
            def altitude_nonzero():
                text = page.locator("#stat-altitude").inner_text()
                digits = "".join(ch for ch in text if ch.isdigit() or ch == ".")
                try:
                    return float(digits) > 0
                except ValueError:
                    return False

            page.wait_for_function(
                """
                () => {
                    const el = document.querySelector('#stat-altitude');
                    if (!el) return false;
                    const digits = el.textContent.replace(/[^0-9.]/g, '');
                    const v = parseFloat(digits);
                    return !Number.isNaN(v) && v > 0;
                }
                """,
                timeout=20000,
            )
            assert altitude_nonzero()

            # --- Stop the simulation ------------------------------------------
            page.locator("#btn-stop").click()
            # Confirm the "Stop simulation?" modal.
            page.wait_for_selector("#confirm-overlay:not(.hidden)", timeout=5000)
            page.locator("#confirm-ok").click()

            page.wait_for_function(
                "document.querySelector('#status-text')?.textContent !== 'RUNNING'",
                timeout=15000,
            )
            status_text = page.locator("#status-text").inner_text()
            assert status_text.upper() in ("STOPPING", "STOPPED", "IDLE")
        finally:
            browser.close()

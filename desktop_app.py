"""Windows desktop entry point: hosts the FastAPI backend and opens a native window."""

from __future__ import annotations

import logging
import os
import socket
import sys
import threading
import time
from pathlib import Path

_PACKAGE_PARENT = str(Path(__file__).resolve().parent.parent)
if _PACKAGE_PARENT not in sys.path:
    sys.path.insert(0, _PACKAGE_PARENT)

import uvicorn
import webview

from rlv_sim.server import app

logger = logging.getLogger("rlv_desktop")


def _find_free_port(preferred: int = 8756) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind(("127.0.0.1", preferred))
            return preferred
        except OSError:
            s.bind(("127.0.0.1", 0))
            return s.getsockname()[1]


def _run_server(host: str, port: int) -> None:
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    server.run()


def _wait_for_server(host: str, port: int, timeout_s: float = 10.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.15)
    return False


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    os.environ.setdefault("RLV_ENV", "desktop")
    host = "127.0.0.1"
    port = _find_free_port()

    server_thread = threading.Thread(target=_run_server, args=(host, port), daemon=True)
    server_thread.start()

    if not _wait_for_server(host, port):
        logger.error("Backend did not start in time on %s:%s", host, port)
        webview.create_window(
            "Boostback - startup error",
            html=(
                "<body style='background:#05070b;color:#e6e6e6;font-family:sans-serif;padding:24px'>"
                "<h2>The simulation backend failed to start.</h2>"
                f"<p>No server responded on {host}:{port} within 10 seconds. "
                "Close this window and try again; if it keeps happening, run "
                "<code>python -m rlv_sim.server</code> from a terminal to see the error.</p></body>"
            ),
            width=640,
            height=260,
        )
        webview.start(private_mode=False)
        return

    webview.create_window(
        "Boostback",
        url=f"http://{host}:{port}/",
        width=1440,
        height=900,
        min_size=(1100, 700),
        background_color="#05070b",
    )
    webview.start(private_mode=False)


if __name__ == "__main__":
    main()

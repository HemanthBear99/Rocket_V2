"""Per-user data directory for saved scenarios, run history, imported aero
decks and the license key.

Windows: %APPDATA%\\Vyom; elsewhere ~/.vyom. Data from the app's
earlier name (Boostback) is moved over on first use. RLV_DATA_DIR
overrides both (tests point it at a temp dir). Kept outside the program
folder so it survives reinstalls and the one-file exe's temp extraction.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _migrate(old: Path, new: Path) -> None:
    """Keep scenarios, runs and the license when upgrading from Boostback."""
    if old.is_dir() and not new.exists():
        try:
            old.rename(new)
        except OSError:
            pass


def data_dir() -> Path:
    override = os.environ.get("RLV_DATA_DIR")
    if override:
        root = Path(override)
    elif os.name == "nt" and os.environ.get("APPDATA"):
        root = Path(os.environ["APPDATA"]) / "Vyom"
        _migrate(Path(os.environ["APPDATA"]) / "Boostback", root)
    else:
        root = Path.home() / ".vyom"
        _migrate(Path.home() / ".boostback", root)
    root.mkdir(parents=True, exist_ok=True)
    return root


def sub_dir(name: str) -> Path:
    path = data_dir() / name
    path.mkdir(parents=True, exist_ok=True)
    return path


def user_aero_dir() -> Path:
    return sub_dir("aero_decks")


def frontend_dir() -> Path:
    """Location of the web UI (repo ``frontend/``).

    RLV_FRONTEND_DIR overrides it; the PyInstaller build bundles the UI
    as ``frontend/`` inside its extraction directory.
    """
    override = os.environ.get("RLV_FRONTEND_DIR")
    if override:
        return Path(override)
    if getattr(sys, "frozen", False):
        return Path(getattr(sys, "_MEIPASS", ".")) / "frontend"
    # backend/rlv_sim/server/app_paths.py -> repo root
    return Path(__file__).resolve().parents[3] / "frontend"

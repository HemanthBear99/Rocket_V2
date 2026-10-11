"""Per-user data directory for saved scenarios, run history, imported aero
decks and the license key.

Windows: %APPDATA%\\Boostback; elsewhere ~/.boostback. RLV_DATA_DIR
overrides both (tests point it at a temp dir). Kept outside the program
folder so it survives reinstalls and the one-file exe's temp extraction.
"""

from __future__ import annotations

import os
from pathlib import Path


def data_dir() -> Path:
    override = os.environ.get("RLV_DATA_DIR")
    if override:
        root = Path(override)
    elif os.name == "nt" and os.environ.get("APPDATA"):
        root = Path(os.environ["APPDATA"]) / "Boostback"
    else:
        root = Path.home() / ".boostback"
    root.mkdir(parents=True, exist_ok=True)
    return root


def sub_dir(name: str) -> Path:
    path = data_dir() / name
    path.mkdir(parents=True, exist_ok=True)
    return path


def user_aero_dir() -> Path:
    return sub_dir("aero_decks")

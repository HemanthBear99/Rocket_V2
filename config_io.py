"""JSON import/export for reproducible mission configuration profiles."""

from __future__ import annotations

import json
from pathlib import Path

from .config_definition import SimulationConfig
from .config_factory import create_default_config


def load_config_json(path: str | Path) -> SimulationConfig:
    """Load validated configuration overrides from a JSON object."""
    source = Path(path)
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("Simulation configuration JSON must contain an object")
    try:
        return create_default_config(**payload)
    except TypeError as exc:
        raise ValueError(f"Invalid simulation configuration: {exc}") from exc


__all__ = ["load_config_json"]

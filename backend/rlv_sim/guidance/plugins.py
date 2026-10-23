"""User landing-guidance plugins.

A plugin is a Python file in the user plugin folder (``app_paths.user_plugin_dir()``,
e.g. ``%APPDATA%\\Vyom\\plugins``) or in the bundled ``plugin_examples`` folder.
Select it with ``booster_landing_guidance="plugin"`` and
``booster_landing_plugin="<file stem>"``. It must define::

    NAME = "My guidance"            # optional, shown in the app
    DESCRIPTION = "What it does"    # optional

    def landing_command(state: LandingState) -> sequence of 3 floats:
        # commanded acceleration from thrust, m/s^2, ECI frame
        # (include the counter-gravity part; the sim adds gravity itself)

The command is converted to a thrust direction and a throttle
(|a| / state.max_accel_mps2, clipped to [0, 1]). The sim still enforces the
vehicle limits: the touchdown tilt cone, attitude dynamics, engine spool
rate and the engine cutoff below 10 m once the descent rate is safe. The
minimum throttle is NOT enforced on the booster path (known limit; the
built-in booster guidance also commands deep throttle).
It is called once per guidance step during the landing burn only; boostback
and entry use the built-in guidance.

Plugins are chosen by name only, from those two folders: the API cannot load
a file from an arbitrary path or upload code.
"""

from __future__ import annotations

import importlib.util
import math
import re
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np

BUNDLED_DIR = Path(__file__).resolve().parent / "plugin_examples"
_NAME_RE = re.compile(r"^[A-Za-z0-9_\-]{1,64}$")
_loaded: dict[str, tuple[float, ModuleType]] = {}


@dataclass(frozen=True)
class LandingState:
    """What a plugin sees each landing-burn step. Vectors are ECI, SI units."""

    t: float                          # mission time, s
    altitude_m: float                 # above the landing pad's surface
    up: np.ndarray                    # local vertical unit vector at the pad
    position_error_m: np.ndarray      # horizontal vector from vehicle to pad
    velocity_mps: np.ndarray          # ground-relative velocity
    vertical_speed_mps: float         # + up
    horizontal_velocity_mps: np.ndarray
    mass_kg: float
    propellant_kg: float              # usable propellant left
    max_accel_mps2: float             # landing thrust / mass at full throttle
    gravity_mps2: float
    time_to_go_s: float               # built-in estimate of time to touchdown
    tilt_limit_deg: float             # sim-enforced cone around vertical
    target_descent_rate_mps: float    # touchdown speed the built-in law aims for


def _plugin_dirs() -> list[tuple[str, Path]]:
    from rlv_sim.server.app_paths import user_plugin_dir

    return [("user", user_plugin_dir()), ("bundled", BUNDLED_DIR)]


def _find(name: str) -> tuple[str, Path] | None:
    if not _NAME_RE.match(name or ""):
        return None
    for source, folder in _plugin_dirs():
        path = folder / f"{name}.py"
        if path.is_file():
            return source, path
    return None


def _load(name: str) -> ModuleType:
    found = _find(name)
    if found is None:
        raise ValueError(f"Guidance plugin {name!r} not found")
    path = found[1]
    mtime = path.stat().st_mtime
    cached = _loaded.get(name)
    if cached and cached[0] == mtime:
        return cached[1]
    spec = importlib.util.spec_from_file_location(f"vyom_plugin_{name}", path)
    if spec is None or spec.loader is None:
        raise ValueError(f"Guidance plugin {name!r} could not be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not callable(getattr(module, "landing_command", None)):
        raise TypeError(f"Guidance plugin {name!r} has no landing_command(state) function")
    _loaded[name] = (mtime, module)
    return module


def plugin_error(name: str) -> str | None:
    """Config-validation message for an unusable plugin name, else None."""
    if not _NAME_RE.match(name or ""):
        return ("booster_landing_plugin must name a plugin (letters, digits, '_' or '-') "
                f"when booster_landing_guidance='plugin', got {name!r}")
    try:
        _load(name)
    except Exception as exc:  # noqa: BLE001 - any import failure is a config error
        return f"booster_landing_plugin {name!r}: {exc}"
    return None


def list_plugins() -> list[dict[str, Any]]:
    """Every plugin file in the user and bundled folders, with load status."""
    seen: set[str] = set()
    rows = []
    for source, folder in _plugin_dirs():
        for path in sorted(folder.glob("*.py")):
            name = path.stem
            if name in seen or name.startswith("_") or not _NAME_RE.match(name):
                continue
            seen.add(name)
            row: dict[str, Any] = {"name": name, "source": source, "path": str(path)}
            try:
                module = _load(name)
                row["title"] = str(getattr(module, "NAME", name))
                row["description"] = str(getattr(module, "DESCRIPTION", ""))
                row["error"] = None
            except Exception as exc:  # noqa: BLE001 - report, don't crash the listing
                row["title"], row["description"], row["error"] = name, "", str(exc)
            rows.append(row)
    return rows


def landing_command(name: str, state: LandingState) -> tuple[np.ndarray, float]:
    """Run the plugin; return (unit thrust direction, throttle in [0, 1])."""
    module = _load(name)
    try:
        raw = module.landing_command(state)
    except Exception as exc:
        raise RuntimeError(f"Guidance plugin {name!r} raised: {exc}") from exc
    try:
        accel = np.asarray(raw, dtype=float).reshape(3)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(
            f"Guidance plugin {name!r} must return 3 numbers (m/s^2), got {raw!r}"
        ) from exc
    if not all(math.isfinite(x) for x in accel):
        raise RuntimeError(f"Guidance plugin {name!r} returned a non-finite command {accel.tolist()}")
    magnitude = float(np.linalg.norm(accel))
    if magnitude < 1e-9:
        return np.array(state.up, dtype=float), 0.0
    throttle = float(np.clip(magnitude / max(state.max_accel_mps2, 1e-6), 0.0, 1.0))
    return accel / magnitude, throttle

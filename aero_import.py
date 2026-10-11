"""Import user aero data (wind tunnel / CFD) as an aero deck.

Accepts either the native deck JSON ({"axes": ..., "coefficients": ...}) or a
CSV table with one row per grid point:

    mach,alpha_deg,CD[,CL][,CM]

The rows must cover every (mach, alpha) combination. Beta and fin deflection
are not varied (single 0 deg point). The deck is validated, saved inside the
package's aero_decks/ directory (the only place aero_deck_path may point),
and compared against the built-in Cd(Mach) model.
"""

from __future__ import annotations

import csv
import io
import itertools
import json
import math
import re
from pathlib import Path
from typing import Any

import numpy as np

from . import constants as C
from .aero_database import AeroDatabase
from .aero_validation import run_deck_interpolation_checks

DECK_DIR = Path(__file__).resolve().parent / "aero_decks"
USER_PREFIX = "user_"
COEFFICIENTS = ("CD", "CL", "CM")
MAX_GRID_POINTS = 20000


def parse_csv(text: str) -> dict[str, Any]:
    """CSV rows (mach, alpha_deg, CD[, CL, CM]) -> deck dict."""
    reader = csv.DictReader(io.StringIO(text))
    fields = {f.strip().lower(): f for f in (reader.fieldnames or [])}
    for required in ("mach", "alpha_deg", "cd"):
        if required not in fields:
            raise ValueError(f"CSV needs columns mach, alpha_deg, CD (missing {required!r})")
    coefs = [c for c in COEFFICIENTS if c.lower() in fields]
    points: dict[tuple[float, float], dict[str, float]] = {}
    for n, row in enumerate(reader, start=2):
        try:
            key = (float(row[fields["mach"]]), float(row[fields["alpha_deg"]]))
            points[key] = {c: float(row[fields[c.lower()]]) for c in coefs}
        except (TypeError, ValueError) as exc:
            raise ValueError(f"CSV line {n}: non-numeric value ({exc})") from exc
    machs = sorted({k[0] for k in points})
    alphas = sorted({k[1] for k in points})
    missing = [(m, a) for m in machs for a in alphas if (m, a) not in points]
    if missing:
        raise ValueError(f"CSV grid incomplete: {len(missing)} (mach, alpha) points missing, e.g. {missing[0]}")
    return {
        "axes": {"mach": machs, "alpha": alphas, "beta": [0.0], "fin": [0.0]},
        "coefficients": {
            c: [[[[points[(m, a)][c]]] for a in alphas] for m in machs] for c in coefs
        },
    }


def validate_deck(deck: dict[str, Any]) -> list[str]:
    """Return blocking errors (empty when the deck is usable)."""
    errors = []
    axes, coefs = deck.get("axes", {}), deck.get("coefficients", {})
    names = ("mach", "alpha", "beta", "fin")
    for name in names:
        values = axes.get(name)
        if not values:
            errors.append(f"axis {name!r} missing or empty")
        elif any(b <= a for a, b in itertools.pairwise(values)):
            errors.append(f"axis {name!r} must be strictly increasing")
    if errors:
        return errors
    shape = tuple(len(axes[n]) for n in names)
    if math.prod(shape) > MAX_GRID_POINTS:
        errors.append(f"grid too large ({math.prod(shape)} points, max {MAX_GRID_POINTS})")
    if "CD" not in coefs:
        errors.append("coefficient 'CD' is required")
    for name, table in coefs.items():
        arr = np.asarray(table, dtype=float)
        if arr.shape != shape:
            errors.append(f"{name} has shape {arr.shape}, expected {shape} (mach x alpha x beta x fin)")
        elif not np.all(np.isfinite(arr)):
            errors.append(f"{name} contains non-finite values")
        elif name == "CD" and np.any(arr <= 0.0):
            errors.append("CD must be positive everywhere")
    return errors


def deck_warnings(deck: dict[str, Any]) -> list[str]:
    machs = deck["axes"]["mach"]
    out = []
    if machs[0] > 0.3:
        out.append(f"Lowest Mach is {machs[0]}: below it CD is held constant.")
    if machs[-1] < 5.0:
        out.append(f"Highest Mach is {machs[-1]}: above it (most of ascent and entry) CD is held constant.")
    if len(deck["axes"]["alpha"]) == 1:
        out.append("Single angle of attack: CD will not vary with alpha.")
    if "CL" not in deck["coefficients"]:
        out.append("No CL column: the built-in lift model is used.")
    if "CM" not in deck["coefficients"]:
        out.append("No CM column: the built-in pitching-moment model is used.")
    return out


def _slug(name: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")[:40]
    return slug or "deck"


def import_deck(raw: bytes, filename: str, name: str | None = None) -> dict[str, Any]:
    """Parse, validate, save and compare one uploaded deck."""
    text = raw.decode("utf-8-sig")
    deck = json.loads(text) if filename.lower().endswith(".json") else parse_csv(text)
    if not isinstance(deck, dict):
        raise ValueError("Deck JSON must be an object with 'axes' and 'coefficients'")  # noqa: TRY004 - mapped to 422
    errors = validate_deck(deck)
    if errors:
        raise ValueError("; ".join(errors))
    DECK_DIR.mkdir(exist_ok=True)
    path = DECK_DIR / f"{USER_PREFIX}{_slug(name or Path(filename).stem)}.json"
    path.write_text(json.dumps(deck), encoding="utf-8")
    db = AeroDatabase(str(path))
    checks = [c.__dict__ for c in run_deck_interpolation_checks(db)]
    return {
        "name": path.stem,
        "aero_deck_path": str(path),
        "grid": {k: len(v) for k, v in deck["axes"].items()},
        "coefficients": sorted(deck["coefficients"]),
        "warnings": deck_warnings(deck),
        "checks": checks,
        "comparison": compare_to_model(db, deck["axes"]["mach"]),
    }


def compare_to_model(db: AeroDatabase, machs: list[float]) -> list[dict[str, float]]:
    """CD at zero alpha: imported deck vs the built-in Cd(Mach) table the sim
    uses when no deck is selected."""
    grid = sorted(set(machs) | {0.5, 0.9, 1.05, 1.3, 2.0, 3.0, 5.0})
    rows = []
    for m in grid:
        cd_user = db.interpolate_4d(m, 0.0, 0.0, 0.0, "CD")
        cd_geo = float(np.interp(m, C.MACH_BREAKPOINTS, C.CD_VALUES))
        rows.append({"mach": m, "cd_imported": float(cd_user), "cd_model": float(cd_geo),
                     "ratio": float(cd_user) / float(cd_geo) if cd_geo else float("nan")})
    return rows


def list_decks() -> list[dict[str, str]]:
    if not DECK_DIR.exists():
        return []
    return [{"name": p.stem, "aero_deck_path": str(p), "user": p.stem.startswith(USER_PREFIX)}
            for p in sorted(DECK_DIR.glob("*.json"))]

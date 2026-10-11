"""Saved scenarios (named setups) and run history (completed missions and
campaigns) as JSON files in the user data directory."""

from __future__ import annotations

import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .app_paths import sub_dir

_ID_RE = re.compile(r"^[a-z0-9_-]{1,80}$")
RUN_KINDS = ("mission", "campaign")


def _slug(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")[:40] or "untitled"


def _path(folder: str, item_id: str) -> Path:
    if not _ID_RE.match(item_id):
        raise ValueError(f"invalid id {item_id!r}")
    return sub_dir(folder) / f"{item_id}.json"


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _read(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


# ------------------------------------------------------------------ scenarios
def save_scenario(name: str, setup: dict[str, Any]) -> dict[str, Any]:
    item_id = _slug(name)
    record = {"id": item_id, "name": name.strip() or item_id, "saved_at": _now(), "setup": setup}
    _path("scenarios", item_id).write_text(json.dumps(record), encoding="utf-8")
    return {k: record[k] for k in ("id", "name", "saved_at")}


def list_scenarios() -> list[dict[str, Any]]:
    out = []
    for p in sorted(sub_dir("scenarios").glob("*.json")):
        rec = _read(p)
        out.append({k: rec.get(k) for k in ("id", "name", "saved_at")})
    return sorted(out, key=lambda r: r["saved_at"] or "", reverse=True)


def get_scenario(item_id: str) -> dict[str, Any]:
    return _read(_path("scenarios", item_id))


def delete_scenario(item_id: str) -> None:
    _path("scenarios", item_id).unlink(missing_ok=True)


# ------------------------------------------------------------------ run history
def save_run(kind: str, name: str, payload: dict[str, Any], headline: dict[str, Any]) -> dict[str, Any]:
    if kind not in RUN_KINDS:
        raise ValueError(f"kind must be one of {RUN_KINDS}")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    item_id = f"{kind}_{stamp}_{_slug(name)}"
    record = {"id": item_id, "kind": kind, "name": name.strip() or item_id, "saved_at": _now(),
              "headline": headline, "payload": payload}
    _path("runs", item_id).write_text(json.dumps(record), encoding="utf-8")
    return {k: record[k] for k in ("id", "kind", "name", "saved_at", "headline")}


def list_runs(kind: str | None = None) -> list[dict[str, Any]]:
    out = []
    for p in sub_dir("runs").glob("*.json"):
        rec = _read(p)
        if kind is None or rec.get("kind") == kind:
            out.append({k: rec.get(k) for k in ("id", "kind", "name", "saved_at", "headline")})
    return sorted(out, key=lambda r: r["saved_at"] or "", reverse=True)


def get_run(item_id: str) -> dict[str, Any]:
    return _read(_path("runs", item_id))


def delete_run(item_id: str) -> None:
    _path("runs", item_id).unlink(missing_ok=True)

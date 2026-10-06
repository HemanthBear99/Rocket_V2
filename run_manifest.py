"""Immutable run provenance for reproducible mission execution."""

from __future__ import annotations

import hashlib
import json
import platform
import sys
from collections.abc import Sequence
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from . import __version__
from .config_definition import SimulationConfig


def _hash_file(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_run_manifest(
    config: SimulationConfig,
    *,
    cli_argv: Sequence[str] | None = None,
    output_dir: str | Path | None = None,
    artifacts: dict[str, str] | None = None,
    assessment: dict[str, Any] | None = None,
    termination: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Build a machine-readable record of how a mission run was executed."""
    repo_root = Path(__file__).resolve().parent.parent
    lock_path = repo_root / "requirements-lock.txt"
    manifest: dict[str, Any] = {
        "schema_version": "1.0",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "package": {
            "name": "rlv-sim",
            "version": __version__,
        },
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
        },
        "inputs": {
            "cli_argv": list(cli_argv or []),
            "config": asdict(config),
        },
        "dependencies": {
            "requirements_lock_sha256": _hash_file(lock_path),
        },
        "outputs": {
            "directory": str(output_dir) if output_dir is not None else None,
            "artifacts": artifacts or {},
        },
        "results": {
            "validation_version": "1.0",
            "assessment": assessment,
            "termination": termination,
        },
    }
    return manifest


def _json_safe(value: Any) -> Any:
    """Recursively coerce manifest values to JSON-serializable types."""
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


def write_run_manifest(
    manifest: dict[str, Any],
    output_dir: str | Path,
    filename: str = "run_manifest.json",
) -> str:
    """Write the run manifest next to mission outputs."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    path = out / filename
    manifest["outputs"]["artifacts"]["run_manifest"] = str(path)
    safe_manifest = _json_safe(manifest)
    path.write_text(json.dumps(safe_manifest, indent=2, sort_keys=True), encoding="utf-8")
    return str(path)
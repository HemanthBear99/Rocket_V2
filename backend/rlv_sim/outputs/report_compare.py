"""Load and compare mission assessment reports (run manifest or legacy JSON)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def _numeric(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _assessment_from_report(report: dict[str, Any]) -> dict[str, Any] | None:
    assessment = report.get("mission_assessment")
    if isinstance(assessment, dict):
        return assessment
    results = report.get("results")
    if isinstance(results, dict):
        nested = results.get("assessment")
        if isinstance(nested, dict):
            return nested
    return None


def load_run_report(report_path: str | Path) -> dict[str, Any]:
    """Load a run manifest or legacy validation_report JSON file."""
    path = Path(report_path)
    if not path.exists():
        raise FileNotFoundError(f"Run report not found: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def compare_assessments(
    reference: dict[str, Any],
    candidate: dict[str, Any],
    tolerances: dict[str, float] | None = None,
) -> dict[str, Any]:
    """Compare two run reports and return metric differences."""
    if tolerances is None:
        tolerances = {
            "orbit.perigee_altitude_km": 5.0,
            "orbit.apogee_altitude_km": 5.0,
            "orbit.eccentricity": 0.01,
            "landing.touchdown_speed_mps": 2.0,
            "landing.site_error_m": 100.0,
        }

    def _flatten(prefix: str, source: Any, target: dict[str, float]) -> None:
        if isinstance(source, dict):
            for key, value in source.items():
                _flatten(f"{prefix}.{key}" if prefix else key, value, target)
        elif _numeric(source):
            target[prefix] = float(source)

    ref_assessment = _assessment_from_report(reference)
    cand_assessment = _assessment_from_report(candidate)
    if not isinstance(ref_assessment, dict) or not isinstance(cand_assessment, dict):
        return {
            "comparison": {},
            "status_comparison": {},
            "missing_metrics": ["mission_assessment"],
            "overall_pass": False,
        }

    ref_flat: dict[str, float] = {}
    cand_flat: dict[str, float] = {}
    _flatten("", ref_assessment, ref_flat)
    _flatten("", cand_assessment, cand_flat)

    differences: dict[str, dict[str, Any]] = {}
    missing_metrics: list[str] = []
    for key in sorted(tolerances):
        if key not in ref_flat or key not in cand_flat:
            missing_metrics.append(key)
            continue
        ref_value = ref_flat[key]
        cand_value = cand_flat[key]
        diff = cand_value - ref_value
        threshold = tolerances[key]
        differences[key] = {
            "reference": ref_value,
            "candidate": cand_value,
            "difference": diff,
            "threshold": threshold,
            "passes": abs(diff) <= threshold,
        }

    def _path(source: dict[str, Any], dotted: str) -> Any:
        value: Any = source
        for part in dotted.split("."):
            if not isinstance(value, dict) or part not in value:
                return None
            value = value[part]
        return value

    status_fields = (
        "mission_success",
        "status",
        "separation_occurred",
        "orbit.success",
        "orbit.status",
        "landing.success",
        "landing.status",
    )
    status_comparison: dict[str, dict[str, Any]] = {}
    for key in status_fields:
        ref_value = _path(ref_assessment, key)
        cand_value = _path(cand_assessment, key)
        status_comparison[key] = {
            "reference": ref_value,
            "candidate": cand_value,
            "passes": ref_value is not None and cand_value is not None and ref_value == cand_value,
        }

    return {
        "comparison": differences,
        "status_comparison": status_comparison,
        "missing_metrics": missing_metrics,
        "overall_pass": (
            not missing_metrics
            and bool(differences)
            and all(item["passes"] for item in differences.values())
            and all(item["passes"] for item in status_comparison.values())
        ),
    }


__all__ = ["compare_assessments", "load_run_report"]
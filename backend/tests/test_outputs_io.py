"""Unit tests for config loading, run manifests, report comparison and telemetry."""

import json
from dataclasses import replace

import numpy as np
import pytest

from rlv_sim.config.factory import create_default_config
from rlv_sim.config.loader import load_config_json
from rlv_sim.core.state import create_initial_state
from rlv_sim.mission.full_mission import MissionProgress
from rlv_sim.outputs.manifest import (
    _hash_file,
    _json_safe,
    build_run_manifest,
    credibility_block,
    write_run_manifest,
)
from rlv_sim.outputs.report_compare import compare_assessments, load_run_report
from rlv_sim.outputs.telemetry import (
    _grid_fin_status,
    _landing_leg_status,
    extract_mission_progress,
    extract_telemetry_point,
)

# --- config loader -----------------------------------------------------------


def test_load_config_json_applies_overrides(tmp_path):
    path = tmp_path / "cfg.json"
    path.write_text(json.dumps({"dt": 0.1, "max_time": 50.0}), encoding="utf-8")
    config = load_config_json(path)
    assert config.dt == 0.1
    assert config.max_time == 50.0


def test_load_config_json_rejects_non_object(tmp_path):
    path = tmp_path / "cfg.json"
    path.write_text("[1, 2]", encoding="utf-8")
    with pytest.raises(ValueError, match="must contain an object"):
        load_config_json(path)


def test_load_config_json_rejects_unknown_field(tmp_path):
    path = tmp_path / "cfg.json"
    path.write_text(json.dumps({"not_a_field": 1}), encoding="utf-8")
    with pytest.raises(ValueError, match="Invalid simulation configuration"):
        load_config_json(path)


# --- run manifest ------------------------------------------------------------


def test_hash_file_missing_returns_none(tmp_path):
    assert _hash_file(tmp_path / "absent") is None


def test_hash_file_matches_sha256(tmp_path):
    import hashlib

    path = tmp_path / "f.bin"
    path.write_bytes(b"abc")
    assert _hash_file(path) == hashlib.sha256(b"abc").hexdigest()


def test_json_safe_coerces_paths_and_tuples(tmp_path):
    out = _json_safe({1: (tmp_path, [tmp_path])})
    assert out == {"1": [str(tmp_path), [str(tmp_path)]]}


def test_credibility_score_is_minimum_pedigree():
    block = credibility_block()
    scores = [v["pedigree"] for v in block["model_inputs"].values()]
    scores += [v["pedigree"] for v in block["dispersions"].values()]
    assert block["input_pedigree_score"] == min(scores)


def test_manifest_round_trips_to_disk(tmp_path):
    manifest = build_run_manifest(
        create_default_config(),
        cli_argv=["--quiet"],
        output_dir=tmp_path,
        termination={"ascent": "orbit"},
    )
    assert manifest["inputs"]["cli_argv"] == ["--quiet"]
    assert manifest["outputs"]["directory"] == str(tmp_path)
    path = write_run_manifest(manifest, tmp_path / "out")
    loaded = json.loads((tmp_path / "out" / "run_manifest.json").read_text(encoding="utf-8"))
    assert loaded["outputs"]["artifacts"]["run_manifest"] == path
    assert loaded["results"]["termination"] == {"ascent": "orbit"}


def test_manifest_defaults_without_output_dir():
    manifest = build_run_manifest(create_default_config())
    assert manifest["outputs"] == {"directory": None, "artifacts": {}}
    assert manifest["inputs"]["cli_argv"] == []


# --- report comparison -------------------------------------------------------


def _assessment(perigee=200.0, landing_ok=True):
    return {
        "mission_success": True,
        "status": "ok",
        "separation_occurred": True,
        "orbit": {
            "perigee_altitude_km": perigee,
            "apogee_altitude_km": 210.0,
            "eccentricity": 0.001,
            "success": True,
            "status": "orbit",
            "flag": True,  # bools are not metrics
        },
        "landing": {
            "touchdown_speed_mps": 1.5,
            "site_error_m": 10.0,
            "success": landing_ok,
            "status": "landed",
        },
    }


def test_load_run_report_missing_file(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_run_report(tmp_path / "nope.json")


def test_load_run_report_reads_json(tmp_path):
    path = tmp_path / "r.json"
    path.write_text('{"a": 1}', encoding="utf-8")
    assert load_run_report(path) == {"a": 1}


def test_compare_identical_reports_pass():
    report = {"mission_assessment": _assessment()}
    result = compare_assessments(report, report)
    assert result["overall_pass"] is True
    assert result["missing_metrics"] == []
    assert "orbit.flag" not in result["comparison"]


def test_compare_manifest_shape_and_out_of_tolerance():
    ref = {"results": {"assessment": _assessment()}}
    cand = {"mission_assessment": _assessment(perigee=220.0)}
    result = compare_assessments(ref, cand)
    item = result["comparison"]["orbit.perigee_altitude_km"]
    assert item["difference"] == pytest.approx(20.0)
    assert item["passes"] is False
    assert result["overall_pass"] is False


def test_compare_status_mismatch_fails():
    ref = {"mission_assessment": _assessment()}
    cand = {"mission_assessment": _assessment(landing_ok=False)}
    result = compare_assessments(ref, cand)
    assert result["status_comparison"]["landing.success"]["passes"] is False
    assert result["overall_pass"] is False


def test_compare_without_assessment():
    result = compare_assessments({}, {"results": {"assessment": None}})
    assert result["missing_metrics"] == ["mission_assessment"]
    assert result["overall_pass"] is False


def test_compare_missing_metric_and_custom_tolerance():
    ref = {"mission_assessment": _assessment()}
    result = compare_assessments(ref, ref, tolerances={"orbit.absent": 1.0})
    assert result["missing_metrics"] == ["orbit.absent"]
    assert result["overall_pass"] is False


# --- telemetry ---------------------------------------------------------------


@pytest.mark.parametrize(
    ("fraction", "expected"),
    [(None, "STOWED"), (0.0, "STOWED"), (0.5, "DEPLOYING"), (1.0, "DEPLOYED")],
)
def test_grid_fin_status(fraction, expected):
    assert _grid_fin_status({"grid_fin_deployed_fraction": fraction}) == expected


@pytest.mark.parametrize(
    ("status", "expected"),
    [(None, "STOWED"), ("", "STOWED"), ("locked", "DEPLOYED"), ("failed", "FAILED")],
)
def test_landing_leg_status(status, expected):
    assert _landing_leg_status({"landing_leg_status": status}) == expected


def test_telemetry_point_at_liftoff():
    config = create_default_config()
    state = create_initial_state(config)
    guidance = {"throttle": 1.0, "pitch_angle": np.pi / 2, "v_rel": np.zeros(3)}
    point = extract_telemetry_point(state, guidance, "VERTICAL", config)
    assert point["phase"] == "VERTICAL"
    assert point["throttle"] == 1.0
    assert point["pitch_angle"] == pytest.approx(90.0)
    assert point["downrange"] == pytest.approx(0.0, abs=1.0)
    assert point["dynamic_pressure"] == pytest.approx(0.0, abs=1.0)


def test_telemetry_point_without_atmosphere():
    config = replace(create_default_config(), enable_atmosphere=False)
    state = create_initial_state(config)
    point = extract_telemetry_point(state, {}, "COAST", config)
    assert point["dynamic_pressure"] == 0.0
    assert point["heat_flux"] == 0.0


def test_mission_progress_ascent_reads_s1_fuel():
    config = create_default_config()
    state = create_initial_state(config)
    payload = extract_mission_progress(MissionProgress("ascent", state, {}, "VERTICAL"), config)
    assert payload["fuel_stage"] == "S1"
    assert payload["fuel_remaining_pct"] > 99.0
    assert "booster_altitude" not in payload


def test_mission_progress_dual_adds_booster_fields():
    config = create_default_config()
    state = create_initial_state(config)
    booster = create_initial_state(config)
    progress = MissionProgress(
        "dual",
        state,
        {},
        "COAST",
        booster_state=booster,
        booster_guidance={"phase": "BOOSTBACK", "grid_fin_deployed_fraction": 1.0},
    )
    payload = extract_mission_progress(progress, config)
    assert payload["fuel_stage"] == "S2"
    assert payload["booster_phase"] == "BOOSTBACK"
    assert payload["booster_grid_fins"] == "DEPLOYED"
    assert payload["booster_landing_legs"] == "STOWED"
    assert payload["booster_landing_error"] >= 0.0


def test_mission_progress_dual_without_booster_state():
    config = create_default_config()
    state = create_initial_state(config)
    payload = extract_mission_progress(MissionProgress("dual", state, {}, "COAST"), config)
    assert "booster_altitude" not in payload

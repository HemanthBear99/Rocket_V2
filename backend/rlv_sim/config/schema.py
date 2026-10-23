"""Machine-readable description of every SimulationConfig field.

Drives the app's "All settings" panel: type, default (the research profile
from create_default_config), unit, bounds mirrored from
SimulationConfig.validate(), a group, and the source comment as help text.
"""

from __future__ import annotations

import dataclasses
import inspect
import math
import re
import typing
from functools import lru_cache
from typing import Any

from rlv_sim.config.definition import SimulationConfig
from rlv_sim.config.field_docs import FIELD_DOCS

# Fields set elsewhere in the app (aero import page) or not user-facing.
_HIDDEN = {"verbose", "aero_deck_path"}

# validate() range checks that are not simple positive/nonneg rules:
# name -> (min, max, min_exclusive, max_exclusive)
_RANGES: dict[str, tuple[float | None, float | None, bool, bool]] = {
    "attitude_integral_windup_fraction": (0.0, 1.0, True, False),
    "max_gimbal_angle_deg": (0.0, 30.0, False, False),
    "min_engine_throttle_fraction": (0.0, 1.0, True, False),
    "max_engine_throttle_fraction": (0.0, 1.0, True, False),
    "launch_site_lat_deg": (-90.0, 90.0, False, False),
    "launch_site_lon_deg": (-180.0, 180.0, False, False),
    "booster_landing_site_lat_deg": (-90.0, 90.0, False, False),
    "booster_landing_site_lon_deg": (-180.0, 180.0, False, False),
    "orbit_ecc_max": (0.0, 1.0, True, True),
    "booster_landing_planning_throttle": (0.3, 1.0, False, False),
    "stage1_engine_count": (1, 50, False, False),
    "boostback_engine_count": (1, 50, False, False),
    "entry_engine_count": (1, 50, False, False),
    "landing_engine_count": (1, 50, False, False),
    "gfold_nodes": (5, 200, False, False),
    "gfold_glideslope_deg": (0.0, 90.0, True, True),
    "dt": (0.0, None, True, False),
    "demo_coast_max_dt": (0.0, None, True, False),
    "tvc_lever_arm_m": (0.0, None, True, False),
}
_RULE_BOUNDS = {
    "positive": (0.0, True),
    "nonneg": (0.0, False),
    "at_least_1": (1.0, False),
}
_CHOICES = {
    "booster_landing_guidance": ["heuristic", "gfold", "plugin"],
    "gravity_model": ["central", "j2", "egm96"],
}
# (prefix or exact name, group). First match wins.
_GROUPS: tuple[tuple[str, str], ...] = (
    ("runtime_", "Wind & dispersions"),
    ("wind_", "Wind & dispersions"),
    ("enable_stochastic_wind", "Wind & dispersions"),
    ("booster_descent_drag_scale", "Wind & dispersions"),
    ("stage1_engine_ring", "Vehicle geometry & mass properties"),
    ("stage1_nozzle", "Vehicle geometry & mass properties"),
    ("stage2_nozzle", "Vehicle geometry & mass properties"),
    ("stack_", "Vehicle geometry & mass properties"),
    ("booster_cg_", "Vehicle geometry & mass properties"),
    ("booster_ixx", "Vehicle geometry & mass properties"),
    ("booster_izz", "Vehicle geometry & mass properties"),
    ("booster_tail_first_cp", "Vehicle geometry & mass properties"),
    ("vehicle_diameter_m", "Vehicle geometry & mass properties"),
    ("stage1_", "Vehicle"),
    ("stage2_", "Vehicle"),
    ("payload_", "Vehicle"),
    ("boostback_engine", "Vehicle"),
    ("entry_engine", "Vehicle"),
    ("landing_engine", "Vehicle"),
    ("min_engine", "Vehicle"),
    ("max_engine", "Vehicle"),
    ("engine_", "Vehicle"),
    ("enable_engine", "Vehicle"),
    ("gfold_", "Landing guidance"),
    ("booster_landing_guidance", "Landing guidance"),
    ("booster_landing_plugin", "Landing guidance"),
    ("booster_terminal", "Landing guidance"),
    ("booster_divert", "Landing guidance"),
    ("booster_max_divert", "Landing guidance"),
    ("booster_vertical_load", "Landing guidance"),
    ("booster_fuel_critical", "Landing guidance"),
    ("booster_landing", "Landing guidance"),
    ("booster_powered_descent", "Landing guidance"),
    ("booster_pad_tolerance", "Landing guidance"),
    ("booster_enforce_pad", "Landing guidance"),
    ("landing_target_", "Landing guidance"),
    ("enable_movable_landing_target", "Landing guidance"),
    ("booster_entry", "Boostback & entry"),
    ("booster_boostback", "Boostback & entry"),
    ("booster_apogee", "Boostback & entry"),
    ("booster_flip", "Boostback & entry"),
    ("booster_late_entry", "Boostback & entry"),
    ("booster_recovery_max", "Boostback & entry"),
    ("stage1_landing_fuel_reserve", "Recovery propellant"),
    ("auto_size_recovery_reserve", "Recovery propellant"),
    ("recovery_reserve_", "Recovery propellant"),
    ("booster_", "Booster recovery (other)"),
    ("enable_recovery_planner", "Booster recovery (other)"),
    ("grid_fin", "Grid fins & landing legs"),
    ("enable_grid_fins", "Grid fins & landing legs"),
    ("landing_leg", "Grid fins & landing legs"),
    ("enable_landing_legs", "Grid fins & landing legs"),
    ("s2_", "Stage-2 recovery"),
    ("enable_s2_recovery", "Stage-2 recovery"),
    ("orbit_", "Orbit & mission"),
    ("target_inclination", "Orbit & mission"),
    ("stage_sep", "Orbit & mission"),
    ("launch_site", "Orbit & mission"),
    ("separation_", "Orbit & mission"),
    ("enable_separation", "Orbit & mission"),
    ("gravity_turn", "Ascent guidance"),
    ("min_velocity_for_turn", "Ascent guidance"),
    ("pitchover", "Ascent guidance"),
    ("ascent_", "Ascent guidance"),
    ("kp_", "Attitude control"),
    ("kd_", "Attitude control"),
    ("ki_", "Attitude control"),
    ("attitude_", "Attitude control"),
    ("recovery_attitude", "Attitude control"),
    ("max_torque", "Attitude control"),
    ("max_gimbal", "Attitude control"),
    ("tvc_", "Attitude control"),
    ("rcs_", "Attitude control"),
    ("enable_rcs", "Attitude control"),
    ("gps_", "Sensors & navigation"),
    ("imu_", "Sensors & navigation"),
    ("enable_gps", "Sensors & navigation"),
    ("enable_imu", "Sensors & navigation"),
    ("landing_altimeter", "Sensors & navigation"),
    ("enable_landing_altimeter", "Sensors & navigation"),
    ("use_navigation", "Sensors & navigation"),
    ("abort_", "Aborts & limits"),
    ("enable_abort", "Aborts & limits"),
    ("q_alpha_max", "Aborts & limits"),
    ("enable_atmosphere", "Environment"),
    ("enable_high_fidelity", "Environment"),
    ("enable_upper_atmosphere", "Environment"),
    ("atmosphere_model", "Environment"),
    ("enable_drag", "Environment"),
    ("enable_lift", "Environment"),
    ("enable_j2", "Environment"),
    ("j2_", "Environment"),
    ("gravity_model", "Environment"),
)
_UNITS = (
    ("_kg_per_mps", "kg/(m/s)"), ("_pa_rad", "Pa·rad"), ("_kgm2", "kg·m²"),
    ("_mps2", "m/s²"), ("_mps", "m/s"), ("_kg", "kg"), ("_km", "km"), ("_deg", "deg"),
    ("_pa", "Pa"), ("_rad", "rad"), ("_m2", "m²"), ("_n", "N"), ("_s", "s"), ("_m", "m"),
)
# Form fields the Mission Setup tabs already edit (server.app.map_config).
FORM_FIELDS = frozenset({
    "dt", "max_time", "orbit_target_altitude_m", "orbit_insertion_start_altitude_m",
    "target_inclination_deg", "stage_sep_velocity", "booster_landing_site_lat_deg",
    "booster_landing_site_lon_deg", "enable_grid_fins", "enable_landing_legs",
    "enable_j2", "enable_atmosphere", "enable_drag", "enable_lift", "enable_imu",
    "enable_gps", "enable_landing_altimeter", "stage1_engine_count",
    "boostback_engine_count", "entry_engine_count", "landing_engine_count",
    "vehicle_diameter_m", "stage1_engine_thrust_n", "runtime_isp_scale",
    "stage1_prop_mass", "max_gimbal_angle_deg", "min_engine_throttle_fraction",
    "max_engine_throttle_fraction", "stage1_dry_mass", "stage2_dry_mass",
    "stage2_prop_mass", "payload_mass", "stage2_thrust_vac", "stage2_isp_vac",
    "enable_demo_mode", "demo_coast_max_dt", "auto_size_recovery_reserve",
    "enable_s2_recovery",
})


def _group(name: str) -> str:
    for prefix, group in _GROUPS:
        if name.startswith(prefix):
            return group
    return "General"


def _unit(name: str) -> str:
    for suffix, unit in _UNITS:
        if name.endswith(suffix):
            return unit
    return ""


def _kind(tp: Any) -> tuple[str, bool, list[str] | None]:
    """(kind, nullable, choices) for a dataclass field type."""
    if typing.get_origin(tp) is typing.Literal:
        return "choice", False, [str(x) for x in typing.get_args(tp)]
    args = typing.get_args(tp)
    nullable = type(None) in args
    base = next((a for a in args if a is not type(None)), tp) if args else tp
    if base is bool:
        return "bool", nullable, None
    if base is int:
        return "int", nullable, None
    if base is float:
        return "float", nullable, None
    if base is tuple:
        return "wind_modes", nullable, None
    return "str", nullable, None


def _comments() -> dict[str, str]:
    """Field name -> the comment block above it plus any inline comment."""
    try:
        lines = inspect.getsource(SimulationConfig).splitlines()
    except OSError:  # no source (frozen build without definition.py)
        return {}
    field_re = re.compile(r"^\s{4}(\w+)\s*:\s*[^=]+?(?:=\s*(.*))?$")
    out: dict[str, str] = {}
    pending: list[str] = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("#"):
            pending.append(stripped.lstrip("#").strip())
            continue
        match = field_re.match(line)
        if match and not stripped.startswith(("def ", "@", "return")):
            parts = list(pending)
            tail = match.group(2) or ""
            if "#" in tail:
                parts.append(tail.split("#", 1)[1].strip())
            if parts:
                out[match.group(1)] = " ".join(p for p in parts if p)
        pending = []
    return out


def _rule_bounds() -> dict[str, tuple[float, bool]]:
    try:
        source = inspect.getsource(SimulationConfig.validate)
    except OSError:
        return {}
    return {
        name: _RULE_BOUNDS[rule]
        for name, rule in re.findall(r'\("(\w+)",\s*"(positive|nonneg|at_least_1)"\)', source)
    }


def _json_value(value: Any) -> Any:
    if isinstance(value, tuple):
        return [_json_value(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


@lru_cache(maxsize=1)
def config_schema() -> list[dict[str, Any]]:
    from rlv_sim.config.factory import create_default_config

    defaults = create_default_config(verbose=False)
    comments = _comments()
    rules = _rule_bounds()
    rows = []
    for field in dataclasses.fields(SimulationConfig):
        name = field.name
        if name in _HIDDEN:
            continue
        kind, nullable, choices = _kind(field.type)
        if name in _CHOICES:
            kind, choices = "choice", _CHOICES[name]
        row: dict[str, Any] = {
            "name": name,
            "kind": kind,
            "nullable": nullable,
            "default": _json_value(getattr(defaults, name)),
            "unit": _unit(name),
            "group": _group(name),
            "description": FIELD_DOCS.get(name) or comments.get(name, ""),
            "on_form": name in FORM_FIELDS,
        }
        if choices:
            row["choices"] = choices
        if name in _RANGES:
            lo, hi, lo_ex, hi_ex = _RANGES[name]
            row.update(min=lo, max=hi, min_exclusive=lo_ex, max_exclusive=hi_ex)
        elif name in rules:
            lo, lo_ex = rules[name]
            row.update(min=lo, max=None, min_exclusive=lo_ex, max_exclusive=False)
        rows.append(row)
    return rows

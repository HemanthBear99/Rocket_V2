"""Pre-launch feasibility check: rocket-equation budgets against the mission.

Cheap (no simulation) so the UI can run it on every edit. Loss and recovery
thresholds are calibrated against full-mission runs of the reference vehicle:
payloads up to 11 t reach orbit, 12 t does not, and the booster's RTLS uses
~3600 m/s (67.5 t reserve, landing with ~5.4 t left).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

from . import constants as C
from .config_definition import SimulationConfig

G0 = 9.80665
# Gravity + drag + steering losses implied by the simulated ascent (calibrated).
ASCENT_LOSSES_MPS = 2600.0
# Booster recovery delta-v (boostback + entry + landing) the reference RTLS uses,
# measured down to booster_landing_reserve_kg.
RECOVERY_DV_REQUIRED_MPS = 3600.0
RECOVERY_DV_MARGIN_MPS = 50.0
MIN_LIFTOFF_TWR = 1.15
MARGIN_WARN_MPS = 300.0


@dataclass(frozen=True)
class Check:
    name: str
    value: float
    required: float
    unit: str
    status: str  # "ok" | "warn" | "fail"
    message: str


def size_recovery_reserve(config: SimulationConfig) -> float:
    """Stage-1 propellant to hold back at MECO for a full RTLS of this vehicle."""
    isp = C.ISP_VAC * float(config.runtime_isp_scale) * G0
    end = float(config.stage1_dry_mass) + float(config.booster_landing_reserve_kg)
    dv = RECOVERY_DV_REQUIRED_MPS + RECOVERY_DV_MARGIN_MPS
    return end * math.exp(dv / isp) - float(config.stage1_dry_mass)


def _status(value: float, required: float, warn_band: float) -> str:
    if value < required:
        return "fail"
    return "warn" if value < required + warn_band else "ok"


def plan_mission(config: SimulationConfig) -> dict:
    """Return delta-v / thrust budgets and a pass/warn/fail verdict."""
    isp_scale = float(config.runtime_isp_scale)
    s1_dry = float(config.stage1_dry_mass)
    s2_wet = float(config.stage2_dry_mass) + float(config.stage2_prop_mass)
    payload = float(config.payload_mass)
    m0 = s1_dry + float(config.stage1_prop_mass) + s2_wet + payload

    s1_isp = 0.5 * (C.ISP + C.ISP_VAC) * isp_scale
    dv_s1 = s1_isp * G0 * math.log(m0 / config.meco_mass_kg)
    s2_final = float(config.stage2_dry_mass) + payload + float(config.s2_landing_propellant_reserve_kg)
    dv_s2 = float(config.stage2_isp_vac) * isp_scale * G0 * math.log((s2_wet + payload) / s2_final)

    r_orbit = C.R_EARTH + float(config.orbit_target_altitude_m)
    v_circ = math.sqrt(C.MU_EARTH / r_orbit)
    v_rot = C.EARTH_ROTATION_RATE * C.R_EARTH * math.cos(math.radians(float(config.launch_site_lat_deg)))
    dv_required = v_circ + ASCENT_LOSSES_MPS - v_rot

    rec_start = s1_dry + float(config.stage1_landing_fuel_reserve_kg)
    rec_end = s1_dry + float(config.booster_landing_reserve_kg)
    dv_recovery = C.ISP_VAC * isp_scale * G0 * math.log(rec_start / max(rec_end, 1.0))

    twr = config.stage1_thrust_n * float(config.runtime_thrust_scale) / (m0 * G0)

    checks = [
        Check("orbit_delta_v", dv_s1 + dv_s2, dv_required, "m/s",
              _status(dv_s1 + dv_s2, dv_required, MARGIN_WARN_MPS),
              f"Stack delta-v {dv_s1 + dv_s2:.0f} m/s vs {dv_required:.0f} m/s to reach "
              f"{config.orbit_target_altitude_m / 1000:.0f} km"),
        Check("liftoff_twr", twr, MIN_LIFTOFF_TWR, "",
              _status(twr, MIN_LIFTOFF_TWR, 0.1),
              f"Liftoff thrust-to-weight {twr:.2f} (min {MIN_LIFTOFF_TWR})"),
        Check("recovery_delta_v", dv_recovery, RECOVERY_DV_REQUIRED_MPS, "m/s",
              _status(dv_recovery, RECOVERY_DV_REQUIRED_MPS, 0.0),
              f"Booster recovery delta-v {dv_recovery:.0f} m/s vs ~{RECOVERY_DV_REQUIRED_MPS:.0f} m/s for RTLS "
              f"(sized reserve {size_recovery_reserve(config) / 1000:.1f} t, "
              f"set {config.stage1_landing_fuel_reserve_kg / 1000:.1f} t)"),
    ]
    statuses = {c.status for c in checks}
    verdict = "fail" if "fail" in statuses else "warn" if "warn" in statuses else "ok"
    return {
        "verdict": verdict,
        "stage1_delta_v_mps": dv_s1,
        "stage2_delta_v_mps": dv_s2,
        "liftoff_mass_kg": m0,
        "sized_recovery_reserve_kg": size_recovery_reserve(config),
        "checks": [asdict(c) for c in checks],
    }

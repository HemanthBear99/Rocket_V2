"""API request models — single source for server mapping and OpenAPI schema."""

from __future__ import annotations

from typing import Literal

from pydantic import AliasChoices, BaseModel, ConfigDict, Field


class VehicleSetup(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)

    stage1_dry_mass: float = Field(gt=0)
    stage1_prop_mass: float = Field(gt=0)
    stage2_dry_mass: float = Field(gt=0)
    stage2_prop_mass: float = Field(gt=0)
    payload_mass: float = Field(ge=0)
    thrust_sl: float = Field(gt=0)
    thrust_vac: float = Field(gt=0)
    isp_sl: float = Field(gt=0)
    isp_vac: float = Field(gt=0)
    max_gimbal_deg: float = Field(ge=0, le=30)
    throttle_min: float = Field(gt=0, le=1)
    throttle_max: float = Field(gt=0, le=1)
    # Vehicle builder: engine cluster and geometry (defaults = reference vehicle).
    stage1_engines: int = Field(9, ge=1, le=50)
    boostback_engines: int = Field(3, ge=1, le=50)
    entry_engines: int = Field(3, ge=1, le=50)
    landing_engines: int = Field(1, ge=1, le=50)
    diameter_m: float = Field(3.7, gt=0, le=20)


class MissionSetup(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)

    target_alt_km: float = Field(gt=0)
    target_inclination_deg: float = Field(ge=0, le=180)
    orbit_insertion_start_alt_km: float = Field(
        gt=0,
        validation_alias=AliasChoices(
            "orbit_insertion_start_alt_km",
            "stage_sep_alt_km",
        ),
    )
    stage_sep_velocity: float = Field(gt=0)
    dt: float = Field(gt=0, le=5)
    t_max: float = Field(gt=0, le=7200)


class RecoverySetup(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)

    mode: Literal["RTLS"] = "RTLS"
    landing_lat: float = Field(ge=-90, le=90)
    landing_lon: float = Field(ge=-180, le=180)
    landing_burn_alt: Literal["auto"] = "auto"
    grid_fins: bool
    landing_legs: bool
    suicide_burn: Literal[True] = True
    # Landing-burn guidance: False = heuristic suicide burn, True = G-FOLD.
    gfold_guidance: bool = False


class PhysicsSetup(BaseModel):
    model_config = ConfigDict(extra="forbid")

    j2: bool
    atmosphere: bool
    drag: bool
    lift: bool
    sensor_noise: bool


class SimulationSetup(BaseModel):
    model_config = ConfigDict(extra="forbid")

    vehicle: VehicleSetup
    mission: MissionSetup
    recovery: RecoverySetup
    physics: PhysicsSetup
    realtime_mode: bool = False
    demo_mode: bool = False
    enable_s2_recovery: bool = False
    auto_recovery_reserve: bool = False
    # Escape hatch for SimulationConfig fields the curated form doesn't
    # expose (e.g. s2_landing_propellant_reserve_kg, orbit_altitude_tolerance_m).
    # Populated when a client uploads a full config JSON containing fields
    # outside the form schema, so they aren't silently dropped on launch.
    # Passed straight to create_default_config()/dataclasses.replace(), which
    # raises TypeError on an unknown field name -- surfaced as a 422 below.
    advanced_overrides: dict[str, float | int | bool | str] | None = None


class CampaignRequest(BaseModel):
    """Request body for POST /api/campaigns/start.

    Was previously defined inline in server.py, not here, despite this
    module's docstring claiming to be the "single source for server
    mapping and OpenAPI schema" -- any OpenAPI-schema-driven tooling that
    only inspected api_models.py would have missed the campaign contract.
    """

    model_config = ConfigDict(extra="forbid")

    mode: str = Field(default="monte_carlo", pattern="^(monte_carlo|sensitivity)$")
    runs: int = Field(default=100, gt=0, le=2000)
    seed: int = Field(default=42, ge=0)
    workers: int = Field(default=1, ge=1, le=16)
    setup: SimulationSetup | None = None


__all__ = [
    "CampaignRequest",
    "MissionSetup",
    "PhysicsSetup",
    "RecoverySetup",
    "SimulationSetup",
    "VehicleSetup",
]
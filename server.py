import asyncio
import hashlib
import hmac
import json
import logging
import os
import secrets
import time
import uuid
from collections import defaultdict, deque
from concurrent.futures import ProcessPoolExecutor, as_completed
from contextlib import asynccontextmanager
from dataclasses import asdict
from dataclasses import asdict as _dc_asdict
from pathlib import Path
from typing import Any

import numpy as np
import uvicorn
from fastapi import (
    Depends,
    FastAPI,
    File,
    Header,
    HTTPException,
    Request,
    UploadFile,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.security import APIKeyHeader
from fastapi.staticfiles import StaticFiles

from rlv_sim import campaign as campaign_mod
from rlv_sim import constants as C
from rlv_sim._run_full_mission import MissionProgress, run_full_mission
from rlv_sim.aero_validation import (
    build_geometry_deck_database as _build_geometry_deck_database,
)
from rlv_sim.aero_validation import (
    run_all_checks as run_aero_validation_checks,
)
from rlv_sim.aero_validation import (
    run_tier2_report as run_aero_tier2_report,
)
from rlv_sim.api_models import CampaignRequest, SimulationSetup
from rlv_sim.config_definition import SimulationConfig
from rlv_sim.config_factory import create_default_config, create_demo_config
from rlv_sim.mission_summary import assess_full_mission
from rlv_sim.telemetry import extract_mission_progress

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("rlv_server")

async def _periodic_cleanup_loop(interval_s: float = 60.0, retention_s: float = 600.0) -> None:
    """Periodically prune stale finished simulations from memory."""
    while True:
        try:
            await asyncio.sleep(interval_s)
            cutoff = time.monotonic() - retention_s
            stale = [
                sim_id
                for sim_id, meta in active_simulations.items()
                if meta.get("finished_at", float("inf")) < cutoff
            ]
            for sim_id in stale:
                active_simulations.pop(sim_id, None)
                logger.info(f"Pruned stale simulation from memory: {sim_id}")
            # Campaigns were never pruned, so every campaign retained up to
            # `runs` full result rows for the lifetime of the process. Reap
            # them on the same schedule. _run_campaign_task's finally block
            # always stamps finished_at, including on cancel/exception.
            stale_campaigns = [
                campaign_id
                for campaign_id, meta in active_campaigns.items()
                if meta.get("finished_at", float("inf")) < cutoff
            ]
            for campaign_id in stale_campaigns:
                active_campaigns.pop(campaign_id, None)
                logger.info(f"Pruned stale campaign from memory: {campaign_id}")
        except asyncio.CancelledError:
            break
        except Exception as e:  # noqa: BLE001 - keep the cleanup loop alive
            logger.error(f"Error in periodic cleanup loop: {e}")

class StartupConfigurationError(RuntimeError):
    """Raised to abort server startup on an unsafe configuration."""


@asynccontextmanager
async def lifespan(app: FastAPI):
    env = os.getenv("RLV_ENV", "").lower()
    if not _configured_api_keys() and env in ("production", "prod"):
        # Previously this only logged a warning and let the server start
        # anyway, meaning an unauthenticated production deployment was the
        # default unless an operator noticed the log line. Now refuses to
        # start unless the operator explicitly opts out via
        # RLV_ALLOW_NO_AUTH=1 (e.g. for a deliberately open internal demo).
        if os.getenv("RLV_ALLOW_NO_AUTH", "").lower() not in ("1", "true", "yes"):
            raise StartupConfigurationError(
                "RLV_API_KEYS is unset while RLV_ENV=production. Refusing to start "
                "unauthenticated in production. Set RLV_API_KEYS to one or more "
                "comma-separated keys, or set RLV_ALLOW_NO_AUTH=1 to explicitly "
                "run without authentication (not recommended)."
            )
        logger.warning(
            "RLV_API_KEYS is unset while RLV_ENV=production, but RLV_ALLOW_NO_AUTH=1 "
            "was set explicitly -- starting unauthenticated."
        )

    cleanup_task = asyncio.create_task(_periodic_cleanup_loop())
    yield

    cleanup_task.cancel()
    await asyncio.gather(cleanup_task, return_exceptions=True)

app = FastAPI(title="Boostback API", lifespan=lifespan)

_api_key_header = APIKeyHeader(name="X-RLV-API-Key", auto_error=False)
_start_attempts: dict[str, deque[float]] = defaultdict(deque)


def _configured_api_keys() -> tuple[str, ...]:
    return tuple(
        key.strip()
        for key in os.getenv("RLV_API_KEYS", "").split(",")
        if key.strip()
    )


def require_api_key(api_key: str | None = Depends(_api_key_header)) -> None:
    """Require an API key when ``RLV_API_KEYS`` is configured."""
    keys = _configured_api_keys()
    if not keys:
        return
    if api_key is None or not any(hmac.compare_digest(api_key, key) for key in keys):
        raise HTTPException(status_code=401, detail="A valid API key is required")


@app.middleware("http")
async def protect_and_measure_requests(request: Request, call_next):
    """Apply small public-API resource limits and emit request timing logs."""
    request_id = request.headers.get("x-request-id") or uuid.uuid4().hex
    started = time.monotonic()
    max_body_bytes = int(os.getenv("RLV_MAX_REQUEST_BYTES", "65536"))
    if request.url.path == "/api/config/validate":
        # Config uploads have their own, larger limit; allow multipart overhead.
        max_body_bytes = max(
            max_body_bytes,
            int(os.getenv("RLV_MAX_CONFIG_UPLOAD_BYTES", "262144")) + 16384,
        )
    content_length = request.headers.get("content-length")
    if content_length:
        try:
            request_bytes = int(content_length)
        except ValueError:
            return _json_error(400, "Invalid Content-Length header", request_id)
        if request_bytes > max_body_bytes:
            return _json_error(413, "Request body is too large", request_id)

    if request.method == "POST" and request.url.path == "/api/simulations/start":
        client = request.client.host if request.client else "unknown"
        supplied_key = request.headers.get("x-rlv-api-key")
        rate_identity = (
            "key:" + hashlib.sha256(supplied_key.encode("utf-8")).hexdigest()
            if supplied_key
            else "ip:" + client
        )
        now = time.monotonic()
        window_s = float(os.getenv("RLV_START_RATE_WINDOW_SECONDS", "60"))
        limit = int(os.getenv("RLV_START_RATE_LIMIT", "10"))
        attempts = _start_attempts[rate_identity]
        while attempts and attempts[0] <= now - window_s:
            attempts.popleft()
        if len(attempts) >= limit:
            return _json_error(429, "Simulation start rate limit exceeded", request_id)
        attempts.append(now)

    response = await call_next(request)
    response.headers["X-Request-ID"] = request_id
    logger.info(
        "request_id=%s method=%s path=%s status=%s duration_ms=%.1f",
        request_id,
        request.method,
        request.url.path,
        response.status_code,
        (time.monotonic() - started) * 1000.0,
    )
    return response


def _json_error(status_code: int, detail: str, request_id: str):
    from fastapi.responses import JSONResponse

    return JSONResponse(
        status_code=status_code,
        content={"detail": detail, "request_id": request_id},
        headers={"X-Request-ID": request_id},
    )


_cors_origins = [
    origin.strip()
    for origin in os.getenv(
        "RLV_CORS_ORIGINS",
        (
            "http://localhost:3000,"
            "http://localhost:8080,http://127.0.0.1:8080,"
            "http://localhost:4173,http://127.0.0.1:4173,"
            "http://localhost:5173,http://127.0.0.1:5173"
        ),
    ).split(",")
    if origin.strip()
]
app.add_middleware(
    CORSMiddleware,
    allow_origins=_cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

static_dir = Path(__file__).resolve().parent / "static"
if static_dir.exists():
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

    @app.get("/")
    async def serve_index():
        return FileResponse(static_dir / "index.html")


active_simulations: dict[str, dict] = {}


def _require_unchanged(
    section: str,
    values: dict[str, Any],
    expected: dict[str, Any],
) -> None:
    """Reject UI fields the current runtime cannot model instead of ignoring them."""
    unsupported = []
    for field, default in expected.items():
        if field not in values:
            continue
        supplied = values[field]
        if isinstance(default, (int, float)) and not isinstance(default, bool):
            try:
                changed = not np.isclose(float(supplied), float(default), rtol=0.0, atol=1e-9)
            except (TypeError, ValueError):
                changed = True
        else:
            changed = supplied != default
        if changed:
            unsupported.append(field)
    if unsupported:
        raise ValueError(
            f"Unsupported {section} override(s): {', '.join(sorted(unsupported))}. "
            "These controls are not part of the validated runtime model."
        )


def map_config(setup: SimulationSetup) -> SimulationConfig:
    """Map frontend configuration JSON structure to Python SimulationConfig."""
    overrides = {}


    m = setup.mission.model_dump()
    if "dt" in m:
        overrides["dt"] = float(m["dt"])
    if "t_max" in m:
        overrides["max_time"] = float(m["t_max"])
    if "target_alt_km" in m:
        overrides["orbit_target_altitude_m"] = float(m["target_alt_km"]) * 1000.0
    if "orbit_insertion_start_alt_km" in m:
        overrides["orbit_insertion_start_altitude_m"] = (
            float(m["orbit_insertion_start_alt_km"]) * 1000.0
        )
    if "target_inclination_deg" in m:
        overrides["target_inclination_deg"] = float(m["target_inclination_deg"])
    if "stage_sep_velocity" in m:
        overrides["stage_sep_velocity"] = float(m["stage_sep_velocity"])


    r = setup.recovery.model_dump()
    if "landing_lat" in r:
        overrides["booster_landing_site_lat_deg"] = float(r["landing_lat"])
    if "landing_lon" in r:
        overrides["booster_landing_site_lon_deg"] = float(r["landing_lon"])
    if "grid_fins" in r:
        overrides["enable_grid_fins"] = bool(r["grid_fins"])
    if "landing_legs" in r:
        overrides["enable_landing_legs"] = bool(r["landing_legs"])
    _require_unchanged(
        "recovery",
        r,
        {
            "mode": "RTLS",
            "landing_burn_alt": "auto",
            "suicide_burn": True,
        },
    )


    p = setup.physics.model_dump()
    if "j2" in p:
        overrides["enable_j2"] = bool(p["j2"])
    if "atmosphere" in p:
        overrides["enable_atmosphere"] = bool(p["atmosphere"])
    if "drag" in p:
        overrides["enable_drag"] = bool(p["drag"])
    if "lift" in p:
        overrides["enable_lift"] = bool(p["lift"])
    if "sensor_noise" in p:
        overrides["enable_imu"] = bool(p["sensor_noise"])
        overrides["enable_gps"] = bool(p["sensor_noise"])
        overrides["enable_landing_altimeter"] = bool(p["sensor_noise"])


    v = setup.vehicle.model_dump()
    default_thrust = C.THRUST_MAGNITUDE
    default_isp    = C.ISP
    if "thrust_sl" in v and float(v["thrust_sl"]) > 0:
        overrides["runtime_thrust_scale"] = float(v["thrust_sl"]) / default_thrust
    if "isp_sl" in v and float(v["isp_sl"]) > 0:
        overrides["runtime_isp_scale"] = float(v["isp_sl"]) / default_isp
    if "stage1_prop_mass" in v and float(v["stage1_prop_mass"]) > 0:
        overrides["stage1_prop_mass"] = float(v["stage1_prop_mass"])
    if "max_gimbal_deg" in v:
        overrides["max_gimbal_angle_deg"] = float(v["max_gimbal_deg"])
    if "throttle_min" in v:
        overrides["min_engine_throttle_fraction"] = float(v["throttle_min"])
    if "throttle_max" in v:
        overrides["max_engine_throttle_fraction"] = float(v["throttle_max"])
    if "stage1_dry_mass" in v:
        overrides["stage1_dry_mass"] = float(v["stage1_dry_mass"])
    if "stage2_dry_mass" in v:
        overrides["stage2_dry_mass"] = float(v["stage2_dry_mass"])
    if "stage2_prop_mass" in v:
        overrides["stage2_prop_mass"] = float(v["stage2_prop_mass"])
    if "payload_mass" in v:
        overrides["payload_mass"] = float(v["payload_mass"])
    if "thrust_vac" in v:
        overrides["stage2_thrust_vac"] = float(v["thrust_vac"])
    if "isp_vac" in v:
        overrides["stage2_isp_vac"] = float(v["isp_vac"])

    if setup.demo_mode:
        # Previously this unconditionally overwrote overrides["dt"] with
        # create_demo_config()'s dt (0.1), silently discarding whatever
        # dt the caller explicitly supplied in setup.mission.dt (set above
        # from `m["dt"]`) with no warning surfaced anywhere. Demo mode
        # should only toggle the demo coast-skipping behavior, not
        # override an explicitly supplied integration step.
        demo = create_demo_config()
        overrides["enable_demo_mode"] = True
        overrides["demo_coast_max_dt"] = demo.demo_coast_max_dt

    if setup.enable_s2_recovery:
        overrides["enable_s2_recovery"] = True
        overrides["max_time"] = max(
            float(overrides.get("max_time", C.MAX_TIME)),
            C.S2_RECOVERY_MIN_MAX_TIME_S,
        )
        # NOTE: a naive stage2_prop_mass increase + s2_landing_propellant_
        # reserve_kg floor was tried here and reverted -- it reproduced the
        # exact failure config_definition.py's s2_landing_propellant_reserve_kg
        # docstring already documents: this stage's sub-1.0 T/W means added
        # propellant mass increases gravity losses during insertion, which
        # eats into the added headroom, so orbit insertion itself aborted at
        # the reserve floor (reentered: perigee=-233.5km) instead of
        # reaching orbit. See that docstring before changing this again --
        # a real fix needs either a thrust increase or more efficient
        # insertion/deorbit guidance, not just added propellant mass.

    if setup.advanced_overrides:
        # advanced_overrides wins on key collision with the curated-form
        # overrides above -- it's the explicit, deliberate escape hatch, so
        # it should not be silently clobbered by a form default.
        overrides.update(setup.advanced_overrides)

    return create_default_config(**overrides)

async def _publish(sim_meta: dict, payload: dict) -> None:
    """Publish telemetry to all active client queues without allowing them to stall the simulation."""
    sim_meta["last_payload"] = payload
    queues = list(sim_meta.get("queues", []))
    for queue in queues:
        outgoing = payload
        if queue.full():
            try:
                queue.get_nowait()
                sim_meta["telemetry_dropped"] = (
                    int(sim_meta.get("telemetry_dropped", 0)) + 1
                )
                outgoing = {
                    **payload,
                    "telemetry_gap": True,
                    "telemetry_dropped": sim_meta["telemetry_dropped"],
                }
            except asyncio.QueueEmpty:
                pass
        await queue.put(outgoing)


def _finish_simulation(sim_meta: dict, status: str, **details: Any) -> None:
    sim_meta["status"] = status
    sim_meta["finished_at"] = time.monotonic()
    sim_meta["duration_s"] = (
        sim_meta["finished_at"] - sim_meta.get("started_at", sim_meta["finished_at"])
    )
    sim_meta.update(details)


def _prune_finished_simulations(retention_s: float = 3600.0) -> None:
    cutoff = time.monotonic() - retention_s
    stale = [
        sim_id
        for sim_id, meta in active_simulations.items()
        if meta.get("finished_at", float("inf")) < cutoff
    ]
    for sim_id in stale:
        active_simulations.pop(sim_id, None)


@app.get("/api/health")
async def health() -> dict[str, str]:
    return {"status": "ok"}


@app.get("/api/aero/validation", dependencies=[Depends(require_api_key)])
async def aero_validation() -> dict[str, Any]:
    """Tier 1 aerodynamic model validation report.

    Physical-sanity checks against classical aerodynamic theory (shape,
    symmetry, published slender-body bounds) -- NOT validation against
    wind-tunnel, CFD, or flight-measured data for this specific vehicle.
    See aero_validation.py's module docstring for what this does and does
    not prove.
    """
    return run_aero_validation_checks(deck=_build_geometry_deck_database())


@app.get("/api/aero/validation/tier2", dependencies=[Depends(require_api_key)])
async def aero_validation_tier2() -> dict[str, Any]:
    """Tier 2 aerodynamic validation report: comparison against published
    external aerodynamic literature (transonic drag-divergence behavior for
    slender bodies). See aero_validation.py's Tier 2 section docstring for
    what this does and does not prove, and each check's 'source' field for
    citations.
    """
    return run_aero_tier2_report()


@app.get("/api/config/defaults", dependencies=[Depends(require_api_key)])
async def frontend_config_defaults() -> dict[str, Any]:
    """Return UI defaults derived from the validated backend configuration."""
    config = create_default_config()
    return {
        "vehicle": {
            "stage1_dry_mass": config.stage1_dry_mass,
            "stage1_prop_mass": config.stage1_prop_mass,
            "stage2_dry_mass": config.stage2_dry_mass,
            "stage2_prop_mass": config.stage2_prop_mass,
            "payload_mass": config.payload_mass,
            "thrust_sl": C.THRUST_MAGNITUDE * config.runtime_thrust_scale,
            "thrust_vac": config.stage2_thrust_vac,
            "isp_sl": C.ISP * config.runtime_isp_scale,
            "isp_vac": config.stage2_isp_vac,
            "max_gimbal_deg": config.max_gimbal_angle_deg,
            "throttle_min": config.min_engine_throttle_fraction,
            "throttle_max": config.max_engine_throttle_fraction,
        },
        "mission": {
            "target_alt_km": config.orbit_target_altitude_m / 1000.0,
            "target_inclination_deg": config.target_inclination_deg,
            "orbit_insertion_start_alt_km": (
                config.orbit_insertion_start_altitude_m / 1000.0
            ),
            "stage_sep_velocity": config.stage_sep_velocity,
            "dt": config.dt,
            "t_max": config.max_time,
        },
        "recovery": {
            "mode": "RTLS",
            "landing_lat": (
                config.booster_landing_site_lat_deg
                if config.booster_landing_site_lat_deg is not None
                else config.launch_site_lat_deg
            ),
            "landing_lon": (
                config.booster_landing_site_lon_deg
                if config.booster_landing_site_lon_deg is not None
                else config.launch_site_lon_deg
            ),
            "landing_burn_alt": "auto",
            "grid_fins": config.enable_grid_fins,
            "landing_legs": config.enable_landing_legs,
            "suicide_burn": True,
        },
        "demo_mode": False,
        "enable_s2_recovery": bool(config.enable_s2_recovery),
        "physics": {
            "j2": config.enable_j2,
            "atmosphere": config.enable_atmosphere,
            "drag": config.enable_drag,
            "lift": config.enable_lift,
            "sensor_noise": (
                config.enable_imu
                or config.enable_gps
                or config.enable_landing_altimeter
            ),
        },
    }


@app.post("/api/config/validate", dependencies=[Depends(require_api_key)])
async def validate_config_upload(file: UploadFile = File(...)) -> dict[str, Any]:  # noqa: B008 - FastAPI idiom
    """Parse and validate an uploaded configuration JSON file without starting a run."""
    if file.filename and not file.filename.lower().endswith(".json"):
        raise HTTPException(status_code=422, detail="Configuration file must be JSON")
    max_bytes = int(os.getenv("RLV_MAX_CONFIG_UPLOAD_BYTES", "262144"))
    raw = await file.read(max_bytes + 1)
    if len(raw) > max_bytes:
        raise HTTPException(status_code=413, detail="Configuration file is too large")
    try:
        payload = json.loads(raw.decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("Configuration JSON must contain an object")  # noqa: TRY004 - mapped to 422 below
        config = create_default_config(**payload)
    except (json.JSONDecodeError, ValueError, TypeError) as exc:
        raise HTTPException(status_code=422, detail=f"Invalid configuration: {exc}") from exc
    return {"valid": True, "config": asdict(config)}


async def run_simulation_task(sim_id: str, sim_meta: dict) -> None:
    """Run the canonical mission orchestrator and stream progress to clients."""
    config = sim_meta["config"]
    event_loop = asyncio.get_running_loop()

    def control_signal() -> str:
        return str(sim_meta.get("control_signal", "run"))

    def publish_progress(progress: MissionProgress) -> None:
        """Schedule telemetry publish without blocking the physics thread."""
        payload = extract_mission_progress(progress, config)

        def _schedule_publish() -> None:
            task = asyncio.create_task(_publish(sim_meta, payload))

            def _log_publish_failure(done: asyncio.Task) -> None:
                try:
                    done.result()
                except Exception:
                    logger.exception("Telemetry publish failed for simulation %s", sim_id)

            task.add_done_callback(_log_publish_failure)

        event_loop.call_soon_threadsafe(_schedule_publish)

    logger.info("Simulation %s started", sim_id)

    try:
        result = await asyncio.to_thread(
            run_full_mission,
            config=config,
            verbose=False,
            progress_callback=publish_progress,
            control_callback=control_signal,
            realtime_factor=(
                2.0 if sim_meta.get("realtime_mode", False) else None
            ),
        )

        if control_signal() == "stop":
            _finish_simulation(sim_meta, "stopped")
            await _publish(sim_meta, {"event": "stopped", "status": "stopped"})
            logger.info("Simulation %s stopped", sim_id)
            return

        outcomes = {
            "orbiter_success": bool(result.orbiter_success),
            "booster_success": bool(result.booster_landing_success),
            "orbiter_reason": result.orbiter_reason,
            "booster_reason": result.booster_reason,
            "reason": (
                "Stage separation was not reached"
                if result.separation_time is None
                else f"Orbiter: {result.orbiter_reason}; Booster: {result.booster_reason}"
            ),
        }
        try:
            outcomes["assessment"] = _dc_asdict(assess_full_mission(result, config))
        except Exception:
            logger.exception("Mission assessment failed for simulation %s", sim_id)
            outcomes["assessment"] = None
        outcome_status = (
            "completed"
            if outcomes["orbiter_success"] and outcomes["booster_success"]
            else "failed"
        )
        _finish_simulation(
            sim_meta,
            outcome_status,
            completed=True,
            outcomes=outcomes,
        )
        await _publish(
            sim_meta,
            {
                "event": "completed",
                "status": outcome_status,
                "completed": True,
                "outcomes": outcomes,
            },
        )
        logger.info(
            "simulation_id=%s status=%s duration_s=%.3f orbiter_reason=%r booster_reason=%r",
            sim_id,
            outcome_status,
            sim_meta.get("duration_s", 0.0),
            result.orbiter_reason,
            result.booster_reason,
        )
    except Exception as exc:
        logger.exception("Simulation %s failed", sim_id)
        _finish_simulation(sim_meta, "error", error=str(exc))
        await _publish(sim_meta, {"error": str(exc)})


@app.post("/api/simulations/start", dependencies=[Depends(require_api_key)])
async def start_sim(setup: SimulationSetup):
    _prune_finished_simulations()


    active_runs = sum(
        1 for meta in active_simulations.values()
        if meta.get("status") in ("running", "paused")
    )
    if active_runs >= 5:
        raise HTTPException(
            status_code=429,
            detail="Maximum limit of concurrent simulations reached. Please stop active simulations or try again later.",
        )

    sim_id = f"sim_{uuid.uuid4().hex}"
    try:
        config = map_config(setup)
    except (ValueError, TypeError) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc

    # No placeholder queue here. A queue inserted into sim_meta["queues"] but
    # never drained made every _publish() get/put into a permanently-full
    # 100-slot queue for the whole run. WebSocket clients create and register
    # their own queue on connect (websocket_telemetry), so the set starts empty
    # and only real subscribers are ever written to.
    sim_meta = {
        "config": config,
        "status": "running",
        "control_signal": "run",
        "queues": set(),
        "clients": set(),
        "realtime_mode": setup.realtime_mode,
        "telemetry_dropped": 0,
        "access_token": secrets.token_urlsafe(32),
        "started_at": time.monotonic(),
    }
    active_simulations[sim_id] = sim_meta


    sim_meta["task"] = asyncio.create_task(run_simulation_task(sim_id, sim_meta))

    logger.info(f"Started simulation {sim_id}")
    return {
        "simulation_id": sim_id,
        "status": "running",
        "access_token": sim_meta["access_token"],
    }

def _require_simulation_token(sim_meta: dict, supplied: str | None) -> None:
    expected = sim_meta.get("access_token")
    if expected and (supplied is None or not hmac.compare_digest(supplied, expected)):
        raise HTTPException(status_code=403, detail="Invalid simulation access token")


@app.post("/api/simulations/{sim_id}/pause", dependencies=[Depends(require_api_key)])
async def pause_sim(
    sim_id: str,
    x_simulation_token: str | None = Header(default=None, alias="X-Simulation-Token"),
):
    if sim_id not in active_simulations:
        raise HTTPException(status_code=404, detail="Simulation not found")
    sim_meta = active_simulations[sim_id]
    _require_simulation_token(sim_meta, x_simulation_token)
    if sim_meta["status"] != "running":
        raise HTTPException(
            status_code=409,
            detail=f"Simulation is not running (status={sim_meta['status']})",
        )
    sim_meta["control_signal"] = "paused"
    sim_meta["status"] = "paused"
    return {"status": "paused"}

@app.post("/api/simulations/{sim_id}/resume", dependencies=[Depends(require_api_key)])
async def resume_sim(
    sim_id: str,
    x_simulation_token: str | None = Header(default=None, alias="X-Simulation-Token"),
):
    if sim_id not in active_simulations:
        raise HTTPException(status_code=404, detail="Simulation not found")
    sim_meta = active_simulations[sim_id]
    _require_simulation_token(sim_meta, x_simulation_token)
    if sim_meta["status"] != "paused":
        raise HTTPException(
            status_code=409,
            detail=f"Simulation is not paused (status={sim_meta['status']})",
        )
    sim_meta["control_signal"] = "run"
    sim_meta["status"] = "running"
    return {"status": "running"}

@app.post("/api/simulations/{sim_id}/stop", dependencies=[Depends(require_api_key)])
async def stop_sim(
    sim_id: str,
    x_simulation_token: str | None = Header(default=None, alias="X-Simulation-Token"),
):
    if sim_id not in active_simulations:
        raise HTTPException(status_code=404, detail="Simulation not found")
    sim_meta = active_simulations[sim_id]
    _require_simulation_token(sim_meta, x_simulation_token)
    if sim_meta["status"] not in ("running", "paused"):
        raise HTTPException(
            status_code=409,
            detail=f"Simulation cannot be stopped (status={sim_meta['status']})",
        )
    sim_meta["control_signal"] = "stop"
    # Previously this set status="stopped" here, before the background
    # physics thread had actually observed control_signal=="stop" and
    # halted -- a status/reality race where GET /api/simulations/{id}
    # (and by extension anything polling it) could report "stopped" while
    # the simulation loop was still running. The real terminal transition
    # happens in run_simulation_task's control_signal()=="stop" branch,
    # which calls _finish_simulation(sim_meta, "stopped") only once the
    # background thread has actually returned. "stopping" here reflects
    # that the halt has been requested but not yet confirmed; the
    # WebSocket stream's own {"event": "stopped"} message (or a follow-up
    # GET showing status=="stopped") remains the true completion signal.
    sim_meta["status"] = "stopping"
    return {"status": "stopping"}

@app.get("/api/simulations/{sim_id}", dependencies=[Depends(require_api_key)])
async def get_status(
    sim_id: str,
    x_simulation_token: str | None = Header(default=None, alias="X-Simulation-Token"),
):
    if sim_id not in active_simulations:
        raise HTTPException(status_code=404, detail="Simulation not found")
    meta = active_simulations[sim_id]
    _require_simulation_token(meta, x_simulation_token)
    response = {"simulation_id": sim_id, "status": meta["status"]}
    if "outcomes" in meta:
        response["outcomes"] = meta["outcomes"]
    if "error" in meta:
        response["error"] = meta["error"]
    if "duration_s" in meta:
        response["duration_s"] = meta["duration_s"]
    return response

async def _graceful_abort_check(sim_id: str):
    """Wait for 10 seconds, then abort the simulation task if no clients have reconnected."""
    await asyncio.sleep(10.0)
    if sim_id in active_simulations:
        sim_meta = active_simulations[sim_id]
        # "paused" must be included: a paused simulation blocks forever in
        # _run_full_mission's `while signal == "paused"` loop, so excluding it
        # meant a paused-then-abandoned run was never reaped -- leaking an
        # executor thread and a registry slot until the 5-run cap wedged
        # /api/simulations/start permanently.
        if not sim_meta.get("clients") and sim_meta.get("status") in ("running", "paused"):
            logger.info(f"Simulation {sim_id} orphaned. Aborting background task.")
            sim_meta["control_signal"] = "stop"
            # Use the single terminal-transition helper so finished_at is always
            # stamped. Setting status="stopped" directly (as this used to) left
            # finished_at unset, and both pruners test
            # `meta.get("finished_at", inf) < cutoff`, so `inf < cutoff` was
            # always False and the entry was never removed from
            # active_simulations -- an unbounded leak of config, queues and
            # task references.
            _finish_simulation(sim_meta, "stopped", reason="orphaned: no telemetry clients reconnected")
            task = sim_meta.get("task")
            if task and not task.done():
                # Cancels the coroutine awaiting asyncio.to_thread. The physics
                # thread itself keeps running until it observes control_signal,
                # so publish the terminal frame here rather than relying on
                # run_simulation_task (whose CancelledError is not an Exception
                # and therefore never reaches its own publish/finish path).
                await _publish(
                    sim_meta,
                    {"event": "stopped", "reason": "orphaned: no telemetry clients reconnected"},
                )
                task.cancel()

@app.websocket("/ws/simulations/{sim_id}/telemetry")
async def websocket_telemetry(websocket: WebSocket, sim_id: str):
    api_key = websocket.headers.get("x-rlv-api-key") or websocket.query_params.get("api_key")
    configured_keys = _configured_api_keys()
    if configured_keys and (
        api_key is None
        or not any(hmac.compare_digest(api_key, key) for key in configured_keys)
    ):
        await websocket.close(code=1008, reason="Invalid API key")
        return
    await websocket.accept()
    if sim_id not in active_simulations:
        await websocket.send_json({"error": "Simulation setup not found"})
        await websocket.close()
        return

    sim_meta = active_simulations[sim_id]
    supplied_token = websocket.query_params.get("access_token")
    try:
        _require_simulation_token(sim_meta, supplied_token)
    except HTTPException:
        await websocket.send_json({"error": "Invalid simulation access token"})
        await websocket.close(code=1008, reason="Invalid simulation access token")
        return
    max_clients = int(os.getenv("RLV_MAX_WS_CLIENTS_PER_SIM", "5"))
    if len(sim_meta.get("clients", ())) >= max_clients:
        await websocket.send_json({"error": "Telemetry client limit reached"})
        await websocket.close(code=1013, reason="Telemetry client limit reached")
        return


    queue = asyncio.Queue(maxsize=100)
    sim_meta["queues"].add(queue)
    if "last_payload" in sim_meta:
        queue.put_nowait(sim_meta["last_payload"])


    client_id = id(websocket)
    sim_meta["clients"].add(client_id)

    logger.info(f"Client connected for telemetry streaming: {sim_id} (active clients: {len(sim_meta['clients'])})")

    try:
        while True:

            telemetry = await queue.get()
            await websocket.send_json(telemetry)

            if "event" in telemetry and telemetry["event"] in ("completed", "stopped"):
                break
            if "error" in telemetry:
                break

    except WebSocketDisconnect:
        logger.info(f"Client disconnected from {sim_id}")
    except Exception:
        logger.exception("Error in websocket streaming")
    finally:

        sim_meta["queues"].discard(queue)
        sim_meta["clients"].discard(client_id)


        # Keep a strong reference to the reaper task. asyncio's event loop only
        # holds a weak reference to a task, so an un-referenced create_task can
        # be garbage-collected before it ever runs, silently skipping the abort.
        if not sim_meta["clients"] and sim_meta["status"] in ("running", "paused"):
            orphan_check = asyncio.create_task(_graceful_abort_check(sim_id))
            sim_meta["orphan_check_task"] = orphan_check


active_campaigns: dict[str, dict] = {}


def _run_campaign_task(campaign_id: str, meta: dict) -> None:
    """Run a campaign's cases sequentially or across worker processes.

    Previously this always ran a plain sequential for-loop and never read
    ``meta["workers"]``/``request.workers`` at all -- the field was accepted
    and validated (1-16) by CampaignRequest but had no effect, while the
    CLI's own campaign.run_campaign() genuinely parallelizes via
    ProcessPoolExecutor. This now does the same for server-triggered
    campaigns when workers > 1, while preserving the incremental
    completed-count polling and cooperative cancellation behavior that the
    previous sequential loop provided (campaign.run_campaign() itself has
    no progress/cancel hooks, so its ProcessPoolExecutor pattern is
    reproduced here rather than delegated to it).
    """
    try:
        base = meta["config"]
        if meta["mode"] == "sensitivity":
            cases = campaign_mod.build_sensitivity_cases()
        else:
            cases = campaign_mod.sample_monte_carlo_cases(meta["runs"], meta["seed"])
        meta["total"] = len(cases)
        meta["completed"] = 0
        meta["completed_success"] = 0
        workers = max(1, int(meta.get("workers", 1)))
        results_by_id: dict[str, dict[str, Any]] = {}

        def _record(case_id: str, result: dict[str, Any]) -> None:
            results_by_id[case_id] = result
            meta["completed"] = len(results_by_id)
            if result.get("mission_success"):
                meta["completed_success"] = meta.get("completed_success", 0) + 1

        if workers <= 1:
            for case in cases:
                if meta.get("cancel"):
                    break
                _record(case["case_id"], campaign_mod._run_case(base, case))
        else:
            with ProcessPoolExecutor(max_workers=workers) as executor:
                futures = {
                    executor.submit(campaign_mod._run_case, base, case): case["case_id"]
                    for case in cases
                }
                for future in as_completed(futures):
                    if meta.get("cancel"):
                        for pending in futures:
                            pending.cancel()
                        executor.shutdown(wait=False, cancel_futures=True)
                        break
                    _record(futures[future], future.result())

        results = [
            results_by_id[case["case_id"]]
            for case in cases
            if case["case_id"] in results_by_id
        ]
        summary = campaign_mod.summarize_results(results) if results else {}
        meta["status"] = "cancelled" if meta.get("cancel") else "completed"
        meta["results"] = results
        meta["summary"] = summary
    except Exception as exc:
        logger.exception("Campaign %s failed", campaign_id)
        meta["status"] = "error"
        meta["error"] = str(exc)
    finally:
        meta["finished_at"] = time.monotonic()


@app.post("/api/campaigns/start", dependencies=[Depends(require_api_key)])
async def start_campaign(request: CampaignRequest):
    active_runs = sum(
        1 for meta in active_campaigns.values() if meta.get("status") == "running"
    )
    if active_runs >= 2:
        raise HTTPException(
            status_code=429,
            detail="Maximum limit of concurrent campaigns reached.",
        )
    try:
        base = map_config(request.setup) if request.setup else create_default_config()
    except (ValueError, TypeError) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc

    campaign_id = f"camp_{uuid.uuid4().hex}"
    meta: dict[str, Any] = {
        "config": base,
        "mode": request.mode,
        "runs": request.runs,
        "seed": request.seed,
        "workers": request.workers,
        "status": "running",
        "completed": 0,
        "total": request.runs if request.mode == "monte_carlo" else 0,
        "started_at": time.monotonic(),
        "cancel": False,
    }
    active_campaigns[campaign_id] = meta
    meta["future"] = asyncio.get_running_loop().run_in_executor(
        None, _run_campaign_task, campaign_id, meta
    )
    return {"campaign_id": campaign_id, "status": "running"}


@app.get("/api/campaigns/{campaign_id}", dependencies=[Depends(require_api_key)])
async def get_campaign(campaign_id: str):
    if campaign_id not in active_campaigns:
        raise HTTPException(status_code=404, detail="Campaign not found")
    meta = active_campaigns[campaign_id]
    response: dict[str, Any] = {
        "campaign_id": campaign_id,
        "status": meta["status"],
        "completed": meta.get("completed", 0),
        "completed_success": meta.get("completed_success", 0),
        "total": meta.get("total", 0),
    }
    if meta["status"] in ("completed", "cancelled"):
        response["summary"] = meta.get("summary")
        response["results"] = meta.get("results")
    if "error" in meta:
        response["error"] = meta["error"]
    return response


@app.post("/api/campaigns/{campaign_id}/cancel", dependencies=[Depends(require_api_key)])
async def cancel_campaign(campaign_id: str):
    if campaign_id not in active_campaigns:
        raise HTTPException(status_code=404, detail="Campaign not found")
    active_campaigns[campaign_id]["cancel"] = True
    return {"status": "cancelling"}


def _resolve_host() -> str:
    """Resolve the bind host from RLV_HOST, defaulting to loopback-only.

    Previously defaulted to "0.0.0.0" (all interfaces) even for a plain
    `python -m rlv_sim.server` local run. Default is now loopback-only;
    set RLV_HOST=0.0.0.0 (or a specific interface) explicitly to expose
    the server beyond localhost.
    """
    host = os.getenv("RLV_HOST", "127.0.0.1")
    if host in ("0.0.0.0", "::"):
        logger.warning(
            "RLV_HOST=%s exposes the server on all network interfaces. "
            "Ensure RLV_API_KEYS is set if this is reachable outside a trusted network.",
            host,
        )
    return host


def main() -> None:
    host = _resolve_host()
    port = int(os.getenv("PORT", os.getenv("RLV_PORT", "8000")))
    uvicorn.run("rlv_sim.server:app", host=host, port=port)


if __name__ == "__main__":
    main()

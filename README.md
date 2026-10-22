# Vyom

A 6-DOF reusable launch vehicle mission simulator: stacked ascent, stage
separation, orbit insertion, and RTLS booster recovery (boostback burn,
entry burn, propulsive landing) — tracked as two independent vehicles in
lockstep after separation.

Physics: EGM96 spherical-harmonic gravity, US-76 atmosphere with optional
upper-atmosphere extension, Mach-dependent drag/lift, TVC-gimbaled thrust,
aerodynamic moments, engine spool dynamics, optional GPS/IMU sensor noise
for closed-loop guidance. See
[backend/rlv_sim/physics/forces.py](backend/rlv_sim/physics/forces.py) for
the force model and what in it is validated vs. an engineering
approximation — it's documented inline rather than left implicit.

## Repository layout

Each top-level folder is one shippable piece.

```
rlv_sim/
├── backend/            Python simulator + API server (pip-installable package)
│   ├── rlv_sim/
│   │   ├── core/        constants, state vector, frames, shared math
│   │   ├── physics/     dynamics, forces, mass, RK4 integrator, atmosphere
│   │   ├── aero/        aero decks (decks/), deck import, geometry, validation
│   │   ├── vehicle/     actuator, RCS, navigation sensors, attitude control, grid fins/legs
│   │   ├── guidance/    ascent, orbit insertion, booster recovery, G-FOLD, abort
│   │   ├── mission/     phase manager, step loop, full-mission runner, planner, Monte Carlo
│   │   ├── config/      SimulationConfig, factory, JSON loader
│   │   ├── outputs/     telemetry, mission summary, run manifest, plots/
│   │   ├── server/      FastAPI app, API models, storage, licensing, data paths
│   │   ├── validation/  flight-data comparison (Falcon 9 telemetry) + drag fit
│   │   └── cli.py       command-line entry point
│   ├── tests/          pytest suite (+ one node test for the UI CSV export)
│   ├── tools/          license-key issuing
│   └── pyproject.toml, uv.lock, requirements*.txt
├── frontend/           Web UI (static HTML/CSS/JS + Three.js), served by the backend
├── desktop/            Windows app: launcher, PyInstaller spec, Inno Setup installer, dist/
├── website/            Marketing site (Next.js, deployed separately)
└── docs/               Architecture map, validation reports, math references
```

## What to ship

| Client needs | Give them |
|---|---|
| Windows app, no Python | `desktop/dist/VyomSetup-<version>.exe` (or `Vyom.exe`) |
| Simulator as a Python library / CLI | `backend/` (`pip install ./backend`, then `rlv-sim --help`) |
| Web app on their own server | `backend/` + `frontend/` (backend serves the UI) |
| UI only, pointed at your server | `frontend/` (any static host; set the backend URL in the UI's Settings view, and add the UI's origin to `RLV_CORS_ORIGINS` on the server) |
| Marketing site | `website/` |

## Quick start

```bash
cd backend
uv sync --extra test
uv run pytest -q                                   # fast tests
uv run rlv-sim --mission full                      # one full mission, CLI
uv run uvicorn rlv_sim.server.app:app --port 8000  # web app at localhost:8000
```

Optional speed-up: `uv sync --extra fast` installs Numba, which compiles the
EGM96 gravity model and the booster impact predictor (~1.6x faster full
missions, identical outcomes). Without it the pure-Python paths are used.

### Desktop build (Windows)

```bash
cd backend && uv pip install -e ".[desktop]"
cd ../desktop
pyinstaller desktop_app.spec --noconfirm        # -> desktop/dist/Vyom.exe
ISCC installer\vyom.iss                         # -> desktop/dist/VyomSetup-<version>.exe
```

## Status

Research/educational tool, not flight-qualified software. Pass/fail
mission gates (orbit insertion accuracy, touchdown speed/site error,
dynamic-pressure limits) are enforced in
[backend/rlv_sim/outputs/summary.py](backend/rlv_sim/outputs/summary.py); see a run's
`mission_summary.md` for exactly what passed and why.

Flight-data comparisons live in `docs/VALIDATION.md` and
`docs/VALIDATION_DRAG.md`.

### Known limitation: Stage-2 recovery (`enable_s2_recovery`)

Booster (Stage-1) RTLS recovery is validated and reliable. Optional
second-stage recovery is experimental: with the default propellant/thrust
budget the vehicle is propellant-starved for the `S2_LANDING` burn, and
`S2_DEORBIT` can fail to converge before the mission time budget runs out.
See the `s2_landing_propellant_reserve_kg` docstring in
[backend/rlv_sim/config/definition.py](backend/rlv_sim/config/definition.py)
for the investigation. Do not enable this flag for a delivered/production
scenario until it's retuned.

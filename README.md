# Boostback

A 6-DOF reusable launch vehicle mission simulator: stacked ascent, stage
separation, orbit insertion, and RTLS booster recovery (boostback burn,
entry burn, propulsive landing) — tracked as two independent vehicles in
lockstep after separation.

Physics: EGM96 spherical-harmonic gravity, US-76 atmosphere with optional
upper-atmosphere extension, Mach-dependent drag/lift, TVC-gimbaled thrust,
aerodynamic moments, engine spool dynamics, optional GPS/IMU sensor noise
for closed-loop guidance. See [forces.py](forces.py) for the force model
and what in it is validated vs. an engineering approximation — it's
documented inline rather than left implicit.

## Interfaces

- **CLI** (`cli.py`) — batch/research runs with plots, telemetry CSVs, and
  a structured mission-summary report (JSON + Markdown).
- **Web app** (`server.py` + `static/`) — FastAPI backend, live telemetry
  over WebSocket, a 3D flight visualization, and Monte Carlo campaign runs.
- **Desktop app** (`desktop_app.py`) — the same web app packaged as a
  standalone Windows executable (PyInstaller), no separate server needed.

## Quick start

```bash
uv sync
uv run pytest tests/ -q                         # 200+ tests
uv run python -m rlv_sim.cli --mission full      # one full mission, CLI
uv run uvicorn rlv_sim.server:app --port 8000    # web app at localhost:8000
```

Optional speed-up: `uv sync --extra fast` installs Numba, which compiles the
EGM96 gravity model and the booster impact predictor (~1.6x faster full
missions, identical outcomes). Without it the pure-Python paths are used.

### Desktop build (Windows)

```bash
uv pip install -e ".[desktop]"
pyinstaller desktop_app.spec
dist/Boostback.exe
```

## Status

Research/educational tool, not flight-qualified software. Pass/fail
mission gates (orbit insertion accuracy, touchdown speed/site error,
dynamic-pressure limits) are enforced in
[mission_summary.py](mission_summary.py); see a run's
`mission_summary.md` for exactly what passed and why.

No published validation report against external reference data exists
yet — that's the next real step toward this being citable rather than
just internally self-consistent.

### Known limitation: Stage-2 recovery (`enable_s2_recovery`)

Booster (Stage-1) RTLS recovery is validated and reliable. Optional
second-stage recovery is experimental: with the default propellant/thrust
budget the vehicle is propellant-starved for the `S2_LANDING` burn, and
`S2_DEORBIT` can fail to converge before the mission time budget runs out.
See the `s2_landing_propellant_reserve_kg` docstring in
[config_definition.py](config_definition.py) for the investigation. Do not
enable this flag for a delivered/production scenario until it's retuned.

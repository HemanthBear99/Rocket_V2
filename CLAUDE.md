# Working agreement

- Be concise. Skip preamble, skip restating the task, skip trailing summaries unless asked.
- Route mechanical work (renames, reformatting, summarizing, scraping/extracting data) to a Haiku sub-agent instead of doing it inline.
- Never suggest `/compact` as a cost-saving measure — it's a context-management tool, not a cost lever to recommend proactively.

## Codebase Overview

Vyom (`rlv_sim`; formerly Boostback) is a Python 6-DOF two-stage reusable launch vehicle simulator: stacked ascent → separation → S2 orbit insertion (optional S2 recovery) + S1 booster boostback/entry/landing. Front ends: CLI (`rlv_sim/cli.py`), FastAPI web app (`rlv_sim/server/app.py` + `frontend/`), pywebview desktop app (`desktop/desktop_app.py`).

**Stack**: Python ≥3.10, numpy, matplotlib, FastAPI/uvicorn, pydantic v2, vanilla JS + Three.js, PyInstaller, uv, pytest/Playwright.
**Structure**: `backend/` (Python package `backend/rlv_sim/` with subpackages core, physics, aero, vehicle, guidance, mission, config, outputs, server, validation; plus `tests/`, `pyproject.toml`), `frontend/` (static web UI), `desktop/` (launcher, PyInstaller spec, installer, `dist/`), `website/` (Next.js). Package paths below are relative to `backend/`. Mission loop `rlv_sim/mission/full_mission.py` → `rlv_sim/mission/step.py` → guidance (`rlv_sim/guidance/`) / `rlv_sim/vehicle/control.py` → RK4 `rlv_sim/physics/integrators.py` → `rlv_sim/physics/dynamics.py`/`rlv_sim/physics/forces.py`. Config in `rlv_sim/config/definition.py` (frozen) + `rlv_sim/config/factory.py`. Outputs via `rlv_sim/outputs/summary.py`, `rlv_sim/outputs/plots/`, `rlv_sim/outputs/manifest.py`.
**Code navigation**: CodeGraph MCP (`codegraph_explore`) indexes this repo — use it before reading files.
**Verify changes** (venv is `backend/.venv`; run from `backend/`): `.venv/Scripts/python -m pytest -q` (fast), `pytest -m slow` (~35 s full-mission regression), `ruff check .` (clean; shared config in root `ruff.toml`). For refactors, compare full-mission telemetry CSVs before/after. Use `core.utils.cross3` / `core.utils.vec_norm` in hot paths, not `np.cross` / `np.linalg.norm`.

For detailed architecture, see [docs/CODEBASE_MAP.md](docs/CODEBASE_MAP.md).

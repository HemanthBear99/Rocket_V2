# Working agreement

- Be concise. Skip preamble, skip restating the task, skip trailing summaries unless asked.
- Route mechanical work (renames, reformatting, summarizing, scraping/extracting data) to a Haiku sub-agent instead of doing it inline.
- Never suggest `/compact` as a cost-saving measure — it's a context-management tool, not a cost lever to recommend proactively.

## Codebase Overview

Boostback (`rlv_sim`) is a Python 6-DOF two-stage reusable launch vehicle simulator: stacked ascent → separation → S2 orbit insertion (optional S2 recovery) + S1 booster boostback/entry/landing. Front ends: CLI (`cli.py`), FastAPI web app (`server.py` + `static/`), pywebview desktop app (`desktop_app.py`).

**Stack**: Python ≥3.10, numpy, matplotlib, FastAPI/uvicorn, pydantic v2, vanilla JS + Three.js, PyInstaller, uv, pytest/Playwright.
**Structure**: repo root is the `rlv_sim` package (parent dir must be on `sys.path`). Mission loop `_run_full_mission.py` → `_simulation_step.py` → guidance (`_guidance_*.py`) / `control.py` → RK4 `integrators.py` → `dynamics.py`/`forces.py`. Config in `config_definition.py` (frozen) + `config_factory.py`. Outputs via `mission_summary.py`, `_plotting_*.py`, `run_manifest.py`.
**Code navigation**: CodeGraph MCP (`codegraph_explore`) indexes this repo — use it before reading files.

For detailed architecture, see [docs/CODEBASE_MAP.md](docs/CODEBASE_MAP.md).

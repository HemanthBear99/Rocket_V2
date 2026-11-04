# AGENTS.md — handoff for Codex (and any coding agent)

Product: **Vyom** (formerly "Boostback"). Python package name stays `rlv_sim`.
Repo: github.com/HemanthBear99/Rocket_V2, branch `main`. Owner works on Windows (`D:\rlv_sim`).

## Working agreement (from the owner)
- Be concise. No preamble, no restating the task, no trailing summaries unless asked.
- Commit and push to `main` after each finished task.
- Keep token use low; don't re-derive things documented here.
- Never claim results you didn't verify; report failing tests honestly.

## What it is
6-DOF two-stage reusable launch vehicle simulator: stacked ascent → separation →
stage-2 orbit insertion (optional S2 recovery) + stage-1 booster flip/boostback/
coast/entry/landing (return to launch site, LZ-1).
Front ends: CLI (`rlv_sim/cli.py`), FastAPI + vanilla-JS web app (`rlv_sim/server/app.py`, `frontend/`),
pywebview desktop app (`desktop/desktop_app.py`, PyInstaller `desktop/desktop_app.spec`).
Layout: `backend/` (package `backend/rlv_sim/`, tests, pyproject), `frontend/`, `desktop/`, `website/`, `docs/`.
Package paths below are relative to `backend/`.
Marketing website: `website/` (Next.js 14 + React Three Fiber, separate app).

Architecture details: `docs/CODEBASE_MAP.md`. Short summary: `CLAUDE.md`.
Mission loop `rlv_sim/mission/full_mission.py` → `rlv_sim/mission/step.py` → guidance
(`rlv_sim/guidance/ascent.py`, `rlv_sim/guidance/booster.py`, `rlv_sim/guidance/orbit.py`, `rlv_sim/guidance/gfold.py`) /
`rlv_sim/vehicle/control.py` → RK4 `rlv_sim/physics/integrators.py` → `rlv_sim/physics/dynamics.py` / `rlv_sim/physics/forces.py`.
Config: `rlv_sim/config/definition.py` (frozen dataclass, validated) + `rlv_sim/config/factory.py`
(`create_default_config(**overrides)`; `RETIRED_FIELDS` keeps old saved configs loading).

## Environment (Windows quirks)
- Python venv: `D:\rlv_sim\backend\.venv` (package installed editable via `uv pip install -e ".[test]"`).
  Run Python commands from `backend/`, e.g. `cd /d/rlv_sim/backend && .venv/Scripts/python.exe -m pytest -q`.
- Git needs `git -c safe.directory=D:/rlv_sim ...` (dubious-ownership warning).
- GitHub Actions CI exists but the account is billing-locked; runs don't start. Ignore.
- Tests set `RLV_DATA_DIR` to a temp dir (backend/tests/conftest.py); never write to the real `%APPDATA%\Vyom` in tests.

## Verify every change
- Fast (from `backend/`): `.venv/Scripts/python.exe -m pytest -q -p no:cacheprovider` (~255 tests, ~30 s, includes a Playwright UI test).
- Slow: add `-m "slow or not slow"` (full suite ~8 min). `-m slow -k regression` = full-mission reference values.
- Lint: `.venv/Scripts/ruff check .` must be clean (from `backend/` or repo root; config in root `ruff.toml`).
- Hot paths: use `core.utils.cross3` / `core.utils.vec_norm`, not `np.cross` / `np.linalg.norm`.
- Guidance/physics changes: run a Monte Carlo before changing defaults:
  `.venv/Scripts/python.exe -m rlv_sim.mission.campaign --mode monte-carlo --runs 100 --seed 42 --workers 6 [--set field=value]`
  Current baseline (heuristic guidance, seed 42): **98% mission success** (90% lower bound 94.8%); see `docs/MONTE_CARLO_RESULTS.md`.
- Full mission ≈ 35 s.

## Build & release
- Desktop exe (from `desktop/`): `../backend/.venv/Scripts/python.exe -m PyInstaller desktop_app.spec --noconfirm -y` → `desktop/dist/Vyom.exe` (~32 MB without cvxpy; numba is excluded from the bundle).
  Needs `uv pip install -e ".[desktop]"` in `backend/`.
- Installer (from `desktop/`): `"%LOCALAPPDATA%\Programs\Inno Setup 6\ISCC.exe" installer\vyom.iss` → `desktop/dist/VyomSetup-0.3.0.exe`.
  Same AppId as the old Boostback installer, so it upgrades in place.
- Version lives in `backend/rlv_sim/__init__.py`, `backend/pyproject.toml`, `desktop/version_info.txt`, `desktop/installer/vyom.iss`, website copy.
- Licensing (`rlv_sim/server/licensing.py`): Ed25519-signed offline keys, 14-day trial, enforced only in the frozen exe
  (or `RLV_ENFORCE_LICENSE=1`). Issue keys (from `backend/`): `python -m tools.issue_license "Name" --plan pro --days 365`.
  **Private key is at `%USERPROFILE%\.boostback_signing\license_private.pem` — never commit or ship it.**
- User data: `%APPDATA%\Vyom` (scenarios, run history, imported aero decks, license); auto-migrated from `%APPDATA%\Boostback`.

## Website (`website/`)
- `cd website && npm install && npm run dev` (port 3000; `.claude/launch.json` uses 3005). `npm run build` must pass.
- Deploy on Vercel: Root Directory = `website`. Domain to be added later by the owner.
- TODO before launch: set `DOWNLOAD_URL` and replace `sales@example.com` in `website/lib/data.js`.
- 3D flight: `components/flight/*` (FlightScene = choreography/camera/bloom; Rocket, Earth, LaunchSite, particles).
  Scroll progress → pose/HUD in `lib/mission.js`. All displayed numbers in `lib/data.js` (real sim/flight data).
- Visual check without a GPU: headless Chromium with SwiftShader
  (`--use-gl=angle --use-angle=swiftshader --enable-unsafe-swiftshader`) via Playwright from the Python venv.

## Validation status (be honest in copy)
- `docs/VALIDATION.md` (`python -m rlv_sim.validation.flight_compare`): stage-1 cutoff speed within 1.4–3.9% of
  four Falcon 9 webcast flights; cutoff 6–10% early (large 67.5 t recovery reserve); peak q 9–29% high.
- `docs/VALIDATION_DRAG.md` (`python -m rlv_sim.validation.drag_fit`): booster descent drag ~1.5× (up to 2.2× near
  Mach 1) too high; fitted `booster_descent_drag_scale = 0.68` NOT default (MC drops 94% → 86%).
- No wind-tunnel/CFD data; aero is geometry/literature based. Product is "not for flight-critical use".

## Open work (owner's backlog, highest value first)
1. ~~Rebuild the exe and installer~~ — done 2026-10-23 (`Vyom.exe` + `VyomSetup-0.3.0.exe`, smoke-tested).
2. Make landing guidance robust with realistic descent drag (scale 0.68) so it can become default (target ≥94% MC; now 88%: tailwind cases arrive 100–380 m off target at 250 m).
3. ~~Adaptive recovery reserve~~ — done (wind-aware reserve, 383 kg per m/s headwind).
4. ~~Stage-2 sizing in the mission planner~~ — done (planner calculates required Stage-2 propellant and proposes fix).
5. Website: real app screenshots/recording section, Open Graph image, Vercel deploy.
6. Code-signing certificate (owner to buy) → sign exe/installer in the build.
7. Larger: entry/landing redesign (short hard entry burn) to close the Falcon 9 timing gap; more vehicle types
   (drone-ship landing, 3 stages, solids).

## Conventions
- Match surrounding style; comments explain *why*. Physics/guidance constants carry a note on how they were tuned.
- New config fields: add to `rlv_sim/config/definition.py` with a default that keeps current behaviour, validate them,
  add a help line to `rlv_sim/config/field_docs.py` (a test checks every field has one; it shows in the app's All
  Settings tab), and only change defaults after a Monte Carlo comparison.
- Research features: settings schema `config/schema.py` (`/api/config/schema`), guidance plugins `guidance/plugins.py`
  (`/api/plugins`, examples in `guidance/plugin_examples/`), model card `outputs/model_card.py` (`/api/model`; keep in
  step with `docs/MATHEMATICAL_REFERENCES.md`), flight comparison `validation/flight_compare.compare_flights`
  (`/api/validation/flights`), campaign cases `mission/campaign.build_cases` (CLI, server and single-case replay).
- Commit messages: imperative subject, body with what/why and measured results.

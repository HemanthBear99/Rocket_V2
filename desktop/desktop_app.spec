# PyInstaller build spec for the Vyom desktop app.
# Build with:  pyinstaller desktop_app.spec  (run from D:\rlv_sim\desktop)
# Output: desktop\dist\Vyom.exe

import sys
from pathlib import Path

from PyInstaller.utils.hooks import collect_data_files

# cvxpy (optional G-FOLD guidance) inspects its own package directory at
# import time, so its files must exist on disk inside the bundle, not only
# in the compressed module archive.
_optional_package_files = []
try:
    _optional_package_files += collect_data_files('cvxpy', include_py_files=True)
except Exception:
    pass

block_cipher = None
desktop_dir = Path(SPECPATH)
repo_root = desktop_dir.parent
backend_dir = repo_root / 'backend'

a = Analysis(
    ['desktop_app.py'],
    pathex=[str(desktop_dir), str(backend_dir)],
    binaries=[],
    datas=[
        (str(repo_root / 'frontend'), 'frontend'),
        (str(backend_dir / 'rlv_sim' / 'aero' / 'decks'), 'rlv_sim/aero/decks'),
    ] + _optional_package_files,
    hiddenimports=[
        'uvicorn.logging',
        'uvicorn.loops',
        'uvicorn.loops.auto',
        'uvicorn.protocols',
        'uvicorn.protocols.http',
        'uvicorn.protocols.http.auto',
        'uvicorn.protocols.websockets',
        'uvicorn.protocols.websockets.auto',
        'uvicorn.lifespan',
        'uvicorn.lifespan.on',
        'webview.platforms.winforms',
        'webview.platforms.edgechromium',
        'rlv_sim.server.app',
        'rlv_sim.mission.campaign',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    # numba (optional speed-up) would JIT-compile inside the frozen exe with
    # no on-disk cache; keep the bundle on the pure-Python paths.
    excludes=['matplotlib.tests', 'numpy.tests', 'numba', 'llvmlite'],
    noarchive=False,
    cipher=block_cipher,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='Vyom',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=str(repo_root / 'frontend' / 'app_icon.ico'),
    # Publisher/product identity for the exe's Properties dialog. This is
    # NOT a substitute for Authenticode code-signing -- Windows SmartScreen
    # still treats an unsigned exe as untrusted regardless of this metadata
    # -- but an exe with no version info at all is an extra red flag on top
    # of being unsigned, and a real Authenticode signature is a paid
    # certificate + business identity verification that has to be a
    # deliberate decision, not something this build can do on its own.
    version=str(desktop_dir / 'version_info.txt'),
)

# PyInstaller build spec for the Boostback desktop app.
# Build with:  pyinstaller desktop_app.spec  (run from D:\rlv_sim)

import sys
from pathlib import Path

block_cipher = None
project_root = Path(SPECPATH)

a = Analysis(
    ['desktop_app.py'],
    pathex=[str(project_root), str(project_root.parent)],
    binaries=[],
    datas=[
        (str(project_root / 'static'), 'rlv_sim/static'),
        (str(project_root / 'aero_decks'), 'rlv_sim/aero_decks'),
    ],
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
        'rlv_sim.server',
        'rlv_sim.campaign',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=['matplotlib.tests', 'numpy.tests'],
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
    name='Boostback',
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
    icon=str(project_root / 'static' / 'app_icon.ico'),
    # Publisher/product identity for the exe's Properties dialog. This is
    # NOT a substitute for Authenticode code-signing -- Windows SmartScreen
    # still treats an unsigned exe as untrusted regardless of this metadata
    # -- but an exe with no version info at all is an extra red flag on top
    # of being unsigned, and a real Authenticode signature is a paid
    # certificate + business identity verification that has to be a
    # deliberate decision, not something this build can do on its own.
    version=str(project_root / 'version_info.txt'),
)

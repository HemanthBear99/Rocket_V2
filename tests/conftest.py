"""Pytest bootstrap: make the ``rlv_sim`` package importable.

The repo root directory is itself named ``rlv_sim`` and its modules use
package-relative imports (``from . import constants``), so the package must
be imported as ``rlv_sim`` with the repo root's *parent* directory on
``sys.path``. The project's editable pip install does not currently resolve
this layout (see the "package import" note in the verified test-architecture
report), so tests add the parent directory directly rather than relying on
``pip install -e .``.
"""

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
_REPO_PARENT = _REPO_ROOT.parent

if str(_REPO_PARENT) not in sys.path:
    sys.path.insert(0, str(_REPO_PARENT))

# Saved scenarios, run history, licenses and imported aero decks live in a
# per-user data directory (app_paths); keep tests out of the real one.
import os
import tempfile

os.environ.setdefault("RLV_DATA_DIR", tempfile.mkdtemp(prefix="rlv_test_data_"))

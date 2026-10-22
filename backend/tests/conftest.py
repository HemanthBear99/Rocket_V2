"""Pytest bootstrap: make the ``rlv_sim`` package importable.

The package lives in ``backend/rlv_sim``. ``pip install -e backend`` makes it
importable everywhere; adding ``backend/`` to ``sys.path`` here keeps the
tests runnable without that install.
"""

import os
import sys
import tempfile
from pathlib import Path

_BACKEND_ROOT = Path(__file__).resolve().parent.parent

if str(_BACKEND_ROOT) not in sys.path:
    sys.path.insert(0, str(_BACKEND_ROOT))

# Saved scenarios, run history, licenses and imported aero decks live in a
# per-user data directory (app_paths); keep tests out of the real one.
os.environ.setdefault("RLV_DATA_DIR", tempfile.mkdtemp(prefix="rlv_test_data_"))

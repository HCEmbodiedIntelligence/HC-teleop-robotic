"""HC teleoperation middleware."""

import sys
from pathlib import Path

_UPLOAD_CLIENT = Path(__file__).resolve().parents[2] / "upload_client"
if _UPLOAD_CLIENT.is_dir() and str(_UPLOAD_CLIENT) not in sys.path:
    sys.path.insert(0, str(_UPLOAD_CLIENT))

__version__ = "0.1.0"

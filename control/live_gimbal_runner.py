from __future__ import annotations

import sys
from pathlib import Path

if __package__ in {None, ""}:
    ROOT_DIR = Path(__file__).resolve().parent.parent
    if str(ROOT_DIR) not in sys.path:
        sys.path.insert(0, str(ROOT_DIR))

from control.main import YoloUdpReceiver, load_telemetry_config, main, setup_logging


if __name__ == "__main__":
    raise SystemExit(main())

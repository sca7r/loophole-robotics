"""Logging configuration shared across CLI and library use."""
from __future__ import annotations

import logging
import sys


def configure_logging(level: str = "INFO") -> None:
    """Idempotent logger setup: structured single-line format, stderr stream."""
    root = logging.getLogger()
    # Don't double-attach handlers if called multiple times
    if any(getattr(h, "_loophole_arm_handler", False) for h in root.handlers):
        root.setLevel(level)
        return

    handler = logging.StreamHandler(stream=sys.stderr)
    handler.setFormatter(
        logging.Formatter(
            fmt="%(asctime)s %(levelname)s %(name)s: %(message)s",
            datefmt="%H:%M:%S",
        )
    )
    handler._loophole_arm_handler = True  # type: ignore[attr-defined]
    root.addHandler(handler)
    root.setLevel(level)

"""Entrypoint for ``python -m loophole_arm``."""
import sys

from loophole_arm.cli import main

if __name__ == "__main__":
    sys.exit(main())

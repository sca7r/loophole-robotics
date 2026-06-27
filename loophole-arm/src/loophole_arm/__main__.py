"""Entry point for ``python -m loophole_arm`` (sim CLI only)."""

import sys

from loophole_arm.sim_cli import main

if __name__ == "__main__":
    sys.exit(main())

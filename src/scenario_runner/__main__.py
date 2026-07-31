"""Allow ``python -m scenario_runner`` to drive the command line interface."""

from __future__ import annotations

import sys

from scenario_runner.cli import main

if __name__ == "__main__":
    sys.exit(main())

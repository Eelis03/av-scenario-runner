"""Compare a fresh run of the suite against the recorded baseline.

    uv run python examples/regression_check.py

The comparison separates an assertion that newly fails from a metric that moved
without changing a verdict. Running with ``--max-steps`` truncates every
scenario and therefore produces differences on purpose, which is a convenient
way to see the two categories side by side.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from scenario_runner.algorithm import Tolerance, compare_suites
from scenario_runner.analysis import load_suite_record, render_comparison
from scenario_runner.pipeline import run_suite

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIR = REPO_ROOT / "scenarios"
BASELINE_PATH = REPO_ROOT / "baselines" / "reference_suite.json"


def main(argv: list[str] | None = None) -> int:
    """Run the suite, compare it against the baseline, and print the differences."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-steps", type=int, default=None, help="cap the number of steps")
    parser.add_argument("--relative", type=float, default=1e-6, help="relative tolerance")
    arguments = parser.parse_args(argv)

    paths = sorted(SCENARIO_DIR.glob("*.toml"))
    current = run_suite(paths, max_steps=arguments.max_steps).to_record()
    baseline = load_suite_record(BASELINE_PATH)

    report = compare_suites(current, baseline, Tolerance(relative=arguments.relative))
    print(render_comparison(report))

    if arguments.max_steps is not None:
        print(
            "\nthe run was truncated with --max-steps, so these differences are expected; "
            "use `python -m scenario_runner compare` for the gating form"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())

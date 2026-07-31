"""Run the shipped scenario suite and write the text report and JSON result.

    uv run python examples/run_scenario_suite.py

Wiring only. Everything it calls lives in ``scenario_runner``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from scenario_runner.analysis import dump_suite_result, render_suite
from scenario_runner.pipeline import run_suite

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIR = REPO_ROOT / "scenarios"
OUTPUT_DIR = REPO_ROOT / "outputs"


def main(argv: list[str] | None = None) -> int:
    """Run every scenario, print the report, and store the artefacts."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-steps", type=int, default=None, help="cap the number of steps")
    arguments = parser.parse_args(argv)

    paths = sorted(SCENARIO_DIR.glob("*.toml"))
    suite = run_suite(paths, max_steps=arguments.max_steps)

    report = render_suite(suite)
    print(report)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUTPUT_DIR / "suite_report.txt").write_text(report + "\n", encoding="utf-8")
    dump_suite_result(suite, OUTPUT_DIR / "suite_result.json")
    print(f"\nwrote {OUTPUT_DIR / 'suite_report.txt'}")
    print(f"wrote {OUTPUT_DIR / 'suite_result.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

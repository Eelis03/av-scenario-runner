"""Render a figure for one scenario: paths, footprints, speed, and safety metrics.

uv run python examples/plot_scenario.py --scenario cut_in_moderate
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from scenario_runner.analysis import plot_run
from scenario_runner.model import load_scenario
from scenario_runner.pipeline import run_scenario

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIR = REPO_ROOT / "scenarios"
OUTPUT_DIR = REPO_ROOT / "outputs"


def main(argv: list[str] | None = None) -> int:
    """Simulate one scenario and write a three panel summary figure."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default="lead_vehicle_braking", help="scenario stem")
    parser.add_argument("--max-steps", type=int, default=None, help="cap the number of steps")
    arguments = parser.parse_args(argv)

    scenario = load_scenario(SCENARIO_DIR / f"{arguments.scenario}.toml")
    result, trace = run_scenario(scenario, max_steps=arguments.max_steps)

    target = plot_run(scenario, trace, OUTPUT_DIR / f"{scenario.name}.png", result.assertions)
    print(f"{scenario.name}: {result.outcome} after {trace.duration:.2f} s ({trace.steps} steps)")
    for item in result.assertions:
        moment = "n/a" if item.worst_time is None else f"{item.worst_time:.2f} s"
        print(
            f"  {item.outcome:<4} {item.name:<28} "
            f"worst {item.worst_value:10.3f} {item.unit:<6} at {moment}"
        )
    print(f"wrote {target}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

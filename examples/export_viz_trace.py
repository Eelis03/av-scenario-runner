"""Export a run as the JSON trace the browser playback in ``viz/`` reads.

    uv run python examples/export_viz_trace.py --scenario aggressive_cut_in

The Python package never imports ``viz/`` and does not need it to be built.
This script only writes a file.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from scenario_runner.analysis import dump_trace
from scenario_runner.model import load_scenario
from scenario_runner.pipeline import run_scenario

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIR = REPO_ROOT / "scenarios"
TRACE_DIR = REPO_ROOT / "viz" / "traces"
SCRATCH_DIR = REPO_ROOT / "outputs" / "traces"


def main(argv: list[str] | None = None) -> int:
    """Simulate one scenario and write its browser trace."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default="cut_in_moderate", help="scenario stem")
    parser.add_argument("--max-steps", type=int, default=None, help="cap the number of steps")
    arguments = parser.parse_args(argv)

    scenario = load_scenario(SCENARIO_DIR / f"{arguments.scenario}.toml")
    result, trace = run_scenario(scenario, max_steps=arguments.max_steps)

    # A truncated run is not the recording the browser page ships with, so it is
    # written somewhere else rather than overwriting a stored trace.
    directory = TRACE_DIR if arguments.max_steps is None else SCRATCH_DIR
    target = dump_trace(trace, directory / f"{scenario.name}.json", result.assertions)

    size_kb = target.stat().st_size / 1024.0
    print(f"{scenario.name}: {trace.steps} samples, {len(trace.actors)} actors, {result.outcome}")
    print(f"wrote {target} ({size_kb:.1f} kB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

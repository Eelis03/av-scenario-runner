"""Validate every scenario document and show what a rejection looks like.

    uv run python examples/validate_scenarios.py

Each shipped scenario is parsed, rendered back to TOML, and parsed again, which
checks that the format and the model agree. Then each malformed fixture is
parsed so the located error messages can be read.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from scenario_runner.model import ScenarioError, load_scenario, parse_scenario, to_toml

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIR = REPO_ROOT / "scenarios"
MALFORMED_DIR = REPO_ROOT / "tests" / "fixtures" / "malformed"


def main(argv: list[str] | None = None) -> int:
    """Round trip every scenario and print the rejection message for every fixture."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-steps", type=int, default=None, help="accepted and ignored")
    parser.parse_args(argv)

    print("valid scenarios")
    for path in sorted(SCENARIO_DIR.glob("*.toml")):
        scenario = load_scenario(path)
        restored = parse_scenario(to_toml(scenario), source=path.name)
        status = "round trip ok" if restored == scenario else "ROUND TRIP MISMATCH"
        print(
            f"  {path.name:<34} version {scenario.format_version}  "
            f"{len(scenario.actors)} actors  {len(scenario.assertions)} assertions  {status}"
        )

    print("\nmalformed scenarios, each rejected with its located message")
    for path in sorted(MALFORMED_DIR.glob("*.toml")):
        try:
            load_scenario(path)
        except ScenarioError as error:
            print(f"  {error}")
        else:
            print(f"  {path.name}: NOT REJECTED, which is itself a defect")
    return 0


if __name__ == "__main__":
    sys.exit(main())

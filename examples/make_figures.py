"""Regenerate the three figures embedded in the README.

    uv run python examples/make_figures.py

Every figure is produced from a real run of a shipped scenario, so a figure and
the numbers beside it in the README cannot drift apart without this script
saying so. The files are written to ``docs/figures/`` and are tracked, because a
reader of the repository page has no way to run anything.

Passing ``--max-steps`` truncates the runs, which makes the figures wrong. A
truncated run therefore writes to ``outputs/figures/`` instead, leaving the
published files untouched; the integration tier relies on that.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from scenario_runner.analysis import (
    BoundUse,
    bound_utilisation,
    plot_bound_utilisation,
    plot_encounter,
    plot_safety_timeline,
)
from scenario_runner.model import load_scenario
from scenario_runner.pipeline import run_scenario

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIR = REPO_ROOT / "scenarios"
FIGURE_DIR = REPO_ROOT / "docs" / "figures"
SCRATCH_DIR = REPO_ROOT / "outputs" / "figures"

#: Matches the budget the portfolio validator applies to ``docs/figures``.
BUDGET_BYTES = 250 * 1024

ENCOUNTER_SCENARIO = "cut_in_moderate"
TIMELINE_SCENARIO = "aggressive_cut_in"


def _utilisations(max_steps: int | None) -> list[BoundUse]:
    """Score every assertion in the suite as a fraction of its own bound."""
    entries: list[BoundUse] = []
    for path in sorted(SCENARIO_DIR.glob("*.toml")):
        scenario = load_scenario(path)
        result, _ = run_scenario(scenario, max_steps=max_steps)
        by_name = {item.name: item for item in result.assertions}
        for assertion in scenario.assertions:
            fraction = bound_utilisation(assertion, by_name[assertion.name])
            if fraction is None:
                continue
            entries.append(
                BoundUse(
                    scenario=scenario.name,
                    assertion=assertion.name,
                    fraction=fraction,
                    passed=by_name[assertion.name].passed,
                )
            )
    return entries


def main(argv: list[str] | None = None) -> int:
    """Write the three published figures and report their size against the budget."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-steps", type=int, default=None, help="cap the number of steps")
    arguments = parser.parse_args(argv)
    directory = FIGURE_DIR if arguments.max_steps is None else SCRATCH_DIR

    written: list[Path] = []

    scenario = load_scenario(SCENARIO_DIR / f"{ENCOUNTER_SCENARIO}.toml")
    _, trace = run_scenario(scenario, max_steps=arguments.max_steps)
    written.append(plot_encounter(scenario, trace, directory / "cut_in_encounter.png"))

    scenario = load_scenario(SCENARIO_DIR / f"{TIMELINE_SCENARIO}.toml")
    result, trace = run_scenario(scenario, max_steps=arguments.max_steps)
    written.append(
        plot_safety_timeline(
            scenario, trace, directory / "aggressive_cut_in_timeline.png", result.assertions
        )
    )

    entries = _utilisations(arguments.max_steps)
    written.append(plot_bound_utilisation(entries, directory / "bound_utilisation.png"))

    total = 0
    for target in written:
        size = target.stat().st_size
        total += size
        print(f"wrote {target.relative_to(REPO_ROOT).as_posix()} ({size / 1024.0:.1f} kB)")
    print(
        f"{len(written)} figures, {total / 1024.0:.1f} kB total, "
        f"budget {BUDGET_BYTES / 1024:.0f} kB"
    )
    print(f"scored {len(entries)} assertions against their bounds")
    if total > BUDGET_BYTES:
        print("over budget: reduce the figure size or the dots per inch")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

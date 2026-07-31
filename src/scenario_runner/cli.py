"""Command line entry point.

Two subcommands:

``run``
    Execute one or more scenario files, print a text report, optionally write a
    JSON result, and exit non-zero when the suite is not clean. ``--expect``
    switches the exit code from the literal rule (any failing assertion fails
    the suite) to the expectation rule (any scenario disagreeing with its own
    ``expected_outcome`` fails the suite), which is what lets a suite that
    deliberately contains failing scenarios still gate CI.

``compare``
    Compare a JSON result against a stored baseline and exit non-zero when an
    assertion newly fails or a scenario disappeared. A metric that moved
    without changing a verdict is reported but does not fail the comparison.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path

from scenario_runner.algorithm import Tolerance, compare_suites
from scenario_runner.analysis import (
    dump_suite_result,
    load_suite_record,
    render_comparison,
    render_suite,
)
from scenario_runner.pipeline import run_suite

__all__ = ["build_parser", "main"]


def build_parser() -> argparse.ArgumentParser:
    """Construct the argument parser for the whole command line interface."""
    parser = argparse.ArgumentParser(
        prog="scenario-runner",
        description="Run a declarative autonomous driving scenario suite.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    run = subparsers.add_parser("run", help="run scenario files and report the verdicts")
    run.add_argument("scenarios", nargs="+", type=Path, help="scenario TOML files")
    run.add_argument("--json", type=Path, default=None, help="write the JSON result here")
    run.add_argument("--report", type=Path, default=None, help="write the text report here")
    run.add_argument("--max-steps", type=int, default=None, help="cap the number of steps")
    run.add_argument("--seed", type=int, default=None, help="override the scenario seed")
    run.add_argument(
        "--expect",
        action="store_true",
        help="score against each scenario's declared expected_outcome",
    )
    run.add_argument("--quiet", action="store_true", help="suppress the text report on stdout")

    compare = subparsers.add_parser("compare", help="compare a JSON result against a baseline")
    compare.add_argument("current", type=Path, help="JSON result from this run")
    compare.add_argument("baseline", type=Path, help="stored baseline JSON result")
    compare.add_argument("--relative", type=float, default=1e-6, help="relative tolerance")
    compare.add_argument("--absolute", type=float, default=1e-9, help="absolute tolerance")

    return parser


def _run(arguments: argparse.Namespace) -> int:
    suite = run_suite(arguments.scenarios, seed=arguments.seed, max_steps=arguments.max_steps)
    text = render_suite(suite)
    if not arguments.quiet:
        print(text)
    if arguments.report is not None:
        report_path = Path(arguments.report)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(text + "\n", encoding="utf-8")
    if arguments.json is not None:
        dump_suite_result(suite, arguments.json)
    return suite.exit_code(check_expectations=bool(arguments.expect))


def _compare(arguments: argparse.Namespace) -> int:
    current = load_suite_record(arguments.current)
    baseline = load_suite_record(arguments.baseline)
    tolerance = Tolerance(relative=arguments.relative, absolute=arguments.absolute)
    report = compare_suites(current, baseline, tolerance)
    print(render_comparison(report))
    return report.exit_code()


def main(argv: Sequence[str] | None = None) -> int:
    """Parse ``argv`` and dispatch. Returns the process exit code."""
    arguments = build_parser().parse_args(argv)
    if arguments.command == "run":
        return _run(arguments)
    return _compare(arguments)

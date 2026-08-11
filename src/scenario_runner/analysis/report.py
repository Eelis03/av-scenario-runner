"""Text rendering of suite results and baseline comparisons.

The report prints the worst observed value and its time for every assertion,
passing ones included. A suite that only prints its failures hides the case
that matters most in practice: the scenario that still passes but whose margin
has collapsed since the last run.
"""

from __future__ import annotations

import math

from scenario_runner.algorithm import AssertionResult, ComparisonReport
from scenario_runner.pipeline import ScenarioResult, SuiteResult

__all__ = ["render_comparison", "render_scenario", "render_suite"]

_NAME_WIDTH = 30


def _value(value: float) -> str:
    if math.isnan(value):
        return "nan"
    if math.isinf(value):
        return "inf" if value > 0 else "-inf"
    return f"{value:.3f}"


def _time(value: float | None) -> str:
    return "  n/a  " if value is None else f"{value:6.2f}s"


def _assertion_line(item: AssertionResult) -> str:
    return (
        f"    {item.outcome.upper():<4} {item.name:<{_NAME_WIDTH}} "
        f"worst {_value(item.worst_value):>9} {item.unit:<6} "
        f"at {_time(item.worst_time)}  bound {item.bound}"
    )


def render_scenario(result: ScenarioResult) -> str:
    """Render one scenario result, including every assertion."""
    header = (
        f"{result.outcome.upper():<4} {result.name:<{_NAME_WIDTH}} "
        f"expect {result.expected_outcome:<4} "
        f"{len(result.assertions):>2} assertions  "
        f"{result.duration:6.2f} s  {result.steps:>4} steps  "
        f"ended on {result.terminated}"
    )
    lines = [header]
    if result.error is not None:
        lines.append(f"    LOAD ERROR {result.error}")
    lines.extend(_assertion_line(item) for item in result.assertions)
    if not result.matches_expectation:
        lines.append(
            f"    UNEXPECTED scenario declared expected_outcome = "
            f"{result.expected_outcome!r} but the run was {result.outcome!r}"
        )
    return "\n".join(lines)


def render_suite(suite: SuiteResult) -> str:
    """Render a whole suite, scenario by scenario, with a summary line."""
    lines = [
        f"scenario suite: {len(suite.scenarios)} scenarios, "
        f"{len(suite.passed)} passed, {len(suite.failed)} failed, "
        f"{len(suite.unexpected)} unexpected",
        "",
    ]
    for result in suite.scenarios:
        lines.append(render_scenario(result))
        lines.append("")
    failures = suite.failed
    if failures:
        lines.append("failing scenarios:")
        for result in failures:
            names = ", ".join(item.name for item in result.failed_assertions) or "load error"
            lines.append(f"  {result.name}: {names}")
        lines.append("")
    lines.append(
        f"literal exit code {suite.exit_code()}, "
        f"expectation exit code {suite.exit_code(check_expectations=True)}"
    )
    return "\n".join(lines)


def render_comparison(report: ComparisonReport) -> str:
    """Render a baseline comparison, separating verdict changes from metric drift."""
    if not report.changes:
        lines = ["comparison against baseline: no differences"]
    else:
        lines = [f"comparison against baseline: {len(report.changes)} difference(s)", ""]
        for change in report.changes:
            target = (
                change.scenario
                if not change.assertion
                else (f"{change.scenario}.{change.assertion}")
            )
            lines.append(f"  {change.kind.value:<18} {target:<48} {change.detail}")
        lines.append("")
    lines.append(
        f"new failures {len(report.new_failures)}, "
        f"renames {len(report.renames)}, "
        f"moved metrics {len(report.moved_metrics)}, "
        f"regressed {report.regressed}"
    )
    lines.append(
        f"tolerance: relative {report.tolerance.relative:g}, absolute {report.tolerance.absolute:g}"
    )
    return "\n".join(lines)

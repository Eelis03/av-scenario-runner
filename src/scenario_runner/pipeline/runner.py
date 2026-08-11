"""Execution of a scenario suite with a structured result per scenario.

A scenario that fails to load is a failure, not a skip. Treating a malformed
document as absent is the exact failure mode that lets a suite report green
while testing nothing, so a load error becomes a scenario result whose verdict
is a failure and whose message is the located parse error.

Exit codes have two modes:

* the literal mode returns non-zero when any scenario has a failing assertion;
* the expectation mode returns non-zero when any scenario disagrees with the
  ``expected_outcome`` it declares, which is how the shipped suite, containing
  scenarios that are supposed to fail, can still gate CI.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from scenario_runner.algorithm import (
    AssertionRecord,
    AssertionResult,
    ScenarioRecord,
    SuiteRecord,
    evaluate_assertions,
)
from scenario_runner.model import Scenario, ScenarioError, load_scenario
from scenario_runner.pipeline.simulator import simulate
from scenario_runner.pipeline.trace import Trace

__all__ = ["ScenarioResult", "SuiteResult", "run_scenario", "run_scenario_file", "run_suite"]


@dataclass(frozen=True, slots=True)
class ScenarioResult:
    """The outcome of one scenario, whether it ran or failed to load."""

    name: str
    source: str
    expected_outcome: str
    passed: bool
    assertions: tuple[AssertionResult, ...]
    steps: int
    duration: float
    terminated: str
    error: str | None = None

    @property
    def outcome(self) -> str:
        """``"pass"`` or ``"fail"``."""
        return "pass" if self.passed else "fail"

    @property
    def matches_expectation(self) -> bool:
        """True when the scenario behaved as its document says it should."""
        return self.outcome == self.expected_outcome

    @property
    def failed_assertions(self) -> tuple[AssertionResult, ...]:
        """Every assertion that did not hold."""
        return tuple(item for item in self.assertions if not item.passed)

    def to_record(self) -> ScenarioRecord:
        """Reduce to the comparable form used against a baseline."""
        return ScenarioRecord(
            name=self.name,
            passed=self.passed,
            assertions=tuple(
                AssertionRecord(
                    name=item.name,
                    kind=item.kind,
                    passed=item.passed,
                    worst_value=item.worst_value,
                    worst_time=item.worst_time,
                )
                for item in self.assertions
            ),
        )


@dataclass(frozen=True, slots=True)
class SuiteResult:
    """The outcome of a whole suite."""

    scenarios: tuple[ScenarioResult, ...]

    @property
    def passed(self) -> tuple[ScenarioResult, ...]:
        """Scenarios whose assertions all held."""
        return tuple(item for item in self.scenarios if item.passed)

    @property
    def failed(self) -> tuple[ScenarioResult, ...]:
        """Scenarios with at least one failing assertion or a load error."""
        return tuple(item for item in self.scenarios if not item.passed)

    @property
    def unexpected(self) -> tuple[ScenarioResult, ...]:
        """Scenarios that disagree with their declared expectation."""
        return tuple(item for item in self.scenarios if not item.matches_expectation)

    def exit_code(self, *, check_expectations: bool = False) -> int:
        """Zero when the suite is clean under the selected mode, one otherwise."""
        if check_expectations:
            return 1 if self.unexpected else 0
        return 1 if self.failed else 0

    def to_record(self) -> SuiteRecord:
        """Reduce to the comparable form used against a baseline."""
        return SuiteRecord(scenarios=tuple(item.to_record() for item in self.scenarios))


def run_scenario(
    scenario: Scenario, *, seed: int | None = None, max_steps: int | None = None
) -> tuple[ScenarioResult, Trace]:
    """Simulate ``scenario`` and score its assertions, returning the result and the trace."""
    trace = simulate(scenario, seed=seed, max_steps=max_steps)
    assertions = evaluate_assertions(scenario, trace)
    return (
        ScenarioResult(
            name=scenario.name,
            source=scenario.source,
            expected_outcome=scenario.expected_outcome,
            passed=all(item.passed for item in assertions),
            assertions=assertions,
            steps=trace.steps,
            duration=trace.duration,
            terminated=trace.terminated,
        ),
        trace,
    )


def run_scenario_file(
    path: Path | str, *, seed: int | None = None, max_steps: int | None = None
) -> ScenarioResult:
    """Load, run, and score the scenario stored at ``path``."""
    source = Path(path).name
    try:
        scenario = load_scenario(path)
    except ScenarioError as exc:
        return ScenarioResult(
            name=source,
            source=source,
            expected_outcome="pass",
            passed=False,
            assertions=(),
            steps=0,
            duration=0.0,
            terminated="load_error",
            error=str(exc),
        )
    result, _ = run_scenario(scenario, seed=seed, max_steps=max_steps)
    return result


def run_suite(
    paths: Sequence[Path | str], *, seed: int | None = None, max_steps: int | None = None
) -> SuiteResult:
    """Run every scenario in ``paths``, in order."""
    return SuiteResult(
        scenarios=tuple(run_scenario_file(path, seed=seed, max_steps=max_steps) for path in paths)
    )

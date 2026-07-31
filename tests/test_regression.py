"""Tier two: the shipped suite against its recorded baseline.

What a regression test may pin
------------------------------

The point of a regression test is to fail when behaviour changes and to pass
otherwise, on any machine that runs it. A baseline that pins a raw floating
point number taken from late in a simulated run does not have that property. It
fails when the platform library computes ``sin`` one unit in the last place
differently, when a reduction is ordered differently, or when a threshold
crossing lands one step earlier, and none of those is a change in the code
under test. A test that fails for reasons unrelated to the code under test
teaches its readers to ignore it, which is worse than having no test.

So this file pins, exactly:

* the verdict of every scenario;
* the verdict of every assertion, by name;
* the number of scenarios and the number of assertions in each;
* the sign and decade of every worst observed value.

and pins, with an explicit numeric tolerance:

* the worst observed value of assertions whose worst sample was taken in the
  first two seconds of the run, where the integration has not had time to
  amplify a difference in the last bit into a visible one.

The trailing part of a run is left to the class comparison on purpose. The
argument is set out in ``docs/design-notes.md``.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from scenario_runner.algorithm import ChangeKind, Tolerance, compare_suites, magnitude_class
from scenario_runner.analysis import load_suite_record, load_suite_result
from scenario_runner.pipeline import run_suite
from tests.conftest import BASELINE_PATH

#: Assertions whose worst sample lands no later than this are pinned numerically.
EARLY_WINDOW_SECONDS = 2.0

#: Tolerance for the numerically pinned values. Roughly nine significant digits,
#: which is far tighter than any behavioural change worth catching and far
#: looser than the last bit of a double.
EARLY_TOLERANCE = Tolerance(relative=1e-9, absolute=1e-9)


@pytest.fixture(scope="module")
def current_paths() -> list[Path]:
    """Every scenario the baseline was recorded from."""
    from tests.conftest import SCENARIO_DIR

    return sorted(SCENARIO_DIR.glob("*.toml"))


def test_baseline_file_exists() -> None:
    """The recorded baseline is committed, not generated on the fly by the test."""
    assert BASELINE_PATH.is_file(), f"missing recorded baseline at {BASELINE_PATH}"


def test_suite_verdicts_match_the_baseline_exactly(current_paths: list[Path]) -> None:
    """No assertion may change verdict, and no scenario or assertion may disappear."""
    current = run_suite(current_paths).to_record()
    baseline = load_suite_record(BASELINE_PATH)
    report = compare_suites(current, baseline, EARLY_TOLERANCE)
    breaking = [
        change
        for change in report.changes
        if change.kind
        in {
            ChangeKind.NEW_FAILURE,
            ChangeKind.RESOLVED_FAILURE,
            ChangeKind.SCENARIO_ADDED,
            ChangeKind.SCENARIO_REMOVED,
            ChangeKind.ASSERTION_ADDED,
            ChangeKind.ASSERTION_REMOVED,
        }
    ]
    assert not breaking, [
        f"{change.kind.value} {change.scenario}.{change.assertion}: {change.detail}"
        for change in breaking
    ]


def test_suite_shape_matches_the_baseline(current_paths: list[Path]) -> None:
    """Scenario count, assertion counts, and pass and fail counts are pinned."""
    current = run_suite(current_paths)
    baseline = load_suite_result(BASELINE_PATH)
    assert len(current.scenarios) == len(baseline.scenarios)
    assert len(current.passed) == len(baseline.passed)
    assert len(current.failed) == len(baseline.failed)
    assert not current.unexpected

    current_by_name = {item.name: item for item in current.scenarios}
    for reference in baseline.scenarios:
        observed = current_by_name[reference.name]
        assert len(observed.assertions) == len(reference.assertions), reference.name
        assert observed.terminated == reference.terminated, reference.name
        assert observed.steps == reference.steps, reference.name


def test_worst_value_magnitude_classes_match_the_baseline(current_paths: list[Path]) -> None:
    """The sign and decade of every worst value is pinned; the raw digits are not."""
    current = run_suite(current_paths)
    baseline = load_suite_result(BASELINE_PATH)
    reference = {
        (scenario.name, item.name): item
        for scenario in baseline.scenarios
        for item in scenario.assertions
    }
    checked = 0
    for scenario in current.scenarios:
        for item in scenario.assertions:
            expected = reference[(scenario.name, item.name)]
            assert magnitude_class(item.worst_value) == magnitude_class(expected.worst_value), (
                f"{scenario.name}.{item.name}: {item.worst_value} against {expected.worst_value}"
            )
            checked += 1
    assert checked >= 40


def test_margin_signs_match_the_baseline(current_paths: list[Path]) -> None:
    """Whether each assertion cleared its bound, and by which side, is pinned."""
    current = run_suite(current_paths)
    baseline = load_suite_result(BASELINE_PATH)
    reference = {
        (scenario.name, item.name): item
        for scenario in baseline.scenarios
        for item in scenario.assertions
    }
    for scenario in current.scenarios:
        for item in scenario.assertions:
            expected = reference[(scenario.name, item.name)]
            assert item.passed == expected.passed, f"{scenario.name}.{item.name}"
            assert item.bound == expected.bound, f"{scenario.name}.{item.name}"


def test_early_worst_values_match_the_baseline_within_tolerance(
    current_paths: list[Path],
) -> None:
    """Values observed in the first two seconds are pinned numerically."""
    current = run_suite(current_paths)
    baseline = load_suite_result(BASELINE_PATH)
    reference = {
        (scenario.name, item.name): item
        for scenario in baseline.scenarios
        for item in scenario.assertions
    }
    compared = 0
    for scenario in current.scenarios:
        for item in scenario.assertions:
            expected = reference[(scenario.name, item.name)]
            if expected.worst_time is None or expected.worst_time > EARLY_WINDOW_SECONDS:
                continue
            if not math.isfinite(expected.worst_value):
                continue
            assert EARLY_TOLERANCE.unchanged(item.worst_value, expected.worst_value), (
                f"{scenario.name}.{item.name}: {item.worst_value!r} "
                f"against {expected.worst_value!r}"
            )
            assert item.worst_time == pytest.approx(expected.worst_time, abs=1e-9)
            compared += 1
    assert compared >= 6, "the early window pinned too few values to be meaningful"


def test_rerunning_the_suite_reproduces_itself_exactly(current_paths: list[Path]) -> None:
    """Two runs in the same process agree bit for bit, so drift comes from elsewhere."""
    first = run_suite(current_paths).to_record()
    second = run_suite(current_paths).to_record()
    assert first == second

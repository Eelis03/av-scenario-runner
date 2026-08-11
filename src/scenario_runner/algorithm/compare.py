"""Comparison of a suite result against a stored baseline.

The comparison separates two questions that a single pass or fail count cannot
distinguish:

* did an assertion change verdict, which is a behavioural regression; and
* did a metric move while the verdict held, which is drift that may be benign
  or may be the last quiet step before a failure.

Both are reported. Only the first, together with a scenario or assertion that
disappeared, is treated as a regression for exit code purposes.

Names are the only handle a stored baseline has on a scenario, so renaming one
would otherwise read as a removal, which is a regression, plus an unrelated
addition. That is noise in the loudest possible place. A rename is therefore
detected the way a version control system detects one, by content rather than
by name: a scenario that left the suite and a scenario that joined it are paired
when every assertion they carry agrees in name, kind, verdict and worst observed
value, and when that pairing is unambiguous on both sides. The result is one
informational ``scenario_renamed`` line and no regression. The same rule applies
to a single assertion renamed inside a scenario.

``magnitude_class`` exists for the regression tests. Pinning a raw float from
late in a simulated run makes a test that fails on a different machine for
reasons that have nothing to do with the code under test, so the recorded
baseline pins the sign and the decade of the worst value instead. See
``docs/design-notes.md`` for the full argument.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from enum import StrEnum
from typing import Final

__all__ = [
    "AssertionRecord",
    "Change",
    "ChangeKind",
    "ComparisonReport",
    "ScenarioRecord",
    "SuiteRecord",
    "Tolerance",
    "compare_suites",
    "magnitude_class",
    "match_renames",
]


class ChangeKind(StrEnum):
    """The kinds of difference a comparison can report, ordered by severity."""

    NEW_FAILURE = "new_failure"
    SCENARIO_REMOVED = "scenario_removed"
    ASSERTION_REMOVED = "assertion_removed"
    RESOLVED_FAILURE = "resolved_failure"
    SCENARIO_ADDED = "scenario_added"
    ASSERTION_ADDED = "assertion_added"
    SCENARIO_RENAMED = "scenario_renamed"
    ASSERTION_RENAMED = "assertion_renamed"
    METRIC_MOVED = "metric_moved"


_SEVERITY: Final[dict[ChangeKind, int]] = {
    ChangeKind.NEW_FAILURE: 0,
    ChangeKind.SCENARIO_REMOVED: 1,
    ChangeKind.ASSERTION_REMOVED: 2,
    ChangeKind.RESOLVED_FAILURE: 3,
    ChangeKind.SCENARIO_ADDED: 4,
    ChangeKind.ASSERTION_ADDED: 5,
    ChangeKind.SCENARIO_RENAMED: 6,
    ChangeKind.ASSERTION_RENAMED: 7,
    ChangeKind.METRIC_MOVED: 8,
}

_REGRESSIONS: Final[frozenset[ChangeKind]] = frozenset(
    {ChangeKind.NEW_FAILURE, ChangeKind.SCENARIO_REMOVED, ChangeKind.ASSERTION_REMOVED}
)


@dataclass(frozen=True, slots=True)
class Tolerance:
    """Numeric tolerance used to decide whether a metric moved."""

    relative: float = 1e-6
    absolute: float = 1e-9

    def unchanged(self, first: float, second: float) -> bool:
        """True when the two values agree to within this tolerance."""
        if math.isnan(first) and math.isnan(second):
            return True
        if math.isinf(first) or math.isinf(second):
            return first == second
        return abs(first - second) <= self.absolute + self.relative * abs(second)


@dataclass(frozen=True, slots=True)
class AssertionRecord:
    """The comparable part of one assertion verdict."""

    name: str
    kind: str
    passed: bool
    worst_value: float
    worst_time: float | None


@dataclass(frozen=True, slots=True)
class ScenarioRecord:
    """The comparable part of one scenario result."""

    name: str
    passed: bool
    assertions: tuple[AssertionRecord, ...]


@dataclass(frozen=True, slots=True)
class SuiteRecord:
    """The comparable part of a whole suite result."""

    scenarios: tuple[ScenarioRecord, ...]

    def by_name(self) -> dict[str, ScenarioRecord]:
        """Index the scenarios by name."""
        return {scenario.name: scenario for scenario in self.scenarios}


@dataclass(frozen=True, slots=True)
class Change:
    """One difference between a current run and a baseline."""

    scenario: str
    assertion: str
    kind: ChangeKind
    baseline_value: float | None
    current_value: float | None
    detail: str

    @property
    def is_regression(self) -> bool:
        """True when this change should fail a regression gate."""
        return self.kind in _REGRESSIONS


@dataclass(frozen=True, slots=True)
class ComparisonReport:
    """Every difference between a current run and a baseline."""

    changes: tuple[Change, ...]
    tolerance: Tolerance

    @property
    def regressed(self) -> bool:
        """True when any change is a regression."""
        return any(change.is_regression for change in self.changes)

    @property
    def new_failures(self) -> tuple[Change, ...]:
        """Assertions that passed in the baseline and fail now."""
        return tuple(c for c in self.changes if c.kind is ChangeKind.NEW_FAILURE)

    @property
    def moved_metrics(self) -> tuple[Change, ...]:
        """Assertions whose verdict held but whose worst value moved."""
        return tuple(c for c in self.changes if c.kind is ChangeKind.METRIC_MOVED)

    @property
    def renames(self) -> tuple[Change, ...]:
        """Scenarios and assertions matched across a rename by their results."""
        return tuple(
            c
            for c in self.changes
            if c.kind in {ChangeKind.SCENARIO_RENAMED, ChangeKind.ASSERTION_RENAMED}
        )

    def exit_code(self) -> int:
        """Zero when nothing regressed, one otherwise."""
        return 1 if self.regressed else 0


def magnitude_class(value: float) -> str:
    """Classify ``value`` by sign and decade.

    The result is a short string such as ``"-1e0"`` for -3.2, ``"+1e1"`` for
    42.0, ``"zero"`` for 0.0, and ``"+inf"`` for a value that never closed. It
    is stable against the small numeric differences that appear between
    platforms, and it still changes when behaviour changes by an amount worth
    noticing. A value sitting within a rounding error of a decade boundary can
    still flip class, which is why the regression baseline is recorded from a
    run and inspected rather than generated blind.
    """
    if math.isnan(value):
        return "nan"
    if math.isinf(value):
        return "+inf" if value > 0 else "-inf"
    if value == 0.0:
        return "zero"
    sign = "+" if value > 0 else "-"
    return f"{sign}1e{math.floor(math.log10(abs(value)))}"


def match_renames[T](
    gone: Sequence[T], arrived: Sequence[T], same: Callable[[T, T], bool]
) -> list[tuple[T, T]]:
    """Pair the things that left with the things that arrived, when it is unambiguous.

    ``same`` decides whether two items carry identical results. A pair is
    accepted only when each side is the other's single candidate, so a
    simultaneous removal and addition of two items that both match is left
    reported as a removal and an addition rather than guessed at. The cost of
    the rule is stated where it bites: two items whose recorded results agree to
    the comparison tolerance are indistinguishable here, so a genuine removal
    accompanied by a genuine addition that happens to reproduce every worst
    value is reported as a rename.
    """
    matches: list[tuple[T, T]] = []
    for left in gone:
        candidates = [right for right in arrived if same(left, right)]
        if len(candidates) != 1:
            continue
        right = candidates[0]
        if sum(1 for other in gone if same(other, right)) != 1:
            continue
        matches.append((left, right))
    return matches


def _same_assertion(current: AssertionRecord, baseline: AssertionRecord, band: Tolerance) -> bool:
    return (
        current.kind == baseline.kind
        and current.passed == baseline.passed
        and band.unchanged(current.worst_value, baseline.worst_value)
    )


def _same_scenario(current: ScenarioRecord, baseline: ScenarioRecord, band: Tolerance) -> bool:
    if current.passed != baseline.passed:
        return False
    now = {item.name: item for item in current.assertions}
    before = {item.name: item for item in baseline.assertions}
    if set(now) != set(before):
        return False
    return all(_same_assertion(now[name], before[name], band) for name in now)


def compare_suites(
    current: SuiteRecord, baseline: SuiteRecord, tolerance: Tolerance | None = None
) -> ComparisonReport:
    """Report every difference between ``current`` and ``baseline``."""
    band = tolerance or Tolerance()
    changes: list[Change] = []
    current_by_name = current.by_name()
    baseline_by_name = baseline.by_name()

    gone = sorted(set(baseline_by_name) - set(current_by_name))
    arrived = sorted(set(current_by_name) - set(baseline_by_name))
    renamed = match_renames(
        gone,
        arrived,
        lambda old, new: _same_scenario(current_by_name[new], baseline_by_name[old], band),
    )
    for old, new in renamed:
        changes.append(
            Change(
                scenario=f"{old} -> {new}",
                assertion="",
                kind=ChangeKind.SCENARIO_RENAMED,
                baseline_value=None,
                current_value=None,
                detail=(
                    f"matched to the baseline scenario {old!r} by identical results, "
                    f"{len(baseline_by_name[old].assertions)} assertions unchanged"
                ),
            )
        )
    paired_old = {old for old, _ in renamed}
    paired_new = {new for _, new in renamed}

    for name in gone:
        if name in paired_old:
            continue
        changes.append(
            Change(
                scenario=name,
                assertion="",
                kind=ChangeKind.SCENARIO_REMOVED,
                baseline_value=None,
                current_value=None,
                detail="scenario is in the baseline but not in this run",
            )
        )
    for name in arrived:
        if name in paired_new:
            continue
        changes.append(
            Change(
                scenario=name,
                assertion="",
                kind=ChangeKind.SCENARIO_ADDED,
                baseline_value=None,
                current_value=None,
                detail="scenario is new since the baseline was recorded",
            )
        )

    for name in sorted(set(current_by_name) & set(baseline_by_name)):
        changes.extend(_compare_scenario(current_by_name[name], baseline_by_name[name], band))

    changes.sort(key=lambda change: (_SEVERITY[change.kind], change.scenario, change.assertion))
    return ComparisonReport(changes=tuple(changes), tolerance=band)


def _compare_scenario(
    current: ScenarioRecord, baseline: ScenarioRecord, tolerance: Tolerance
) -> list[Change]:
    changes: list[Change] = []
    current_by_name = {item.name: item for item in current.assertions}
    baseline_by_name = {item.name: item for item in baseline.assertions}

    gone = sorted(set(baseline_by_name) - set(current_by_name))
    arrived = sorted(set(current_by_name) - set(baseline_by_name))
    renamed = match_renames(
        gone,
        arrived,
        lambda old, new: _same_assertion(current_by_name[new], baseline_by_name[old], tolerance),
    )
    for old, new in renamed:
        changes.append(
            Change(
                scenario=current.name,
                assertion=f"{old} -> {new}",
                kind=ChangeKind.ASSERTION_RENAMED,
                baseline_value=baseline_by_name[old].worst_value,
                current_value=current_by_name[new].worst_value,
                detail=(
                    f"matched to the baseline assertion {old!r} by an unchanged "
                    f"{baseline_by_name[old].kind} verdict and worst value"
                ),
            )
        )
    paired_old = {old for old, _ in renamed}
    paired_new = {new for _, new in renamed}

    for name in gone:
        if name in paired_old:
            continue
        changes.append(
            Change(
                scenario=current.name,
                assertion=name,
                kind=ChangeKind.ASSERTION_REMOVED,
                baseline_value=baseline_by_name[name].worst_value,
                current_value=None,
                detail="assertion is in the baseline but not in this run",
            )
        )
    for name in arrived:
        if name in paired_new:
            continue
        changes.append(
            Change(
                scenario=current.name,
                assertion=name,
                kind=ChangeKind.ASSERTION_ADDED,
                baseline_value=None,
                current_value=current_by_name[name].worst_value,
                detail="assertion is new since the baseline was recorded",
            )
        )

    for name in sorted(set(current_by_name) & set(baseline_by_name)):
        now = current_by_name[name]
        before = baseline_by_name[name]
        if now.passed != before.passed:
            kind = ChangeKind.RESOLVED_FAILURE if now.passed else ChangeKind.NEW_FAILURE
            changes.append(
                Change(
                    scenario=current.name,
                    assertion=name,
                    kind=kind,
                    baseline_value=before.worst_value,
                    current_value=now.worst_value,
                    detail=(
                        f"verdict moved from {_verdict(before.passed)} to {_verdict(now.passed)}, "
                        f"worst value {before.worst_value:.6g} to {now.worst_value:.6g}"
                    ),
                )
            )
            continue
        if not tolerance.unchanged(now.worst_value, before.worst_value):
            changes.append(
                Change(
                    scenario=current.name,
                    assertion=name,
                    kind=ChangeKind.METRIC_MOVED,
                    baseline_value=before.worst_value,
                    current_value=now.worst_value,
                    detail=(
                        f"verdict held at {_verdict(now.passed)}, worst value "
                        f"{before.worst_value:.6g} to {now.worst_value:.6g} "
                        f"({_delta(before.worst_value, now.worst_value)})"
                    ),
                )
            )
    return changes


def _verdict(passed: bool) -> str:
    return "pass" if passed else "fail"


def _delta(before: float, after: float) -> str:
    if math.isinf(before) or math.isinf(after) or before == 0.0:
        return f"{after - before:+.6g} absolute"
    return f"{100.0 * (after - before) / abs(before):+.3f} percent"

"""Tier one: safety metrics, the assertion vocabulary, and baseline comparison.

Every assertion is scored against a trace whose correct verdict can be read off
by hand, and every assertion is scored a second time with the observed value
placed exactly on its threshold, because the boundary is where a comparison
operator that is off by one direction stops being detectable.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from scenario_runner.algorithm import (
    AssertionRecord,
    ChangeKind,
    RunMetrics,
    ScenarioRecord,
    SuiteRecord,
    Tolerance,
    clearance_series,
    compare_suites,
    evaluate_assertion,
    magnitude_class,
    time_headway_series,
    time_to_collision_series,
)
from scenario_runner.model import (
    DEFAULT_SHAPE,
    GoalReached,
    LateralAcceleration,
    LongitudinalAcceleration,
    MinDistance,
    MinTimeHeadway,
    MinTimeToCollision,
    NoCollision,
    SpeedLimit,
)
from tests.conftest import FakeBody, FakeTrace, build_trace, constant_body

STEPS = 21
DT = 0.1


def _metrics(
    clearance: list[float], ttc: list[float], headway: list[float] | None = None
) -> RunMetrics:
    return RunMetrics(
        clearance=np.asarray(clearance, dtype=np.float64),
        time_to_collision=np.asarray(ttc, dtype=np.float64),
        time_headway=np.asarray(
            _uniform(math.inf) if headway is None else headway, dtype=np.float64
        ),
    )


def _flat_trace(**series: list[float]) -> FakeTrace:
    ego = constant_body("ego", STEPS, x0=0.0, speed=10.0, dt=DT)
    arrays = {key: np.asarray(value, dtype=np.float64) for key, value in series.items()}
    return build_trace(
        ego,
        dt=DT,
        accel=arrays.get("accel"),
        lateral_accel=arrays.get("lateral_accel"),
    )


def _uniform(value: float) -> list[float]:
    return [value] * STEPS


# ---------------------------------------------------------------------------
# Metric geometry
# ---------------------------------------------------------------------------


def test_clearance_is_infinite_without_actors() -> None:
    """With nothing to hit, clearance is infinite and has no worst time."""
    trace = _flat_trace()
    assert np.all(np.isinf(clearance_series(trace)))
    assert np.all(np.isinf(time_to_collision_series(trace)))
    assert np.all(np.isinf(time_headway_series(trace)))


def test_clearance_matches_the_disc_cover_geometry() -> None:
    """Two vehicles nose to tail are separated by the analytic disc cover distance."""
    separation = 12.0
    ego = constant_body("ego", STEPS, x0=0.0, speed=0.0, dt=DT)
    lead = constant_body("lead", STEPS, x0=separation, speed=0.0, dt=DT)
    trace = build_trace(ego, (lead,), dt=DT)
    shape = DEFAULT_SHAPE
    expected = separation - 2.0 * shape.length / 3.0 - 2.0 * shape.disc_radius
    assert clearance_series(trace) == pytest.approx(expected, abs=1e-12)


def test_clearance_is_negative_when_footprints_overlap() -> None:
    """A vehicle placed inside another reports negative clearance."""
    ego = constant_body("ego", STEPS, x0=0.0, speed=0.0, dt=DT)
    lead = constant_body("lead", STEPS, x0=1.0, speed=0.0, dt=DT)
    trace = build_trace(ego, (lead,), dt=DT)
    assert float(np.min(clearance_series(trace))) < 0.0


def test_time_to_collision_matches_the_closing_geometry() -> None:
    """A vehicle closing on a stationary one collides after gap divided by speed."""
    separation = 50.0
    closing = 10.0
    ego = constant_body("ego", STEPS, x0=0.0, speed=closing, dt=DT)
    lead = constant_body("lead", STEPS, x0=separation, speed=0.0, dt=DT)
    trace = build_trace(ego, (lead,), dt=DT)
    shape = DEFAULT_SHAPE
    gap = separation - 2.0 * shape.length / 3.0 - 2.0 * shape.disc_radius
    assert float(time_to_collision_series(trace)[0]) == pytest.approx(gap / closing, rel=1e-12)


def test_time_to_collision_is_infinite_when_separating() -> None:
    """A vehicle pulling away is never a collision partner."""
    ego = constant_body("ego", STEPS, x0=0.0, speed=5.0, dt=DT)
    lead = constant_body("lead", STEPS, x0=50.0, speed=20.0, dt=DT)
    trace = build_trace(ego, (lead,), dt=DT)
    assert np.all(np.isinf(time_to_collision_series(trace)))


def test_time_to_collision_sees_a_lateral_encounter() -> None:
    """A vehicle in the neighbouring lane closing laterally is scored, not ignored."""
    steps = STEPS
    time = np.arange(steps, dtype=np.float64) * DT
    ego = constant_body("ego", steps, x0=0.0, speed=15.0, dt=DT)
    crossing = FakeBody(
        identifier="crossing",
        shape=DEFAULT_SHAPE,
        x=25.0 + 10.0 * time,
        y=3.5 - 0.7 * time,
        yaw=np.full(steps, math.atan2(-0.7, 10.0)),
        speed=np.full(steps, math.hypot(10.0, 0.7)),
    )
    trace = build_trace(ego, (crossing,), dt=DT)
    scored = time_to_collision_series(trace)
    assert np.all(np.isfinite(scored))
    assert float(scored[0]) < 5.0


def test_time_headway_is_the_clearance_covered_at_the_current_ego_speed() -> None:
    """Nose to tail, headway is the clearance divided by the speed that closes it."""
    separation = 40.0
    speed = 12.5
    ego = constant_body("ego", STEPS, x0=0.0, speed=speed, dt=DT)
    lead = constant_body("lead", STEPS, x0=separation, speed=speed, dt=DT)
    trace = build_trace(ego, (lead,), dt=DT)
    clearance = float(clearance_series(trace)[0])
    assert float(time_headway_series(trace)[0]) == pytest.approx(clearance / speed, rel=1e-12)


def test_time_headway_scores_a_follower_that_time_to_collision_cannot() -> None:
    """Two vehicles holding one speed never close, so only headway sees the short gap."""
    ego = constant_body("ego", STEPS, x0=0.0, speed=13.0, dt=DT)
    lead = constant_body("lead", STEPS, x0=11.0, speed=13.0, dt=DT)
    trace = build_trace(ego, (lead,), dt=DT)
    assert np.all(np.isinf(time_to_collision_series(trace)))
    assert float(time_headway_series(trace)[0]) < 0.5


def test_time_headway_ignores_a_vehicle_behind_the_ego() -> None:
    """Headway is a forward query: a follower is scored by clearance instead."""
    ego = constant_body("ego", STEPS, x0=50.0, speed=13.0, dt=DT)
    follower = constant_body("follower", STEPS, x0=40.0, speed=13.0, dt=DT)
    trace = build_trace(ego, (follower,), dt=DT)
    assert np.all(np.isfinite(clearance_series(trace)))
    assert np.all(np.isinf(time_headway_series(trace)))


def test_time_headway_ignores_a_vehicle_a_lane_over() -> None:
    """A vehicle offset by a lane width sits outside the corridor the ego sweeps."""
    ego = constant_body("ego", STEPS, x0=0.0, speed=13.0, dt=DT)
    neighbour = constant_body("neighbour", STEPS, x0=25.0, speed=13.0, y=3.5, dt=DT)
    trace = build_trace(ego, (neighbour,), dt=DT)
    assert np.all(np.isinf(time_headway_series(trace)))


def test_time_headway_is_infinite_at_standstill() -> None:
    """A stopped ego covers no ground, so it has no headway to the vehicle ahead."""
    ego = constant_body("ego", STEPS, x0=0.0, speed=0.0, dt=DT)
    lead = constant_body("lead", STEPS, x0=12.0, speed=0.0, dt=DT)
    trace = build_trace(ego, (lead,), dt=DT)
    assert np.all(np.isinf(time_headway_series(trace)))


# ---------------------------------------------------------------------------
# Assertion verdicts, one hand checked case and one boundary case each
# ---------------------------------------------------------------------------


def test_no_collision_passes_with_positive_clearance() -> None:
    """Clearance that never reaches zero is not a collision."""
    trace = _flat_trace()
    metrics = _metrics(_uniform(4.0), _uniform(math.inf))
    result = evaluate_assertion(NoCollision(), trace, metrics)
    assert result.passed
    assert result.worst_value == 4.0


def test_no_collision_fails_at_exactly_zero_clearance() -> None:
    """Zero clearance is contact, so the one strict comparison in the vocabulary fails."""
    trace = _flat_trace()
    clearance = _uniform(4.0)
    clearance[7] = 0.0
    result = evaluate_assertion(NoCollision(), trace, _metrics(clearance, _uniform(math.inf)))
    assert not result.passed
    assert result.worst_value == 0.0
    assert result.worst_time == pytest.approx(0.7)


def test_min_time_to_collision_passes_exactly_at_the_threshold() -> None:
    """A lower bound is inclusive: equality passes."""
    trace = _flat_trace()
    ttc = _uniform(9.0)
    ttc[4] = 1.5
    result = evaluate_assertion(
        MinTimeToCollision(threshold=1.5), trace, _metrics(_uniform(3.0), ttc)
    )
    assert result.passed
    assert result.worst_value == 1.5
    assert result.worst_time == pytest.approx(0.4)


def test_min_time_to_collision_fails_just_below_the_threshold() -> None:
    """One step below the boundary flips the verdict and reports where."""
    trace = _flat_trace()
    ttc = _uniform(9.0)
    ttc[4] = 1.4999
    result = evaluate_assertion(
        MinTimeToCollision(threshold=1.5), trace, _metrics(_uniform(3.0), ttc)
    )
    assert not result.passed
    assert result.worst_value == pytest.approx(1.4999)


def test_min_time_headway_passes_exactly_at_the_threshold() -> None:
    """The lower bound on headway is inclusive, like every other lower bound."""
    trace = _flat_trace()
    headway = _uniform(3.0)
    headway[6] = 1.0
    result = evaluate_assertion(
        MinTimeHeadway(threshold=1.0), trace, _metrics(_uniform(3.0), _uniform(math.inf), headway)
    )
    assert result.passed
    assert result.worst_value == 1.0
    assert result.worst_time == pytest.approx(0.6)


def test_min_time_headway_fails_just_below_the_threshold() -> None:
    """A run whose time to collision never closes still fails on a short headway."""
    trace = _flat_trace()
    headway = _uniform(3.0)
    headway[6] = 0.9999
    result = evaluate_assertion(
        MinTimeHeadway(threshold=1.0), trace, _metrics(_uniform(3.0), _uniform(math.inf), headway)
    )
    assert not result.passed
    assert result.worst_value == pytest.approx(0.9999)


def test_min_distance_passes_exactly_at_the_threshold() -> None:
    """A lower bound on clearance is inclusive."""
    trace = _flat_trace()
    clearance = _uniform(5.0)
    clearance[11] = 2.0
    result = evaluate_assertion(
        MinDistance(threshold=2.0), trace, _metrics(clearance, _uniform(math.inf))
    )
    assert result.passed
    assert result.worst_time == pytest.approx(1.1)


def test_min_distance_fails_below_the_threshold() -> None:
    """Clearance under the threshold fails and reports the worst value."""
    trace = _flat_trace()
    clearance = _uniform(5.0)
    clearance[11] = 1.5
    result = evaluate_assertion(
        MinDistance(threshold=2.0), trace, _metrics(clearance, _uniform(math.inf))
    )
    assert not result.passed
    assert result.worst_value == pytest.approx(1.5)


def test_longitudinal_acceleration_passes_on_both_boundaries() -> None:
    """A value sitting on each end of the comfort band is inside it."""
    accel = _uniform(0.0)
    accel[3] = -3.0
    accel[9] = 2.0
    trace = _flat_trace(accel=accel)
    result = evaluate_assertion(
        LongitudinalAcceleration(minimum=-3.0, maximum=2.0),
        trace,
        _metrics(_uniform(9.0), _uniform(math.inf)),
    )
    assert result.passed
    assert result.worst_value == pytest.approx(-3.0)
    assert result.worst_time == pytest.approx(0.3)


def test_longitudinal_acceleration_fails_outside_the_band() -> None:
    """The reported worst value is the one of greatest magnitude, sign kept."""
    accel = _uniform(0.0)
    accel[5] = -4.2
    accel[6] = 1.0
    trace = _flat_trace(accel=accel)
    result = evaluate_assertion(
        LongitudinalAcceleration(minimum=-3.0, maximum=2.0),
        trace,
        _metrics(_uniform(9.0), _uniform(math.inf)),
    )
    assert not result.passed
    assert result.worst_value == pytest.approx(-4.2)
    assert result.worst_time == pytest.approx(0.5)


def test_lateral_acceleration_passes_exactly_at_the_limit() -> None:
    """The lateral bound is on magnitude and is inclusive."""
    lateral = _uniform(0.0)
    lateral[2] = -2.5
    trace = _flat_trace(lateral_accel=lateral)
    result = evaluate_assertion(
        LateralAcceleration(limit=2.5), trace, _metrics(_uniform(9.0), _uniform(math.inf))
    )
    assert result.passed
    assert result.worst_value == pytest.approx(-2.5)


def test_lateral_acceleration_fails_above_the_limit() -> None:
    """Exceeding the magnitude bound in either direction fails."""
    lateral = _uniform(0.0)
    lateral[2] = 2.6
    trace = _flat_trace(lateral_accel=lateral)
    result = evaluate_assertion(
        LateralAcceleration(limit=2.5), trace, _metrics(_uniform(9.0), _uniform(math.inf))
    )
    assert not result.passed


def test_speed_limit_passes_exactly_at_the_limit() -> None:
    """The upper bound on speed is inclusive, and tolerance widens it."""
    ego = constant_body("ego", STEPS, x0=0.0, speed=13.9, dt=DT)
    trace = build_trace(ego, dt=DT)
    result = evaluate_assertion(
        SpeedLimit(limit=13.9, tolerance=0.0), trace, _metrics(_uniform(9.0), _uniform(math.inf))
    )
    assert result.passed
    assert result.worst_value == pytest.approx(13.9)


def test_speed_limit_fails_above_the_limit() -> None:
    """A speed above the limit plus tolerance fails."""
    ego = constant_body("ego", STEPS, x0=0.0, speed=14.2, dt=DT)
    trace = build_trace(ego, dt=DT)
    result = evaluate_assertion(
        SpeedLimit(limit=13.9, tolerance=0.1), trace, _metrics(_uniform(9.0), _uniform(math.inf))
    )
    assert not result.passed


def test_goal_reached_passes_exactly_on_the_time_budget() -> None:
    """Arriving at the last permitted instant is arriving in time."""
    ego = constant_body("ego", STEPS, x0=0.0, speed=10.0, dt=DT)
    trace = build_trace(ego, dt=DT)
    # Ego arc length is 0, 1, ... 20 m; 10 m is reached at t = 1.0 s.
    result = evaluate_assertion(
        GoalReached(goal_s=10.0, time_budget=1.0),
        trace,
        _metrics(_uniform(9.0), _uniform(math.inf)),
    )
    assert result.passed
    assert result.worst_value == pytest.approx(1.0)


def test_goal_reached_fails_after_the_time_budget() -> None:
    """Arriving one step late fails, and the reported value is the arrival time."""
    ego = constant_body("ego", STEPS, x0=0.0, speed=10.0, dt=DT)
    trace = build_trace(ego, dt=DT)
    result = evaluate_assertion(
        GoalReached(goal_s=10.0, time_budget=0.9),
        trace,
        _metrics(_uniform(9.0), _uniform(math.inf)),
    )
    assert not result.passed
    assert result.worst_value == pytest.approx(1.0)


def test_goal_reached_reports_the_shortfall_when_never_reached() -> None:
    """A goal beyond the run reports the end of the run and how far short it fell."""
    ego = constant_body("ego", STEPS, x0=0.0, speed=10.0, dt=DT)
    trace = build_trace(ego, dt=DT)
    result = evaluate_assertion(
        GoalReached(goal_s=500.0, time_budget=5.0),
        trace,
        _metrics(_uniform(9.0), _uniform(math.inf)),
    )
    assert not result.passed
    assert "not reached" in result.detail
    assert result.worst_value == pytest.approx(2.0)


def test_every_result_reports_a_worst_value_and_a_bound() -> None:
    """No assertion may return a verdict without the evidence behind it."""
    trace = _flat_trace()
    metrics = _metrics(_uniform(3.0), _uniform(4.0), _uniform(2.0))
    assertions = (
        NoCollision(),
        MinTimeToCollision(threshold=1.5),
        LongitudinalAcceleration(minimum=-3.0, maximum=2.0),
        LateralAcceleration(limit=2.0),
        SpeedLimit(limit=13.9),
        GoalReached(goal_s=5.0, time_budget=3.0),
        MinDistance(threshold=1.0),
        MinTimeHeadway(threshold=1.0),
    )
    for assertion in assertions:
        result = evaluate_assertion(assertion, trace, metrics)
        assert result.bound
        assert result.detail
        assert result.unit
        assert math.isfinite(result.worst_value)


# ---------------------------------------------------------------------------
# Magnitude classes and baseline comparison
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (0.0, "zero"),
        (3.2, "+1e0"),
        (-3.2, "-1e0"),
        (42.0, "+1e1"),
        (-0.084, "-1e-2"),
        (math.inf, "+inf"),
        (-math.inf, "-inf"),
        (math.nan, "nan"),
    ],
)
def test_magnitude_class(value: float, expected: str) -> None:
    """Sign and decade classification is stable and readable."""
    assert magnitude_class(value) == expected


def _record(name: str, passed: bool, value: float, *, assertion: str = "min_ttc") -> ScenarioRecord:
    return ScenarioRecord(
        name=name,
        passed=passed,
        assertions=(
            AssertionRecord(
                name=assertion,
                kind="min_time_to_collision",
                passed=passed,
                worst_value=value,
                worst_time=1.0,
            ),
        ),
    )


def test_comparison_detects_a_newly_failing_assertion() -> None:
    """An assertion that passed in the baseline and fails now is a regression."""
    baseline = SuiteRecord(scenarios=(_record("s", True, 2.0),))
    current = SuiteRecord(scenarios=(_record("s", False, 1.1),))
    report = compare_suites(current, baseline)
    assert report.regressed
    assert report.exit_code() == 1
    assert [change.kind for change in report.changes] == [ChangeKind.NEW_FAILURE]
    assert not report.moved_metrics


def test_comparison_detects_a_metric_that_moved_without_failing() -> None:
    """Drift inside a held verdict is reported separately and is not a regression."""
    baseline = SuiteRecord(scenarios=(_record("s", True, 2.0),))
    current = SuiteRecord(scenarios=(_record("s", True, 2.4),))
    report = compare_suites(current, baseline)
    assert not report.regressed
    assert report.exit_code() == 0
    assert [change.kind for change in report.changes] == [ChangeKind.METRIC_MOVED]
    assert not report.new_failures
    assert "20.000 percent" in report.moved_metrics[0].detail


def test_comparison_ignores_movement_inside_the_tolerance() -> None:
    """A difference under tolerance is not a difference."""
    baseline = SuiteRecord(scenarios=(_record("s", True, 2.0),))
    current = SuiteRecord(scenarios=(_record("s", True, 2.0 + 1e-12),))
    report = compare_suites(current, baseline, Tolerance(relative=1e-6))
    assert not report.changes


def test_comparison_reports_a_resolved_failure_separately() -> None:
    """An assertion that started passing is a change, but not a regression."""
    baseline = SuiteRecord(scenarios=(_record("s", False, 1.0),))
    current = SuiteRecord(scenarios=(_record("s", True, 2.0),))
    report = compare_suites(current, baseline)
    assert not report.regressed
    assert [change.kind for change in report.changes] == [ChangeKind.RESOLVED_FAILURE]


def test_comparison_reports_added_and_removed_scenarios() -> None:
    """A scenario that disappeared is a regression; a new and different one is not."""
    baseline = SuiteRecord(scenarios=(_record("gone", True, 2.0), _record("kept", True, 2.0)))
    current = SuiteRecord(scenarios=(_record("kept", True, 2.0), _record("fresh", True, 9.5)))
    report = compare_suites(current, baseline)
    kinds = {change.kind for change in report.changes}
    assert kinds == {ChangeKind.SCENARIO_REMOVED, ChangeKind.SCENARIO_ADDED}
    assert report.regressed


def test_comparison_reports_added_and_removed_assertions() -> None:
    """Deleting an assertion is a regression even when everything still passes."""
    baseline = SuiteRecord(scenarios=(_record("s", True, 2.0, assertion="a"),))
    current = SuiteRecord(scenarios=(_record("s", True, 7.25, assertion="b"),))
    report = compare_suites(current, baseline)
    kinds = {change.kind for change in report.changes}
    assert kinds == {ChangeKind.ASSERTION_REMOVED, ChangeKind.ASSERTION_ADDED}
    assert report.regressed


def test_a_renamed_scenario_is_matched_by_its_results() -> None:
    """Renaming a scenario is one informational change, not a removal and an addition."""
    baseline = SuiteRecord(scenarios=(_record("merge_from_ramp", True, 8.011),))
    current = SuiteRecord(scenarios=(_record("ramp_merge", True, 8.011),))
    report = compare_suites(current, baseline)
    assert [change.kind for change in report.changes] == [ChangeKind.SCENARIO_RENAMED]
    assert not report.regressed
    assert report.exit_code() == 0
    change = report.renames[0]
    assert change.scenario == "merge_from_ramp -> ramp_merge"
    assert "merge_from_ramp" in change.detail


def test_a_renamed_assertion_is_matched_by_its_results() -> None:
    """The same rule applies one level down, to an assertion inside a kept scenario."""
    baseline = SuiteRecord(scenarios=(_record("s", True, 2.0, assertion="min_ttc"),))
    current = SuiteRecord(scenarios=(_record("s", True, 2.0, assertion="time_to_collision"),))
    report = compare_suites(current, baseline)
    assert [change.kind for change in report.changes] == [ChangeKind.ASSERTION_RENAMED]
    assert not report.regressed
    assert report.renames[0].assertion == "min_ttc -> time_to_collision"


def test_a_rename_that_also_changed_the_result_is_not_matched() -> None:
    """A rename is only a rename when nothing else moved; otherwise it stays loud."""
    baseline = SuiteRecord(scenarios=(_record("old", True, 2.0),))
    current = SuiteRecord(scenarios=(_record("new", True, 3.4),))
    report = compare_suites(current, baseline)
    kinds = {change.kind for change in report.changes}
    assert kinds == {ChangeKind.SCENARIO_REMOVED, ChangeKind.SCENARIO_ADDED}
    assert report.regressed


def test_a_rename_that_flipped_a_verdict_is_not_matched() -> None:
    """Identical worst values are not enough; the verdict has to match as well."""
    baseline = SuiteRecord(scenarios=(_record("old", True, 2.0),))
    current = SuiteRecord(scenarios=(_record("new", False, 2.0),))
    report = compare_suites(current, baseline)
    assert {change.kind for change in report.changes} == {
        ChangeKind.SCENARIO_REMOVED,
        ChangeKind.SCENARIO_ADDED,
    }


def test_an_ambiguous_rename_is_not_guessed() -> None:
    """Two candidates on each side leave the removal and the addition reported as they are."""
    baseline = SuiteRecord(scenarios=(_record("a", True, 2.0), _record("b", True, 2.0)))
    current = SuiteRecord(scenarios=(_record("c", True, 2.0), _record("d", True, 2.0)))
    report = compare_suites(current, baseline)
    kinds = sorted(change.kind for change in report.changes)
    assert kinds.count(ChangeKind.SCENARIO_REMOVED) == 2
    assert kinds.count(ChangeKind.SCENARIO_ADDED) == 2
    assert not report.renames
    assert report.regressed


def test_a_rename_is_matched_inside_the_comparison_tolerance() -> None:
    """A rename recorded on another machine still matches within the stated tolerance."""
    baseline = SuiteRecord(scenarios=(_record("old", True, 2.0),))
    current = SuiteRecord(scenarios=(_record("new", True, 2.0 + 1e-12),))
    report = compare_suites(current, baseline, Tolerance(relative=1e-6))
    assert [change.kind for change in report.changes] == [ChangeKind.SCENARIO_RENAMED]


def test_a_rename_alongside_an_unrelated_removal_keeps_both_readable() -> None:
    """One rename and one genuine removal are reported as one of each."""
    baseline = SuiteRecord(scenarios=(_record("old", True, 2.0), _record("dropped", True, 5.0)))
    current = SuiteRecord(scenarios=(_record("new", True, 2.0),))
    report = compare_suites(current, baseline)
    kinds = [change.kind for change in report.changes]
    assert kinds.count(ChangeKind.SCENARIO_RENAMED) == 1
    assert kinds.count(ChangeKind.SCENARIO_REMOVED) == 1
    assert report.regressed, "the genuine removal must still set the exit code"


def test_comparison_handles_infinite_metrics() -> None:
    """An unchanged infinite metric is unchanged, and a finite one is movement."""
    baseline = SuiteRecord(scenarios=(_record("s", True, math.inf),))
    same = SuiteRecord(scenarios=(_record("s", True, math.inf),))
    assert not compare_suites(same, baseline).changes
    moved = SuiteRecord(scenarios=(_record("s", True, 12.0),))
    report = compare_suites(moved, baseline)
    assert [change.kind for change in report.changes] == [ChangeKind.METRIC_MOVED]

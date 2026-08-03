"""Assertion evaluation, safety metrics, and baseline comparison.

This layer scores a run. It does no simulation, no file I/O, and no rendering.
It reaches the run through a structural view (``trace_view``) rather than by
importing the simulator, so the dependency arrow points only downward.
"""

from __future__ import annotations

from scenario_runner.algorithm.assertions import (
    AssertionResult,
    RunMetrics,
    evaluate_assertion,
    evaluate_assertions,
)
from scenario_runner.algorithm.compare import (
    AssertionRecord,
    Change,
    ChangeKind,
    ComparisonReport,
    ScenarioRecord,
    SuiteRecord,
    Tolerance,
    compare_suites,
    magnitude_class,
    match_renames,
)
from scenario_runner.algorithm.metrics import (
    clearance_series,
    disc_centres,
    footprint_corners,
    nearest_actor,
    time_to_collision_series,
)
from scenario_runner.algorithm.trace_view import BodyView, FloatArray, TraceView

__all__ = [
    "AssertionRecord",
    "AssertionResult",
    "BodyView",
    "Change",
    "ChangeKind",
    "ComparisonReport",
    "FloatArray",
    "RunMetrics",
    "ScenarioRecord",
    "SuiteRecord",
    "Tolerance",
    "TraceView",
    "clearance_series",
    "compare_suites",
    "disc_centres",
    "evaluate_assertion",
    "evaluate_assertions",
    "footprint_corners",
    "magnitude_class",
    "match_renames",
    "nearest_actor",
    "time_to_collision_series",
]

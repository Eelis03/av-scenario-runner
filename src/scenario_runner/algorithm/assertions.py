"""Evaluation of the assertion vocabulary over a run.

Every assertion returns the worst value it observed and the time at which that
value occurred. A verdict without evidence cannot be acted on: knowing that
time to collision dipped below the threshold is far less useful than knowing it
reached 0.84 s at t = 6.15 s, and the difference between a scenario that passes
with 0.02 m of margin and one that passes with 4 m of margin does not appear in
the verdict at all.

Boundary convention, applied uniformly and tested at the boundary:

* a lower bound passes when the observed value is greater than or equal to it;
* an upper bound passes when the observed value is less than or equal to it;
* ``no_collision`` is the one strict comparison, because zero clearance is
  contact rather than a near miss.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from scenario_runner.algorithm.metrics import clearance_series, time_to_collision_series
from scenario_runner.algorithm.trace_view import FloatArray, TraceView
from scenario_runner.model import (
    Assertion,
    GoalReached,
    LateralAcceleration,
    LongitudinalAcceleration,
    MinDistance,
    MinTimeToCollision,
    NoCollision,
    Scenario,
    SpeedLimit,
)

__all__ = ["AssertionResult", "RunMetrics", "evaluate_assertion", "evaluate_assertions"]


@dataclass(frozen=True, slots=True)
class AssertionResult:
    """The verdict of one assertion, with the evidence that produced it."""

    name: str
    kind: str
    passed: bool
    worst_value: float
    worst_time: float | None
    unit: str
    bound: str
    detail: str

    @property
    def outcome(self) -> str:
        """``"pass"`` or ``"fail"``."""
        return "pass" if self.passed else "fail"


@dataclass(frozen=True, slots=True)
class RunMetrics:
    """Per-step metric series shared by every assertion evaluated over one run."""

    clearance: FloatArray
    time_to_collision: FloatArray

    @classmethod
    def from_trace(cls, trace: TraceView) -> RunMetrics:
        """Compute every derived series once, so assertions do not recompute them."""
        return cls(
            clearance=clearance_series(trace),
            time_to_collision=time_to_collision_series(trace),
        )


def _at(trace: TraceView, index: int, value: float) -> float | None:
    """Time of the worst sample, or ``None`` when the worst value is not finite."""
    if not math.isfinite(value):
        return None
    return float(trace.time[index])


def _argmin(series: FloatArray) -> int:
    return int(np.argmin(series))


def _argmax(series: FloatArray) -> int:
    return int(np.argmax(series))


def _extreme_signed(series: FloatArray) -> int:
    """Index of the sample with the largest magnitude, keeping the original sign."""
    return int(np.argmax(np.abs(series)))


def evaluate_assertion(
    assertion: Assertion, trace: TraceView, metrics: RunMetrics
) -> AssertionResult:
    """Score a single assertion against one run and its precomputed metrics."""
    match assertion:
        case NoCollision():
            index = _argmin(metrics.clearance)
            worst = float(metrics.clearance[index])
            passed = worst > 0.0
            return AssertionResult(
                name=assertion.name,
                kind=assertion.kind,
                passed=passed,
                worst_value=worst,
                worst_time=_at(trace, index, worst),
                unit="m",
                bound="> 0 m",
                detail=(
                    "no actor in this scenario"
                    if not trace.actors
                    else f"closest approach {worst:.3f} m"
                ),
            )

        case MinTimeToCollision(threshold=threshold):
            index = _argmin(metrics.time_to_collision)
            worst = float(metrics.time_to_collision[index])
            return AssertionResult(
                name=assertion.name,
                kind=assertion.kind,
                passed=worst >= threshold,
                worst_value=worst,
                worst_time=_at(trace, index, worst),
                unit="s",
                bound=f">= {threshold:g} s",
                detail=f"margin {worst - threshold:+.3f} s"
                if math.isfinite(worst)
                else "never closing",
            )

        case LongitudinalAcceleration(minimum=minimum, maximum=maximum):
            index = _extreme_signed(trace.ego_accel)
            worst = float(trace.ego_accel[index])
            inside = bool(np.all((trace.ego_accel >= minimum) & (trace.ego_accel <= maximum)))
            return AssertionResult(
                name=assertion.name,
                kind=assertion.kind,
                passed=inside,
                worst_value=worst,
                worst_time=_at(trace, index, worst),
                unit="m/s^2",
                bound=f"in [{minimum:g}, {maximum:g}] m/s^2",
                detail=f"peak magnitude {abs(worst):.3f} m/s^2",
            )

        case LateralAcceleration(limit=limit):
            index = _extreme_signed(trace.ego_lateral_accel)
            worst = float(trace.ego_lateral_accel[index])
            return AssertionResult(
                name=assertion.name,
                kind=assertion.kind,
                passed=abs(worst) <= limit,
                worst_value=worst,
                worst_time=_at(trace, index, worst),
                unit="m/s^2",
                bound=f"|a| <= {limit:g} m/s^2",
                detail=f"peak magnitude {abs(worst):.3f} m/s^2",
            )

        case SpeedLimit(limit=limit, tolerance=tolerance):
            index = _argmax(trace.ego.speed)
            worst = float(trace.ego.speed[index])
            return AssertionResult(
                name=assertion.name,
                kind=assertion.kind,
                passed=worst <= limit + tolerance,
                worst_value=worst,
                worst_time=_at(trace, index, worst),
                unit="m/s",
                bound=f"<= {limit + tolerance:g} m/s",
                detail=f"exceedance {worst - limit:+.3f} m/s",
            )

        case GoalReached(goal_s=goal_s, time_budget=time_budget):
            reached = np.flatnonzero(trace.ego_s >= goal_s)
            if reached.size == 0:
                worst = float(trace.time[-1])
                shortfall = float(goal_s - trace.ego_s[-1])
                return AssertionResult(
                    name=assertion.name,
                    kind=assertion.kind,
                    passed=False,
                    worst_value=worst,
                    worst_time=worst,
                    unit="s",
                    bound=f"<= {time_budget:g} s",
                    detail=f"goal not reached, {shortfall:.3f} m short at end of run",
                )
            index = int(reached[0])
            worst = float(trace.time[index])
            return AssertionResult(
                name=assertion.name,
                kind=assertion.kind,
                passed=worst <= time_budget,
                worst_value=worst,
                worst_time=worst,
                unit="s",
                bound=f"<= {time_budget:g} s",
                detail=f"reached {goal_s:g} m with {time_budget - worst:+.3f} s to spare",
            )

        case MinDistance(threshold=threshold):
            index = _argmin(metrics.clearance)
            worst = float(metrics.clearance[index])
            return AssertionResult(
                name=assertion.name,
                kind=assertion.kind,
                passed=worst >= threshold,
                worst_value=worst,
                worst_time=_at(trace, index, worst),
                unit="m",
                bound=f">= {threshold:g} m",
                detail=f"margin {worst - threshold:+.3f} m"
                if math.isfinite(worst)
                else "no actor in this scenario",
            )


def evaluate_assertions(scenario: Scenario, trace: TraceView) -> tuple[AssertionResult, ...]:
    """Score every assertion the scenario declares, in declaration order."""
    metrics = RunMetrics.from_trace(trace)
    return tuple(evaluate_assertion(assertion, trace, metrics) for assertion in scenario.assertions)

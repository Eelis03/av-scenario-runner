"""Machine readable suite results.

The JSON document is the interchange format between a run and everything that
consumes one: a CI artefact, a stored regression baseline, and the comparison
tool. It is written with ``allow_nan=False`` so the output is strict JSON;
values that are not finite, which occur whenever a metric such as time to
collision never closes, are encoded as the strings ``"inf"``, ``"-inf"`` and
``"nan"`` and decoded back on read.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Final

from scenario_runner.algorithm import AssertionRecord, AssertionResult, ScenarioRecord, SuiteRecord
from scenario_runner.pipeline import ScenarioResult, SuiteResult

__all__ = [
    "RESULT_VERSION",
    "dump_suite_result",
    "load_suite_record",
    "load_suite_result",
    "suite_from_json",
    "suite_to_json",
]

RESULT_VERSION: Final[str] = "1.0"


def _encode(value: float) -> float | str:
    if math.isnan(value):
        return "nan"
    if math.isinf(value):
        return "inf" if value > 0 else "-inf"
    return value


def _decode(value: object) -> float:
    if isinstance(value, str):
        return {"inf": math.inf, "-inf": -math.inf, "nan": math.nan}[value]
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"expected a number, got {value!r}")
    return float(value)


def _encode_optional(value: float | None) -> float | str | None:
    return None if value is None else _encode(value)


def _decode_optional(value: object) -> float | None:
    return None if value is None else _decode(value)


def _assertion_to_json(item: AssertionResult) -> dict[str, Any]:
    return {
        "name": item.name,
        "kind": item.kind,
        "passed": item.passed,
        "worst_value": _encode(item.worst_value),
        "worst_time": _encode_optional(item.worst_time),
        "unit": item.unit,
        "bound": item.bound,
        "detail": item.detail,
    }


def _assertion_from_json(raw: dict[str, Any]) -> AssertionResult:
    return AssertionResult(
        name=str(raw["name"]),
        kind=str(raw["kind"]),
        passed=bool(raw["passed"]),
        worst_value=_decode(raw["worst_value"]),
        worst_time=_decode_optional(raw["worst_time"]),
        unit=str(raw["unit"]),
        bound=str(raw["bound"]),
        detail=str(raw["detail"]),
    )


def _scenario_to_json(item: ScenarioResult) -> dict[str, Any]:
    return {
        "name": item.name,
        "source": item.source,
        "expected_outcome": item.expected_outcome,
        "passed": item.passed,
        "matches_expectation": item.matches_expectation,
        "steps": item.steps,
        "duration": item.duration,
        "terminated": item.terminated,
        "error": item.error,
        "assertions": [_assertion_to_json(entry) for entry in item.assertions],
    }


def _scenario_from_json(raw: dict[str, Any]) -> ScenarioResult:
    return ScenarioResult(
        name=str(raw["name"]),
        source=str(raw["source"]),
        expected_outcome=str(raw["expected_outcome"]),
        passed=bool(raw["passed"]),
        assertions=tuple(_assertion_from_json(entry) for entry in raw["assertions"]),
        steps=int(raw["steps"]),
        duration=float(raw["duration"]),
        terminated=str(raw["terminated"]),
        error=None if raw["error"] is None else str(raw["error"]),
    )


def suite_to_json(suite: SuiteResult) -> dict[str, Any]:
    """Render ``suite`` as a JSON compatible object."""
    return {
        "result_version": RESULT_VERSION,
        "summary": {
            "total": len(suite.scenarios),
            "passed": len(suite.passed),
            "failed": len(suite.failed),
            "unexpected": len(suite.unexpected),
        },
        "scenarios": [_scenario_to_json(item) for item in suite.scenarios],
    }


def suite_from_json(raw: dict[str, Any]) -> SuiteResult:
    """Rebuild a suite result from a JSON compatible object."""
    version = str(raw.get("result_version", ""))
    if version != RESULT_VERSION:
        raise ValueError(
            f"unsupported result_version {version!r}; this build reads {RESULT_VERSION!r}"
        )
    return SuiteResult(
        scenarios=tuple(_scenario_from_json(entry) for entry in raw["scenarios"])
    )


def dump_suite_result(suite: SuiteResult, path: Path | str) -> Path:
    """Write ``suite`` to ``path`` as strict JSON and return the path."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(suite_to_json(suite), indent=2, sort_keys=False, allow_nan=False)
    target.write_text(text + "\n", encoding="utf-8")
    return target


def load_suite_result(path: Path | str) -> SuiteResult:
    """Read a suite result written by ``dump_suite_result``."""
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    return suite_from_json(raw)


def load_suite_record(path: Path | str) -> SuiteRecord:
    """Read a stored baseline and reduce it to its comparable form."""
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    version = str(raw.get("result_version", ""))
    if version != RESULT_VERSION:
        raise ValueError(
            f"unsupported result_version {version!r}; this build reads {RESULT_VERSION!r}"
        )
    return SuiteRecord(
        scenarios=tuple(
            ScenarioRecord(
                name=str(entry["name"]),
                passed=bool(entry["passed"]),
                assertions=tuple(
                    AssertionRecord(
                        name=str(item["name"]),
                        kind=str(item["kind"]),
                        passed=bool(item["passed"]),
                        worst_value=_decode(item["worst_value"]),
                        worst_time=_decode_optional(item["worst_time"]),
                    )
                    for item in entry["assertions"]
                ),
            )
            for entry in raw["scenarios"]
        )
    )

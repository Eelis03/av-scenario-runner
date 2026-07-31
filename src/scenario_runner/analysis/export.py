"""Export of a run to the JSON trace consumed by the browser playback.

The Python package never imports the ``viz`` directory and never requires it to
be built. This module writes a file; whether anything reads it is not the
package's concern. The schema is versioned so the browser can refuse a trace it
does not understand rather than draw nonsense.

Values that are not finite are written as ``null``, because the consumer is
JavaScript and ``JSON.parse`` has no representation for infinity.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Final

from scenario_runner.algorithm import (
    AssertionResult,
    FloatArray,
    clearance_series,
    time_to_collision_series,
)
from scenario_runner.model import VehicleShape
from scenario_runner.pipeline import BodyTrace, Trace

__all__ = ["TRACE_VERSION", "dump_trace", "trace_to_json"]

TRACE_VERSION: Final[str] = "1.0"
_DECIMALS: Final[int] = 4


def _series(values: FloatArray) -> list[float | None]:
    return [None if not math.isfinite(v) else round(float(v), _DECIMALS) for v in values]


def _shape(shape: VehicleShape) -> dict[str, Any]:
    return {
        "length": shape.length,
        "width": shape.width,
        "front_offset": shape.front_offset,
        "rear_offset": shape.rear_offset,
    }


def _body(body: BodyTrace, role: str) -> dict[str, Any]:
    return {
        "id": body.identifier,
        "role": role,
        "shape": _shape(body.shape),
        "x": _series(body.x),
        "y": _series(body.y),
        "yaw": _series(body.yaw),
        "speed": _series(body.speed),
    }


def trace_to_json(
    trace: Trace, assertions: tuple[AssertionResult, ...] = ()
) -> dict[str, Any]:
    """Render ``trace`` as the JSON object the browser playback reads."""
    road = trace.road
    return {
        "trace_version": TRACE_VERSION,
        "scenario": trace.scenario_name,
        "dt": trace.dt,
        "seed": trace.seed,
        "terminated": trace.terminated,
        "collided": trace.collided,
        "road": {
            "kind": road.kind,
            "lanes": road.lanes,
            "lane_width": road.lane_width,
            "length": road.length,
            "curvature": road.curvature,
            "speed_limit": road.speed_limit,
        },
        "time": _series(trace.time),
        "bodies": [
            _body(trace.ego, "ego"),
            *(_body(actor, "actor") for actor in trace.actors),
        ],
        "metrics": {
            "clearance": _series(clearance_series(trace)),
            "time_to_collision": _series(time_to_collision_series(trace)),
            "ego_accel": _series(trace.ego_accel),
            "ego_lateral_accel": _series(trace.ego_lateral_accel),
            "ego_s": _series(trace.ego_s),
        },
        "assertions": [
            {
                "name": item.name,
                "kind": item.kind,
                "passed": item.passed,
                "worst_value": None
                if not math.isfinite(item.worst_value)
                else round(item.worst_value, _DECIMALS),
                "worst_time": item.worst_time,
                "unit": item.unit,
                "bound": item.bound,
            }
            for item in assertions
        ],
    }


def dump_trace(
    trace: Trace, path: Path | str, assertions: tuple[AssertionResult, ...] = ()
) -> Path:
    """Write the browser trace for ``trace`` to ``path`` and return the path."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(trace_to_json(trace, assertions), indent=1, allow_nan=False)
    target.write_text(payload + "\n", encoding="utf-8")
    return target

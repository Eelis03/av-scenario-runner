"""Parsing and validation of scenario documents.

Every value read from the document is checked before it reaches the typed
model, and every rejection names the offending field and, where the field can
be found in the source text, its line. Unknown keys are rejected rather than
ignored: a scenario with a misspelled key is a scenario that does not test what
its author believes it tests, and silently passing such a document is the
failure mode this harness exists to prevent.
"""

from __future__ import annotations

import math
import tomllib
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Final, NoReturn

from scenario_runner.model.errors import ScenarioError
from scenario_runner.model.locate import SourceIndex, decode_error_line
from scenario_runner.model.scenario import (
    ASSERTION_KINDS,
    KNOWN_CONTROLLERS,
    SUPPORTED_FORMAT_VERSIONS,
    ActorSpec,
    Assertion,
    EgoSpec,
    GoalReached,
    IdmParams,
    LaneChangeSpec,
    LateralAcceleration,
    LateralParams,
    LongitudinalAcceleration,
    MinDistance,
    MinTimeHeadway,
    MinTimeToCollision,
    NoCollision,
    PerceptionSpec,
    ReactiveBehaviour,
    RoadSpec,
    Scenario,
    ScheduleEntry,
    ScriptedBehaviour,
    SpeedLimit,
    TerminationSpec,
    VehicleShape,
)

__all__ = ["load_scenario", "load_suite", "parse_scenario"]

_ROOT_KEYS: Final[frozenset[str]] = frozenset(
    {
        "format_version",
        "name",
        "description",
        "expected_outcome",
        "dt",
        "seed",
        "road",
        "ego",
        "actor",
        "termination",
        "assert",
        "perception",
    }
)
_ROAD_KEYS_COMMON: Final[frozenset[str]] = frozenset(
    {"kind", "lanes", "lane_width", "length", "speed_limit"}
)
_ROAD_KEYS_ARC: Final[frozenset[str]] = frozenset({"radius", "direction"})
_EGO_KEYS: Final[frozenset[str]] = frozenset(
    {"lane", "s", "speed", "controller", "target_lane", "longitudinal", "lateral", "shape"}
)
_ACTOR_KEYS: Final[frozenset[str]] = frozenset(
    {"id", "lane", "s", "speed", "behaviour", "schedule", "lane_change", "reactive", "shape"}
)
_TERMINATION_KEYS: Final[frozenset[str]] = frozenset(
    {"max_time", "goal_s", "stop_at_goal", "stop_on_collision"}
)
_PERCEPTION_KEYS: Final[frozenset[str]] = frozenset({"range_noise_std", "speed_noise_std"})
_IDM_KEYS: Final[frozenset[str]] = frozenset(
    {
        "desired_speed",
        "time_gap",
        "min_gap",
        "max_accel",
        "comfort_decel",
        "max_decel",
        "exponent",
    }
)
_LATERAL_KEYS: Final[frozenset[str]] = frozenset(
    {"lookahead_gain", "lookahead_min", "lookahead_max", "max_steer", "max_steer_rate"}
)
_SHAPE_KEYS: Final[frozenset[str]] = frozenset(
    {"wheelbase", "front_overhang", "rear_overhang", "width", "disc_count"}
)
_ASSERT_KEYS: Final[Mapping[str, frozenset[str]]] = {
    "no_collision": frozenset(),
    "min_time_to_collision": frozenset({"threshold"}),
    "min_time_headway": frozenset({"threshold"}),
    "longitudinal_acceleration": frozenset({"minimum", "maximum"}),
    "lateral_acceleration": frozenset({"limit"}),
    "speed_limit": frozenset({"limit", "tolerance"}),
    "goal_reached": frozenset({"goal_s", "time_budget"}),
    "min_distance": frozenset({"threshold"}),
}
_PERCEPTION_MIN_VERSION: Final[str] = "1.1"


class _Reader:
    """Field access helpers that raise a located ``ScenarioError`` on any problem."""

    def __init__(self, source: str, index: SourceIndex) -> None:
        self.source = source
        self.index = index

    def fail(self, field: str, message: str) -> NoReturn:
        raise ScenarioError(
            message, source=self.source, field=field, line=self.index.locate(field)
        )

    def table(self, data: Mapping[str, object], key: str, field: str) -> Mapping[str, object]:
        value = data.get(key)
        if value is None:
            return {}
        if not isinstance(value, dict):
            self.fail(field, f"must be a table, got {_type_name(value)}")
        return value

    def require_table(
        self, data: Mapping[str, object], key: str, field: str
    ) -> Mapping[str, object]:
        if key not in data:
            self.fail(field, "required table is absent")
        return self.table(data, key, field)

    def reject_unknown(
        self, data: Mapping[str, object], allowed: frozenset[str], prefix: str
    ) -> None:
        for key in data:
            if key not in allowed:
                field = f"{prefix}.{key}" if prefix else key
                permitted = ", ".join(sorted(allowed)) or "none"
                self.fail(field, f"unknown key; permitted keys are: {permitted}")

    def text(self, data: Mapping[str, object], key: str, field: str, default: str | None) -> str:
        value = data.get(key)
        if value is None:
            if default is None:
                self.fail(field, "required field is absent")
            return default
        if not isinstance(value, str) or not value.strip():
            self.fail(field, f"must be a non-empty string, got {_type_name(value)}")
        return value

    def number(
        self,
        data: Mapping[str, object],
        key: str,
        field: str,
        default: float | None,
        *,
        minimum: float = -math.inf,
        maximum: float = math.inf,
        exclusive_min: bool = False,
    ) -> float:
        value = data.get(key)
        if value is None:
            if default is None:
                self.fail(field, "required field is absent")
            return default
        if isinstance(value, bool) or not isinstance(value, int | float):
            self.fail(field, f"must be a number, got {_type_name(value)}")
        number = float(value)
        if not math.isfinite(number):
            self.fail(field, f"must be finite, got {number}")
        low_ok = number > minimum if exclusive_min else number >= minimum
        if not low_ok or number > maximum:
            bound = _range_text(minimum, maximum, exclusive_min)
            self.fail(field, f"must be {bound}, got {number}")
        return number

    def integer(
        self,
        data: Mapping[str, object],
        key: str,
        field: str,
        default: int | None,
        *,
        minimum: int,
        maximum: int,
    ) -> int:
        value = data.get(key)
        if value is None:
            if default is None:
                self.fail(field, "required field is absent")
            return default
        if isinstance(value, bool) or not isinstance(value, int):
            self.fail(field, f"must be an integer, got {_type_name(value)}")
        if not minimum <= value <= maximum:
            self.fail(field, f"must be in [{minimum}, {maximum}], got {value}")
        return value

    def flag(self, data: Mapping[str, object], key: str, field: str, default: bool) -> bool:
        value = data.get(key)
        if value is None:
            return default
        if not isinstance(value, bool):
            self.fail(field, f"must be a boolean, got {_type_name(value)}")
        return value

    def choice(
        self,
        data: Mapping[str, object],
        key: str,
        field: str,
        default: str | None,
        options: Sequence[str],
    ) -> str:
        value = self.text(data, key, field, default)
        if value not in options:
            self.fail(field, f"must be one of {', '.join(sorted(options))}, got {value!r}")
        return value

    def table_list(
        self, data: Mapping[str, object], key: str, field: str
    ) -> tuple[Mapping[str, object], ...]:
        value = data.get(key)
        if value is None:
            return ()
        if not isinstance(value, list):
            self.fail(field, f"must be an array of tables, got {_type_name(value)}")
        items: list[Mapping[str, object]] = []
        for position, item in enumerate(value):
            if not isinstance(item, dict):
                self.fail(f"{field}[{position}]", f"must be a table, got {_type_name(item)}")
            items.append(item)
        return tuple(items)


def _type_name(value: object) -> str:
    if isinstance(value, bool):
        return "boolean"
    return {
        int: "integer",
        float: "float",
        str: "string",
        list: "array",
        dict: "table",
    }.get(type(value), type(value).__name__)


def _range_text(minimum: float, maximum: float, exclusive_min: bool) -> str:
    low = "greater than" if exclusive_min else "at least"
    if maximum == math.inf:
        return f"{low} {minimum:g}"
    if minimum == -math.inf:
        return f"at most {maximum:g}"
    open_bracket = "(" if exclusive_min else "["
    return f"in {open_bracket}{minimum:g}, {maximum:g}]"


def parse_scenario(text: str, *, source: str = "<scenario>") -> Scenario:
    """Parse and validate one scenario document."""
    index = SourceIndex(text)
    reader = _Reader(source, index)
    try:
        raw = tomllib.loads(text)
    except tomllib.TOMLDecodeError as exc:
        raise ScenarioError(
            f"invalid TOML: {exc}", source=source, line=decode_error_line(str(exc))
        ) from exc

    version = reader.choice(
        raw, "format_version", "format_version", None, SUPPORTED_FORMAT_VERSIONS
    )
    reader.reject_unknown(raw, _ROOT_KEYS, "")

    name = reader.text(raw, "name", "name", None)
    description = reader.text(raw, "description", "description", "")
    expected = reader.choice(
        raw, "expected_outcome", "expected_outcome", "pass", ("pass", "fail")
    )
    dt = reader.number(raw, "dt", "dt", 0.05, minimum=0.0, maximum=1.0, exclusive_min=True)
    seed = reader.integer(raw, "seed", "seed", 0, minimum=0, maximum=2**31 - 1)

    road = _parse_road(reader, reader.require_table(raw, "road", "road"))
    ego = _parse_ego(reader, reader.require_table(raw, "ego", "ego"), road)
    actors = _parse_actors(reader, reader.table_list(raw, "actor", "actor"), road)
    termination = _parse_termination(reader, reader.table(raw, "termination", "termination"), road)
    assertions = _parse_assertions(
        reader, reader.table_list(raw, "assert", "assert"), road, termination
    )
    perception = _parse_perception(reader, raw, version)

    return Scenario(
        name=name,
        format_version=version,
        description=description,
        expected_outcome=expected,
        dt=dt,
        seed=seed,
        road=road,
        ego=ego,
        actors=actors,
        termination=termination,
        assertions=assertions,
        perception=perception,
        source=source,
    )


def load_scenario(path: Path | str) -> Scenario:
    """Read and validate the scenario document stored at ``path``."""
    file_path = Path(path)
    try:
        text = file_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ScenarioError(f"cannot read scenario: {exc}", source=str(file_path)) from exc
    return parse_scenario(text, source=file_path.name)


def load_suite(paths: Sequence[Path | str]) -> tuple[Scenario, ...]:
    """Load every scenario in ``paths``, preserving order."""
    return tuple(load_scenario(path) for path in paths)


def _parse_road(reader: _Reader, raw: Mapping[str, object]) -> RoadSpec:
    kind = reader.choice(raw, "kind", "road.kind", "straight", ("straight", "arc"))
    allowed = _ROAD_KEYS_COMMON | (_ROAD_KEYS_ARC if kind == "arc" else frozenset())
    reader.reject_unknown(raw, allowed, "road")

    lanes = reader.integer(raw, "lanes", "road.lanes", 1, minimum=1, maximum=8)
    lane_width = reader.number(
        raw, "lane_width", "road.lane_width", 3.5, minimum=0.0, exclusive_min=True
    )
    length = reader.number(raw, "length", "road.length", 400.0, minimum=0.0, exclusive_min=True)
    speed_limit = reader.number(
        raw, "speed_limit", "road.speed_limit", 13.9, minimum=0.0, exclusive_min=True
    )
    radius = 0.0
    direction = "left"
    if kind == "arc":
        radius = reader.number(
            raw, "radius", "road.radius", None, minimum=1.0, exclusive_min=False
        )
        direction = reader.choice(
            raw, "direction", "road.direction", "left", ("left", "right")
        )
    return RoadSpec(
        kind=kind,
        lanes=lanes,
        lane_width=lane_width,
        length=length,
        speed_limit=speed_limit,
        radius=radius,
        direction=direction,
    )


def _parse_idm(reader: _Reader, raw: Mapping[str, object], prefix: str) -> IdmParams:
    reader.reject_unknown(raw, _IDM_KEYS, prefix)
    defaults = IdmParams()
    return IdmParams(
        desired_speed=reader.number(
            raw,
            "desired_speed",
            f"{prefix}.desired_speed",
            defaults.desired_speed,
            minimum=0.0,
            exclusive_min=True,
        ),
        time_gap=reader.number(
            raw, "time_gap", f"{prefix}.time_gap", defaults.time_gap, minimum=0.0
        ),
        min_gap=reader.number(
            raw, "min_gap", f"{prefix}.min_gap", defaults.min_gap, minimum=0.0
        ),
        max_accel=reader.number(
            raw,
            "max_accel",
            f"{prefix}.max_accel",
            defaults.max_accel,
            minimum=0.0,
            exclusive_min=True,
        ),
        comfort_decel=reader.number(
            raw,
            "comfort_decel",
            f"{prefix}.comfort_decel",
            defaults.comfort_decel,
            minimum=0.0,
            exclusive_min=True,
        ),
        max_decel=reader.number(
            raw,
            "max_decel",
            f"{prefix}.max_decel",
            defaults.max_decel,
            minimum=0.0,
            exclusive_min=True,
        ),
        exponent=reader.number(
            raw,
            "exponent",
            f"{prefix}.exponent",
            defaults.exponent,
            minimum=0.0,
            exclusive_min=True,
        ),
    )


def _parse_lateral(reader: _Reader, raw: Mapping[str, object], prefix: str) -> LateralParams:
    reader.reject_unknown(raw, _LATERAL_KEYS, prefix)
    defaults = LateralParams()
    lookahead_min = reader.number(
        raw,
        "lookahead_min",
        f"{prefix}.lookahead_min",
        defaults.lookahead_min,
        minimum=0.0,
        exclusive_min=True,
    )
    lookahead_max = reader.number(
        raw,
        "lookahead_max",
        f"{prefix}.lookahead_max",
        defaults.lookahead_max,
        minimum=lookahead_min,
    )
    return LateralParams(
        lookahead_gain=reader.number(
            raw, "lookahead_gain", f"{prefix}.lookahead_gain", defaults.lookahead_gain, minimum=0.0
        ),
        lookahead_min=lookahead_min,
        lookahead_max=lookahead_max,
        max_steer=reader.number(
            raw,
            "max_steer",
            f"{prefix}.max_steer",
            defaults.max_steer,
            minimum=0.0,
            maximum=math.pi / 2,
            exclusive_min=True,
        ),
        max_steer_rate=reader.number(
            raw,
            "max_steer_rate",
            f"{prefix}.max_steer_rate",
            defaults.max_steer_rate,
            minimum=0.0,
            exclusive_min=True,
        ),
    )


def _parse_shape(reader: _Reader, raw: Mapping[str, object], prefix: str) -> VehicleShape:
    reader.reject_unknown(raw, _SHAPE_KEYS, prefix)
    defaults = VehicleShape()
    return VehicleShape(
        wheelbase=reader.number(
            raw,
            "wheelbase",
            f"{prefix}.wheelbase",
            defaults.wheelbase,
            minimum=0.0,
            exclusive_min=True,
        ),
        front_overhang=reader.number(
            raw, "front_overhang", f"{prefix}.front_overhang", defaults.front_overhang, minimum=0.0
        ),
        rear_overhang=reader.number(
            raw, "rear_overhang", f"{prefix}.rear_overhang", defaults.rear_overhang, minimum=0.0
        ),
        width=reader.number(
            raw, "width", f"{prefix}.width", defaults.width, minimum=0.0, exclusive_min=True
        ),
        disc_count=reader.integer(
            raw, "disc_count", f"{prefix}.disc_count", defaults.disc_count, minimum=1, maximum=8
        ),
    )


def _parse_ego(reader: _Reader, raw: Mapping[str, object], road: RoadSpec) -> EgoSpec:
    reader.reject_unknown(raw, _EGO_KEYS, "ego")
    lane = reader.integer(raw, "lane", "ego.lane", 0, minimum=0, maximum=road.lanes - 1)
    controller = reader.text(raw, "controller", "ego.controller", "idm_lane_keep")
    if controller not in KNOWN_CONTROLLERS:
        reader.fail(
            "ego.controller",
            f"unknown controller {controller!r}; known controllers are: "
            f"{', '.join(sorted(KNOWN_CONTROLLERS))}",
        )
    return EgoSpec(
        lane=lane,
        s=reader.number(raw, "s", "ego.s", 0.0, minimum=0.0, maximum=road.length),
        speed=reader.number(raw, "speed", "ego.speed", 0.0, minimum=0.0),
        controller=controller,
        target_lane=reader.integer(
            raw, "target_lane", "ego.target_lane", lane, minimum=0, maximum=road.lanes - 1
        ),
        longitudinal=_parse_idm(
            reader, reader.table(raw, "longitudinal", "ego.longitudinal"), "ego.longitudinal"
        ),
        lateral=_parse_lateral(reader, reader.table(raw, "lateral", "ego.lateral"), "ego.lateral"),
        shape=_parse_shape(reader, reader.table(raw, "shape", "ego.shape"), "ego.shape"),
    )


def _parse_schedule(
    reader: _Reader, raw: Mapping[str, object], prefix: str
) -> tuple[ScheduleEntry, ...]:
    items = reader.table_list(raw, "schedule", f"{prefix}.schedule")
    entries: list[ScheduleEntry] = []
    previous = -math.inf
    for position, item in enumerate(items):
        field = f"{prefix}.schedule[{position}]"
        reader.reject_unknown(item, frozenset({"time", "accel"}), field)
        time = reader.number(item, "time", f"{field}.time", None, minimum=0.0)
        accel = reader.number(item, "accel", f"{field}.accel", None, minimum=-20.0, maximum=20.0)
        if position == 0 and time != 0.0:
            reader.fail(f"{field}.time", f"first schedule entry must start at 0.0, got {time}")
        if time <= previous:
            reader.fail(
                f"{field}.time",
                f"schedule times must strictly increase, got {time} after {previous}",
            )
        previous = time
        entries.append(ScheduleEntry(time=time, accel=accel))
    return tuple(entries)


def _parse_lane_change(
    reader: _Reader, raw: Mapping[str, object], prefix: str, road: RoadSpec
) -> LaneChangeSpec | None:
    value = raw.get("lane_change")
    if value is None:
        return None
    field = f"{prefix}.lane_change"
    if not isinstance(value, dict):
        reader.fail(field, f"must be a table, got {_type_name(value)}")
    reader.reject_unknown(value, frozenset({"target_lane", "start_time", "duration"}), field)
    return LaneChangeSpec(
        target_lane=reader.integer(
            value, "target_lane", f"{field}.target_lane", None, minimum=0, maximum=road.lanes - 1
        ),
        start_time=reader.number(value, "start_time", f"{field}.start_time", None, minimum=0.0),
        duration=reader.number(
            value, "duration", f"{field}.duration", None, minimum=0.0, exclusive_min=True
        ),
    )


def _parse_actors(
    reader: _Reader, items: tuple[Mapping[str, object], ...], road: RoadSpec
) -> tuple[ActorSpec, ...]:
    actors: list[ActorSpec] = []
    seen: set[str] = set()
    for position, raw in enumerate(items):
        prefix = f"actor[{position}]"
        reader.reject_unknown(raw, _ACTOR_KEYS, prefix)
        identifier = reader.text(raw, "id", f"{prefix}.id", None)
        if identifier in seen:
            reader.fail(f"{prefix}.id", f"duplicate actor id {identifier!r}")
        if identifier == "ego":
            reader.fail(f"{prefix}.id", "actor id 'ego' is reserved for the vehicle under test")
        seen.add(identifier)

        kind = reader.choice(
            raw, "behaviour", f"{prefix}.behaviour", None, ("scripted", "reactive")
        )
        behaviour: ScriptedBehaviour | ReactiveBehaviour
        if kind == "scripted":
            if "reactive" in raw:
                reader.fail(
                    f"{prefix}.reactive", "only a reactive actor may carry a 'reactive' table"
                )
            behaviour = ScriptedBehaviour(
                schedule=_parse_schedule(reader, raw, prefix),
                lane_change=_parse_lane_change(reader, raw, prefix, road),
            )
        else:
            for forbidden in ("schedule", "lane_change"):
                if forbidden in raw:
                    reader.fail(
                        f"{prefix}.{forbidden}",
                        f"a reactive actor may not carry a {forbidden!r} field",
                    )
            behaviour = ReactiveBehaviour(
                idm=_parse_idm(
                    reader,
                    reader.table(raw, "reactive", f"{prefix}.reactive"),
                    f"{prefix}.reactive",
                )
            )

        actors.append(
            ActorSpec(
                identifier=identifier,
                lane=reader.integer(
                    raw, "lane", f"{prefix}.lane", None, minimum=0, maximum=road.lanes - 1
                ),
                s=reader.number(raw, "s", f"{prefix}.s", None),
                speed=reader.number(raw, "speed", f"{prefix}.speed", None, minimum=0.0),
                behaviour=behaviour,
                shape=_parse_shape(
                    reader, reader.table(raw, "shape", f"{prefix}.shape"), f"{prefix}.shape"
                ),
            )
        )
    return tuple(actors)


def _parse_termination(
    reader: _Reader, raw: Mapping[str, object], road: RoadSpec
) -> TerminationSpec:
    reader.reject_unknown(raw, _TERMINATION_KEYS, "termination")
    goal_s: float | None = None
    if "goal_s" in raw:
        goal_s = reader.number(
            raw, "goal_s", "termination.goal_s", None, minimum=0.0, maximum=road.length
        )
    stop_at_goal = reader.flag(raw, "stop_at_goal", "termination.stop_at_goal", False)
    if stop_at_goal and goal_s is None:
        reader.fail("termination.stop_at_goal", "requires termination.goal_s to be set")
    return TerminationSpec(
        max_time=reader.number(
            raw, "max_time", "termination.max_time", 20.0, minimum=0.0, exclusive_min=True
        ),
        goal_s=goal_s,
        stop_at_goal=stop_at_goal,
        stop_on_collision=reader.flag(
            raw, "stop_on_collision", "termination.stop_on_collision", True
        ),
    )


def _parse_assertions(
    reader: _Reader,
    items: tuple[Mapping[str, object], ...],
    road: RoadSpec,
    termination: TerminationSpec,
) -> tuple[Assertion, ...]:
    if not items:
        reader.fail("assert", "a scenario must declare at least one assertion")
    assertions: list[Assertion] = []
    counts: dict[str, int] = {}
    names: set[str] = set()
    for position, raw in enumerate(items):
        prefix = f"assert[{position}]"
        kind = reader.choice(raw, "kind", f"{prefix}.kind", None, ASSERTION_KINDS)
        reader.reject_unknown(raw, _ASSERT_KEYS[kind] | frozenset({"kind", "name"}), prefix)
        counts[kind] = counts.get(kind, 0) + 1
        default_name = kind if counts[kind] == 1 else f"{kind}_{counts[kind]}"
        name = reader.text(raw, "name", f"{prefix}.name", default_name)
        if name in names:
            reader.fail(f"{prefix}.name", f"duplicate assertion name {name!r}")
        names.add(name)
        assertions.append(_parse_assertion(reader, raw, prefix, kind, name, road, termination))
    return tuple(assertions)


def _parse_assertion(
    reader: _Reader,
    raw: Mapping[str, object],
    prefix: str,
    kind: str,
    name: str,
    road: RoadSpec,
    termination: TerminationSpec,
) -> Assertion:
    if kind == "no_collision":
        return NoCollision(name=name)
    if kind == "min_time_to_collision":
        return MinTimeToCollision(
            name=name,
            threshold=reader.number(
                raw, "threshold", f"{prefix}.threshold", None, minimum=0.0, exclusive_min=True
            ),
        )
    if kind == "min_time_headway":
        return MinTimeHeadway(
            name=name,
            threshold=reader.number(
                raw, "threshold", f"{prefix}.threshold", None, minimum=0.0, exclusive_min=True
            ),
        )
    if kind == "longitudinal_acceleration":
        minimum = reader.number(raw, "minimum", f"{prefix}.minimum", None, maximum=0.0)
        maximum = reader.number(raw, "maximum", f"{prefix}.maximum", None, minimum=0.0)
        if minimum >= maximum:
            reader.fail(
                f"{prefix}.minimum", f"must be below maximum, got {minimum} against {maximum}"
            )
        return LongitudinalAcceleration(name=name, minimum=minimum, maximum=maximum)
    if kind == "lateral_acceleration":
        return LateralAcceleration(
            name=name,
            limit=reader.number(
                raw, "limit", f"{prefix}.limit", None, minimum=0.0, exclusive_min=True
            ),
        )
    if kind == "speed_limit":
        return SpeedLimit(
            name=name,
            limit=reader.number(
                raw, "limit", f"{prefix}.limit", road.speed_limit, minimum=0.0, exclusive_min=True
            ),
            tolerance=reader.number(raw, "tolerance", f"{prefix}.tolerance", 0.0, minimum=0.0),
        )
    if kind == "goal_reached":
        if "goal_s" not in raw and termination.goal_s is None:
            reader.fail(
                f"{prefix}.goal_s",
                "required because termination.goal_s is not set",
            )
        return GoalReached(
            name=name,
            goal_s=reader.number(
                raw, "goal_s", f"{prefix}.goal_s", termination.goal_s, minimum=0.0
            ),
            time_budget=reader.number(
                raw,
                "time_budget",
                f"{prefix}.time_budget",
                termination.max_time,
                minimum=0.0,
                exclusive_min=True,
            ),
        )
    return MinDistance(
        name=name,
        threshold=reader.number(raw, "threshold", f"{prefix}.threshold", None, minimum=0.0),
    )


def _parse_perception(
    reader: _Reader, raw: Mapping[str, object], version: str
) -> PerceptionSpec:
    if "perception" not in raw:
        return PerceptionSpec()
    if version < _PERCEPTION_MIN_VERSION:
        reader.fail(
            "perception",
            f"the [perception] table requires format_version {_PERCEPTION_MIN_VERSION} "
            f"or later; this document declares {version}",
        )
    table = reader.table(raw, "perception", "perception")
    reader.reject_unknown(table, _PERCEPTION_KEYS, "perception")
    return PerceptionSpec(
        range_noise_std=reader.number(
            table, "range_noise_std", "perception.range_noise_std", 0.0, minimum=0.0
        ),
        speed_noise_std=reader.number(
            table, "speed_noise_std", "perception.speed_noise_std", 0.0, minimum=0.0
        ),
    )

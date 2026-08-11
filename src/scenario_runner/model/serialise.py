"""Rendering a validated scenario back to TOML.

Every field is written explicitly rather than relying on defaults, so
``parse_scenario(to_toml(scenario))`` reconstructs an equal scenario. Floats are
written with ``repr``, which is the shortest decimal string that reads back to
the same double.
"""

from __future__ import annotations

import json
from collections.abc import Iterable

from scenario_runner.model.scenario import (
    ActorSpec,
    Assertion,
    GoalReached,
    IdmParams,
    LateralAcceleration,
    LateralParams,
    LongitudinalAcceleration,
    MinDistance,
    MinTimeHeadway,
    MinTimeToCollision,
    NoCollision,
    Scenario,
    ScriptedBehaviour,
    SpeedLimit,
    VehicleShape,
)

__all__ = ["to_toml"]


def _num(value: float) -> str:
    return repr(float(value))


def _text(value: str) -> str:
    return json.dumps(value)


def _flag(value: bool) -> str:
    return "true" if value else "false"


def _inline(pairs: Iterable[tuple[str, str]]) -> str:
    return "{ " + ", ".join(f"{key} = {value}" for key, value in pairs) + " }"


def _idm_pairs(params: IdmParams) -> list[tuple[str, str]]:
    return [
        ("desired_speed", _num(params.desired_speed)),
        ("time_gap", _num(params.time_gap)),
        ("min_gap", _num(params.min_gap)),
        ("max_accel", _num(params.max_accel)),
        ("comfort_decel", _num(params.comfort_decel)),
        ("max_decel", _num(params.max_decel)),
        ("exponent", _num(params.exponent)),
    ]


def _lateral_pairs(params: LateralParams) -> list[tuple[str, str]]:
    return [
        ("lookahead_gain", _num(params.lookahead_gain)),
        ("lookahead_min", _num(params.lookahead_min)),
        ("lookahead_max", _num(params.lookahead_max)),
        ("max_steer", _num(params.max_steer)),
        ("max_steer_rate", _num(params.max_steer_rate)),
    ]


def _shape_pairs(shape: VehicleShape) -> list[tuple[str, str]]:
    return [
        ("wheelbase", _num(shape.wheelbase)),
        ("front_overhang", _num(shape.front_overhang)),
        ("rear_overhang", _num(shape.rear_overhang)),
        ("width", _num(shape.width)),
        ("disc_count", str(shape.disc_count)),
    ]


def _actor_lines(actor: ActorSpec) -> list[str]:
    lines = [
        "[[actor]]",
        f"id = {_text(actor.identifier)}",
        f"lane = {actor.lane}",
        f"s = {_num(actor.s)}",
        f"speed = {_num(actor.speed)}",
        f"behaviour = {_text(actor.behaviour.kind)}",
    ]
    if isinstance(actor.behaviour, ScriptedBehaviour):
        entries = ", ".join(
            _inline([("time", _num(entry.time)), ("accel", _num(entry.accel))])
            for entry in actor.behaviour.schedule
        )
        lines.append(f"schedule = [{entries}]")
        change = actor.behaviour.lane_change
        if change is not None:
            lines.append(
                "lane_change = "
                + _inline(
                    [
                        ("target_lane", str(change.target_lane)),
                        ("start_time", _num(change.start_time)),
                        ("duration", _num(change.duration)),
                    ]
                )
            )
    else:
        lines.append("reactive = " + _inline(_idm_pairs(actor.behaviour.idm)))
    lines.append("shape = " + _inline(_shape_pairs(actor.shape)))
    return lines


def _assertion_lines(assertion: Assertion) -> list[str]:
    lines = ["[[assert]]", f"kind = {_text(assertion.kind)}", f"name = {_text(assertion.name)}"]
    match assertion:
        case NoCollision():
            pass
        case MinTimeToCollision(threshold=threshold) | MinTimeHeadway(threshold=threshold):
            lines.append(f"threshold = {_num(threshold)}")
        case LongitudinalAcceleration(minimum=minimum, maximum=maximum):
            lines.append(f"minimum = {_num(minimum)}")
            lines.append(f"maximum = {_num(maximum)}")
        case LateralAcceleration(limit=limit):
            lines.append(f"limit = {_num(limit)}")
        case SpeedLimit(limit=limit, tolerance=tolerance):
            lines.append(f"limit = {_num(limit)}")
            lines.append(f"tolerance = {_num(tolerance)}")
        case GoalReached(goal_s=goal_s, time_budget=time_budget):
            lines.append(f"goal_s = {_num(goal_s)}")
            lines.append(f"time_budget = {_num(time_budget)}")
        case MinDistance(threshold=threshold):
            lines.append(f"threshold = {_num(threshold)}")
    return lines


def to_toml(scenario: Scenario) -> str:
    """Render ``scenario`` as a TOML document that parses back to an equal object."""
    road = scenario.road
    ego = scenario.ego
    lines = [
        f"format_version = {_text(scenario.format_version)}",
        f"name = {_text(scenario.name)}",
        f"description = {_text(scenario.description)}",
        f"expected_outcome = {_text(scenario.expected_outcome)}",
        f"dt = {_num(scenario.dt)}",
        f"seed = {scenario.seed}",
        "",
        "[road]",
        f"kind = {_text(road.kind)}",
        f"lanes = {road.lanes}",
        f"lane_width = {_num(road.lane_width)}",
        f"length = {_num(road.length)}",
        f"speed_limit = {_num(road.speed_limit)}",
    ]
    if road.kind == "arc":
        lines.append(f"radius = {_num(road.radius)}")
        lines.append(f"direction = {_text(road.direction)}")

    lines.extend(
        [
            "",
            "[ego]",
            f"lane = {ego.lane}",
            f"s = {_num(ego.s)}",
            f"speed = {_num(ego.speed)}",
            f"controller = {_text(ego.controller)}",
            f"target_lane = {ego.target_lane}",
            "",
            "[ego.longitudinal]",
            *(f"{key} = {value}" for key, value in _idm_pairs(ego.longitudinal)),
            "",
            "[ego.lateral]",
            *(f"{key} = {value}" for key, value in _lateral_pairs(ego.lateral)),
            "",
            "[ego.shape]",
            *(f"{key} = {value}" for key, value in _shape_pairs(ego.shape)),
        ]
    )

    for actor in scenario.actors:
        lines.append("")
        lines.extend(_actor_lines(actor))

    termination = scenario.termination
    lines.extend(["", "[termination]", f"max_time = {_num(termination.max_time)}"])
    if termination.goal_s is not None:
        lines.append(f"goal_s = {_num(termination.goal_s)}")
    lines.append(f"stop_at_goal = {_flag(termination.stop_at_goal)}")
    lines.append(f"stop_on_collision = {_flag(termination.stop_on_collision)}")

    for assertion in scenario.assertions:
        lines.append("")
        lines.extend(_assertion_lines(assertion))

    if scenario.format_version >= "1.1":
        lines.extend(
            [
                "",
                "[perception]",
                f"range_noise_std = {_num(scenario.perception.range_noise_std)}",
                f"speed_noise_std = {_num(scenario.perception.speed_noise_std)}",
            ]
        )

    return "\n".join(lines) + "\n"

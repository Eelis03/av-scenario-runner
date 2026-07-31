"""Scenario schema, typed model, validation, and serialisation.

This layer performs no I/O beyond reading a scenario file and does no
simulation. Everything downstream depends on it; it depends on nothing.
"""

from __future__ import annotations

from scenario_runner.model.errors import ScenarioError
from scenario_runner.model.geometry import cartesian_to_frenet, frenet_to_cartesian, heading_at
from scenario_runner.model.parser import load_scenario, load_suite, parse_scenario
from scenario_runner.model.scenario import (
    CURRENT_FORMAT_VERSION,
    DEFAULT_SHAPE,
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
from scenario_runner.model.serialise import to_toml

__all__ = [
    "CURRENT_FORMAT_VERSION",
    "DEFAULT_SHAPE",
    "KNOWN_CONTROLLERS",
    "SUPPORTED_FORMAT_VERSIONS",
    "ActorSpec",
    "Assertion",
    "EgoSpec",
    "GoalReached",
    "IdmParams",
    "LaneChangeSpec",
    "LateralAcceleration",
    "LateralParams",
    "LongitudinalAcceleration",
    "MinDistance",
    "MinTimeToCollision",
    "NoCollision",
    "PerceptionSpec",
    "ReactiveBehaviour",
    "RoadSpec",
    "Scenario",
    "ScenarioError",
    "ScheduleEntry",
    "ScriptedBehaviour",
    "SpeedLimit",
    "TerminationSpec",
    "VehicleShape",
    "cartesian_to_frenet",
    "frenet_to_cartesian",
    "heading_at",
    "load_scenario",
    "load_suite",
    "parse_scenario",
    "to_toml",
]

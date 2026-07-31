"""Simulation and suite execution.

This layer turns a validated scenario into a structured trace and a structured
result. It renders nothing and writes no files.
"""

from __future__ import annotations

from scenario_runner.pipeline.actors import (
    ActorState,
    actor_pose,
    advance_actor,
    initial_actor_state,
    lateral_profile,
    scheduled_acceleration,
)
from scenario_runner.pipeline.controllers import (
    CONTROLLERS,
    ConstantSpeedController,
    ControlCommand,
    Controller,
    IdmLaneKeepController,
    Observation,
    build_controller,
)
from scenario_runner.pipeline.dynamics import (
    BicycleState,
    bicycle_step,
    idm_acceleration,
    lateral_acceleration,
    wrap_angle,
)
from scenario_runner.pipeline.runner import (
    ScenarioResult,
    SuiteResult,
    run_scenario,
    run_scenario_file,
    run_suite,
)
from scenario_runner.pipeline.simulator import simulate
from scenario_runner.pipeline.trace import BodyTrace, Trace, TraceRecorder

__all__ = [
    "CONTROLLERS",
    "ActorState",
    "BicycleState",
    "BodyTrace",
    "ConstantSpeedController",
    "ControlCommand",
    "Controller",
    "IdmLaneKeepController",
    "Observation",
    "ScenarioResult",
    "SuiteResult",
    "Trace",
    "TraceRecorder",
    "actor_pose",
    "advance_actor",
    "bicycle_step",
    "build_controller",
    "idm_acceleration",
    "initial_actor_state",
    "lateral_acceleration",
    "lateral_profile",
    "run_scenario",
    "run_scenario_file",
    "run_suite",
    "scheduled_acceleration",
    "simulate",
    "wrap_angle",
]

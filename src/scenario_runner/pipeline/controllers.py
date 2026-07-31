"""Reference controllers for the vehicle under test.

Two are provided. ``idm_lane_keep`` combines Intelligent Driver Model car
following with pure pursuit lane keeping (Coulter, 1992) and is the controller
a scenario is normally written against. ``constant_speed`` holds its desired
speed and never reacts to anything ahead; it exists so that the suite contains
scenarios which are supposed to fail, and so that a failure is demonstrably
detected rather than assumed.

The model layer validates a controller name against
``scenario_runner.model.KNOWN_CONTROLLERS``. This module registers exactly
those names; ``tests/test_model.py`` pins the two sets equal.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Final, Protocol

from scenario_runner.model import EgoSpec, RoadSpec, frenet_to_cartesian
from scenario_runner.pipeline.dynamics import BicycleState, idm_acceleration, wrap_angle

__all__ = ["CONTROLLERS", "ControlCommand", "Controller", "Observation", "build_controller"]


@dataclass(frozen=True, slots=True)
class Observation:
    """Everything a controller is allowed to see at one step."""

    time: float
    dt: float
    road: RoadSpec
    state: BicycleState
    s: float
    d: float
    target_offset: float
    speed_limit: float
    steer: float
    leader_gap: float | None
    leader_speed: float | None


@dataclass(frozen=True, slots=True)
class ControlCommand:
    """Longitudinal acceleration and front wheel steering angle."""

    accel: float
    steer: float


class Controller(Protocol):
    """A closed loop policy for the vehicle under test."""

    def control(self, observation: Observation) -> ControlCommand:
        """Return the command to hold over the next step."""


def _clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def _pure_pursuit(spec: EgoSpec, observation: Observation) -> float:
    params = spec.lateral
    lookahead = _clamp(
        params.lookahead_gain * observation.state.speed + params.lookahead_min,
        params.lookahead_min,
        params.lookahead_max,
    )
    target_x, target_y, _ = frenet_to_cartesian(
        observation.road, observation.s + lookahead, observation.target_offset
    )
    bearing = math.atan2(target_y - observation.state.y, target_x - observation.state.x)
    alpha = wrap_angle(bearing - observation.state.yaw)
    steer = math.atan2(2.0 * spec.shape.wheelbase * math.sin(alpha), lookahead)
    slew = params.max_steer_rate * observation.dt
    steer = _clamp(steer, observation.steer - slew, observation.steer + slew)
    return _clamp(steer, -params.max_steer, params.max_steer)


@dataclass(frozen=True, slots=True)
class IdmLaneKeepController:
    """IDM car following with pure pursuit lane keeping."""

    spec: EgoSpec

    def control(self, observation: Observation) -> ControlCommand:
        """Return an IDM acceleration and a pure pursuit steering angle."""
        desired = min(self.spec.longitudinal.desired_speed, observation.speed_limit)
        accel = idm_acceleration(
            self.spec.longitudinal,
            observation.state.speed,
            desired,
            observation.leader_gap,
            observation.leader_speed,
        )
        return ControlCommand(accel=accel, steer=_pure_pursuit(self.spec, observation))


@dataclass(frozen=True, slots=True)
class ConstantSpeedController:
    """Holds the desired speed and ignores every vehicle ahead.

    Deliberately unsafe. It is the counterexample controller that lets the
    suite show a real failure with real evidence.
    """

    spec: EgoSpec
    gain: float = 0.6

    def control(self, observation: Observation) -> ControlCommand:
        """Return a proportional speed hold and a pure pursuit steering angle."""
        error = self.spec.longitudinal.desired_speed - observation.state.speed
        accel = _clamp(
            self.gain * error,
            -self.spec.longitudinal.max_decel,
            self.spec.longitudinal.max_accel,
        )
        return ControlCommand(accel=accel, steer=_pure_pursuit(self.spec, observation))


CONTROLLERS: Final[Mapping[str, Callable[[EgoSpec], Controller]]] = {
    "idm_lane_keep": IdmLaneKeepController,
    "constant_speed": ConstantSpeedController,
}


def build_controller(spec: EgoSpec) -> Controller:
    """Instantiate the controller named by ``spec``."""
    factory = CONTROLLERS.get(spec.controller)
    if factory is None:  # pragma: no cover - unreachable while the model validates the name
        raise KeyError(f"no controller registered under {spec.controller!r}")
    return factory(spec)

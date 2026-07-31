"""Actor state and advancement in road coordinates.

Actors move along the road reference line, so their motion is described by arc
length, speed, and a lateral offset profile rather than by a steering angle.
This keeps a scripted actor exactly reproducible: its trajectory is a closed
form function of time, not the output of a controller that might be perturbed
by the vehicle it is meant to provoke.

Longitudinal integration is exact for the piecewise constant acceleration the
schedule prescribes, including the instant a braking actor reaches standstill,
so an actor never reverses through a rounding error.

Lateral offset during a lane change follows the smoothstep polynomial
``3u^2 - 2u^3``, which starts and ends with zero lateral rate and therefore does
not inject a step change in the actor heading.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from scenario_runner.model import (
    ActorSpec,
    RoadSpec,
    ScriptedBehaviour,
    VehicleShape,
    frenet_to_cartesian,
    heading_at,
)

__all__ = [
    "ActorState",
    "actor_pose",
    "advance_actor",
    "initial_actor_state",
    "lateral_profile",
    "scheduled_acceleration",
]


@dataclass(frozen=True, slots=True)
class ActorState:
    """Road frame state of one actor at one instant."""

    identifier: str
    shape: VehicleShape
    s: float
    d: float
    speed: float
    lateral_rate: float


def scheduled_acceleration(behaviour: ScriptedBehaviour, time: float) -> float:
    """Acceleration prescribed by the schedule at ``time``, held piecewise constant."""
    accel = 0.0
    for entry in behaviour.schedule:
        if time + 1e-12 < entry.time:
            break
        accel = entry.accel
    return accel


def lateral_profile(spec: ActorSpec, road: RoadSpec, time: float) -> tuple[float, float]:
    """Lateral offset and lateral rate of ``spec`` at ``time``."""
    start_offset = road.lane_offset(spec.lane)
    behaviour = spec.behaviour
    if not isinstance(behaviour, ScriptedBehaviour) or behaviour.lane_change is None:
        return start_offset, 0.0
    change = behaviour.lane_change
    end_offset = road.lane_offset(change.target_lane)
    span = end_offset - start_offset
    progress = (time - change.start_time) / change.duration
    if progress <= 0.0:
        return start_offset, 0.0
    if progress >= 1.0:
        return end_offset, 0.0
    shaped = progress * progress * (3.0 - 2.0 * progress)
    rate = span * 6.0 * progress * (1.0 - progress) / change.duration
    return start_offset + span * shaped, rate


def initial_actor_state(spec: ActorSpec, road: RoadSpec) -> ActorState:
    """Build the state of ``spec`` at time zero."""
    offset, rate = lateral_profile(spec, road, 0.0)
    return ActorState(
        identifier=spec.identifier,
        shape=spec.shape,
        s=spec.s,
        d=offset,
        speed=spec.speed,
        lateral_rate=rate,
    )


def advance_actor(
    state: ActorState, spec: ActorSpec, road: RoadSpec, accel: float, time: float, dt: float
) -> ActorState:
    """Advance ``state`` by ``dt`` under constant ``accel``, then reapply the lateral profile."""
    if accel < 0.0 and state.speed + accel * dt < 0.0:
        stop_time = -state.speed / accel
        travelled = state.speed * stop_time + 0.5 * accel * stop_time * stop_time
        speed = 0.0
    else:
        travelled = state.speed * dt + 0.5 * accel * dt * dt
        speed = max(0.0, state.speed + accel * dt)
    offset, rate = lateral_profile(spec, road, time + dt)
    return ActorState(
        identifier=state.identifier,
        shape=state.shape,
        s=state.s + travelled,
        d=offset,
        speed=speed,
        lateral_rate=rate,
    )


def actor_pose(state: ActorState, road: RoadSpec) -> tuple[float, float, float, float]:
    """Return ``(x, y, yaw, speed)`` in the Cartesian frame for one actor state."""
    x, y, _ = frenet_to_cartesian(road, state.s, state.d)
    slip = math.atan2(state.lateral_rate, max(state.speed, 1e-6))
    yaw = heading_at(road, state.s) + slip
    speed = math.hypot(state.speed, state.lateral_rate)
    return x, y, yaw, speed

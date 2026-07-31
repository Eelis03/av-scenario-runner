"""Deterministic fixed step simulation of one scenario.

The loop is a pure function of the scenario document and the seed. Nothing is
read from the clock, from the environment, or from an iterative solver, and the
only source of randomness is the perception noise the scenario declares, drawn
from a generator seeded by the scenario. With the default noise free perception
model the seed has no effect at all and two runs agree bit for bit.

Each step records state first and integrates second, so sample ``k`` holds the
state at ``t = k dt`` together with the command that was applied over
``[k dt, (k + 1) dt)``. Termination is checked on the recorded sample, which
means a collision appears in the trace rather than being stepped past.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from scenario_runner.model import (
    PerceptionSpec,
    ReactiveBehaviour,
    Scenario,
    ScriptedBehaviour,
    VehicleShape,
    cartesian_to_frenet,
    frenet_to_cartesian,
)
from scenario_runner.pipeline.actors import (
    ActorState,
    actor_pose,
    advance_actor,
    initial_actor_state,
    scheduled_acceleration,
)
from scenario_runner.pipeline.controllers import Observation, build_controller
from scenario_runner.pipeline.dynamics import (
    BicycleState,
    bicycle_step,
    idm_acceleration,
    lateral_acceleration,
)
from scenario_runner.pipeline.trace import Trace, TraceRecorder

__all__ = ["simulate"]

_LATERAL_MARGIN = 0.2


@dataclass(frozen=True, slots=True)
class _Body:
    """The minimum a leader search needs to know about a vehicle."""

    identifier: str
    s: float
    d: float
    speed: float
    shape: VehicleShape


def _leader(follower: _Body, candidates: tuple[_Body, ...]) -> tuple[float, float] | None:
    """Return ``(gap, speed)`` of the nearest vehicle ahead of ``follower`` in its lane."""
    best: tuple[float, float] | None = None
    for other in candidates:
        if other.identifier == follower.identifier:
            continue
        band = 0.5 * (follower.shape.width + other.shape.width) + _LATERAL_MARGIN
        if abs(other.d - follower.d) >= band:
            continue
        gap = (other.s - follower.s) - follower.shape.front_offset + other.shape.rear_offset
        if other.s <= follower.s:
            continue
        if best is None or gap < best[0]:
            best = (gap, other.speed)
    return best


def _clearance(
    ego_pose: tuple[float, float, float, float],
    ego_shape: VehicleShape,
    actor_pose_value: tuple[float, float, float, float],
    actor_shape: VehicleShape,
) -> float:
    ego_centres = _disc_centres(ego_pose, ego_shape)
    actor_centres = _disc_centres(actor_pose_value, actor_shape)
    radii = ego_shape.disc_radius + actor_shape.disc_radius
    return min(
        math.hypot(a[0] - b[0], a[1] - b[1]) - radii
        for a in ego_centres
        for b in actor_centres
    )


def _disc_centres(
    pose: tuple[float, float, float, float], shape: VehicleShape
) -> list[tuple[float, float]]:
    x, y, yaw, _ = pose
    cos, sin = math.cos(yaw), math.sin(yaw)
    return [(x + offset * cos, y + offset * sin) for offset in shape.disc_offsets]


def simulate(scenario: Scenario, *, seed: int | None = None, max_steps: int | None = None) -> Trace:
    """Run ``scenario`` and return its trace."""
    road = scenario.road
    effective_seed = scenario.seed if seed is None else seed
    rng = np.random.default_rng(effective_seed)
    perception = scenario.perception

    controller = build_controller(scenario.ego)
    ego_shape = scenario.ego.shape
    target_offset = road.lane_offset(scenario.ego.target_lane)
    start_x, start_y, start_yaw = frenet_to_cartesian(
        road, scenario.ego.s, road.lane_offset(scenario.ego.lane)
    )
    state = BicycleState(x=start_x, y=start_y, yaw=start_yaw, speed=scenario.ego.speed)
    steer = 0.0

    actors: list[ActorState] = [initial_actor_state(spec, road) for spec in scenario.actors]
    recorder = TraceRecorder(scenario.name, scenario.dt, effective_seed, road)

    total_steps = scenario.max_steps if max_steps is None else min(scenario.max_steps, max_steps)
    dt = scenario.dt
    time = 0.0
    collided = False
    goal_time: float | None = None
    terminated = "max_time"

    for step in range(total_steps + 1):
        ego_s, ego_d = cartesian_to_frenet(road, state.x, state.y)
        ego_pose = (state.x, state.y, state.yaw, state.speed)
        poses = {actor.identifier: actor_pose(actor, road) for actor in actors}

        for actor in actors:
            if _clearance(ego_pose, ego_shape, poses[actor.identifier], actor.shape) <= 0.0:
                collided = True

        goal = scenario.termination.goal_s
        if goal_time is None and goal is not None and ego_s >= goal:
            goal_time = time

        bodies = tuple(
            _Body(actor.identifier, actor.s, actor.d, actor.speed, actor.shape) for actor in actors
        )
        ego_body = _Body("ego", ego_s, ego_d, state.speed, ego_shape)
        measured = _leader(ego_body, bodies)
        leader_gap, leader_speed = _measure(measured, perception, rng)

        command = controller.control(
            Observation(
                time=time,
                dt=dt,
                road=road,
                state=state,
                s=ego_s,
                d=ego_d,
                target_offset=target_offset,
                speed_limit=road.speed_limit,
                steer=steer,
                leader_gap=leader_gap,
                leader_speed=leader_speed,
            )
        )
        accel = command.accel
        if state.speed + accel * dt < 0.0:
            accel = -state.speed / dt
        lateral = lateral_acceleration(state.speed, command.steer, ego_shape.wheelbase)

        recorder.record_ego(time, ego_pose, ego_s, ego_d, accel, lateral, command.steer)
        for actor in actors:
            recorder.record_actor(actor.identifier, actor.shape, poses[actor.identifier])

        if collided and scenario.termination.stop_on_collision:
            terminated = "collision"
            break
        if goal_time is not None and scenario.termination.stop_at_goal:
            terminated = "goal"
            break
        if step == total_steps:
            terminated = "max_time" if max_steps is None else "step_limit"
            break

        state = bicycle_step(state, accel, command.steer, ego_shape.wheelbase, dt)
        steer = command.steer
        actors = _advance_actors(scenario, actors, ego_body, time, dt)
        time += dt

    return recorder.finish(ego_shape, terminated, collided, goal_time)


def _measure(
    measured: tuple[float, float] | None,
    perception: PerceptionSpec,
    rng: np.random.Generator,
) -> tuple[float | None, float | None]:
    if measured is None:
        return None, None
    gap, speed = measured
    if perception.range_noise_std > 0.0:
        gap += float(rng.normal(0.0, perception.range_noise_std))
    if perception.speed_noise_std > 0.0:
        speed += float(rng.normal(0.0, perception.speed_noise_std))
    return gap, speed


def _advance_actors(
    scenario: Scenario,
    actors: list[ActorState],
    ego_body: _Body,
    time: float,
    dt: float,
) -> list[ActorState]:
    bodies = tuple(
        _Body(actor.identifier, actor.s, actor.d, actor.speed, actor.shape) for actor in actors
    )
    candidates = (*bodies, ego_body)
    advanced: list[ActorState] = []
    for actor, spec in zip(actors, scenario.actors, strict=True):
        behaviour = spec.behaviour
        if isinstance(behaviour, ScriptedBehaviour):
            accel = scheduled_acceleration(behaviour, time)
        else:
            assert isinstance(behaviour, ReactiveBehaviour)
            follower = _Body(actor.identifier, actor.s, actor.d, actor.speed, actor.shape)
            measured = _leader(follower, candidates)
            gap = None if measured is None else measured[0]
            speed = None if measured is None else measured[1]
            accel = idm_acceleration(
                behaviour.idm, actor.speed, behaviour.idm.desired_speed, gap, speed
            )
        advanced.append(advance_actor(actor, spec, scenario.road, accel, time, dt))
    return advanced

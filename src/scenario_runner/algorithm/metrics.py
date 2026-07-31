"""Safety metrics computed from a run.

Footprints are covered by equal discs along the vehicle axis, the standard
conservative approximation used for fast collision queries (Ziegler and Stiller,
2010). Clearance between two vehicles is the smallest distance between any pair
of their discs, less the two radii, so it is zero at contact and negative when
the disc covers overlap.

Time to collision follows the constant velocity definition of Hayward (1972),
generalised to two dimensions so that a lateral encounter such as a cut in is
scored rather than ignored. For every disc pair the smallest non-negative root
of ``|p + v t| = R`` is taken, and the run value is the smallest root over all
pairs. Vehicles whose relative motion never closes to contact score infinity.
"""

from __future__ import annotations

from typing import Final

import numpy as np

from scenario_runner.algorithm.trace_view import BodyView, FloatArray, TraceView
from scenario_runner.model import VehicleShape

__all__ = [
    "clearance_series",
    "disc_centres",
    "footprint_corners",
    "nearest_actor",
    "time_to_collision_series",
]

_SPEED_EPSILON: Final[float] = 1e-12


def disc_centres(body: BodyView) -> FloatArray:
    """Return the disc centres of ``body`` at every step, shaped ``(steps, discs, 2)``."""
    offsets = np.asarray(body.shape.disc_offsets, dtype=np.float64)
    cos = np.cos(body.yaw)[:, None]
    sin = np.sin(body.yaw)[:, None]
    x = body.x[:, None] + offsets[None, :] * cos
    y = body.y[:, None] + offsets[None, :] * sin
    return np.stack((x, y), axis=-1)


def _velocity(body: BodyView) -> FloatArray:
    return np.stack((body.speed * np.cos(body.yaw), body.speed * np.sin(body.yaw)), axis=-1)


def _pair_clearance(ego: BodyView, actor: BodyView) -> FloatArray:
    ego_discs = disc_centres(ego)[:, :, None, :]
    actor_discs = disc_centres(actor)[:, None, :, :]
    separation = np.linalg.norm(actor_discs - ego_discs, axis=-1)
    radii = ego.shape.disc_radius + actor.shape.disc_radius
    return np.min(separation - radii, axis=(1, 2)).astype(np.float64)


def clearance_series(trace: TraceView) -> FloatArray:
    """Clearance in metres from the ego footprint to the nearest actor footprint."""
    if not trace.actors:
        return np.full(trace.time.shape, np.inf, dtype=np.float64)
    per_actor = np.stack([_pair_clearance(trace.ego, actor) for actor in trace.actors], axis=0)
    return np.min(per_actor, axis=0).astype(np.float64)


def _pair_time_to_collision(ego: BodyView, actor: BodyView) -> FloatArray:
    relative_position = disc_centres(actor)[:, None, :, :] - disc_centres(ego)[:, :, None, :]
    relative_velocity = (_velocity(actor) - _velocity(ego))[:, None, None, :]
    radii = ego.shape.disc_radius + actor.shape.disc_radius

    quad = np.sum(relative_velocity * relative_velocity, axis=-1)
    linear = 2.0 * np.sum(relative_position * relative_velocity, axis=-1)
    constant = np.sum(relative_position * relative_position, axis=-1) - radii * radii

    discriminant = linear * linear - 4.0 * quad * constant
    root = np.sqrt(np.maximum(discriminant, 0.0))
    with np.errstate(divide="ignore", invalid="ignore"):
        smallest = (-linear - root) / (2.0 * quad)
    closing = (quad > _SPEED_EPSILON) & (discriminant >= 0.0) & (smallest >= 0.0)
    times = np.where(closing, smallest, np.inf)
    times = np.where(constant <= 0.0, 0.0, times)
    return np.min(times, axis=(1, 2)).astype(np.float64)


def time_to_collision_series(trace: TraceView) -> FloatArray:
    """Time to collision in seconds between the ego and the nearest threatening actor."""
    if not trace.actors:
        return np.full(trace.time.shape, np.inf, dtype=np.float64)
    per_actor = np.stack(
        [_pair_time_to_collision(trace.ego, actor) for actor in trace.actors], axis=0
    )
    return np.min(per_actor, axis=0).astype(np.float64)


def nearest_actor(trace: TraceView, step: int) -> str | None:
    """Identifier of the actor closest to the ego at ``step``, or ``None`` with no actors."""
    if not trace.actors:
        return None
    distances = [float(_pair_clearance(trace.ego, actor)[step]) for actor in trace.actors]
    return trace.actors[int(np.argmin(distances))].identifier


def footprint_corners(
    x: float, y: float, yaw: float, shape: VehicleShape
) -> list[tuple[float, float]]:
    """Return the four footprint corners, used by the report figures and the trace export."""
    cos, sin = np.cos(yaw), np.sin(yaw)
    half_width = shape.width / 2.0
    local = (
        (shape.rear_offset, -half_width),
        (shape.front_offset, -half_width),
        (shape.front_offset, half_width),
        (shape.rear_offset, half_width),
    )
    return [
        (float(x + along * cos - across * sin), float(y + along * sin + across * cos))
        for along, across in local
    ]

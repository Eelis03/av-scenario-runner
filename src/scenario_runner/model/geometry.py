"""Road frame conversions for a straight or constant curvature road.

Coordinates follow the usual road convention: ``s`` is arc length along the
lane 0 reference line, ``d`` is lateral offset with positive to the left, and
the Cartesian frame has the reference line starting at the origin heading along
positive ``x``.

For curvature ``k = 1 / R`` (positive turning left, centre of curvature at
``(0, R)``) a point at ``(s, d)`` is::

    theta = s * k
    u     = R - d
    x     = u * sin(theta)
    y     = R - u * cos(theta)

which reduces to ``(s, d)`` as ``k`` tends to zero.
"""

from __future__ import annotations

import math

from scenario_runner.model.scenario import RoadSpec

__all__ = ["cartesian_to_frenet", "frenet_to_cartesian", "heading_at"]


def frenet_to_cartesian(road: RoadSpec, s: float, d: float) -> tuple[float, float, float]:
    """Return ``(x, y, heading)`` for the road point at arc length ``s``, offset ``d``."""
    curvature = road.curvature
    if curvature == 0.0:
        return s, d, 0.0
    radius = 1.0 / curvature
    theta = s * curvature
    effective = radius - d
    x = effective * math.sin(theta)
    y = radius - effective * math.cos(theta)
    return x, y, theta


def cartesian_to_frenet(road: RoadSpec, x: float, y: float) -> tuple[float, float]:
    """Return ``(s, d)`` for a Cartesian point. Exact inverse of ``frenet_to_cartesian``."""
    curvature = road.curvature
    if curvature == 0.0:
        return x, y
    radius = 1.0 / curvature
    sign = math.copysign(1.0, radius)
    distance = math.hypot(x, radius - y)
    theta = math.atan2(sign * x, sign * (radius - y))
    return theta * radius, radius - sign * distance


def heading_at(road: RoadSpec, s: float) -> float:
    """Heading of the road reference line at arc length ``s``."""
    return s * road.curvature

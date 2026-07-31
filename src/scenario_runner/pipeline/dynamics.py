"""Vehicle motion models used by the simulator.

Lateral and longitudinal motion of the ego vehicle follows the kinematic
bicycle model with the reference point at the rear axle (Kong, Pfeiffer,
Schildbach and Borrelli, 2015)::

    x'   = v cos(psi)
    y'   = v sin(psi)
    psi' = v tan(delta) / L
    v'   = a

integrated with a fixed step classical Runge-Kutta scheme. The step is fixed
rather than adaptive so that a run is a pure function of the scenario and the
seed, which is what makes a regression baseline meaningful at all.

Car following uses the Intelligent Driver Model (Treiber, Hennecke and Helbing,
2000), which is collision free for a single leader under its own parameters and
therefore gives a reference controller that passes a reasonable scenario
without being tuned per scenario.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from scenario_runner.model import IdmParams

__all__ = [
    "BicycleState",
    "bicycle_step",
    "idm_acceleration",
    "lateral_acceleration",
    "wrap_angle",
]


@dataclass(frozen=True, slots=True)
class BicycleState:
    """Rear axle pose and forward speed of a kinematic bicycle."""

    x: float
    y: float
    yaw: float
    speed: float


def wrap_angle(angle: float) -> float:
    """Wrap ``angle`` to the interval ``(-pi, pi]``."""
    return math.remainder(angle, 2.0 * math.pi)


def _derivative(
    state: BicycleState, accel: float, steer: float, wheelbase: float
) -> tuple[float, float, float, float]:
    return (
        state.speed * math.cos(state.yaw),
        state.speed * math.sin(state.yaw),
        state.speed * math.tan(steer) / wheelbase,
        accel,
    )


def _shift(
    state: BicycleState, slope: tuple[float, float, float, float], step: float
) -> BicycleState:
    return BicycleState(
        x=state.x + step * slope[0],
        y=state.y + step * slope[1],
        yaw=state.yaw + step * slope[2],
        speed=state.speed + step * slope[3],
    )


def bicycle_step(
    state: BicycleState, accel: float, steer: float, wheelbase: float, dt: float
) -> BicycleState:
    """Advance ``state`` by ``dt`` under constant ``accel`` and ``steer``."""
    k1 = _derivative(state, accel, steer, wheelbase)
    k2 = _derivative(_shift(state, k1, dt / 2.0), accel, steer, wheelbase)
    k3 = _derivative(_shift(state, k2, dt / 2.0), accel, steer, wheelbase)
    k4 = _derivative(_shift(state, k3, dt), accel, steer, wheelbase)
    weighted = tuple(
        (a + 2.0 * b + 2.0 * c + d) / 6.0 for a, b, c, d in zip(k1, k2, k3, k4, strict=True)
    )
    advanced = _shift(state, (weighted[0], weighted[1], weighted[2], weighted[3]), dt)
    return BicycleState(
        x=advanced.x,
        y=advanced.y,
        yaw=wrap_angle(advanced.yaw),
        speed=max(0.0, advanced.speed),
    )


def lateral_acceleration(speed: float, steer: float, wheelbase: float) -> float:
    """Lateral acceleration of a bicycle at ``speed`` with steering angle ``steer``."""
    return speed * speed * math.tan(steer) / wheelbase


def idm_acceleration(
    params: IdmParams,
    speed: float,
    desired_speed: float,
    gap: float | None,
    leader_speed: float | None,
) -> float:
    """Intelligent Driver Model acceleration in metres per second squared.

    ``gap`` is bumper to bumper and ``leader_speed`` is the speed of the vehicle
    ahead; both are ``None`` when the road ahead is clear.
    """
    free = 1.0 - math.pow(speed / desired_speed, params.exponent)
    interaction = 0.0
    if gap is not None and leader_speed is not None:
        approach = speed - leader_speed
        coupling = 2.0 * math.sqrt(params.max_accel * params.comfort_decel)
        dynamic = speed * params.time_gap + speed * approach / coupling
        target_gap = params.min_gap + max(0.0, dynamic)
        ratio = target_gap / max(gap, 0.05)
        interaction = ratio * ratio
    accel = params.max_accel * (free - interaction)
    return min(params.max_accel, max(-params.max_decel, accel))

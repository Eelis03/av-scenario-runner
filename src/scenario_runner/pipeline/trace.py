"""The structured trace produced by one simulated run.

A trace holds only sampled state, never derived safety metrics. Clearance, time
to collision, and every assertion verdict are computed from the trace by the
algorithm layer, so a stored trace can be rescored under different assertions
without rerunning the simulation.

``Trace`` and ``BodyTrace`` satisfy the ``TraceView`` and ``BodyView``
protocols of the algorithm layer structurally, which is how the scoring code
reads a run without depending on the simulator.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from scenario_runner.algorithm import FloatArray
from scenario_runner.model import RoadSpec, VehicleShape

__all__ = ["BodyTrace", "Trace", "TraceRecorder"]


@dataclass(frozen=True, slots=True)
class BodyTrace:
    """Sampled pose and speed of one vehicle over a run."""

    identifier: str
    shape: VehicleShape
    x: FloatArray
    y: FloatArray
    yaw: FloatArray
    speed: FloatArray


@dataclass(frozen=True, slots=True)
class Trace:
    """One complete run of one scenario."""

    scenario_name: str
    dt: float
    seed: int
    road: RoadSpec
    time: FloatArray
    ego: BodyTrace
    actors: tuple[BodyTrace, ...]
    ego_s: FloatArray
    ego_d: FloatArray
    ego_accel: FloatArray
    ego_lateral_accel: FloatArray
    ego_steer: FloatArray
    terminated: str
    collided: bool
    goal_time: float | None

    @property
    def steps(self) -> int:
        """Number of recorded samples."""
        return int(self.time.size)

    @property
    def duration(self) -> float:
        """Simulated time span in seconds."""
        return float(self.time[-1]) if self.time.size else 0.0


class TraceRecorder:
    """Accumulates per-step samples and freezes them into a ``Trace``."""

    def __init__(self, scenario_name: str, dt: float, seed: int, road: RoadSpec) -> None:
        self._scenario_name = scenario_name
        self._dt = dt
        self._seed = seed
        self._road = road
        self._time: list[float] = []
        self._ego: list[tuple[float, float, float, float]] = []
        self._ego_extra: list[tuple[float, float, float, float, float]] = []
        self._actors: dict[str, list[tuple[float, float, float, float]]] = {}
        self._order: list[str] = []
        self._shapes: dict[str, VehicleShape] = {}

    def record_ego(
        self,
        time: float,
        pose: tuple[float, float, float, float],
        s: float,
        d: float,
        accel: float,
        lateral_accel: float,
        steer: float,
    ) -> None:
        """Append one ego sample."""
        self._time.append(time)
        self._ego.append(pose)
        self._ego_extra.append((s, d, accel, lateral_accel, steer))

    def record_actor(
        self, identifier: str, shape: VehicleShape, pose: tuple[float, float, float, float]
    ) -> None:
        """Append one actor sample."""
        if identifier not in self._actors:
            self._actors[identifier] = []
            self._order.append(identifier)
            self._shapes[identifier] = shape
        self._actors[identifier].append(pose)

    def finish(
        self, ego_shape: VehicleShape, terminated: str, collided: bool, goal_time: float | None
    ) -> Trace:
        """Freeze the recorded samples into an immutable trace."""
        ego = np.asarray(self._ego, dtype=np.float64).reshape(-1, 4)
        extra = np.asarray(self._ego_extra, dtype=np.float64).reshape(-1, 5)
        actors = tuple(
            BodyTrace(
                identifier=identifier,
                shape=self._shapes[identifier],
                **_columns(np.asarray(self._actors[identifier], dtype=np.float64).reshape(-1, 4)),
            )
            for identifier in self._order
        )
        return Trace(
            scenario_name=self._scenario_name,
            dt=self._dt,
            seed=self._seed,
            road=self._road,
            time=np.asarray(self._time, dtype=np.float64),
            ego=BodyTrace(identifier="ego", shape=ego_shape, **_columns(ego)),
            actors=actors,
            ego_s=np.ascontiguousarray(extra[:, 0]),
            ego_d=np.ascontiguousarray(extra[:, 1]),
            ego_accel=np.ascontiguousarray(extra[:, 2]),
            ego_lateral_accel=np.ascontiguousarray(extra[:, 3]),
            ego_steer=np.ascontiguousarray(extra[:, 4]),
            terminated=terminated,
            collided=collided,
            goal_time=goal_time,
        )


def _columns(block: FloatArray) -> dict[str, FloatArray]:
    return {
        "x": np.ascontiguousarray(block[:, 0]),
        "y": np.ascontiguousarray(block[:, 1]),
        "yaw": np.ascontiguousarray(block[:, 2]),
        "speed": np.ascontiguousarray(block[:, 3]),
    }

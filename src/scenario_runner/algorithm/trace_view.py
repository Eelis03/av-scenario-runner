"""The read-only view of a run that the algorithm layer requires.

The simulator lives in the pipeline layer, one level above this one, so the
algorithm layer cannot import its trace type without inverting the dependency
order. Instead it declares, structurally, the members it reads. Any object with
these members can be scored, which also makes hand constructed traces in the
unit tests first class rather than a testing special case.
"""

from __future__ import annotations

from typing import Protocol

import numpy as np
import numpy.typing as npt

from scenario_runner.model import VehicleShape

__all__ = ["BodyView", "FloatArray", "TraceView"]

FloatArray = npt.NDArray[np.float64]


class BodyView(Protocol):
    """One rigid body sampled at every step of a run."""

    @property
    def identifier(self) -> str:
        """Name of the body, unique within a run."""

    @property
    def shape(self) -> VehicleShape:
        """Footprint used for clearance and collision queries."""

    @property
    def x(self) -> FloatArray:
        """Reference point abscissa in metres, one sample per step."""

    @property
    def y(self) -> FloatArray:
        """Reference point ordinate in metres, one sample per step."""

    @property
    def yaw(self) -> FloatArray:
        """Heading in radians, one sample per step."""

    @property
    def speed(self) -> FloatArray:
        """Forward speed in metres per second, one sample per step."""


class TraceView(Protocol):
    """One complete run of one scenario."""

    @property
    def scenario_name(self) -> str:
        """Name of the scenario that produced this run."""

    @property
    def dt(self) -> float:
        """Fixed integration step in seconds."""

    @property
    def time(self) -> FloatArray:
        """Sample times in seconds, strictly increasing, starting at zero."""

    @property
    def ego(self) -> BodyView:
        """The vehicle under test."""

    @property
    def actors(self) -> tuple[BodyView, ...]:
        """Every other vehicle in the run."""

    @property
    def ego_s(self) -> FloatArray:
        """Ego arc length along the road reference line, in metres."""

    @property
    def ego_accel(self) -> FloatArray:
        """Applied ego longitudinal acceleration in metres per second squared."""

    @property
    def ego_lateral_accel(self) -> FloatArray:
        """Ego lateral acceleration in metres per second squared, signed left positive."""

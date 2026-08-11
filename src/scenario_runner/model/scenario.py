"""Typed scenario model.

The dataclasses in this module are the whole contract between a scenario file
and everything downstream. They are frozen so a loaded scenario cannot be
mutated by a controller, an actor, or a report renderer.

Two format versions are recognised. Version ``1.0`` is the original document
shape. Version ``1.1`` adds the optional ``[perception]`` table. A ``1.0``
document loads without change and receives the default (noise free) perception
model; a ``1.0`` document that uses a ``1.1`` field is rejected by name.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import ClassVar, Final

__all__ = [
    "CURRENT_FORMAT_VERSION",
    "DEFAULT_SHAPE",
    "KNOWN_CONTROLLERS",
    "SUPPORTED_FORMAT_VERSIONS",
    "ActorSpec",
    "Assertion",
    "AssertionSpec",
    "EgoSpec",
    "GoalReached",
    "IdmParams",
    "LaneChangeSpec",
    "LateralAcceleration",
    "LateralParams",
    "LongitudinalAcceleration",
    "MinDistance",
    "MinTimeHeadway",
    "MinTimeToCollision",
    "NoCollision",
    "PerceptionSpec",
    "ReactiveBehaviour",
    "RoadSpec",
    "Scenario",
    "ScheduleEntry",
    "ScriptedBehaviour",
    "SpeedLimit",
    "TerminationSpec",
    "VehicleShape",
]

CURRENT_FORMAT_VERSION: Final[str] = "1.1"
SUPPORTED_FORMAT_VERSIONS: Final[tuple[str, ...]] = ("1.0", "1.1")

#: Controller identifiers the model accepts. The pipeline layer registers an
#: implementation for exactly these names; ``tests/test_model.py`` pins the
#: two sets equal so a name can never be accepted here and missing there.
KNOWN_CONTROLLERS: Final[frozenset[str]] = frozenset({"idm_lane_keep", "constant_speed"})


@dataclass(frozen=True, slots=True)
class VehicleShape:
    """Rectangular footprint measured from the rear axle reference point.

    The footprint spans ``[-rear_overhang, wheelbase + front_overhang]``
    longitudinally and ``[-width / 2, width / 2]`` laterally. Collision and
    clearance queries cover it with ``disc_count`` equal discs, following the
    circle-cover construction of Ziegler and Stiller (2010).
    """

    wheelbase: float = 2.8
    front_overhang: float = 0.9
    rear_overhang: float = 0.9
    width: float = 1.9
    disc_count: int = 3

    @property
    def length(self) -> float:
        """Total bumper to bumper length."""
        return self.rear_overhang + self.wheelbase + self.front_overhang

    @property
    def front_offset(self) -> float:
        """Longitudinal distance from the reference point to the front bumper."""
        return self.wheelbase + self.front_overhang

    @property
    def rear_offset(self) -> float:
        """Longitudinal distance from the reference point to the rear bumper.

        Negative, because the rear bumper is behind the reference point.
        """
        return -self.rear_overhang

    @property
    def disc_radius(self) -> float:
        """Radius of each covering disc."""
        segment = self.length / self.disc_count
        return 0.5 * math.hypot(segment, self.width)

    @property
    def disc_offsets(self) -> tuple[float, ...]:
        """Longitudinal offsets of the disc centres from the reference point."""
        segment = self.length / self.disc_count
        return tuple(self.rear_offset + (index + 0.5) * segment for index in range(self.disc_count))


DEFAULT_SHAPE: Final[VehicleShape] = VehicleShape()


@dataclass(frozen=True, slots=True)
class RoadSpec:
    """Road geometry: a straight or a constant curvature arc with parallel lanes.

    Lane 0 is the rightmost lane. Lane centres sit at lateral offset
    ``lane * lane_width``, with lateral offset increasing to the left.
    """

    kind: str = "straight"
    lanes: int = 1
    lane_width: float = 3.5
    length: float = 400.0
    speed_limit: float = 13.9
    radius: float = 0.0
    direction: str = "left"

    @property
    def curvature(self) -> float:
        """Signed curvature in 1/m. Positive turns left, zero for a straight road."""
        if self.kind == "straight":
            return 0.0
        sign = 1.0 if self.direction == "left" else -1.0
        return sign / self.radius

    def lane_offset(self, lane: int) -> float:
        """Lateral offset of the centre of ``lane``."""
        return lane * self.lane_width


@dataclass(frozen=True, slots=True)
class IdmParams:
    """Intelligent Driver Model parameters (Treiber, Hennecke and Helbing, 2000)."""

    desired_speed: float = 13.9
    time_gap: float = 1.5
    min_gap: float = 2.0
    max_accel: float = 1.4
    comfort_decel: float = 2.0
    max_decel: float = 8.0
    exponent: float = 4.0


@dataclass(frozen=True, slots=True)
class LateralParams:
    """Pure pursuit lane keeping parameters (Coulter, 1992)."""

    lookahead_gain: float = 0.7
    lookahead_min: float = 5.0
    lookahead_max: float = 30.0
    max_steer: float = 0.6
    max_steer_rate: float = 0.8


@dataclass(frozen=True, slots=True)
class PerceptionSpec:
    """Zero mean Gaussian noise added to the measurements the controller sees.

    Both standard deviations default to zero, which makes a run bit for bit
    reproducible without reference to the seed. A non-zero value makes the seed
    load bearing, which is what ``tests/test_pipeline.py`` exercises.
    """

    range_noise_std: float = 0.0
    speed_noise_std: float = 0.0

    @property
    def is_noiseless(self) -> bool:
        """True when no noise is injected at all."""
        return self.range_noise_std == 0.0 and self.speed_noise_std == 0.0


@dataclass(frozen=True, slots=True)
class EgoSpec:
    """Initial state and controller configuration of the vehicle under test."""

    lane: int = 0
    s: float = 0.0
    speed: float = 0.0
    controller: str = "idm_lane_keep"
    target_lane: int = 0
    longitudinal: IdmParams = field(default_factory=IdmParams)
    lateral: LateralParams = field(default_factory=LateralParams)
    shape: VehicleShape = DEFAULT_SHAPE


@dataclass(frozen=True, slots=True)
class ScheduleEntry:
    """One breakpoint of a piecewise constant acceleration schedule."""

    time: float
    accel: float


@dataclass(frozen=True, slots=True)
class LaneChangeSpec:
    """A single lateral manoeuvre with a smooth (smoothstep) offset profile."""

    target_lane: int
    start_time: float
    duration: float


@dataclass(frozen=True, slots=True)
class ScriptedBehaviour:
    """An actor that follows a fixed acceleration schedule and lane plan."""

    kind: ClassVar[str] = "scripted"
    schedule: tuple[ScheduleEntry, ...] = ()
    lane_change: LaneChangeSpec | None = None


@dataclass(frozen=True, slots=True)
class ReactiveBehaviour:
    """An actor that follows the nearest vehicle ahead in its lane using IDM."""

    kind: ClassVar[str] = "reactive"
    idm: IdmParams = field(default_factory=IdmParams)


@dataclass(frozen=True, slots=True)
class ActorSpec:
    """One non-ego vehicle, positioned in road (arc length, lane) coordinates."""

    identifier: str
    lane: int
    s: float
    speed: float
    behaviour: ScriptedBehaviour | ReactiveBehaviour
    shape: VehicleShape = DEFAULT_SHAPE


@dataclass(frozen=True, slots=True)
class TerminationSpec:
    """When the simulation stops."""

    max_time: float = 20.0
    goal_s: float | None = None
    stop_at_goal: bool = False
    stop_on_collision: bool = True


@dataclass(frozen=True, slots=True)
class NoCollision:
    """The ego footprint must never touch an actor footprint."""

    kind: ClassVar[str] = "no_collision"
    name: str = "no_collision"


@dataclass(frozen=True, slots=True)
class MinTimeToCollision:
    """Time to collision must stay at or above ``threshold`` seconds."""

    kind: ClassVar[str] = "min_time_to_collision"
    name: str = "min_time_to_collision"
    threshold: float = 1.5


@dataclass(frozen=True, slots=True)
class MinTimeHeadway:
    """Time headway to the vehicle ahead must stay at or above ``threshold`` seconds."""

    kind: ClassVar[str] = "min_time_headway"
    name: str = "min_time_headway"
    threshold: float = 1.0


@dataclass(frozen=True, slots=True)
class LongitudinalAcceleration:
    """Ego longitudinal acceleration must stay inside a comfort band."""

    kind: ClassVar[str] = "longitudinal_acceleration"
    name: str = "longitudinal_acceleration"
    minimum: float = -3.0
    maximum: float = 2.0


@dataclass(frozen=True, slots=True)
class LateralAcceleration:
    """Ego lateral acceleration magnitude must stay at or below ``limit``."""

    kind: ClassVar[str] = "lateral_acceleration"
    name: str = "lateral_acceleration"
    limit: float = 3.0


@dataclass(frozen=True, slots=True)
class SpeedLimit:
    """Ego speed must stay at or below ``limit`` plus ``tolerance``."""

    kind: ClassVar[str] = "speed_limit"
    name: str = "speed_limit"
    limit: float = 13.9
    tolerance: float = 0.0


@dataclass(frozen=True, slots=True)
class GoalReached:
    """Ego must pass arc length ``goal_s`` no later than ``time_budget``."""

    kind: ClassVar[str] = "goal_reached"
    name: str = "goal_reached"
    goal_s: float = 0.0
    time_budget: float = 0.0


@dataclass(frozen=True, slots=True)
class MinDistance:
    """Clearance to the nearest actor footprint must stay at or above ``threshold``."""

    kind: ClassVar[str] = "min_distance"
    name: str = "min_distance"
    threshold: float = 1.0


Assertion = (
    NoCollision
    | MinTimeToCollision
    | MinTimeHeadway
    | LongitudinalAcceleration
    | LateralAcceleration
    | SpeedLimit
    | GoalReached
    | MinDistance
)

#: Alias kept for readability at call sites that do not care which variant.
AssertionSpec = Assertion

ASSERTION_KINDS: Final[tuple[str, ...]] = (
    NoCollision.kind,
    MinTimeToCollision.kind,
    MinTimeHeadway.kind,
    LongitudinalAcceleration.kind,
    LateralAcceleration.kind,
    SpeedLimit.kind,
    GoalReached.kind,
    MinDistance.kind,
)


@dataclass(frozen=True, slots=True)
class Scenario:
    """A complete, validated scenario document."""

    name: str
    format_version: str = CURRENT_FORMAT_VERSION
    description: str = ""
    expected_outcome: str = "pass"
    dt: float = 0.05
    seed: int = 0
    road: RoadSpec = field(default_factory=RoadSpec)
    ego: EgoSpec = field(default_factory=EgoSpec)
    actors: tuple[ActorSpec, ...] = ()
    termination: TerminationSpec = field(default_factory=TerminationSpec)
    assertions: tuple[Assertion, ...] = ()
    perception: PerceptionSpec = field(default_factory=PerceptionSpec)
    # Provenance only. Excluded from equality so a parse, serialise, parse round
    # trip compares equal even though the second parse names a different source.
    source: str = field(default="<scenario>", compare=False)

    @property
    def max_steps(self) -> int:
        """Number of integration steps implied by ``max_time`` and ``dt``."""
        return round(self.termination.max_time / self.dt)

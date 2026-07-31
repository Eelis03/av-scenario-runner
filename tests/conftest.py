"""Shared fixtures and hand constructed traces.

The synthetic trace helpers exist so an assertion can be checked against a run
whose correct verdict is known by inspection rather than by rerunning the
simulator. If the assertion and the simulator were both wrong in the same way,
a test that only ever scores simulated runs would not notice.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest

from scenario_runner.algorithm import FloatArray
from scenario_runner.model import DEFAULT_SHAPE, VehicleShape

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIR = REPO_ROOT / "scenarios"
BASELINE_PATH = REPO_ROOT / "baselines" / "reference_suite.json"
FIXTURE_DIR = Path(__file__).parent / "fixtures"
MALFORMED_DIR = FIXTURE_DIR / "malformed"
EXAMPLES_DIR = REPO_ROOT / "examples"


@dataclass(frozen=True, slots=True)
class FakeBody:
    """A body whose pose over time is written down rather than simulated."""

    identifier: str
    shape: VehicleShape
    x: FloatArray
    y: FloatArray
    yaw: FloatArray
    speed: FloatArray


@dataclass(frozen=True, slots=True)
class FakeTrace:
    """A run whose every series is written down rather than simulated."""

    scenario_name: str
    dt: float
    time: FloatArray
    ego: FakeBody
    actors: tuple[FakeBody, ...]
    ego_s: FloatArray
    ego_accel: FloatArray
    ego_lateral_accel: FloatArray


def constant_body(
    identifier: str,
    steps: int,
    *,
    x0: float,
    speed: float,
    y: float = 0.0,
    dt: float = 0.1,
    shape: VehicleShape = DEFAULT_SHAPE,
) -> FakeBody:
    """A body driving straight along ``x`` at constant speed."""
    time = np.arange(steps, dtype=np.float64) * dt
    return FakeBody(
        identifier=identifier,
        shape=shape,
        x=x0 + speed * time,
        y=np.full(steps, y, dtype=np.float64),
        yaw=np.zeros(steps, dtype=np.float64),
        speed=np.full(steps, speed, dtype=np.float64),
    )


def build_trace(
    ego: FakeBody,
    actors: tuple[FakeBody, ...] = (),
    *,
    dt: float = 0.1,
    accel: FloatArray | None = None,
    lateral_accel: FloatArray | None = None,
    name: str = "synthetic",
) -> FakeTrace:
    """Assemble a synthetic trace from bodies and optional acceleration series."""
    steps = int(ego.x.size)
    time = np.arange(steps, dtype=np.float64) * dt
    return FakeTrace(
        scenario_name=name,
        dt=dt,
        time=time,
        ego=ego,
        actors=actors,
        ego_s=ego.x.copy(),
        ego_accel=np.zeros(steps, dtype=np.float64) if accel is None else accel,
        ego_lateral_accel=(
            np.zeros(steps, dtype=np.float64) if lateral_accel is None else lateral_accel
        ),
    )


@pytest.fixture(scope="session")
def scenario_paths() -> tuple[Path, ...]:
    """Every scenario shipped with the repository, in a stable order."""
    return tuple(sorted(SCENARIO_DIR.glob("*.toml")))


@pytest.fixture(scope="session")
def malformed_paths() -> tuple[Path, ...]:
    """Every malformed scenario fixture, in a stable order."""
    return tuple(sorted(MALFORMED_DIR.glob("*.toml")))



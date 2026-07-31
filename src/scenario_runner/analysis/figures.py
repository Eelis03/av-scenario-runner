"""Figures summarising one run.

The Agg canvas is constructed explicitly rather than through ``pyplot`` so that
rendering never depends on an interactive backend, a display, or global figure
state. Every function returns the path it wrote.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from matplotlib.axes import Axes
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

from scenario_runner.algorithm import (
    AssertionResult,
    clearance_series,
    footprint_corners,
    time_to_collision_series,
)
from scenario_runner.model import Scenario, frenet_to_cartesian
from scenario_runner.pipeline import BodyTrace, Trace

__all__ = ["plot_run"]

_SNAPSHOTS = 5


def _draw_road(axes: Axes, scenario: Scenario, trace: Trace) -> None:
    road = scenario.road
    span = np.linspace(0.0, float(np.max(trace.ego_s)) + 40.0, 400)
    for lane in range(road.lanes + 1):
        offset = road.lane_offset(lane) - road.lane_width / 2.0
        points = np.array(
            [frenet_to_cartesian(road, float(s), offset)[:2] for s in span], dtype=np.float64
        )
        style = "-" if lane in (0, road.lanes) else "--"
        axes.plot(points[:, 0], points[:, 1], style, color="0.75", linewidth=0.8, zorder=0)


def _draw_footprints(axes: Axes, trace: Trace) -> None:
    steps = trace.steps
    indices = np.linspace(0, steps - 1, min(_SNAPSHOTS, steps)).astype(int)
    for rank, index in enumerate(indices):
        alpha = 0.25 + 0.65 * rank / max(1, len(indices) - 1)
        _polygon(axes, trace.ego, index, "#1f4e79", alpha)
        for actor in trace.actors:
            _polygon(axes, actor, index, "#a13d2d", alpha)


def _polygon(axes: Axes, body: BodyTrace, index: int, colour: str, alpha: float) -> None:
    corners = footprint_corners(
        float(body.x[index]), float(body.y[index]), float(body.yaw[index]), body.shape
    )
    closed = [*corners, corners[0]]
    axes.plot(
        [point[0] for point in closed],
        [point[1] for point in closed],
        color=colour,
        alpha=alpha,
        linewidth=1.2,
    )


def plot_run(
    scenario: Scenario,
    trace: Trace,
    path: Path | str,
    assertions: tuple[AssertionResult, ...] = (),
) -> Path:
    """Write a three panel summary of ``trace`` to ``path`` and return the path."""
    figure = Figure(figsize=(11.0, 8.5), dpi=120)
    FigureCanvasAgg(figure)
    grid = figure.add_gridspec(3, 1, height_ratios=(1.2, 1.0, 1.0), hspace=0.38)

    top = figure.add_subplot(grid[0])
    _draw_road(top, scenario, trace)
    top.plot(trace.ego.x, trace.ego.y, color="#1f4e79", linewidth=1.6, label="ego")
    for actor in trace.actors:
        top.plot(actor.x, actor.y, linewidth=1.2, label=actor.identifier)
    _draw_footprints(top, trace)
    top.set_aspect("equal", adjustable="datalim")
    top.set_xlabel("x [m]")
    top.set_ylabel("y [m]")
    top.set_title(f"{scenario.name}: paths and footprints, ended on {trace.terminated}")
    top.legend(loc="upper left", fontsize=8, ncols=4)
    top.grid(True, alpha=0.25)

    middle = figure.add_subplot(grid[1])
    middle.plot(trace.time, trace.ego.speed, color="#1f4e79", label="ego speed [m/s]")
    middle.axhline(
        scenario.road.speed_limit, color="0.5", linestyle=":", linewidth=1.0, label="speed limit"
    )
    twin = middle.twinx()
    twin.plot(
        trace.time, trace.ego_accel, color="#a13d2d", linewidth=1.0, label="longitudinal a"
    )
    twin.plot(
        trace.time,
        trace.ego_lateral_accel,
        color="#2d7d4f",
        linewidth=1.0,
        label="lateral a",
    )
    twin.set_ylabel("acceleration [m/s^2]")
    middle.set_xlabel("time [s]")
    middle.set_ylabel("speed [m/s]")
    middle.grid(True, alpha=0.25)
    handles, labels = middle.get_legend_handles_labels()
    extra_handles, extra_labels = twin.get_legend_handles_labels()
    middle.legend(handles + extra_handles, labels + extra_labels, loc="upper right", fontsize=8)

    bottom = figure.add_subplot(grid[2])
    clearance = clearance_series(trace)
    ttc = time_to_collision_series(trace)
    bottom.plot(
        trace.time,
        np.where(np.isfinite(clearance), clearance, np.nan),
        label="clearance [m]",
    )
    bottom.plot(
        trace.time,
        np.where(np.isfinite(ttc), np.minimum(ttc, 20.0), np.nan),
        label="time to collision [s], clipped at 20",
    )
    bottom.axhline(0.0, color="0.5", linewidth=0.8)
    bottom.set_xlabel("time [s]")
    bottom.set_ylabel("metric")
    bottom.grid(True, alpha=0.25)
    bottom.legend(loc="upper right", fontsize=8)

    if assertions:
        verdicts = ", ".join(f"{item.name}: {item.outcome}" for item in assertions)
        figure.text(0.01, 0.005, verdicts, fontsize=7, color="0.3")

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(target, bbox_inches="tight")
    return target

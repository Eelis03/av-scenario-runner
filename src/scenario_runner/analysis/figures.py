"""Figures summarising one run.

The Agg canvas is constructed explicitly rather than through ``pyplot`` so that
rendering never depends on an interactive backend, a display, or global figure
state. Every function returns the path it wrote.

The three functions whose output is published under ``docs/figures`` are
``plot_encounter``, ``plot_safety_timeline``, and ``plot_bound_utilisation``.
They are deliberately small: figure size and dots per inch are chosen so the
three files together stay well inside a quarter of a megabyte without a
compression dependency, since a repository page that costs a megabyte to open
is a repository page that does not get opened.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from matplotlib.axes import Axes
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

from scenario_runner.algorithm import (
    AssertionResult,
    FloatArray,
    clearance_series,
    footprint_corners,
    time_to_collision_series,
)
from scenario_runner.model import (
    Assertion,
    GoalReached,
    LateralAcceleration,
    LongitudinalAcceleration,
    MinDistance,
    MinTimeHeadway,
    MinTimeToCollision,
    NoCollision,
    Scenario,
    SpeedLimit,
    VehicleShape,
    cartesian_to_frenet,
    frenet_to_cartesian,
)
from scenario_runner.pipeline import BodyTrace, Trace

__all__ = [
    "BoundUse",
    "bound_utilisation",
    "plot_bound_utilisation",
    "plot_encounter",
    "plot_run",
    "plot_safety_timeline",
]

_SNAPSHOTS = 5
_EGO_COLOUR = "#1f4e79"
_ACTOR_COLOUR = "#a13d2d"
_PASS_COLOUR = "#2d7d4f"


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

    return _write(figure, path)


def _write(figure: Figure, path: Path | str) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(target, bbox_inches="tight")
    return target


def _road_frame(trace: Trace, body: BodyTrace) -> tuple[FloatArray, FloatArray]:
    """Return the arc length and lateral offset series of ``body`` on the road."""
    values = [
        cartesian_to_frenet(trace.road, float(x), float(y))
        for x, y in zip(body.x, body.y, strict=True)
    ]
    return (
        np.array([item[0] for item in values], dtype=np.float64),
        np.array([item[1] for item in values], dtype=np.float64),
    )


def _footprint_box(
    axes: Axes, along: float, across: float, shape: VehicleShape, colour: str, alpha: float
) -> None:
    """Draw one footprint as a rectangle in the road frame."""
    corners = [
        (along + shape.rear_offset, across - shape.width / 2.0),
        (along + shape.front_offset, across - shape.width / 2.0),
        (along + shape.front_offset, across + shape.width / 2.0),
        (along + shape.rear_offset, across + shape.width / 2.0),
    ]
    closed = [*corners, corners[0]]
    axes.fill(
        [point[0] for point in closed],
        [point[1] for point in closed],
        color=colour,
        alpha=alpha * 0.35,
        zorder=3,
    )
    axes.plot(
        [point[0] for point in closed],
        [point[1] for point in closed],
        color=colour,
        alpha=alpha,
        linewidth=1.0,
        zorder=4,
    )


def _spread_indices(along: FloatArray, across: FloatArray, count: int) -> list[int]:
    """Pick ``count`` samples spaced evenly along the relative path, not along time.

    Even spacing in time wastes most of the snapshots on the settled following
    state, where nothing moves relative to the ego and the footprints stack on
    top of one another. Spacing them by distance travelled in the relative frame
    puts them where the geometry actually changes.
    """
    steps = int(along.size)
    if steps <= count:
        return list(range(steps))
    travelled = np.concatenate(
        ([0.0], np.cumsum(np.hypot(np.diff(along), np.diff(across))))
    )
    total = float(travelled[-1])
    if total <= 0.0:
        return [int(value) for value in np.linspace(0, steps - 1, count)]
    targets = np.linspace(0.0, total, count)
    return [int(np.searchsorted(travelled, target)) for target in targets[:-1]] + [steps - 1]


def plot_encounter(scenario: Scenario, trace: Trace, path: Path | str) -> Path:
    """Write the encounter as the ego sees it: lanes, relative range, footprints.

    A scenario document says where the vehicles start and what they do. It does
    not say what that looks like, and a plan view in absolute coordinates does
    not either, because a road is two orders of magnitude longer than it is
    wide and the vehicles collapse to specks. Holding the ego at the origin and
    plotting everything relative to it keeps the footprints true rectangles at
    an honest one to one scale, and the shape of the manoeuvre, which is the
    thing a table of worst values cannot carry, becomes visible.
    """
    figure = Figure(figsize=(7.4, 2.4), dpi=110)
    FigureCanvasAgg(figure)
    axes = figure.add_subplot(111)
    road = trace.road
    centre = float(np.mean(trace.ego_d))

    for lane in range(road.lanes + 1):
        offset = road.lane_offset(lane) - road.lane_width / 2.0 - centre
        style = "-" if lane in (0, road.lanes) else "--"
        axes.axhline(offset, color="0.7", linestyle=style, linewidth=0.8, zorder=0)

    _footprint_box(axes, 0.0, 0.0, scenario.ego.shape, _EGO_COLOUR, 1.0)
    axes.annotate(
        "ego", xy=(0.0, 0.0), xytext=(0, 14), textcoords="offset points", fontsize=7,
        color=_EGO_COLOUR, ha="center",
    )

    count = min(_SNAPSHOTS, trace.steps)
    for body in trace.actors:
        along, across = _road_frame(trace, body)
        along = along - trace.ego_s
        across = across - centre
        axes.plot(along, across, color=_ACTOR_COLOUR, linewidth=1.0, alpha=0.55, zorder=1)
        for rank, index in enumerate(_spread_indices(along, across, count)):
            alpha = 0.3 + 0.7 * rank / max(1, count - 1)
            _footprint_box(
                axes, float(along[index]), float(across[index]), body.shape, _ACTOR_COLOUR, alpha
            )
            axes.annotate(
                f"{trace.time[index]:.1f} s",
                xy=(float(along[index]), float(across[index])),
                xytext=(0, 13 + 11 * (rank % 2)),
                textcoords="offset points",
                fontsize=6,
                color="0.35",
                ha="center",
            )

    axes.set_ylim(-road.lane_width, road.lanes * road.lane_width + road.lane_width)
    axes.set_aspect("equal", adjustable="box")
    axes.set_xlabel("range ahead of the ego [m]", fontsize=8)
    axes.set_ylabel("across [m]", fontsize=8)
    axes.set_title(
        f"{scenario.name}: the encounter in the ego frame, {count} footprint snapshots "
        f"over {trace.duration:.0f} s",
        fontsize=9,
    )
    axes.grid(True, axis="x", alpha=0.2)
    axes.tick_params(labelsize=7)
    return _write(figure, path)


def _threshold(assertions: Sequence[Assertion], kind: str) -> Assertion | None:
    return next((item for item in assertions if item.kind == kind), None)


def _mark_worst(axes: Axes, time: float, value: float, text: str, colour: str) -> None:
    axes.plot([time], [value], "o", color=colour, markersize=4, zorder=5)
    axes.annotate(
        text,
        xy=(time, value),
        xytext=(6, 6),
        textcoords="offset points",
        fontsize=7,
        color=colour,
    )


def plot_safety_timeline(
    scenario: Scenario,
    trace: Trace,
    path: Path | str,
    assertions: tuple[AssertionResult, ...] = (),
) -> Path:
    """Write clearance, time to collision, and braking against time, with the bounds.

    A verdict says only that the run failed. These three series say how it
    failed: whether the vehicles ever touched, how long the ego had before they
    would have, and how hard it had to brake to keep them apart. A run can hold
    the first and lose the other two, which is the case a crash counter cannot
    see at all.
    """
    figure = Figure(figsize=(7.4, 6.2), dpi=110)
    FigureCanvasAgg(figure)
    grid = figure.add_gridspec(3, 1, hspace=0.32)
    time = trace.time
    verdicts = {item.name: item for item in assertions}

    clearance = clearance_series(trace)
    top = figure.add_subplot(grid[0])
    top.plot(time, clearance, color=_EGO_COLOUR, linewidth=1.4)
    top.axhline(0.0, color=_ACTOR_COLOUR, linewidth=1.0, linestyle="-")
    top.annotate(
        "contact", xy=(time[0], 0.0), xytext=(4, 7), textcoords="offset points", fontsize=7,
        color=_ACTOR_COLOUR,
    )
    distance = _threshold(scenario.assertions, "min_distance")
    if isinstance(distance, MinDistance):
        top.axhline(distance.threshold, color="0.45", linewidth=1.0, linestyle="--")
        top.annotate(
            f"min_distance bound {distance.threshold:g} m",
            xy=(time[-1], distance.threshold),
            xytext=(-4, 5),
            textcoords="offset points",
            fontsize=7,
            color="0.35",
            ha="right",
        )
    index = int(np.argmin(clearance))
    _mark_worst(
        top,
        float(time[index]),
        float(clearance[index]),
        f"{clearance[index]:.3f} m at {time[index]:.2f} s",
        _EGO_COLOUR,
    )
    top.set_ylabel("clearance [m]", fontsize=8)
    top.set_title(f"{scenario.name}: how the run failed, not merely that it did", fontsize=9)

    ttc = time_to_collision_series(trace)
    middle = figure.add_subplot(grid[1])
    collision_time = _threshold(scenario.assertions, "min_time_to_collision")
    cap = 4.0
    if isinstance(collision_time, MinTimeToCollision):
        cap = max(4.0 * collision_time.threshold, 2.0)
    shown = np.where(np.isfinite(ttc), np.minimum(ttc, cap), np.nan)
    middle.plot(time, shown, color=_EGO_COLOUR, linewidth=1.4)
    if isinstance(collision_time, MinTimeToCollision):
        middle.axhspan(0.0, collision_time.threshold, color=_ACTOR_COLOUR, alpha=0.12)
        middle.axhline(collision_time.threshold, color=_ACTOR_COLOUR, linewidth=1.0, linestyle="--")
        middle.annotate(
            f"bound {collision_time.threshold:g} s",
            xy=(time[-1], collision_time.threshold),
            xytext=(-4, 5),
            textcoords="offset points",
            fontsize=7,
            color=_ACTOR_COLOUR,
            ha="right",
        )
    index = int(np.argmin(ttc))
    if math.isfinite(float(ttc[index])):
        _mark_worst(
            middle,
            float(time[index]),
            float(ttc[index]),
            f"{ttc[index]:.3f} s at {time[index]:.2f} s",
            _EGO_COLOUR,
        )
    middle.set_ylim(0.0, cap * 1.08)
    middle.set_ylabel(f"time to collision [s]\nclipped at {cap:g}", fontsize=8)

    bottom = figure.add_subplot(grid[2])
    bottom.plot(time, trace.ego_accel, color=_EGO_COLOUR, linewidth=1.4)
    band = _threshold(scenario.assertions, "longitudinal_acceleration")
    if isinstance(band, LongitudinalAcceleration):
        bottom.axhspan(band.minimum, band.maximum, color=_PASS_COLOUR, alpha=0.12)
        bottom.axhline(band.minimum, color="0.45", linewidth=1.0, linestyle="--")
        bottom.annotate(
            f"comfort band [{band.minimum:g}, {band.maximum:g}] m/s^2",
            xy=(time[0], band.minimum),
            xytext=(4, -12),
            textcoords="offset points",
            fontsize=7,
            color="0.35",
        )
    index = int(np.argmax(np.abs(trace.ego_accel)))
    _mark_worst(
        bottom,
        float(time[index]),
        float(trace.ego_accel[index]),
        f"{trace.ego_accel[index]:.3f} m/s^2 at {time[index]:.2f} s",
        _EGO_COLOUR,
    )
    bottom.set_ylabel("longitudinal a [m/s^2]", fontsize=8)
    bottom.set_xlabel("time [s]", fontsize=8)

    for axes in (top, middle, bottom):
        axes.grid(True, alpha=0.25)
        axes.tick_params(labelsize=7)

    if verdicts:
        summary = ", ".join(f"{name}: {item.outcome}" for name, item in verdicts.items())
        figure.text(0.5, 0.045, summary, fontsize=7, color="0.3", ha="center")

    return _write(figure, path)


@dataclass(frozen=True, slots=True)
class BoundUse:
    """How much of one assertion's bound a run consumed."""

    scenario: str
    assertion: str
    fraction: float
    passed: bool

    @property
    def label(self) -> str:
        """The row label used on the figure."""
        return f"{self.scenario}.{self.assertion}"


def bound_utilisation(assertion: Assertion, result: AssertionResult) -> float | None:
    """Return the fraction of ``assertion``'s bound that ``result`` consumed.

    One means the worst observed value sat exactly on the bound, and anything
    above one is a violation. The point of the ratio is that it is
    dimensionless, so a clearance in metres, a time to collision in seconds and
    a deceleration in metres per second squared become comparable, and the
    assertions that only just held can be read off beside the ones that failed.

    ``no_collision`` returns ``None``: its bound is zero, so no ratio to it
    exists. Nothing is lost, because every scenario that declares
    ``no_collision`` also declares ``min_distance`` over the same series with a
    stated threshold.

    Two cases are reported as infinite rather than as a number. A clearance, a
    time to collision or a headway that reached zero has consumed a bound it can
    no longer be divided by, and a goal that was never reached has not merely
    overspent its time budget but has no finishing time at all.
    """
    worst = result.worst_value
    match assertion:
        case NoCollision():
            return None
        case (
            MinTimeToCollision(threshold=threshold)
            | MinTimeHeadway(threshold=threshold)
            | MinDistance(threshold=threshold)
        ):
            if not math.isfinite(worst):
                return 0.0
            return math.inf if worst <= 0.0 else threshold / worst
        case LongitudinalAcceleration(minimum=minimum, maximum=maximum):
            return worst / minimum if worst < 0.0 else worst / maximum
        case LateralAcceleration(limit=limit):
            return abs(worst) / limit
        case SpeedLimit(limit=limit, tolerance=tolerance):
            return worst / (limit + tolerance)
        case GoalReached(time_budget=time_budget):
            if not result.passed and worst <= time_budget:
                return math.inf
            return worst / time_budget


def plot_bound_utilisation(entries: Sequence[BoundUse], path: Path | str) -> Path:
    """Write one bar per assertion showing how much of its bound the run used.

    The suite report prints a verdict and a worst value for every assertion, but
    a reader cannot tell from a column of metres, seconds and accelerations
    which of the passes were comfortable and which were nearly failures. On this
    axis they are on the same scale, and the ones sitting against the line at
    one are the assertions that will fail first when the controller changes.
    """
    ordered = sorted(entries, key=lambda item: item.fraction)
    figure = Figure(figsize=(7.4, 7.6), dpi=100)
    FigureCanvasAgg(figure)
    axes = figure.add_subplot(111)

    positions = np.arange(len(ordered), dtype=np.float64)
    largest = max((item.fraction for item in ordered if math.isfinite(item.fraction)), default=1.0)
    ceiling = max(largest, 1.0) * 1.08
    widths = [min(item.fraction, ceiling) for item in ordered]
    colours = [_PASS_COLOUR if item.passed else _ACTOR_COLOUR for item in ordered]
    axes.barh(positions, widths, color=colours, height=0.72)
    for position, item in zip(positions, ordered, strict=True):
        if not math.isfinite(item.fraction):
            axes.annotate(
                "off scale",
                xy=(ceiling, position),
                xytext=(-4, -2),
                textcoords="offset points",
                fontsize=6,
                color="white",
                ha="right",
            )
    axes.axvline(1.0, color="0.2", linewidth=1.1)
    axes.annotate(
        "the bound",
        xy=(1.0, -0.7),
        xytext=(4, 0),
        textcoords="offset points",
        fontsize=8,
        color="0.2",
    )
    axes.set_yticks(positions)
    axes.set_yticklabels([item.label for item in ordered], fontsize=6)
    axes.set_ylim(-0.8, len(ordered) - 0.2)
    axes.set_xlim(0.0, ceiling)
    axes.set_xlabel("fraction of the assertion bound consumed by the worst observed value")
    axes.set_title(
        "Every assertion in the suite on one scale: green held, red did not", fontsize=9
    )
    axes.grid(True, axis="x", alpha=0.25)
    axes.tick_params(axis="x", labelsize=7)
    return _write(figure, path)

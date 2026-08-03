"""What the repository ships: the typing marker and the published figures.

These are the two artefacts that are consumed without running anything. A
package without ``py.typed`` type checks perfectly in its own repository and
delivers nothing to the project that installs it, and a figure that is
generated but ignored is a figure nobody reading the page will ever see. Both
failure modes are silent, so both are asserted here.
"""

from __future__ import annotations

import math
import re
from pathlib import Path

import pytest

from scenario_runner.algorithm import AssertionResult
from scenario_runner.analysis import (
    BoundUse,
    bound_utilisation,
    plot_bound_utilisation,
    plot_encounter,
    plot_run,
    plot_safety_timeline,
)
from scenario_runner.model import (
    GoalReached,
    MinDistance,
    MinTimeToCollision,
    NoCollision,
    Scenario,
    SpeedLimit,
    load_scenario,
)
from scenario_runner.pipeline import Trace, run_scenario
from tests.conftest import REPO_ROOT, SCENARIO_DIR

PACKAGE_DIR = REPO_ROOT / "src" / "scenario_runner"
FIGURE_DIR = REPO_ROOT / "docs" / "figures"
README = REPO_ROOT / "README.md"

#: The budget the portfolio validator enforces on ``docs/figures``.
FIGURE_BUDGET_BYTES = 250 * 1024

#: Alt text has to describe the finding rather than number the figure.
MINIMUM_ALT_TEXT = 20

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"

_IMAGE = re.compile(r"!\[(?P<alt>[^\]]*)\]\((?P<target>[^)\s]+)\)")


@pytest.fixture(scope="module")
def short_run() -> tuple[Scenario, Trace]:
    """A real but short run, so the figure tests render something truthful quickly."""
    scenario = load_scenario(SCENARIO_DIR / "aggressive_cut_in.toml")
    _, trace = run_scenario(scenario, max_steps=80)
    return scenario, trace


def test_the_typing_marker_ships_inside_the_package() -> None:
    """PEP 561: without this file the package exports no types to its installers."""
    marker = PACKAGE_DIR / "py.typed"
    assert marker.is_file(), f"missing PEP 561 marker at {marker}"
    assert marker.parent == PACKAGE_DIR
    assert (marker.parent / "__init__.py").is_file(), "the marker must sit beside the package"
    assert marker.read_bytes() == b"", "the marker is a presence flag, not a document"


def test_published_figures_are_real_images() -> None:
    """Every file under docs/figures is a non-empty PNG, not a placeholder."""
    figures = sorted(FIGURE_DIR.glob("*.png"))
    assert figures, f"no published figure in {FIGURE_DIR}"
    for figure in figures:
        assert figure.stat().st_size > 0, figure.name
        assert figure.read_bytes()[:8] == PNG_MAGIC, figure.name


def test_published_figures_fit_the_budget() -> None:
    """The tracked figures stay inside the budget without a compression dependency."""
    figures = sorted(FIGURE_DIR.glob("*.png"))
    total = sum(figure.stat().st_size for figure in figures)
    assert total <= FIGURE_BUDGET_BYTES, f"{total} bytes over {FIGURE_BUDGET_BYTES}"


def test_the_readme_embeds_every_published_figure() -> None:
    """A figure that is generated but never referenced is a figure nobody sees."""
    text = README.read_text(encoding="utf-8")
    embedded = {
        Path(match.group("target")).name: match.group("alt") for match in _IMAGE.finditer(text)
    }
    for figure in sorted(FIGURE_DIR.glob("*.png")):
        assert figure.name in embedded, f"{figure.name} is published but not embedded"
        alt = embedded[figure.name].strip()
        assert len(alt) >= MINIMUM_ALT_TEXT, f"{figure.name} alt text is too short: {alt!r}"


def test_plot_encounter_writes_a_png(short_run: tuple[Scenario, Trace], tmp_path: Path) -> None:
    """The plan view renders from a real trace."""
    scenario, trace = short_run
    target = plot_encounter(scenario, trace, tmp_path / "encounter.png")
    assert target.read_bytes()[:8] == PNG_MAGIC


def test_plot_safety_timeline_writes_a_png(
    short_run: tuple[Scenario, Trace], tmp_path: Path
) -> None:
    """The clearance, time to collision and braking figure renders from a real trace."""
    scenario, trace = short_run
    result, _ = run_scenario(scenario, max_steps=80)
    target = plot_safety_timeline(
        scenario, trace, tmp_path / "timeline.png", result.assertions
    )
    assert target.read_bytes()[:8] == PNG_MAGIC


def test_plot_run_writes_a_png(short_run: tuple[Scenario, Trace], tmp_path: Path) -> None:
    """The three panel per scenario summary that ``examples/plot_scenario.py`` writes."""
    scenario, trace = short_run
    result, _ = run_scenario(scenario, max_steps=80)
    target = plot_run(scenario, trace, tmp_path / "run.png", result.assertions)
    assert target.read_bytes()[:8] == PNG_MAGIC
    assert target.stat().st_size > 1024


def test_plot_run_handles_a_curved_road_and_several_actors(tmp_path: Path) -> None:
    """The road drawing path covers an arc, not only the straight case."""
    scenario = load_scenario(SCENARIO_DIR / "curved_lane_keeping.toml")
    result, trace = run_scenario(scenario, max_steps=80)
    target = plot_run(scenario, trace, tmp_path / "curved.png", result.assertions)
    assert target.read_bytes()[:8] == PNG_MAGIC


def test_plot_bound_utilisation_writes_a_png(tmp_path: Path) -> None:
    """The suite wide bar chart renders, including an entry that is off the scale."""
    entries = [
        BoundUse(scenario="a", assertion="min_distance", fraction=0.25, passed=True),
        BoundUse(scenario="b", assertion="min_time_to_collision", fraction=2.4, passed=False),
        BoundUse(scenario="c", assertion="no_goal", fraction=math.inf, passed=False),
    ]
    target = plot_bound_utilisation(entries, tmp_path / "bounds.png")
    assert target.read_bytes()[:8] == PNG_MAGIC


def _result(name: str, kind: str, passed: bool, worst: float) -> AssertionResult:
    return AssertionResult(
        name=name,
        kind=kind,
        passed=passed,
        worst_value=worst,
        worst_time=1.0,
        unit="",
        bound="",
        detail="",
    )


def test_bound_utilisation_is_exactly_one_on_the_bound() -> None:
    """A worst value sitting on its threshold consumes the bound exactly."""
    assertion = MinTimeToCollision(threshold=1.5)
    result = _result("min_time_to_collision", assertion.kind, True, 1.5)
    assert bound_utilisation(assertion, result) == 1.0


def test_bound_utilisation_has_no_ratio_for_no_collision() -> None:
    """The bound of ``no_collision`` is zero, so no fraction of it exists."""
    assertion = NoCollision()
    assert bound_utilisation(assertion, _result("no_collision", assertion.kind, True, 4.0)) is None


def test_bound_utilisation_is_zero_when_the_metric_never_closed() -> None:
    """An infinite time to collision consumed none of its bound."""
    assertion = MinTimeToCollision(threshold=1.5)
    result = _result("min_time_to_collision", assertion.kind, True, math.inf)
    assert bound_utilisation(assertion, result) == 0.0


def test_bound_utilisation_is_unbounded_on_contact() -> None:
    """Clearance that reached zero cannot be expressed as a fraction of a threshold."""
    assertion = MinDistance(threshold=1.0)
    result = _result("min_distance", assertion.kind, False, -0.408)
    assert bound_utilisation(assertion, result) == math.inf


def test_bound_utilisation_is_unbounded_for_a_goal_never_reached() -> None:
    """A goal that was never reached is a failure even though its clock stopped early."""
    assertion = GoalReached(goal_s=200.0, time_budget=18.0)
    result = _result("goal_reached", assertion.kind, False, 7.3)
    assert bound_utilisation(assertion, result) == math.inf


def test_bound_utilisation_uses_the_tolerance_on_a_speed_limit() -> None:
    """The bound a speed limit assertion is scored against includes its tolerance."""
    assertion = SpeedLimit(limit=13.9, tolerance=0.1)
    result = _result("speed_limit", assertion.kind, True, 7.0)
    assert bound_utilisation(assertion, result) == pytest.approx(0.5)


def test_every_suite_assertion_scores_against_its_own_bound() -> None:
    """Across the shipped suite, the fraction agrees with the verdict it came from."""
    checked = 0
    for path in sorted(SCENARIO_DIR.glob("*.toml")):
        scenario = load_scenario(path)
        result, _ = run_scenario(scenario)
        by_name = {item.name: item for item in result.assertions}
        for assertion in scenario.assertions:
            fraction = bound_utilisation(assertion, by_name[assertion.name])
            if fraction is None:
                continue
            assert fraction >= 0.0, f"{scenario.name}.{assertion.name}"
            assert (fraction <= 1.0) == by_name[assertion.name].passed, (
                f"{scenario.name}.{assertion.name}: fraction {fraction} "
                f"against verdict {by_name[assertion.name].outcome}"
            )
            checked += 1
    assert checked >= 30

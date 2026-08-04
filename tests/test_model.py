"""Tier one: scenario parsing, validation, versioning, and round tripping."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from scenario_runner.model import (
    CURRENT_FORMAT_VERSION,
    KNOWN_CONTROLLERS,
    SUPPORTED_FORMAT_VERSIONS,
    MinTimeHeadway,
    ReactiveBehaviour,
    Scenario,
    ScenarioError,
    ScriptedBehaviour,
    cartesian_to_frenet,
    frenet_to_cartesian,
    load_scenario,
    parse_scenario,
    to_toml,
)
from scenario_runner.pipeline import CONTROLLERS
from tests.conftest import FIXTURE_DIR, MALFORMED_DIR

# Each malformed fixture is paired with the field its rejection message must
# name. A parser that rejects the document for the wrong reason is only
# accidentally correct, so the field, not merely the failure, is asserted.
MALFORMED_EXPECTATIONS: dict[str, str] = {
    "accel_band_inverted.toml": "assert[0].minimum",
    "arc_without_radius.toml": "road.radius",
    "assertion_missing_threshold.toml": "assert[0].threshold",
    "bad_syntax.toml": "invalid TOML",
    "duplicate_actor_id.toml": "actor[1].id",
    "duplicate_assertion_name.toml": "assert[1].name",
    "ego_lane_out_of_range.toml": "ego.lane",
    "ego_speed_as_string.toml": "ego.speed",
    "goal_reached_without_goal.toml": "assert[0].goal_s",
    "missing_format_version.toml": "format_version",
    "missing_name.toml": "name",
    "negative_lane_width.toml": "road.lane_width",
    "no_assertions.toml": "assert",
    "perception_in_version_1_0.toml": "perception",
    "reactive_negative_time_gap.toml": "actor[0].reactive.time_gap",
    "schedule_not_monotonic.toml": "actor[0].schedule[1].time",
    "scripted_with_reactive_table.toml": "actor[0].reactive",
    "stop_at_goal_without_goal.toml": "termination.stop_at_goal",
    "unknown_assertion_kind.toml": "assert[0].kind",
    "unknown_controller.toml": "ego.controller",
    "unknown_format_version.toml": "format_version",
    "unknown_road_key.toml": "road.lane_with",
}


def test_every_malformed_fixture_is_covered(malformed_paths: tuple[Path, ...]) -> None:
    """The expectation table and the fixture directory must agree exactly."""
    on_disk = {path.name for path in malformed_paths}
    assert on_disk == set(MALFORMED_EXPECTATIONS), (
        "malformed fixture directory and expectation table disagree"
    )
    assert len(on_disk) >= 20


@pytest.mark.parametrize("filename", sorted(MALFORMED_EXPECTATIONS))
def test_malformed_scenario_is_rejected_naming_the_field(filename: str) -> None:
    """Every malformed document is rejected with a message naming the offending field."""
    with pytest.raises(ScenarioError) as caught:
        load_scenario(MALFORMED_DIR / filename)
    message = str(caught.value)
    assert MALFORMED_EXPECTATIONS[filename] in message, message
    assert filename in message, message


def test_rejection_message_carries_a_line_number() -> None:
    """A field that exists in the source is located, not merely named."""
    with pytest.raises(ScenarioError) as caught:
        load_scenario(MALFORMED_DIR / "negative_lane_width.toml")
    error = caught.value
    assert error.field == "road.lane_width"
    assert error.line == 6
    assert ":6:" in str(error)


def test_syntax_error_carries_a_line_number() -> None:
    """A TOML syntax error keeps the position ``tomllib`` reported."""
    with pytest.raises(ScenarioError) as caught:
        load_scenario(MALFORMED_DIR / "bad_syntax.toml")
    assert caught.value.line is not None


def test_unknown_format_version_names_the_supported_set() -> None:
    """An unreadable version is refused with the versions this build accepts."""
    with pytest.raises(ScenarioError) as caught:
        load_scenario(MALFORMED_DIR / "unknown_format_version.toml")
    message = str(caught.value)
    for version in SUPPORTED_FORMAT_VERSIONS:
        assert version in message


def test_older_format_version_still_loads() -> None:
    """A 1.0 document loads and receives the default perception model."""
    scenario = load_scenario(FIXTURE_DIR.parents[1] / "scenarios" / "speed_limit_compliance.toml")
    assert scenario.format_version == "1.0"
    assert scenario.perception.is_noiseless


def test_new_field_is_refused_in_an_older_document() -> None:
    """A 1.0 document may not use a field that 1.1 introduced."""
    with pytest.raises(ScenarioError) as caught:
        load_scenario(MALFORMED_DIR / "perception_in_version_1_0.toml")
    message = str(caught.value)
    assert "perception" in message
    assert "1.1" in message


def test_valid_scenario_round_trips_through_serialise_and_parse() -> None:
    """Rendering a scenario and parsing it back reconstructs an equal object."""
    original = load_scenario(FIXTURE_DIR / "valid_minimal.toml")
    rendered = to_toml(original)
    restored = parse_scenario(rendered, source="round_trip.toml")
    assert restored == original
    assert to_toml(restored) == rendered


def test_every_shipped_scenario_round_trips(scenario_paths: tuple[Path, ...]) -> None:
    """The round trip holds for every scenario in the shipped suite."""
    assert len(scenario_paths) >= 8
    for path in scenario_paths:
        original = load_scenario(path)
        restored = parse_scenario(to_toml(original), source=path.name)
        assert restored == original, path.name


def test_model_and_pipeline_agree_on_controller_names() -> None:
    """The names the model accepts are exactly the names the pipeline implements."""
    assert set(CONTROLLERS) == set(KNOWN_CONTROLLERS)


def test_current_format_version_is_supported() -> None:
    """The version the serialiser writes is a version the parser reads."""
    assert CURRENT_FORMAT_VERSION in SUPPORTED_FORMAT_VERSIONS


def test_parsed_behaviours_have_the_declared_types() -> None:
    """A scripted actor parses to a scripted behaviour and likewise for reactive."""
    scenario = load_scenario(FIXTURE_DIR / "valid_minimal.toml")
    by_id = {actor.identifier: actor for actor in scenario.actors}
    assert isinstance(by_id["lead"].behaviour, ScriptedBehaviour)
    assert isinstance(by_id["trail"].behaviour, ReactiveBehaviour)
    lead = by_id["lead"].behaviour
    assert isinstance(lead, ScriptedBehaviour)
    assert lead.lane_change is not None
    assert lead.lane_change.target_lane == 1


def test_goal_reached_inherits_the_termination_goal() -> None:
    """An assertion without its own goal takes the one termination declares."""
    scenario = load_scenario(FIXTURE_DIR / "valid_minimal.toml")
    goal = next(item for item in scenario.assertions if item.kind == "goal_reached")
    assert goal.goal_s == scenario.termination.goal_s


def test_min_time_headway_carries_its_threshold_through_the_round_trip() -> None:
    """The headway assertion survives the serialiser with its bound intact."""
    document = (FIXTURE_DIR / "valid_minimal.toml").read_text(encoding="utf-8")
    document += '\n[[assert]]\nkind = "min_time_headway"\nthreshold = 1.2\n'
    scenario = parse_scenario(document, source="headway.toml")
    headway = next(item for item in scenario.assertions if item.kind == "min_time_headway")
    assert isinstance(headway, MinTimeHeadway)
    assert headway.threshold == 1.2
    assert parse_scenario(to_toml(scenario), source="headway.toml") == scenario


def test_min_time_headway_without_a_threshold_is_rejected() -> None:
    """The bound is required rather than defaulted, so an omission cannot pass quietly."""
    document = (FIXTURE_DIR / "valid_minimal.toml").read_text(encoding="utf-8")
    document += '\n[[assert]]\nkind = "min_time_headway"\n'
    with pytest.raises(ScenarioError) as caught:
        parse_scenario(document, source="headway.toml")
    assert "assert[3].threshold" in str(caught.value)


def test_default_assertion_names_are_unique_per_kind() -> None:
    """Repeating a kind yields distinct default names rather than a collision."""
    document = (FIXTURE_DIR / "valid_minimal.toml").read_text(encoding="utf-8")
    document += '\n[[assert]]\nkind = "no_collision"\n'
    scenario = parse_scenario(document, source="duplicated.toml")
    names = [item.name for item in scenario.assertions]
    assert names.count("no_collision") == 1
    assert "no_collision_2" in names


@pytest.mark.parametrize("radius", [80.0, 150.0, 400.0])
@pytest.mark.parametrize("direction", ["left", "right"])
def test_road_frame_conversions_are_inverse(radius: float, direction: str) -> None:
    """Cartesian and road frame conversions invert each other on a curved road."""
    document = f"""
format_version = "1.1"
name = "arc"

[road]
kind = "arc"
radius = {radius}
direction = "{direction}"
lanes = 2

[ego]
speed = 10.0

[termination]
max_time = 5.0

[[assert]]
kind = "no_collision"
"""
    road = parse_scenario(document, source="arc.toml").road
    for arc_length in (0.0, 12.5, 60.0, 133.3):
        for offset in (-1.75, 0.0, 3.5):
            x, y, _ = frenet_to_cartesian(road, arc_length, offset)
            recovered_s, recovered_d = cartesian_to_frenet(road, x, y)
            assert math.isclose(recovered_s, arc_length, abs_tol=1e-9)
            assert math.isclose(recovered_d, offset, abs_tol=1e-9)


def test_straight_road_frame_is_the_identity() -> None:
    """With zero curvature the road frame and the Cartesian frame coincide."""
    scenario: Scenario = load_scenario(FIXTURE_DIR / "valid_minimal.toml")
    x, y, heading = frenet_to_cartesian(scenario.road, 37.5, 1.75)
    assert (x, y, heading) == (37.5, 1.75, 0.0)


def test_vehicle_disc_cover_contains_the_footprint() -> None:
    """Every footprint corner lies inside the disc cover used for clearance."""
    scenario = load_scenario(FIXTURE_DIR / "valid_minimal.toml")
    shape = scenario.ego.shape
    corners = [
        (along, across)
        for along in (shape.rear_offset, shape.front_offset)
        for across in (-shape.width / 2.0, shape.width / 2.0)
    ]
    for along, across in corners:
        covered = min(
            math.hypot(along - offset, across) for offset in shape.disc_offsets
        )
        assert covered <= shape.disc_radius + 1e-12

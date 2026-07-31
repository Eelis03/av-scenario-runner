"""Tier one: simulator determinism, actor behaviour, and runner exit codes."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from scenario_runner.analysis import (
    dump_suite_result,
    load_suite_record,
    load_suite_result,
    render_suite,
    trace_to_json,
)
from scenario_runner.cli import main
from scenario_runner.model import load_scenario, parse_scenario
from scenario_runner.pipeline import (
    BicycleState,
    bicycle_step,
    idm_acceleration,
    lateral_acceleration,
    run_scenario,
    run_scenario_file,
    run_suite,
    simulate,
)
from tests.conftest import MALFORMED_DIR, SCENARIO_DIR

EXPECTED_FAILURES = ("aggressive_cut_in.toml", "stopped_obstacle_no_brake.toml")

NOISY_SCENARIO = """
format_version = "1.1"
name = "noisy_follow"
dt = 0.05
seed = 7

[road]
kind = "straight"
lanes = 1
lane_width = 3.5
length = 400.0
speed_limit = 13.9

[ego]
lane = 0
s = 0.0
speed = 12.0
controller = "idm_lane_keep"

[[actor]]
id = "lead"
lane = 0
s = 30.0
speed = 9.0
behaviour = "scripted"
schedule = [{ time = 0.0, accel = 0.0 }]

[termination]
max_time = 8.0

[[assert]]
kind = "no_collision"

[perception]
range_noise_std = 1.5
speed_noise_std = 0.4
"""


# ---------------------------------------------------------------------------
# Motion models
# ---------------------------------------------------------------------------


def test_bicycle_step_with_zero_steering_is_straight_line_motion() -> None:
    """With no steering the integrator reproduces the closed form of uniform acceleration."""
    state = BicycleState(x=0.0, y=0.0, yaw=0.0, speed=10.0)
    dt, accel = 0.1, 1.5
    advanced = bicycle_step(state, accel, 0.0, 2.8, dt)
    assert advanced.x == pytest.approx(10.0 * dt + 0.5 * accel * dt * dt, rel=1e-12)
    assert advanced.y == pytest.approx(0.0, abs=1e-15)
    assert advanced.yaw == pytest.approx(0.0, abs=1e-15)
    assert advanced.speed == pytest.approx(10.0 + accel * dt, rel=1e-15)


def test_bicycle_step_traces_a_circle_of_the_expected_radius() -> None:
    """Constant steering at constant speed closes a circle of radius L / tan(delta)."""
    wheelbase, steer, speed = 2.8, 0.15, 10.0
    radius = wheelbase / math.tan(steer)
    state = BicycleState(x=0.0, y=0.0, yaw=0.0, speed=speed)
    dt = 0.01
    for _ in range(round(2.0 * math.pi * radius / speed / dt)):
        state = bicycle_step(state, 0.0, steer, wheelbase, dt)
    assert state.x == pytest.approx(0.0, abs=0.05)
    assert state.y == pytest.approx(0.0, abs=0.05)


def test_lateral_acceleration_matches_the_circular_identity() -> None:
    """Lateral acceleration equals v squared over the turning radius."""
    wheelbase, steer, speed = 2.8, 0.12, 14.0
    radius = wheelbase / math.tan(steer)
    assert lateral_acceleration(speed, steer, wheelbase) == pytest.approx(
        speed * speed / radius, rel=1e-12
    )


def test_idm_brakes_when_the_gap_closes() -> None:
    """The interaction term dominates once the gap falls below the desired gap."""
    from scenario_runner.model import IdmParams

    params = IdmParams()
    free = idm_acceleration(params, 12.0, 13.9, None, None)
    tight = idm_acceleration(params, 12.0, 13.9, 5.0, 6.0)
    assert free > 0.0
    assert tight < -1.0
    assert tight >= -params.max_decel


# ---------------------------------------------------------------------------
# Determinism
# ---------------------------------------------------------------------------


def test_noiseless_simulation_is_identical_across_seeds() -> None:
    """Without perception noise the seed cannot influence the result at all."""
    scenario = load_scenario(SCENARIO_DIR / "lead_vehicle_braking.toml")
    first = simulate(scenario, seed=1)
    second = simulate(scenario, seed=99)
    assert np.array_equal(first.ego.x, second.ego.x)
    assert np.array_equal(first.ego.speed, second.ego.speed)
    assert np.array_equal(first.ego_accel, second.ego_accel)


def test_noisy_simulation_is_reproducible_for_a_given_seed() -> None:
    """With noise on, the same seed reproduces the run exactly."""
    scenario = parse_scenario(NOISY_SCENARIO, source="noisy.toml")
    first = simulate(scenario, seed=7)
    second = simulate(scenario, seed=7)
    assert np.array_equal(first.ego.x, second.ego.x)
    assert np.array_equal(first.ego_accel, second.ego_accel)
    assert np.array_equal(first.ego.yaw, second.ego.yaw)


def test_noisy_simulation_differs_between_seeds() -> None:
    """With noise on, the seed is load bearing, so the test above is not vacuous."""
    scenario = parse_scenario(NOISY_SCENARIO, source="noisy.toml")
    first = simulate(scenario, seed=7)
    second = simulate(scenario, seed=8)
    assert not np.array_equal(first.ego_accel, second.ego_accel)


def test_scenario_seed_is_used_when_no_override_is_given() -> None:
    """The seed recorded in the trace is the one the document declared."""
    scenario = parse_scenario(NOISY_SCENARIO, source="noisy.toml")
    assert simulate(scenario).seed == 7
    assert simulate(scenario, seed=3).seed == 3


# ---------------------------------------------------------------------------
# Simulation behaviour
# ---------------------------------------------------------------------------


def test_scripted_actor_follows_its_schedule() -> None:
    """A braking actor reaches standstill and never reverses."""
    scenario = load_scenario(SCENARIO_DIR / "lead_vehicle_braking.toml")
    trace = simulate(scenario)
    lead = next(actor for actor in trace.actors if actor.identifier == "lead")
    assert float(np.min(lead.speed)) == 0.0
    assert np.all(np.diff(lead.x) >= -1e-12)


def test_lane_change_moves_the_actor_across_the_lane_width() -> None:
    """A cutting actor ends in the ego lane, having started one lane over."""
    scenario = load_scenario(SCENARIO_DIR / "cut_in_moderate.toml")
    trace = simulate(scenario)
    cutter = next(actor for actor in trace.actors if actor.identifier == "cutter")
    assert float(cutter.y[0]) == pytest.approx(scenario.road.lane_width)
    assert float(cutter.y[-1]) == pytest.approx(0.0, abs=1e-9)


def test_simulation_stops_on_collision_when_asked() -> None:
    """A collision terminates the run and is visible in the recorded clearance."""
    scenario = load_scenario(SCENARIO_DIR / "stopped_obstacle_no_brake.toml")
    trace = simulate(scenario)
    assert trace.collided
    assert trace.terminated == "collision"
    assert trace.duration < scenario.termination.max_time


def test_max_steps_truncates_a_run() -> None:
    """The step cap used by the example integration tests really shortens the run."""
    scenario = load_scenario(SCENARIO_DIR / "straight_cruise.toml")
    trace = simulate(scenario, max_steps=40)
    assert trace.steps == 41
    assert trace.terminated == "step_limit"


def test_ego_speed_never_goes_negative() -> None:
    """Braking to standstill clamps at zero rather than reversing."""
    scenario = load_scenario(SCENARIO_DIR / "stopped_obstacle.toml")
    trace = simulate(scenario)
    assert float(np.min(trace.ego.speed)) >= 0.0
    assert float(np.min(trace.ego.speed)) == pytest.approx(0.0, abs=1e-6)


def test_curved_run_holds_the_lane_within_a_quarter_metre() -> None:
    """Pure pursuit tracks the arc centre line closely enough to be a lane keeper."""
    scenario = load_scenario(SCENARIO_DIR / "curved_lane_keeping.toml")
    trace = simulate(scenario)
    assert float(np.max(np.abs(trace.ego_d))) < 0.25


# ---------------------------------------------------------------------------
# Runner and exit codes
# ---------------------------------------------------------------------------


def test_runner_returns_non_zero_for_the_scenarios_expected_to_fail() -> None:
    """Running only the two failing scenarios exits non-zero under the literal rule."""
    paths = [SCENARIO_DIR / name for name in EXPECTED_FAILURES]
    suite = run_suite(paths)
    assert len(suite.failed) == 2
    assert suite.exit_code() == 1
    assert main(["run", *[str(path) for path in paths], "--quiet"]) == 1


def test_the_same_two_scenarios_exit_zero_under_the_expectation_rule() -> None:
    """Those scenarios declare that they fail, so the expectation rule is satisfied."""
    paths = [SCENARIO_DIR / name for name in EXPECTED_FAILURES]
    suite = run_suite(paths)
    assert not suite.unexpected
    assert suite.exit_code(check_expectations=True) == 0
    assert main(["run", *[str(path) for path in paths], "--quiet", "--expect"]) == 0


def test_runner_returns_zero_for_a_passing_scenario() -> None:
    """A clean scenario exits zero under both rules."""
    path = SCENARIO_DIR / "straight_cruise.toml"
    suite = run_suite([path])
    assert suite.exit_code() == 0
    assert suite.exit_code(check_expectations=True) == 0


def test_whole_shipped_suite_matches_its_declared_expectations(
    scenario_paths: tuple[Path, ...],
) -> None:
    """Every shipped scenario behaves the way its own document says it will."""
    suite = run_suite(list(scenario_paths))
    assert len(suite.scenarios) >= 8
    assert not suite.unexpected, [item.name for item in suite.unexpected]
    assert len(suite.failed) == len(EXPECTED_FAILURES)


def test_malformed_scenario_becomes_a_failing_result_not_a_skip() -> None:
    """A document that will not load fails the suite instead of vanishing from it."""
    result = run_scenario_file(MALFORMED_DIR / "unknown_controller.toml")
    assert not result.passed
    assert result.error is not None
    assert "ego.controller" in result.error
    assert run_suite([MALFORMED_DIR / "unknown_controller.toml"]).exit_code() == 1


def test_collision_scenario_reports_negative_clearance_with_a_time() -> None:
    """The failing collision scenario reports the evidence, not only the verdict."""
    scenario = load_scenario(SCENARIO_DIR / "stopped_obstacle_no_brake.toml")
    result, _ = run_scenario(scenario)
    collision = next(item for item in result.assertions if item.kind == "no_collision")
    assert not collision.passed
    assert collision.worst_value <= 0.0
    assert collision.worst_time is not None
    assert 0.0 < collision.worst_time <= scenario.termination.max_time


# ---------------------------------------------------------------------------
# Serialisation of results and traces
# ---------------------------------------------------------------------------


def test_suite_result_round_trips_through_json(tmp_path: Path) -> None:
    """A written result reads back with the same verdicts and the same worst values."""
    suite = run_suite(
        [SCENARIO_DIR / "straight_cruise.toml", SCENARIO_DIR / "cut_in_moderate.toml"]
    )
    path = dump_suite_result(suite, tmp_path / "result.json")
    restored = load_suite_result(path)
    assert restored == suite


def test_json_result_is_strict_json_even_with_infinite_metrics(tmp_path: Path) -> None:
    """Infinity is encoded as a string so the document is valid JSON everywhere."""
    suite = run_suite([SCENARIO_DIR / "straight_cruise.toml"])
    path = dump_suite_result(suite, tmp_path / "result.json")
    text = path.read_text(encoding="utf-8")
    assert "Infinity" not in text
    assert '"inf"' in text
    raw = json.loads(text)
    assert raw["result_version"] == "1.0"
    record = load_suite_record(path)
    assert math.isinf(record.scenarios[0].assertions[0].worst_value)


def test_report_names_every_scenario_and_both_exit_codes() -> None:
    """The text report is complete enough to act on without the JSON."""
    suite = run_suite([SCENARIO_DIR / name for name in EXPECTED_FAILURES])
    text = render_suite(suite)
    assert "aggressive_cut_in" in text
    assert "stopped_obstacle_no_brake" in text
    assert "literal exit code 1" in text
    assert "expectation exit code 0" in text


def test_viz_trace_export_is_self_describing() -> None:
    """The browser trace carries its schema version, the road, and every body."""
    scenario = load_scenario(SCENARIO_DIR / "cut_in_moderate.toml")
    result, trace = run_scenario(scenario, max_steps=60)
    payload = trace_to_json(trace, result.assertions)
    assert payload["trace_version"] == "1.0"
    assert payload["road"]["lanes"] == scenario.road.lanes
    identifiers = [body["id"] for body in payload["bodies"]]
    assert identifiers == ["ego", "cutter"]
    assert len(payload["time"]) == trace.steps
    assert len(payload["metrics"]["clearance"]) == trace.steps


def test_cli_writes_the_artefacts_it_is_asked_for(tmp_path: Path) -> None:
    """The run subcommand produces both the text report and the JSON result."""
    report = tmp_path / "report.txt"
    result = tmp_path / "result.json"
    code = main(
        [
            "run",
            str(SCENARIO_DIR / "straight_cruise.toml"),
            "--json",
            str(result),
            "--report",
            str(report),
            "--quiet",
        ]
    )
    assert code == 0
    assert "straight_cruise" in report.read_text(encoding="utf-8")
    assert json.loads(result.read_text(encoding="utf-8"))["summary"]["total"] == 1


def test_cli_compare_subcommand_reports_a_regression(tmp_path: Path) -> None:
    """Comparing a failing run against a passing baseline exits non-zero."""
    baseline = tmp_path / "baseline.json"
    current = tmp_path / "current.json"
    dump_suite_result(run_suite([SCENARIO_DIR / "stopped_obstacle.toml"]), baseline)

    raw = json.loads(baseline.read_text(encoding="utf-8"))
    raw["scenarios"][0]["assertions"][0]["passed"] = True
    for entry in raw["scenarios"]:
        entry["passed"] = True
    baseline.write_text(json.dumps(raw), encoding="utf-8")

    degraded = json.loads(baseline.read_text(encoding="utf-8"))
    degraded["scenarios"][0]["assertions"][0]["passed"] = False
    degraded["scenarios"][0]["passed"] = False
    current.write_text(json.dumps(degraded), encoding="utf-8")

    assert main(["compare", str(current), str(baseline)]) == 1
    assert main(["compare", str(baseline), str(baseline)]) == 0

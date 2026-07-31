# Av Scenario Runner

Declarative scenario format and regression harness for evaluating autonomous driving controllers.

[![CI](https://github.com/Eelis03/av-scenario-runner/actions/workflows/ci.yml/badge.svg)](https://github.com/Eelis03/av-scenario-runner/actions/workflows/ci.yml)
[![Python](https://img.shields.io/badge/python-3.12-blue)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

## Overview

This repository defines a scenario in a single validated TOML document, simulates it
deterministically, and scores it against an assertion vocabulary that reports the worst value
observed and the time it occurred. A suite of scenarios runs from one command, produces a text
report and a machine readable JSON result, returns a non-zero exit code when something fails, and
can be compared against a stored baseline that separates an assertion which newly fails from a
metric that merely moved. It is intended for anyone building a planning or control component who
needs a repeatable answer to the question of whether a change made behaviour worse.

## Problem

A controller is normally tested by running it and looking at the plots. That does not scale, it is
not repeatable, and it produces no artefact that a build server can act on. Three specific failures
follow from it.

The first is silent misconfiguration. A scenario written in a permissive format, where a misspelled
key is ignored and a missing field takes a default, tests something other than what its author
believes. It then passes, and the pass is worthless. A scenario document must therefore be
validated strictly and rejected loudly, with the offending field named and located.

The second is a verdict without evidence. Knowing that a run passed says nothing about whether it
passed with four metres of margin or two centimetres. The second case is a failure waiting for a
slightly different initial condition, and it is indistinguishable from the first in any report that
prints only a verdict.

The third is a baseline that cannot be trusted. A regression harness that pins raw floating point
numbers from late in a simulated run fails on a different machine for reasons unrelated to the code
under test, and a test that fails for unrelated reasons is one that people learn to ignore. Which
quantities a baseline may pin is the central design question here, and it is answered in
`docs/design-notes.md`.

## Approach

Scenarios are described in the style of the ASAM OpenSCENARIO standard: a road, an ego vehicle with
an initial state, actors with either a scripted manoeuvre or a reactive behaviour, termination
conditions, and the assertions the run must satisfy. The format is TOML rather than XML, parsed
with the standard library `tomllib`, and every field is validated against a typed model before
anything runs. Unknown keys are rejected. The document carries a `format_version`, and a version
this build cannot read is refused by name rather than partially interpreted.

Safety is scored with published metrics rather than invented ones. Time to collision follows the
constant velocity definition of Hayward (1972), generalised to two dimensions so that a lateral
encounter such as a cut in is scored rather than ignored. Footprints are covered by equal discs
along the vehicle axis, the standard fast collision approximation of Ziegler and Stiller (2010),
which makes clearance and time to collision the same geometric query. The reference controller
combines the Intelligent Driver Model of Treiber, Hennecke and Helbing (2000) for car following
with pure pursuit lane keeping (Coulter, 1992), driving a kinematic bicycle in the rear axle
formulation studied by Kong, Pfeiffer, Schildbach and Borrelli (2015). The simulator uses a fixed
step Runge-Kutta integrator with no adaptive control and no iterative solve, so a run is a pure
function of the document and the seed.

The reasons for choosing these over the alternatives, and the reasons for the specific choice of
what a regression baseline may pin, are in [docs/design-notes.md](docs/design-notes.md).

## Installation

Requires Python 3.12 or later.

```bash
git clone https://github.com/Eelis03/av-scenario-runner.git
cd av-scenario-runner
uv sync
```

Using pip instead of uv:

```bash
python -m venv .venv
.venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -e ".[dev]"
```

## Usage

A complete scenario, in full:

```toml
format_version = "1.1"
name = "lead_vehicle_braking"
description = "Lead vehicle decelerates at 3 m/s^2 to standstill ahead of the ego."
expected_outcome = "pass"
dt = 0.05

[road]
kind = "straight"
lanes = 2
lane_width = 3.5
length = 400.0
speed_limit = 13.9

[ego]
lane = 0
s = 0.0
speed = 13.0
controller = "idm_lane_keep"

[[actor]]
id = "lead"
lane = 0
s = 40.0
speed = 13.0
behaviour = "scripted"
schedule = [{ time = 0.0, accel = 0.0 }, { time = 4.0, accel = -3.0 }]

[termination]
max_time = 20.0

[[assert]]
kind = "min_time_to_collision"
threshold = 1.5
```

Run a suite and gate a build on it:

```bash
uv run python -m scenario_runner run scenarios/*.toml --expect \
    --json outputs/suite_result.json --report outputs/suite_report.txt
```

`--expect` scores each scenario against the `expected_outcome` it declares, so a suite containing
scenarios that are supposed to fail still exits zero when every one of them behaves as documented.
Without the flag the exit code is literal: any failing assertion fails the suite.

Compare a run against a recorded baseline:

```bash
uv run python -m scenario_runner compare outputs/suite_result.json baselines/reference_suite.json
```

From Python:

```python
from scenario_runner import load_scenario
from scenario_runner.pipeline import run_scenario

scenario = load_scenario("scenarios/lead_vehicle_braking.toml")
result, trace = run_scenario(scenario)
for item in result.assertions:
    print(f"{item.outcome} {item.name}: worst {item.worst_value:.3f} {item.unit} at {item.worst_time:.2f} s")
```

Runnable examples live in `examples/`:

```bash
uv run python examples/run_scenario_suite.py
uv run python examples/regression_check.py
uv run python examples/plot_scenario.py --scenario cut_in_moderate
uv run python examples/validate_scenarios.py
uv run python examples/export_viz_trace.py --scenario aggressive_cut_in
```

## Results

The shipped suite is ten scenarios covering free driving, lead vehicle braking, a stopped obstacle,
two cut ins, a merge, a reactive tailgater, a curved road, and speed limit compliance. Two of them
are expected to fail. All figures below are the real output of

```bash
uv run python -m scenario_runner run scenarios/*.toml --expect
```

on Python 3.12 with numpy 2.5.1, at `dt = 0.05` s.

```
scenario suite: 10 scenarios, 8 passed, 2 failed, 0 unexpected

FAIL aggressive_cut_in              expect fail  5 assertions   16.00 s   321 steps  ended on max_time
    PASS no_collision                   worst     1.652 m      at   1.90s  bound > 0 m
    PASS min_distance                   worst     1.652 m      at   1.90s  bound >= 1 m
    FAIL min_time_to_collision          worst     0.707 s      at   1.45s  bound >= 1.5 s
    FAIL longitudinal_acceleration      worst    -8.000 m/s^2  at   0.95s  bound in [-3, 2] m/s^2
    PASS lateral_acceleration           worst     0.000 m/s^2  at   0.00s  bound |a| <= 1.5 m/s^2

PASS curved_lane_keeping            expect pass  5 assertions   20.00 s   401 steps  ended on max_time
    PASS no_collision                   worst    35.492 m      at  20.00s  bound > 0 m
    PASS min_distance                   worst    35.492 m      at  20.00s  bound >= 1 m
    PASS lateral_acceleration           worst     1.231 m/s^2  at   8.40s  bound |a| <= 2.5 m/s^2
    PASS longitudinal_acceleration      worst     0.894 m/s^2  at   0.00s  bound in [-3, 2] m/s^2
    PASS speed_limit                    worst    13.588 m/s    at   8.40s  bound <= 15.1 m/s

PASS cut_in_moderate                expect pass  6 assertions   20.00 s   401 steps  ended on max_time
    PASS no_collision                   worst    24.799 m      at  20.00s  bound > 0 m
    PASS min_distance                   worst    24.799 m      at  20.00s  bound >= 0.8 m
    PASS min_time_to_collision          worst    15.738 s      at   2.10s  bound >= 1.5 s
    PASS longitudinal_acceleration      worst    -1.157 m/s^2  at   3.30s  bound in [-3.5, 2] m/s^2
    PASS lateral_acceleration           worst     0.000 m/s^2  at   0.00s  bound |a| <= 1.5 m/s^2
    PASS speed_limit                    worst    13.792 m/s    at   3.30s  bound <= 14 m/s

PASS follower_tailgate              expect pass  5 assertions   20.00 s   401 steps  ended on max_time
    PASS no_collision                   worst     5.546 m      at  19.50s  bound > 0 m
    PASS min_distance                   worst     5.546 m      at  19.50s  bound >= 0.5 m
    PASS min_time_to_collision          worst     5.159 s      at   7.70s  bound >= 1.2 s
    PASS longitudinal_acceleration      worst    -1.161 m/s^2  at   7.00s  bound in [-3.5, 2] m/s^2
    PASS speed_limit                    worst    13.023 m/s    at   3.05s  bound <= 14 m/s

PASS lead_vehicle_braking           expect pass  5 assertions   20.00 s   401 steps  ended on max_time
    PASS no_collision                   worst     1.090 m      at  17.20s  bound > 0 m
    PASS min_distance                   worst     1.090 m      at  17.20s  bound >= 0.8 m
    PASS min_time_to_collision          worst     1.812 s      at  10.85s  bound >= 1.5 s
    PASS longitudinal_acceleration      worst    -2.726 m/s^2  at   8.35s  bound in [-4, 2] m/s^2
    PASS speed_limit                    worst    13.000 m/s    at   0.00s  bound <= 14 m/s

PASS merge_from_ramp                expect pass  5 assertions   22.00 s   441 steps  ended on max_time
    PASS no_collision                   worst    21.184 m      at   8.90s  bound > 0 m
    PASS min_distance                   worst    21.184 m      at   8.90s  bound >= 0.8 m
    PASS min_time_to_collision          worst     8.011 s      at   3.05s  bound >= 1.5 s
    PASS longitudinal_acceleration      worst    -3.370 m/s^2  at   4.25s  bound in [-4, 2] m/s^2
    PASS goal_reached                   worst    16.900 s      at  16.90s  bound <= 21 s

PASS speed_limit_compliance         expect pass  4 assertions   20.00 s   401 steps  ended on max_time
    PASS speed_limit                    worst    13.900 m/s    at  20.00s  bound <= 14 m/s
    PASS longitudinal_acceleration      worst     1.318 m/s^2  at   0.00s  bound in [-3, 2] m/s^2
    PASS no_collision                   worst       inf m      at   n/a    bound > 0 m
    PASS goal_reached                   worst    17.950 s      at  17.95s  bound <= 19.5 s

PASS stopped_obstacle               expect pass  5 assertions   25.00 s   501 steps  ended on max_time
    PASS no_collision                   worst     1.566 m      at  16.30s  bound > 0 m
    PASS min_distance                   worst     1.566 m      at  16.30s  bound >= 1 m
    PASS min_time_to_collision          worst     1.964 s      at  11.40s  bound >= 1.5 s
    PASS longitudinal_acceleration      worst    -1.647 m/s^2  at   9.65s  bound in [-4, 2] m/s^2
    PASS speed_limit                    worst    13.500 m/s    at   0.00s  bound <= 14 m/s

FAIL stopped_obstacle_no_brake      expect fail  5 assertions    7.30 s   147 steps  ended on collision
    FAIL no_collision                   worst    -0.408 m      at   7.30s  bound > 0 m
    FAIL min_distance                   worst    -0.408 m      at   7.30s  bound >= 1 m
    FAIL min_time_to_collision          worst     0.000 s      at   7.30s  bound >= 1.5 s
    FAIL goal_reached                   worst     7.300 s      at   7.30s  bound <= 18 s
    PASS speed_limit                    worst    13.000 m/s    at   0.00s  bound <= 14 m/s

PASS straight_cruise                expect pass  5 assertions   19.50 s   391 steps  ended on goal
    PASS no_collision                   worst       inf m      at   n/a    bound > 0 m
    PASS speed_limit                    worst    13.895 m/s    at  19.50s  bound <= 14 m/s
    PASS longitudinal_acceleration      worst     1.246 m/s^2  at   0.00s  bound in [-3, 2] m/s^2
    PASS lateral_acceleration           worst     0.000 m/s^2  at   0.00s  bound |a| <= 1.5 m/s^2
    PASS goal_reached                   worst    19.500 s      at  19.50s  bound <= 22 s

failing scenarios:
  aggressive_cut_in: min_time_to_collision, longitudinal_acceleration
  stopped_obstacle_no_brake: no_collision, min_distance, min_time_to_collision, goal_reached

literal exit code 1, expectation exit code 0
```

The two failing scenarios fail differently, which is the point of shipping both.
`stopped_obstacle_no_brake` runs the deliberately unsafe `constant_speed` controller into a
stationary vehicle: clearance reaches -0.408 m at t = 7.30 s, the run terminates on contact, and
four of its five assertions fail. `aggressive_cut_in` runs the normal controller against a cut in
that is too close to absorb comfortably: no contact occurs, so `no_collision` and `min_distance`
both hold with 1.652 m to spare, but time to collision falls to 0.707 s against a 1.5 s bound and
the braking demand saturates at -8.000 m/s^2 against a comfort band of [-3, 2] m/s^2. A harness
that reported only crashes would call this second run clean.

Two of the passing numbers are worth reading as checks rather than as results.
`curved_lane_keeping` reports a peak lateral acceleration of 1.231 m/s^2 at t = 8.40 s and a peak
speed of 13.588 m/s at the same instant; on a 150 m radius, `v^2 / R` is 1.2309 m/s^2, so the
simulator and the metric agree with the closed form to four figures. `speed_limit_compliance`
configures a desired speed of 20.0 m/s on a 13.9 m/s road and reports a peak of 13.900 m/s, which
is the controller respecting the limit exactly rather than approximately.

Baseline comparison, run against a deliberately truncated suite so that both categories of
difference appear:

```bash
uv run python -m scenario_runner run scenarios/*.toml --json outputs/truncated.json \
    --max-steps 120 --quiet
uv run python -m scenario_runner compare outputs/truncated.json baselines/reference_suite.json
```

```
comparison against baseline: 29 difference(s)

  new_failure        merge_from_ramp.goal_reached                     verdict moved from pass to fail, worst value 16.9 to 6
  new_failure        speed_limit_compliance.goal_reached              verdict moved from pass to fail, worst value 17.95 to 6
  new_failure        straight_cruise.goal_reached                     verdict moved from pass to fail, worst value 19.5 to 6
  resolved_failure   stopped_obstacle_no_brake.min_distance           verdict moved from fail to pass, worst value -0.408205 to 16.4918
  resolved_failure   stopped_obstacle_no_brake.no_collision           verdict moved from fail to pass, worst value -0.408205 to 16.4918
  metric_moved       curved_lane_keeping.lateral_acceleration         verdict held at pass, worst value 1.23085 to 1.21473 (-1.310 percent)
  metric_moved       lead_vehicle_braking.min_time_to_collision       verdict held at pass, worst value 1.81197 to 6.53204 (+260.494 percent)
  metric_moved       stopped_obstacle.longitudinal_acceleration       verdict held at pass, worst value -1.64704 to -0.917018 (+44.323 percent)
  ...

new failures 3, moved metrics 24, regressed True
tolerance: relative 1e-06, absolute 1e-09
```

An unmodified run against the same baseline reports no differences and exits zero.

The full suite of 122 tests, including the ten scenario runs, the recorded baseline comparison, and
five example scripts executed as subprocesses, completes in about four seconds on one core.

## Architecture

Five layers, each depending only on those below it. The dependency direction is enforced by the
import graph, not by convention: the algorithm layer reads a run through a structural protocol
(`algorithm/trace_view.py`) rather than by importing the simulator that produces it.

| Module | Responsibility |
| --- | --- |
| `src/scenario_runner/model/scenario.py` | Typed, frozen scenario dataclasses and the supported format versions |
| `src/scenario_runner/model/parser.py` | TOML parsing and strict validation, one located error per rejection |
| `src/scenario_runner/model/locate.py` | Recovery of a source line number for a dotted field path |
| `src/scenario_runner/model/serialise.py` | Rendering a validated scenario back to TOML for the round trip |
| `src/scenario_runner/model/geometry.py` | Road frame conversions for a straight or constant curvature road |
| `src/scenario_runner/model/errors.py` | The located `ScenarioError` raised by everything above |
| `src/scenario_runner/algorithm/metrics.py` | Disc cover clearance and two dimensional time to collision |
| `src/scenario_runner/algorithm/assertions.py` | The assertion vocabulary, each reporting worst value and time |
| `src/scenario_runner/algorithm/compare.py` | Baseline comparison and the magnitude class used by the baseline |
| `src/scenario_runner/algorithm/trace_view.py` | The structural view of a run the scoring code requires |
| `src/scenario_runner/pipeline/dynamics.py` | Kinematic bicycle integration and the Intelligent Driver Model |
| `src/scenario_runner/pipeline/controllers.py` | The two reference controllers and their registry |
| `src/scenario_runner/pipeline/actors.py` | Scripted and reactive actor state in road coordinates |
| `src/scenario_runner/pipeline/simulator.py` | The deterministic fixed step loop |
| `src/scenario_runner/pipeline/trace.py` | The structured trace and its recorder |
| `src/scenario_runner/pipeline/runner.py` | Per scenario results, suite results, and the two exit code rules |
| `src/scenario_runner/analysis/report.py` | Text rendering of suite results and comparisons |
| `src/scenario_runner/analysis/results.py` | Strict JSON encoding and decoding of suite results |
| `src/scenario_runner/analysis/figures.py` | Three panel matplotlib summary of one run |
| `src/scenario_runner/analysis/export.py` | Versioned JSON trace for the browser playback |
| `src/scenario_runner/cli.py` | Argument parsing and wiring for `run` and `compare` |
| `viz/` | Self-contained TypeScript and Canvas playback, never imported by `src/` or `tests/` |

`scenarios/` holds the shipped suite, `baselines/reference_suite.json` the recorded baseline, and
`examples/` five wiring scripts with no logic of their own.

The `viz/` directory is additive. It reads a JSON trace written by `analysis/export.py`, has no
runtime dependencies and no content delivery network reference, and is never imported by the Python
package or its tests; a test asserts that no file under `src/` or `tests/` mentions it. See
[viz/README.md](viz/README.md) for the build.

## Testing

```bash
uv run pytest
uv run ruff check .
uv run mypy
```

The suite has three tiers.

Tier one covers properties and invariants. A valid scenario round trips through parse and
serialise, and so does every scenario in the shipped suite. Twenty two malformed documents are each
rejected with a message that names the offending field, checked against a table that must match the
fixture directory exactly, so a parser that rejects a document for the wrong reason is caught.
An unknown format version is refused with the versions this build accepts, and a field introduced
in 1.1 is refused inside a 1.0 document. Every assertion type is scored against a hand constructed
trace whose correct verdict is known by inspection, and each is scored a second time with the
observed value placed exactly on its threshold. The runner is checked to return a non-zero exit
code by running the two scenarios that are expected to fail. The simulator is checked to be
identical across seeds when perception noise is off, identical for a repeated seed when it is on,
and different across seeds when it is on, so the determinism test is not vacuous. The baseline
comparison is checked to detect a newly failing assertion and, separately, a metric that moved
without changing a verdict.

Tier two compares the whole suite against the recorded baseline in `baselines/reference_suite.json`
with a numeric tolerance. What it pins, and what it deliberately does not pin, is the subject of
`docs/design-notes.md`.

Tier three runs every script in `examples/` as a subprocess under a reduced step count and requires
each to exit zero.

## References

Scenario description and safety assessment:

- ASAM e.V. *ASAM OpenSCENARIO XML*, specification 1.2.0, 2022. Stable URL:
  <https://www.asam.net/standards/detail/openscenario-xml/>. The document structure here follows
  its separation of road, entities, storyboard, and stop trigger.
- Menzel, T., Bagschik, G., Maurer, M. "Scenarios for Development, Test and Validation of Automated
  Vehicles." *IEEE Intelligent Vehicles Symposium*, 2018, pp. 1821-1827. DOI:
  [10.1109/IVS.2018.8500406](https://doi.org/10.1109/IVS.2018.8500406). Source of the functional,
  logical, and concrete scenario distinction; every document here is a concrete scenario.
- Riedmaier, S., Ponn, T., Ludwig, D., Schick, B., Diermeyer, F. "Survey on Scenario-Based Safety
  Assessment of Automated Vehicles." *IEEE Access*, 8, 2020, pp. 87456-87477. DOI:
  [10.1109/ACCESS.2020.2993730](https://doi.org/10.1109/ACCESS.2020.2993730).
- Kalra, N., Paddock, S. M. "Driving to safety: How many miles of driving would it take to
  demonstrate autonomous vehicle reliability?" *Transportation Research Part A: Policy and
  Practice*, 94, 2016, pp. 182-193. DOI:
  [10.1016/j.tra.2016.09.010](https://doi.org/10.1016/j.tra.2016.09.010). The reason a scenario
  suite is evidence about its own scenarios and nothing more.

Safety metrics:

- Hayward, J. C. "Near-miss determination through use of a scale of danger." *Highway Research
  Record*, 384, Highway Research Board, 1972, pp. 24-34. Stable URL:
  <https://onlinepubs.trb.org/Onlinepubs/hrr/1972/384/384-004.pdf>. The constant velocity time to
  collision definition implemented in `algorithm/metrics.py`.
- Shalev-Shwartz, S., Shammah, S., Shashua, A. "On a Formal Model of Safe and Scalable Self-Driving
  Cars." arXiv preprint, 2017. DOI:
  [10.48550/arXiv.1708.06374](https://doi.org/10.48550/arXiv.1708.06374). Responsibility sensitive
  safety, the source of the longitudinal and lateral safe distance framing the `min_distance`
  assertion approximates. The full formal model is not implemented here; see the design notes.
- Ziegler, J., Stiller, C. "Fast collision checking for intelligent vehicle motion planning."
  *IEEE Intelligent Vehicles Symposium*, 2010, pp. 518-522. DOI:
  [10.1109/IVS.2010.5547976](https://doi.org/10.1109/IVS.2010.5547976). The disc cover
  approximation of a rectangular footprint used for clearance and collision.

Vehicle and driver models:

- Treiber, M., Hennecke, A., Helbing, D. "Congested traffic states in empirical observations and
  microscopic simulations." *Physical Review E*, 62(2), 2000, pp. 1805-1824. DOI:
  [10.1103/PhysRevE.62.1805](https://doi.org/10.1103/PhysRevE.62.1805). The Intelligent Driver
  Model used by the reference controller and by reactive actors.
- Kong, J., Pfeiffer, M., Schildbach, G., Borrelli, F. "Kinematic and dynamic vehicle models for
  autonomous driving control design." *IEEE Intelligent Vehicles Symposium*, 2015, pp. 1094-1099.
  DOI: [10.1109/IVS.2015.7225830](https://doi.org/10.1109/IVS.2015.7225830). The kinematic bicycle
  model and the conditions under which it is an adequate substitute for a dynamic model.
- Coulter, R. C. *Implementation of the Pure Pursuit Path Tracking Algorithm.* Technical Report
  CMU-RI-TR-92-01, Robotics Institute, Carnegie Mellon University, 1992. Stable URL:
  <https://www.ri.cmu.edu/publications/implementation-of-the-pure-pursuit-path-tracking-algorithm/>.
  The lane keeping law.

Dependencies:

| Package | Purpose | Licence |
| --- | --- | --- |
| [numpy](https://numpy.org/) | Trace storage and the vectorised clearance and time to collision queries | BSD-3-Clause |
| [matplotlib](https://matplotlib.org/) | The three panel run summary figure, rendered through the Agg canvas | Matplotlib licence, BSD compatible |
| [pytest](https://docs.pytest.org/) | Test runner, development only | MIT |
| [ruff](https://docs.astral.sh/ruff/) | Linting, development only | MIT |
| [mypy](https://mypy-lang.org/) | Static type checking in strict mode, development only | MIT |
| [typescript](https://www.typescriptlang.org/) | Compiles `viz/src` to ES modules, development only, not a runtime dependency of anything | Apache-2.0 |

Scenario parsing uses the standard library `tomllib` and adds no dependency. The `viz/` layer has
no runtime dependencies at all.

## License

Released under the MIT license. See [LICENSE](LICENSE).

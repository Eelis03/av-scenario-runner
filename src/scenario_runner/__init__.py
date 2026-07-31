"""Declarative scenario format and regression harness for autonomous driving controllers.

The package is arranged in five layers, each depending only on those below it:

``model``
    Scenario schema, typed dataclasses, validation, serialisation.
``algorithm``
    Safety metrics, the assertion vocabulary, and baseline comparison.
``pipeline``
    Deterministic simulator, structured trace, and suite runner.
``analysis``
    Text reports, machine readable results, figures, and trace export.
``cli``
    Argument parsing and wiring, no logic of its own.
"""

from __future__ import annotations

from scenario_runner.model import Scenario, ScenarioError, load_scenario, parse_scenario
from scenario_runner.pipeline import ScenarioResult, SuiteResult, run_suite, simulate

__all__ = [
    "Scenario",
    "ScenarioError",
    "ScenarioResult",
    "SuiteResult",
    "__version__",
    "load_scenario",
    "parse_scenario",
    "run_suite",
    "simulate",
]

__version__ = "0.1.0"

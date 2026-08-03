"""Report rendering, machine readable results, figures, and trace export.

This layer presents what the layers below computed. It performs no simulation
and makes no verdicts of its own.
"""

from __future__ import annotations

from scenario_runner.analysis.export import TRACE_VERSION, dump_trace, trace_to_json
from scenario_runner.analysis.figures import (
    BoundUse,
    bound_utilisation,
    plot_bound_utilisation,
    plot_encounter,
    plot_run,
    plot_safety_timeline,
)
from scenario_runner.analysis.report import render_comparison, render_scenario, render_suite
from scenario_runner.analysis.results import (
    RESULT_VERSION,
    dump_suite_result,
    load_suite_record,
    load_suite_result,
    suite_from_json,
    suite_to_json,
)

__all__ = [
    "RESULT_VERSION",
    "TRACE_VERSION",
    "BoundUse",
    "bound_utilisation",
    "dump_suite_result",
    "dump_trace",
    "load_suite_record",
    "load_suite_result",
    "plot_bound_utilisation",
    "plot_encounter",
    "plot_run",
    "plot_safety_timeline",
    "render_comparison",
    "render_scenario",
    "render_suite",
    "suite_from_json",
    "suite_to_json",
    "trace_to_json",
]

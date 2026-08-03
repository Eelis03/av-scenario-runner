"""Tier three: every example script runs to completion under a reduced step count."""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

import pytest

from tests.conftest import EXAMPLES_DIR, REPO_ROOT

MAX_STEPS = "40"


def _example_scripts() -> list[Path]:
    return sorted(path for path in EXAMPLES_DIR.glob("*.py"))


def test_every_example_is_discovered() -> None:
    """The integration tier must actually have scripts to run."""
    assert len(_example_scripts()) >= 4


@pytest.mark.parametrize("script", _example_scripts(), ids=lambda path: path.stem)
def test_example_runs_to_completion(script: Path) -> None:
    """Each example exits zero and prints something under a reduced step count."""
    completed = subprocess.run(
        [sys.executable, str(script), "--max-steps", MAX_STEPS],
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
        timeout=120,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.strip()


def test_a_truncated_export_does_not_overwrite_a_stored_trace() -> None:
    """Running the examples under a step cap must leave the shipped traces alone."""
    stored = REPO_ROOT / "viz" / "traces" / "cut_in_moderate.json"
    before = stored.read_bytes()
    completed = subprocess.run(
        [
            sys.executable,
            str(EXAMPLES_DIR / "export_viz_trace.py"),
            "--scenario",
            "cut_in_moderate",
            "--max-steps",
            MAX_STEPS,
        ],
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
        timeout=120,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    assert stored.read_bytes() == before
    assert "outputs" in completed.stdout


def test_a_truncated_figure_run_does_not_overwrite_a_published_figure() -> None:
    """The published figures come from full runs, so a step cap must not touch them."""
    published = sorted((REPO_ROOT / "docs" / "figures").glob("*.png"))
    assert published, "no published figure to protect"
    before = {path.name: path.read_bytes() for path in published}
    completed = subprocess.run(
        [sys.executable, str(EXAMPLES_DIR / "make_figures.py"), "--max-steps", MAX_STEPS],
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
        timeout=180,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    assert {path.name: path.read_bytes() for path in published} == before
    assert "outputs" in completed.stdout


def test_python_sources_never_reference_the_viz_layer() -> None:
    """The package and its tests must pass with ``viz/`` absent entirely."""
    pattern = re.compile(r"^\s*(?:import|from)\s+viz\b", re.MULTILINE)
    offenders: list[str] = []
    for root in (REPO_ROOT / "src", REPO_ROOT / "tests"):
        for path in root.rglob("*.py"):
            if pattern.search(path.read_text(encoding="utf-8")):
                offenders.append(str(path))
    assert not offenders, offenders


def test_the_command_line_entry_point_runs_the_suite() -> None:
    """``python -m scenario_runner`` is a working entry point, not only an import."""
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "scenario_runner",
            "run",
            str(REPO_ROOT / "scenarios" / "straight_cruise.toml"),
            "--max-steps",
            MAX_STEPS,
        ],
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
        timeout=120,
        check=False,
    )
    assert "straight_cruise" in completed.stdout

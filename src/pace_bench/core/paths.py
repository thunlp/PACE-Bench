"""Central path resolution for installed and source-checkout use."""

from __future__ import annotations

import os
from pathlib import Path

# This module lives under ``pace_bench/core``; package data remains one level up.
PACKAGE_ROOT = Path(__file__).resolve().parent.parent
TASKS_ROOT = PACKAGE_ROOT / "tasks"
TASK_CATEGORIES_ROOT = TASKS_ROOT / "categories"
TASK_DEMOS_ROOT = TASKS_ROOT / "demos"


def repository_root() -> Path | None:
    """Return the checkout root when running from a source tree."""

    for parent in PACKAGE_ROOT.parents:
        if (parent / "src").is_dir() and (parent / "pyproject.toml").is_file():
            return parent
    return None


def default_output_root() -> Path:
    """Return the configured output root without creating it."""

    override = os.environ.get("PACE_BENCH_OUTPUT_DIR", "").strip()
    if override:
        return Path(override).expanduser().resolve()
    root = repository_root()
    return (root / "results" if root else Path.cwd() / "results").resolve()


def ensure_output_path(path: str | Path | None) -> Path:
    """Resolve and create a run output directory."""

    resolved = Path(path).expanduser().resolve() if path else default_output_root()
    resolved.mkdir(parents=True, exist_ok=True)
    return resolved

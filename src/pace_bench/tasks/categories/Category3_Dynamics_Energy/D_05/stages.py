from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

import re

from typing import Any, Dict, List

_DEFAULT_SHELL_BREAK_FORCE = 5000.0

_DEFAULT_GRAVITY = (0, -10.0)

_INVISIBLE_ENV_WARNING = """

Physical conditions in this stage may differ from the default.
"""

def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,

) -> str:
    description = base_description
    target_terrain_config = target_terrain_config or {}
    base_terrain_config = base_terrain_config or {}
    target_break = float(target_terrain_config.get("shell_break_force", _DEFAULT_SHELL_BREAK_FORCE))
    base_break = float(base_terrain_config.get("shell_break_force", _DEFAULT_SHELL_BREAK_FORCE))
    if target_break != base_break:
        pattern = r"(Delivers a strike with enough kinetic energy and )force\s*[\(]?(?:≥|>=|>)\s*[\d.]+\s*N[\)]?( to break the shell\.)"
        replacement = (
            r"\1force ≥ " + f"{target_break:.0f} N (originally {base_break:.0f} N in the source environment)" + r"\2"
        )
        description, count = re.subn(pattern, replacement, description, count=1)
        if count != 1:
            raise ValueError(f"D_05 expected one shell-force prompt target; found {count}")
    return description

def update_success_criteria_for_visible_changes(base_success_criteria: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]) -> str:
    return base_success_criteria

_D05_UNIFORM_SUFFIX = uniform_suffix_for_task("D_05")

def get_d05_curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Stage-1",
            "title": "Harder Shell",
            "mutation_description": "Shell break threshold increased (16000 N). Original impact does not break.",
            "task_description_suffix": uniform_suffix_for_task("D_05"),
            "terrain_config": {"shell_break_force": 14000.0},
            "physics_config": {},
        },
        {
            "stage_id": "Stage-2",
            "title": "Shifted Slot Bar Phase",
            "mutation_description": "Slot oscillating bar omega 0.014; safe window at step ~336. Original 380/398/408 timing hits bar.",
            "task_description_suffix": uniform_suffix_for_task("D_05"),
            "terrain_config": {"slot_bar_omega": 0.014},
            "physics_config": {},
        },
        {
            "stage_id": "Stage-3",
            "title": "Harder Shell and Damping",
            "mutation_description": "Shell break 13000 N + angular damping 0.6. Less kinetic energy at impact; original swing insufficient.",
            "task_description_suffix": uniform_suffix_for_task("D_05"),
            "terrain_config": {"shell_break_force": 13000.0},
            "physics_config": {"angular_damping": 0.6},
        },
        {
            "stage_id": "Stage-4",
            "title": "Gravity, Shell, Bar Phase and Damping",
            "mutation_description": "Gravity -14, shell 11000 N, slot_bar_omega 0.013, angular_damping 0.35. Multi-parameter; original timing and impact fail.",
            "task_description_suffix": uniform_suffix_for_task("D_05"),
            "terrain_config": {"shell_break_force": 11000.0, "slot_bar_omega": 0.013},
            "physics_config": {"gravity": (0, -14.0), "angular_damping": 0.35},
        },
    ]

from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List

_DEFAULT_OVERHEAT_LIMIT = 72000.0


def _replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise ValueError(f"{label}: expected exactly one match, found {count}")
    return text.replace(old, new, 1)


def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,
    *,
    stage: Dict[str, Any] = None,

) -> str:
    description = base_description
    target_physics_config = target_physics_config or {}
    base_physics_config = base_physics_config or {}
    if stage is not None:
        target_physics_config = dict(stage.get("physics_config") or {})
        base_physics_config = {}
    target_overheat = float(target_physics_config.get("overheat_limit", _DEFAULT_OVERHEAT_LIMIT))
    base_overheat = float(base_physics_config.get("overheat_limit", _DEFAULT_OVERHEAT_LIMIT))
    if target_overheat != base_overheat:
        old_line = (
            f"The overheat limit is {base_overheat:.0f} N·s; "
            "exceeding it causes mission failure."
        )
        new_line = (
            f"The overheat limit is {target_overheat:.0f} N·s "
            f"(originally {base_overheat:.0f} N·s in the source environment); "
            "exceeding it causes mission failure."
        )
        description = _replace_once(
            description, old_line, new_line, "task overheat limit"
        )
    return description

def update_success_criteria_for_visible_changes(
    base_success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,
    *,
    stage: Dict[str, Any] = None,

) -> str:
    criteria = base_success_criteria
    target_physics_config = target_physics_config or {}
    base_physics_config = base_physics_config or {}
    if stage is not None:
        target_physics_config = dict(stage.get("physics_config") or {})
        base_physics_config = {}
    target_overheat = float(target_physics_config.get("overheat_limit", _DEFAULT_OVERHEAT_LIMIT))
    base_overheat = float(base_physics_config.get("overheat_limit", _DEFAULT_OVERHEAT_LIMIT))
    if target_overheat != base_overheat:
        old_line = (
            "2. **Thermal Safety**: Craft heat stays below the overheat "
            f"limit ({base_overheat:.0f} N·s)."
        )
        new_line = (
            "2. **Thermal Safety**: Craft heat stays below the overheat "
            f"limit ({target_overheat:.0f} N·s; originally "
            f"{base_overheat:.0f} N·s in the source environment)."
        )
        criteria = _replace_once(
            criteria, old_line, new_line, "success overheat limit"
        )
    return criteria

UNIFORM_SUFFIX = uniform_suffix_for_task("E_02")

def get_e02_curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Stage-1",
            "title": "Environment Variation 1",
            "mutation_description": "A subset of the listed environmental properties differs from the source environment.",
            "task_description_suffix": uniform_suffix_for_task("E_02"),
            "terrain_config": {},
            "physics_config": {
                "linear_damping": 12.0,
                "constant_force_x": -180.0,
                "overheat_limit": 30000.0,
            },
        },
        {
            "stage_id": "Stage-2",
            "title": "Environment Variation 2",
            "mutation_description": "A subset of the listed environmental properties differs from the source environment.",
            "task_description_suffix": uniform_suffix_for_task("E_02"),
            "terrain_config": {},
            "physics_config": {
                "constant_force_y": 500.0,
            },
        },
        {
            "stage_id": "Stage-3",
            "title": "Environment Variation 3",
            "mutation_description": "A subset of the listed environmental properties differs from the source environment.",
            "task_description_suffix": uniform_suffix_for_task("E_02"),
            "terrain_config": {},
            "physics_config": {
                "linear_damping": 7.0,
                "drain_velocity_factor": 0.0,
                "slip_backward_force": -520.0,
                "overheat_limit": 9000.0,
                "constant_force_x": 260.0,
                "constant_force_y": -450.0,
                "wind_amplitude": 85.0,
                "wind_omega": 0.23,
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "Environment Variation 4",
            "mutation_description": "A subset of the listed environmental properties differs from the source environment.",
            "task_description_suffix": uniform_suffix_for_task("E_02"),
            "terrain_config": {},
            "physics_config": {
                "linear_damping": 4.5,
                "constant_force_x": -48.0,
                "slip_backward_force": -75.0,
                "overheat_limit": 6000.0,
                "wind_amplitude": 105.0,
                "wind_omega": 0.48,
            },
        },
    ]

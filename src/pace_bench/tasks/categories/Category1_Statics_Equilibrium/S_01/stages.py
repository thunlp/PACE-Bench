from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List

import re


def _replace_once(
    text: str,
    pattern: str,
    replacement: Any,
    *,
    field: str,
    flags: int = 0,
) -> str:
    updated, count = re.subn(pattern, replacement, text, flags=flags)
    if count != 1:
        raise ValueError(
            f"S_01 prompt update for {field} expected exactly one match, got {count}"
        )
    return updated


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
    default_gap_width = 15.0
    default_max_structure_mass = 2000.0
    target_terrain_config = target_terrain_config or {}
    base_terrain_config = base_terrain_config or {}
    target_physics_config = target_physics_config or {}
    base_physics_config = base_physics_config or {}
    if stage is not None:
        target_physics_config = dict(stage.get("physics_config") or {})
        base_physics_config = {}
    target_gap_width = target_terrain_config.get("gap_width", default_gap_width)
    base_gap_width = base_terrain_config.get("gap_width", default_gap_width)
    target_right_cliff_start = 10.0 + target_gap_width
    base_right_cliff_start = 10.0 + base_gap_width
    target_max_mass = target_terrain_config.get("max_structure_mass", default_max_structure_mass)
    base_max_mass = base_terrain_config.get("max_structure_mass", default_max_structure_mass)
    base_target_x = base_right_cliff_start + 5.0
    target_x = target_right_cliff_start + 5.0
    if target_gap_width != base_gap_width:
        right_cliff_pattern = r"(- \*\*Right Cliff\*\*: Starts at x=)(\d+\.?\d*)(m.*)$"
        description = _replace_once(
            description,
            right_cliff_pattern,
            lambda m: f"{m.group(1)}{target_right_cliff_start:.1f}m (originally {base_right_cliff_start:.1f}m in the source environment){m.group(3)[1:]}",
            field="right cliff",
            flags=re.MULTILINE,
        )
        build_zone_pattern = r"(- \*\*Build Zone\*\*: Structure must be built within x=\[10, )(\d+\.?\d*)(\], y=\[5, 15\] \([^)]+\)\.)"
        description = _replace_once(
            description,
            build_zone_pattern,
            lambda m: f"{m.group(1)}{target_x:.1f}] (originally [10, {base_target_x:.1f}] in the source environment), y=[5, 15] (the upper x-bound is the target position so the deck can reach the goal).",
            field="build zone",
        )
        target_desc_pattern = r"(- \*\*Target\*\*: The vehicle must fully cross the gap and reach at least x=)(\d+\.?\d*)m( on the right side.)"
        description = _replace_once(
            description,
            target_desc_pattern,
            lambda m: f"{m.group(1)}{target_x:.1f}m (originally {base_target_x:.1f}m in the source environment){m.group(3)}",
            field="target",
        )
    if target_max_mass != base_max_mass:
        mass_desc_pattern = r"(- \*\*Mass Budget\*\*: Total structure mass must be at most )(\d+\.?\d*) kg\."
        description = _replace_once(
            description,
            mass_desc_pattern,
            f"\\g<1>{target_max_mass:.0f} kg (originally {base_max_mass:.0f} kg in the source environment).",
            field="mass budget",
        )
    for key, label, default in [
        ("joint_max_force", "Joint Strength", 80.0),
        ("joint_max_torque", "Joint Strength", 300.0),
        ("anchor_max_force", "Anchor Strength", 100.0),
        ("anchor_max_torque", "Anchor Strength", 500.0)
    ]:
        target_val = target_physics_config.get(key, default)
        base_val = base_physics_config.get(key, default)
        if target_val != base_val:
            if "force" in key:
                pattern = rf"(- \*\*{label}\*\*: Maximum linear force for .*? is )(\d+\.?\d*)[;.]"
                replacement = f"\\g<1>{target_val:.1f} (originally {base_val:.1f} in the source environment);"
            else:
                pattern = rf"(- \*\*{label}\*\*: .*? maximum torque is )(\d+\.?\d*)([;.] ?)"
                replacement = f"\\g<1>{target_val:.1f} (originally {base_val:.1f} in the source environment)\\g<3>"
            description = _replace_once(
                description,
                pattern,
                replacement,
                field=key,
                flags=re.IGNORECASE,
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
    default_gap_width = 15.0
    default_max_structure_mass = 2000.0
    target_terrain_config = target_terrain_config or {}
    base_terrain_config = base_terrain_config or {}
    target_physics_config = target_physics_config or {}
    base_physics_config = base_physics_config or {}
    if stage is not None:
        target_physics_config = dict(stage.get("physics_config") or {})
        base_physics_config = {}
    target_gap_width = target_terrain_config.get("gap_width", default_gap_width)
    base_gap_width = base_terrain_config.get("gap_width", default_gap_width)
    target_right_cliff_start = 10.0 + target_gap_width
    base_right_cliff_start = 10.0 + base_gap_width
    target_max_mass = target_terrain_config.get("max_structure_mass", default_max_structure_mass)
    base_max_mass = base_terrain_config.get("max_structure_mass", default_max_structure_mass)
    if target_gap_width != base_gap_width:
        base_target_x = base_right_cliff_start + 5.0
        target_x = target_right_cliff_start + 5.0
        target_pattern = r"(1\. \*\*Passage\*\*: Vehicle reaches x >= )(\d+\.?\d*)m\."
        criteria = _replace_once(
            criteria,
            target_pattern,
            f"\\g<1>{target_x:.1f}m (originally {base_target_x:.1f}m in the source environment).",
            field="passage target",
        )
    if target_max_mass != base_max_mass:
        mass_pattern = r"(- \*\*Mass Budget\*\*: <= )(\d+\.?\d*) kg\."
        criteria = _replace_once(
            criteria,
            mass_pattern,
            f"\\g<1>{target_max_mass:.0f} kg (originally {base_max_mass:.0f} kg in the source environment).",
            field="success mass budget",
        )
    for key, label, default in [
        ("joint_max_force", "Joint Strength", 80.0),
        ("joint_max_torque", "Joint Strength", 300.0),
        ("anchor_max_force", "Anchor Strength", 100.0),
        ("anchor_max_torque", "Anchor Strength", 500.0)
    ]:
        target_val = target_physics_config.get(key, default)
        base_val = base_physics_config.get(key, default)
        if target_val != base_val:
            if "force" in key:
                pattern = rf"(- \*\*{label}\*\*: Maximum linear force for .*? is )(\d+\.?\d*)[;.]"
                replacement = f"\\g<1>{target_val:.1f} (originally {base_val:.1f} in the source environment);"
            else:
                pattern = rf"(- \*\*{label}\*\*: .*? maximum torque is )(\d+\.?\d*)([;.] ?)"
                replacement = f"\\g<1>{target_val:.1f} (originally {base_val:.1f} in the source environment)\\g<3>"
            criteria = _replace_once(
                criteria,
                pattern,
                replacement,
                field=f"success {key}",
                flags=re.IGNORECASE,
            )
    return criteria

def get_s01_curriculum_stages() -> List[Dict[str, Any]]:
    UNIFORM_SUFFIX = uniform_suffix_for_task("S_01")
    return [
        {
            "stage_id": "Stage-1",
            "title": "Brittle Isostatic Threshold",
            "mutation_description": "Near-zero torque capacity forces a fully pin-jointed load path, while closely coupled mass, joint-force, and anchor-force ceilings leave almost no reserve for redundant members or concentrated reactions.",
            "task_description_suffix": uniform_suffix_for_task("S_01"),
            "terrain_config": {
                "max_structure_mass": 382.5,
            },
            "physics_config": {
                "joint_max_force": 22.0,
                "anchor_max_force": 28.0,
                "joint_max_torque": 0.02,
                "anchor_max_torque": 0.02,
            },
        },
        {
            "stage_id": "Stage-2",
            "title": "Paper Joints",
            "mutation_description": "A low joint-force threshold and limited anchor capacity require finer load distribution and multiple anchoring levels.",
            "task_description_suffix": uniform_suffix_for_task("S_01"),
            "terrain_config": {
                "max_structure_mass": 1000.0,
            },
            "physics_config": {
                "joint_max_force": 5.5,
                "anchor_max_force": 15.0,
            },
        },
        {
            "stage_id": "Stage-3",
            "title": "The Vortex Gorge",
            "mutation_description": "A wider span combines a lower mass allowance, altered gravity and wind, and low torque limits.",
            "task_description_suffix": uniform_suffix_for_task("S_01"),
            "terrain_config": {
                "gap_width": 20.0,
                "max_structure_mass": 550.0,
            },
            "physics_config": {
                "gravity": (0, -20.0),
                "wind_force": (-20.0, -4.0),
                "joint_max_torque": 0.2,
                "anchor_max_torque": 0.2,
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "Abyssal Crossing",
            "mutation_description": "A long span combines a low mass allowance, altered gravity and wind, and bounded joint and anchor loads.",
            "task_description_suffix": uniform_suffix_for_task("S_01"),
            "terrain_config": {
                "gap_width": 28.0,
                "max_structure_mass": 330.0,
            },
            "physics_config": {
                "gravity": (0, -28.0),
                "wind_force": (-45.0, -8.0),
                "joint_max_force": 40.0,
                "anchor_max_force": 60.0,
                "joint_max_torque": 80.0,
                "anchor_max_torque": 120.0,
            },
        },
    ]

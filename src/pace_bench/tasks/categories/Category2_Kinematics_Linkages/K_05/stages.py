from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List

import re

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
    target_y = target_terrain_config.get("target_object_y", 9.0)
    base_y = base_terrain_config.get("target_object_y", 9.0)
    default_sustain_s = 3.0
    default_max_structure_mass = 60.0
    target_sustain_s = float(target_terrain_config.get("min_sustain_s", default_sustain_s))
    base_sustain_s = float(base_terrain_config.get("min_sustain_s", default_sustain_s))
    target_max_mass = float(target_terrain_config.get("max_structure_mass", default_max_structure_mass))
    base_max_mass = float(base_terrain_config.get("max_structure_mass", default_max_structure_mass))
    if target_y != base_y:
        pattern = r"(at least y=)(\d+\.?\d*)m( \(originally [^)]+ in the source environment\))?"
        description = re.sub(pattern, f"\\g<1>{target_y:.1f}m (originally y={base_y:.1f}m in the source environment)", description)
        pattern_y_ge = r"(reaches y >= )(\d+\.?\d*)m( \(originally [^)]+ in the source environment\))?"
        description = re.sub(pattern_y_ge, f"\\g<1>{target_y:.1f}m (originally y >= {base_y:.1f}m in the source environment)", description)
    if target_sustain_s != base_sustain_s:
        sustain_pattern = r"(for at least )(\d+\.?\d*)( seconds \()"
        if re.search(sustain_pattern, description):
            description = re.sub(
                sustain_pattern,
                f"\\g<1>{target_sustain_s:.1f} seconds (originally {base_sustain_s:.1f} seconds in the source environment) (",
                description,
            )
    if target_max_mass != base_max_mass:
        mass_budget_pattern = r"(Total structure mass must be less than )(\d+\.?\d*)( kg\.)"
        if re.search(mass_budget_pattern, description):
            description = re.sub(
                mass_budget_pattern,
                f"\\g<1>{target_max_mass:.0f} kg (originally {base_max_mass:.0f} kg in the source environment).",
                description,
            )
    target_ceiling = target_terrain_config.get("ceiling_gap")
    base_ceiling = base_terrain_config.get("ceiling_gap")
    if target_ceiling:
        c_y = target_ceiling.get("y", 6.0)
        c_x_min = target_ceiling.get("x_min", 3.0)
        c_x_max = target_ceiling.get("x_max", 5.0)
        gap_width = c_x_max - c_x_min
        ceiling_new = f"Gap at y={c_y:.1f}m, x=[{c_x_min:.1f}, {c_x_max:.1f}] (gap width {gap_width:.1f}m)"
        if not base_ceiling:
            ceiling_originally = " (originally no ceiling in the source environment)"
        else:
            by = base_ceiling.get("y", 6.0)
            bx_min = base_ceiling.get("x_min", 3.0)
            bx_max = base_ceiling.get("x_max", 5.0)
            ceiling_originally = f" (originally gap at y={by:.1f}m, x=[{bx_min:.1f}, {bx_max:.1f}] in the source environment)"
        ceiling_pattern_none = r"(- \*\*Ceiling\*\*: )None \(no vertical obstacle\)\."
        ceiling_pattern_none_with_orig = r"(- \*\*Ceiling\*\*: )None \(no vertical obstacle\) \(originally [^)]+\ in the source environment\)\."
        ceiling_pattern_gap = r"(- \*\*Ceiling\*\*: )Gap at y=[\d.]+m, x=\[[\d.]+, [\d.]+\] \(gap width [\d.]+m\)( \(originally [^)]+\))?\."
        if re.search(ceiling_pattern_none_with_orig, description):
            description = re.sub(
                ceiling_pattern_none_with_orig,
                f"\\g<1>{ceiling_new} (originally no ceiling in the source environment).",
                description,
            )
        elif re.search(ceiling_pattern_none, description):
            description = re.sub(
                ceiling_pattern_none,
                f"\\g<1>{ceiling_new}{ceiling_originally}.",
                description,
            )
        elif re.search(ceiling_pattern_gap, description):
            description = re.sub(
                ceiling_pattern_gap,
                f"\\g<1>{ceiling_new}{ceiling_originally}.",
                description,
            )
    elif base_ceiling and not target_ceiling:
        by = base_ceiling.get("y", 6.0)
        bx_min = base_ceiling.get("x_min", 3.0)
        bx_max = base_ceiling.get("x_max", 5.0)
        ceiling_originally_gap = f"(originally gap at y={by:.1f}m, x=[{bx_min:.1f}, {bx_max:.1f}] in the source environment)"
        ceiling_pattern_gap = r"(- \*\*Ceiling\*\*: )Gap at y=[\d.]+m, x=\[[\d.]+, [\d.]+\] \(gap width [\d.]+m\)( \(originally [^)]+\))?\."
        ceiling_pattern_none_full = r"(- \*\*Ceiling\*\*: )None \(no vertical obstacle\) \(originally [^)]+\ in the source environment\)\."
        if re.search(ceiling_pattern_none_full, description):
            description = re.sub(
                ceiling_pattern_none_full,
                f"\\g<1>None (no vertical obstacle) {ceiling_originally_gap}.",
                description,
            )
        elif re.search(ceiling_pattern_gap, description):
            description = re.sub(
                ceiling_pattern_gap,
                f"\\g<1>None (no vertical obstacle) {ceiling_originally_gap}.",
                description,
            )
    default_max_joint_force = float("inf")
    target_max_joint_force = target_physics_config.get("max_joint_force", default_max_joint_force)
    base_max_joint_force = base_physics_config.get("max_joint_force", default_max_joint_force)
    if target_max_joint_force != base_max_joint_force and target_max_joint_force < float("inf"):
        joint_limit_pattern = r"(- \*\*Joint reaction limit\*\*: )Structural joints do not break under reaction force in the base environment\."
        if re.search(joint_limit_pattern, description):
            description = re.sub(
                joint_limit_pattern,
                f"\\g<1>Structural joints break if reaction force exceeds {target_max_joint_force:.0f} N (originally no limit in the source environment).",
                description,
            )
        else:
            joint_limit_numeric_pattern = r"(- \*\*Joint reaction limit\*\*: Structural joints break if reaction force exceeds )([\d.]+|inf)( N \(originally )([^)]+)( in the source environment\)\.)"
            if re.search(joint_limit_numeric_pattern, description):
                base_str_val = f"{base_max_joint_force:.0f} N" if base_max_joint_force < float("inf") else "no limit"
                description = re.sub(
                    joint_limit_numeric_pattern,
                    f"\\g<1>{target_max_joint_force:.0f}\\g<3>{base_str_val} in the source environment).",
                    description,
                )
    return description

def update_success_criteria_for_visible_changes(base_success_criteria: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]) -> str:
    criteria = base_success_criteria
    target_y = target_terrain_config.get("target_object_y", 9.0)
    base_y = base_terrain_config.get("target_object_y", 9.0)
    default_sustain_s = 3.0
    default_max_structure_mass = 60.0
    target_sustain_s = float(target_terrain_config.get("min_sustain_s", default_sustain_s))
    base_sustain_s = float(base_terrain_config.get("min_sustain_s", default_sustain_s))
    target_max_mass = float(target_terrain_config.get("max_structure_mass", default_max_structure_mass))
    base_max_mass = float(base_terrain_config.get("max_structure_mass", default_max_structure_mass))
    if target_y != base_y:
        pattern = r"(reaches y >= )(\d+\.?\d*)m( \(originally [^)]+ in the source environment\))?"
        criteria = re.sub(pattern, f"\\g<1>{target_y:.1f}m (originally y >= {base_y:.1f}m in the source environment)", criteria, flags=re.IGNORECASE)
    if target_sustain_s != base_sustain_s:
        sustain_pattern = r"(for >= )(\d+\.?\d*)( seconds \()"
        if re.search(sustain_pattern, criteria):
            criteria = re.sub(
                sustain_pattern,
                f"\\g<1>{target_sustain_s:.1f} seconds (originally {base_sustain_s:.1f} seconds in the source environment) (",
                criteria,
            )
    if target_max_mass != base_max_mass:
        mass_budget_pattern = r"(- \*\*Mass Budget\*\*: < )(\d+\.?\d*)( kg\.)"
        if re.search(mass_budget_pattern, criteria):
            criteria = re.sub(
                mass_budget_pattern,
                f"\\g<1>{target_max_mass:.0f} kg (originally {base_max_mass:.0f} kg in the source environment).",
                criteria,
            )
    return criteria

def get_k05_curriculum_stages() -> List[Dict[str, Any]]:
    task_description_suffix = uniform_suffix_for_task("K_05")
    return [
        {
            "stage_id": "Stage-1",
            "title": "Severe Hurricane Wind",
            "mutation_description": "Powerful lateral wind blows everything away.",
            "task_description_suffix": uniform_suffix_for_task("K_05"),
            "terrain_config": {},
            "physics_config": {"wind_force": (400.0, 0.0)},
        },
        {
            "stage_id": "Stage-2",
            "title": "Crushing Gravity",
            "mutation_description": "Gravitational acceleration massively increased — lifting requires extreme power-to-weight ratio as every gram exerts tremendous downward force.",
            "task_description_suffix": uniform_suffix_for_task("K_05"),
            "terrain_config": {},
            "physics_config": {"gravity": (0.0, -150.0)},
        },
        {
            "stage_id": "Stage-3",
            "title": "The Tipping Gauntlet",
            "mutation_description": "Ultra-heavy, off-balance, near-frictionless object with strong crosswind and tight mass budget — every design choice trades off against another constraint.",
            "task_description_suffix": uniform_suffix_for_task("K_05"),
            "terrain_config": {
                "target_object_y": 12.0,
                "max_structure_mass": 40.0,
                "object": {
                    "mass": 90.0,
                    "friction": 0.05,
                    "com_offset": (0.25, 0.15)
                }
            },
            "physics_config": {
                "wind_force": (200.0, 0.0),
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "The Master's Gauntlet",
            "mutation_description": "Combined wind, narrow gap, heavy load, and fragile joints.",
            "task_description_suffix": uniform_suffix_for_task("K_05"),
            "terrain_config": {
                "ceiling_gap": {"x_min": 3.2, "x_max": 4.8, "y": 6.0},
                "target_object_y": 10.0,
                "object": {"mass": 40.0, "friction": 0.2}
            },
            "physics_config": {
                "wind_force": (150.0, 0.0),
                "max_joint_force": 1500.0
            },
        },
    ]

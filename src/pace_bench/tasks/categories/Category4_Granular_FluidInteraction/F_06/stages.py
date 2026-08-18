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

) -> str:
    description = base_description
    target_physics_config = target_physics_config or {}
    base_physics_config = base_physics_config or {}
    target_y_min = target_terrain_config.get("target_y_min", 0.0)
    target_y_max = target_terrain_config.get("target_y_max", 1.5)
    base_y_min = base_terrain_config.get("target_y_min", 0.0)
    base_y_max = base_terrain_config.get("target_y_max", 1.5)
    if target_y_min != base_y_min or target_y_max != base_y_max:
        target_zone_y_pattern = r"(- \*\*Target Zone\*\*: x in \[\d+\.?\d*, \d+\.?\d*\] m, y in \[)(\d+\.?\d*)(, )(\d+\.?\d*)(\] m\.?)"
        if re.search(target_zone_y_pattern, description):
            description = re.sub(
                target_zone_y_pattern,
                f"\\g<1>{target_y_min:.1f}, {target_y_max:.1f}] m (originally y in [{base_y_min:.1f}, {base_y_max:.1f}] m in the source environment).",
                description,
            )
    default_x_min, default_x_max = 18.0, 22.0
    target_x_min = float(target_terrain_config.get("target_x_min", default_x_min))
    target_x_max = float(target_terrain_config.get("target_x_max", default_x_max))
    base_x_min = float(base_terrain_config.get("target_x_min", default_x_min))
    base_x_max = float(base_terrain_config.get("target_x_max", default_x_max))
    if target_x_min != base_x_min or target_x_max != base_x_max:
        target_zone_x_pattern = r"(- \*\*Target Zone\*\*: x in \[)(\d+\.?\d*)(, )(\d+\.?\d*)(\] m, y in )"
        if re.search(target_zone_x_pattern, description):
            description = re.sub(
                target_zone_x_pattern,
                f"\\g<1>{target_x_min:.1f}, {target_x_max:.1f}] m (originally x in [{base_x_min:.1f}, {base_x_max:.1f}] m in the source environment), y in ",
                description,
            )
    target_delivery = target_terrain_config.get("min_delivery_ratio", 0.90)
    base_delivery = base_terrain_config.get("min_delivery_ratio", 0.90)
    if target_delivery != base_delivery:
        pattern = r"(at least )(\d+)(% of released fluid particles(?: into the target zone)?)"
        description = re.sub(
            pattern,
            f"\\g<1>{int(target_delivery*100)}\\g<3> (originally {int(base_delivery*100)}% in the source environment)",
            description,
        )
    target_fluid = target_terrain_config.get("fluid", {})
    base_fluid = base_terrain_config.get("fluid", {})
    target_count = int(target_fluid.get("count", 60))
    base_count = int(base_fluid.get("count", 60))
    if target_count != base_count:
        pattern = r"(A batch of )(\d+)( small fluid particles)"
        if re.search(pattern, description):
            description = re.sub(
                pattern,
                f"\\g<1>{target_count}\\g<3> (originally {base_count} particles in the source environment)",
                description,
            )
    default_force_budget = 12000.0
    target_force = float(target_physics_config.get("force_budget", default_force_budget))
    base_force = float(base_physics_config.get("force_budget", default_force_budget))
    if target_force != base_force:
        pattern = r"(a per-step force budget of )(\d+)( N)"
        description = re.sub(
            pattern,
            f"\\g<1>{int(target_force)}\\g<3> (originally {int(base_force)} N in the source environment)",
            description,
        )
    return description

def update_success_criteria_for_visible_changes(
    base_success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,

) -> str:
    criteria = base_success_criteria
    target_physics_config = target_physics_config or {}
    base_physics_config = base_physics_config or {}
    target_delivery = target_terrain_config.get("min_delivery_ratio", 0.90)
    base_delivery = base_terrain_config.get("min_delivery_ratio", 0.90)
    if target_delivery != base_delivery:
        pattern = r"(At least )(\d+)(% of released particles reach the target zone\.)"
        criteria = re.sub(
            pattern,
            f"\\g<1>{int(target_delivery*100)}\\g<3> (originally {int(base_delivery*100)}% in the source environment)",
            criteria,
        )
    default_force_budget = 12000.0
    target_force = float(target_physics_config.get("force_budget", default_force_budget))
    base_force = float(base_physics_config.get("force_budget", default_force_budget))
    if target_force != base_force:
        pattern = r"(must not exceed )(\d+)( N per step\.)"
        if re.search(pattern, criteria):
            criteria = re.sub(
                pattern,
                f"\\g<1>{int(target_force)} N per step (originally {int(base_force)} N per step in the source environment).",
                criteria,
            )
        design_force_pattern = r"(- \*\*Force Budget\*\*: )(\d+)( N per step\.)"
        if re.search(design_force_pattern, criteria):
            criteria = re.sub(
                design_force_pattern,
                f"\\g<1>{int(target_force)} N per step (originally {int(base_force)} N per step in the source environment).",
                criteria,
            )
    default_max_mass = 380.0
    target_mass = float(target_terrain_config.get("max_structure_mass", default_max_mass))
    base_mass = float(base_terrain_config.get("max_structure_mass", default_max_mass))
    if target_mass != base_mass:
        mass_pattern = r"(- \*\*Mass Budget\*\*: Total structure mass <= )(\d+\.?\d*)( kg\.)"
        if re.search(mass_pattern, criteria):
            criteria = re.sub(
                mass_pattern,
                f"\\g<1>{target_mass:.0f} kg (originally {base_mass:.0f} kg in the source environment).",
                criteria,
            )
    return criteria

def get_f06_curriculum_stages() -> List[Dict[str, Any]]:
    UNIFORM_SUFFIX = uniform_suffix_for_task("F_06")
    return [
        {
            "stage_id": "Stage-1",
            "title": "Localized Transport Anomaly",
            "mutation_description": "One or more candidate environmental variables differ from the source environment.",
            "task_description_suffix": uniform_suffix_for_task("F_06"),
            "terrain_config": {
                "gravwell_fy": -2500.0,
                "fluid": {"viscosity": 20.0, "count": 20},
                "min_delivery_ratio": 0.45,
            },
            "physics_config": {
                "max_steps": 2400,
                "max_time_seconds": 40.0,
                "force_budget": 5000.0,
            },
        },
        {
            "stage_id": "Stage-2",
            "title": "Raised Delivery Target",
            "mutation_description": "Target zone moved to higher elevation.",
            "task_description_suffix": uniform_suffix_for_task("F_06"),
            "terrain_config": {
                "target_y_min": 2.5,
                "target_y_max": 4.0,
                "fluid": {"count": 20},
                "min_delivery_ratio": 0.45,
            },
            "physics_config": {
                "max_steps": 2400,
                "max_time_seconds": 40.0,
                "force_budget": 12000.0,
            },
        },
        {
            "stage_id": "Stage-3",
            "title": "Transport Anomaly",
            "mutation_description": "One or more candidate environmental variables differ from the source environment.",
            "task_description_suffix": uniform_suffix_for_task("F_06"),
            "terrain_config": {
                "fluid": {"viscosity": 30.0, "count": 20},
                "min_delivery_ratio": 0.52,
            },
            "physics_config": {
                "max_steps": 2400,
                "max_time_seconds": 40.0,
                "force_budget": 12000.0,
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "Raised Delivery Adaptation",
            "mutation_description": "The target zone is visibly raised; other candidate environmental variables may also differ.",
            "task_description_suffix": uniform_suffix_for_task("F_06"),
            "terrain_config": {
                "target_y_min": 2.5,
                "target_y_max": 4.0,
                "fluid": {"viscosity": 2.0, "count": 20},
                "min_delivery_ratio": 0.45,
            },
            "physics_config": {
                "gravity": (0, -15.0),
                "max_steps": 2400,
                "max_time_seconds": 40.0,
                "force_budget": 12000.0,
            },
        },
    ]

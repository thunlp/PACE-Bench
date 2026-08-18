from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

import math

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
    target_terrain_config = target_terrain_config or {}
    base_terrain_config = base_terrain_config or {}
    if stage is not None:
        target_physics_config = dict(stage.get("physics_config") or {})
        base_physics_config = {}
    target_physics_config = target_physics_config or {}
    base_physics_config = base_physics_config or {}

    def replace_once(text: str, old: str, new: str, label: str) -> str:
        matches = text.count(old)
        if matches != 1:
            raise ValueError(f"K_02 {label} update expected 1 replacement, got {matches}")
        return text.replace(old, new, 1)

    target_y_max = float(target_terrain_config.get("build_zone_y_max", 25.0))
    base_y_max = float(base_terrain_config.get("build_zone_y_max", 25.0))
    if target_y_max != base_y_max:
        old = f"- **Build Zone**: x=[0, 5], y=[0, {base_y_max:g}]. All structure components must be placed within this zone."
        new = f"- **Build Zone**: x=[0, 5], y=[0, {target_y_max:g}] (originally y=[0, {base_y_max:g}]). All structure components must be placed within this zone."
        description = replace_once(description, old, new, "build-zone")

    target_min_mass = float(target_terrain_config.get("min_structure_mass", 0.0))
    base_min_mass = float(base_terrain_config.get("min_structure_mass", 0.0))
    target_max_mass = float(target_terrain_config.get("max_structure_mass", 50.0))
    base_max_mass = float(base_terrain_config.get("max_structure_mass", 50.0))
    if target_min_mass != base_min_mass or target_max_mass != base_max_mass:
        old = f"- **Mass Budget**: Total structure mass must be at least {base_min_mass:g} kg and less than {base_max_mass:g} kg."
        min_text = f"{target_min_mass:g} kg"
        max_text = f"{target_max_mass:g} kg"
        if target_min_mass != base_min_mass:
            min_text += f" (originally {base_min_mass:g} kg)"
        if target_max_mass != base_max_mass:
            max_text += f" (originally {base_max_mass:g} kg)"
        new = f"- **Mass Budget**: Total structure mass must be at least {min_text} and less than {max_text}."
        description = replace_once(description, old, new, "mass-budget")

    inf_val = float("inf")
    target_joint_force = float(target_physics_config.get("max_joint_force", inf_val))
    target_joint_torque = float(target_physics_config.get("max_joint_torque", inf_val))
    force_changed = math.isfinite(target_joint_force)
    torque_changed = math.isfinite(target_joint_torque)
    if force_changed or torque_changed:
        old = "- **Joint strength**: Maximum joint reaction force and maximum joint torque are unlimited in the default environment (joints do not break)."
        parts = []
        if force_changed:
            parts.append(f"maximum joint reaction force {target_joint_force:g} N")
        if torque_changed:
            parts.append(f"maximum joint torque {target_joint_torque:g} N·m")
        new = f"- **Joint strength**: {'; '.join(parts)} (originally unlimited; joints break if a stated limit is exceeded)."
        description = replace_once(description, old, new, "joint-strength")
    return description

def update_success_criteria_for_visible_changes(base_success_criteria: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]) -> str:
    criteria = base_success_criteria
    target_terrain_config = target_terrain_config or {}
    base_terrain_config = base_terrain_config or {}

    def replace_once(text: str, old: str, new: str, label: str) -> str:
        matches = text.count(old)
        if matches != 1:
            raise ValueError(f"K_02 criteria {label} update expected 1 replacement, got {matches}")
        return text.replace(old, new, 1)

    target_y_max = float(target_terrain_config.get("build_zone_y_max", 25.0))
    base_y_max = float(base_terrain_config.get("build_zone_y_max", 25.0))
    if target_y_max != base_y_max:
        old = f"- **Build zone**: x=[0, 5], y=[0, {base_y_max:g}]."
        new = f"- **Build zone**: x=[0, 5], y=[0, {target_y_max:g}] (originally y=[0, {base_y_max:g}])."
        criteria = replace_once(criteria, old, new, "build-zone")

    target_min = float(target_terrain_config.get("min_structure_mass", 0.0))
    base_min = float(base_terrain_config.get("min_structure_mass", 0.0))
    target_max = float(target_terrain_config.get("max_structure_mass", 50.0))
    base_max = float(base_terrain_config.get("max_structure_mass", 50.0))
    if target_min != base_min or target_max != base_max:
        old = f"- **Mass Budget**: Minimum {base_min:g} kg, maximum < {base_max:g} kg."
        min_text = f"{target_min:g} kg" + (f" (originally {base_min:g} kg)" if target_min != base_min else "")
        max_text = f"{target_max:g} kg" + (f" (originally {base_max:g} kg)" if target_max != base_max else "")
        new = f"- **Mass Budget**: Minimum {min_text}, maximum < {max_text}."
        criteria = replace_once(criteria, old, new, "mass-budget")
    return criteria

def get_k02_curriculum_stages() -> List[Dict[str, Any]]:
    task_description_suffix = uniform_suffix_for_task("K_02")
    return [
        {
            "stage_id": "Stage-1",
            "title": "Fragile Structural Integrity",
            "mutation_description": "Near-zero joint strength thresholds — only molecular-scale structures survive. Joints shatter under even minimal load.",
            "task_description_suffix": uniform_suffix_for_task("K_02"),
            "terrain_config": {
                "build_zone_y_max": 5.0,
            },
            "physics_config": {
                "max_joint_force": 1.5,
                "max_joint_torque": 3.0,
            },
        },
        {
            "stage_id": "Stage-2",
            "title": "Triple Adhesion Blackout",
            "mutation_description": "Three separated adhesion bands leave two five-metre wall regions with no suction. A leftward crosswind and rapidly intensifying gravity punish lightweight short-stroke crawlers; each transfer must keep one distant pad cluster attached while the other crosses an entire blackout.",
            "task_description_suffix": uniform_suffix_for_task("K_02"),
            "terrain_config": {
                "build_zone_y_max": 9.0,
                "wind_force": -1.0,
                "suction_zones": [(0, 10), (13, 20), (23, 35)],
            },
            "physics_config": {
                "gravity": (0, -16.0),
                "gravity_evolution": -0.20,
                "angular_damping": 3.0,
            },
        },
        {
            "stage_id": "Stage-3",
            "title": "Discontinuous Adhesion Corridor",
            "mutation_description": "Two separated wall-adhesion gaps combine with a narrow mass interval, limited joints, evolving gravity, lateral wind, and rotational damping. A compact alternating crawler loses both anchors in each gap; a viable climber must span a gap while balancing the full structure through the remaining adhesion point.",
            "task_description_suffix": uniform_suffix_for_task("K_02"),
            "terrain_config": {
                "build_zone_y_max": 5.0,
                "min_structure_mass": 41.0,
                "max_structure_mass": 43.0,
                "wind_force": -40.0,
                "suction_zones": [(0, 8), (11, 16), (19, 35)],
            },
            "physics_config": {
                "max_joint_force": 1650.0,
                "max_joint_torque": 250.0,
                "gravity": (0, -26.0),
                "gravity_evolution": -0.35,
                "angular_damping": 8.0,
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "The Resonant Singularity",
            "mutation_description": "Extreme suction adhesion gaps combined with lateral wind and height-triggered vortex forces. Forces ultra-long-reach precision.",
            "task_description_suffix": uniform_suffix_for_task("K_02"),
            "terrain_config": {
                "build_zone_y_max": 5.0,
                "min_structure_mass": 25.0,
                "wind_force": -15.0,
                "vortex_y": 5.0,
                "vortex_force_x": 15.0,
                "vortex_force_y": -5.0,
                "suction_zones": [(0, 7), (9, 16), (18, 25), (27, 35)],
            },
            "physics_config": {
                "max_joint_force": 3000.0,
            },
        },
    ]

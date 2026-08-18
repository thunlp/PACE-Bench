from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List

import re

def update_task_description_for_visible_changes(base_description: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any], target_physics_config: Dict[str, Any] = None, base_physics_config: Dict[str, Any] = None, **kwargs) -> str:
    description = base_description
    target_physics_config = target_physics_config or {}
    base_physics_config = base_physics_config or {}
    target_terrain_config = target_terrain_config or {}
    base_terrain_config = base_terrain_config or {}
    for key, pattern, default, suffix in [
        ("max_joint_force", r"(- \*\*Joint Strength\*\*: Maximum linear force for a joint is )(\w+\.?\d*)", float('inf'), ";"),
        ("max_joint_torque", r"(; maximum torque is )(inf|\d+\.?\d*|unlimited)", float('inf'), ".")
    ]:
        target_val = target_physics_config.get(key, default)
        base_val = base_physics_config.get(key, default)
        if target_val != base_val:
            target_str = f"{target_val:.1f}" if target_val != float('inf') else "inf"
            base_str = f"{base_val:.1f}" if base_val != float('inf') else "inf"
            if re.search(pattern, description):
                description = re.sub(
                    pattern,
                    f"\\g<1>{target_str} (originally {base_str} in the source environment)",
                    description
                )
    return description

def update_success_criteria_for_visible_changes(base_success_criteria: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]) -> str:
    return base_success_criteria

def get_s02_curriculum_stages() -> List[Dict[str, Any]]:
    UNIFORM_SUFFIX = uniform_suffix_for_task("S_02")
    return [
        {
            "stage_id": "Stage-1",
            "title": "The Brittle Foundation",
            "mutation_description": "Near-zero structural torque and force limits. Any joint in any standard design will snap within milliseconds of the earthquake starting — the combined seismic and wind loads on normal structural mass instantly exceed the razor-thin joint margins. A heavy tower with even moderate joint count will suffer cascading joint failures from the base upward. Survival demands an ultra-light skeleton with maximum base width for torque distribution across as many connection points as possible, using the absolute minimum of joints and the lightest feasible materials.",
            "task_description_suffix": uniform_suffix_for_task("S_02"),
            "terrain_config": {
            },
            "physics_config": {
                "max_joint_torque": 5000.0,
                "max_joint_force": 7000.0,
            },
        },
        {
            "stage_id": "Stage-2",
            "title": "The Glass Skeleton",
            "mutation_description": "Extremely brittle joints: the maximum torque any joint can withstand is reduced to a razor-thin margin. Every kilogram of structural mass and every millimeter of height multiplies the torque imposed by seismic acceleration and wind loading. Standard designs snap their base joints within milliseconds of the earthquake starting. Survival demands an ultra-light skeleton with the absolute minimum number of joints and the widest possible base to distribute torque across as many connection points as possible.",
            "task_description_suffix": uniform_suffix_for_task("S_02"),
            "terrain_config": {
            },
            "physics_config": {
                "max_joint_torque": 5000.0,
            },
        },
        {
            "stage_id": "Stage-3",
            "title": "The Resonant Collapse",
            "mutation_description": "Near-resonant seismic frequency combined with elevated gravity and extremely brittle joints. The shifted earthquake frequency drives the tower near its fundamental sway mode, while heavier gravity amplifies joint loads. Joints fracture under forces that would be safe in any other environment. Survival demands an ultra-light structure with precision-tuned harmonic damping.",
            "task_description_suffix": uniform_suffix_for_task("S_02"),
            "terrain_config": {
                "earthquake_amplitude": 0.8,
                "earthquake_frequency": 1.4,
            },
            "physics_config": {
                "gravity": (0, -12.5),
                "max_joint_force": 28000.0,
                "max_joint_torque": 33000.0,
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "The Coupled Storm",
            "mutation_description": "A rising, off-rate foundation oscillation acts simultaneously with time-varying low-altitude wind, altitude-dependent lateral loading, elevated gravity, and brittle beam joints. The combined forcing invalidates a single tapered load path: lateral reactions concentrate at its lowest welds while its upper tiers are driven through a changing response regime. A viable structure must separate and redistribute vertical and lateral reactions while dissipating relative sway.",
            "task_description_suffix": uniform_suffix_for_task("S_02"),
            "terrain_config": {
                "earthquake_amplitude": 0.85,
                "earthquake_frequency": 3.4,
                "earthquake_amplitude_evolution": 0.012,
                "wind_force": 250.0,
                "wind_height_threshold": 5.0,
                "wind_shear_factor": 0.08,
                "wind_oscillation_frequency": 4.5,
            },
            "physics_config": {
                "gravity": (0, -20.0),
                "max_joint_force": 5000.0,
                "max_joint_torque": 10000.0,
            },
        },
    ]

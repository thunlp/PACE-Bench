from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List

import re

_UNIFORM_SUFFIX_BASE = uniform_suffix_for_task("K_04")

def _build_uniform_suffix() -> str:
    return _UNIFORM_SUFFIX_BASE

def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],

) -> str:
    description = base_description
    target_dist = target_terrain_config.get("target_distance", 10.0)
    base_dist = base_terrain_config.get("target_distance", 10.0)
    if target_dist != base_dist:
        target_pattern = r"(- \*\*Target\*\*: Push the object to at least x=)(\d+\.?\d*)(m)( \([^)]+\)\.)"
        description, replacements = re.subn(
                target_pattern,
                f"\\g<1>{8.0 + target_dist:.1f}\\g<3> (originally x={8.0 + base_dist:.1f}m in the source environment).",
                description,
                count=1,
            )
        if replacements != 1:
            raise ValueError(f"K_04 target update expected 1 replacement, got {replacements}")
        distance_pattern = r"(- \*\*Distance\*\*: The object center reaches x >= )(\d+\.?\d*)(m)\."
        description, replacements = re.subn(
                distance_pattern,
                f"\\g<1>{8.0 + target_dist:.1f}\\g<3> (originally {8.0 + base_dist:.1f}\\g<3> in the source environment).",
                description,
                count=1,
            )
        if replacements != 1:
            raise ValueError(f"K_04 distance update expected 1 replacement, got {replacements}")
    target_bz = target_terrain_config.get("build_zone", {})
    base_bz = base_terrain_config.get("build_zone", {})
    target_x = target_bz.get("x", [0.0, 15.0])
    target_y = target_bz.get("y", [1.5, 8.0])
    base_x = base_bz.get("x", [0.0, 15.0])
    base_y = base_bz.get("y", [1.5, 8.0])
    if (target_x != base_x or target_y != base_y) and isinstance(target_x, (list, tuple)) and isinstance(target_y, (list, tuple)):
        x_min_t, x_max_t = float(target_x[0]), float(target_x[1])
        y_min_t, y_max_t = float(target_y[0]), float(target_y[1])
        x_min_b, x_max_b = float(base_x[0]), float(base_x[1])
        y_min_b, y_max_b = float(base_y[0]), float(base_y[1])
        bz_desc_pattern = r"(- \*\*Build Zone\*\*: x=\[)(\d+\.?\d*),\s*(\d+\.?\d*)(\], y=\[)(\d+\.?\d*),\s*(\d+\.?\d*)(\].)"
        if re.search(bz_desc_pattern, description):
            description = re.sub(
                bz_desc_pattern,
                lambda m: (
                    m.group(1)
                    + f"{x_min_t:.1f}, {x_max_t:.1f}"
                    + m.group(4)
                    + f"{y_min_t:.1f}, {y_max_t:.1f}"
                    + m.group(7)
                ),
                description
            )
        bz_constraint_pattern = r"(All components must stay within x=\[)(\d+\.?\d*),\s*(\d+\.?\d*)(\], y=\[)(\d+\.?\d*),\s*(\d+\.?\d*)(\].)"
        if re.search(bz_constraint_pattern, description):
            description = re.sub(
                bz_constraint_pattern,
                f"\\g<1>{x_min_t:.1f}, {x_max_t:.1f}\\g<4>{y_min_t:.1f}, {y_max_t:.1f}\\g<7> (originally x=[{x_min_b:.1f}, {x_max_b:.1f}], y=[{y_min_b:.1f}, {y_max_b:.1f}] in the source environment).",
                description
            )
    target_mass = target_terrain_config.get("max_structure_mass", 40.0)
    base_mass = base_terrain_config.get("max_structure_mass", 40.0)
    if target_mass != base_mass:
        mass_pattern = r"(- \*\*Mass Budget\*\*: Total structure mass must be less than )(\d+\.?\d*)( kg\.)"
        description, replacements = re.subn(
                mass_pattern,
                f"\\g<1>{target_mass:.0f} kg (originally {base_mass:.0f} kg in the source environment).",
                description,
                count=1,
            )
        if replacements != 1:
            raise ValueError(f"K_04 mass-budget update expected 1 replacement, got {replacements}")
    return description

def update_success_criteria_for_visible_changes(base_success_criteria: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]) -> str:
    criteria = base_success_criteria
    target_dist = target_terrain_config.get("target_distance", 10.0)
    base_dist = base_terrain_config.get("target_distance", 10.0)
    if target_dist != base_dist:
        dist_pattern = r"(\*\*Movement\*\*: Object reaches x >= )(\d+\.?\d*)(m)\."
        criteria, replacements = re.subn(
                dist_pattern,
                f"\\g<1>{8.0 + target_dist:.1f}\\g<3> (originally x >= {8.0 + base_dist:.1f}\\g<3> in the source environment).",
                criteria,
                count=1,
            )
        if replacements != 1:
            raise ValueError(f"K_04 criteria distance update expected 1 replacement, got {replacements}")
    target_mass = target_terrain_config.get("max_structure_mass", 40.0)
    base_mass = base_terrain_config.get("max_structure_mass", 40.0)
    if target_mass != base_mass:
        mass_pattern = r"(\*\*Mass Budget\*\*: < )(\d+\.?\d*)( kg)(\.)"
        criteria, replacements = re.subn(
                mass_pattern,
                f"\\g<1>{target_mass:.0f}\\g<3> (originally {base_mass:.0f}\\g<3> in the source environment)\\g<4>",
                criteria,
                count=1,
            )
        if replacements != 1:
            raise ValueError(f"K_04 criteria mass-budget update expected 1 replacement, got {replacements}")
    return criteria

def get_k04_curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Base",
            "title": "Initial Environment",
            "mutation_description": "The base pusher environment with default parameters.",
            "task_description_suffix": "",
            "terrain_config": {},
            "physics_config": {},
        },
        {
            "stage_id": "Stage-1",
            "title": "Tipping Hazard and Mass Budget",
            "mutation_description": "Tight mass budget and object center-of-mass offset; a heavy front-plate pusher exceeds the budget and the object tips if pushed from below.",
            "task_description_suffix": uniform_suffix_for_task("K_04"),
            "terrain_config": {
                "object": {"center_of_mass_offset": [0.2, 0.25]},
                "max_structure_mass": 26.0,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-2",
            "title": "Payload Mass Variation",
            "mutation_description": "Object mass is very high. The initial pusher structure cannot accelerate it to the target distance in time.",
            "task_description_suffix": uniform_suffix_for_task("K_04"),
            "terrain_config": {
                "object": {"mass": 95.0},
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-3",
            "title": "Modified Friction, Damping, and Mass Budget",
            "mutation_description": "Tight mass budget, significantly reduced friction, and altered object damping. Heavy wheeled pusher exceeds budget; light designs must overcome slip and damping effects.",
            "task_description_suffix": uniform_suffix_for_task("K_04"),
            "terrain_config": {
                "ground_friction": 0.3,
                "object": {"friction": 0.08, "linear_damping": 4.0},
                "max_structure_mass": 26.0,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-4",
            "title": "Heavy Object, Slippery Surfaces, Strong Damping, Tight Budget, and High Gravity",
            "mutation_description": "An ultra-heavy object rests on near-frictionless surfaces with extreme linear damping, very high gravity, a far target distance, the tightest mass budget, and a shifted center-of-mass. Initial reference designs fail on mass budget; even compliant designs must overcome massive inertia, aggressive damping, and near-zero surface grip.",
            "task_description_suffix": uniform_suffix_for_task("K_04"),
            "terrain_config": {
                "ground_friction": 0.04,
                "object": {
                    "mass": 120.0,
                    "friction": 0.02,
                    "linear_damping": 15.0,
                    "center_of_mass_offset": [0.24, 0.28],
                },
                "target_distance": 17.0,
                "max_structure_mass": 20.0,
            },
            "physics_config": {"gravity": (0, -20.0)},
        },
    ]

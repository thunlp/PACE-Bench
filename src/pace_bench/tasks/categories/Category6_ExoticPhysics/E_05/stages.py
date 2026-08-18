from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

import re

from typing import Any, Dict, List

_DEFAULT_MAGNETS = [
    (12.0, 4.0, -300.0), (12.0, 5.0, -300.0), (12.0, 6.0, -300.0),
    (12.0, 7.0, -300.0), (12.0, 8.0, -280.0), (12.0, 8.3, -260.0),
    (11.0, 9.7, -200.0), (13.0, 9.7, -200.0), (15.0, 9.7, -200.0),
    (17.0, 9.7, -200.0), (19.0, 9.7, -200.0), (21.0, 9.7, -180.0),
    (15.0, 9.0, -250.0, 230.0, 0.12), (20.0, 9.0, -350.0, 330.0, 0.15, 3.14159),
    (19.0, 3.0, 160.0), (21.0, 3.5, 130.0),
    (24.0, 5.0, -190.0), (24.0, 8.2, -180.0),
    (24.0, 6.6, -180.0, 160.0, 0.165),
    (26.0, 5.5, -130.0), (27.0, 9.5, -120.0), (29.5, 7.5, 95.0),

]

def _magnets_stage1() -> List[tuple]:
    return [(14.0, y, -800.0) for y in range(0, 15)]

def _magnets_stage2() -> List[tuple]:
    curtains = (
        (13.0, (1.0, 2.0, 6.0, 8.0)),
        (18.0, (1.0, 3.0, 4.0, 5.0, 9.0)),
        (23.0, (1.0, 3.0, 4.0, 5.0, 9.0)),
    )
    magnets = [
        (x, y, -140.0)
        for x, vertical_sources in curtains
        for y in vertical_sources
    ]
    magnets.extend((float(x), 10.0, -140.0) for x in range(4, 39, 2))
    return magnets

def _magnets_stage3() -> List[tuple]:
    return [
        (12.0, 4.0, -420.0), (12.0, 5.0, -420.0), (12.0, 6.0, -420.0),
        (12.0, 7.0, -420.0), (12.0, 8.0, -400.0), (12.0, 8.3, -380.0),
        (11.0, 9.7, -280.0), (13.0, 9.7, -280.0), (15.0, 9.7, -280.0),
        (17.0, 9.7, -280.0), (19.0, 9.7, -280.0), (21.0, 9.7, -260.0),
        (15.0, 9.0, -420.0, 380.0, 0.12),
        (20.0, 9.0, -480.0, 440.0, 0.15, 3.14159),
        (19.0, 3.0, 320.0), (21.0, 3.5, 280.0),
        (24.0, 5.0, -250.0), (24.0, 8.2, -240.0),
        (24.0, 6.6, -240.0, 200.0, 0.165),
        (26.0, 5.5, -190.0), (27.0, 9.5, -190.0),
        (29.5, 7.5, 95.0),
    ]

def _magnets_stage4() -> List[tuple]:
    magnets = []

    magnets.append((20.0, 8.0, -250.0, 240.0, 0.12))

    magnets.append((12.0, 7.0, -80.0))
    magnets.append((16.0, 7.0, -60.0))
    magnets.append((24.0, 7.0, -90.0))
    magnets.append((28.0, 7.0, -70.0))

    magnets.append((29.5, 7.5, 120.0))
    magnets.append((30.0, 8.0, 80.0))
    return magnets

UNIFORM_SUFFIX = uniform_suffix_for_task("E_05")

_DEFAULT_MAX_THRUST = 165.0

_DEFAULT_GRAVITY = (0, -10.0)

_DEFAULT_LINEAR_DAMPING = 0.28

_DEFAULT_ANGULAR_DAMPING = 0.15

def update_task_description_for_visible_changes(base_description: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any], target_physics_config: Dict[str, Any] = None, base_physics_config: Dict[str, Any] = None) -> str:
    description = base_description
    target_terrain_config = target_terrain_config or {}
    base_terrain_config = base_terrain_config or {}
    target_physics_config = target_physics_config or {}
    base_physics_config = base_physics_config or {}
    target_max_thrust = float(target_terrain_config.get("max_thrust", _DEFAULT_MAX_THRUST))
    base_max_thrust = float(base_terrain_config.get("max_thrust", _DEFAULT_MAX_THRUST))
    if target_max_thrust != base_max_thrust:
        thrust_cap_pattern = r"(- \*\*Maximum Thrust\*\*:.*?must not exceed )(\d+\.?\d*)( \(engine limit\)\.)"
        if re.search(thrust_cap_pattern, description):
            description = re.sub(
                thrust_cap_pattern,
                lambda m: f"{m.group(1)}{target_max_thrust:.1f} (originally {base_max_thrust:.1f} in the source environment){m.group(3)}",
                description,
            )
    return description

def update_success_criteria_for_visible_changes(base_success_criteria: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]) -> str:
    criteria = base_success_criteria
    target_terrain_config = target_terrain_config or {}
    base_terrain_config = base_terrain_config or {}
    target_max_thrust = float(target_terrain_config.get("max_thrust", _DEFAULT_MAX_THRUST))
    base_max_thrust = float(base_terrain_config.get("max_thrust", _DEFAULT_MAX_THRUST))
    if target_max_thrust != base_max_thrust:
        thrust_constraint_pattern = r"(- \*\*Maximum Thrust\*\*:.*?must not exceed )(\d+\.?\d*)( \(engine limit\)\.)"
        if re.search(thrust_constraint_pattern, criteria):
            criteria = re.sub(
                thrust_constraint_pattern,
                lambda m: f"{m.group(1)}{target_max_thrust:.1f} (originally {base_max_thrust:.1f} in the source environment){m.group(3)}",
                criteria,
            )
    return criteria

def get_e05_curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Stage-1",
            "title": "Field Variant I",
            "mutation_description": (
                "This stage applies an undisclosed field configuration; exact "
                "source locations, strengths, and mutation directions are hidden."
            ),
            "task_description_suffix": uniform_suffix_for_task("E_05"),
            "terrain_config": {
                "magnets": _magnets_stage1(),
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-2",
            "title": "Field Variant II",
            "mutation_description": (
                "This stage applies an undisclosed field configuration; exact "
                "source locations, strengths, and mutation directions are hidden."
            ),
            "task_description_suffix": uniform_suffix_for_task("E_05"),
            "terrain_config": {
                "magnets": _magnets_stage2(),
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-3",
            "title": "Field Variant III",
            "mutation_description": (
                "Published engine limits and documented public physics readings "
                "accompany an undisclosed field configuration."
            ),
            "task_description_suffix": uniform_suffix_for_task("E_05"),
            "terrain_config": {
                "magnets": _magnets_stage3(),
                "max_thrust": 420.0,
            },
            "physics_config": {
                "gravity": (0, -28.0),
                "linear_damping": 8.0,
                "angular_damping": 4.0,
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "Field Variant IV",
            "mutation_description": (
                "Published engine limits and documented public physics readings "
                "accompany an undisclosed field configuration."
            ),
            "task_description_suffix": uniform_suffix_for_task("E_05"),
            "terrain_config": {
                "magnets": _magnets_stage4(),
                "max_thrust": 570.0,
            },
            "physics_config": {
                "gravity": (0, -53.0),
                "linear_damping": 11.0,
                "angular_damping": 5.5,
            },
        },
    ]

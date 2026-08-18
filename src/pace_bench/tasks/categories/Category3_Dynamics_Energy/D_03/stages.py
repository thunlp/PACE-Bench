from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

import re

from typing import Any, Dict, List

_DEFAULT_IMPULSE_MAGNITUDE = 1.5

_DEFAULT_IMPULSE2_MAGNITUDE = 0.55

_DEFAULT_DECEL_DAMPING = 3.2

_DEFAULT_MUD_DAMPING = 4.2

_DEFAULT_GRAVITY = (0, -10.0)

def update_task_description_for_visible_changes(
    base_description: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None, base_physics_config: Dict[str, Any] = None,
    **kwargs,

) -> str:
    del target_terrain_config, base_terrain_config, target_physics_config, base_physics_config, kwargs
    # Impulse magnitudes and damping coefficients are latent dynamics.
    return base_description

def update_success_criteria_for_visible_changes(
    base_success_criteria: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any],
    **kwargs,

) -> str:
    return base_success_criteria

_D03_SUFFIX = uniform_suffix_for_task("D_03")

def get_d03_curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Stage-1",
            "title": "Massive first impulse",
            "mutation_description": "First impulse magnitude massively increased; requires forward thrust to survive speed trap v(9)>=2.8.",
            "task_description_suffix": uniform_suffix_for_task("D_03"),
            "terrain_config": {
                "impulse_magnitude": 65.0,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-2",
            "title": "Viscous recoil threshold",
            "mutation_description": "Severe ambient drag and a large one-shot reverse impulse create a discontinuous speed transition: the cart must accumulate momentum before the impulse, recover without recrossing the trigger, then brake into the downstream speed window.",
            "task_description_suffix": uniform_suffix_for_task("D_03"),
            "terrain_config": {
                "impulse_magnitude": 78.0,
            },
            "physics_config": {
                "linear_damping": 8.5,
            },
        },
        {
            "stage_id": "Stage-3",
            "title": "Extreme resistance cascade",
            "mutation_description": "Massively amplified impulses, severe mud and decel zone damping, plus aggressive global linear drag. Initial reference speeds plummet below threshold; requires dense heavy build plus constant forward propulsion with careful phase-dependent speed modulation.",
            "task_description_suffix": uniform_suffix_for_task("D_03"),
            "terrain_config": {
                "impulse_magnitude": 39.0,
                "impulse2_magnitude": 5.2,
                "decel_damping": 14.5,
                "mud_damping": 12.5,
            },
            "physics_config": {
                "linear_damping": 3.2,
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "Double-recoil gravity vise",
            "mutation_description": "Extreme mud and ambient drag oppose motion before a violent first recoil, while a second recoil lands inside a high-damping corridor under heavy gravity. The controller must use distinct momentum, recovery, checkpoint, and terminal phases.",
            "task_description_suffix": uniform_suffix_for_task("D_03"),
            "terrain_config": {
                "impulse_magnitude": 85.0,
                "impulse2_magnitude": 18.0,
                "decel_damping": 24.0,
                "mud_damping": 20.0,
            },
            "physics_config": {
                "gravity": (0, -20),
                "linear_damping": 6.0,
                "angular_damping": 2.5,
            },
        },
    ]

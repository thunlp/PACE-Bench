from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List

def get_d04_curriculum_stages() -> List[Dict[str, Any]]:
    union_variables = {
        "Actuator Dead Zone": "The swing's primary force thrusters may exhibit spatial or engagement anomalies; use feedback to infer where and when thrust is available.",
        "Quadratic Damping Anomaly": "The environment may exhibit anomalous energy dissipation; use feedback to infer the actual behavior.",
        "Directional Actuator Fault": "The force actuators may exhibit directional or engagement anomalies; use feedback to infer how thrust is available.",
        "Extreme Atmospheric Conditions": "Atmospheric or wind conditions may differ from the initial environment in ways that affect the swing's equilibrium and trajectory; use feedback to infer the actual behavior.",
    }
    bullet_points = "\n".join([f" - **{k}**: {v}" for k, v in union_variables.items()])
    _D04_SUFFIX = uniform_suffix_for_task("D_04")
    return [
        {
            "stage_id": "Stage-1",
            "title": "The Velocity-Gated Dead Zone",
            "mutation_description": "Actuator fails in an asymmetric central region unless horizontal speed exceeds a critical threshold; thrust is only available in narrow side bands or when crossing the zone at high speed—discovery of the velocity-gate and side-band strategy is required.",
            "task_description_suffix": uniform_suffix_for_task("D_04"),
            "terrain_config": {
                "dead_zone": [9.2, 10.4],
                "dead_zone_min_speed": 16.0,
                "wind_strength": 10.0,
                "wind_period": 0.0,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-2",
            "title": "Quadratic Energy Drain",
            "mutation_description": "Extreme quadratic damping penalizes any motion; drag approaches max pump force at peak swing speeds, making horizontal-only pumping self-defeating. Requires velocity-aligned continuous energy injection and careful phase-timing to accumulate net energy per cycle.",
            "task_description_suffix": uniform_suffix_for_task("D_04"),
            "terrain_config": {
                "quadratic_damping": 0.39,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-3",
            "title": "One-Way Actuator & Gale",
            "mutation_description": "Directional actuator fault combined with strong constant wind; thrust is available in only one horizontal direction and wind acts in a fixed direction. The agent must discover which directions apply via feedback.",
            "task_description_suffix": uniform_suffix_for_task("D_04"),
            "terrain_config": {
                "actuator_fault": "left_only",
                "wind_strength": 35.0,
                "wind_period": 0.0,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-4",
            "title": "The Ultimate Crucible",
            "mutation_description": "Combined directional actuator fault, central dead zone, quadratic damping, and strong constant wind; all directions and magnitudes must be inferred from feedback.",
            "task_description_suffix": uniform_suffix_for_task("D_04"),
            "terrain_config": {
                "actuator_fault": "right_only",
                "dead_zone": [9.7, 10.3],
                "quadratic_damping": 0.13,
                "wind_strength": -30.0,
                "wind_period": 0.0,
            },
            "physics_config": {},
        },
    ]

def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,
    *,
    stage: Dict[str, Any] = None,

) -> str:
    return base_description

def update_success_criteria_for_visible_changes(
    base_success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    *,
    stage: Dict[str, Any] = None,

) -> str:
    del target_terrain_config, base_terrain_config, stage
    # Dead zones and directional faults are latent actuator behavior.
    return base_success_criteria

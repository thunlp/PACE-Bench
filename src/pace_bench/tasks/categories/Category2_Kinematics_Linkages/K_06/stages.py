from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List

import re

def _get_uniform_task_description_suffix() -> str:
    return uniform_suffix_for_task("K_06")

def update_task_description_for_visible_changes(base_description: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]) -> str:
    description = base_description
    target_count = target_terrain_config.get("particles", {}).get("count", 45)
    base_count = base_terrain_config.get("particles", {}).get("count", 45)
    if target_count != base_count:
        pattern = r"(- \*\*Particles\*\*: )(\d+)( small particles)"
        description = re.sub(pattern, f"\\g<1>{target_count} small particles (originally {base_count} small particles in the source environment)", description)
    target_mass = target_terrain_config.get("max_structure_mass", 15.0)
    base_mass = base_terrain_config.get("max_structure_mass", 15.0)
    if target_mass != base_mass:
        pattern = r"(Total structure mass must be less than )(\d+\.?\d*)( kg)"
        description = re.sub(pattern, f"\\g<1>{target_mass:.2f} kg (originally {base_mass:.2f} kg in the source environment)", description)
    target_motor_cap = target_terrain_config.get("max_motor_torque")
    base_motor_cap = base_terrain_config.get("max_motor_torque")
    if target_motor_cap is not None:
        old_val = f"{base_motor_cap:.1f} N·m" if base_motor_cap is not None else "no cap"
        pattern = r"(- \*\*Motor torque\*\*: )No environment cap \(solver may request up to API limits\)\."
        replacement = f"\\g<1>Capped at {target_motor_cap:.1f} N·m (originally {old_val} in the source environment)."
        if re.search(pattern, description):
            description = re.sub(pattern, replacement, description)
    return description

def update_success_criteria_for_visible_changes(base_success_criteria: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]) -> str:
    criteria = base_success_criteria
    target_mass = target_terrain_config.get("max_structure_mass", 15.0)
    base_mass = base_terrain_config.get("max_structure_mass", 15.0)
    if target_mass != base_mass:
        pattern = r"(\n- \*\*Mass Budget\*\*: < )(\d+\.?\d*)( kg\.)"
        replacement = rf"\g<1>{target_mass:.2f} kg (originally {base_mass:.2f} kg in the source environment)."
        criteria = re.sub(pattern, replacement, criteria)
    return criteria

def get_k06_curriculum_stages() -> List[Dict[str, Any]]:
    UNIFORM_SUFFIX = uniform_suffix_for_task("K_06")
    return [
        {
            "stage_id": "Stage-1",
            "title": "Extreme Motor Torque Starvation",
            "mutation_description": "Motor torque capped at 1.0 N·m. The standard high-torque wiper (requesting 4500 N·m) is completely starved — tip force at full span drops far below particle friction threshold, making single-pivot designs impossible.",
            "task_description_suffix": uniform_suffix_for_task("K_06"),
            "terrain_config": {
                "max_motor_torque": 1.0,
                "particles": {"count": 55, "seed": 42, "friction": 0.35, "mass": 0.15},
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-2",
            "title": "Severe Torque Starvation with Near-Maximum Particle Adhesion",
            "mutation_description": "Max motor torque capped at 10.0 N·m; particle friction 0.88. Standard wiper is completely neutered — barely any torque and particles resist sliding.",
            "task_description_suffix": uniform_suffix_for_task("K_06"),
            "terrain_config": {
                "max_motor_torque": 6.0,
                "particles": {"count": 70, "seed": 42, "friction": 0.96, "mass": 0.25},
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-3",
            "title": "Mass Starvation vs Heavy Sticky Particle Overload with Capped Torque",
            "mutation_description": "Mass budget 0.10 kg, motor torque cap 28 N·m, 78 heavy sticky particles (0.40 kg each, friction 0.76). Extreme mass constraint forces ultra-minimalist design while heavy high-friction particles demand robust pushing power and limited motor torque restricts sweeping force.",
            "task_description_suffix": uniform_suffix_for_task("K_06"),
            "terrain_config": {
                "max_structure_mass": 0.065,
                "max_motor_torque": 20.0,
                "particles": {"count": 100, "seed": 37, "friction": 0.90, "mass": 0.50},
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-4",
            "title": "Extreme: Double Gravity, Glass Adhesion & Torque Starvation Paradox",
            "mutation_description": "Gravity 2.2× (22 m/s²), mass budget 0.06 kg, motor cap 10.0 N·m, glass friction 0.55, 65 particles (0.15 kg each, friction 0.75). Combines extreme conflicting constraints: doubled gravity makes particles nearly immovable, heightened glass friction glues particles to the surface, ultra-tight mass budget forbids any reinforcement, and severely starved torque limits all actuators. Per-particle sliding resistance ≈2.12 N vs 2.06 N in Stage-3 — but torque is cut 2.8× and gravity doubled.",
            "task_description_suffix": uniform_suffix_for_task("K_06"),
            "terrain_config": {
                "max_structure_mass": 0.05,
                "max_motor_torque": 8.0,
                "glass_friction": 0.65,
                "particles": {"count": 75, "seed": 51, "friction": 0.82, "mass": 0.20},
            },
            "physics_config": {
                "gravity": (0, -24),
            },
        },
    ]

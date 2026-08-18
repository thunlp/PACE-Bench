from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List

import re

def update_task_description_for_visible_changes(base_description: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]) -> str:
    description = base_description
    base_terrain_config = base_terrain_config or {}
    default_spawn = [-10.0, 0.0]
    default_ceiling = 100.0
    default_mass = 20000.0
    default_stability_time = 10.0
    default_floor_length = 20.0
    target_floor_length = target_terrain_config.get("floor_length", default_floor_length)
    base_floor_length = base_terrain_config.get("floor_length", default_floor_length)
    if target_floor_length != base_floor_length:
        target_edge = -10.0 + target_floor_length / 2.0
        base_edge = -10.0 + base_floor_length / 2.0
        pattern = r"(- \*\*Table\*\*: A horizontal surface extending from x=-20 to x=)(\d+\.?\d*)(\. The table edge is at x=)(\d+\.?\d*)(\.)"
        if re.search(pattern, description):
            description = re.sub(
                pattern,
                f"\\g<1>{target_edge:.1f}\\g<3>{target_edge:.1f} (was x={base_edge:.1f}).",
                description,
            )
    target_overhang = target_terrain_config.get("target_overhang", 0.1)
    base_overhang = base_terrain_config.get("target_overhang", 0.1)
    if target_overhang != base_overhang:
        pattern = r"(\s*-\s*\*\*Goal\*\*: Reach x >= )(\d+\.?\d*)m( beyond the edge\.)"
        if re.search(pattern, description):
            description = re.sub(pattern, f"\\g<1>{target_overhang:.2f}m (was {base_overhang:.2f}m)\\g<3>", description)
    target_spawn = target_terrain_config.get("spawn_zone", default_spawn)
    base_spawn = base_terrain_config.get("spawn_zone", default_spawn)
    if target_spawn != base_spawn:
        pattern = r"(\*\*Spawn Rule\*\*: Blocks must be initialized within the permitted build access zone: x in )(\[.*?\])(\.)"
        if re.search(pattern, description):
            base_str = f"[{base_spawn[0]:.2f}, {base_spawn[1]:.2f}]"
            description = re.sub(pattern, f"\\g<1>[{target_spawn[0]:.2f}, {target_spawn[1]:.2f}] (was {base_str})\\g<3>", description)
    target_ceiling = target_terrain_config.get("ceiling_y", default_ceiling)
    base_ceiling = base_terrain_config.get("ceiling_y", default_ceiling)
    if target_ceiling != base_ceiling:
        pattern = r"(\s*-\s*\*\*Ceiling Boundary\*\*: Structure cannot exceed y = )(\d+\.?\d*)m( in height\.)"
        if re.search(pattern, description):
            description = re.sub(pattern, f"\\g<1>{target_ceiling:.1f}m in height (was {base_ceiling:.1f}m).", description)
    target_mass = target_terrain_config.get("max_total_mass", default_mass)
    base_mass = base_terrain_config.get("max_total_mass", default_mass)
    if target_mass != base_mass:
        pattern = r"(\s*-\s*\*\*Mass Budget\*\*: Total structure mass must be less than or equal to )(\d+\.?\d*)( units\.)"
        if re.search(pattern, description):
            description = re.sub(pattern, f"\\g<1>{target_mass:.1f} units (was {base_mass:.1f} units).", description)
    return description

def update_success_criteria_for_visible_changes(base_success_criteria: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]) -> str:
    criteria = base_success_criteria
    base_terrain_config = base_terrain_config or {}
    target_overhang = target_terrain_config.get("target_overhang", 0.1)
    base_overhang = base_terrain_config.get("target_overhang", 0.1)
    if target_overhang != base_overhang:
        pattern = r"(\(Tip reaches x >= )(\d+\.?\d*)m(\)\.)"
        if re.search(pattern, criteria):
            criteria = re.sub(pattern, f"\\g<1>{target_overhang:.2f}m (was {base_overhang:.2f}m)\\g<3>", criteria)
    target_mass = target_terrain_config.get("max_total_mass", 20000.0)
    base_mass = base_terrain_config.get("max_total_mass", 20000.0)
    if target_mass != base_mass:
        pattern = r"(\s*-\s*\*\*Mass Budget\*\*: Total mass must be <= )(\d+\.?\d*)( units\.)"
        if re.search(pattern, criteria):
            criteria = re.sub(pattern, f"\\g<1>{target_mass:.1f} units (was {base_mass:.1f} units).", criteria)
    target_stability_time = target_terrain_config.get("stability_time", 10.0)
    base_stability_time = base_terrain_config.get("stability_time", 10.0)
    if target_stability_time != base_stability_time:
        pattern = r"(\s*-\s*\*\*Stability Time\*\*: Structure must remain motionless for at least )(\d+\.?\d*)( seconds\.)"
        if re.search(pattern, criteria):
            criteria = re.sub(pattern, f"\\g<1>{target_stability_time:.1f} (was {base_stability_time:.1f})\\g<3>", criteria)
    return criteria

def get_s06_curriculum_stages() -> List[Dict[str, Any]]:
    UNIFORM_SUFFIX = uniform_suffix_for_task("S_06")
    return [
        {
            "stage_id": "Stage-1",
            "title": "The Near-Lift Mass Staircase",
            "mutation_description": "A nearly gravity-cancelling upward-left disturbance makes light outboard blocks lose contact while a 24-unit ceiling forbids uniform heavy ballast. Reaching the edge requires a three-level inboard-to-outboard mass staircase whose nested centers of mass remain supported under asymmetric acceleration.",
            "task_description_suffix": uniform_suffix_for_task("S_06"),
            "terrain_config": {
                "target_overhang": 0.60,
                "floor_length": 20.0,
                "spawn_zone": [-10.0, 0.10],
                "max_total_mass": 24.0,
                "table_friction": 0.025,
                "block_friction": 0.80,
                "oscillate": False,
                "osc_amplitude": 0.0,
                "osc_frequency": 0.0,
            },
            "physics_config": {
                "gravity": (0, -10.0),
                "wind_force": (-3.0, 9.0),
            },
        },
        {
            "stage_id": "Stage-2",
            "title": "The Wind-Loaded Mass Cascade",
            "mutation_description": "A 1.6 N per-body wind acts on a four-tier cantilever under a 27-unit mass cap. Meeting the 0.61 overhang with centers restricted to x<=0.12 requires a steep inboard-to-outboard mass gradient and nested support at every interface.",
            "task_description_suffix": uniform_suffix_for_task("S_06"),
            "terrain_config": {
                "target_overhang": 0.61,
                "floor_length": 20.0,
                "spawn_zone": [-10.0, 0.12],
                "max_total_mass": 27.0,
                "table_friction": 0.03,
                "block_friction": 0.45,
                "oscillate": False,
                "osc_amplitude": 0.0,
                "osc_frequency": 0.0,
            },
            "physics_config": {
                "gravity": (0, -10.0),
                "wind_force": 1.6,
            },
        },
        {
            "stage_id": "Stage-3",
            "title": "The Harmonic Lift Cantilever",
            "mutation_description": "A long-reach harmonic cantilever must terminate at x=1.38 even though block centers may be initialized only through x=0.89. The total mass limit is 18.0, table friction is 0.22, and inter-block friction is 0.8. A persistent external force vector couples the reach problem to contact retention: every added layer incurs another fixed force contribution, while insufficiently massive outboard layers lose normal load and cannot remain seated on their support. A two-block ballast pair cannot generate the required lever arm within the budget, and low-layer graded stacks lack enough theoretical reach once each member carries the contact-retention mass floor. The reference instead uses five independently weighted layers whose nested centers of mass remain supported while the inboard mass gradient supplies the table reaction. This replaces the previous equal-density pair with coupled harmonic geometry, per-interface mass allocation, contact-retention thresholds, and whole-structure counterbalance.",
            "task_description_suffix": uniform_suffix_for_task("S_06"),
            "terrain_config": {
                "target_overhang": 1.38,
                "floor_length": 20.0,
                "spawn_zone": [-10.0, 0.89],
                "max_total_mass": 18.0,
                "table_friction": 0.22,
                "block_friction": 0.8,
                "oscillate": False,
                "osc_amplitude": 0.0,
                "osc_frequency": 0.0,
            },
            "physics_config": {
                "gravity": (0, -10.0),
                "wind_force": (-3.0, 9.5),
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "The Gravitational Siege",
            "mutation_description": "Stage-4 combines 12 m/s^2 gravity, 0.16 m table oscillation at 3.6 rad/s, a (-3,10) per-body disturbance, and an 18-unit mass cap. A three-layer graded stack is needed to retain contact and reach the target under the coupled vertical and lateral forcing.",
            "task_description_suffix": uniform_suffix_for_task("S_06"),
            "terrain_config": {
                "target_overhang": 0.65,
                "floor_length": 20.0,
                "spawn_zone": [-10.0, 0.05],
                "max_total_mass": 17.0,
                "table_friction": 0.21,
                "block_friction": 0.85,
                "oscillate": True,
                "osc_amplitude": 0.18,
                "osc_frequency": 3.8,
            },
            "physics_config": {
                "gravity": (0, -13.0),
                "wind_force": (-3.2, 10.5),
            },
        },
    ]

from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List

import re

DEFAULT_METEOR_COUNT = 12

DEFAULT_CORE_MAX_FORCE = 150.0

DEFAULT_CORE_X = 10.0

DEFAULT_CORE_Y = 1.0

DEFAULT_MAX_MASS = 300.0

DEFAULT_METEOR_SPAWN_INTERVAL = 30

DEFAULT_WIND_FORCE = 0.0

DEFAULT_METEOR_RESTITUTION = 0.2

DEFAULT_METEOR_DENSITY = 5.0

DEFAULT_FLOOR_FRICTION = 0.5

DEFAULT_MAX_JOINT_FORCE = 1e12

DEFAULT_MAX_JOINT_TORQUE = 1e12

DEFAULT_HAS_WALLS = False

DEFAULT_GRAVITY = (0, -10.0)

def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,

) -> str:
    description = base_description
    target_has_walls = bool(target_terrain_config.get("has_walls", DEFAULT_HAS_WALLS))
    base_has_walls = bool(base_terrain_config.get("has_walls", DEFAULT_HAS_WALLS))
    if target_has_walls != base_has_walls:
        if target_has_walls:
            description = re.sub(
                r"(Boulders will spawn above the core, mostly targeting the center, but some will fall from the left and right sides\.)( The core is extremely delicate)",
                r"\1 The scene is enclosed by lateral walls; debris ricochets and creates secondary impacts.\2",
                description,
                count=1
            )
        else:
            description = re.sub(
                r"(Boulders will spawn above the core, mostly targeting the center, but some will fall from the left and right sides\.) The scene is enclosed by lateral walls; debris ricochets and creates secondary impacts\.( The core is extremely delicate)",
                r"\1\2",
                description,
                count=1
            )
    target_meteor_count = int(target_terrain_config.get("meteor_count", DEFAULT_METEOR_COUNT))
    base_meteor_count = int(base_terrain_config.get("meteor_count", DEFAULT_METEOR_COUNT))
    target_spawn_interval = int(target_terrain_config.get("meteor_spawn_interval", DEFAULT_METEOR_SPAWN_INTERVAL))
    base_spawn_interval = int(base_terrain_config.get("meteor_spawn_interval", DEFAULT_METEOR_SPAWN_INTERVAL))
    if target_meteor_count != base_meteor_count or target_spawn_interval != base_spawn_interval:
        boulder_pattern = r"(In the nominal mission, )(\d+)( boulders spawn from above \(one every )(\d+)( simulation steps\))(?:, and 4 additional boulders spawn from the left and right sides \(every 90 steps\)\.)"
        if re.search(boulder_pattern, description):
            side_count = target_meteor_count // 3
            side_interval = target_spawn_interval * 3
            base_side_count = base_meteor_count // 3
            base_side_interval = base_spawn_interval * 3
            description = re.sub(
                boulder_pattern,
                f"\\g<1>{target_meteor_count} boulders spawn from above (one every {target_spawn_interval} simulation steps) (originally {base_meteor_count} boulders, one every {base_spawn_interval} simulation steps in the source environment), and {side_count} additional boulders spawn from the left and right sides (every {side_interval} steps) (originally {base_side_count} additional boulders, every {base_side_interval} steps in the source environment).",
                description,
                count=1,
            )
    target_max_mass = target_terrain_config.get("max_structure_mass", DEFAULT_MAX_MASS)
    base_max_mass = base_terrain_config.get("max_structure_mass", DEFAULT_MAX_MASS)
    if target_max_mass != base_max_mass:
        mass_desc_pattern = r"(- \*\*Mass Budget\*\*: Total structure mass must not exceed )(\d+\.?\d*) kg\."
        if re.search(mass_desc_pattern, description):
            description = re.sub(
                mass_desc_pattern,
                f"\\g<1>{target_max_mass:.1f} kg (originally {base_max_mass:.1f} kg in the source environment).",
                description
            )
    target_core_x = target_terrain_config.get("core_x", DEFAULT_CORE_X)
    target_core_y = target_terrain_config.get("core_y", DEFAULT_CORE_Y)
    base_core_x = base_terrain_config.get("core_x", DEFAULT_CORE_X)
    base_core_y = base_terrain_config.get("core_y", DEFAULT_CORE_Y)
    if target_core_x != base_core_x or target_core_y != base_core_y:
        core_pos_pattern = r"(Protect a fragile Core \(a sensitive circular object at x=)(\d+\.?\d*)(, y=)(\d+\.?\d*)(\) from)"
        description = re.sub(core_pos_pattern,
                            f"\\g<1>{target_core_x:.1f}\\g<3>{target_core_y:.1f}) (originally x={base_core_x:.1f}, y={base_core_y:.1f} in the source environment). From",
                            description)
        env_core_pattern = r"(- \*\*Core\*\*: A circular object centered at \()(\d+\.?\d*)(, )(\d+\.?\d*)(\) with radius 0\.5 m\.)"
        description = re.sub(env_core_pattern,
                            f"\\g<1>{target_core_x:.1f}\\g<3>{target_core_y:.1f}) (originally centered at ({base_core_x:.1f}, {base_core_y:.1f}) in the source environment) with radius 0.5 m.",
                            description)
        koz_pattern = r"(- \*\*Keep-Out Zone\*\*: You cannot build any structural components within 1.3m of the core center \()(\d+\.?\d*)(, )(\d+\.?\d*)(\)\.)"
        description = re.sub(koz_pattern,
                            f"\\g<1>{target_core_x:.1f}\\g<3>{target_core_y:.1f}) (originally centered at ({base_core_x:.1f}, {base_core_y:.1f}) in the source environment). ",
                            description)
    target_core_force = target_terrain_config.get("max_core_force", DEFAULT_CORE_MAX_FORCE)
    base_core_force = base_terrain_config.get("max_core_force", DEFAULT_CORE_MAX_FORCE)
    if target_core_force != base_core_force:
        description = re.sub(
            r"exceeds (\d+\.?\d*) N \(its structural tolerance\)\.",
            f"exceeds {target_core_force:.1f} N (its structural tolerance) (originally {base_core_force:.1f} N in the source environment).",
            description
        )
    target_joint_force = float(target_terrain_config.get("max_joint_force", DEFAULT_MAX_JOINT_FORCE))
    base_joint_force = float(base_terrain_config.get("max_joint_force", DEFAULT_MAX_JOINT_FORCE))
    target_joint_torque = float(target_terrain_config.get("max_joint_torque", DEFAULT_MAX_JOINT_TORQUE))
    base_joint_torque = float(base_terrain_config.get("max_joint_torque", DEFAULT_MAX_JOINT_TORQUE))
    _num = r"\d+(?:\.\d+)?(?:e[+-]?\d+)?"
    if target_joint_force != base_joint_force:
        description, count = re.subn(
            rf"(maximum linear force capacity of ){_num} N( and a maximum torque capacity of )",
            f"\\g<1>{target_joint_force:.1e} N (originally {base_joint_force:.1e} N in the source environment)\\g<2>",
            description,
            count=1
        )
        if count != 1:
            raise ValueError(
                f"S_05 expected one joint-force prompt target; found {count}"
            )
    if target_joint_torque != base_joint_torque:
        description, count = re.subn(
            rf"(maximum torque capacity of ){_num} Nm( in the nominal mission; these limits may be restricted in mission variants\.)\s*",
            (
                rf"\g<1>{target_joint_torque:.1e} Nm\g<2> "
                rf"(originally {base_joint_torque:.1e} Nm in the source environment)."
                "\n\n"
            ),
            description,
            count=1
        )
        if count != 1:
            raise ValueError(
                f"S_05 expected one joint-torque prompt target; found {count}"
            )
    target_has_walls = bool(target_terrain_config.get("has_walls", DEFAULT_HAS_WALLS))
    base_has_walls = bool(base_terrain_config.get("has_walls", DEFAULT_HAS_WALLS))
    if target_has_walls != base_has_walls:
        if target_has_walls:
            description = re.sub(
                r"(- \*\*Lateral boundaries\*\*: )The scene has no lateral containment walls; the build zone is open at the sides\.",
                r"\g<1>The scene is enclosed by lateral walls (originally no lateral containment walls in the source environment).",
                description
            )
        else:
            description = re.sub(
                r"(- \*\*Lateral boundaries\*\*: )The scene is enclosed by lateral walls \(originally no lateral containment walls in the source environment\)\.",
                r"\g<1>The scene has no lateral containment walls; the build zone is open at the sides (originally enclosed by lateral walls in the source environment).",
                description
            )
    # Restitution and density are latent material properties, not constraints or
    # directly visible geometry. Their target/source values must remain hidden.
    return description

def update_success_criteria_for_visible_changes(
    base_success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,

) -> str:
    criteria = base_success_criteria
    target_max_mass = target_terrain_config.get("max_structure_mass", DEFAULT_MAX_MASS)
    base_max_mass = base_terrain_config.get("max_structure_mass", DEFAULT_MAX_MASS)
    if target_max_mass != base_max_mass:
        mass_pattern = r"(- \*\*Mass Budget\*\*: <= )(\d+\.?\d*) kg\."
        if re.search(mass_pattern, criteria):
            criteria = re.sub(
                mass_pattern,
                f"\\g<1>{target_max_mass:.1f} kg (originally {base_max_mass:.1f} kg in the source environment).",
                criteria
            )
    target_core_x = target_terrain_config.get("core_x", DEFAULT_CORE_X)
    target_core_y = target_terrain_config.get("core_y", DEFAULT_CORE_Y)
    base_core_x = base_terrain_config.get("core_x", DEFAULT_CORE_X)
    base_core_y = base_terrain_config.get("core_y", DEFAULT_CORE_Y)
    if target_core_x != base_core_x or target_core_y != base_core_y:
        koz_criteria_pattern = r"(- \*\*Keep-Out Zone\*\*: Beam center distance to \()(\d+\.?\d*)(, )(\d+\.?\d*)(\) must be >= 1.3m\.)"
        criteria = re.sub(koz_criteria_pattern,
                         f"\\g<1>{target_core_x:.1f}\\g<3>{target_core_y:.1f}) must be >= 1.3m (originally distance to ({base_core_x:.1f}, {base_core_y:.1f}) in the source environment).",
                         criteria)
    target_core_force = target_terrain_config.get("max_core_force", DEFAULT_CORE_MAX_FORCE)
    base_core_force = base_terrain_config.get("max_core_force", DEFAULT_CORE_MAX_FORCE)
    if target_core_force != base_core_force:
        criteria = re.sub(
            r"(peak impact force on the core must not exceed )(\d+\.?\d*)( N\.)",
            f"\\g<1>{target_core_force:.1f} N (originally {base_core_force:.1f} N in the source environment).",
            criteria
        )
        criteria = re.sub(
            r"(Peak force on core <= )(\d+\.?\d*)( N\.)",
            f"\\g<1>{target_core_force:.1f} N (originally {base_core_force:.1f} N in the source environment).",
            criteria
        )
    target_has_walls = bool(target_terrain_config.get("has_walls", DEFAULT_HAS_WALLS))
    base_has_walls = bool(base_terrain_config.get("has_walls", DEFAULT_HAS_WALLS))
    if target_has_walls != base_has_walls:
        if target_has_walls:
            criteria = re.sub(
                r"(- \*\*Lateral boundaries\*\*: )The scene has no lateral containment walls\.",
                r"\g<1>The scene is enclosed by lateral walls (originally no lateral containment walls in the source environment).",
                criteria
            )
        else:
            criteria = re.sub(
                r"(- \*\*Lateral boundaries\*\*: )The scene is enclosed by lateral walls \(originally no lateral containment walls in the source environment\)\.",
                r"\g<1>The scene has no lateral containment walls (originally enclosed by lateral walls in the source environment).",
                criteria
            )
    return criteria

UNIFORM_SUFFIX = uniform_suffix_for_task("S_05")

def get_s05_curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Stage-1",
            "title": "The Fragile Anchor",
            "mutation_description": "Joints and ground anchors have severely reduced shear and torque capacity. A standard heavy shelter concentrates impact loads into few weld joints; when any reaction force or torque exceeds the hidden limit, that joint fails and the structure collapses. The agent must discover the failure mode from feedback and redesign so loads are distributed (e.g. more joints, lighter members, or geometry that keeps no single connection above the threshold).",
            "task_description_suffix": uniform_suffix_for_task("S_05"),
            "terrain_config": {
                "max_joint_force": 5000.0,
                "max_joint_torque": 1e12,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-2",
            "title": "The Gale-Force Crucible",
            "mutation_description": "Extreme hurricane-force winds push everything violently leftward while joints are brittle and snap under even moderate loads. Boulders are much denser and more elastic than nominal, creating devastating high-momentum impacts that ricochet unpredictably. The core is extremely fragile — any direct contact is fatal. A standard shelter would have its joints shredded by wind forces or its roof punctured by heavy, bouncy boulders within seconds. The agent must discover the extreme wind, joint fragility, boulder density, boulder elasticity, and core fragility from catastrophic failure feedback and build an ultra-light, multi-anchor distributed-aerodynamic structure that can survive all of these interacting hazards simultaneously.",
            "task_description_suffix": uniform_suffix_for_task("S_05"),
            "terrain_config": {
                "core_x": 5.0,
                "max_core_force": 30.0,
                "wind_force": -250.0,
                "max_joint_force": 3500.0,
                "max_joint_torque": 3500.0,
                "max_structure_mass": 100.0,
                "meteor_density": 9.0,
                "meteor_restitution": 0.8,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-3",
            "title": "The Oblique Crossfire",
            "mutation_description": "A broad hidden distribution of oblique boulder trajectories combines with stronger gravity, elastic shelter surfaces, lateral containment, a near-zero direct-impact tolerance, a severe mass budget, and restricted joints. Flat roofs collect concentrated impulses and return debris into the enclosure; survival requires a lightweight convex shell with closed side paths and distributed ground load paths.",
            "task_description_suffix": uniform_suffix_for_task("S_05"),
            "terrain_config": {
                "max_structure_mass": 1.0,
                "max_joint_force": 7600.0,
                "max_core_force": 10.0,
                "meteor_vx_range": [-16.0, 16.0],
                "structure_restitution": 0.60,
                "has_walls": True,
                "seed": 123,
            },
            "physics_config": {
                "gravity": (0, -18.0),
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "The Eccentric Gauntlet",
            "mutation_description": "A boundary-adjacent core, restrictive connection capacities, a reduced mass allowance, and lateral containment combine with non-standard gravity, wind, debris trajectories, collision properties, and impact tolerance. The shifted keep-out zone invalidates centered shells, while repeated oblique ricochets punish flat collectors and monolithic cantilevers. Survival requires an asymmetric, independently anchored deflection corridor that distributes its load path and rejects debris before secondary impacts reach the core.",
            "task_description_suffix": uniform_suffix_for_task("S_05"),
            "terrain_config": {
                "core_x": 14.0,
                "max_core_force": 3.5,
                "max_structure_mass": 0.7,
                "max_joint_force": 12000.0,
                "max_joint_torque": 27000.0,
                "wind_force": 95.0,
                "has_walls": True,
                "meteor_restitution": 0.9,
                "meteor_density": 8.0,
                "meteor_vx_range": [-19.0, 19.0],
                "structure_restitution": 0.65,
            },
            "physics_config": {
                "gravity": (0, -23.0),
            },
        },
    ]

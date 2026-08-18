from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

import math

from typing import Any, Dict, List

def gravity_resonant_violent(t: float) -> tuple:
    g_y = 28.0 * math.sin(2.0 * math.pi * t / 0.5)
    return (0.0, g_y)

def gravity_biased_extreme(t: float) -> tuple:
    g_x = 8.0
    g_y = 15.0 * math.sin(2.0 * math.pi * t / 2.0)
    return (g_x, g_y)

def gravity_supercritical(t: float) -> tuple:
    phase = t % 3.6
    if phase < 2.6:
        return (60.0, 45.0)
    if phase < 3.15:
        return (-140.0, 120.0)
    return (30.0, 260.0)

def gravity_chaotic_max(t: float) -> tuple:
    g_x = 18.0 * math.sin(2.0 * math.pi * t / 1.0)
    g_y = 35.0 * math.cos(2.0 * math.pi * t / 0.7)
    return (g_x, g_y)

def gravity_vortex_singularity(t: float) -> tuple:
    phase = t % 2.4
    if phase < 1.45:
        return (-6200.0, 3100.0)
    if phase < 2.0:
        return (11500.0, -7600.0)
    return (2400.0, 12800.0)

def gravity_cataclysm(t: float) -> tuple:
    phase = t % 1.6
    if phase < 0.9:
        return (22000.0, 9000.0)
    if phase < 1.25:
        return (-48000.0, -26000.0)
    return (7000.0, 54000.0)

TASK_DESCRIPTION_SUFFIX = uniform_suffix_for_task("E_01")

import re

def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,

) -> str:
    description = base_description
    target_terrain_config = target_terrain_config or {}
    base_terrain_config = base_terrain_config or {}
    target_physics_config = target_physics_config or {}
    base_physics_config = base_physics_config or {}
    target_arena_y_max = target_terrain_config.get("arena_y_max", 20.0)
    base_arena_y_max = base_terrain_config.get("arena_y_max", 20.0)
    if target_arena_y_max != base_arena_y_max:
        arena_pattern = r"(- \*\*Arena\*\*: A bounded region with x in \[0, 40\] m and y in \[0, )(\d+\.?\d*)(\] m\.)"
        description, count = re.subn(
            arena_pattern,
            f"\\g<1>{target_arena_y_max:.1f}\\g<3> (originally y in [0, {base_arena_y_max:.1f}] m in the source environment).",
            description,
            count=1,
        )
        if count != 1:
            raise ValueError(f"E_01 expected one arena-bound prompt target; found {count}")
    target_bz_y_max = target_terrain_config.get("build_zone_y_max", 18.0)
    base_bz_y_max = base_terrain_config.get("build_zone_y_max", 18.0)
    if target_bz_y_max != base_bz_y_max:
        bz_pattern = r"(- \*\*Build Zone\*\*: Every beam center must be placed within x=\[12\.0, 28\.0\], y=\[6\.0, )(\d+\.?\d*)(\] at build time\.)"
        description, count = re.subn(
            bz_pattern,
            (
                f"\\g<1>{target_bz_y_max:.1f}] at build time "
                f"(originally y=[6.0, {base_bz_y_max:.1f}] "
                "in the source environment)."
            ),
            description,
            count=1,
        )
        if count != 1:
            raise ValueError(f"E_01 expected one build-zone prompt target; found {count}")
    target_joint_limit = float(target_physics_config.get("joint_force_limit", math.inf))
    base_joint_limit = float(base_physics_config.get("joint_force_limit", math.inf))
    if target_joint_limit != base_joint_limit:
        source = (
            "unlimited"
            if math.isinf(base_joint_limit)
            else f"{base_joint_limit:g} N"
        )
        description, count = re.subn(
            r"^- \*\*Joint strength\*\*:.*$",
            (
                f"- **Joint strength**: Joints fail if reaction force exceeds "
                f"{target_joint_limit:g} N (originally {source} in the source "
                "environment)."
            ),
            description,
            count=1,
            flags=re.MULTILINE,
        )
        if count != 1:
            raise ValueError(f"E_01 expected one joint-limit prompt target; found {count}")
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
    target_terrain_config = target_terrain_config or {}
    base_terrain_config = base_terrain_config or {}
    default_mass = 200.0
    target_mass = target_physics_config.get("max_structure_mass", default_mass)
    base_mass = base_physics_config.get("max_structure_mass", default_mass)
    if target_mass != base_mass:
        mass_pattern = r"- \*\*Mass Budget\*\*:.*"
        if re.search(mass_pattern, criteria):
            criteria = re.sub(
                mass_pattern,
                (
                    f"- **Mass Budget**: Total structure mass must not exceed "
                    f"{target_mass:g} kg (source limit {base_mass:g} kg)."
                ),
                criteria,
                count=1,
            )
    default_beams = 12
    target_beams = target_physics_config.get("max_beam_count", default_beams)
    base_beams = base_physics_config.get("max_beam_count", default_beams)
    if target_beams != base_beams:
        beam_pattern = r"- \*\*Beam Limit\*\*:.*"
        if re.search(beam_pattern, criteria):
            criteria = re.sub(
                beam_pattern,
                (
                    f"- **Beam Limit**: Maximum {int(target_beams)} beams "
                    f"(source limit {int(base_beams)})."
                ),
                criteria,
                count=1,
            )
    target_joint_limit = float(target_physics_config.get("joint_force_limit", math.inf))
    base_joint_limit = float(base_physics_config.get("joint_force_limit", math.inf))
    if target_joint_limit != base_joint_limit:
        source = (
            "unlimited"
            if math.isinf(base_joint_limit)
            else f"{base_joint_limit:g} N"
        )
        criteria, count = re.subn(
            r"^- \*\*Joint strength\*\*:.*$",
            (
                f"- **Joint strength**: Reaction force must remain at or below "
                f"{target_joint_limit:g} N (originally {source} in the source "
                "environment); the structure must remain intact."
            ),
            criteria,
            count=1,
            flags=re.MULTILINE,
        )
        if count != 1:
            raise ValueError(
                f"E_01 expected one joint-limit success target; found {count}"
            )
    return criteria

def get_e01_curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Stage-1",
            "title": "Altered Field I",
            "mutation_description": (
                "This stage applies undisclosed non-standard physical conditions; "
                "exact hidden values and mutation directions are not exposed."
            ),
            "task_description_suffix": uniform_suffix_for_task("E_01"),
            "terrain_config": {
                "arena_y_max": 20.0,
                "build_zone_y_max": 18.0,
            },
            "physics_config": {
                "gravity": gravity_resonant_violent,
                "joint_force_limit": 1.0,
            },
        },
        {
            "stage_id": "Stage-2",
            "title": "Altered Field II",
            "mutation_description": (
                "Published geometry and resource limits accompany undisclosed "
                "non-standard physical conditions."
            ),
            "task_description_suffix": uniform_suffix_for_task("E_01"),
            "terrain_config": {
                "arena_y_max": 6.8,
                "build_zone_y_max": 6.6,
                "friction": 0.02,
            },
            "physics_config": {
                "gravity": gravity_supercritical,
                "beam_density_scale": 1e40,
                "linear_damping": 6.0,
                "angular_damping": 4.0,
                "joint_force_limit": 0.5,
                "max_beam_count": 3,
                "max_structure_mass": 0.001,
            },
        },
        {
            "stage_id": "Stage-3",
            "title": "Altered Field III",
            "mutation_description": (
                "Published geometry and resource limits accompany undisclosed "
                "non-standard physical conditions."
            ),
            "task_description_suffix": uniform_suffix_for_task("E_01"),
            "terrain_config": {
                "arena_y_max": 6.52,
                "build_zone_y_max": 6.0,
                "friction": 0.005,
            },
            "physics_config": {
                "gravity": gravity_vortex_singularity,
                "beam_density_scale": 1e41,
                "linear_damping": -42.0,
                "angular_damping": -38.0,
                "joint_force_limit": 0.2,
                "max_beam_count": 2,
                "max_structure_mass": 0.0005,
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "Altered Field IV",
            "mutation_description": (
                "Published geometry and resource limits accompany undisclosed "
                "non-standard physical conditions."
            ),
            "task_description_suffix": uniform_suffix_for_task("E_01"),
            "terrain_config": {
                "arena_y_max": 6.5,
                "build_zone_y_max": 6.0,
                "friction": 0.0,
            },
            "physics_config": {
                "gravity": gravity_cataclysm,
                "beam_density_scale": 1e42,
                "linear_damping": -59.0,
                "angular_damping": -58.0,
                "joint_force_limit": 0.1,
                "max_beam_count": 1,
                "max_structure_mass": 0.0001,
            },
        },
    ]

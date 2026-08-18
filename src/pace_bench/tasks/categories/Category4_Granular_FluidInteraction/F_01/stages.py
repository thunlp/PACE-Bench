from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List

import re

UNIFORM_SUFFIX = uniform_suffix_for_task("F_01")

def _replace_weld_constraint_line(
    description: str,
    *,
    target_force: float,
    base_force: float,
    target_steps: int,
    base_steps: int,

) -> str:
    marker = "- **Constraint**: Beam-to-beam welds break when reaction force **reaches or exceeds** "
    start = description.find(marker)
    if start == -1:
        return description
    end = description.find("\n", start)
    if end == -1:
        end = len(description)
    force_part = f"{target_force:.0f} N"
    if target_force != base_force:
        force_part += f" (originally {base_force:.0f} N in the source environment)"
    steps_part = f"{target_steps} consecutive simulation steps"
    if target_steps != base_steps:
        steps_part += f" (originally {base_steps} in the source environment)"
    new_line = f"{marker}{force_part} for {steps_part}."
    return description[:start] + new_line + description[end:]

def _fmt_float_short(x: float) -> str:
    s = f"{float(x):.6f}".rstrip("0").rstrip(".")
    return s if s else "0"

def _replace_debris_velocity_line(
    description: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]

) -> str:
    default_vx = 2.2
    default_vy = 0.0
    tvx = float(target_terrain_config.get("debris_linear_velocity_x", default_vx))
    bvx = float(base_terrain_config.get("debris_linear_velocity_x", default_vx))
    tvy = float(target_terrain_config.get("debris_linear_velocity_y", default_vy))
    bvy = float(base_terrain_config.get("debris_linear_velocity_y", default_vy))
    if tvx == bvx and tvy == bvy:
        return description
    pat = re.compile(r'\*\*\(([\d.]+),\s*(-?[\d.]+)\)\*\*\s*m/s')
    m = pat.search(description)
    if not m:
        return description
    vx_seg = _fmt_float_short(tvx)
    vy_seg = _fmt_float_short(tvy)
    if tvx != bvx:
        vx_seg += f" (originally {_fmt_float_short(bvx)} in the source environment)"
    if tvy != bvy:
        vy_seg += f" (originally {_fmt_float_short(bvy)} in the source environment)"
    new_text = f"**({vx_seg}, {vy_seg})** m/s"
    return pat.sub(new_text, description, count=1)

_DEFAULT_GRAVITY_F01 = (0, -10.0)

def update_task_description_for_visible_changes(
    base_description: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None, base_physics_config: Dict[str, Any] = None,

) -> str:
    description = base_description
    default_leakage = 0.001
    default_joint_break_force = 50000.0
    default_joint_break_consecutive_steps = 3
    default_fluid_height = 7.0
    target_leakage = target_terrain_config.get("max_leakage_rate", default_leakage)
    base_leakage = base_terrain_config.get("max_leakage_rate", default_leakage)
    if target_leakage != base_leakage:
        before = description
        pattern_obj = r"(the leakage rate does not exceed )(\d+\.?\d*%)"
        if re.search(pattern_obj, description):
            description = re.sub(
                pattern_obj,
                f"\\g<1>{target_leakage*100:.2f}% (originally {base_leakage*100:.2f}% in the source environment)",
                description,
            )
        pattern_legacy = r"(leakage rate remains below )(\d+\.?\d*%)"
        if re.search(pattern_legacy, description):
            description = re.sub(
                pattern_legacy,
                f"\\g<1>{target_leakage*100:.2f}% (originally {base_leakage*100:.2f}% in the source environment)",
                description,
            )
        if description == before:
            raise ValueError("F-01 prompt updater could not replace visible leakage limit")
    target_break = float(target_terrain_config.get("joint_break_force", default_joint_break_force))
    base_break = float(base_terrain_config.get("joint_break_force", default_joint_break_force))
    target_steps = int(target_terrain_config.get("joint_break_consecutive_steps", default_joint_break_consecutive_steps))
    base_steps = int(base_terrain_config.get("joint_break_consecutive_steps", default_joint_break_consecutive_steps))
    if target_break != base_break or target_steps != base_steps:
        before = description
        description = _replace_weld_constraint_line(
            description,
            target_force=target_break,
            base_force=base_break,
            target_steps=target_steps,
            base_steps=base_steps,
        )
        if description == before:
            raise ValueError("F-01 prompt updater could not replace visible weld constraint")
    target_height = target_terrain_config.get("fluid_height", default_fluid_height)
    base_height = base_terrain_config.get("fluid_height", default_fluid_height)
    if target_height != base_height:
        before = description
        pattern = r"(\*\*Reservoir fill height\*\*: )(\d+\.?\d*)( m\.)"
        if re.search(pattern, description):
            description = re.sub(
                pattern,
                f"\\g<1>{target_height:.1f} m (originally {base_height:.1f} m in the source environment).",
                description,
            )
        if description == before:
            raise ValueError("F-01 prompt updater could not replace visible reservoir height")
    return description

def update_success_criteria_for_visible_changes(
    base_success_criteria: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None, base_physics_config: Dict[str, Any] = None,

) -> str:
    criteria = base_success_criteria
    default_leakage = 0.001
    target_leakage = target_terrain_config.get("max_leakage_rate", default_leakage)
    base_leakage = base_terrain_config.get("max_leakage_rate", default_leakage)
    if target_leakage != base_leakage:
        before = criteria
        pattern_le = r"(1\. \*\*Leakage Rate\*\*: Total leakage <= )(\d+\.?\d*%)"
        if re.search(pattern_le, criteria):
            criteria = re.sub(
                pattern_le,
                f"\\g<1>{target_leakage*100:.2f}% (originally {base_leakage*100:.2f}% in the source environment)",
                criteria,
            )
        else:
            pattern_lt = r"(1\. \*\*Leakage Rate\*\*: Total leakage < )(\d+\.?\d*%)"
            if re.search(pattern_lt, criteria):
                criteria = re.sub(
                    pattern_lt,
                    f"\\g<1>{target_leakage*100:.2f}% (originally {base_leakage*100:.2f}% in the source environment)",
                    criteria,
                )
        if criteria == before:
            raise ValueError("F-01 criteria updater could not replace visible leakage limit")
    default_mass = 380.0
    target_mass = float(target_terrain_config.get("max_structure_mass", default_mass))
    base_mass = float(base_terrain_config.get("max_structure_mass", default_mass))
    if target_mass != base_mass:
        before = criteria
        pattern_mass = r"(\*\*Mass Budget\*\*: Total structure mass <= )(\d+\.?\d*)( kg\.)"
        if re.search(pattern_mass, criteria):
            criteria = re.sub(
                pattern_mass,
                f"\\g<1>{target_mass:.0f} kg (originally {base_mass:.0f} kg in the source environment).",
                criteria,
            )
        if criteria == before:
            raise ValueError("F-01 criteria updater could not replace visible mass budget")
    return criteria

def get_f01_curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Stage-1",
            "title": "Active levitation sluice",
            "mutation_description": "A persistent horizontal gravity current drives an adhesive reservoir through a wide moving gate while a sub-source mass ceiling removes passive column stacks. The viable structure is a lightweight zipper truss whose individual cells actively cancel the continuous side load.",
            "task_description_suffix": UNIFORM_SUFFIX,
            "terrain_config": {
                "joint_break_force": 150000.0,
                "joint_break_consecutive_steps": 3,
                "downstream_wall_amplitude": 1.6,
                "downstream_wall_phase_divisor": 90.0,
                "fluid_particle_restitution": 0.2,
                "fluid_particle_friction": 15.0,
                "debris_linear_velocity_x": 7.0,
                "earthquake_impulse_x": 2.0,
                "upward_surge_impulse_y": 2.0,
                "backward_slosh_impulse_x": -1.5,
                "surge_impulses": [2.0, 2.4, 2.8, 3.2, 3.6, 4.0, 4.4, 4.8, 5.2],
                "max_structure_mass": 90.0,
                "max_leakage_rate": 0.003,
            },
            "physics_config": {
                "gravity": (9.0, -7.0),
            },
        },
        {
            "stage_id": "Stage-2",
            "title": "Alternating ballistic siphon",
            "mutation_description": "A low-friction near-elastic reservoir is repeatedly reversed by alternating impulses while stronger horizontal gravity and a fast gate reopen the downstream path. A serpentine kinetic brace must recenter each segment between reversals; a passive vertical pile drifts out of the seal.",
            "task_description_suffix": UNIFORM_SUFFIX,
            "terrain_config": {
                "joint_break_force": 130000.0,
                "joint_break_consecutive_steps": 2,
                "downstream_wall_amplitude": 2.0,
                "downstream_wall_phase_divisor": 40.0,
                "fluid_particle_restitution": 0.98,
                "fluid_particle_friction": 0.001,
                "debris_linear_velocity_x": 8.0,
                "earthquake_impulse_x": 3.0,
                "upward_surge_impulse_y": 2.5,
                "backward_slosh_impulse_x": -2.5,
                "surge_impulses": [1.5, -2.0, 2.5, -3.0, 3.5, -4.0, 4.5, -5.0, 5.5],
                "max_structure_mass": 70.0,
                "max_leakage_rate": 0.002,
            },
            "physics_config": {
                "gravity": (2.0, -10.0),
            },
        },
        {
            "stage_id": "Stage-3",
            "title": "Adhesive fatigue press",
            "mutation_description": "A stronger diagonal gravity current compacts a highly adhesive reservoir through a rapidly cycling gate while debris and asymmetric surges fatigue every concentrated weld. A radial hub must actively distribute and phase-cancel loads that overwhelm a passive column.",
            "task_description_suffix": UNIFORM_SUFFIX,
            "terrain_config": {
                "joint_break_force": 21500.0,
                "joint_break_consecutive_steps": 1,
                "downstream_wall_amplitude": 1.8,
                "downstream_wall_phase_divisor": 60.0,
                "fluid_particle_restitution": 0.15,
                "fluid_particle_friction": 20.0,
                "debris_linear_velocity_x": 20.0,
                "earthquake_impulse_x": 28.0,
                "upward_surge_impulse_y": 4.0,
                "backward_slosh_impulse_x": -4.0,
                "surge_impulses": [2.5, -3.0, 3.5, -4.0, 4.5, -5.0, 5.5, -6.0, 6.5],
                "max_structure_mass": 50.0,
                "max_leakage_rate": 0.001,
            },
            "physics_config": {
                "gravity": (9.0, -8.0),
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "High-frequency stiction reversal crusher",
            "mutation_description": "The strongest diagonal gravity, fastest viable gate, extreme particle adhesion, alternating impulses, debris, and one-step weld failure form a reversal crusher under the smallest mass ceiling. Only a redundant distributed lattice with phase-aware active cancellation remains sealed.",
            "task_description_suffix": UNIFORM_SUFFIX,
            "terrain_config": {
                "joint_break_force": 18000.0,
                "joint_break_consecutive_steps": 1,
                "downstream_wall_amplitude": 2.4,
                "downstream_wall_phase_divisor": 25.0,
                "fluid_particle_restitution": 0.1,
                "fluid_particle_friction": 30.0,
                "debris_linear_velocity_x": 35.0,
                "earthquake_impulse_x": 45.0,
                "upward_surge_impulse_y": 7.0,
                "backward_slosh_impulse_x": -7.0,
                "surge_impulses": [4.0, -5.0, 6.0, -7.0, 8.0, -9.0, 10.0, -11.0, 12.0],
                "max_structure_mass": 35.0,
                "max_leakage_rate": 0.0,
            },
            "physics_config": {
                "gravity": (10.0, -9.0),
            },
        },
    ]

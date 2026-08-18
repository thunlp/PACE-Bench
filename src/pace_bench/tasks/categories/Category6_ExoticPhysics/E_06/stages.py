"""Curriculum stages and visible-prompt updates for E-06."""

from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

import re
from typing import Any, Dict, List


DEFAULT_JOINT_BREAK_FORCE = 280800.0
DEFAULT_JOINT_BREAK_TORQUE = 414000.0
DEFAULT_DAMAGE_LIMIT = 100.0
DEFAULT_BEAM_ANGVEL_THRESH = 2.2
DEFAULT_BEAM_ANGVEL_TOLERANCE_STEPS = 10
DEFAULT_MASS = 120.0
DEFAULT_ANCHOR_LO = 5.0
DEFAULT_ANCHOR_HI = 6.5

TASK_DESCRIPTION_SUFFIX = uniform_suffix_for_task("E_06")


def _visible(current: float, source: float, unit: str = "") -> str:
    text = f"{current:g}{unit}"
    if current != source:
        text += f" (source {source:g}{unit})"
    return text


def _replace_once(text: str, pattern: str, replacement: str, label: str) -> str:
    updated, count = re.subn(pattern, replacement, text, count=1, flags=re.MULTILINE)
    if count != 1:
        raise ValueError(f"E_06 visible prompt update failed for {label}: {count} matches")
    return updated


def _values(
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any],
    base_physics_config: Dict[str, Any],
) -> Dict[str, float]:
    return {
        "force": float(
            target_physics_config.get(
                "joint_break_force", DEFAULT_JOINT_BREAK_FORCE
            )
        ),
        "base_force": float(
            base_physics_config.get(
                "joint_break_force", DEFAULT_JOINT_BREAK_FORCE
            )
        ),
        "torque": float(
            target_physics_config.get(
                "joint_break_torque", DEFAULT_JOINT_BREAK_TORQUE
            )
        ),
        "base_torque": float(
            base_physics_config.get(
                "joint_break_torque", DEFAULT_JOINT_BREAK_TORQUE
            )
        ),
        "damage": float(
            target_physics_config.get("damage_limit", DEFAULT_DAMAGE_LIMIT)
        ),
        "base_damage": float(
            base_physics_config.get("damage_limit", DEFAULT_DAMAGE_LIMIT)
        ),
        "spin": float(
            target_physics_config.get(
                "beam_angvel_thresh", DEFAULT_BEAM_ANGVEL_THRESH
            )
        ),
        "base_spin": float(
            base_physics_config.get(
                "beam_angvel_thresh", DEFAULT_BEAM_ANGVEL_THRESH
            )
        ),
        "spin_steps": int(
            target_physics_config.get(
                "beam_angvel_tolerance_steps",
                DEFAULT_BEAM_ANGVEL_TOLERANCE_STEPS,
            )
        ),
        "base_spin_steps": int(
            base_physics_config.get(
                "beam_angvel_tolerance_steps",
                DEFAULT_BEAM_ANGVEL_TOLERANCE_STEPS,
            )
        ),
        "mass": float(
            target_terrain_config.get("max_structure_mass", DEFAULT_MASS)
        ),
        "base_mass": float(
            base_terrain_config.get("max_structure_mass", DEFAULT_MASS)
        ),
        "anchor_lo": float(
            target_terrain_config.get("allowed_anchor_x_lo", DEFAULT_ANCHOR_LO)
        ),
        "base_anchor_lo": float(
            base_terrain_config.get("allowed_anchor_x_lo", DEFAULT_ANCHOR_LO)
        ),
        "anchor_hi": float(
            target_terrain_config.get("allowed_anchor_x_hi", DEFAULT_ANCHOR_HI)
        ),
        "base_anchor_hi": float(
            base_terrain_config.get("allowed_anchor_x_hi", DEFAULT_ANCHOR_HI)
        ),
    }


def _anchor_text(values: Dict[str, float]) -> str:
    current = f"x=[{values['anchor_lo']:g}, {values['anchor_hi']:g}] m"
    if (
        values["anchor_lo"] != values["base_anchor_lo"]
        or values["anchor_hi"] != values["base_anchor_hi"]
    ):
        current += (
            f" (source x=[{values['base_anchor_lo']:g}, "
            f"{values['base_anchor_hi']:g}] m)"
        )
    return current


def _structural_text(values: Dict[str, float]) -> str:
    steps = int(values["spin_steps"])
    base_steps = int(values["base_spin_steps"])
    step_text = str(steps)
    if steps != base_steps:
        step_text += f" (source {base_steps})"
    return (
        "- **Structural limits**: Joints fail above "
        f"{_visible(values['force'], values['base_force'], ' N')} reaction force "
        f"or {_visible(values['torque'], values['base_torque'], ' N·m')} reaction "
        f"torque; cumulative damage fails at "
        f"{_visible(values['damage'], values['base_damage'], ' pts')}. A beam is "
        f"destroyed if angular velocity exceeds "
        f"{_visible(values['spin'], values['base_spin'], ' rad/s')} for "
        f"{step_text} consecutive simulation step(s)."
    )


def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,
    **kwargs,
) -> str:
    del kwargs
    values = _values(
        target_terrain_config or {},
        base_terrain_config or {},
        target_physics_config or {},
        base_physics_config or {},
    )
    description = _replace_once(
        base_description,
        r"^- \*\*Support\*\*:.*$",
        (
            "- **Support**: Exactly one ground anchor is required within "
            f"{_anchor_text(values)}, at the ground surface y=1.0 m."
        ),
        "task support",
    )
    description = _replace_once(
        description,
        r"^- \*\*Structural limits\*\*:.*$",
        _structural_text(values),
        "task structural limits",
    )
    description = _replace_once(
        description,
        r"^1\. Anchors to the ground.*$",
        (
            "1. Anchors to the ground at exactly one point within "
            f"{_anchor_text(values)}."
        ),
        "task anchor objective",
    )
    return description


def update_success_criteria_for_visible_changes(
    base_success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,
    **kwargs,
) -> str:
    del kwargs
    values = _values(
        target_terrain_config or {},
        base_terrain_config or {},
        target_physics_config or {},
        base_physics_config or {},
    )
    criteria = _replace_once(
        base_success_criteria,
        r"^3\. \*\*Mass\*\*:.*$",
        (
            "3. **Mass**: Initial structure mass does not exceed "
            f"{_visible(values['mass'], values['base_mass'], ' kg')}."
        ),
        "criteria mass objective",
    )
    criteria = _replace_once(
        criteria,
        r"^- \*\*Anchor Limit\*\*:.*$",
        (
            "- **Anchor Limit**: Exactly 1 ground anchor within "
            f"{_anchor_text(values)}, at y=1.0 m."
        ),
        "criteria anchor",
    )
    criteria = _replace_once(
        criteria,
        r"^- \*\*Mass Budget\*\*:.*$",
        (
            "- **Mass Budget**: Initial structure mass <= "
            f"{_visible(values['mass'], values['base_mass'], ' kg')}."
        ),
        "criteria mass budget",
    )
    criteria = _replace_once(
        criteria,
        r"^- \*\*Joints\*\*:.*$",
        _structural_text(values).replace(
            "- **Structural limits**:",
            "- **Joints**: At most 75 joints;",
            1,
        ),
        "criteria structural limits",
    )
    return criteria


def get_e06_curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Stage-1",
            "title": "Endurance Variant I",
            "mutation_description": (
                "Published structural constraints accompany undisclosed "
                "non-standard loading and dissipation conditions."
            ),
            "task_description_suffix": uniform_suffix_for_task("E_06"),
            "terrain_config": {"max_structure_mass": 12.0},
            "physics_config": {
                "gravity": (0, 16),
                "angular_damping": 0.0,
                "joint_break_force": 10800.0,
                "joint_break_torque": 14400.0,
                "beam_angvel_thresh": 2.1,
                "beam_angvel_tolerance_steps": 1,
            },
        },
        {
            "stage_id": "Stage-2",
            "title": "Endurance Variant II",
            "mutation_description": (
                "Published structural constraints accompany undisclosed "
                "non-standard loading, fatigue, and dissipation conditions."
            ),
            "task_description_suffix": uniform_suffix_for_task("E_06"),
            "terrain_config": {"max_structure_mass": 1.0},
            "physics_config": {
                "gravity": (1500, 0),
                "linear_damping": 0.0,
                "angular_damping": 8.0,
                "noise_strength": 0.0,
                "coherent_pulse_interval": 501,
                "coherent_pulse_force": 0.0,
                "joint_break_force": 7200.0,
                "joint_break_torque": 4320.0,
                "damage_limit": 12.0,
                "damage_force_thresh": 3600.0,
                "damage_torque_thresh": 7200.0,
                "cascade_shock_damage": 40.0,
                "beam_angvel_thresh": 6.0,
                "beam_angvel_tolerance_steps": 3,
                "phased_storm_mult": 1.0,
                "phased_storm_start": 501,
                "phased_storm_end": 501,
                "burst_prob": 0.0,
            },
        },
        {
            "stage_id": "Stage-3",
            "title": "Endurance Variant III",
            "mutation_description": (
                "Published structural constraints accompany undisclosed "
                "non-standard loading, fatigue, cascade, and storm conditions."
            ),
            "task_description_suffix": uniform_suffix_for_task("E_06"),
            "terrain_config": {"max_structure_mass": 30.0},
            "physics_config": {
                "gravity": (0, -19),
                "joint_break_force": 6500.0,
                "joint_break_torque": 11000.0,
                "coherent_pulse_interval": 1,
                "coherent_pulse_force": 160.0,
                "angular_damping": 0.0,
                "linear_damping": 0.0,
                "noise_strength": 18.0,
                "damage_limit": 1.0,
                "damage_force_thresh": 1260.0,
                "damage_torque_thresh": 2880.0,
                "cascade_shock_damage": 500.0,
                "beam_angvel_thresh": 1.5,
                "beam_angvel_tolerance_steps": 2,
                "phased_storm_mult": 4.5,
                "phased_storm_start": 15,
                "burst_prob": 0.15,
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "Endurance Variant IV",
            "mutation_description": (
                "Published structural and anchor constraints accompany "
                "undisclosed non-standard loading, fatigue, cascade, and storm "
                "conditions."
            ),
            "task_description_suffix": uniform_suffix_for_task("E_06"),
            "terrain_config": {
                "max_structure_mass": 20.0,
                "allowed_anchor_x_lo": 5.72,
                "allowed_anchor_x_hi": 5.78,
            },
            "physics_config": {
                "gravity": (0, -30),
                "noise_strength": 2.0,
                "coherent_pulse_interval": 1,
                "coherent_pulse_force": 550.0,
                "angular_damping": 0.20,
                "linear_damping": 0.0,
                "joint_break_force": 7200.0,
                "joint_break_torque": 12600.0,
                "damage_limit": 0.9,
                "damage_force_thresh": 792.0,
                "damage_torque_thresh": 1368.0,
                "cascade_shock_damage": 5000.0,
                "beam_angvel_thresh": 1.0,
                "beam_angvel_tolerance_steps": 2,
                "phased_storm_mult": 4.5,
                "burst_prob": 0.4,
                "phased_storm_start": 0,
                "phased_storm_end": 500,
            },
        },
    ]

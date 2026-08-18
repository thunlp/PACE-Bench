from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List


DEFAULT_JOINT_BREAK_FORCE = 6.0
DEFAULT_JOINT_BREAK_TORQUE = 10.0


def _fmt(value: float) -> str:
    return f"{float(value):.6g}"


def _replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise ValueError(f"{label}: expected exactly one match, found {count}")
    return text.replace(old, new, 1)


def _annotated_value(target: float, base: float, unit: str) -> str:
    value = f"{_fmt(target)} {unit}"
    if target == base:
        return value
    return (
        f"{value} (originally {_fmt(base)} {unit} "
        "in the source environment)"
    )


def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,
) -> str:
    del target_terrain_config, base_terrain_config
    target_physics_config = target_physics_config or {}
    base_physics_config = base_physics_config or {}
    base_force = float(
        base_physics_config.get(
            "joint_break_force", DEFAULT_JOINT_BREAK_FORCE
        )
    )
    base_torque = float(
        base_physics_config.get(
            "joint_break_torque", DEFAULT_JOINT_BREAK_TORQUE
        )
    )
    target_force = float(
        target_physics_config.get("joint_break_force", base_force)
    )
    target_torque = float(
        target_physics_config.get("joint_break_torque", base_torque)
    )

    description = base_description
    if target_force != base_force or target_torque != base_torque:
        old = (
            "- **Joint Limits (nominal)**: Joints fail if reaction force exceeds "
            f"{_fmt(base_force)} N or reaction torque exceeds "
            f"{_fmt(base_torque)} N·m (before fatigue decay)."
        )
        new = (
            "- **Joint Limits (nominal)**: Joints fail if reaction force exceeds "
            f"{_annotated_value(target_force, base_force, 'N')} or reaction "
            "torque exceeds "
            f"{_annotated_value(target_torque, base_torque, 'N·m')} "
            "(before fatigue decay)."
        )
        description = _replace_once(description, old, new, "joint limits")
    return description


def update_success_criteria_for_visible_changes(
    base_success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,
) -> str:
    del (
        target_terrain_config,
        base_terrain_config,
        target_physics_config,
        base_physics_config,
    )
    return base_success_criteria


TASK_DESCRIPTION_SUFFIX = uniform_suffix_for_task("E_04")


def get_e04_curriculum_stages() -> List[Dict[str, Any]]:
    stage1_physics = {
        "mass_freq_1": 0.45,
        "mass_amp_1": 0.75,
        "joint_break_force": 100.0,
        "joint_break_torque": 0.002,
        "wind_pressure": 10.0,
    }
    stage2_physics = {
        "joint_break_force": 27000.0,
        "joint_break_torque": 1e-10,
        "wind_pressure": 12000.0,
        "fatigue_tau_seconds": 350.0,
    }
    stage3_physics = {
        "gravity": (9000.0, -20.0),
        "mass_freq_1": 4.0,
        "mass_amp_1": 0.8,
        "mass_phase_gradient": 60.0,
        "joint_break_force": 15000.0,
        "joint_break_torque": 0.00000001,
        "wind_pressure": 9000.0,
        "fatigue_tau_seconds": 90.0,
    }
    stage4_physics = {
        "gravity": (15000.0, -30.0),
        "joint_break_force": 10000.0,
        "joint_break_torque": 1e-10,
        "fatigue_tau_seconds": 60.0,
    }
    stages = []
    for number, physics in enumerate(
        (stage1_physics, stage2_physics, stage3_physics, stage4_physics),
        start=1,
    ):
        stages.append(
            {
                "stage_id": f"Stage-{number}",
                "title": f"Environment Variation {number}",
                "mutation_description": (
                    "A subset of the listed environmental properties differs "
                    "from the source environment."
                ),
                "task_description_suffix": uniform_suffix_for_task("E_04"),
                "terrain_config": {},
                "physics_config": physics,
            }
        )
    return stages

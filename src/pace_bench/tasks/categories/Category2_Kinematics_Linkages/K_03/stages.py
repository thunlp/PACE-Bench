from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List

import re

def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,

) -> str:
    description = base_description
    target_physics_config = target_physics_config or {}
    base_physics_config = base_physics_config or {}
    target_obj = target_terrain_config.get("objects", {})
    base_obj = base_terrain_config.get("objects", {})
    target_shape = target_obj.get("shape", "box")
    base_shape = base_obj.get("shape", "box")
    if target_shape != base_shape:
        names = {"box": "rectangular block", "circle": "circular disk", "triangle": "triangular block"}
        base_name = names.get(base_shape, str(base_shape))
        target_name = names.get(target_shape, str(target_shape))
        shape_pattern = rf"(\*\*Target Object\*\*: )A {re.escape(base_name)}"
        description, replacements = re.subn(
            shape_pattern,
            rf"\1A {target_name} (originally a {base_name})",
            description,
            count=1,
        )
        if replacements != 1:
            raise ValueError(f"K_03 visible shape update expected 1 replacement, got {replacements}")
    return description

def update_success_criteria_for_visible_changes(base_success_criteria: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]) -> str:
    return base_success_criteria

def get_k03_curriculum_stages():
    task_description_suffix = uniform_suffix_for_task("K_03")
    return [
        {
            "stage_id": "Stage-1",
            "title": "Off-Axis Payload",
            "mutation_description": "The otherwise source-equivalent payload is displaced far outside the original gripper axis. Centered clamps descend beside empty space regardless of their force; the viable mechanism must discover the offset and transmit motion laterally from the gantry before grasping and lifting.",
            "task_description_suffix": uniform_suffix_for_task("K_03"),
            "terrain_config": {"objects": {"shape": "box", "mass": 1.0, "friction": 0.6, "x": 8.2, "y": 2.0}},
            "physics_config": {},
        },
        {
            "stage_id": "Stage-2",
            "title": "Off-Axis Crushing Load",
            "mutation_description": "The off-axis payload trap persists while a ten-kilogram box under triple gravity creates roughly thirty times the source static load. Mild damping suppresses uncontrolled rebound, requiring a laterally supported power clamp rather than the Stage-1 passive wedge cage.",
            "task_description_suffix": uniform_suffix_for_task("K_03"),
            "terrain_config": {"objects": {"shape": "box", "mass": 10.0, "friction": 0.6, "x": 8.2, "y": 2.0}},
            "physics_config": {"gravity": (0, -30.0), "linear_damping": 0.5, "angular_damping": 0.5},
        },
        {
            "stage_id": "Stage-3",
            "title": "Off-Axis Slippery Crushing Load",
            "mutation_description": "Stage-2's offset and crushing load intensify: mass rises to eleven kilograms while triple gravity persists, friction falls to 0.15, and both damping coefficients rise to 1.5. Effective static load increases from roughly 300 to 330 N while the lower friction requires deeper geometric capture and greater lateral bracing.",
            "task_description_suffix": uniform_suffix_for_task("K_03"),
            "terrain_config": {"objects": {"shape": "box", "mass": 11.0, "friction": 0.15, "x": 8.2, "y": 2.0}},
            "physics_config": {
                "gravity": (0, -30.0),
                "linear_damping": 1.5,
                "angular_damping": 1.5,
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "Maximum-Difficulty Off-Axis Flywheel",
            "mutation_description": "Every Stage-3 constraint tightens while the box becomes a rolling disk: mass rises from 11 to 12 kg, gravity from -30 to -32 m/s², friction falls from 0.15 to 0.08, and damping rises from 1.5 to 3. Effective static load increases from roughly 330 to 384 N; the off-axis circular payload can also escape sideways, demanding shape-specific multi-point enclosure.",
            "task_description_suffix": uniform_suffix_for_task("K_03"),
            "terrain_config": {"objects": {"shape": "circle", "mass": 12.0, "friction": 0.08, "x": 8.2, "y": 2.0}},
            "physics_config": {
                "gravity": (0, -32.0),
                "linear_damping": 3.0,
                "angular_damping": 3.0,
            },
        },
    ]

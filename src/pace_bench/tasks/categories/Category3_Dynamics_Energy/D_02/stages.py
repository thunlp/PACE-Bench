from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

import re

from typing import Any, Dict, List

_DEFAULT_SLOT1_FLOOR, _DEFAULT_SLOT1_CEIL = 13.2, 14.7

_DEFAULT_SLOT2_FLOOR, _DEFAULT_SLOT2_CEIL = 11.3, 13.3

_DEFAULT_SLOT3_FLOOR, _DEFAULT_SLOT3_CEIL = 12.4, 14.2

_DEFAULT_GRAVITY = (0, -14.0)

def update_task_description_for_visible_changes(
    base_description: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None, base_physics_config: Dict[str, Any] = None,

) -> str:
    description = base_description
    target = target_terrain_config or {}
    base = base_terrain_config or {}
    t_s1_f = target.get("slot1_floor", _DEFAULT_SLOT1_FLOOR)
    t_s1_c = target.get("slot1_ceil", _DEFAULT_SLOT1_CEIL)
    t_s2_f = target.get("slot2_floor", _DEFAULT_SLOT2_FLOOR)
    t_s2_c = target.get("slot2_ceil", _DEFAULT_SLOT2_CEIL)
    t_s3_f = target.get("slot3_floor", _DEFAULT_SLOT3_FLOOR)
    t_s3_c = target.get("slot3_ceil", _DEFAULT_SLOT3_CEIL)
    b_s1_f = base.get("slot1_floor", _DEFAULT_SLOT1_FLOOR)
    b_s1_c = base.get("slot1_ceil", _DEFAULT_SLOT1_CEIL)
    b_s2_f = base.get("slot2_floor", _DEFAULT_SLOT2_FLOOR)
    b_s2_c = base.get("slot2_ceil", _DEFAULT_SLOT2_CEIL)
    b_s3_f = base.get("slot3_floor", _DEFAULT_SLOT3_FLOOR)
    b_s3_c = base.get("slot3_ceil", _DEFAULT_SLOT3_CEIL)
    if (t_s1_f, t_s1_c) != (b_s1_f, b_s1_c) or (t_s2_f, t_s2_c) != (b_s2_f, b_s2_c) or (t_s3_f, t_s3_c) != (b_s3_f, b_s3_c):
        slot1_pattern = r"(\*\*Slot 1\*\* \(x ≈ 17 m\): y in )\[(\d+\.?\d*), (\d+\.?\d*)\]([;.])"
        description, slot1_count = re.subn(
                slot1_pattern,
                lambda m: f"{m.group(1)}[{t_s1_f:.1f}, {t_s1_c:.1f}] (originally [{b_s1_f:.1f}, {b_s1_c:.1f}] in the source environment){m.group(4)}",
                description,
                count=1,
            )
        if slot1_count != 1:
            raise ValueError(
                "D_02 visible-update contract expected exactly one Slot 1 elevation target; "
                f"found {slot1_count}"
            )
        slot2_pattern = r"(\*\*Slot 2\*\* \(x ≈ 21 m\): y in )\[(\d+\.?\d*), (\d+\.?\d*)\]([;.])"
        description, slot2_count = re.subn(
                slot2_pattern,
                lambda m: f"{m.group(1)}[{t_s2_f:.1f}, {t_s2_c:.1f}] (originally [{b_s2_f:.1f}, {b_s2_c:.1f}] in the source environment){m.group(4)}",
                description,
                count=1,
            )
        if slot2_count != 1:
            raise ValueError(
                "D_02 visible-update contract expected exactly one Slot 2 elevation target; "
                f"found {slot2_count}"
            )
        slot3_pattern = r"(\*\*Slot 3\*\* \(x ≈ 19 m\): y in )\[(\d+\.?\d*), (\d+\.?\d*)\]([;.])"
        description, slot3_count = re.subn(
                slot3_pattern,
                lambda m: f"{m.group(1)}[{t_s3_f:.1f}, {t_s3_c:.1f}] (originally [{b_s3_f:.1f}, {b_s3_c:.1f}] in the source environment){m.group(4)}",
                description,
                count=1,
            )
        if slot3_count != 1:
            raise ValueError(
                "D_02 visible-update contract expected exactly one Slot 3 elevation target; "
                f"found {slot3_count}"
            )
    return description

def update_success_criteria_for_visible_changes(
    base_success_criteria: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]

) -> str:
    return base_success_criteria

UNIFORM_SUFFIX = uniform_suffix_for_task("D_02")

def get_d02_curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Stage-1",
            "title": "Viscous Void",
            "mutation_description": "The atmosphere has become significantly more viscous; the jumper will experience rapid velocity decay due to extreme air resistance.",
            "task_description_suffix": uniform_suffix_for_task("D_02"),
            "terrain_config": {},
            "physics_config": {
                "linear_damping": 2.0,
            },
        },
        {
            "stage_id": "Stage-2",
            "title": "Alternating Passages",
            "mutation_description": "The three visible barrier slots occupy a new alternating elevation profile.",
            "task_description_suffix": uniform_suffix_for_task("D_02"),
            "terrain_config": {
                "slot1_floor": 4.8,
                "slot1_ceil": 5.7,
                "slot3_floor": 15.0,
                "slot3_ceil": 15.9,
                "slot2_floor": 4.4,
                "slot2_ceil": 5.3,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-3",
            "title": "Gale-Force Gravity",
            "mutation_description": "Extreme gravitational pull combined with a powerful headwind will rapidly deplete the jumper's momentum and force it toward the pit.",
            "task_description_suffix": uniform_suffix_for_task("D_02"),
            "terrain_config": {},
            "physics_config": {
                "gravity": (0, -35.0),
                "wind": (-20.0, 0),
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "The Perfect Storm",
            "mutation_description": "A combination of extreme gravity, fierce headwind, and significant air resistance creates a nearly impassable barrier.",
            "task_description_suffix": uniform_suffix_for_task("D_02"),
            "terrain_config": {},
            "physics_config": {
                "gravity": (0, -30.0),
                "wind": (-15.0, 0),
                "linear_damping": 1.0,
            },
        },
    ]

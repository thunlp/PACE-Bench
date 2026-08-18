from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

import math
import re
from typing import Any, Dict, List, Optional

try:
    from environment import (
        BARRIER_Y_BOTTOM,
        BARRIER_Y_TOP,
        DEFAULT_TIME_STEP,
        DEFAULT_TIME_STEP_LABEL,
        LAND_TOLERANCE,
        MAX_EPISODE_STEPS,
        MAX_LANDING_ANGLE,
        MAX_SAFE_VERTICAL_SPEED,
        MAX_THRUST,
        MAX_TORQUE,
        MIN_FUEL_REMAINING_AT_LANDING,
        PLATFORM_HALF_WIDTH,
        THRUST_DELAY_STEPS,
        TOTAL_FUEL_IMPULSE,
    )
except ImportError:
    from pace_bench.tasks.categories.Category5_Cybernetics_Control.C_02.environment import (
        BARRIER_Y_BOTTOM,
        BARRIER_Y_TOP,
        DEFAULT_TIME_STEP,
        DEFAULT_TIME_STEP_LABEL,
        LAND_TOLERANCE,
        MAX_EPISODE_STEPS,
        MAX_LANDING_ANGLE,
        MAX_SAFE_VERTICAL_SPEED,
        MAX_THRUST,
        MAX_TORQUE,
        MIN_FUEL_REMAINING_AT_LANDING,
        PLATFORM_HALF_WIDTH,
        THRUST_DELAY_STEPS,
        TOTAL_FUEL_IMPULSE,
    )


_SCALAR = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
_FRACTION_OR_SCALAR = rf"(?:{_SCALAR}/{_SCALAR}|{_SCALAR})"


def _replace_one(text: str, pattern: str, replacement: str, field: str) -> str:
    updated, count = re.subn(pattern, replacement, text, count=1)
    if count != 1:
        raise ValueError(
            f"C-02 prompt update for {field!r} expected one match, found {count}"
        )
    return updated


def _merged_stage_configs(
    target_terrain_config: Dict[str, Any],
    target_physics_config: Optional[Dict[str, Any]],
    stage: Optional[Dict[str, Any]],
) -> tuple[Dict[str, Any], Dict[str, Any]]:
    terrain = dict((stage or {}).get("terrain_config") or {})
    physics = dict((stage or {}).get("physics_config") or {})
    terrain.update(target_terrain_config or {})
    physics.update(target_physics_config or {})
    return terrain, physics


def _fmt_scalar(value: float) -> str:
    numeric = float(value)
    if abs(numeric - round(numeric)) < 1e-9:
        return str(int(round(numeric)))
    return f"{numeric:g}"


def _format_time_step(seconds: float) -> str:
    for numerator, denominator in ((1, 60), (1, 30), (1, 120), (1, 100), (1, 50)):
        if abs(seconds - numerator / denominator) < 1e-9:
            return f"{numerator}/{denominator}"
    return f"{seconds:g}"


def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Optional[Dict[str, Any]] = None,
    base_physics_config: Optional[Dict[str, Any]] = None,
    *,
    stage: Optional[Dict[str, Any]] = None,
) -> str:
    del base_terrain_config, base_physics_config
    description = base_description
    _, physics = _merged_stage_configs(
        target_terrain_config, target_physics_config, stage
    )

    if (
        "barrier_y_top" in physics
        and float(physics["barrier_y_top"]) != BARRIER_Y_TOP
    ):
        target = float(physics["barrier_y_top"])
        description = _replace_one(
            description,
            rf"lower bound is y={_SCALAR} m",
            (
                f"lower bound is y={target:.1f} m "
                f"(originally {BARRIER_Y_TOP:.1f} m in the source environment)"
            ),
            "barrier_y_top",
        )
    if (
        "barrier_y_bottom" in physics
        and float(physics["barrier_y_bottom"]) != BARRIER_Y_BOTTOM
    ):
        target = float(physics["barrier_y_bottom"])
        description = _replace_one(
            description,
            rf"upper bound is y={_SCALAR} m",
            (
                f"upper bound is y={target:.1f} m "
                f"(originally {BARRIER_Y_BOTTOM:.1f} m in the source environment)"
            ),
            "barrier_y_bottom",
        )

    if (
        "platform_half_width" in physics
        and float(physics["platform_half_width"]) != PLATFORM_HALF_WIDTH
    ):
        target_half = float(physics["platform_half_width"])
        target_width = 2.0 * target_half
        source_width = 2.0 * PLATFORM_HALF_WIDTH
        description = _replace_one(
            description,
            rf"valid landing area is {_SCALAR} m total \(center ± {_SCALAR} m\)",
            (
                f"valid landing area is {target_width:.1f} m total "
                f"(center ± {target_half:.1f} m) "
                f"(originally {source_width:.1f} m total "
                f"(center ± {PLATFORM_HALF_WIDTH:.1f} m) "
                "in the source environment)"
            ),
            "platform_half_width",
        )

    if (
        "total_fuel_impulse" in physics
        and float(physics["total_fuel_impulse"]) != TOTAL_FUEL_IMPULSE
    ):
        target = _fmt_scalar(float(physics["total_fuel_impulse"]))
        source = _fmt_scalar(TOTAL_FUEL_IMPULSE)
        description = _replace_one(
            description,
            rf"Total fuel impulse is {_SCALAR} N·s",
            (
                f"Total fuel impulse is {target} N·s "
                f"(originally {source} N·s in the source environment)"
            ),
            "total_fuel_impulse",
        )

    if "max_thrust" in physics and float(physics["max_thrust"]) != MAX_THRUST:
        target = _fmt_scalar(float(physics["max_thrust"]))
        source = _fmt_scalar(MAX_THRUST)
        description = _replace_one(
            description,
            rf"\(world \+y when upright; max {_SCALAR} N\)",
            (
                f"(world +y when upright; max {target} N "
                f"(originally {source} N in the source environment))"
            ),
            "max_thrust",
        )

    if "max_torque" in physics and float(physics["max_torque"]) != MAX_TORQUE:
        target = _fmt_scalar(float(physics["max_torque"]))
        source = _fmt_scalar(MAX_TORQUE)
        description = _replace_one(
            description,
            rf"torque \(max {_SCALAR} N·m\)",
            (
                f"torque (max {target} N·m "
                f"(originally {source} N·m in the source environment))"
            ),
            "max_torque",
        )

    # Command latency is latent actuator behavior, not a hard grading
    # constraint or directly visible state, so its numeric value stays hidden.

    if (
        "max_episode_steps" in physics
        and int(physics["max_episode_steps"]) != MAX_EPISODE_STEPS
    ):
        target = int(physics["max_episode_steps"])
        description = _replace_one(
            description,
            rf"limited to \*\*{_SCALAR} simulation steps\*\*",
            (
                f"limited to **{target} simulation steps** "
                f"(originally {MAX_EPISODE_STEPS} in the source environment)"
            ),
            "max_episode_steps",
        )

    if (
        "time_step" in physics
        and abs(float(physics["time_step"]) - DEFAULT_TIME_STEP) > 1e-12
    ):
        target = _format_time_step(float(physics["time_step"]))
        description = _replace_one(
            description,
            rf"Fixed time step {_FRACTION_OR_SCALAR} s per step",
            (
                f"Fixed time step {target} s per step "
                f"(originally {DEFAULT_TIME_STEP_LABEL} in the source environment)"
            ),
            "time_step",
        )

    if (
        "land_tolerance" in physics
        and abs(float(physics["land_tolerance"]) - LAND_TOLERANCE) > 1e-12
    ):
        target = _fmt_scalar(float(physics["land_tolerance"]))
        source = _fmt_scalar(LAND_TOLERANCE)
        description = _replace_one(
            description,
            rf"within {_SCALAR} m of the ground surface",
            (
                f"within {target} m of the ground surface "
                f"(originally {source} m in the source environment)"
            ),
            "land_tolerance",
        )

    return description


def update_success_criteria_for_visible_changes(
    base_success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Optional[Dict[str, Any]] = None,
    base_physics_config: Optional[Dict[str, Any]] = None,
    *,
    stage: Optional[Dict[str, Any]] = None,
) -> str:
    del base_terrain_config, base_physics_config
    criteria = base_success_criteria
    terrain, physics = _merged_stage_configs(
        target_terrain_config, target_physics_config, stage
    )

    if (
        "max_safe_vertical_speed" in terrain
        and float(terrain["max_safe_vertical_speed"]) != MAX_SAFE_VERTICAL_SPEED
    ):
        target = float(terrain["max_safe_vertical_speed"])
        criteria = _replace_one(
            criteria,
            rf"\|vy\| <= {_SCALAR} m/s",
            (
                f"|vy| <= {target:.2f} m/s "
                f"(originally {MAX_SAFE_VERTICAL_SPEED:.2f} m/s "
                "in the source environment)"
            ),
            "max_safe_vertical_speed",
        )

    if (
        "max_landing_angle" in terrain
        and float(terrain["max_landing_angle"]) != MAX_LANDING_ANGLE
    ):
        target_deg = math.degrees(float(terrain["max_landing_angle"]))
        source_deg = math.degrees(MAX_LANDING_ANGLE)
        criteria = _replace_one(
            criteria,
            rf"\(\|angle\| <= {_SCALAR} degrees\)",
            (
                f"(|angle| <= {target_deg:.2f} degrees "
                f"(originally {source_deg:.2f} degrees "
                "in the source environment))"
            ),
            "max_landing_angle primary",
        )
        criteria = _replace_one(
            criteria,
            rf"\*\*Landing Orientation\*\*: \|angle\| <= {_SCALAR} degrees\.",
            (
                f"**Landing Orientation**: |angle| <= {target_deg:.2f} degrees "
                f"(originally {source_deg:.2f} degrees "
                "in the source environment)."
            ),
            "max_landing_angle summary",
        )

    if (
        "platform_half_width" in physics
        and float(physics["platform_half_width"]) != PLATFORM_HALF_WIDTH
    ):
        target_half = float(physics["platform_half_width"])
        target_width = 2.0 * target_half
        source_width = 2.0 * PLATFORM_HALF_WIDTH
        criteria = _replace_one(
            criteria,
            rf"\*\*{_SCALAR} m total \(center ± {_SCALAR} m\)\*\*",
            (
                f"**{target_width:.1f} m total (center ± {target_half:.1f} m) "
                f"(originally {source_width:.1f} m total "
                f"(center ± {PLATFORM_HALF_WIDTH:.1f} m) "
                "in the source environment)**"
            ),
            "platform_half_width success",
        )

    if (
        "min_fuel_remaining_at_landing" in physics
        and float(physics["min_fuel_remaining_at_landing"])
        != MIN_FUEL_REMAINING_AT_LANDING
    ):
        target = _fmt_scalar(float(physics["min_fuel_remaining_at_landing"]))
        source = _fmt_scalar(MIN_FUEL_REMAINING_AT_LANDING)
        criteria = _replace_one(
            criteria,
            rf"Land with at least {_SCALAR} N·s",
            (
                f"Land with at least {target} N·s "
                f"(originally {source} N·s in the source environment)"
            ),
            "min_fuel_remaining_at_landing",
        )

    return criteria


def apply_visible_prompt_updates(
    task_description: str,
    success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Optional[Dict[str, Any]] = None,
    base_physics_config: Optional[Dict[str, Any]] = None,
    *,
    stage: Optional[Dict[str, Any]] = None,
) -> tuple[str, str]:
    return (
        update_task_description_for_visible_changes(
            task_description,
            target_terrain_config,
            base_terrain_config,
            target_physics_config,
            base_physics_config,
            stage=stage,
        ),
        update_success_criteria_for_visible_changes(
            success_criteria,
            target_terrain_config,
            base_terrain_config,
            target_physics_config,
            base_physics_config,
            stage=stage,
        ),
    )


UNIFORM_SUFFIX = uniform_suffix_for_task("C_02")


def get_c02_curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Stage-1",
            "title": "Fragile Touchdown",
            "mutation_description": "Log only: curriculum stage mutation (Stage-1).",
            "task_description_suffix": uniform_suffix_for_task("C_02"),
            "terrain_config": {"max_safe_vertical_speed": 0.05},
            "physics_config": {},
        },
        {
            "stage_id": "Stage-2",
            "title": "Micro-Corridor",
            "mutation_description": "Log only: curriculum stage mutation (Stage-2).",
            "task_description_suffix": uniform_suffix_for_task("C_02"),
            "terrain_config": {},
            "physics_config": {"barrier_y_bottom": 7.0},
        },
        {
            "stage_id": "Stage-3",
            "title": "The Squeeze",
            "mutation_description": "Log only: curriculum stage mutation (Stage-3).",
            "task_description_suffix": uniform_suffix_for_task("C_02"),
            "terrain_config": {
                "max_safe_vertical_speed": 1.0,
                "max_landing_angle": math.radians(4.5),
            },
            "physics_config": {
                "barrier_y_bottom": 8.5,
                "total_fuel_impulse": 5000.0,
                "max_thrust": 650.0,
                "min_fuel_remaining_at_landing": 650.0,
                "wind_amplitude": 38.0,
                "gust_amplitude": 75.0,
                "platform_half_width": 1.2,
                "thrust_delay_steps": 6,
                "gravity_mutation": {
                    "at_step": 250,
                    "gravity_after": (0, -11.8),
                },
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "The Perfect Storm",
            "mutation_description": "Log only: curriculum stage mutation (Stage-4).",
            "task_description_suffix": uniform_suffix_for_task("C_02"),
            "terrain_config": {
                "max_safe_vertical_speed": 2.6,
                "max_landing_angle": math.radians(7.0),
            },
            "physics_config": {
                "thrust_delay_steps": 12,
                "total_fuel_impulse": 95000.0,
                "max_thrust": 1200.0,
                "min_fuel_remaining_at_landing": 600.0,
                "wind_amplitude": 15.0,
                "gust_amplitude": 20.0,
                "platform_half_width": 1.5,
                "barrier_y_bottom": 15.5,
                "gravity_mutation": {
                    "at_step": 150,
                    "gravity_after": (0, -11.5),
                },
            },
        },
    ]

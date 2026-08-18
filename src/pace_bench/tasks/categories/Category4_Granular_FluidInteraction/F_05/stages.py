from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List

import math

import re

UNIFORM_SUFFIX = uniform_suffix_for_task("F_05")

from pace_bench.tasks.categories.Category4_Granular_FluidInteraction.F_05.environment import WELD_TORQUE_FORCE_RATIO

def _fmt_build_zone_axis(y: float) -> str:
    t = round(y * 100) / 100
    if math.isclose(t, round(t, 1), abs_tol=1e-9):
        return f"{round(t, 1):.1f}"
    s = f"{t:.2f}".rstrip("0").rstrip(".")
    return s

_ROCK_FIXTURES_SUFFIX = " Each rock uses environment-defined contact parameters."

def _f05_joint_limit_float(terrain_config: Dict[str, Any]) -> float:
    inf = float("inf")
    v = terrain_config.get("joint_max_force", inf)
    if v is None:
        return inf
    try:
        return float(v)
    except (TypeError, ValueError):
        return inf

_ROCK_COUNT_WORDS = (
    "zero",
    "one",
    "two",
    "three",
    "four",
    "five",
    "six",
    "seven",
    "eight",
    "nine",
    "ten",

)

def _rock_count_phrase(n: int) -> str:
    if 0 <= n < len(_ROCK_COUNT_WORDS):
        return f"{_ROCK_COUNT_WORDS[n]} rocks"
    return f"{n} rocks"

def _format_rocks_summary(terrain_config: Dict[str, Any]) -> str:
    rocks = terrain_config.get("rocks")
    if not rocks:
        rocks = [
            {"x": 13.5, "y": 1.0, "r": 0.24},
            {"x": 14.5, "y": 1.1, "r": 0.22},
            {"x": 15.5, "y": 1.05, "r": 0.23},
            {"x": 16.5, "y": 1.08, "r": 0.22},
        ]
    parts = []
    for r in rocks:
        rx = float(r.get("x", 15.0))
        ry = float(r.get("y", 1.0))
        rr = float(r.get("radius", r.get("r", 0.2)))
        parts.append(f"({rx:.2f}, {ry:.2f}, r={rr:.2f})")
    return f"{_rock_count_phrase(len(rocks))}: " + "; ".join(parts)

def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] | None = None,
    base_physics_config: Dict[str, Any] | None = None,
    *,
    stage: Dict[str, Any] | None = None,

) -> str:
    description = base_description
    if target_physics_config is None:
        target_physics_config = dict((stage or {}).get("physics_config") or {})
    else:
        target_physics_config = dict(target_physics_config)
    base_physics_config = dict(base_physics_config or {})
    target_x_min = target_terrain_config.get("build_zone_x_min", 12.0)
    target_x_max = target_terrain_config.get("build_zone_x_max", 18.0)
    target_y_min = target_terrain_config.get("build_zone_y_min", 2.0)
    target_y_max = target_terrain_config.get("build_zone_y_max", 4.5)
    base_x_min = base_terrain_config.get("build_zone_x_min", 12.0)
    base_x_max = base_terrain_config.get("build_zone_x_max", 18.0)
    base_y_min = base_terrain_config.get("build_zone_y_min", 2.0)
    base_y_max = base_terrain_config.get("build_zone_y_max", 4.5)
    if (
        target_x_min != base_x_min
        or target_x_max != base_x_max
        or target_y_min != base_y_min
        or target_y_max != base_y_max
    ):
        bz_pat = r"(- \*\*Build zone\*\*: Beam centers must lie in )x=\[[^\]]+\](, y=\[)[^\]]+\]"
        tx0 = f"{float(target_x_min):.1f}"
        tx1 = f"{float(target_x_max):.1f}"
        ty0 = _fmt_build_zone_axis(float(target_y_min))
        ty1 = _fmt_build_zone_axis(float(target_y_max))
        bx0 = f"{float(base_x_min):.1f}"
        bx1 = f"{float(base_x_max):.1f}"
        by0 = _fmt_build_zone_axis(float(base_y_min))
        by1 = _fmt_build_zone_axis(float(base_y_max))
        box = f"x=[{tx0}, {tx1}], y=[{ty0}, {ty1}] (originally x=[{bx0}, {bx1}], y=[{by0}, {by1}] in the source environment)"
        replacement = f"\\g<1>{box}"
        if bz_pat and re.search(bz_pat, description):
            description = re.sub(bz_pat, replacement, description, count=1)
    default_boat_off = 0.0
    target_off = float(target_terrain_config.get("boat_y_offset", default_boat_off))
    base_off = float(base_terrain_config.get("boat_y_offset", default_boat_off))
    if target_off != base_off:
        target_y = 2.5 + target_off
        base_y = 2.5 + base_off
        def _boat_repl(_m: re.Match[str]) -> str:
            return f"{_m.group(1)}{target_y:.1f} m (originally {base_y:.1f} m in the source environment)."
        boat_pat_mut = r"(- \*\*Boat\*\*: Hull center at x\u2248 15 m, y\u2248 )([\d.]+)( m\.(?: originally [\d.]+ in the source environment\.)?)"
        boat_pat_plain = r"(- \*\*Boat\*\*: Hull center at x≈15 m, y≈)([\d.]+)( m\.)"
        if boat_pat_mut and re.search(boat_pat_mut, description):
            description = re.sub(boat_pat_mut, _boat_repl, description, count=1)
        elif re.search(boat_pat_plain, description):
            description = re.sub(boat_pat_plain, _boat_repl, description, count=1)
    target_rocks = _format_rocks_summary(target_terrain_config)
    base_rocks = _format_rocks_summary(base_terrain_config)
    if target_rocks != base_rocks:
        obs_pat_full = r"(- \*\*Submerged obstacles\*\*: )([^\n]+)"
        if obs_pat_full and re.search(obs_pat_full, description):
            description = re.sub(
                obs_pat_full,
                f"\\g<1>{target_rocks} (originally {base_rocks} in the source environment).",
                description,
                count=1,
            )
        else:
            obs_pat_legacy = r"(- \*\*Submerged obstacles\*\*: )([^\n]+)"
            if re.search(obs_pat_legacy, description):
                description = re.sub(
                    obs_pat_legacy,
                    f"\\g<1>{target_rocks} (originally {base_rocks} in the source environment).{_ROCK_FIXTURES_SUFFIX}",
                    description,
                    count=1,
                )
    return description

def update_success_criteria_for_visible_changes(
    base_success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    *,
    stage: Dict[str, Any] | None = None,

) -> str:
    criteria = base_success_criteria
    inf = float("inf")
    target_joint = _f05_joint_limit_float(target_terrain_config)
    base_joint = _f05_joint_limit_float(base_terrain_config)
    _tr = WELD_TORQUE_FORCE_RATIO
    joint_limits_generic = r"Structure remains intact \(all welds survive the episode\)\."
    pattern_generic = (
        r"(3\. \*\*Tertiary\*\*: )" + joint_limits_generic
    )
    pattern_any_terse = r"(3\. \*\*Tertiary\*\*: )Structure remains intact \(all welds survive the episode\)\."
    if not math.isinf(target_joint):
        target_val = float(target_joint)
        torque_limit = target_val * WELD_TORQUE_FORCE_RATIO
        if math.isinf(base_joint):
            orig_f = "∞ in the source environment"
            orig_t = "∞ in the source environment"
        else:
            base_val = float(base_joint)
            base_torque = base_val * WELD_TORQUE_FORCE_RATIO
            orig_f = f"{base_val:.0f} N in the source environment"
            orig_t = f"{base_torque:.0f} N*m in the source environment"
        def _joint_repl(m: re.Match[str]) -> str:
            return m.group(1) + f"Joint weld force <= {target_val:.0f} N (originally {orig_f}) and torque <= {torque_limit:.0f} N*m (originally {orig_t})."
        if pattern_generic and re.search(pattern_generic, criteria):
            criteria = re.sub(pattern_generic, _joint_repl, criteria, count=1)
        elif pattern_any_terse and re.search(pattern_any_terse, criteria):
            criteria = re.sub(pattern_any_terse, _joint_repl, criteria, count=1)
    elif re.search(pattern_generic, criteria):
        criteria = re.sub(pattern_generic, r"3. **Tertiary**: Structure remains intact (all welds survive the episode).", criteria, count=1)
    default_mass = 60.0
    target_mass = float(target_terrain_config.get("max_structure_mass", default_mass))
    base_mass = float(base_terrain_config.get("max_structure_mass", default_mass))
    if target_mass != base_mass:
        mass_pat_plain = r"(- \*\*Mass Budget\*\*: Total structure mass <= )([\d.]+)( kg\.)"
        mass_pat_mut = r"(- \*\*Mass Budget\*\*: Total structure mass <= )([\d.]+)( kg\.)( \(originally [\d.]+ kg in the source environment\))?"
        mass_repl = f"\\g<1>{target_mass:.0f} kg (originally {base_mass:.0f} kg in the source environment)."
        if mass_pat_mut and re.search(mass_pat_mut, criteria):
            criteria = re.sub(mass_pat_mut, mass_repl, criteria, count=1)
        elif re.search(mass_pat_plain, criteria):
            criteria = re.sub(mass_pat_plain, mass_repl, criteria, count=1)
    default_cap = 18.0
    target_cap = float(target_terrain_config.get("max_capsize_angle_deg", default_cap))
    base_cap = float(base_terrain_config.get("max_capsize_angle_deg", default_cap))
    if target_cap != base_cap:
        marker = "at or below "
        idx_marker = criteria.find(marker)
        if idx_marker != -1:
            angle_start = idx_marker + len(marker)
            space_idx = criteria.find(" ", angle_start)
            if space_idx != -1:
                old_angle = criteria[angle_start:space_idx]
                dec_prec = len(old_angle.split(".")[1]) if "." in old_angle else 0
                new_angle = f"{target_cap:.{dec_prec}f}"
                tmp = criteria[:angle_start] + new_angle + criteria[space_idx:]
                orig_idx = tmp.find("(originally", angle_start)
                if orig_idx != -1:
                    orig_end = tmp.find(").", orig_idx)
                    if orig_end != -1:
                        tmp = tmp[:orig_idx] + f"(originally {base_cap:.1f} degrees in the source environment)."
                else:
                    degrees_idx = tmp.find("degrees", angle_start)
                    if degrees_idx != -1:
                        dot_idx = tmp.find(".", degrees_idx)
                        if dot_idx != -1:
                            tmp = tmp[:dot_idx] + f" (originally {base_cap:.1f} degrees in the source environment)."
                criteria = tmp
    default_cwy = 1.90
    default_grace = 180
    target_cwy = float(target_terrain_config.get("cargo_water_y", default_cwy))
    base_cwy = float(base_terrain_config.get("cargo_water_y", default_cwy))
    target_grace = int(target_terrain_config.get("cargo_loss_grace_steps", default_grace))
    base_grace = int(base_terrain_config.get("cargo_loss_grace_steps", default_grace))
    if target_cwy != base_cwy or target_grace != base_grace:
        retention_pat = r"1\. \*\*Cargo Retention\*\*: A particle fails if its center \*\*ever\*\* falls below y = ([\d.]+) m after the first (\d+) physics steps \(brief spawn/settling is ignored\)\."
        def _retention_repl(m: re.Match[str]) -> str:
            grace_val = int(m.group(2))
            grace_part = (
                f"{target_grace} (originally {base_grace} in the source environment)"
                if target_grace != grace_val
                else str(grace_val)
            )
            y_part = (
                f"{target_cwy:.2f} m (originally {base_cwy:.2f} m in the source environment)"
                if not math.isclose(target_cwy, base_cwy, rel_tol=0.0, abs_tol=1e-9)
                else f"{target_cwy:.2f} m"
            )
            return (
                f"1. **Cargo Retention**: A particle fails if its center **ever** falls below {y_part} "
                f"after the first {grace_part} physics steps (brief spawn/settling is ignored)."
            )
        if retention_pat and re.search(retention_pat, criteria, re.IGNORECASE | re.DOTALL):
            criteria = re.sub(retention_pat, _retention_repl, criteria, count=1)
    return criteria

def _merge_f05_terrain(prev: Dict[str, Any], delta: Dict[str, Any]) -> Dict[str, Any]:
    out = dict(prev)
    for k, v in delta.items():
        if k == "cargo" and isinstance(v, dict):
            base_c = out.get("cargo")
            out["cargo"] = {**(base_c if isinstance(base_c, dict) else {}), **v}
        else:
            out[k] = v
    return out

def _merge_f05_physics(prev: Dict[str, Any], delta: Dict[str, Any]) -> Dict[str, Any]:
    out = dict(prev)
    out.update(delta)
    return out

def get_f05_curriculum_stages() -> List[Dict[str, Any]]:
    _reef_barrier_rocks = [
        {"x": 13.15, "y": 2.02, "r": 0.44},
        {"x": 15.0, "y": 1.97, "r": 0.5},
        {"x": 16.85, "y": 2.02, "r": 0.44},
    ]
    _stage3_reef_field = [
        {"x": 13.05, "y": 2.06, "r": 0.46},
        {"x": 15.0, "y": 2.02, "r": 0.52},
        {"x": 16.95, "y": 2.06, "r": 0.46},
        {"x": 15.0, "y": 1.48, "r": 0.33},
    ]
    _raw_stages: List[Dict[str, Any]] = [
        {
            "stage_id": "Stage-1",
            "title": "Metacentric Deficit",
            "mutation_description": "Roll impulses and a changed retention plane test passive stability and closed cargo containment.",
            "task_description_suffix": uniform_suffix_for_task("F_05"),
            "terrain_config": {
                "cargo_water_y": 1.96,
                "cargo_loss_grace_steps": 180,
                "hull_roll_impulse_amplitude": 40.0,
                "hull_roll_impulse_interval_steps": 22,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-2",
            "title": "Raised Loss Plane",
            "mutation_description": "A tighter retention plane and altered roll impulses reduce cargo and stability margins.",
            "task_description_suffix": uniform_suffix_for_task("F_05"),
            "terrain_config": {
                "cargo_water_y": 1.98,
                "hull_roll_impulse_amplitude": 50.0,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-3",
            "title": "Shoal Lock-In",
            "mutation_description": "Coupled obstacle, placement, build-region, weld, forcing, and cargo-contact changes test structural and retention margins.",
            "task_description_suffix": uniform_suffix_for_task("F_05"),
            "terrain_config": {
                "boat_y_offset": -0.10,
                "rocks": list(_stage3_reef_field),
                "build_zone_y_min": 2.58,
                "cargo_water_y": 2.28,
                "joint_max_force": 1750.0,
                "current_strength": 0.64,
                "cargo_restitution": 0.36,
                "cargo": {"friction": 0.62, "linear_damping": 0.24},
                "hull_roll_impulse_amplitude": 20.5,
                "hull_roll_impulse_interval_steps": 73,
                "restoring_coeff": 240.0,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-4",
            "title": "Perfect Storm Assembly Budget",
            "mutation_description": "Coupled forcing, gravity, damping, contact, obstacle, mass, and weld changes test a lightweight passive containment design.",
            "task_description_suffix": uniform_suffix_for_task("F_05"),
            "terrain_config": {
                "boat_y_offset": -0.10,
                "rocks": list(_reef_barrier_rocks),
                "build_zone_y_min": 2.58,
                "current_strength": 2.0,
                "deck_friction": 0.0,
                "max_structure_mass": 18.0,
                "restoring_coeff": 30.0,
                "wind_amplitude": 40.0,
                "wind_frequency": 0.32,
                "lateral_impulse_amplitude": 340.0,
                "lateral_impulse_interval_steps": 35,
                "joint_max_force": 99999.0,
                "hull_roll_impulse_amplitude": 160.0,
                "hull_roll_impulse_interval_steps": 14,
                "rogue_amplitude": 65.0,
                "rogue_interval_steps": 90,
                "cargo_restitution": 0.88,
                "cargo": {"friction": 0.018, "linear_damping": 0.003},
                "wave_amplitude": 55.0,
                "wave_frequency": 0.85,
                "wave2_amplitude": 35.0,
                "wave2_frequency": 0.65,
                "gust_amplitude": 35.0,
                "gust_interval_steps": 16,
            },
            "physics_config": {
                "gravity": (0, -42.0),
                "linear_damping": 0.002,
                "angular_damping": 0.0008,
            },
        },
    ]
    merged_tc: Dict[str, Any] = {}
    merged_pc: Dict[str, Any] = {}
    out_stages: List[Dict[str, Any]] = []
    for s in _raw_stages:
        s = dict(s)
        merged_tc = _merge_f05_terrain(merged_tc, s.get("terrain_config") or {})
        merged_pc = _merge_f05_physics(merged_pc, s.get("physics_config") or {})
        s["terrain_config"] = dict(merged_tc)
        s["physics_config"] = dict(merged_pc)
        s["task_description_suffix"] = uniform_suffix_for_task("F_05")
        out_stages.append(s)
    return out_stages

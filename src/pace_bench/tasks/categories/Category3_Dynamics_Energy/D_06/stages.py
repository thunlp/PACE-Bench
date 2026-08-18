from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

from typing import Any, Dict, List

import re

def _safe_re_sub(pattern, repl, text):
    if not pattern:
        return text
    try:
        return re.sub(pattern, repl, text)
    except TypeError:
        return text

def _safe_re_search(pattern, text):
    if not pattern:
        return None
    try:
        return re.search(pattern, text)
    except TypeError:
        return None

_DEFAULT_DENSITY = 95.0

_LAUNCH_KEYS = (
    ("second_ball_launch_time", 0.4),
    ("third_ball_launch_time", 1.0),
    ("fourth_ball_launch_time", 1.3),
    ("fifth_ball_launch_time", 1.8),
    ("sixth_ball_launch_time", 2.2),
    ("seventh_ball_launch_time", 2.7),

)

_VEL_KEYS = (
    ("ball_velocity_x", -24.0),
    ("ball2_velocity_x", -26.0),
    ("ball3_velocity_x", -24.0),
    ("ball4_velocity_x", -28.0),
    ("ball5_velocity_x", -25.0),
    ("ball6_velocity_x", -26.0),
    ("ball7_velocity_x", -25.0),

)

_FORBIDDEN_ZONE_DEFAULTS = (
    ("forbidden_zone_x_min", "forbidden_zone_x_max", 8.5, 9.5),
    ("forbidden_zone_2_x_min", "forbidden_zone_2_x_max", 7.35, 7.75),
    ("forbidden_zone_3_x_min", "forbidden_zone_3_x_max", 7.78, 8.55),
    ("forbidden_zone_4_x_min", "forbidden_zone_4_x_max", 10.0, 10.5),
    ("forbidden_zone_5_x_min", "forbidden_zone_5_x_max", 7.18, 7.34),

)

_SWEEPER_BAND_DEFAULTS = (
    ("sweeper_band_1_y_min", "sweeper_band_1_y_max", 2.95, 3.55),
    ("sweeper_band_2_y_min", "sweeper_band_2_y_max", 4.15, 4.75),
    ("sweeper_band_3_y_min", "sweeper_band_3_y_max", 1.0, 1.5),
    ("sweeper_band_4_y_min", "sweeper_band_4_y_max", 2.0, 2.5),

)

def _merge_terrain(
    base_terrain_config: Dict[str, Any], target_terrain_config: Dict[str, Any]

) -> Dict[str, Any]:
    merged = dict(base_terrain_config or {})
    merged.update(target_terrain_config or {})
    return merged

def _fmt_time_val(v: float) -> str:
    return f"{float(v):g}"

def _launch_schedule_sentence(tc: Dict[str, Any]) -> str:
    parts = []
    for i, (key, dflt) in enumerate(_LAUNCH_KEYS, start=2):
        v = float(tc.get(key, dflt))
        parts.append(f"ball {i} at t={_fmt_time_val(v)} s")
    return ", ".join(parts)

def _speed_list_sentence(tc: Dict[str, Any]) -> str:
    bits = []
    for i, (key, dflt) in enumerate(_VEL_KEYS, start=1):
        v = float(tc.get(key, dflt))
        if abs(v - round(v)) < 1e-9:
            bits.append(f"ball{i}={int(round(v))}")
        else:
            bits.append(f"ball{i}={v:g}")
    return ", ".join(bits)

def _forbidden_zones_sentence(tc: Dict[str, Any], defaults: tuple) -> str:
    parts = []
    for (kmin, kmax, dmin, dmax) in defaults:
        vmin = float(tc.get(kmin, dmin))
        vmax = float(tc.get(kmax, dmax))
        parts.append(f"[{vmin}, {vmax}]")
    return ", ".join(parts)

def _sweeper_bands_sentence(tc: Dict[str, Any], defaults: tuple) -> str:
    parts = []
    for (kmin, kmax, dmin, dmax) in defaults:
        vmin = float(tc.get(kmin, dmin))
        vmax = float(tc.get(kmax, dmax))
        parts.append(f"[{vmin}, {vmax}]")
    return ", ".join(parts)

def update_task_description_for_visible_changes(
    base_description: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]

) -> str:
    description = base_description
    base_tc = base_terrain_config or {}
    target_tc = target_terrain_config or {}
    base_m = _merge_terrain(base_tc, {})
    tgt_m = _merge_terrain(base_tc, target_tc)
    bx0 = float(base_tc.get("build_zone_x_min", 7.0))
    bx1 = float(base_tc.get("build_zone_x_max", 11.0))
    by0 = float(base_tc.get("build_zone_y_min", 0.5))
    by1 = float(base_tc.get("build_zone_y_max", 5.5))
    tx0 = float(target_tc.get("build_zone_x_min", bx0))
    tx1 = float(target_tc.get("build_zone_x_max", bx1))
    ty0 = float(target_tc.get("build_zone_y_min", by0))
    ty1 = float(target_tc.get("build_zone_y_max", by1))
    if (tx0, tx1, ty0, ty1) != (bx0, bx1, by0, by1):
        bz_pat = r"- \*\*Build Zone\*\*: x=\[[\d.]+\, [\d.]+\] m, y=\[[\d.]+\, [\d.]+\] m\."
        bz_repl = (
            f"- **Build Zone**: x=[{tx0}, {tx1}] m, y=[{ty0}, {ty1}] m "
            f"(originally x=[{bx0}, {bx1}] m, y=[{by0}, {by1}] m in the source environment)."
        )
        description, count = re.subn(bz_pat, bz_repl, description, count=1)
        if count != 1:
            raise ValueError(f"D_06 expected one Build Zone prompt target; found {count}")
        target_pat = r"target zone \(x=\[7, 11\], y=\[0\.5, 5\.5\]\)"
        target_repl = (
            f"target zone (x=[{tx0:g}, {tx1:g}], y=[{ty0:g}, {ty1:g}]) "
            f"(originally x=[{bx0:g}, {bx1:g}], y=[{by0:g}, {by1:g}] "
            "in the source environment)"
        )
        description, count = re.subn(target_pat, target_repl, description, count=1)
        if count != 1:
            raise ValueError(f"D_06 expected one target-zone prompt target; found {count}")
        if tx0 != bx0:
            wall_x = tx0 - 0.05
            base_wall_x = bx0 - 0.05
            obj_pat = r"((?:approximately )x=)([\d.]+)( m \(just left of the build zone minimum when the build zone starts at x=)([\d.]+)( m\))"
            obj_new = (
                rf"\1{wall_x:.2f}\3{tx0}\5 "
                rf"(originally approximately x={base_wall_x:.2f} m when the build zone starts at x={bx0:.1f} m)"
            )
            description, count = re.subn(obj_pat, obj_new, description, count=1)
            if count != 1:
                raise ValueError(f"D_06 expected one left-boundary prompt target; found {count}")
    sched_new = _launch_schedule_sentence(tgt_m)
    sched_old = _launch_schedule_sentence(base_m)
    if sched_new != sched_old:
        sched_pat = r"(ball 2 at t=[\d.]+ s, ball 3 at t=[\d.]+ s, ball 4 at t=[\d.]+ s, ball 5 at t=[\d.]+ s, ball 6 at t=[\d.]+ s, ball 7 at t=[\d.]+ s\.)"
        description, count = re.subn(
                sched_pat,
                rf"{sched_new} (originally {sched_old} in the source environment).",
                description,
                count=1,
            )
        if count != 1:
            raise ValueError(f"D_06 expected one launch-schedule prompt target; found {count}")
    spd_new = _speed_list_sentence(tgt_m)
    spd_old = _speed_list_sentence(base_m)
    if spd_new != spd_old:
        spd_pat = r"(ball1=-?[\d.]+, ball2=-?[\d.]+, ball3=-?[\d.]+, ball4=-?[\d.]+, ball5=-?[\d.]+, ball6=-?[\d.]+, ball7=-?[\d.]+\.)"
        description, count = re.subn(
                spd_pat,
                rf"{spd_new} (originally {spd_old} in the source environment).",
                description,
                count=1,
            )
        if count != 1:
            raise ValueError(f"D_06 expected one launch-speed prompt target; found {count}")
    # Projectile density is latent inertia, not visible geometry or a constraint.
    fz_new = _forbidden_zones_sentence(tgt_m, _FORBIDDEN_ZONE_DEFAULTS)
    fz_old = _forbidden_zones_sentence(base_m, _FORBIDDEN_ZONE_DEFAULTS)
    if fz_new != fz_old:
        fz_pat = r"(\(x in )(\[[\d., ]+\](?:, \[[\d., ]+\])*)(\)\.)"
        description, count = re.subn(
                fz_pat,
                rf"\1{fz_new} (originally {fz_old} in the source environment)\3",
                description,
                count=1,
            )
        if count != 1:
            raise ValueError(f"D_06 expected one Forbidden Zones prompt target; found {count}")
    sb_new = _sweeper_bands_sentence(tgt_m, _SWEEPER_BAND_DEFAULTS)
    sb_old = _sweeper_bands_sentence(base_m, _SWEEPER_BAND_DEFAULTS)
    if sb_new != sb_old:
        sb_pat = r"(\(y in )(\[[\d., ]+\](?:, \[[\d., ]+\])*)(\)\.)"
        description, count = re.subn(
                sb_pat,
                rf"\1{sb_new} (originally {sb_old} in the source environment)\3",
                description,
                count=1,
            )
        if count != 1:
            raise ValueError(f"D_06 expected one Sweeper Bands prompt target; found {count}")
    return description

def update_success_criteria_for_visible_changes(
    base_success_criteria: str, target_terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]

) -> str:
    criteria = base_success_criteria
    default_mass = 10.0
    target_mass = target_terrain_config.get("max_structure_mass", default_mass)
    base_mass = base_terrain_config.get("max_structure_mass", default_mass)
    if target_mass != base_mass:
        pattern = r"(Total structure mass must be strictly less than )(\d+\.?\d*)( kg\.)"
        criteria, count = re.subn(
                pattern,
                f"\\g<1>{target_mass:.1f}\\g<3> (originally {base_mass:.1f} kg in the source environment).",
                criteria,
                count=1,
            )
        if count != 1:
            raise ValueError(f"D_06 expected one mass-budget criteria target; found {count}")
    default_beams = 9
    target_beams = target_terrain_config.get("max_beam_count", default_beams)
    base_beams = base_terrain_config.get("max_beam_count", default_beams)
    if target_beams != base_beams:
        pattern = r"(- \*\*Beam Limit\*\*: Maximum )(\d+)( beams\.)"
        criteria, count = re.subn(
                pattern,
                f"\\g<1>{target_beams} beams (originally {base_beams} beams in the source environment).",
                criteria,
                count=1,
            )
        if count != 1:
            raise ValueError(f"D_06 expected one beam-limit criteria target; found {count}")
    default_joint_force = 880.0
    target_joint = target_terrain_config.get("max_joint_force", default_joint_force)
    base_joint = base_terrain_config.get("max_joint_force", default_joint_force)
    if target_joint != base_joint:
        pattern = r"(reaches or exceeds )(\d+ N)( \(peak failure\)\.)"
        criteria, count = re.subn(
                pattern,
                f"\\g<1>{target_joint:.0f} N (originally {base_joint:.0f} N in the source environment)\\g<3>",
                criteria,
                count=1,
            )
        if count != 1:
            raise ValueError(f"D_06 expected one peak-force criteria target; found {count}")
    default_fatigue = 760.0
    target_fatigue = target_terrain_config.get("joint_fatigue_threshold", default_fatigue)
    base_fatigue = base_terrain_config.get("joint_fatigue_threshold", default_fatigue)
    if target_fatigue != base_fatigue:
        pattern = r"(is strictly greater than )(\d+ N)( for two consecutive simulation steps, the joint fails \(fatigue\)\.)"
        criteria, count = re.subn(
                pattern,
                f"\\g<1>{target_fatigue:.0f} N (originally {base_fatigue:.0f} N in the source environment)\\g<3>",
                criteria,
                count=1,
            )
        if count != 1:
            raise ValueError(f"D_06 expected one fatigue-force criteria target; found {count}")
    return criteria

UNIFORM_SUFFIX = uniform_suffix_for_task("D_06")

def get_d06_curriculum_stages() -> List[Dict[str, Any]]:
    return [
        {
            "stage_id": "Stage-1",
            "title": "Needle-slot contactless anchor",
            "mutation_description": "A near-zero joint limit conflicts with low projectile damping and a closed right corridor. A load-bearing anchored catcher shatters, while waiting for the boundary causes an order violation; interception must occur with detached absorption through the narrow legal x-slot.",
            "task_description_suffix": uniform_suffix_for_task("D_06"),
            "terrain_config": {
                "max_joint_force": 0.6,
                "forbidden_zone_4_x_min": 9.5,
                "forbidden_zone_4_x_max": 11.0,
                "ball_linear_damping": 0.08,
                "ball_density": 180.0,
                "ball_velocity_x": -34.0,
                "ball2_velocity_x": -36.0,
                "ball3_velocity_x": -35.0,
                "ball4_velocity_x": -38.0,
                "ball5_velocity_x": -35.0,
                "ball6_velocity_x": -37.0,
                "ball7_velocity_x": -36.0,
                "second_ball_launch_time": 0.75,
                "third_ball_launch_time": 1.5,
                "fourth_ball_launch_time": 2.25,
                "fifth_ball_launch_time": 3.0,
                "sixth_ball_launch_time": 3.75,
                "seventh_ball_launch_time": 4.5,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-2",
            "title": "Wind-locked high-inertia barrage",
            "mutation_description": "The extended legal corridor is split by a broad forbidden strip while dense, weakly damped projectiles arrive in a compressed nonuniform sequence. Extreme structure-coupled crosswind sweeps detached absorbers out of alignment, but impact and repeated wind reactions still approach the joint peak and fatigue limits, requiring a connected distributed absorption cage.",
            "task_description_suffix": uniform_suffix_for_task("D_06"),
            "terrain_config": {
                "build_zone_x_max": 12.5,
                "forbidden_zone_4_x_min": 10.1,
                "forbidden_zone_4_x_max": 11.4,
                "ball_linear_damping": 0.06,
                "ball_density": 300.0,
                "max_joint_force": 1000.0,
                "joint_fatigue_threshold": 850.0,
                "ball_velocity_x": -43.0,
                "ball2_velocity_x": -47.0,
                "ball3_velocity_x": -44.0,
                "ball4_velocity_x": -50.0,
                "ball5_velocity_x": -45.0,
                "ball6_velocity_x": -48.0,
                "ball7_velocity_x": -46.0,
                "second_ball_launch_time": 0.34,
                "third_ball_launch_time": 0.71,
                "fourth_ball_launch_time": 1.11,
                "fifth_ball_launch_time": 1.54,
                "sixth_ball_launch_time": 2.0,
                "seventh_ball_launch_time": 2.49,
                "wind_on_structure": True,
                "structure_wind_scale": 56.0,
                "wind_amplitude": 50.0,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-3",
            "title": "Extreme density storm — fragile joints, violent gravity, ultra-compressed launch",
            "mutation_description": "Curriculum variant: extreme projectile inertia combined with near-zero structural tolerance, ultra-compressed sequential launch, and violent gravity oscillation creating conflicting demands — heavy fast balls require strong absorption but joints shatter on contact, while compressed timing leaves no recovery window (evaluation metadata only).",
            "task_description_suffix": uniform_suffix_for_task("D_06"),
            "terrain_config": {
                "ball_density": 520.0,
                "max_joint_force": 55.0,
                "joint_fatigue_threshold": 30.0,
                "ball_velocity_x": -52.0,
                "ball2_velocity_x": -54.0,
                "ball3_velocity_x": -52.0,
                "ball4_velocity_x": -58.0,
                "ball5_velocity_x": -53.0,
                "ball6_velocity_x": -54.0,
                "ball7_velocity_x": -53.0,
                "second_ball_launch_time": 0.14,
                "third_ball_launch_time": 0.34,
                "fourth_ball_launch_time": 0.54,
                "fifth_ball_launch_time": 0.74,
                "sixth_ball_launch_time": 0.94,
                "seventh_ball_launch_time": 1.14,
                "gravity_pulse_amplitude": 10.0,
                "gravity_pulse_period": 0.4,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-4",
            "title": "Stage-3 storm + bouncy balls, lighter self-damping, stronger structural gusts",
            "mutation_description": "Curriculum variant: Stage-3-class hazards plus additional projectile and coupling stressors (evaluation metadata only).",
            "task_description_suffix": uniform_suffix_for_task("D_06"),
            "terrain_config": {
                "ball_density": 232.0,
                "max_joint_force": 300.0,
                "joint_fatigue_threshold": 225.0,
                "ball_velocity_x": -36.0,
                "ball2_velocity_x": -38.0,
                "ball3_velocity_x": -36.0,
                "ball4_velocity_x": -40.0,
                "ball5_velocity_x": -37.0,
                "ball6_velocity_x": -38.0,
                "ball7_velocity_x": -37.0,
                "second_ball_launch_time": 0.26,
                "third_ball_launch_time": 0.58,
                "fourth_ball_launch_time": 0.88,
                "fifth_ball_launch_time": 1.18,
                "sixth_ball_launch_time": 1.48,
                "seventh_ball_launch_time": 1.78,
                "gravity_pulse_amplitude": 3.5,
                "gravity_pulse_period": 1.0,
                "wind_on_structure": True,
                "structure_wind_scale": 0.178,
                "wind_amplitude": 9.0,
                "ball_restitution": 0.24,
                "ball_linear_damping": 0.52,
            },
            "physics_config": {},
        },
    ]

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
    target_reach = target_terrain_config.get("target_reach", 12.0)
    base_reach = base_terrain_config.get("target_reach", 12.0)
    if target_reach != base_reach:
        pattern = r"(- \*\*Goal\*\*: Reach x >= )(\d+\.?\d*)m(?: \(originally [^)]+\))?\.?"
        description = re.sub(pattern, f"\\g<1>{target_reach:.1f}m (originally {base_reach:.1f}m in the source environment).", description)
    target_mass = target_terrain_config.get("max_structure_mass", 15000.0)
    base_mass = base_terrain_config.get("max_structure_mass", 15000.0)
    if target_mass != base_mass:
        pattern = r"(- \*\*Mass Limit\*\*: < )([\d,]+)( kg)(?: \(originally [^)]+\))?\.?"
        description = re.sub(pattern, f"\\g<1>{target_mass:,.0f} kg (originally {base_mass:,.0f} kg in the source environment).", description)
    target_load_mass = target_terrain_config.get("load_mass", 500.0)
    base_load_mass = base_terrain_config.get("load_mass", 500.0)
    if target_load_mass != base_load_mass:
        pattern = r"(Each payload has mass \*\*)(\d+,?\d*)( kg\*\*)(.*?)(\s+The first payload)"
        def _replace_mass(m):
            tail = m.group(4)
            if tail.endswith(")."):
                tail = tail[:-2] + ") (originally {0:,.0f} kg in the source environment).".format(base_load_mass)
            else:
                tail = tail + " (originally {0:,.0f} kg in the source environment).".format(base_load_mass)
            return f"{m.group(1)}{target_load_mass:,.0f}{m.group(3)}{tail}{m.group(5)}"
        if re.search(pattern, description):
            description = re.sub(pattern, _replace_mass, description, count=1)
    default_load_attach = 5.0
    default_load_2_attach = 15.0
    target_t1 = float(target_terrain_config.get("load_attach_time", default_load_attach))
    target_t2 = float(target_terrain_config.get("load_2_attach_time", default_load_2_attach))
    base_t1 = float(base_terrain_config.get("load_attach_time", default_load_attach))
    base_t2 = float(base_terrain_config.get("load_2_attach_time", default_load_2_attach))
    if target_t1 != base_t1 or target_t2 != base_t2:
        pattern1 = r"\(e\.g\., at (t=)(\d+\.?\d*)(s and t=)(\d+\.?\d*)(s)\)\.\s*"
        pattern2 = r"\(applied at (t=)(\d+\.?\d*)(s and t=)(\d+\.?\d*)(s)\)\.\s*"
        if re.search(pattern1, description):
            description = re.sub(
                pattern1,
                f"(e.g., at \\g<1>{target_t1:.1f}s and t={target_t2:.1f}s (originally {base_t1:.1f}s and {base_t2:.1f}s in the source environment). ",
                description,
            )
        if re.search(pattern2, description):
            description = re.sub(
                pattern2,
                f"(applied at \\g<1>{target_t1:.1f}s and t={target_t2:.1f}s (originally {base_t1:.1f}s and {base_t2:.1f}s in the source environment). ",
                description,
            )
    target_duration = float(target_terrain_config.get("load_duration", 10.0))
    base_duration = float(base_terrain_config.get("load_duration", 10.0))
    if target_duration != base_duration:
        pattern = r"(Support all applied payloads for )(\d+\.?\d*)( seconds each)"
        if re.search(pattern, description):
            description = re.sub(
                pattern,
                f"\\g<1>{target_duration:.1f} seconds each (originally {base_duration:.1f} seconds in the source environment) ",
                description,
            )
    default_internal_force = 100000000.0
    default_internal_torque = 100000000.0
    target_f = target_terrain_config.get("max_internal_force", default_internal_force)
    base_f = base_terrain_config.get("max_internal_force", default_internal_force)
    target_t = target_terrain_config.get("max_internal_torque", default_internal_torque)
    base_t = base_terrain_config.get("max_internal_torque", default_internal_torque)
    if target_f != base_f:
        pattern = r"(Beam-to-beam joints fail if force exceeds \*\*)([\d,]+)( N\*\*)(?: \(originally [^)]+\))?"
        description = re.sub(pattern, f"\\g<1>{target_f:,.0f} N** (originally {base_f:,.0f} N in the source environment)", description)
    if target_t != base_t:
        pattern = r"(or torque exceeds \*\*)([\d,]+)( N·m\*\*)(?: \(originally [^)]+\))?\.?"
        description = re.sub(pattern, f"\\g<1>{target_t:,.0f} N·m** (originally {base_t:,.0f} N·m in the source environment).", description, count=1)
    default_anchor_f = 100000000.0
    default_anchor_t = 100000000.0
    target_af = target_terrain_config.get("max_anchor_force", default_anchor_f)
    base_af = base_terrain_config.get("max_anchor_force", default_anchor_f)
    target_at = target_terrain_config.get("max_anchor_torque", default_anchor_t)
    base_at = base_terrain_config.get("max_anchor_torque", default_anchor_t)
    if target_af != base_af or target_at != base_at:
        pattern_wa = r"(- \*\*Wall Anchor Limits\*\*: Wall anchors fail if force exceeds \*\*)([\d,]+)( N\*\* or torque exceeds \*\*)([\d,]+)( N·m\*\*)(?: \(originally [^)]+\))? \(exceeding causes anchor failure\)\."
        if re.search(pattern_wa, description):
            description = re.sub(
                pattern_wa,
                f"\\g<1>{target_af:,.0f} N** or torque exceeds **{target_at:,.0f} N·m** (originally {base_af:,.0f} N and {base_at:,.0f} N·m in the source environment) (exceeding causes anchor failure).",
                description,
            )
    target_mth = target_terrain_config.get("min_tip_height_limit", -15.0)
    base_mth = base_terrain_config.get("min_tip_height_limit", -15.0)
    if target_mth != base_mth:
        pattern = r"(- \*\*Minimum Tip Height\*\*: The structure must not sag below y = )(-?\d+\.?\d*)( m )(?:\(originally [^)]+\) )?"
        if re.search(pattern, description):
            description = re.sub(pattern, f"\\g<1>{target_mth:.1f} m (originally {base_mth:.1f} m in the source environment) ", description)
    default_tol = 1.0
    target_tol = float(target_terrain_config.get("reach_tolerance", default_tol))
    base_tol = float(base_terrain_config.get("reach_tolerance", default_tol))
    if target_tol != base_tol:
        pattern = r"(- \*\*Reach Deflection Tolerance\*\*: .*? within )(\d+\.?\d*)( m )(?:\(originally [^)]+\) )?of the target\."
        if re.search(pattern, description):
            description = re.sub(pattern, f"\\g<1>{target_tol:.1f} m (originally {base_tol:.1f} m in the source environment) of the target.", description)
    target_forbidden = target_terrain_config.get("forbidden_anchor_y")
    base_forbidden = base_terrain_config.get("forbidden_anchor_y")
    if target_forbidden is not None and len(target_forbidden) >= 2:
        y_min, y_max = float(target_forbidden[0]), float(target_forbidden[1])
        base_str = "no restrictions"
        if base_forbidden is not None and len(base_forbidden) >= 2:
            base_str = f"y = [{float(base_forbidden[0]):.1f}, {float(base_forbidden[1]):.1f}] m"
        pattern_initial = r"(- \*\*Forbidden Anchor Zones\*\*: )Wall anchors may be restricted to certain vertical segments \(y range\)\. In the source environment there are no restrictions\."
        pattern_updated = r"(- \*\*Forbidden Anchor Zones\*\*: )Anchors are forbidden in y = \[[^\]]+\] m \(originally [^)]+\)\."
        replacement = f"\\g<1>Anchors are forbidden in y = [{y_min:.1f}, {y_max:.1f}] m (originally {base_str} in the source environment)."
        if re.search(pattern_initial, description):
            description = re.sub(pattern_initial, replacement, description)
        elif re.search(pattern_updated, description):
            description = re.sub(pattern_updated, replacement, description)
    if target_terrain_config.get("obstacle_active", False):
        rects = target_terrain_config.get("obstacle_rects", [])
        if rects:
            parts = []
            for rect in rects:
                if len(rect) >= 4:
                    x_min, y_min, x_max, y_max = float(rect[0]), float(rect[1]), float(rect[2]), float(rect[3])
                    parts.append(f"x = [{x_min:.1f}, {x_max:.1f}] m, y = [{y_min:.1f}, {y_max:.1f}] m")
            obstacle_desc = "; ".join(parts) if parts else "static obstructions present"
        else:
            obstacle_desc = "static obstructions present"
        base_rects = base_terrain_config.get("obstacle_rects", [])
        if base_terrain_config.get("obstacle_active", False) and base_rects:
            base_parts = []
            for rect in base_rects:
                if len(rect) >= 4:
                    x_min, y_min, x_max, y_max = float(rect[0]), float(rect[1]), float(rect[2]), float(rect[3])
                    base_parts.append(f"x = [{x_min:.1f}, {x_max:.1f}] m, y = [{y_min:.1f}, {y_max:.1f}] m")
            originally_str = ("; ".join(base_parts) + " in the source environment") if base_parts else "static obstructions in the source environment"
        else:
            originally_str = "none in the source environment"
        pattern = r"(- \*\*Obstacles\*\*: )(.*?)( \(originally )(none in the source environment|static obstructions in the source environment|.*?)(\)\.)"
        if re.search(pattern, description):
            replacement = r"\g<1>Static obstructions occupy axis-aligned region(s): " + obstacle_desc + " (originally " + originally_str + ")."
            description = re.sub(pattern, replacement, description)
    target_load_type = target_terrain_config.get("load_type", "static")
    base_load_type = base_terrain_config.get("load_type", "static")
    target_drop = float(target_terrain_config.get("drop_height", 10.0))
    base_drop = float(base_terrain_config.get("drop_height", 10.0))
    if target_load_type == "dropped":
        pattern_static = r"in the source environment payloads are placed statically \(no drop\)\.?"
        drop_sentence = f"Payloads are **dropped** from {target_drop:.1f} m height (originally placed statically in the source environment)."
        if re.search(pattern_static, description):
            description = re.sub(pattern_static, drop_sentence, description)
    elif base_load_type == "dropped" and target_load_type == "static":
        pattern_dropped = r"Payloads are \*\*dropped\*\* from [\d.]+ m height \(originally placed statically in the source environment\)\. "
        if re.search(pattern_dropped, description):
            description = re.sub(pattern_dropped, "in the source environment payloads are placed statically (no drop). ", description)
    target_strength_map = target_terrain_config.get("anchor_strength_map", None)
    base_strength_map = base_terrain_config.get("anchor_strength_map", None)
    if target_strength_map and len(target_strength_map) > 0:
        parts = []
        for entry in target_strength_map:
            if len(entry) >= 4:
                y_lo, y_hi, f_mult, t_mult = float(entry[0]), float(entry[1]), float(entry[2]), float(entry[3])
                parts.append(f"y = [{y_lo:.1f}, {y_hi:.1f}] m: force and torque at {f_mult*100:.2f}% and {t_mult*100:.2f}% of base limits")
        if parts:
            strength_desc = "; ".join(parts)
            base_str = "none in the source environment"
            if base_strength_map and len(base_strength_map) > 0 and len(base_strength_map[0]) >= 4:
                be = base_strength_map[0]
                base_str = f"y = [{float(be[0]):.1f}, {float(be[1]):.1f}] m at {float(be[2])*100:.2f}%/{float(be[3])*100:.2f}% in the source environment"
            pattern_anchor = r"(When segment-specific anchor strength applies, the vertical segment \(y range\) and force/torque multipliers are stated explicitly\.)"
            pattern_anchor_updated = r"(Regional anchor weakness: .*? \(originally [^)]+\)\.)"
            replacement_anchor = f"Regional anchor weakness: {strength_desc} (originally {base_str})."
            if re.search(pattern_anchor, description):
                description = re.sub(pattern_anchor, replacement_anchor, description)
            elif re.search(pattern_anchor_updated, description):
                description = re.sub(pattern_anchor_updated, replacement_anchor, description)
    return description

def update_success_criteria_for_visible_changes(
    base_success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,

) -> str:
    criteria = base_success_criteria
    target_reach = target_terrain_config.get("target_reach", 12.0)
    base_reach = base_terrain_config.get("target_reach", 12.0)
    if target_reach != base_reach:
        pattern = r"(\(Tip reaches x >= )(\d+\.?\d*)m(?: \(originally [^)]+\))?\)\."
        criteria, count = re.subn(
            pattern,
            (
                f"\\g<1>{target_reach:.1f}m "
                f"(originally {base_reach:.1f}m in the source environment))."
            ),
            criteria,
            count=1,
        )
        if count != 1:
            raise ValueError(
                f"S_03 expected one reach success-criteria target; found {count}"
            )
    target_mass = target_terrain_config.get("max_structure_mass", 15000.0)
    base_mass = base_terrain_config.get("max_structure_mass", 15000.0)
    if target_mass != base_mass:
        pattern = r"(- \*\*Mass Budget\*\*: < )([\d,]+)( kg)(?: \(originally [^)]+\))?\.?"
        criteria = re.sub(pattern, f"\\g<1>{target_mass:,.0f} kg (originally {base_mass:,.0f} kg in the source environment).", criteria)
    target_load_mass = target_terrain_config.get("load_mass", 500.0)
    base_load_mass = base_terrain_config.get("load_mass", 500.0)
    if target_load_mass != base_load_mass:
        pattern = r"(- \*\*Payload Mass\*\*: )([\d,]+)( kg per applied load)(?: \(originally [^)]+\))?\.?"
        criteria = re.sub(pattern, f"\\g<1>{target_load_mass:,.0f}\\g<3> (originally {base_load_mass:,.0f} kg in the source environment).", criteria)
    target_duration = float(target_terrain_config.get("load_duration", 10.0))
    base_duration = float(base_terrain_config.get("load_duration", 10.0))
    if target_duration != base_duration:
        pattern = r"(Successfully supports all payloads for the )(\d+\.?\d*)(s test duration)(?: \(originally [^)]+\))?\.?"
        if re.search(pattern, criteria):
            criteria = re.sub(
                pattern,
                f"\\g<1>{target_duration:.1f}s test duration (originally {base_duration:.1f}s in the source environment).",
                criteria,
            )
        pattern_hold = r"(- \*\*Payload Hold Duration\*\*: .*?hold duration \()(\d+\.?\d*)( s per payload\))(?: \(originally [^)]+\))?\.?"
        if re.search(pattern_hold, criteria):
            criteria = re.sub(
                pattern_hold,
                f"\\g<1>{target_duration:.1f}\\g<3> (originally {base_duration:.1f} s per payload in the source environment).",
                criteria,
            )
    default_internal = 100000000.0
    target_f = target_terrain_config.get("max_internal_force", default_internal)
    base_f = base_terrain_config.get("max_internal_force", default_internal)
    target_t = target_terrain_config.get("max_internal_torque", default_internal)
    base_t = base_terrain_config.get("max_internal_torque", default_internal)
    if target_f != base_f:
        pattern = r"(- \*\*Internal Joint Limits\*\*: Max force )([\d,]+)( N)(?: \(originally [^)]+\))?;"
        criteria = re.sub(pattern, f"\\g<1>{target_f:,.0f} N (originally {base_f:,.0f} N in the source environment);", criteria)
    if target_t != base_t:
        pattern = r"(- \*\*Internal Joint Limits\*\*:.*?max torque )([\d,]+)( N·m )(?:\(originally [^)]+\) )?\("
        criteria = re.sub(pattern, f"\\g<1>{target_t:,.0f} N·m (originally {base_t:,.0f} N·m in the source environment) (", criteria)
    default_anchor = 100000000.0
    target_af = target_terrain_config.get("max_anchor_force", default_anchor)
    base_af = base_terrain_config.get("max_anchor_force", default_anchor)
    target_at = target_terrain_config.get("max_anchor_torque", default_anchor)
    base_at = base_terrain_config.get("max_anchor_torque", default_anchor)
    if target_af != base_af or target_at != base_at:
        pattern_wa = r"(- \*\*Wall Anchor Limits\*\*: Max force )([\d,]+)( N; max torque )([\d,]+)( N·m )(?:\(originally [^)]+\) )?\(exceeding causes failure\)\."
        if re.search(pattern_wa, criteria):
            criteria = re.sub(
                pattern_wa,
                f"\\g<1>{target_af:,.0f}\\g<3>{target_at:,.0f} N·m (originally {base_af:,.0f} N and {base_at:,.0f} N·m in the source environment) (exceeding causes failure).",
                criteria,
            )
    target_mth = target_terrain_config.get("min_tip_height_limit", -15.0)
    base_mth = base_terrain_config.get("min_tip_height_limit", -15.0)
    if target_mth != base_mth:
        pattern_mth = r"(y >= )(-?\d+\.?\d*)( m\))(?: \(originally [^)]+\))?\.?"
        if re.search(pattern_mth, criteria):
            criteria = re.sub(pattern_mth, f"\\g<1>{target_mth:.1f} m) (originally {base_mth:.1f} m in the source environment).", criteria)
    default_tol = 1.0
    target_tol = float(target_terrain_config.get("reach_tolerance", default_tol))
    base_tol = float(base_terrain_config.get("reach_tolerance", default_tol))
    if target_tol != base_tol:
        pattern_tol = r"(- \*\*Reach Tolerance\*\*: Under load, tip x may be up to )(\d+\.?\d*)( m )(?:\(originally [^)]+\) )?short of target and still satisfy reach\."
        if re.search(pattern_tol, criteria):
            criteria = re.sub(pattern_tol, f"\\g<1>{target_tol:.1f} m (originally {base_tol:.1f} m in the source environment) short of target and still satisfy reach.", criteria)
    target_forbidden = target_terrain_config.get("forbidden_anchor_y")
    base_forbidden = base_terrain_config.get("forbidden_anchor_y")
    if target_forbidden is not None and len(target_forbidden) >= 2:
        y_min, y_max = float(target_forbidden[0]), float(target_forbidden[1])
        base_str = "none"
        if base_forbidden is not None and len(base_forbidden) >= 2:
            base_str = f"y = [{float(base_forbidden[0]):.1f}, {float(base_forbidden[1]):.1f}] m"
        pattern_initial = r"(- \*\*Forbidden Anchor Zones\*\*: )None in the source environment\."
        pattern_updated = r"(- \*\*Forbidden Anchor Zones\*\*: )y = \[[^\]]+\] m forbidden \(originally [^)]+\)\."
        replacement = f"\\g<1>y = [{y_min:.1f}, {y_max:.1f}] m forbidden (originally {base_str} in the source environment)."
        if re.search(pattern_initial, criteria):
            criteria = re.sub(pattern_initial, replacement, criteria)
        elif re.search(pattern_updated, criteria):
            criteria = re.sub(pattern_updated, replacement, criteria)
    target_strength_map = target_terrain_config.get("anchor_strength_map", None)
    base_strength_map = base_terrain_config.get("anchor_strength_map", None)
    if target_strength_map and len(target_strength_map) > 0:
        parts = []
        for entry in target_strength_map:
            if len(entry) >= 4:
                y_lo, y_hi, f_mult, t_mult = float(entry[0]), float(entry[1]), float(entry[2]), float(entry[3])
                parts.append(f"y = [{y_lo:.1f}, {y_hi:.1f}] m at {f_mult*100:.2f}%/{t_mult*100:.2f}%")
        if parts:
            strength_desc = "; ".join(parts)
            base_str = "none in the source environment"
            if base_strength_map and len(base_strength_map) > 0 and len(base_strength_map[0]) >= 4:
                be = base_strength_map[0]
                base_str = f"y = [{float(be[0]):.1f}, {float(be[1]):.1f}] m at {float(be[2])*100:.2f}%/{float(be[3])*100:.2f}% in the source environment"
            pattern_ra = r"(- \*\*Regional anchor strength\*\*: )None in the source environment; when present, the vertical segment and force/torque multipliers are stated\.?"
            pattern_ra_updated = r"(- \*\*Regional anchor strength\*\*: ).*? \(originally [^)]+\)\.?"
            repl_ra = f"\\g<1>{strength_desc} (originally {base_str})."
            if re.search(pattern_ra, criteria):
                criteria = re.sub(pattern_ra, repl_ra, criteria)
            elif re.search(pattern_ra_updated, criteria):
                criteria = re.sub(pattern_ra_updated, repl_ra, criteria)
    target_load_type = target_terrain_config.get("load_type", "static")
    base_load_type = base_terrain_config.get("load_type", "static")
    target_drop = float(target_terrain_config.get("drop_height", 10.0))
    if target_load_type == "dropped":
        pattern_payload = r"(- \*\*Payload application\*\*: )Static \(placed on structure at the given times\) in the source environment\.?"
        if re.search(pattern_payload, criteria):
            criteria = re.sub(
                pattern_payload,
                f"\\g<1>Dropped from {target_drop:.1f} m height (originally static in the source environment).",
                criteria,
            )
    elif base_load_type == "dropped":
        pattern_dropped = r"(- \*\*Payload application\*\*: )Dropped from [\d.]+ m height \(originally static in the source environment\)\.?"
        if re.search(pattern_dropped, criteria):
            criteria = re.sub(pattern_dropped, "\\g<1>Static (placed on structure at the given times) in the source environment.", criteria)
    return criteria

def get_s03_curriculum_stages() -> List[Dict[str, Any]]:
    stages_data = [
        {
            "stage_id": "Stage-1",
            "title": "The Torque Threshold",
            "mutation_description": "Single Variable: Wall anchor torque capacity reduced to a near-breaking extreme (1,500 N·m, barely above the torque generated by even a single lightweight beam under offset loading). Any conventional cantilever anchored at mid-height will fail instantly. The agent must discover this through trial failure and adapt by maximising anchor vertical separation to divide the overturning moment into tiny per-anchor torques.",
            "terrain_config": {
                "max_anchor_torque": 1000.0,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-2",
            "title": "The Monolithic Mandate",
            "mutation_description": "Coupled joint-and-anchor threshold: beam-to-beam joints can sustain only 70 N, so segmented trusses fail before both payloads settle. Wall-anchor force is capped at 15,500 N, requiring a jointless monolith with two widely separated wall connections to share payload reactions.",
            "terrain_config": {
                "max_anchor_force": 15500.0,
                "max_internal_force": 70.0,
            },
            "physics_config": {},
        },
        {
            "stage_id": "Stage-3",
            "title": "The Subterranean Gorge",
            "mutation_description": "Multi-variable: Overhead Obstacle + Severely Restricted Anchor Zone + Pervasive Attraction Field + Dropped Loads + Fragile Joints + Heavy Gravity. The entire usable anchor zone is critically weakened; a powerful spatial attraction field pulls every structural element toward its centre, creating enormous internal stresses; payloads crash down from height; internal joints snap under moderate stress; and gravity itself is amplified. Forces an extremely low-slung, ultralight truss that still withstands brutal multi-axis loading.",
            "terrain_config": {
                "target_reach": 35.0,
                "load_mass": 2500.0,
                "max_structure_mass": 7000.0,
                "forbidden_anchor_y": [-6.0, 30.0],
                "anchor_strength_map": [[-20.0, -6.0, 0.04, 0.04]],
                "obstacle_active": True,
                "obstacle_rects": [
                    [0.0, 3.0, 30.0, 30.0],
                ],
                "load_type": "dropped",
                "drop_height": 5.0,
                "max_internal_force": 5000000.0,
                "max_internal_torque": 5000000.0,
            },
            "physics_config": {
                "spatial_force": {
                    "center": (18.0, -8.0),
                    "magnitude": 600000.0,
                    "radius": 32.0,
                    "type": "attraction"
                },
                "wind": {
                    "force": (2000.0, -1500.0),
                    "oscillatory": True,
                    "frequency": 0.35
                },
                "gravity": (0, -16),
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "The Event Horizon",
            "mutation_description": "Multi-variable: 2.5g gravity crushes the structure under its own weight + Ultra-weak internal joints (1.8M N·m, 55× weaker than standard) snap under moderate stress + Critically weakened anchors at 1.5% strength in a narrow subterranean strip (only y < -3, 17m span) + Overhead spatial repulsion field centered above the cantilever tip relentlessly driving it into the ground + Violent oscillatory wind creating cyclic fatigue + Massive payloads dropped from extreme height delivering catastrophic impact impulse + Ultra-tight mass budget. Every dimension pushed to near-breaking extremes simultaneously: joint failure, anchor failure, excessive sag, and budget violation all loom. Only an ultra-precise lightweight truss with expertly distributed internal forces and maximal anchor separation can survive.",
            "terrain_config": {
                "target_reach": 40.0,
                "load_mass": 3500.0,
                "max_structure_mass": 4000.0,
                "forbidden_anchor_y": [-3.0, 30.0],
                "anchor_strength_map": [[-20.0, -3.0, 0.015, 0.015]],
                "load_type": "dropped",
                "drop_height": 20.0,
                "max_internal_force": 1800000.0,
                "max_internal_torque": 1800000.0,
                "min_tip_height_limit": -20.0,
            },
            "physics_config": {
                "spatial_force": {
                    "center": (34.0, 18.0),
                    "magnitude": 250000.0,
                    "radius": 18.0,
                    "type": "repulsion"
                },
                "wind": {
                    "force": (5000.0, -3500.0),
                    "oscillatory": True,
                    "frequency": 0.25
                },
                "gravity": (0, -25),
            },
        },
    ]
    for stage in stages_data:
        stage["task_description_suffix"] = uniform_suffix_for_task("S_03")
    return stages_data

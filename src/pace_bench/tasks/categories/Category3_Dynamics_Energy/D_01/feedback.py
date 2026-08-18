from typing import Dict, Any, List, Optional
import math
from pace_bench.core.simulator import TIME_STEP

def _f(val, default=None):
    if val is None:
        return default
    try:
        fv = float(val)
        return fv if math.isfinite(fv) else default
    except (TypeError, ValueError):
        return default

def _finite(v):
    return _f(v) is not None

def _int(val, default=0):
    finite = _f(val)
    return default if finite is None else int(finite)

def _pct(part, whole, default=None):
    p = _f(part)
    w = _f(whole)
    if p is None or w is None or abs(w) < 1e-9:
        return default
    return 100.0 * p / w

def _kb(mass_kg, speed_ms):
    m = _f(mass_kg)
    s = _f(speed_ms)
    if m is None or s is None:
        return None
    return 0.5 * m * s * s

def _header(title, dim=None):
    h = f"\n### {dim}. {title}\n" if dim else f"\n### {title}\n"
    return h

def _fmt_step(step_val):
    s = _f(step_val)
    if s is None or s < 0:
        return "--"
    return str(int(s))

def _step_to_time(step_val, fps=None):
    s = _f(step_val)
    if s is None or s < 0:
        return None
    return s * TIME_STEP if fps is None else s / fps

def _fmt_time(step_val, fps=60):
    t = _step_to_time(step_val, fps)
    if t is None:
        return "t=?.??s"
    return "t={:.2f}s".format(t)

def _launch_angle(vx, vy):
    x = _f(vx)
    y = _f(vy)
    if x is None or y is None:
        return None
    if abs(x) < 1e-9:
        return 90.0 if y > 0 else -90.0
    return math.degrees(math.atan2(y, x))

def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    if not metrics:
        return ["**No metrics available**"]

    parts: List[str] = []

    success = bool(metrics.get("success", False))
    failed = bool(metrics.get("failed", False))
    fr = metrics.get("failure_reason")
    step_count = _int(metrics.get("step_count"), 0)
    progress = _f(metrics.get("progress"))

    parts.append("## D_01 Diagnostic Report — The Launcher\n")
    status = "SUCCESS" if success else ("FAILED" if failed else "RUNNING")
    parts.append("**Status**: {}".format(status))
    if fr:
        parts.append("  Failure: {}".format(fr))
    progress_text = f"{progress:.1f}%" if progress is not None else "n/a"
    parts.append(f"  Steps: {step_count}  |  Progress: {progress_text}")
    parts.append("")

    parts.append(_header("Temporal Event Chronology", "1"))
    _timeline_report(metrics, parts)

    parts.append(_header("Spatial Diagnostics", "2"))
    _spatial_report(metrics, parts)

    parts.append(_header("Load & Structure", "3"))
    _load_report(metrics, parts)

    parts.append(_header("Energy Flow", "4"))
    _energy_report(metrics, parts)

    parts.append(_header("Constraint Profile", "5"))
    _constraint_report(metrics, parts)

    parts.append(_header("Numerical Health", "6"))
    _numerical_health_report(metrics, parts)

    return parts

def _timeline_report(metrics: Dict[str, Any], parts: List[str]):
    step = _int(metrics.get("step_count"), 0)
    events = []

    spawn_x = _f(metrics.get("projectile_spawn_x"))
    spawn_y = _f(metrics.get("projectile_spawn_y"))
    if spawn_x is not None and spawn_y is not None:
        events.append((0,
            "Simulation start. Projectile at spawn ({:.2f}, {:.2f}) m.".format(spawn_x, spawn_y)
        ))

    springs = metrics.get("spring_states", [])
    for s in springs:
        idx = s.get("index", "?")
        cl = _f(s.get("current_length"))
        rl = _f(s.get("rest_length"))
        cr = _f(s.get("compression_ratio"))
        fe = _f(s.get("force_est"))
        pe = _f(s.get("elastic_pe"))
        is_c = s.get("is_compressed", False)
        if cl is not None and rl is not None:
            state = "COMPRESSED" if is_c else ("TENSIONED" if s.get("is_tensioned") else "RELAXED")
            events.append((0,
                "Spring #{}: {} — force={:.1f} N, PE={:.1f} J, ratio={:.3f}"
                .format(idx, state,
                        fe if fe is not None else 0.0,
                        pe if pe is not None else 0.0,
                        cr if cr is not None else 1.0)
            ))
    if not springs:
        events.append((0, "No springs — zero energy storage."))

    joint_topo = metrics.get("joint_topology", [])
    for j in joint_topo:
        if j.get("projectile_connected"):
            events.append((0,
                "Joint #{} connects projectile — MECHANICALLY TETHERED (cannot fly free)."
                .format(j.get("index", "?"))
            ))

    sc_step = _f(metrics.get("spring_compressed_first_step"))
    if sc_step is not None and sc_step > 0:
        t = _fmt_time(sc_step)
        events.append((int(sc_step),
            "{} (step {}): Spring compression detected — elastic energy driving arm."
            .format(t, int(sc_step))
        ))

    aw_step = _f(metrics.get("arm_awake_first_step"))
    if aw_step is not None and aw_step > 0:
        t = _fmt_time(aw_step)
        events.append((int(aw_step),
            "{} (step {}): Arm became active — began moving."
            .format(t, int(aw_step))
        ))

    peak_ke = _f(metrics.get("peak_arm_ke"))
    peak_ke_step = _f(metrics.get("peak_arm_ke_step"))
    if peak_ke is not None and peak_ke > 0.001 and peak_ke_step is not None and peak_ke_step > 0:
        t = _fmt_time(peak_ke_step)
        peak_av = _f(metrics.get("peak_arm_ang_vel"))
        av_str = ", ω={:.3f} rad/s".format(peak_av) if peak_av is not None else ""
        events.append((int(peak_ke_step),
            "{} (step {}): Peak arm KE = {:.3f} J{} — max energy in arm."
            .format(t, int(peak_ke_step), peak_ke, av_str)
        ))

    ac_step = _f(metrics.get("arm_proj_contact_step"))
    if ac_step is not None and ac_step > 0:
        t = _fmt_time(ac_step)
        events.append((int(ac_step),
            "{} (step {}): Arm contacted projectile — energy transfer began."
            .format(t, int(ac_step))
        ))
    elif ac_step is None and peak_ke is not None and peak_ke > 0.001:
        arm_to_proj = _f(metrics.get("arm_to_projectile_distance"))
        if arm_to_proj is not None:
            events.append((step,
                "Arm moved (peak KE={:.3f} J) but NEVER contacted projectile. "
                "Final arm-to-projectile distance: {:.2f} m."
                .format(peak_ke, arm_to_proj)
            ))

    pl_step = _f(metrics.get("proj_launched_step"))
    pl_vx = _f(metrics.get("proj_launch_vx"))
    pl_vy = _f(metrics.get("proj_launch_vy"))
    if pl_step is not None and pl_step > 0:
        t = _fmt_time(pl_step)
        spd = math.sqrt(pl_vx**2 + pl_vy**2) if pl_vx is not None and pl_vy is not None else None
        angle = _launch_angle(pl_vx, pl_vy)
        angle_str = ", {:.1f} deg".format(angle) if angle is not None else ""
        spd_str = ", {:.2f} m/s".format(spd) if spd is not None else ""
        events.append((int(pl_step),
            "{} (step {}): Projectile launched — v=({:.2f}, {:.2f}){}{}."
            .format(t, int(pl_step),
                    pl_vx if pl_vx is not None else float('nan'),
                    pl_vy if pl_vy is not None else float('nan'),
                    spd_str, angle_str)
        ))

    pps = _f(metrics.get("peak_proj_speed"))
    pps_step = _f(metrics.get("peak_proj_speed_step"))
    if pps is not None and pps > 0.001 and pps_step is not None and pps_step > 0:
        t = _fmt_time(pps_step)
        events.append((int(pps_step),
            "{} (step {}): Peak projectile speed = {:.2f} m/s."
            .format(t, int(pps_step), pps)
        ))

    etx_step = _f(metrics.get("entered_target_x_step"))
    tx_min = _f(metrics.get("target_x_min"))
    tx_max = _f(metrics.get("target_x_max"))
    if etx_step is not None and etx_step > 0 and tx_min is not None and tx_max is not None:
        t = _fmt_time(etx_step)
        events.append((int(etx_step),
            "{} (step {}): Entered target x-band [{:.1f}, {:.1f}] m."
            .format(t, int(etx_step), tx_min, tx_max)
        ))

    ppy = _f(metrics.get("peak_proj_y"))
    ppy_step = _f(metrics.get("peak_proj_y_step"))
    if ppy is not None and ppy_step is not None and ppy_step > 0:
        t = _fmt_time(ppy_step)
        events.append((int(ppy_step),
            "{} (step {}): Peak altitude = {:.2f} m.".format(t, int(ppy_step), ppy)
        ))

    ext_step = _f(metrics.get("exited_target_x_step"))
    if ext_step is not None and ext_step > 0 and tx_min is not None and tx_max is not None:
        t = _fmt_time(ext_step)
        events.append((int(ext_step),
            "{} (step {}): Exited target x-band [{:.1f}, {:.1f}] m — OVERSHOOT."
            .format(t, int(ext_step), tx_min, tx_max)
        ))

    fr = metrics.get("failure_reason", "")
    hit = metrics.get("hit_occurred", False)
    if hit:
        events.append((step,
            "{} (step {}): Projectile entered target zone — HIT."
            .format(_fmt_time(step), step)
        ))
    else:
        events.append((step,
            "{} (step {}): Simulation ended — {}."
            .format(_fmt_time(step), step, fr if fr else "step limit reached")
        ))

    events.sort(key=lambda e: e[0])

    parts.append("**Event reconstruction (chronological):**\n")
    for _, desc in events:
        parts.append("- {}".format(desc))

    if pl_step is None:
        parts.append(
            "\n**Summary**: Projectile NEVER launched. "
            "No velocity >0.1 m/s with displacement >0.1 m detected."
        )
    else:
        final_px = _f(metrics.get("projectile_x"))
        if final_px is not None:
            dx_total = final_px - spawn_x
            parts.append(
                "\n**Summary**: Launched at step {} at {:.2f} m/s. "
                "Total horizontal displacement: {:+.2f} m."
                .format(_fmt_step(pl_step),
                        pps if pps is not None else 0.0,
                        dx_total)
            )

def _spatial_report(metrics: Dict[str, Any], parts: List[str]):
    px = _f(metrics.get("projectile_x"))
    py = _f(metrics.get("projectile_y"))
    tx_min = _f(metrics.get("target_x_min"))
    tx_max = _f(metrics.get("target_x_max"))
    ty_min = _f(metrics.get("target_y_min"))
    ty_max = _f(metrics.get("target_y_max"))
    spawn_x = _f(metrics.get("projectile_spawn_x"))
    spawn_y = _f(metrics.get("projectile_spawn_y"))

    if px is not None and py is not None:
        parts.append("  Position: ({:.2f}, {:.2f}) m".format(px, py))

    if px is not None and py is not None and tx_min is not None and tx_max is not None and ty_min is not None and ty_max is not None:
        x_status = ""
        y_status = ""
        if px < tx_min:
            x_status = "x short by {:.1f} m".format(tx_min - px)
        elif px > tx_max:
            x_status = "x past by {:.1f} m".format(px - tx_max)
        else:
            x_status = "x IN band"
        if py < ty_min:
            y_status = "y low by {:.1f} m".format(ty_min - py)
        elif py > ty_max:
            y_status = "y high by {:.1f} m".format(py - ty_max)
        else:
            y_status = "y IN band"
        parts.append(
            "  Target [{:.0f},{:.0f}]×[{:.0f},{:.0f}] m: {} | {}"
            .format(tx_min, tx_max, ty_min, ty_max, x_status, y_status)
        )

    max_y_tx = _f(metrics.get("max_y_in_target_x"))
    if max_y_tx is not None and tx_min is not None and tx_max is not None:
        parts.append("  Peak y in target x-band: {:.2f} m".format(max_y_tx))
    elif tx_min is not None and tx_max is not None:
        parts.append(
            "  Peak y in target x-band [{:.0f},{:.0f}]: N/A (never entered)"
            .format(tx_min, tx_max)
        )

    ppy = _f(metrics.get("peak_proj_y"))
    ppy_step = _f(metrics.get("peak_proj_y_step"))
    if ppy is not None:
        step_str = " (step {})".format(int(ppy_step)) if ppy_step is not None else ""
        parts.append("  Peak altitude: {:.2f} m{}".format(ppy, step_str))

    arm_to_proj = _f(metrics.get("arm_to_projectile_distance"))
    if arm_to_proj is not None:
        parts.append("  Arm-to-projectile: {:.2f} m".format(arm_to_proj))

    if px is not None and py is not None and spawn_x is not None and spawn_y is not None:
        total_dx = px - spawn_x
        total_dy = py - spawn_y
        total_dist = math.sqrt(total_dx**2 + total_dy**2)
        parts.append(
            "  Displacement from spawn: dx={:+.1f} m, dy={:+.1f} m, dist={:.1f} m"
            .format(total_dx, total_dy, total_dist)
        )

def _load_report(metrics: Dict[str, Any], parts: List[str]):
    springs = metrics.get("spring_states", [])
    structure_mass = _f(metrics.get("structure_mass"))
    max_mass = _f(metrics.get("max_structure_mass"))
    arm_mass = _f(metrics.get("arm_mass"))
    beam_count = metrics.get("beam_count")
    joint_count = metrics.get("joint_count")

    bc_str = str(beam_count) if beam_count is not None else "?"
    jc_str = str(joint_count) if joint_count is not None else "?"
    parts.append("  Components: {} beam(s), {} joint(s), {} spring(s)".format(
        bc_str, jc_str, len(springs)))

    if structure_mass is not None and max_mass is not None and max_mass > 0:
        mass_pct = _pct(structure_mass, max_mass, 0.0)
        parts.append(
            "  Mass: {:.2f} / {:.2f} kg ({:.1f}% used)"
            .format(structure_mass, max_mass, mass_pct)
        )
    if arm_mass is not None:
        parts.append("  Arm mass: {:.3f} kg".format(arm_mass))

    if springs:
        spring_items = []
        for s in springs:
            force = _f(s.get("force_est"))
            pe = _f(s.get("elastic_pe"))
            idx = s.get("index", "?")
            energized = bool(s.get("is_compressed") or s.get("is_tensioned"))
            if force is None:
                continue
            tier = "ENERGIZED" if (energized and force > 0.001) else "RELAXED"
            spring_items.append((force, pe, tier, idx))
        spring_items.sort(key=lambda x: -x[0])

        if len(spring_items) == 1:
            force, pe, tier, idx = spring_items[0]
            pe_str = ", PE={:.1f} J".format(pe) if pe is not None else ""
            parts.append(
                "  Spring #{}: force={:.1f} N [{}]{}"
                .format(idx, force, tier, pe_str)
            )
        else:
            parts.append("  Spring forces (ranked):")
            for force, pe, tier, idx in spring_items:
                pe_str = ", PE={:.1f} J".format(pe) if pe is not None else ""
                parts.append(
                    "    #{}: {:.1f} N [{}]{}".format(idx, force, tier, pe_str)
                )

    arm_ke = _f(metrics.get("arm_kinetic_energy"))
    arm_av = _f(metrics.get("arm_angular_velocity"))
    if arm_ke is not None and arm_av is not None:
        if abs(arm_av) < 0.001 and arm_ke < 0.001:
            parts.append("  Arm: STALLED (KE={:.4f} J, ω={:.4f} rad/s)".format(arm_ke, arm_av))
        else:
            parts.append("  Arm: ACTIVE (KE={:.4f} J, ω={:.4f} rad/s)".format(arm_ke, arm_av))

def _energy_report(metrics: Dict[str, Any], parts: List[str]):
    springs = metrics.get("spring_states", [])
    arm_ke = _f(metrics.get("arm_kinetic_energy"))
    pspeed = _f(metrics.get("projectile_speed"))

    total_spring_pe = 0.0
    any_energized = False
    for s in springs:
        pe = _f(s.get("elastic_pe"))
        if pe is not None:
            total_spring_pe += pe
        if s.get("is_compressed") or s.get("is_tensioned"):
            any_energized = True

    parts.append("**Energy chain**:\n")
    parts.append("  Spring PE (total): {:.1f} J ({} spring(s), {})".format(
        total_spring_pe, len(springs),
        "energized" if any_energized else "all relaxed" if springs else "none"
    ))

    if arm_ke is not None:
        parts.append("  Arm KE (terminal): {:.4f} J".format(arm_ke))
    peak_ke = _f(metrics.get("peak_arm_ke"))
    peak_ke_step = _f(metrics.get("peak_arm_ke_step"))
    if peak_ke is not None and peak_ke > 0.001:
        t_str = _fmt_time(peak_ke_step)
        parts.append("  Arm KE (peak): {:.3f} J at {}".format(peak_ke, t_str))

    if pspeed is not None:
        parts.append("  Projectile speed (terminal): {:.3f} m/s".format(pspeed))
    pps = _f(metrics.get("peak_proj_speed"))
    pps_step = _f(metrics.get("peak_proj_speed_step"))
    if pps is not None:
        t_str = _fmt_time(pps_step) if pps_step is not None else "?"
        parts.append("  Projectile speed (peak): {:.2f} m/s at {}".format(pps, t_str))

def _constraint_report(metrics: Dict[str, Any], parts: List[str]):
    structure_mass = _f(metrics.get("structure_mass"))
    max_mass = _f(metrics.get("max_structure_mass"))
    px = _f(metrics.get("projectile_x"))
    py = _f(metrics.get("projectile_y"))
    fr = metrics.get("failure_reason", "")
    hit = metrics.get("hit_occurred", False)

    constraint_info = metrics.get("constraint_info", {})

    if structure_mass is not None and max_mass is not None and max_mass > 0:
        mass_pct = _pct(structure_mass, max_mass, 0.0)
        passed = structure_mass <= max_mass
        margin = max_mass - structure_mass
        if not passed:
            parts.append(
                "  Mass: FAIL — {:.2f} / {:.2f} kg ({:.1f}%), over by {:.2f} kg"
                .format(structure_mass, max_mass, mass_pct, -margin)
            )
        elif mass_pct > 50:
            parts.append(
                "  Mass: PASS — {:.2f} / {:.2f} kg ({:.1f}%) [NEAR LIMIT]"
                .format(structure_mass, max_mass, mass_pct)
            )
        else:
            parts.append(
                "  Mass: PASS — {:.2f} / {:.2f} kg ({:.1f}%)"
                .format(structure_mass, max_mass, mass_pct)
            )

    bx_min = _f(constraint_info.get("build_zone_x_min"))
    bx_max = _f(constraint_info.get("build_zone_x_max"))
    by_min = _f(constraint_info.get("build_zone_y_min"))
    by_max = _f(constraint_info.get("build_zone_y_max"))
    arm_px = _f(metrics.get("arm_position_x"))
    arm_py = _f(metrics.get("arm_position_y"))
    if all(v is not None for v in (arm_px, arm_py, bx_min, bx_max, by_min, by_max)):
        in_x = bx_min <= arm_px <= bx_max
        in_y = by_min <= arm_py <= by_max
        if in_x and in_y:
            margin_x = min(arm_px - bx_min, bx_max - arm_px)
            margin_y = min(arm_py - by_min, by_max - arm_py)
            near = " [NEAR EDGE]" if (margin_x < 0.5 or margin_y < 0.5) else ""
            parts.append(
                "  Build zone: PASS — arm at ({:.2f}, {:.2f}) m, min margin x={:.2f} y={:.2f} m{}"
                .format(arm_px, arm_py, margin_x, margin_y, near)
            )
        else:
            reasons = []
            if not in_x:
                if arm_px < bx_min:
                    reasons.append("x={:.2f} < x_min={:.1f}".format(arm_px, bx_min))
                else:
                    reasons.append("x={:.2f} > x_max={:.1f}".format(arm_px, bx_max))
            if not in_y:
                if arm_py < by_min:
                    reasons.append("y={:.2f} < y_min={:.1f}".format(arm_py, by_min))
                else:
                    reasons.append("y={:.2f} > y_max={:.1f}".format(arm_py, by_max))
            parts.append(
                "  Build zone: FAIL — arm at ({:.2f}, {:.2f}) m; {}"
                .format(arm_px, arm_py, "; ".join(reasons))
            )
    else:
        parts.append("  Build zone: arm position unavailable")

    tx_min = _f(metrics.get("target_x_min"))
    tx_max = _f(metrics.get("target_x_max"))
    ty_min = _f(metrics.get("target_y_min"))
    ty_max = _f(metrics.get("target_y_max"))
    if tx_min is not None and tx_max is not None and ty_min is not None and ty_max is not None:
        tz_str = "[{:.0f},{:.0f}]×[{:.0f},{:.0f}] m".format(tx_min, tx_max, ty_min, ty_max)
        if hit:
            parts.append("  Target zone {}: HIT".format(tz_str))
        else:
            fail_dims = []
            if px is not None and tx_min is not None:
                if px < tx_min:
                    fail_dims.append("x={:.1f} < {:.0f} (gap={:.1f} m)".format(px, tx_min, tx_min - px))
                elif px > tx_max:
                    fail_dims.append("x={:.1f} > {:.0f} (past by {:.1f} m)".format(px, tx_max, px - tx_max))
                else:
                    fail_dims.append("x OK [{:.0f},{:.0f}]".format(tx_min, tx_max))
            if py is not None and ty_min is not None:
                if py < ty_min:
                    fail_dims.append("y={:.1f} < {:.0f} (gap={:.1f} m)".format(py, ty_min, ty_min - py))
                elif ty_max is not None and py > ty_max:
                    fail_dims.append("y={:.1f} > {:.0f} (past by {:.1f} m)".format(py, ty_max, py - ty_max))
                else:
                    fail_dims.append("y OK [{:.0f},{:.0f}]".format(ty_min, ty_max))
            parts.append(
                "  Target zone {}: MISS — {}"
                .format(tz_str, "; ".join(fail_dims) if fail_dims else "no entry")
            )

def _numerical_health_report(metrics: Dict[str, Any], parts: List[str]):
    issues: List[str] = []

    nan_keys = []
    for key in (
        "projectile_x", "projectile_y", "projectile_vx", "projectile_vy",
        "projectile_speed", "arm_angular_velocity", "arm_angle",
        "arm_kinetic_energy", "structure_mass",
        "proj_launch_vx", "proj_launch_vy",
        "peak_proj_speed", "peak_proj_y",
    ):
        val = metrics.get(key)
        if val is None:
            continue
        try:
            fv = float(val)
            if math.isnan(fv):
                nan_keys.append(key)
            elif math.isinf(fv):
                nan_keys.append("{} (inf)".format(key))
        except (TypeError, ValueError):
            nan_keys.append("{} (non-numeric)".format(key))
    if nan_keys:
        issues.append("NaN/Inf in: {}".format(", ".join(nan_keys)))
    observation_errors = metrics.get("observation_error_count", 0)
    if isinstance(observation_errors, (int, float)) and observation_errors > 0:
        issues.append(
            "Observation API errors: {}; last={}".format(
                int(observation_errors),
                metrics.get("last_observation_error") or "details unavailable",
            )
        )

    pspeed = _f(metrics.get("projectile_speed"))
    if pspeed is not None and pspeed > 100.0:
        issues.append("Extreme projectile speed: {:.1f} m/s".format(pspeed))
    pvx = _f(metrics.get("projectile_vx"))
    pvy = _f(metrics.get("projectile_vy"))
    if pvx is not None and abs(pvx) > 100.0:
        issues.append("Extreme projectile vx: {:.1f} m/s".format(pvx))
    if pvy is not None and abs(pvy) > 100.0:
        issues.append("Extreme projectile vy: {:.1f} m/s".format(pvy))

    arm_av = _f(metrics.get("arm_angular_velocity"))
    peak_av = _f(metrics.get("peak_arm_ang_vel"))
    if arm_av is not None and abs(arm_av) > 50.0:
        issues.append("Extreme arm ω: {:.1f} rad/s".format(arm_av))
    if peak_av is not None and abs(peak_av) > 50.0:
        issues.append("Extreme peak arm ω: {:.1f} rad/s".format(peak_av))

    springs = metrics.get("spring_states", [])
    extreme_springs = []
    for s in springs:
        stiff = _f(s.get("stiffness"))
        cr = _f(s.get("compression_ratio"))
        idx = s.get("index", "?")
        if stiff is not None and stiff > 50000.0:
            extreme_springs.append("#{}: stiffness={:.0f} N/m".format(idx, stiff))
        if cr is not None and 0.0 <= cr < 0.05:
            extreme_springs.append("#{}: compression_ratio={:.4f}".format(idx, cr))
    if extreme_springs:
        issues.append("Extreme springs: {}".format("; ".join(extreme_springs)))

    arm_awake = metrics.get("arm_awake")
    proj_awake = metrics.get("projectile_awake")
    if arm_awake is False and proj_awake is False:
        issues.append("Both arm and projectile sleeping — mechanism completely stalled.")
    elif arm_awake is False:
        issues.append("Arm is sleeping — stopped moving, damping may have absorbed all energy.")
    elif proj_awake is False and pspeed is not None and pspeed < 0.001:
        issues.append("Projectile sleeping with speed=0 — never received momentum.")

    if not metrics.get("spring_ever_compressed", True) and len(springs) > 0:
        issues.append(
            "{} spring(s) present but NONE ever stored measurable elastic energy."
            .format(len(springs))
        )

    arm_ke = _f(metrics.get("arm_kinetic_energy"))
    peak_ke = _f(metrics.get("peak_arm_ke"))
    if (arm_ke is not None and arm_ke < 0.001 and
            peak_ke is not None and peak_ke < 0.001 and
            metrics.get("arm_awake_first_step") is None):
        issues.append(
            "Arm never became active — no driving force or damping dissipated all energy."
        )

    if not issues:
        parts.append("OK — all values finite, within expected ranges.")
    else:
        parts.append("{} issue(s):".format(len(issues)))
        for issue in issues:
            parts.append("  - {}".format(issue))


def get_improvement_suggestions(
    metrics: Dict[str, Any],
    score: float,
    success: bool,
    failed: bool,
    failure_reason: Optional[str] = None,
    error: Optional[str] = None,
) -> List[str]:
    del metrics, score, success, failed, failure_reason
    if error:
        return [f"- Resolve the reported execution error: {error}"]
    return []

try:
    from pace_bench.evaluation.verification.diagnostics import (
        format_generic_execution_metrics,
    )
except ImportError:
    format_generic_execution_metrics = None
